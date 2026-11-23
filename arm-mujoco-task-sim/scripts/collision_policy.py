"""Phase-specific contact checks shared by scene, trajectory and runtime checks."""
from dataclasses import dataclass
from itertools import product
import math
from numbers import Real

from runtime import mj, np


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value) or value < 0:
        raise ValueError(f'{name} must be a finite, nonnegative number')
    return float(value)


def _pair(a, b):
    return (a, b) if a < b else (b, a)


@dataclass(frozen=True)
class Rule:
    endpoints: tuple
    mode: str
    source: str
    max_depth_m: float | None
    max_force_N: float | None
    min_force_N: float
    grace_s: float
    max_violation_s: float
    min_clearance_m: float


class CollisionPolicy:
    """Call observe after mj_forward; create a fresh instance for each reset."""

    def __init__(self, model, config):
        self.model = model
        self.enabled = config is not None
        self.failure = None
        self.runtime_samples = self.path_samples = self.path_segments = self.path_evaluations = 0
        self.visited = {}
        self.cycle_phases = {}
        self.required_phases = []
        self.key = self.last_state = None
        self.previous_positions = {}
        self.established = set()
        self.since = {}
        if not self.enabled:
            return
        if model.nflex:
            raise ValueError('Collision rules currently evaluate rigid geometry pairs; flex contacts require an adapter')
        if not isinstance(config, dict):
            raise ValueError('collision_policy must be an object')
        unknown = config.keys() - {'phases', 'roles', 'default', 'barriers', 'required_phases','max_path_samples'}
        if unknown:
            raise ValueError(f'Unknown collision_policy keys: {sorted(unknown)}')
        self.default = config.get('default', 'forbid')
        self.max_path_samples = config.get('max_path_samples',65536)
        if type(self.max_path_samples) is not int or self.max_path_samples < 3:
            raise ValueError('max_path_samples must allow at least 3 evaluations per segment')
        if self.default not in ('forbid', 'allow'):
            raise ValueError('collision_policy.default must be forbid or allow')
        self.explicit_pairs = {_pair(int(a), int(b)) for a, b in zip(model.pair_geom1, model.pair_geom2)}
        self.excluded = set(map(int, model.exclude_signature))
        self.names = [model.geom(i).name or f'geom#{i}' for i in range(model.ngeom)]
        self.selectors = {name: (i,) for i, name in enumerate(self.names)}
        roles = config.get('roles', {})
        if not isinstance(roles, dict):
            raise ValueError('collision_policy.roles must map role names to geometry names')
        for role, names in roles.items():
            if not isinstance(role, str) or not role or role in self.selectors:
                raise ValueError(f'Role must be nonempty and distinct from geometry names: {role}')
            if not isinstance(names, list) or not names or any(name not in self.names for name in names):
                raise ValueError(f'Role {role} requires existing geometry names: {names}')
            self.selectors[role] = tuple(self.names.index(name) for name in names)

        phases = config.get('phases')
        if not isinstance(phases, dict) or 'initial' not in phases:
            raise ValueError('collision_policy.phases must contain initial and all controller phases')
        mandatory = config.get('required_phases', [])
        if not isinstance(mandatory,list) or any(not isinstance(name,str) or name not in phases or name == 'initial' for name in mandatory):
            raise ValueError('required_phases must list named operating phases required in each cycle')
        self.required_phases = list(dict.fromkeys(mandatory))
        self.phases = {}
        for phase, entries in phases.items():
            if not isinstance(phase, str) or not phase or not isinstance(entries, list):
                raise ValueError('Each collision phase needs a name and a list of rules')
            rules, lookup = [], {}
            for entry in entries:
                rule = self._make_rule(entry, phase)
                a, b = rule.endpoints
                pairs = {_pair(i, j) for i, j in product(self.selectors[a], self.selectors[b]) if i != j}
                if not pairs:
                    raise ValueError(f'{phase}: empty pair selection {a}/{b}')
                for pair in sorted(pairs):
                    if pair in lookup:
                        raise ValueError(f'{phase}: duplicate collision rules for {self._names(pair)}')
                    if not self._detectable(pair):
                        raise ValueError(f'{phase}: collision pair filtered in compiled model: {self._names(pair)}; inspect masks, body exclusions, parent/weld grouping or explicit pairs')
                    lookup[pair] = len(rules)
                rules.append(rule)
            self.phases[phase] = rules, lookup

        self.barriers = []
        barriers = config.get('barriers', [])
        if not isinstance(barriers, list):
            raise ValueError('barriers must be a list')
        for barrier in barriers:
            if not isinstance(barrier, dict) or set(barrier) != {'wall','moving','axis'}:
                raise ValueError('Each barrier needs wall, moving and axis')
            wall = self.selectors.get(barrier['wall'], ())
            moving = self.selectors.get(barrier['moving'], ())
            if len(wall) != 1 or not moving or wall[0] in moving or barrier['axis'] not in ('x', 'y', 'z'):
                raise ValueError(f'Invalid barrier geometry or local normal axis: {barrier}')
            if model.geom_type[wall[0]] != mj.mjtGeom.mjGEOM_BOX:
                raise ValueError(f'Barrier {barrier["wall"]} requires a box collision geom')
            self.barriers.extend((wall[0], item, 'xyz'.index(barrier['axis'])) for item in moving)

        self.filter_flags = int(mj.mjtDisableBit.mjDSBL_CONTACT | mj.mjtDisableBit.mjDSBL_CONSTRAINT |
                                mj.mjtDisableBit.mjDSBL_FILTERPARENT)
        disabled = int(model.opt.disableflags)
        if disabled & int(mj.mjtDisableBit.mjDSBL_CONTACT | mj.mjtDisableBit.mjDSBL_CONSTRAINT):
            raise ValueError('The model globally disables contacts or constraints')
        self.filters = self._filter_signature()
        self.path_geoms = np.flatnonzero((model.geom_contype != 0) | (model.geom_conaffinity != 0))
        if self.explicit_pairs:
            explicit = np.array(sorted(self.explicit_pairs),dtype=int).ravel()
            self.path_geoms = np.union1d(self.path_geoms,explicit)
        self.path_step_m = self._path_step()

    def _make_rule(self, entry, phase):
        fields = {'a','b','mode','source','max_depth_m','max_force_N','min_force_N','grace_s','max_violation_s',
                  'min_clearance_m'}
        if not isinstance(entry, dict) or set(entry) - fields:
            raise ValueError(f'{phase}: unknown collision rule fields: {entry}')
        a, b, mode = (entry.get(k) for k in ('a','b','mode'))
        if not isinstance(a,str) or not isinstance(b,str) or a not in self.selectors or b not in self.selectors or mode not in ('allow','require','forbid'):
            raise ValueError(f'{phase}: invalid geometry, role or mode in {entry}')
        source = entry.get('source', '')
        if not isinstance(source, str) or ((mode != 'forbid' or 'min_clearance_m' in entry) and not source.strip()):
            raise ValueError(f'{phase}: cite the task or threshold source for {a}/{b}')
        values = {k: _number(entry[k], k) for k in fields - {'a','b','mode','source'} if k in entry}
        if mode == 'forbid' and values.keys() - {'min_clearance_m'}:
            raise ValueError(f'{phase}: forbidden contact has no force, depth or time allowance')
        if mode != 'forbid' and 'min_clearance_m' in values:
            raise ValueError(f'{phase}: min_clearance_m applies to forbidden contact')
        if mode != 'require' and ({'grace_s','min_force_N'} & values.keys()):
            raise ValueError(f'{phase}: grace_s and min_force_N apply to required contact')
        if 'max_violation_s' in values and not ({'max_depth_m','max_force_N'} & values.keys()):
            raise ValueError(f'{phase}: max_violation_s requires a depth or force limit')
        if values.get('min_force_N',0) > values.get('max_force_N',math.inf):
            raise ValueError(f'{phase}: min_force_N exceeds max_force_N')
        return Rule((a,b), mode, source, values.get('max_depth_m'), values.get('max_force_N'),
                    values.get('min_force_N',0), values.get('grace_s',0), values.get('max_violation_s',0),
                    values.get('min_clearance_m',0))

    def _detectable(self, pair):
        m = self.model
        i, j = pair
        surfaces = (mj.mjtGeom.mjGEOM_PLANE, mj.mjtGeom.mjGEOM_HFIELD)
        if m.geom_type[i] in surfaces and m.geom_type[j] in surfaces:
            return False
        if pair in self.explicit_pairs:
            return True
        a, b = sorted((int(m.geom_bodyid[i]), int(m.geom_bodyid[j])))
        if (a << 16 | b) in self.excluded:
            return False
        wa, wb = int(m.body_weldid[a]), int(m.body_weldid[b])
        if wa == wb:
            return False
        if wa and wb and not int(m.opt.disableflags) & int(mj.mjtDisableBit.mjDSBL_FILTERPARENT):
            if m.body_weldid[m.body_parentid[wa]] == wb or m.body_weldid[m.body_parentid[wb]] == wa:
                return False
        return bool((m.geom_contype[i] & m.geom_conaffinity[j]) or (m.geom_contype[j] & m.geom_conaffinity[i]))

    def _filter_signature(self):
        m = self.model
        return (int(m.opt.disableflags) & self.filter_flags, m.geom_contype.tobytes(), m.geom_conaffinity.tobytes(),
                m.body_contype.tobytes(), m.body_conaffinity.tobytes(), m.geom_margin.tobytes(), m.geom_gap.tobytes(),
                m.exclude_signature.tobytes(), m.pair_geom1.tobytes(), m.pair_geom2.tobytes(),
                m.pair_margin.tobytes(), m.pair_gap.tobytes())

    def _names(self, pair):
        return [self.names[i] for i in pair]

    def _violate(self, state, code, geoms, expected, measured, hint):
        if self.failure is None:
            self.failure = dict(state, code=code, geoms=list(geoms), expected=expected,
                                measured=measured, hint=hint)
        return self.failure

    def _contacts(self, data, path):
        """Aggregate deepest penetration and simultaneous compressive force per geom pair."""
        contacts = {}
        wrench = np.zeros(6)
        for index, contact in enumerate(data.contact):
            a, b = int(contact.geom1), int(contact.geom2)
            if min(a,b) < 0:   # Flex contacts lack two rigid geoms.
                continue
            force = 0.
            if not path and contact.efc_address >= 0:
                mj.mj_contactForce(self.model, data, index, wrench)
                force = max(0., float(wrench[0]))
            distance = float(contact.dist)
            if distance > 0 and force <= 0:
                continue
            pair = _pair(a,b)
            depth = max(0., -distance)
            record = contacts.setdefault(pair, dict(depth_m=depth, force_N=0., active=False, xyz_m=contact.pos.tolist()))
            if depth > record['depth_m']:
                record.update(depth_m=depth, xyz_m=contact.pos.tolist())
            record['force_N'] += force
            record['active'] |= contact.efc_address >= 0
        return contacts

    def _crossing(self, data, state, previous):
        for wall, moving, axis in self.barriers:
            matrix = data.geom_xmat[wall].reshape(3,3)
            local = matrix.T @ (data.geom_xpos[moving] - data.geom_xpos[wall])
            key = wall, moving, axis
            old = previous.get(key)
            # Retain the last nonzero side when a sample lies exactly on the midplane.
            if local[axis] == 0:
                continue
            previous[key] = local
            if old is None or old[axis] * local[axis] >= 0:
                continue
            intersection = old + (local-old) * (old[axis] / (old[axis]-local[axis]))
            tangent = [i for i in range(3) if i != axis]
            if np.all(np.abs(intersection[tangent]) <= self.model.geom_size[wall,tangent]):
                return self._violate(state,'barrier_crossing',self._names((wall,moving)),
                                     'center remains on the same side within the finite wall footprint',
                                     dict(from_local_m=old.tolist(),to_local_m=local.tolist(),axis='xyz'[axis]),
                                     'Inspect the swept path, wall thickness and physics timestep.')

    def _check(self, data, state, previous):
        phase = state['phase']
        if phase not in self.phases:
            return self._violate(state,'unknown_phase',[],list(self.phases),phase,
                                 'Match the controller phase to the contract.')
        if self.filters != self._filter_signature():
            return self._violate(state,'collision_filters_changed',[],'validated collision filters','changed',
                                 'Inspect controller changes to masks, exclusions or contact disable flags.')
        crossing = self._crossing(data, state, previous)
        if crossing:
            return crossing
        path = state['origin'] == 'path'
        rules, lookup = self.phases[phase]
        contacts = self._contacts(data, path)
        for pair, idx in lookup.items():
            limit = rules[idx].min_clearance_m
            if limit <= 0 or pair in contacts:
                continue
            nearest = np.zeros(6)
            distance = mj.mj_geomDistance(self.model,data,*pair,limit,nearest)
            if distance < limit:
                return self._violate(state,'clearance_limit',self._names(pair),
                                     dict(min_clearance_m=limit,source=rules[idx].source),
                                     dict(distance_m=distance,nearest_xyz_m=nearest.reshape(2,3).tolist()),
                                     'Increase the planned clearance or correct proxy placement.')
        active = set()
        exceeded = set()
        for pair, values in contacts.items():
            idx = lookup.get(pair)
            rule = rules[idx] if idx is not None else None
            mode = rule.mode if rule else self.default
            if mode == 'forbid':
                return self._violate(state,'forbidden_contact',self._names(pair),'separation',values,
                                     'Inspect scene placement, collision proxies and the approach path.')
            if rule is None:
                continue
            if values['active'] and values['force_N'] >= rule.min_force_N:
                active.add(idx)
            for key, limit, code in [('depth_m',rule.max_depth_m,'penetration_limit'),
                                     ('force_N',rule.max_force_N,'force_limit')]:
                if limit is None or values[key] <= limit or (path and (key == 'force_N' or rule.max_violation_s)):
                    continue
                timer = idx, pair, key
                exceeded.add(timer)
                duration = 0. if path else state['time_s']-self.since.setdefault(timer,state['time_s'])
                initial_overlap = phase == 'initial' and key == 'depth_m'
                if initial_overlap or duration + 1e-12 >= rule.max_violation_s:
                    return self._violate(state,code,self._names(pair),{key:limit,'source':rule.source},
                                         dict(values,duration_s=duration),'Check contact surfaces, timing and controller limits.')
        if path:
            return
        for idx, rule in enumerate(rules):
            if rule.mode != 'require':
                continue
            if idx in active:
                self.established.add(idx)
                continue
            timer = idx, 'missing'
            exceeded.add(timer)
            duration = state['time_s'] - self.since.setdefault(timer,state['time_s'])
            if duration + 1e-12 >= rule.grace_s:
                return self._missing(state,rule,duration)
        for timer in self.since.keys() - exceeded:
            del self.since[timer]

    def _missing(self, state, rule, duration):
        return self._violate(state,'required_contact_missing',rule.endpoints,
                             dict(active_contact=True,min_force_N=rule.min_force_N,source=rule.source),
                             dict(missing_s=duration,grace_s=rule.grace_s),
                             'Check contact surfaces, grip or support, phase transition and contact masks.')

    def finish_phase(self):
        if self.failure or self.key is None:
            return self.failure
        rules, _ = self.phases[self.key[1]]
        for idx, rule in enumerate(rules):
            if rule.mode == 'require' and idx not in self.established:
                return self._missing(self.last_state,rule,self.last_state['time_s']-self.since[(idx,'missing')])

    def observe(self, data, cycle, phase):
        if not self.enabled or self.failure:
            return self.failure
        key = cycle, phase
        if key != self.key:
            if self.finish_phase():
                return self.failure
            self.established.clear()
            self.since.clear()
            self.key = key
        self.runtime_samples += 1
        self.visited[phase] = self.visited.get(phase,0) + 1
        self.cycle_phases.setdefault(cycle,set()).add(phase)
        self.last_state = dict(cycle=cycle,phase=phase,time_s=float(data.time),qpos=data.qpos.tolist(),origin='runtime')
        return self._check(data,self.last_state,self.previous_positions)

    def finish_task(self, cycles):
        if not self.enabled or self.finish_phase():
            return self.failure
        for cycle in range(1,cycles+1):
            observed = self.cycle_phases.get(cycle,set())
            for phase in self.required_phases:
                if phase not in observed:
                    return self._violate(self.last_state,'required_phase_missing',[],
                                         dict(cycle=cycle,phase=phase),dict(observed_phases=sorted(observed)),
                                         'Inspect the controller transitions and per-cycle task evidence.')

    def _path_step(self):
        m = self.model
        half_extents = []
        for i in self.path_geoms:
            if m.geom_type[i] == mj.mjtGeom.mjGEOM_PLANE:
                continue
            half_extents.extend(float(v) for v in m.geom_aabb[i,3:] if v > 0)
        return min(half_extents,default=math.inf)

    def _surface_motion(self, start, end):
        """Bound endpoint displacement of a pair of surfaces, including rigid rotation."""
        old_pos, old_rot = start
        pos, rot = end
        frobenius = np.linalg.norm(rot-old_rot, axis=1)
        angle = 2*np.arcsin(np.clip(frobenius/(2*math.sqrt(2)),0,1))
        distance = np.linalg.norm(pos-old_pos, axis=1) + self.model.geom_rbound[self.path_geoms]*angle
        return float(np.sort(distance)[-2:].sum())

    def check_path(self, data, cycle, phase, sample_qpos, *, travel_bound_m, resolution_m=None):
        """sample_qpos(u) evaluates the actual interpolation, with objects, for 0 <= u <= 1.

        travel_bound_m bounds relative surface speed per unit u (m per fraction),
        including rotations and carried objects. It must cover intra-segment peaks.
        """
        if not self.enabled or self.failure:
            return self.failure
        bound = _number(travel_bound_m,'travel_bound_m')
        step = self.path_step_m
        if resolution_m is not None:
            step = min(step,_number(resolution_m,'resolution_m'))
        if step <= 0 or not math.isfinite(step):
            raise ValueError('A positive resolution_m or finite collision geom thickness is required')
        self.path_segments += 1
        evaluations = 0
        last_evaluated = 0.

        def budget_failure(u, qpos):
            state = dict(cycle=cycle,phase=phase,time_s=float(data.time),qpos=qpos.tolist(),origin='path',fraction=u)
            return self._violate(state,'path_budget_exhausted',[],'complete segment at the requested resolution',
                                 dict(max_path_samples=self.max_path_samples, evaluations=evaluations,
                                      travel_bound_m=bound, resolution_m=step),
                                 'Review the interpolation, geometry scales and bound. Split the segment or raise max_path_samples.')

        if bound > (self.max_path_samples-1)*step:
            return budget_failure(0.,data.qpos)
        count = max(2,math.ceil(bound/step))
        probe = mj.MjData(self.model)
        # Current equality states and mocap scene placement must match the actual task.
        probe.eq_active[:] = data.eq_active
        probe.mocap_pos[:] = data.mocap_pos
        probe.mocap_quat[:] = data.mocap_quat
        probe.time = data.time
        previous = {}

        def sample(u):
            nonlocal evaluations, last_evaluated
            evaluations += 1
            self.path_evaluations += 1
            last_evaluated = u
            q = np.asarray(sample_qpos(u),dtype=float)
            if q.shape != data.qpos.shape or not np.isfinite(q).all():
                raise ValueError(f'{phase}: trajectory sample at {u} requires {self.model.nq} finite positions')
            probe.qpos[:] = q
            mj.mj_forward(self.model,probe)
            return probe.geom_xpos[self.path_geoms], probe.geom_xmat[self.path_geoms]

        def check(u):
            self.path_samples += 1
            state = dict(cycle=cycle,phase=phase,time_s=float(data.time),qpos=probe.qpos.tolist(),origin='path',
                         fraction=u,subdivisions=count,resolution_m=step)
            return self._check(probe,state,previous)

        start = 0.
        geometry = sample(start)
        if check(start):
            return self.failure
        for index in range(1,count+1):
            endpoints = [index/count]
            while endpoints:
                end = endpoints.pop()
                if evaluations >= self.max_path_samples:
                    return budget_failure(last_evaluated,probe.qpos)
                next_geometry = sample(end)
                if self._surface_motion(geometry,next_geometry) > step*(1+1e-8):
                    midpoint = (start+end)/2
                    if midpoint <= start or midpoint >= end:
                        raise ValueError(f'{phase}: discontinuous path near fraction {end}; split resets from continuous motion')
                    endpoints.extend([end,midpoint])
                    continue
                if check(end):
                    return self.failure
                start, geometry = end, next_geometry

    def summary(self):
        status = 'failed' if self.failure else 'passed' if self.runtime_samples else 'not_evaluated'
        return dict(status=status,first_violation=self.failure,runtime_samples=self.runtime_samples,
                    path_samples=self.path_samples,path_evaluations=self.path_evaluations,
                    path_segments=self.path_segments,coverage=dict(self.visited),
                    required_phases=self.required_phases,
                    unvisited_phases=sorted(self.phases.keys()-self.visited.keys()) if self.enabled else [],
                    cycle_phases={str(c):sorted(names) for c,names in self.cycle_phases.items()})
