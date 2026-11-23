"""Connect compiled scene checks, adapter path samples and physics-step evidence."""
from collections.abc import Mapping
import copy
import math

from runtime import mj, np
from collision_policy import CollisionPolicy


def check_inherited_parts(model, parts):
    """Verify that composing the task scene preserved upstream collision proxies."""
    for part in parts:
        geom = model.geom(part['mjcf_geom'])
        shape = part['shape']
        size = np.asarray(part['size_m'])
        expected = size / 2 if shape == 'box' else np.array([size[0], size[1] / 2])
        actual = model.geom_size[geom.id, :len(expected)]
        if (model.geom_type[geom.id] != getattr(mj.mjtGeom, 'mjGEOM_' + shape.upper()) or
                not np.allclose(actual, expected, rtol=1e-7, atol=1e-9)):
            raise ValueError(f'{geom.name}: inherited {shape} expected {expected.tolist()} m, '
                             f'compiled dimensions {actual.tolist()} m. Check the copied MJCF and units.')
        if not (model.geom_contype[geom.id] or model.geom_conaffinity[geom.id]):
            explicit = geom.id in model.pair_geom1 or geom.id in model.pair_geom2
            if not explicit:
                raise ValueError(f'{geom.name}: inherited collision geom disabled in the task scene')
        # A fused fixed body can change the local frame while preserving world geometry.
        body = model.body(int(model.geom_bodyid[geom.id]))
        if body.name == part['link']:
            transform = np.asarray(part['local_transform'])
            orientation = np.empty(9)
            mj.mju_quat2Mat(orientation, model.geom_quat[geom.id])
            if (not np.allclose(model.geom_pos[geom.id], transform[:3, 3], atol=1e-8, rtol=1e-7) or
                    not np.allclose(orientation.reshape(3, 3), transform[:3, :3], atol=1e-8, rtol=1e-7)):
                raise ValueError(f'{geom.name}: local collision placement differs from the generator; '
                                 'check link frames and geometry transforms')


def check_geometry(model, data, specifications):
    """Compare independently specified SI dimensions with the compiled scene."""
    dimensions = {'box': 3, 'sphere': 1, 'capsule': 2, 'cylinder': 2, 'ellipsoid': 3}
    if not isinstance(specifications, dict):
        raise ValueError('collision_geometry must map geom names to measurements')
    for name, spec in specifications.items():
        if not isinstance(spec, dict) or spec.get('type') not in dimensions or not spec.get('source'):
            raise ValueError(f'collision_geometry {name}: specify type, size_m, tolerance_m and source')
        geom = model.geom(name)
        kind = spec['type']
        tolerance = spec.get('tolerance_m')
        if isinstance(tolerance, bool) or not isinstance(tolerance, (float, int)) or not math.isfinite(tolerance) or tolerance < 0:
            raise ValueError(f'{name}: tolerance_m must be finite and nonnegative')
        if model.geom_type[geom.id] != getattr(mj.mjtGeom, 'mjGEOM_' + kind.upper()):
            raise ValueError(f'{name}: expected {kind} from {spec["source"]}, check the collision geom type')
        measurements = {'size_m': model.geom_size[geom.id, :dimensions[kind]]}
        if 'initial_center_m' in spec:
            measurements['initial_center_m'] = data.geom_xpos[geom.id]
        for field, actual in measurements.items():
            expected = np.asarray(spec.get(field), dtype=float)
            if expected.shape != actual.shape or not np.isfinite(expected).all():
                raise ValueError(f'{name}: invalid {field}, expected {actual.size} finite SI values')
            if np.any(np.abs(actual - expected) > tolerance):
                raise ValueError(f'{name} {field}: measured {actual.tolist()}, expected {expected.tolist()} '
                                 f'within {tolerance} m from {spec["source"]}. Check units, proxy and placement.')


class TaskCollisionChecks:
    def __init__(self, model, contract):
        self.model = model
        self.contract = contract
        self.policy = None
        self.failure = None
        self.mode = contract.get('collision_path_mode')
        self.segments = 0
        self.generator_status = 'not_evaluated'

    def initialize(self, data, controller, inspection):
        try:
            upstream = inspection.get('collision_review', {})
            self.generator_status = upstream.get('status', 'not_evaluated')
            if inspection.get('ready_for_compile') is False or self.generator_status == 'failed':
                raise ValueError('Generator rejected this model. Inspect model_inspection.json and validation.json.')
            self.policy = CollisionPolicy(self.model, self.contract.get('collision_policy'))
            if not self.contract.get('collision_policy'):
                return
            check_inherited_parts(self.model, upstream.get('parts', []))
            check_geometry(self.model, data, self.contract.get('collision_geometry', {}))
            self.failure = self.policy.observe(data, 0, 'initial')
            if self.failure:
                return
            plan = getattr(controller, 'collision_plan', None)
            if self.mode == 'planned' and not callable(plan):
                raise ValueError('planned path requires controller.collision_plan(model, data)')
            if callable(plan) and self.mode in (None, 'planned'):
                planner_data = copy.copy(data)
                self.check_segments(data, plan(self.model, planner_data), None, None)
        except (KeyError, TypeError, ValueError) as exc:
            self.reject(data, 0, 'initial', exc)

    def reject(self, data, cycle, phase, error):
        self.failure = dict(code='invalid_collision_input', phase=phase, cycle=cycle,
                            time_s=float(data.time), qpos=data.qpos.tolist(), detail=str(error),
                            hint='Check the phase, geom names, SI dimensions and adapter path interface.')

    def check_segments(self, data, segments, cycle, phase):
        """The planner supplies actual interpolation, including moving task objects."""
        state_type = mj.mjtState.mjSTATE_INTEGRATION
        before = np.empty(mj.mj_stateSize(self.model, state_type))
        mj.mj_getState(self.model, data, before, state_type)
        try:
            self._sample_segments(data, segments, cycle, phase)
        finally:
            after = np.empty_like(before)
            mj.mj_getState(self.model, data, after, state_type)
            if not np.array_equal(before, after):
                mj.mj_setState(self.model, data, before, state_type)
                mj.mj_forward(self.model, data)
                raise ValueError('Path sampler modified the live physics state. Return separate qpos samples.')

    def _sample_segments(self, data, segments, cycle, phase):
        found = False
        expected_start = data.qpos.copy()
        delta = np.empty(self.model.nv)
        for segment in segments:
            if not isinstance(segment, Mapping) or not callable(segment.get('sample_qpos')):
                raise ValueError('Each path segment needs a callable sample_qpos(u)')
            segment_phase = segment.get('phase', phase)
            if not isinstance(segment_phase, str) or not segment_phase:
                raise ValueError('Each planned path segment needs its task phase')
            sample = segment['sample_qpos']

            def continuous_sample(u):
                nonlocal expected_start
                qpos = np.asarray(sample(u), dtype=float)
                if qpos.shape != (self.model.nq,):
                    raise ValueError(f'{segment_phase}: expected full nq={self.model.nq} position vector')
                if u == 0:
                    mj.mj_differentiatePos(self.model, delta, 1., expected_start, qpos)
                    if np.any(np.abs(delta) > 1e-8):
                        raise ValueError(f'{segment_phase}: path starts away from the measured state or preceding endpoint; supply the connecting motion')
                if u == 1:
                    expected_start = qpos.copy()
                return qpos

            self.failure = self.policy.check_path(
                data, segment.get('cycle', cycle or 1), segment_phase, continuous_sample,
                travel_bound_m=segment['travel_bound_m'], resolution_m=segment.get('resolution_m'))
            self.segments += 1
            found = True
            if self.failure:
                return
        if not found:
            raise ValueError('Supply at least one actual path segment to check')

    def observe(self, data, cycle, phase, meta):
        if not self.contract.get('collision_policy'):
            return
        try:
            self.failure = self.policy.observe(data, cycle, phase)
            if self.failure:
                return
            segments = meta.get('collision_segments')
            if segments is not None:
                self.check_segments(data, segments, cycle, phase)
            elif self.mode == 'online' and not meta.get('done'):
                raise ValueError('online mode requires the pending command in collision_segments')
            if meta.get('done') and not self.failure:
                self.failure = self.policy.finish_task(self.contract['cycles'])
        except (KeyError, TypeError, ValueError) as exc:
            self.reject(data, cycle, phase, exc)

    def report(self):
        summary = self.policy.summary() if self.policy is not None else {'status': 'not_evaluated'}
        if self.failure:
            summary = dict(summary, status='failed')
        path = dict(mode=self.mode, segments_checked=self.segments,
                    reason=self.contract.get('collision_path_reason'))
        return dict(collision=summary, collision_failure=self.failure, collision_path=path,
                    generator_collision_status=self.generator_status)
