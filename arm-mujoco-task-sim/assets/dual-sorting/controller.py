"""Contact-based sorting controller for the skill's MuJoCo runner."""
from collections import deque
import math

from runtime import mj, np, dump
from scene import OBJECTS, OPEN, SLOTS, TRAY_Y

GRIP_Y = {"twin_posts": -.013}


def yaw_matrix(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, s, 0], [s, -c, 0], [0, 0, -1]])


def rotation_error(actual, target):
    a, b = np.empty(4), np.empty(4)
    mj.mju_mat2Quat(a, actual.ravel())
    mj.mju_mat2Quat(b, target.ravel())
    error = np.empty(3)
    mj.mju_subQuat(error, b, a)
    return actual @ error


class Controller:
    def __init__(self, m, d, contract, run):
        self.cx, self.cy = contract["center_xy_m"]
        self.table = contract["tabletop_z_m"]
        self.carry = self.table+.145
        self.m = m
        self.run = run
        self.rng = np.random.default_rng(contract["seed"])
        self.rounds = contract["cycles"]
        from collision_policy import CollisionPolicy
        self.policy = CollisionPolicy(m,contract["collision_policy"])
        # Absolute link offset sum plus tool, pad and carried part extent.
        self.reach = sum(np.linalg.norm(m.body_pos[m.jnt_bodyid[i]]) for i in range(7)) + .34
        self.site = m.site("grip_tcp").id
        self.kin = mj.MjData(m)
        self.bounds = m.jnt_range[:7] + np.array([.015, -.015])
        self.kp = m.actuator_gainprm[:7, 0].copy()
        self.kv = -m.actuator_biasprm[:7, 2].copy()
        self.filter = m.actuator_dynprm[:7, 0].copy()
        self.jac_pos, self.jac_rot = np.zeros((2, 3, m.nv))
        self.object_bodies = {m.body(o[0]).id for o in OBJECTS}
        self.pad_ids = {m.geom("pad_a").id, m.geom("pad_b").id}
        self.geom_names = [m.geom(i).name for i in range(m.ngeom)]
        self.tray_floors = {m.geom(name).id: name for name in ("tray_left_base", "tray_right_base")}
        self.first_key = m.key("initial").id
        self.home = m.key_qpos[self.first_key, :7].copy()
        self.cycle = 0
        self.results = []
        self.layouts = []
        self.fault = None
        self.reset_pending = False
        self.start_episode(d)

    def start_episode(self, d):
        # Executed only by the runner outside the per-step control callback.
        self.cycle += 1
        t = float(d.time)
        mj.mj_resetDataKeyframe(self.m, d, self.first_key)
        d.time = t
        self.objects = OBJECTS
        self.queue = deque(self.rng.permutation(4).tolist())
        self.initial = {}
        self.per_object = []
        self.slots = {"red": 0, "blue": 0}
        self.target = None
        self.grasp_reference = None
        self.jaws = OPEN
        self.path = None
        self.phase = "settle"
        self.began = t
        self.unexpected = {}
        # Random permutation of four separated cells, with independent jitter and yaw.
        cells = [self.cx,self.cy] + np.array([[-.049,-.035],[-.049,.035],[.049,-.035],[.049,.035]])
        for obj, center in zip(self.objects, cells[self.rng.permutation(4)]):
            name, color = obj[:2]
            x, y = center + self.rng.uniform(-.004, .004, 2)
            yaw = float(self.rng.uniform(-.16, .16))
            j = self.m.joint(f"free_{name}").qposadr[0]
            base_z = self.table
            pose = [x, y, base_z+.0015, math.cos(yaw/2), 0, 0, math.sin(yaw/2)]
            d.qpos[j:j+7] = pose
            self.initial[name] = dict(color=color, qpos=pose, yaw_rad=yaw)
        self.layouts.append(self.initial)
        dump(self.run/"data/layouts.json", self.layouts)
        self.reset_pending = False
        mj.mj_forward(self.m, d)
        return dict(kind="episode_start", cycle=self.cycle, layout=self.initial)

    def ik(self, point, rotation, seed):
        q = seed.copy()
        for _ in range(90):
            self.kin.qpos[:7] = q
            mj.mj_kinematics(self.m, self.kin)
            mj.mj_comPos(self.m, self.kin)
            dp = point - self.kin.site_xpos[self.site]
            dr = rotation_error(self.kin.site_xmat[self.site].reshape(3, 3), rotation)
            if np.linalg.norm(dp) < .00010 and np.linalg.norm(dr) < .0008:
                return q
            mj.mj_jacSite(self.m, self.kin, self.jac_pos, self.jac_rot, self.site)
            jac = np.vstack((self.jac_pos[:, :7], .22*self.jac_rot[:, :7]))
            delta = jac.T @ np.linalg.solve(jac @ jac.T + .00002*np.eye(6), np.r_[dp, .22*dr])
            q = np.clip(q + np.clip(delta, -.13, .13), *self.bounds.T)
        raise RuntimeError(f"IK did not converge for {point.round(4)}")

    def predict(self, q):
        # Keep passive objects at their measured poses. Move a grasped object
        # using its measured TCP transform in this scratch state only.
        self.kin.qpos[:7] = q
        if self.carried_pose is not None:
            mj.mj_kinematics(self.m, self.kin)
            j, p_local, q_local = self.carried_pose
            rotation = self.kin.site_xmat[self.site].reshape(3,3)
            q_world = np.empty(4)
            mj.mju_mat2Quat(q_world, rotation.ravel())
            self.kin.qpos[j:j+3] = self.kin.site_xpos[self.site] + rotation @ p_local
            mj.mju_mulQuat(self.kin.qpos[j+3:j+7], q_world, q_local)
        return self.kin.qpos.copy()

    def is_expected(self, g1, g2, b1, b2, phase):
        if self.geom_names[g1] in ("table_top","tray_left_base","tray_right_base") and b2 in self.object_bodies:
            return True
        if self.geom_names[g2] in ("table_top","tray_left_base","tray_right_base") and b1 in self.object_bodies:
            return True
        if phase in ("descend", "grasp", "lift", "transfer", "lower", "release", "retreat"):
            if self.target_body in (b1, b2) and (g1 in self.pad_ids or g2 in self.pad_ids):
                return True
        return False

    def move(self, d, phase, point=None, duration=.8, home=False):
        self.phase, self.began = phase, float(d.time)
        self.kin.qpos[:] = d.qpos
        self.carried_pose = None
        if phase in ("lift", "transfer", "lower"):
            j = int(self.m.joint(f"free_{self.target[0]}").qposadr[0])
            rotation = d.site_xmat[self.site].reshape(3, 3)
            local = rotation.T @ (d.qpos[j:j+3]-d.site_xpos[self.site])
            q_inv, q_local = np.empty(4), np.empty(4)
            mj.mju_mat2Quat(q_inv, rotation.T.ravel())
            mj.mju_mulQuat(q_local, q_inv, d.qpos[j+3:j+7])
            self.carried_pose = (j, local, q_local)
        if home:
            qs = np.linspace(d.qpos[:7], self.home, 31)
        else:
            p0 = d.site_xpos[self.site].copy()
            r0 = d.site_xmat[self.site].reshape(3, 3).copy()
            a = np.empty(4)
            mj.mju_mat2Quat(a, r0.ravel())
            spin = r0.T @ rotation_error(r0, self.rotation)
            angle = np.linalg.norm(spin)
            n = max(12, int(np.linalg.norm(point - p0)/.009))
            qs = [d.qpos[:7].copy()]
            for i in range(1, n+1):
                f = i/n
                target_r = self.rotation
                if angle > 1e-10:
                    offset, quat, matrix = np.empty(4), np.empty(4), np.empty(9)
                    mj.mju_axisAngle2Quat(offset, spin/angle, angle*f)
                    mj.mju_mulQuat(quat, a, offset)
                    mj.mju_quat2Mat(matrix, quat)
                    target_r = matrix.reshape(3, 3)
                q = self.ik(p0+f*(point-p0), target_r, qs[-1])
                qs.append(q)
            qs = np.array(qs)
        # The geometric path of the minimum-jerk time law is the same
        # piecewise linear joint interpolant. Include the carried object's SE(3).
        def interpolate(u):
            edge = min(int(u*(len(qs)-1)),len(qs)-2)
            fraction = u*(len(qs)-1)-edge
            return self.predict(qs[edge]+fraction*(qs[edge+1]-qs[edge]))
        differences = np.abs(np.diff(qs,axis=0)).sum(axis=1)
        # Two moving surfaces, each no farther than reach from any hinge.
        bound = 2*self.reach*(len(qs)-1)*float(differences.max())
        key = phase if phase=='return_home' else phase+'_'+self.target[0]
        finding = self.policy.check_path(d,self.cycle,key,interpolate,travel_bound_m=bound)
        if finding:
            raise RuntimeError(f"Predicted contact at {key}: {finding['code']} {finding['geoms']}")
        self.path = qs
        self.duration = duration

    def hold(self, d, phase, seconds):
        self.phase, self.began, self.duration = phase, float(d.time), seconds
        self.path = np.array([self.ref.copy()])

    def follow(self, d):
        n = len(self.path)-1
        t = np.clip((d.time-self.began)/self.duration, 0, 1)
        u = n*(10*t**3 - 15*t**4 + 6*t**5)
        i = min(int(u), max(0, n-1))
        q0, q1 = self.path[i], self.path[min(i+1, n)]
        self.ref = q0+(u-i)*(q1-q0)
        qd = (q1-q0)*(n*30*t*t*(1-t)**2/self.duration)
        # Retain upstream filtered servos. Feed forward gravity, motion damping and
        # the position-target filter delay through the same bounded position input.
        ctrl = self.ref + (self.kv*qd+d.qfrc_bias[:7])/self.kp + self.filter*qd
        d.ctrl[:7] = np.clip(ctrl, *self.m.actuator_ctrlrange[:7].T)
        # Ramp each jaw at 0.10 m/s to avoid an impact when the servo closes.
        d.ctrl[7:9] += np.clip(self.jaws-d.ctrl[7:9], -.10*self.m.opt.timestep,.10*self.m.opt.timestep)

    def next_object(self, d):
        self.target = self.objects[self.queue.popleft()]
        name, color, width, h, _ = self.target
        body = d.body(name)
        self.target_body = body.id
        r = body.xmat.reshape(3, 3)
        self.rotation = yaw_matrix(math.atan2(r[1, 0], r[0, 0]))
        self.pick = body.xpos + r @ [0,GRIP_Y.get(name,0),h]
        self.start_z = float(body.xpos[2])
        self.drop = np.array([self.cx+SLOTS[self.slots[color]], self.cy+(-TRAY_Y if color=="red" else TRAY_Y),
                              self.table+.009+h+.024])
        self.slots[color] += 1
        self.pinch = (width-.020)/2-.003
        self.had_grip = False
        self.slip_max = 0.
        self.grasp_reference = None
        self.move(d, "approach", np.r_[self.pick[:2], self.carry], 1.55)

    def observe_contacts(self, d):
        pads = set()
        tray_bodies = {}
        for c in d.contact:
            if c.dist > 0:
                continue
            g1, g2 = int(c.geom1), int(c.geom2)
            b1, b2 = int(self.m.geom_bodyid[g1]), int(self.m.geom_bodyid[g2])
            if self.target and self.target_body in (b1, b2):
                pads.update({g1, g2} & self.pad_ids)
            if g1 in self.tray_floors:
                tray_bodies[b2] = self.tray_floors[g1]
            if g2 in self.tray_floors:
                tray_bodies[b1] = self.tray_floors[g2]
            if c.dist < -.001 and not self.is_expected(g1, g2, b1, b2, self.phase):
                key = f"{self.geom_names[g1]}/{self.geom_names[g2]}"
                self.unexpected[key] = max(self.unexpected.get(key, 0), -float(c.dist))
        return len(pads)==2, tray_bodies

    def placement(self, d, obj, tray_bodies):
        name, color = obj[:2]
        b = d.body(name)
        qd = d.joint(f"free_{name}").qvel
        side = "left" if color=="red" else "right"
        expected_y = self.cy+(-TRAY_Y if color=="red" else TRAY_Y)
        stable = bool(np.linalg.norm(qd[:3]) < .025 and np.linalg.norm(qd[3:]) < .35)
        inside = bool(abs(b.xpos[0]-self.cx)<.090 and abs(b.xpos[1]-expected_y)<.032)
        supported = tray_bodies.get(b.id) == f"tray_{side}_base"
        return dict(object=name, color=color, tray=side, xyz_m=b.xpos.tolist(),
                    stable=stable, inside=inside, floor_contact=supported,
                    placed=stable and inside and supported)

    def complete_round(self, d, tray_bodies):
        end = [self.placement(d, obj, tray_bodies) for obj in self.objects]
        success = all(p["placed"] for p in end) and all(p["success"] for p in self.per_object) and not self.unexpected
        result = dict(cycle=self.cycle, success=success, final_positions=end,
                      grasps=self.per_object, unexpected_penetrations_m=self.unexpected,
                      finish_s=float(d.time))
        self.results.append(result)
        return result

    def control(self, m, d):
        events = []
        if self.phase == "settle" and self.path is None:
            self.ref = self.home.copy()
            self.hold(d, "settle", .65)
        bilateral, trays = self.observe_contacts(d)
        if self.target and self.phase in ("grasp", "lift", "transfer", "lower"):
            pos = d.body(self.target[0]).xpos
            local = d.site_xmat[self.site].reshape(3, 3).T @ (pos - d.site_xpos[self.site])
            if bilateral and self.grasp_reference is None and self.phase == "grasp":
                self.grasp_reference = local
                events.append(dict(kind="bilateral_pad_contact", object=self.target[0]))
            if self.grasp_reference is not None and self.phase in ("transfer", "lower"):
                self.slip_max = max(self.slip_max, float(np.linalg.norm(local-self.grasp_reference)))
            self.had_grip |= bilateral
        try:
            self.follow(d)
            if d.time-self.began >= self.duration + .12:
                phase = self.phase
                if phase == "settle":
                    self.next_object(d)
                elif phase == "approach":
                    self.move(d, "descend", self.pick, .92)
                elif phase == "descend":
                    self.jaws = self.pinch
                    self.hold(d, "grasp", .38)
                elif phase == "grasp":
                    if not bilateral:
                        raise RuntimeError("No bilateral pad contact")
                    self.grasp_reference = local
                    self.move(d, "lift", np.r_[self.pick[:2], self.carry], .95)
                elif phase == "lift":
                    clearance = float(d.body(self.target[0]).xpos[2]-self.start_z)
                    if clearance < .08 or not bilateral:
                        raise RuntimeError(f"Object failed lift, clearance={clearance:.4f}")
                    self.clearance = clearance
                    self.seating_displacement = float(np.linalg.norm(local-self.grasp_reference))
                    self.grasp_reference = local
                    self.move(d, "transfer", np.r_[self.drop[:2], self.carry],
                              1.15)
                elif phase == "transfer":
                    # Set clearance from the lowest carried primitive at the planned
                    # wrist orientation. Asymmetric objects often tilt between pads.
                    R = self.rotation @ d.site_xmat[self.site].reshape(3,3).T
                    lowest = float('inf')
                    for gid in range(m.ngeom):
                        if m.geom_bodyid[gid] != self.target_body:
                            continue
                        center = R @ (d.geom_xpos[gid]-d.site_xpos[self.site])
                        axes = (R @ d.geom_xmat[gid].reshape(3,3))[2]
                        size = m.geom_size[gid]
                        if m.geom_type[gid] == mj.mjtGeom.mjGEOM_CYLINDER:
                            half_z = size[0]*np.linalg.norm(axes[:2])+size[1]*abs(axes[2])
                        else:
                            half_z = abs(axes) @ size
                        lowest=min(lowest,center[2]-half_z)
                    self.drop[2]=self.table+.009+.012-lowest
                    self.move(d, "lower", self.drop, .90)
                elif phase == "lower":
                    if not bilateral:
                        raise RuntimeError("Lost pad contact before release")
                    self.jaws = OPEN
                    self.hold(d, "release", .40)
                elif phase == "release":
                    self.move(d, "retreat", np.r_[self.drop[:2], self.carry], .82)
                elif phase == "retreat":
                    placement = self.placement(d, self.target, trays)
                    placement.update(clearance_m=self.clearance, maximum_slip_m=self.slip_max,
                                     seating_displacement_m=self.seating_displacement,
                                     bilateral_contact=self.had_grip,
                                     success=placement["placed"] and self.slip_max < .030)
                    self.per_object.append(placement)
                    events.append(dict(kind="placement", **placement))
                    if self.queue:
                        self.next_object(d)
                    else:
                        self.move(d, "return_home", duration=1.55, home=True)
                elif phase == "return_home":
                    self.hold(d, "end_dwell", .8)
                elif phase == "end_dwell":
                    events.append(dict(kind="round_result", **self.complete_round(d, trays)))
                    self.reset_pending = self.cycle < self.rounds
                    if not self.reset_pending:
                        return dict(cycle=self.cycle, phase="complete", done=True, events=events)
        except RuntimeError as error:
            self.fault = dict(cycle=self.cycle, phase=self.phase, error=str(error),
                              object=self.target[0] if self.target else None, time_s=float(d.time),
                              qpos=d.qpos[:9].tolist(), tcp=d.site_xpos[self.site].tolist(),
                              target_xyz=d.body(self.target[0]).xpos.tolist() if self.target else None)
            events.append(dict(kind="failure", **self.fault))
            return dict(cycle=self.cycle, phase="failed", done=True, events=events)
        return dict(cycle=self.cycle, phase=self.phase, done=False, events=events)

    def evaluate(self, m, d):
        completed = len(self.results)
        return dict(success=completed==self.rounds and all(r["success"] for r in self.results),
                    cycles_completed=completed, per_cycle=self.results, fault=self.fault)


def create(model, data, contract, run_dir):
    return Controller(model, data, contract, run_dir)
