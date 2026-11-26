"""Reusable free-space cyclic workload, independent of manipulation examples.

Copy to project/code/task.py. Explicit contract.workload parameters are required.
Only direct, unit-gear position actuators are supported by this example adapter.
Contact tasks implement their own adapter using the same recorder/spectrum tools.
"""
import math
from runtime import mj, np


class Controller:
    def __init__(self, m, d, contract):
        w = contract['workload']
        self.period = float(w['period_s']); self.settling = float(w['settling_s'])
        self.tolerance = w['completion_tolerance_SI']; self.cycles = contract['cycles']
        self.excursions = w['excursion_by_joint']; self.channels = []
        if not math.isfinite(self.period) or self.period <= 0 or not math.isfinite(self.settling) or self.settling <= 0:
            raise ValueError('Positive finite period_s and settling_s required')
        if set(self.excursions) != set(contract['robot_joints']) or set(self.tolerance) != set(contract['robot_joints']):
            raise ValueError('Specify excursion and tolerance for every recorded arm joint')
        for name in contract['robot_joints']:
            j = m.joint(name).id; q = int(m.jnt_qposadr[j]); v = int(m.jnt_dofadr[j])
            matches = [a for a in range(m.nu) if int(m.actuator_trntype[a]) == int(mj.mjtTrn.mjTRN_JOINT)
                       and int(m.actuator_trnid[a,0]) == j]
            if len(matches) != 1:
                raise ValueError('Example needs one direct position actuator per joint: '+name)
            a = matches[0]
            # Position actuator: positive gain and length feedback -gain. No name heuristics.
            if not np.allclose(m.actuator_gear[a], [1,0,0,0,0,0]) or m.actuator_gainprm[a,0] <= 0 or not np.isclose(m.actuator_biasprm[a,1], -m.actuator_gainprm[a,0]):
                raise ValueError('Example supports unit-gear position servos only: '+name)
            initial = float(d.qpos[q]); excursion = float(self.excursions[name]); tolerance = float(self.tolerance[name])
            if not math.isfinite(excursion) or not math.isfinite(tolerance) or tolerance <= 0:
                raise ValueError('Finite excursion and positive finite tolerance required: '+name)
            if m.jnt_limited[j] and not all(m.jnt_range[j,0] <= x <= m.jnt_range[j,1] for x in [initial,initial+excursion]):
                raise ValueError('Requested workload exceeds joint range: '+name)
            self.channels.append((name,a,q,v,initial,excursion,tolerance))
        self.start = float(d.time); self.finished = False; self.completed = 0

    def control(self, m, d):
        t = max(0., float(d.time)-self.start); total = self.period*self.cycles
        working = t < total
        factor = .5*(1-math.cos(2*math.pi*t/self.period)) if working else 0.
        for name,a,q,v,initial,excursion,tolerance in self.channels:
            d.ctrl[a] = initial+excursion*factor
        self.completed = min(self.cycles, int((t+1e-10)/self.period))
        self.finished = t >= total+self.settling
        return dict(cycle=min(self.cycles,1+int(t/self.period)), phase='excitation' if working else 'settle', done=self.finished)

    def evaluate(self, m, d):
        errors = {name: abs(float(d.qpos[q])-initial) for name,a,q,v,initial,excursion,tolerance in self.channels}
        return dict(success=self.finished and all(errors[name] <= tol for name,a,q,v,initial,excursion,tol in self.channels),
                    cycles_completed=self.completed, completion_errors_SI=errors,
                    scope='commanded workload duration and measured final return; not tracking/drive-capacity acceptance')


def create(model, data, contract, run_dir):
    return Controller(model, data, contract)
