"""Synthetic regression controller. Angles and forces are never reset during motion."""


def ramp(t, start, end):
    u = min(1.0, max(0.0, (t - start) / (end - start)))
    return u * u * (3 - 2 * u)


class Controller:
    def __init__(self, case):
        self.case = case
        self.lifted = False

    def control(self, m, d):
        t = d.time
        x = 0.06 * ramp(t, 0.55, 0.80)
        z = 0.045 * (ramp(t, 0.26, 0.50) - ramp(t, 0.85, 1.10))
        jaw = 0.012 * (ramp(t, 0.05, 0.15) - ramp(t, 1.20, 1.28))
        elbow = 0.0
        if self.case == 'forearm_collision':
            elbow = -0.12 * (ramp(t, 0.56, 0.68) - ramp(t, 0.70, 0.78))
        if self.case == 'early_drop' and t >= 0.86:
            jaw = 0.0
        d.ctrl[:] = (x, z, elbow, jaw, jaw)
        self.lifted |= d.geom('block').xpos[2] > 0.045
        phase = next((name for until, name in ((0.05, 'approach'), (0.26, 'grip'),
                    (0.55, 'lift'), (0.85, 'carry'), (1.20, 'lower')) if t < until), 'release')
        return dict(cycle=1, phase=phase, done=t >= 1.50)

    def evaluate(self, m, d):
        # This intentionally checks placement only. The runner independently guards the path.
        x, y, z = d.geom('block').xpos
        placed = abs(x - 0.06) < 0.012 and abs(y) < 0.01 and abs(z - 0.016) < 0.002
        success = bool(self.lifted and placed and d.time >= 1.50)
        return dict(success=success, cycles_completed=int(success), final_xyz_m=[float(x), float(y), float(z)])


def create(m, d, contract, run):
    return Controller(contract['fixture_case'])
