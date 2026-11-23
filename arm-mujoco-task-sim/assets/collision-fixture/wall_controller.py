class Controller:
    def control(self, model, data):
        return dict(cycle=1, phase='traverse', done=data.time >= 0.004)

    def evaluate(self, model, data):
        reached = bool(data.geom('tip').xpos[0] > 0.01)
        return dict(success=reached, cycles_completed=int(reached))


class PlannedController(Controller):
    def collision_plan(self, model, data):
        initial = data.qpos.copy()
        # The ideal free slide has constant measured velocity until impact.
        travel = float(data.qvel[0]) * 0.002
        yield dict(phase='traverse', sample_qpos=lambda u: initial + u * travel,
                   travel_bound_m=abs(travel), resolution_m=0.0005)

def create(model, data, contract, run):
    return PlannedController() if contract['collision_path_mode'] == 'planned' else Controller()
