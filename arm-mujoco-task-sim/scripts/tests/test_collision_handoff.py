"""Boundary regressions for generated proxies and adapter-supplied trajectories."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from runtime import mj, np
from collision_checks import TaskCollisionChecks, check_inherited_parts


class HandoffTests(unittest.TestCase):
    def setUp(self):
        self.model = mj.MjModel.from_xml_string(
            '<mujoco><worldbody><body name="link"><joint name="slide" type="slide"/>'
            '<geom name="pad" type="box" size=".02 .03 .04"/></body></worldbody></mujoco>')
        self.data = mj.MjData(self.model)
        mj.mj_forward(self.model, self.data)
        self.part = dict(mjcf_geom='pad', link='link', shape='box', size_m=[.04, .06, .08],
                         local_transform=np.eye(4).tolist())
        contract = dict(collision_policy={'phases': {'initial': [], 'move': []}}, cycles=1)
        self.checks = TaskCollisionChecks(self.model, contract)
        self.checks.initialize(self.data, object(), {'collision_review': {
            'parts': [self.part], 'status': 'passed'}})
        self.assertIsNone(self.checks.failure)

    def test_shifted_or_resized_inherited_proxy_rejected(self):
        self.model.geom_pos[0, 0] += .02
        with self.assertRaisesRegex(ValueError, 'local collision placement'):
            check_inherited_parts(self.model, [self.part])
        self.model.geom_pos[0, 0] = 0.
        self.model.geom_size[0, 0] /= 2
        with self.assertRaisesRegex(ValueError, 'compiled dimensions'):
            check_inherited_parts(self.model, [self.part])

    def test_unchecked_gap_between_plan_segments_rejected(self):
        plan = [dict(phase='move', travel_bound_m=.02, sample_qpos=lambda u: [.02*u]),
                dict(phase='move', travel_bound_m=.02, sample_qpos=lambda u: [.05+.02*u])]
        with self.assertRaisesRegex(ValueError, 'connecting motion'):
            self.checks.check_segments(self.data, plan, 1, 'move')

    def test_sampler_cannot_move_the_live_arm(self):
        def mutate(u):
            self.data.qpos[0] = .02*u
            return self.data.qpos
        with self.assertRaisesRegex(ValueError, 'live physics state'):
            self.checks.check_segments(self.data, [dict(phase='move', travel_bound_m=.02,
                                                       sample_qpos=mutate)], 1, 'move')
        np.testing.assert_array_equal(self.data.qpos, [0.])
