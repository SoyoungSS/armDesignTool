"""Replay published FK, gravity, mass matrix and inverse dynamics cases."""
import argparse
import json
from pathlib import Path

import numpy as np

from arm_sim.drives import mass_matrix_and_gravity
from arm_sim.math3d import forward
from arm_sim.model import normalize

LIBRARY = Path(__file__).resolve().parents[1] / 'assets' / 'reference-arms'


def check_calculations(model, case, names):
    q = np.asarray(case['q_rad'])
    velocity = np.asarray(case['qd_rad_s'])
    acceleration = np.asarray(case['qdd_rad_s2'])

    def mass_and_gravity(values):
        return mass_matrix_and_gravity(model, dict(zip(names, values)))

    matrix, gravity = mass_and_gravity(q)
    # C(q,v)v = Mdot v - 1/2 grad_q(v^T M v), central differences at fixed velocity.
    h = 1e-5
    derivative = np.array([(mass_and_gravity(q + delta)[0] - mass_and_gravity(q - delta)[0]) / (2*h)
                           for delta in h * np.eye(len(q))])
    coriolis = np.einsum('i,ijk,k->j', velocity, derivative, velocity)
    coriolis -= 0.5 * np.einsum('j,ijk,k->i', velocity, derivative, velocity)
    torque = matrix @ acceleration + coriolis + gravity
    frame = forward(model, dict(zip(names, q)))['tool_frame']
    for actual, field in [(frame, 'world_T_tool'), (gravity, 'gravity_hold_Nm'),
                          (matrix, 'mass_matrix_kg_m2'), (torque, 'inverse_dynamics_Nm')]:
        np.testing.assert_allclose(actual, case[field], rtol=1e-6, atol=2e-6, err_msg=field)


def check_mujoco(path, cases, names):
    import mujoco as mj
    m = mj.MjModel.from_xml_path(str(path))
    d = mj.MjData(m)
    assert [m.joint(i).name for i in range(m.njnt)] == names
    for case in cases:
        d.qpos[:] = case['q_rad']
        d.qvel[:] = 0
        mj.mj_forward(m, d)
        np.testing.assert_allclose(d.qfrc_bias, case['gravity_hold_Nm'], rtol=1e-6, atol=2e-6)
        tool = d.body('tool_frame') if path.suffix == '.urdf' else d.site('tool_frame')
        frame = np.eye(4)
        frame[:3, :3], frame[:3, 3] = tool.xmat.reshape(3, 3), tool.xpos
        np.testing.assert_allclose(frame, case['world_T_tool'], atol=2e-6)
        matrix = np.empty((m.nv, m.nv))
        try:
            mj.mj_fullM(m, d, matrix)
        except TypeError:  # 3.3.x binding
            mj.mj_fullM(m, matrix, d.qM)
        np.testing.assert_allclose(matrix, case['mass_matrix_kg_m2'], rtol=1e-6, atol=2e-6)
        d.qvel[:] = case['qd_rad_s']
        mj.mj_forward(m, d)
        d.qacc[:] = case['qdd_rad_s2']
        torque = np.zeros(m.nv)
        mj.mj_rne(m, d, 1, torque)
        np.testing.assert_allclose(torque, case['inverse_dynamics_Nm'], rtol=1e-6, atol=2e-6)


def check_usd(path, model):
    from pxr import Usd, UsdGeom, UsdPhysics
    stage = Usd.Stage.Open(str(path))
    assert stage and stage.GetDefaultPrim() and UsdGeom.GetStageMetersPerUnit(stage) == 1
    assert UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z
    cache = UsdGeom.XformCache()
    frames = forward(model, model['poses']['initial'])
    for link, expected in frames.items():
        prim_path = link if link != 'tool_frame' else model['model']['joints'][-1]['child'] + '/tool_frame'
        prim = stage.GetPrimAtPath('/Robot/' + prim_path)
        assert prim, prim_path
        np.testing.assert_allclose(np.asarray(cache.GetLocalToWorldTransform(prim)).T, expected, atol=2e-6)
    for joint in model['model']['joints']:
        prim = stage.GetPrimAtPath('/Robot/' + joint['name'])
        assert prim.IsA(UsdPhysics.RevoluteJoint)
        articulation = UsdPhysics.RevoluteJoint(prim)
        np.testing.assert_allclose([articulation.GetLowerLimitAttr().Get(), articulation.GetUpperLimitAttr().Get()],
                                   np.rad2deg(joint['limits']), atol=1e-4)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm', choices=('ur5e', 'franka_fr3'))
    parser.add_argument('--mujoco', action='store_true', help='also load both URDF and MJCF')
    parser.add_argument('--usd', action='store_true', help='also load the composed OpenUSD articulation')
    args = parser.parse_args()
    for arm in ([args.arm] if args.arm else ['ur5e', 'franka_fr3']):
        folder = LIBRARY / arm
        model = normalize(json.loads((folder / 'input.json').read_text(encoding='utf-8')))
        # Both reference inputs provide one whole-body COM tensor per moving link.
        model['mass_properties'] = {link: body[0] for link, body in model['components'].items()}
        golden = json.loads((folder / 'expected.json').read_text(encoding='utf-8'))
        names = [joint['name'] for joint in model['model']['joints']]
        assert names == golden['joint_order']
        for case in golden['cases']:
            check_calculations(model, case, names)
        if args.mujoco:
            for name in ('robot.urdf', 'robot.xml'):
                check_mujoco(folder/name, golden['cases'], names)
        if args.usd:
            check_usd(folder/'robot.usd', model)
        print(f'{arm}: {len(golden["cases"])} reference configurations passed'
              f' (URDF/MJCF checked: {args.mujoco}, USD checked: {args.usd})')


if __name__ == '__main__':
    main()
