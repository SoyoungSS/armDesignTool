"""Isaac Sim 5.1 smoke-test helper. Run with Isaac's python, NOT the core Python.

This separate optional helper never marks the offline manifest runtime-passed.
Requires GPU/Isaac; generated evidence must be reviewed with FK/drive results.
"""
import argparse, json, sys, traceback
from pathlib import Path

def main():
    p=argparse.ArgumentParser(); p.add_argument('--run',type=Path,required=True); p.add_argument('--evidence',type=Path,required=True); p.add_argument('--expected-version',default='5.1'); p.add_argument('--gui',action='store_true'); a=p.parse_args()
    result={'status':'failed','requested_version':a.expected_version,'scope':'articulation name/initial-state/finite short-step smoke test; full pose validation remains separate'}; app=None
    try:
        from isaacsim import SimulationApp
        app=SimulationApp({'headless':not a.gui})
        import numpy as np
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.stage import open_stage
        from isaacsim.core.version import get_version
        from pxr import UsdPhysics
        import omni.usd
        version=get_version(); result['actual_version']=str(version)
        if not str(version[0]).startswith(a.expected_version): raise RuntimeError('Version mismatch: review target-version API before running helper')
        n=json.loads((a.run/'report'/'normalized_model.json').read_text(encoding='utf-8'))
        if n['missing_physics']: raise RuntimeError('Visualization USD has no complete physical model')
        open_stage(str((a.run/'isaac'/'robot.usd').resolve())); app.update()
        stage=omni.usd.get_context().get_stage(); roots=[str(x.GetPath()) for x in stage.Traverse() if x.HasAPI(UsdPhysics.ArticulationRootAPI)]
        if len(roots)!=1: raise RuntimeError('Expected exactly one articulation root: '+str(roots))
        dt=n.get('drive_design',{}).get('timestep_s',.001)
        world=World(stage_units_in_meters=1.0,physics_dt=dt,rendering_dt=.01)
        arm=world.scene.add(SingleArticulation(prim_path=roots[0],name='arm')); world.reset()
        expected=n['poses']['initial']; names=arm.dof_names
        if set(names)!=set(expected): raise RuntimeError('DOF naming mismatch '+str(names))
        values=np.array([expected[k] for k in names]); arm.set_joint_positions(values); arm.set_joint_velocities(np.zeros(len(names)))
        np.testing.assert_allclose(arm.get_joint_positions(),values,atol=1e-5)
        for _ in range(50): world.step(render=a.gui)
        positions=arm.get_joint_positions(); velocities=arm.get_joint_velocities()
        if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(velocities)): raise RuntimeError('Nonfinite physics state')
        result.update(status='smoke_passed',articulation_root=roots[0],dof_names=names,steps=50,positions=positions.tolist(),velocities=velocities.tolist(),full_acceptance='pending: compare all body/tip poses and drive behavior in target runtime')
    except Exception as e: result['error']=str(e)
    finally:
        a.evidence.parent.mkdir(parents=True,exist_ok=True); a.evidence.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        if app: app.close()
    print(json.dumps(result,ensure_ascii=False)); return 0 if result['status']=='smoke_passed' else 1

if __name__=='__main__': sys.exit(main())
