"""Portable MuJoCo launcher. Copied into each generated mujoco/ directory.

python run.py [--headless --seconds 5]
The initial keyframe initializes q, actuator controls AND filter activations.
Use viewer actuator controls for targets; position references are radians/metres.
"""
import argparse, time
from pathlib import Path
import numpy as np
import mujoco

def main():
    p=argparse.ArgumentParser(); p.add_argument('--model',type=Path,default=Path(__file__).with_name('robot.xml')); p.add_argument('--headless',action='store_true'); p.add_argument('--seconds',type=float,default=5); a=p.parse_args()
    m=mujoco.MjModel.from_xml_path(str(a.model.resolve())); d=mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m,d,m.key('initial').id); mujoco.mj_forward(m,d)
    def step():
        mujoco.mj_step(m,d)
        if not np.all(np.isfinite(d.qpos)) or np.any(d.warning.number): raise RuntimeError('Simulation warning/nonfinite state')
    if a.headless:
        if not np.isfinite(a.seconds) or a.seconds<=0: p.error('--seconds must be positive and finite')
        for _ in range(int(np.ceil(a.seconds/m.opt.timestep))): step()
        print({'simulation_seconds':d.time,'actuators':m.nu,'qpos':d.qpos.tolist()})
    else:
        from mujoco import viewer as mjviewer
        with mjviewer.launch_passive(m,d) as viewer:
            while viewer.is_running():
                start=time.monotonic()
                with viewer.lock(): step()
                viewer.sync(); time.sleep(max(0,m.opt.timestep-(time.monotonic()-start)))

if __name__=='__main__': main()
