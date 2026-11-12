"""Isolated tool tests; synthetic approvals apply only to this fixture."""
import argparse
import csv
import json
from pathlib import Path
import shutil
from contracts import validate
from runtime import mj,np,dump,Recorder
from run_task import Session

XML='''<mujoco><option timestep="0.001" gravity="0 0 0"/><worldbody>
<body name="object" pos="0 3 0"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body>
<body name="arm"><joint name="turn" type="hinge" axis="0 0 1" range="-120 120"/><geom type="capsule" fromto="0 0 0 1 0 0" size="0.04" mass="1"/>
<body pos="1 0 0"><joint name="slide" type="slide" axis="1 0 0" range="-0.2 0.2"/><geom type="sphere" size="0.05" mass="0.1" contype="0" conaffinity="0"/><site name="tcp"/></body></body>
</worldbody><actuator><motor name="turn_motor" joint="turn" ctrlrange="-20 20"/><motor name="slide_motor" joint="slide" ctrlrange="-20 20"/></actuator></mujoco>'''

ADAPTER='''import math
class Controller:
    def __init__(self): self.completed=set()
    def control(self,m,d):
        t=float(d.time)
        if t>=.1-1e-9: self.completed.add(1)
        if t>=.2-1e-9: self.completed.add(2)
        for i,name in enumerate(['turn','slide']):
            j=m.joint(name).id;q=int(m.jnt_qposadr[j]);v=int(m.jnt_dofadr[j])
            target=.02*math.sin(2*math.pi*t/.1)
            d.ctrl[i]=max(-20,min(20,8*(target-d.qpos[q])-1.2*d.qvel[v]))
        return dict(cycle=min(2,1+int((t+1e-9)/.1)),phase='fixture_motion',done=len(self.completed)==2)
    def evaluate(self,m,d): return dict(success=len(self.completed)==2,cycles_completed=len(self.completed),per_cycle=sorted(self.completed))
def create(m,d,c,run): return Controller()
'''


def run(output):
    out=Path(output).resolve()
    if out.exists(): raise ValueError('Use a new test directory')
    for p in ['scene','code','assets','plan']: (out/p).mkdir(parents=True)
    (out/'scene/scene.xml').write_text(XML)
    (out/'code/task.py').write_text(ADAPTER)
    dump(out/'assets/manifest.json',{'purpose':'tool test only'})
    dump(out/'plan/strategy.json',{'purpose':'tool test only'})
    c=json.loads((Path(__file__).resolve().parents[1]/'assets/task_contract.template.json').read_text(encoding='utf-8'))
    rejected=False
    try: validate(c)
    except ValueError: rejected=True
    assert rejected
    c.update(model_input='synthetic fixture, not a user robot',task_description='Isolated two-cycle recorder verification',initial_conditions='zero joints; free object offset',cycle_definition='0.1 second test interval',between_cycles='continuous',timing='2 x 0.1 s',physics_approval='synthetic automated test only',strategy='deterministic test excitation',state_source='simulation fixture',mode='headless',cycles=2,seed=1,dt_s=.001,max_time_s=.3,robot_joints=['turn','slide'],end_effector_sites=['tcp'],success_criteria=['both test time windows completed'])
    c['confirmation']={k:True for k in ['mode','cycles','task','assets_physics','strategy','timing','outputs']}
    c['confirmation']['evidence']='Synthetic approval for isolated tool self-test, not production task authorization.'
    dump(out/'task_contract.json',c)
    s=Session(out)
    try:
        while not s.done: s.tick()
    finally: s.finish()
    result=json.loads((s.run/'reports/result.json').read_text())
    assert result['status']=='passed' and result['steps']==200 and result['samples']==201
    with (s.run/'data/joints.csv').open(encoding='utf-8-sig') as f: rows=list(csv.DictReader(f))
    assert len(rows)==402 and {r['joint'] for r in rows}=={'turn','slide'}
    assert len(list((s.run/'figures').glob('*.svg')))==25
    # Independent analytic rotating/sliding TCP: r=1, omega=2, radial speed=.3.
    m=mj.MjModel.from_xml_string(XML);d=mj.MjData(m)
    assert int(m.jnt_qposadr[m.joint('turn').id])==7
    d.qvel[int(m.jnt_dofadr[m.joint('turn').id])]=2
    d.qvel[int(m.jnt_dofadr[m.joint('slide').id])]=.3
    mj.mj_forward(m,d);d.qacc[:]=0
    rec=Recorder(m,out/'analytic',['turn','slide'],['tcp']);rec.sample(d,1,'analytic');rec.close()
    with (out/'analytic/end_effectors.csv').open(encoding='utf-8-sig') as f: row=next(csv.DictReader(f))
    measured=np.array([float(row['a_'+a+'_m_s2']) for a in 'xyz'])
    assert np.allclose(measured,[-4,1.2,0],atol=1e-10),measured
    checks=dict(unconfirmed_contract_rejected=True,mixed_joint_types=True,free_body_address_isolation=True,tcp_centripetal_and_coriolis_acceleration=True,complete_two_cycle_export=True,svg_count=25,test_run=str(s.run))
    dump(out/'test_results.json',checks);print(json.dumps(checks,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();run(a.output)
