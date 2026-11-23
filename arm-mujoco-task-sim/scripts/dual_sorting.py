"""Run three reproducible physical red-left / blue-right sorting rounds."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import sys
import uuid
from datetime import datetime, timezone

HERE = Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'assets/dual-sorting'))
sys.path.insert(0,str(HERE.parent/'assets/dual-sorting/path-checker.zip'))
if os.name == 'nt':
    os.environ.setdefault('ARM_SIM_SYSTEM_MSVC','1')
from runtime import mj, np, dump, Recorder
from scene import build, OBJECTS


def policy_spec():
    roles, supports = {}, []
    for name, color, _, _, parts in OBJECTS:
        roles[name] = [f'part_{name}_{i}' for i in range(len(parts))]
        floor = 'tray_left_base' if color=='red' else 'tray_right_base'
        for surface in ('table_top', floor):
            supports.append(dict(a=name,b=surface,mode='allow',max_depth_m=.003,
                                 source='Support indentation allowance of 3 mm, below the 9 mm tray floor.'))
    phases = {key:supports for key in ('initial','settle','end_dwell','return_home')}
    for name, *_ in OBJECTS:
        for key in ('approach','descend','grasp','lift','transfer','lower','release','retreat'):
            rules = list(supports)
            if key in ('descend','grasp','lift','transfer','lower','release','retreat'):
                for pad in ('pad_a','pad_b'):
                    hold = key in ('lift','transfer','lower')
                    rule = dict(a=name,b=pad,mode='require' if hold else 'allow',max_depth_m=.003,
                                source='3 mm maximum virtual pad indentation, below 5 mm pad half-width.')
                    if hold:
                        rule.update(min_force_N=.025,grace_s=.006)
                    rules.append(rule)
            phases[key+'_'+name] = rules
    return dict(roles=roles,default='forbid',phases=phases,required_phases=[p for p in phases if p!='initial'])


def phase_key(ctl):
    return (ctl.phase+'_'+ctl.target[0] if ctl.target and ctl.phase not in
            ('settle','return_home','end_dwell','complete','failed') else ctl.phase)


def prepare(project,arm,source=None):
    import shutil
    import zipfile
    from contracts import validate

    project = project.resolve()
    assets = project/'assets'
    assets.mkdir(parents=True,exist_ok=True)
    resource = HERE.parent/'assets/dual-sorting'
    info = json.loads((resource/'models.json').read_text(encoding='utf-8'))[arm]
    bundle = resource/(arm+'.zip')
    if hashlib.sha256(bundle.read_bytes()).hexdigest()!=info['archive_sha256']:
        raise ValueError('Example model archive SHA256 mismatch')
    source_run = project/'source'
    source_run.mkdir(exist_ok=True)
    with zipfile.ZipFile(bundle) as zipped:
        zipped.extractall(source_run)
    expected = source_run/'mujoco/robot.xml'
    if source and hashlib.sha256(source.read_bytes()).hexdigest()!=info['model_sha256']:
        raise ValueError('This named example requires its recorded source MJCF')
    if hashlib.sha256(expected.read_bytes()).hexdigest()!=info['model_sha256']:
        raise ValueError('Unpacked model differs from the documented design')
    if source:
        # Upstream mesh contents are resolved using the verified source bundle.
        expected = source.resolve()
        from xml.etree import ElementTree as ET
        tree=ET.parse(expected)
        base=expected.parent/tree.find('compiler').get('meshdir','.')
        for ref in tree.findall('asset/mesh[@file]'):
            r=ref.get('file')
            if (base/r).read_bytes()!=(source_run/'mujoco/meshes'/r).read_bytes():
                raise ValueError('Source visual mesh does not match the bundled design: '+r)
    manifest=build(expected,project,arm)
    manifest['source_model']='source/mujoco/robot.xml'
    dump(assets/'manifest.json',manifest)
    shutil.copy2(bundle,assets/'source_model.zip')
    for name in ('controller.py','scene.py'):
        (project/'code').mkdir(exist_ok=True)
        shutil.copy2(resource/name,project/'code'/name)
    (project/'plan').mkdir(exist_ok=True)
    dump(project/'plan/strategy.json',dict(arm=arm,mode='measured object pose, numerical IK and bounded position servos',
        ordered_phases=['settle','approach','descend','grasp','lift','transfer','lower','release','retreat','return_home','end_dwell'],
        path='Adaptive geometry review of each available joint segment, with tool-relative carried object.',
        acceptance='Bilateral loaded pad contact through lift/transfer/lower; correct support, bounded slip, settled free object.'))
    m=mj.MjModel.from_xml_path(str(project/'scene/scene.xml'))
    contract=dict(**manifest,schema_version='1.1',model_input='source',task_description='Three rounds of four free bodies, red left and blue right.',
        scene='scene/scene.xml',adapter='code/controller.py',asset_manifest='assets/manifest.json',
        strategy_file='plan/strategy.json',robot_joints=[m.joint(i).name for i in range(9)],end_effector_sites=['grip_tcp'],
        initial_keyframe='initial',initial_conditions='Recorded task posture with open jaws and four seeded tabletop objects.',
        mode='headless',cycles=3,cycle_definition='Four physical placements and final dwell, all on the assigned tray.',
        between_cycles='Recorded reset, new permutation, yaw and XY jitter, monotonic clock.',
        timing='Physical integration at the source timestep, output dynamics and full scene playback at 20 Hz and phase boundaries.',
        success_criteria=['Each object has two loaded pads during transport, lifts 80 mm, slips less than 30 mm and settles on its assigned tray.',
                          'Every physics step and actual interpolated arm path pass contact and independent arm-part checks.'],
        physics_approval='Worked example with documented task fixture dimensions, masses, friction and position drives.',
        strategy='Bounded IK at Cartesian waypoints, smooth joint interpolation, frictional grip and gravity release.',
        state_source='MuJoCo measured body and pad contact state, known object color metadata.',seed=20261113,
        dt_s=float(m.opt.timestep),max_time_s=150.,output_root='runs',collision_policy=policy_spec(),
        collision_path_mode='online',unresolved=[],confirmation={k:True for k in ('mode','cycles','task','assets_physics','strategy','timing','outputs')})
    contract['confirmation']['evidence']='Requested two worked physical sorting examples, three rounds of four red/blue objects.'
    dump(project/'task_contract.json',validate(contract,project))


class SortingSession:
    """Same step, reset, contact checks and recorder for batch and Studio."""
    def __init__(self,project,mode='headless'):
        import shutil
        from contracts import validate
        from importlib.util import spec_from_file_location,module_from_spec
        self.project=Path(project).resolve()
        self.c=json.loads((self.project/'task_contract.json').read_text(encoding='utf-8'))
        self.c['mode']=mode
        validate(self.c,self.project)
        self.run=self.project/'runs'/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:8])
        for name in ('data','reports','input'): (self.run/name).mkdir(parents=True)
        for name in ('assets','scene','code','plan'):
            shutil.copytree(self.project/name,self.run/'input'/name,ignore=shutil.ignore_patterns('__pycache__'))
        dump(self.run/'input/task_contract.json',self.c)
        (self.run/'figures').mkdir()
        dump(self.run/'reports/environment.json',dict(python=sys.version,mujoco=mj.__version__,numpy=np.__version__,mode=mode))
        self.m=mj.MjModel.from_xml_path(str(self.run/'input'/self.c['scene']))
        self.d=mj.MjData(self.m)
        limits={}
        for i,name in enumerate(self.c['robot_joints']):
            j=self.m.joint(name).id
            limits[name]=dict(position=dict(lower=float(self.m.jnt_range[j,0]),upper=float(self.m.jnt_range[j,1]),source='MJCF joint range'),
                effort=dict(lower=float(self.m.actuator_forcerange[i,0]),upper=float(self.m.actuator_forcerange[i,1]),source='Source bounded actuator force'))
        dump(self.run/'input/joint_limits_resolved.json',dict(joints=limits))
        self.joints=self.c['robot_joints'][:7]
        code=self.run/'input/code/controller.py'
        spec=spec_from_file_location('sorting_controller',code)
        adapter=module_from_spec(spec)
        spec.loader.exec_module(adapter)
        self.controller=adapter.create(self.m,self.d,self.c,self.run)
        self.policy=self.controller.policy
        self.sample_observer=None
        self.done=self.finalized=self.export_queued=False
        self.reason='running'
        if hashlib.sha256((self.run/'input'/self.c['scene']).read_bytes()).hexdigest()!=self.c['scene_sha256']:
            raise ValueError('Scene modified after preparation; rebuild and review before running.')
        self.next_frame=0.
        self.previous=None
        self.steps=0
        self.data=self.run/'data'
        self.rec=Recorder(self.m,self.data,self.c['robot_joints'],['grip_tcp'])
        self.frames=(self.data/'replay.jsonl').open('w',encoding='utf-8')
        self.stream=(self.data/'arm_path.csv').open('w',newline='')
        self.path=csv.writer(self.stream)
        self.path.writerow(['time_s','cycle','phase',*self.joints])
        self.events=(self.data/'events.jsonl').open('w',encoding='utf-8')
        self.fail=self.policy.observe(self.d,1,'initial')
        from check_path import ArmDistances, read_json
        source=self.project/'source'
        checker=ArmDistances(source,read_json(source/'report/normalized_model.json'))
        initial_arm_contacts=list(checker.contacts(self.d.qpos[:7]))
        if initial_arm_contacts and not self.fail:
            pair,dist,where=initial_arm_contacts[0]
            self.fail=dict(code='initial_arm_interference',geoms=list(pair),distance_m=dist,
                           closest_points_world_m=where,phase='initial',cycle=1,time_s=0.)
        dump(self.run/'reports/preflight.json',dict(collision=self.policy.summary(),failure=self.fail,
             source_model_sha256=self.c['source_sha256'],initial_arm_interferences=initial_arm_contacts,scope='Initialized scene; available online motion checked before each segment.'))
        if self.fail: self.done=True; self.reason='collision_violation'

    def tick(self):
        if self.done: return
        c,m,d=self.controller,self.m,self.d
        if c.reset_pending:
            event=c.start_episode(d)
            self.events.write(json.dumps(dict(time_s=d.time,event=event))+'\n')
            self.fail=self.policy.observe(d,c.cycle,'initial')
            if self.fail:
                self.done=True; self.reason='collision_violation'; return
        mj.mj_forward(m,d)
        before=(d.qpos.copy(),d.qvel.copy(),d.time)
        meta=c.control(m,d)
        if not(np.array_equal(d.qpos,before[0]) and np.array_equal(d.qvel,before[1]) and d.time==before[2]):
            raise RuntimeError('Physical state overwritten during motion')
        phase=phase_key(c)
        mj.mj_forward(m,d)
        self.path.writerow([d.time,c.cycle,phase,*d.qpos[:7]])
        self.fail=self.policy.failure or c.fault or self.policy.observe(d,c.cycle,phase)
        boundary=(c.cycle,phase)!=self.previous
        if boundary or meta['done'] or self.fail or d.time+1e-9>=self.next_frame:
            self.frames.write(json.dumps(dict(time_s=d.time,cycle=c.cycle,phase=phase,qpos=d.qpos.tolist()),separators=(',',':'))+'\n')
            self.rec.sample(d,c.cycle,phase)
            if self.sample_observer: self.sample_observer(d,c.cycle)
            if d.time+1e-9>=self.next_frame: self.next_frame+=.05
        if boundary or meta['events']:
            self.events.write(json.dumps(dict(time_s=d.time,cycle=c.cycle,phase=phase,events=meta['events']))+'\n')
        self.previous=c.cycle,phase
        self.steps+=1
        if meta['done'] or self.fail or d.time>=self.c['max_time_s']:
            if meta['done'] and not self.fail: self.fail=self.policy.finish_task(3)
            self.done=True
            self.reason='collision_violation' if self.fail else 'task_finished' if meta['done'] else 'timeout'
            return
        mj.mj_step(m,d)
        if any(x.number for x in d.warning):
            self.done=True
            self.reason='physics_warning'

    def finish(self,error=None):
        if self.finalized:return
        self.finalized=True
        for file in (self.rec,self.frames,self.stream,self.events):file.close()
        assessment=self.controller.evaluate(self.m,self.d)
        arm_review=dict(status='not_evaluated',reason='Sorting did not finish')
        if self.reason=='task_finished' and assessment['success']:
            from check_path import check
            try:
                arm_review=check(self.project/'source',self.data/'arm_path.csv')
            except (OSError,RuntimeError,TypeError,ValueError,KeyError) as exc:
                arm_review=dict(status='needs_review',error=str(exc))
        dump(self.run/'reports/arm_path_collision.json',arm_review)
        self.result=dict(status='passed' if not error and self.reason=='task_finished' and assessment['success']
                         and not self.fail and arm_review['status']=='passed' else 'incomplete_or_failed',
                         reason=self.reason,error=error,arm=self.c['arm'],duration_s=self.d.time,steps=self.steps,
                         samples=self.rec.samples,collision=self.policy.summary(),collision_failure=self.fail,
                         collision_path=dict(mode='online',segments_checked=self.policy.path_segments),
                         arm_path_collision=arm_review['status'],evaluation=assessment,model_sha256=self.c['source_sha256'],
                         scene_sha256=self.c['scene_sha256'],mujoco=mj.__version__,trajectory='data/arm_path.csv',replay='data/replay.jsonl')
        dump(self.run/'reports/result.json',self.result)
        if self.rec.samples:
            from analyze import analyze
            analyze(self.run)
        print('Run output:',self.run, self.result['status'],self.fail or '',flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--arm',choices=['worksheet','fr3'],required=True)
    p.add_argument('--model',type=Path,help='Optional matching generator robot.xml')
    p.add_argument('--project',type=Path,required=True)
    p.add_argument('--prepare-only',action='store_true')
    args=p.parse_args()
    prepare(args.project,args.arm,args.model)
    if not args.prepare_only:
        s=SortingSession(args.project)
        print('Starting',s.run,flush=True)
        try:
            while not s.done:s.tick()
        finally:s.finish()
        raise SystemExit(0 if s.result['status']=='passed' else 2)
