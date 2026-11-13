"""Execute a task adapter with the same recorder in web/native/headless mode."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import random
import shutil
import subprocess
import sys
import time
import uuid
from contracts import validate
from runtime import mj, np, dump, Recorder


class Session:
    def __init__(self, project):
        self.project = Path(project).resolve()
        self.c = validate(json.loads((self.project/'task_contract.json').read_text(encoding='utf-8')), self.project)
        self.run = self.project/self.c.get('output_root','runs')/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:8])
        self.run.mkdir(parents=True)
        for folder in ['input','data','figures','reports','logs']: (self.run/folder).mkdir()
        dump(self.run/'input/task_contract.json',self.c)
        copied = {}
        # Snapshot the executable project assets so subsequent edits do not alter provenance.
        for folder in ['scene','assets','plan','code']:
            src = self.project/folder
            if src.is_dir():
                shutil.copytree(src,self.run/'input'/folder,ignore=shutil.ignore_patterns('__pycache__','.venv','runs'))
        shutil.copytree(Path(__file__).resolve().parent,self.run/'input/runner_tools',ignore=shutil.ignore_patterns('__pycache__'))
        if (self.project/'model_inspection.json').is_file(): shutil.copy2(self.project/'model_inspection.json',self.run/'input/model_inspection.json')
        for f in (self.run/'input').rglob('*'):
            if f.is_file(): copied[f.relative_to(self.run).as_posix()] = hashlib.sha256(f.read_bytes()).hexdigest()
        dump(self.run/'reports/environment.json',dict(mujoco=mj.__version__,numpy=np.__version__,python=platform.python_version(),system=platform.platform(),mode=self.c['mode']))
        self.m = mj.MjModel.from_xml_path(str(self.project/self.c['scene']))
        limits_path = self.run/'input/assets/joint_limits.json'
        limits = json.loads(limits_path.read_text(encoding='utf-8')) if limits_path.is_file() else {'joints':{}}
        for name in self.c['robot_joints']:
            j = self.m.joint(name).id
            signals = limits['joints'].setdefault(name,{})
            if self.m.jnt_limited[j]:
                lower,upper = map(float,self.m.jnt_range[j])
                signals.setdefault('position',dict(lower=lower,upper=upper,source='MJCF joint range'))
        dump(self.run/'input/joint_limits_resolved.json',limits)
        copied['input/joint_limits_resolved.json'] = hashlib.sha256(
            (self.run/'input/joint_limits_resolved.json').read_bytes()).hexdigest()
        dump(self.run/'input/sha256.json',copied)
        self.d = mj.MjData(self.m)
        self.m.opt.timestep = self.c['dt_s']
        if self.c.get('initial_keyframe') is not None:
            key = self.m.key(self.c['initial_keyframe']).id
            mj.mj_resetDataKeyframe(self.m,self.d,key)
        self.d.time = 0
        mj.mj_forward(self.m,self.d)
        random.seed(self.c['seed']); np.random.seed(self.c['seed'])
        sys.path.insert(0,str((self.project/self.c['adapter']).parent))
        spec = importlib.util.spec_from_file_location('task_adapter',self.project/self.c['adapter'])
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        self.controller = module.create(self.m,self.d,self.c,self.run)
        self.rec = Recorder(self.m,self.run/'data',self.c['robot_joints'],self.c['end_effector_sites'])
        self.events = (self.run/'data/events.jsonl').open('w',encoding='utf-8')
        self.last_phase = None; self.done = False; self.reason = 'running'; self.finalized = False
        self.export_queued = False
        self.started = time.perf_counter(); self.steps = 0
        self.sample_observer = None

    def tick(self):
        if self.done: return
        m,d = self.m,self.d
        mj.mj_forward(m,d)
        state = (d.qpos.copy(),d.qvel.copy(),float(d.time))
        meta = self.controller.control(m,d)
        if not (np.array_equal(d.qpos,state[0]) and np.array_equal(d.qvel,state[1]) and d.time==state[2]):
            raise RuntimeError('Controller overwrote physical state; implement explicit recorded reset segments instead')
        cycle,phase = meta['cycle'],meta['phase']
        if type(cycle) is not int or not 1 <= cycle <= self.c['cycles']: raise ValueError('Invalid cycle counter')
        mj.mj_forward(m,d)
        if not all(np.isfinite(a).all() for a in [d.qpos,d.qvel,d.qacc,d.qfrc_actuator,d.ctrl]): raise RuntimeError('Nonfinite dynamics')
        self.rec.sample(d,cycle,phase)
        if self.sample_observer is not None: self.sample_observer(d,cycle)
        if (cycle,phase)!=self.last_phase:
            self.events.write(json.dumps(dict(time_s=float(d.time),cycle=cycle,phase=phase),ensure_ascii=False)+'\n'); self.last_phase=(cycle,phase)
        for event in meta.get('events',[]): self.events.write(json.dumps(dict(time_s=float(d.time),event=event),ensure_ascii=False)+'\n')
        if meta.get('done'):
            self.done=True; self.reason='task_finished'; return
        if self.steps*self.c['dt_s'] >= self.c['max_time_s']-1e-10:
            self.done=True; self.reason='timeout'; return
        mj.mj_step(m,d); self.steps+=1
        if any(w.number for w in d.warning):
            self.done=True; self.reason='physics_warning'

    def finish(self, error=None):
        if self.finalized: return
        self.finalized=True; self.rec.close(); self.events.close()
        try: evaluation=self.controller.evaluate(self.m,self.d)
        except Exception as e: evaluation=dict(success=False,evaluation_error=repr(e))
        passed = error is None and self.reason=='task_finished' and evaluation.get('success') is True and evaluation.get('cycles_completed')==self.c['cycles']
        result=dict(status='passed' if passed else 'incomplete_or_failed',reason=self.reason,error=error,duration_s=float(self.d.time),steps=self.steps,samples=self.rec.samples,wall_time_s=time.perf_counter()-self.started,evaluation=evaluation,warnings=[int(w.number) for w in self.d.warning])
        dump(self.run/'reports/result.json',result)
        analysis_python = os.environ.get('ARM_SIM_ANALYSIS_PYTHON')
        if analysis_python:
            subprocess.run([analysis_python, '-B', str(Path(__file__).with_name('analyze.py')),
                            str(self.run)], check=True)
        else:
            from analyze import analyze
            analyze(self.run)
        print('Run output:',self.run,flush=True)


def launch_viewer(session,exit_on_complete=False,port=0,on_ready=None,open_browser=False,
                  on_session=None,web_idle_s=45):
    from mujoco.studio import app, launch_web, launch_native, messages, plugin, sim, viewer_protocol
    from viewer_ui import viewer_plugins
    s=session
    exports = []
    worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='task-export')
    class Control(plugin.SimPlugin):
        def __init__(self): self.paused=True; self.speed=1.; self.last=time.perf_counter(); self.budget=0.; self.single=False
        def publish(self):
            self.handle.send_to_viewer(messages.StepControlSnapshot(pause_state=sim.PauseState.NORMAL_PAUSED if self.paused else sim.PauseState.UNPAUSED,speed=100*self.speed))
        @messages.handler
        def loaded(self,e:messages.PostModelEvent): self.publish()
        @messages.handler
        def pause(self,e:messages.RequestPauseEvent):
            self.paused=e.pause_state!=sim.PauseState.UNPAUSED or s.done; self.budget=0.; self.last=time.perf_counter(); self.publish(); return True
        @messages.handler
        def settings(self,e:messages.StepControlSnapshot):
            self.speed=max(.01,min(10.,e.speed/100)); self.paused=e.pause_state!=sim.PauseState.UNPAUSED or s.done
            self.budget=0.; self.last=time.perf_counter(); self.publish(); return True
        @messages.handler
        def single_step(self,e:messages.SingleStepEvent): self.single=True; self.paused=True; return True
        @messages.handler
        def reset(self,e:messages.ResetEvent):
            nonlocal s
            self.paused=True; self.single=False; self.budget=0.
            old=s
            replacement=type(old)(old.project)
            if not old.done:
                old.events.write(json.dumps(dict(time_s=float(old.d.time),event=dict(
                    kind='viewer_reset',next_run=replacement.run.name)))+'\n')
                old.done=True; old.reason='viewer_reset'
            if not old.finalized and not old.export_queued:
                exports.append(worker.submit(old.finish))
            s=replacement
            plots.reset_series(s)
            s.sample_observer=plots.sample
            if on_session is not None:
                on_session(s)
            self.handle.set_model(s.m,path=str(s.project/s.c['scene']),data=s.d)
            self.last=time.perf_counter()
            self.publish()
            return True
        @messages.handler
        def block_load(self,e:messages.LoadModelEvent): return True
        @messages.handler
        def block_state(self,e:messages.StateEvent): return True
        @messages.handler
        def block_physics_options(self,e:messages.MjOptionSnapshot): return True
        @messages.handler
        def block_perturb(self,e:messages.PerturbSnapshot): return True
        @messages.handler
        def step(self,e:messages.StepEvent):
            for job in tuple(exports):
                if job.done():
                    exports.remove(job)
                    job.result()
            now=time.perf_counter(); delta=now-self.last; self.last=now
            if self.single and not s.done:
                s.tick(); self.single=False
            if not self.paused and not s.done:
                self.budget=min(.1,self.budget+delta*self.speed); deadline=now+.01
                while self.budget>=s.c['dt_s'] and not s.done and time.perf_counter()<deadline:
                    s.tick(); self.budget-=s.c['dt_s']
            if s.done and not s.finalized and not s.export_queued:
                self.paused=True; self.publish()
                s.export_queued=True
                exports.append(worker.submit(s.finish))
            time.sleep(.001)
    idle = None
    original = None
    if s.c['mode']=='web':
        from mujoco.studio.web import web_server
        original=web_server.bind_public_socket
        web_server.bind_public_socket=lambda host,port: original('127.0.0.1',port)
        launcher=launch_web
    else: launcher=launch_native
    plots, idle = viewer_plugins(s.c, web_idle_s if s.c['mode']=='web' else 0)
    plots.reset_series(s)
    s.sample_observer=plots.sample
    config=viewer_protocol.ViewerConfig(title='MuJoCo task simulation',gfx='web' if s.c['mode']=='web' else '',http_port=port,open_browser=open_browser)
    h=None
    error=None
    try:
        h=launcher.launch(config,plugins=[app.App(app.Config(show_toolbar=True,show_options=False,
                                                           show_inspector=False)),plots,idle,Control()])
        h.set_model(s.m,path=str(s.project/s.c['scene']),data=s.d)
        if on_ready is not None: on_ready(h)
        while h.is_running():
            h.sync()
            if exit_on_complete and s.done: break
            if idle.expired(): break
    except KeyboardInterrupt:
        if not s.done: s.reason='viewer_closed'
    except BaseException as exc:
        error=repr(exc)
        if not s.done: s.reason='exception'
        raise
    finally:
        try:
            if h is not None: h.close()
        finally:
            if original is not None: web_server.bind_public_socket=original
            worker.shutdown(wait=True)
            if not s.done and s.reason=='running': s.reason='viewer_closed'
            s.finish(error)
            for job in exports: job.result()


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--project',required=True);p.add_argument('--port',type=int,default=0);p.add_argument('--exit-on-complete',action='store_true');p.add_argument('--open-browser',action='store_true');a=p.parse_args()
    s=Session(a.project);err=None
    try:
        if s.c['mode']=='headless':
            while not s.done: s.tick()
        else: launch_viewer(s,a.exit_on_complete,a.port,open_browser=a.open_browser)
    except BaseException as e:
        err=repr(e);s.reason='exception';raise
    finally: s.finish(err)
