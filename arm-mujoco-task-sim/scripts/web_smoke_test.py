"""Backend integration test only; it does not claim browser visual acceptance."""
import argparse
import json
from pathlib import Path
import tempfile
from runtime import dump
from self_test import run as fixture
from run_task import Session,launch_viewer
from mujoco.studio import messages,sim


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--port',type=int,default=8879);a=p.parse_args()
    project=Path(a.output)
    fixture(project)
    path=project/'task_contract.json';c=json.loads(path.read_text(encoding='utf-8'));c['mode']='web';dump(path,c)
    s=Session(project)
    def ready(h):
        # Same public event used by the official play button.
        h.dispatch(messages.RequestPauseEvent(pause_state=sim.PauseState.UNPAUSED))
    error=None
    try: launch_viewer(s,exit_on_complete=True,port=a.port,on_ready=ready)
    except BaseException as e: error=repr(e);raise
    finally: s.finish(error)
    result=json.loads((s.run/'reports/result.json').read_text(encoding='utf-8'))
    assert result['status']=='passed' and result['steps']==200,result
    dump(project/'web_smoke_result.json',dict(official_web_server_and_sim_plugin=True,task_completed=True,browser_visual_acceptance='not tested by this script',run=str(s.run)))
