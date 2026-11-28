"""Run the project's scene Skill Session and retain measured browser replay frames."""
import argparse
import json
from pathlib import Path
import sys

p=argparse.ArgumentParser()
p.add_argument('--skill',required=True);p.add_argument('--project',required=True);p.add_argument('--summary',required=True)
p.add_argument('--preflight',action='store_true')
a=p.parse_args()
sys.path.insert(0,str(Path(a.skill)/'scripts'))
from run_task import Session
project=Path(a.project).resolve()
s=Session(project)
summary={'run_directory':str(s.run),'preflight':a.preflight}
if a.preflight:
    s.rec.close();s.events.close()
    summary['ok']=not bool(s.checks.failure)
    Path(a.summary).write_text(json.dumps(summary),encoding='utf8')
    raise SystemExit(0 if summary['ok'] else 2)
replay=s.run/'data/replay.jsonl'
next_frame=[0.0]
with replay.open('w',encoding='utf8') as stream:
    def observe(data,cycle):
        if float(data.time)+1e-10>=next_frame[0]:
            stream.write(json.dumps(dict(time_s=float(data.time),cycle=cycle,qpos=data.qpos.tolist()))+'\n')
            next_frame[0]=float(data.time)+.04
    s.sample_observer=observe
    error=None
    try:
        while not s.done:s.tick()
    except BaseException as exc:
        error=repr(exc);s.reason='exception'
    finally:
        s.finish(error)
# Assign phases from actual recorded transition events rather than another control call.
events=[]
for line in (s.run/'data/events.jsonl').read_text(encoding='utf8').splitlines():
    row=json.loads(line)
    if 'phase' in row:events.append(row)
rows=[];index=0
for line in replay.read_text(encoding='utf8').splitlines():
    row=json.loads(line)
    while index+1<len(events) and events[index+1]['time_s']<=row['time_s']+1e-9:index+=1
    row['phase']=events[index]['phase'] if events else 'unrecorded'
    rows.append(json.dumps(row))
replay.write_text('\n'.join(rows)+('\n' if rows else ''),encoding='utf8')
summary.update(ok=s.result['status']=='passed',status=s.result['status'],error=error)
Path(a.summary).write_text(json.dumps(summary),encoding='utf8')
# Task failure is a valid measured outcome; the HTTP layer reads reports/result.json.
