"""CLI entry point; also import arm_sim.pipeline.generate / validate from scripts/."""
import argparse, json, sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from arm_sim.pipeline import generate, validate

def main():
    p=argparse.ArgumentParser(description='Fixed-base serial arm model generator')
    sub=p.add_subparsers(dest='command',required=True)
    gen=sub.add_parser('generate'); gen.add_argument('--input',required=True,type=Path); gen.add_argument('--output',required=True,type=Path); gen.add_argument('--targets',nargs='+',choices=['urdf','ros1','mujoco','isaac']); gen.add_argument('--runtime',action='store_true')
    val=sub.add_parser('validate'); val.add_argument('--output',required=True,type=Path); val.add_argument('--runtime',action='store_true')
    args=p.parse_args()
    try:
        result=generate(args.input,args.output,args.targets,args.runtime) if args.command=='generate' else validate(args.output,args.runtime)
        print(json.dumps(result,ensure_ascii=False,indent=2)); return 1 if result.get('status')=='failed' else 0
    except Exception as e:
        print(json.dumps({'status':'failed','error':str(e)},ensure_ascii=False),file=sys.stderr); return 1

if __name__=='__main__': sys.exit(main())
