"""Build one default arm and check a named operating motion separately from its general range."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

from arm_sim.model import read_input
from arm_sim.pipeline import generate
from arm_sim.resources import write_json
from check_path import check


DEFAULTS = Path(__file__).resolve().parents[1] / 'assets' / 'examples' / 'defaults'


def assess(arm, source, run, output):
    manifest = read_input(run / 'report' / 'manifest.json')
    if hashlib.sha256(source.read_bytes()).hexdigest() != manifest.get('input_sha256'):
        raise ValueError('default input and run fingerprint differ')
    targets = manifest['targets']
    for key in ('urdf', 'mujoco'):
        target = targets[key]
        if not target['generated'] or target['static_checks'] != 'passed' or target['assembly_validation'] != 'passed':
            raise ValueError(f'{key} export, static checks, or assembly checks failed')
    if targets['mujoco']['runtime_validation'] != 'passed':
        raise ValueError('MuJoCo runtime/drive validation is required')
    validation = read_input(run / 'report' / 'validation.json')['collision_review']
    if any(s['status'] != 'passed' for s in validation['contact_surfaces']):
        raise ValueError('contact surface coverage needs review')
    poses = validation['named_pose_results']
    scope = ('geometric_zero', 'initial') if arm=='worksheet' else ('initial', 'offset')
    for label in scope:
        if poses[label]['status'] != 'passed' or not poses[label]['in_joint_ranges']:
            raise ValueError(f'{label}: outside joint range or collision review did not pass')
    q0, q1 = (poses[key]['q'] for key in scope)
    data = read_input(source)
    duration = max(abs(q1[j]-q0[j])/data['drives'][j]['velocity'] for j in q0)
    csv_path = output / 'nominal_motion.csv'
    with csv_path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=['time_s','cycle','phase',*q0])
        writer.writeheader()
        for t, q in [(0., q0), (max(duration, .001), q1)]:
            writer.writerow(dict(time_s=t, cycle=1, phase='nominal_motion', **q))
    trajectory = check(run, csv_path)
    write_json(output / 'nominal_motion_collision.json', trajectory)
    return dict(status=trajectory['status'], arm=arm, run_directory=str(run), scope=list(scope),
                nominal_motion='linear joint-space interpolation of the two named poses',
                nominal_checked_poses=trajectory['checked_poses'], nominal_motion_collision=trajectory['status'],
                full_range_collision=validation['status'], full_range_worst_case=validation['worst_case'],
                task_trajectory='not evaluated by the model example',
                mjcf=str(run / targets['mujoco']['entry']), urdf=str(run / targets['urdf']['entry']),
                preview=str(run / 'preview.png'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm', required=True, choices=('worksheet','fr3'))
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    source = DEFAULTS / (args.arm + '.json')
    result = generate(source, args.output, ['urdf','mujoco'], runtime=True)
    run = Path(result['run_directory'])
    try:
        if result.get('error'):
            raise ValueError(result['error'])
        acceptance = assess(args.arm, source, run, args.output)
    except (ImportError, OSError, ValueError, KeyError, TypeError) as exc:
        acceptance = dict(status='needs_review', arm=args.arm, run_directory=str(run), error=str(exc))
    write_json(args.output / 'default_acceptance.json', acceptance)
    print(json.dumps(acceptance, indent=2, ensure_ascii=False))
    return 0 if acceptance['status']=='passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
