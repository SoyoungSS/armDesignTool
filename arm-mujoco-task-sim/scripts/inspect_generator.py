"""Read-only generator output discovery and manifest integrity check; stdlib only."""
import argparse
import hashlib
import json
from pathlib import Path


def inspect(path):
    p = Path(path).resolve()
    explicit = p if p.is_file() and p.suffix.lower() == '.xml' else None
    if p.is_file(): p = p.parent
    if p.name == 'mujoco': p = p.parent
    attempt_status = None
    if not (p/'report/manifest.json').is_file() and (p/'latest_attempt.json').is_file():
        attempt = json.loads((p/'latest_attempt.json').read_text(encoding='utf-8'))
        attempt_status = attempt.get('status')
        run_id = attempt.get('run_id')
        directory = attempt.get('run_directory')
        if directory:
            declared = Path(directory.replace('\\', '/'))
            p = (declared if declared.is_absolute() else p/declared).resolve()
        elif run_id: p = (p/'runs'/run_id).resolve()
        else: raise ValueError('latest_attempt has no run_directory or run_id; request the actual run')
    manifest_path = p/'report/manifest.json'
    if not manifest_path.is_file(): raise ValueError('Generator manifest missing; request the complete generated run')
    m = json.loads(manifest_path.read_text(encoding='utf-8'))
    target = m.get('targets', {}).get('mujoco', {})
    problems = []
    if attempt_status not in [None, 'completed']: problems.append('Latest attempt was not completed: '+str(attempt_status))
    if target.get('generated') is not True or target.get('status') == 'failed': problems.append('MuJoCo generation not successful')
    if target.get('static_checks') != 'passed': problems.append('Static checks not passed')
    entry = (p/str(target.get('entry', '')).replace('\\', '/')).resolve()
    if not entry.is_relative_to(p): problems.append('Model entry escapes run directory')
    if not entry.is_file(): problems.append('Manifest entry missing: '+str(entry))
    if explicit is not None and explicit != entry.resolve(): problems.append('Explicit XML differs from manifest entry; confirm provenance')
    verified = 0
    for rel, expected in m.get('files', {}).items():
        f = (p/rel.replace('\\', '/')).resolve()
        if not f.is_relative_to(p): problems.append('Manifest path escapes run: '+rel); continue
        if not f.is_file(): problems.append('Missing file: '+rel); continue
        digest = hashlib.sha256(f.read_bytes()).hexdigest()
        if digest != expected: problems.append('Hash mismatch: '+rel)
        else: verified += 1
    if not m.get('files'): problems.append('No integrity file inventory')
    validation = p/'report/validation.json'
    review = {'status': 'not_evaluated'}
    if validation.is_file():
        review = json.loads(validation.read_text(encoding='utf-8')).get('collision_review', review)
    if review.get('status') == 'failed':
        problems.append('Confirmed model interference. Inspect collision_review in validation.json.')
    handoff = None
    handoff_rel = m.get('load_handoff')
    if handoff_rel:
        handoff_path = (p/handoff_rel).resolve()
        if not handoff_path.is_relative_to(p) or handoff_rel not in m.get('files', {}):
            problems.append('Load handoff must belong to the verified file inventory')
        elif handoff_path.is_file():
            handoff = json.loads(handoff_path.read_text(encoding='utf-8'))
    elif (p/'report/normalized_model.json').is_file():
        normalized = json.loads((p/'report/normalized_model.json').read_text(encoding='utf-8'))
        js = normalized.get('model', {}).get('joints', [])
        handoff = dict(active_joint_names=[j['name'] for j in js if j['type'] != 'fixed'],
                       fixed_joint_names=[j['name'] for j in js if j['type'] == 'fixed'],
                       section_sensors_generated=False)
    return dict(run=str(p), entry=str(entry), run_id=m.get('run_id'), mujoco=target,
                load_handoff=handoff,
                collision_review=review,
                model_grade=m.get('model_grade'), verified_files=verified, problems=problems,
                ready_for_compile=not problems,
                reports={n: str(p/'report'/n) for n in ['normalized_model.json', 'drive_design.json', 'validation.json', 'input_snapshot.json', 'load_handoff.json'] if (p/'report'/n).is_file()})


if __name__ == '__main__':
    a = argparse.ArgumentParser(); a.add_argument('input'); a.add_argument('--out'); args = a.parse_args()
    result = inspect(args.input)
    text = json.dumps(result, ensure_ascii=False, indent=2)
    if args.out:
        dest = Path(args.out); dest.parent.mkdir(parents=True, exist_ok=True); dest.write_text(text, encoding='utf-8')
    print(text)
    if not result['ready_for_compile']: raise SystemExit(2)
