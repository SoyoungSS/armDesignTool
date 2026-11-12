"""Portable contract gate. Confirmation evidence must come from the user."""
import argparse
import json
import math
from pathlib import Path


def validate(c, project=None):
    errors = []
    for key in ['model_input', 'task_description', 'initial_conditions', 'cycle_definition', 'between_cycles',
                'timing', 'physics_approval', 'strategy', 'state_source']:
        if not c.get(key): errors.append('Missing '+key)
    if c.get('mode') not in ['web', 'native', 'headless']: errors.append('Choose mode with user')
    if type(c.get('cycles')) is not int or c['cycles'] < 1: errors.append('Confirm positive integer cycles')
    if type(c.get('seed')) is not int: errors.append('Record an explicit integer seed')
    for key in ['dt_s', 'max_time_s']:
        if not isinstance(c.get(key), (int, float)) or isinstance(c.get(key), bool) or not math.isfinite(c[key]) or c[key] <= 0:
            errors.append('Positive '+key+' required')
    for key in ['robot_joints', 'end_effector_sites', 'success_criteria']:
        if not c.get(key) or not isinstance(c[key], list) or not all(isinstance(x, str) and x.strip() for x in c[key]):
            errors.append('Nonempty string list required: '+key)
    for key in ['robot_joints', 'end_effector_sites']:
        if isinstance(c.get(key), list) and len(set(c[key])) != len(c[key]): errors.append('Duplicate '+key)
    if c.get('unresolved'): errors.append('Unresolved task questions')
    confirmation = c.get('confirmation', {})
    for key in ['mode', 'cycles', 'task', 'assets_physics', 'strategy', 'timing', 'outputs']:
        if confirmation.get(key) is not True: errors.append('User confirmation required: '+key)
    if not confirmation.get('evidence'): errors.append('User reply evidence required')
    if project is not None:
        for key in ['scene', 'adapter', 'asset_manifest', 'strategy_file']:
            if not c.get(key) or not (Path(project)/c[key]).is_file(): errors.append('Missing file '+key)
    if errors: raise ValueError('\n'.join(errors))
    return c


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('contract'); a = p.parse_args()
    path = Path(a.contract)
    validate(json.loads(path.read_text(encoding='utf-8')), path.parent)
    print('Contract gate passed; verify evidence against the actual user conversation.')
