"""Portable contract gate. Confirmation evidence must come from the user."""
import argparse
import json
import math
from pathlib import Path


def strategy_errors(strategy, policy):
    """Check the portable route template when the project adopts its schema."""
    if not isinstance(strategy, dict) or 'strategy_schema' not in strategy:
        return []
    if strategy['strategy_schema'] != '1.0':
        return ['Unknown strategy_schema']
    errors = []
    for field in ('selected_route', 'selection_basis', 'observation_source'):
        if not isinstance(strategy.get(field), str) or not strategy[field].strip():
            errors.append('Fill the selected strategy field: ' + field)
    steps = strategy.get('phases')
    if not isinstance(steps, list) or not steps:
        return errors + ['Define at least one strategy phase']
    names = []
    for index, step in enumerate(steps):
        for field in ('id', 'enter_when', 'command', 'exit_evidence', 'on_failure'):
            value = step.get(field) if isinstance(step, dict) else None
            if not isinstance(value, str) or not value.strip():
                errors.append(f'Strategy phase {index + 1} needs {field}')
        if isinstance(step, dict) and isinstance(step.get('id'), str):
            names.append(step['id'])
    if len(names) != len(set(names)):
        errors.append('Strategy phase IDs must be unique')
    if isinstance(policy, dict) and isinstance(policy.get('phases'), dict):
        expected = set(policy['phases']) - {'initial'}
        if set(names) != expected:
            errors.append(f'Strategy and collision phase IDs differ: {sorted(set(names) ^ expected)}')
    return errors


def validate(c, project=None):
    errors = []
    policy = c.get('collision_policy')
    if c.get('schema_version') == '1.1' or policy is not None:
        if not isinstance(policy, dict) or not isinstance(policy.get('phases'), dict) or 'initial' not in policy['phases']:
            errors.append('Define collision_policy.phases.initial and rules for each operating phase')
    path_mode = c.get('collision_path_mode')
    if path_mode is not None and (path_mode not in ('planned', 'online', 'feedback_only') or not policy):
        errors.append('collision_path_mode requires a policy and a supported path mode')
    if path_mode == 'feedback_only' and not str(c.get('collision_path_reason') or '').strip():
        errors.append('Explain why feedback_only control has no predicted position segment')
    if c.get('schema_version') == '1.1' and path_mode is None:
        errors.append('Choose planned, online or feedback_only collision_path_mode for this task')
    if c.get('schema_version') == '1.1' and isinstance(policy, dict):
        if policy.get('default', 'forbid') != 'forbid':
            errors.append('List intended contact pairs in each phase; new tasks default to forbid')
        phases = policy.get('phases')
        for phase, rules in (phases.items() if isinstance(phases, dict) else ()):
            if not isinstance(rules, list):
                errors.append(f'Expected list of contact rules for {phase}')
                continue
            for rule in rules:
                if isinstance(rule, dict) and rule.get('mode') in ('allow', 'require') and 'max_depth_m' not in rule:
                    errors.append(f'{phase}: set a sourced max_depth_m for {rule.get("a")}/{rule.get("b")}')
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
        if c.get('schema_version') == '1.1' and c.get('strategy_file'):
            strategy_path = Path(project)/c['strategy_file']
            if strategy_path.suffix.lower() == '.json' and strategy_path.is_file():
                errors.extend(strategy_errors(json.loads(strategy_path.read_text(encoding='utf-8')), policy))
    if errors: raise ValueError('\n'.join(errors))
    return c


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('contract'); a = p.parse_args()
    path = Path(a.contract)
    validate(json.loads(path.read_text(encoding='utf-8')), path.parent)
    print('Contract gate passed; verify evidence against the actual user conversation.')
