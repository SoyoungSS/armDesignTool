"""Create isolated contact fixtures and exercise their normal headless task runner."""
import argparse
import csv
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

from run_task import Session
from runtime import dump


FIXTURES = Path(__file__).resolve().parents[1] / 'assets' / 'collision-fixture'
CASES = ('normal', 'forearm_collision', 'early_drop', 'embedded_block', 'thin_wall', 'thin_wall_plan')


def rule(a, b, mode):
    spec = dict(a=a, b=b, mode=mode, source='Synthetic fixture contact schedule')
    if mode != 'forbid':
        support = b == 'table'
        spec['max_depth_m'] = 0.0005 if support else 0.001
        spec['source'] = ('Nominal loaded fixture peaked at 0.247 mm; support allowance 0.5 mm.'
                          if support else 'Nominal loaded fixture peaked at 0.683 mm; pad allowance 1 mm.')
    return spec


def policy(case):
    if case.startswith('thin_wall'):
        return dict(default='forbid', phases={'initial': [], 'traverse': []},
                    barriers=[dict(wall='wall', moving='tip', axis='x')])
    phases = {}
    for phase in ('initial', 'approach', 'grip', 'lift', 'carry', 'lower', 'release'):
        rules = [rule('forearm', 'table', 'forbid')]
        if phase != 'carry':
            rules.append(rule('block', 'table', 'allow'))
        if phase in ('grip', 'lift', 'carry', 'lower', 'release'):
            mode = 'require' if phase in ('lift', 'carry', 'lower') else 'allow'
            rules.extend(rule(pad, 'block', mode) for pad in ('left_pad', 'right_pad'))
        phases[phase] = rules
    return dict(default='forbid', phases=phases)


def make_project(project, case):
    if case not in CASES:
        raise ValueError(f'Unknown regression case {case}')
    project.mkdir(parents=True, exist_ok=False)
    for name in ('assets', 'scene', 'plan', 'code'):
        (project / name).mkdir()
    wall = case.startswith('thin_wall')
    xml = 'thin-wall.xml' if wall else 'tabletop.xml'
    controller = 'wall_controller.py' if wall else 'tabletop_controller.py'
    shutil.copyfile(FIXTURES / xml, project / 'scene/scene.xml')
    if case == 'embedded_block':
        scene = project / 'scene/scene.xml'
        tree = ET.parse(scene)
        tree.find(".//body[@name='block_body']").set('pos', '0 0 0.010')
        tree.write(scene, encoding='utf-8')
    shutil.copyfile(FIXTURES / controller, project / 'code/task.py')
    dump(project / 'assets/manifest.json', {'fixture': xml, 'units': 'SI'})
    dump(project / 'plan/strategy.json', {'fixture': case, 'controller': controller})
    c = json.loads((FIXTURES.parent / 'task_contract.template.json').read_text(encoding='utf-8'))
    c.update(schema_version='1.1', model_input='synthetic regression fixture', task_description=case,
             fixture_case=case, mode='headless', cycles=1, cycle_definition='one fixture sequence',
             between_cycles='continuous', initial_conditions='model defaults or named moving keyframe',
             initial_keyframe='moving' if wall else None, timing='fixed 2 ms physical steps',
             physics_approval='synthetic fixture only', strategy='deterministic physical regression',
             state_source='simulated body poses', seed=1, dt_s=0.002, max_time_s=1.6,
             robot_joints=['slide'] if wall else ['travel', 'lift', 'elbow', 'left_jaw', 'right_jaw'],
             end_effector_sites=['tcp'], success_criteria=['reach declared endpoint while obeying contact policy'],
             collision_policy=policy(case), unresolved=[])
    c['collision_path_mode'] = 'planned' if case == 'thin_wall_plan' else 'feedback_only'
    if c['collision_path_mode'] == 'feedback_only':
        c['collision_path_reason'] = ('Free moving tip observed at simulation ticks for tunneling regression.'
                                      if wall else 'Physical servo fixture sends actuator targets and measures realized poses.')
    c['confirmation'] = dict.fromkeys(c['confirmation'], True)
    c['confirmation']['evidence'] = 'Synthetic approvals apply only to this regression fixture.'
    dump(project / 'task_contract.json', c)


def run_project(project):
    session = Session(project)
    try:
        while not session.done:
            session.tick()
    except Exception as exc:
        session.finish(repr(exc))
        raise
    session.finish()
    return session.run, json.loads((session.run / 'reports/result.json').read_text(encoding='utf-8'))


def observed_contacts(run):
    with (run / 'data/contacts.csv').open(encoding='utf-8-sig', newline='') as stream:
        return {(row['phase'], frozenset((row['geom1'], row['geom2'])))
                for row in csv.DictReader(stream) if float(row['normal_force_N']) > 0}


def check_case(case, run, result):
    if case == 'normal':
        assert result['status'] == 'passed', result
        active = observed_contacts(run)
        for pad in ('left_pad', 'right_pad'):
            assert ('carry', frozenset((pad, 'block'))) in active, f'No loaded {pad} contact in carry'
        assert ('release', frozenset(('block', 'table'))) in active
        return
    code, phase, pair = {
        'forearm_collision': ('forbidden_contact', 'carry', ('forearm', 'table')),
        'early_drop': ('required_contact_missing', 'lower', ('left_pad', 'block')),
        'thin_wall': ('barrier_crossing', 'traverse', ('tip', 'wall')),
        'thin_wall_plan': ('forbidden_contact', 'traverse', ('tip', 'wall')),
        'embedded_block': ('penetration_limit', 'initial', ('block', 'table')),
    }[case]
    violation = result.get('collision_failure')
    assert result['status'] != 'passed' and violation, result
    assert (violation['code'], violation['phase']) == (code, phase), violation
    if case in ('embedded_block', 'thin_wall_plan'):
        assert result['steps'] == 0, 'Initial and planned collisions must stop before stepping'
    if case == 'thin_wall':
        assert result['steps'] > 0 and not observed_contacts(run), 'Expected a between-sample crossing'
    measured_pair = set(violation['geoms'])
    if case == 'early_drop':
        assert 'block' in measured_pair and measured_pair & {'left_pad', 'right_pad'}, violation
    else:
        assert measured_pair == set(pair), violation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path, help='New directory for isolated projects and evidence')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    for case in CASES:
        project = args.output / case
        make_project(project, case)
        run, result = run_project(project)
        check_case(case, run, result)
        print(f'{case}: OK ({result["reason"]})')


if __name__ == '__main__':
    main()
