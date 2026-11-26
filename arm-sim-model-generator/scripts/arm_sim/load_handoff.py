"""Portable joint/section contract shared by model export and task recording."""
import copy


def build(n, targets):
    joints = []
    for j in n['model']['joints']:
        name = j['name']
        active = j['type'] != 'fixed'
        joints.append(dict(
            name=name, type=j['type'], parent=j['parent'], child=j['child'],
            axis_child=j.get('axis'), origin=copy.deepcopy(j['origin']),
            mjcf_joint=name if active else None,
            position_unit=('rad' if j['type'] == 'revolute' else 'm') if active else None,
            effort_unit=('N m' if j['type'] == 'revolute' else 'N') if active else None,
            drive=copy.deepcopy(n['drives'].get(name)) if active else None,
            section=dict(site='load_site_'+name, force_sensor='load_force_'+name,
                         torque_sensor='load_torque_'+name, body=j['child'],
                         position_in_child_m=[0, 0, 0], quaternion_wxyz=[1, 0, 0, 0])))
    return dict(
        schema_version='1.0', kind='arm-load-handoff', joints=joints,
        active_joint_names=[j['name'] for j in joints if j['mjcf_joint']],
        fixed_joint_names=[j['name'] for j in joints if not j['mjcf_joint']],
        mjcf_entry=targets.get('mujoco', {}).get('entry'),
        section_sensors_generated=targets.get('mujoco', {}).get('generated', False),
        initial_keyframe='initial', end_effector_sites=['tool_frame'],
        installation=copy.deepcopy(n['installation']), model_grade=n['model_grade'],
        missing_physics=n['missing_physics'],
        measurement=dict(frame='instantaneous child-attached site frame', reference='site origin',
                         sign='MuJoCo force/torque sensor convention: child towards parent',
                         meaning='body-tree interface wrench including downstream inertia, contacts and external forces; not individual bearing loads',
                         fixed='fixed connections have section wrench but no motion DOF or actuator effort'),
        limitations=['Rigid-body ideal joints; no bearing load sharing or structural flexibility.',
                     'Actuator generalized effort and section wrench are distinct signals.',
                     'Model generation does not certify task execution or physical accuracy.'])


def check_xml(n, tree):
    """Check every sensor, including fixed connections, without requiring MuJoCo."""
    root = tree.getroot()
    for j in n['model']['joints']:
        name = j['name']
        body = root.find(f'.//body[@name="{j["child"]}"]')
        site = body.find(f'site[@name="load_site_{name}"]') if body is not None else None
        if site is None or site.get('pos') != '0 0 0' or site.get('quat') != '1 0 0 0':
            raise ValueError('Missing or displaced load section: '+name)
        for kind in ('force', 'torque'):
            sensor = root.find(f'sensor/{kind}[@name="load_{kind}_{name}"]')
            if sensor is None or sensor.get('site') != 'load_site_'+name:
                raise ValueError('Missing or mismatched load sensor: '+name)
    return dict(status='passed', section_count=len(n['model']['joints']))
