"""Resolve arm DOFs and load sections by name, never by array prefix."""
import numpy as np
from runtime import mj


def resolve(m, contract, inspection):
    handoff = inspection.get('load_handoff') or {}
    expected = handoff.get('active_joint_names', [])
    requested = contract['robot_joints']
    if requested == 'all':
        if not expected:
            raise ValueError('robot_joints=all requires generator load_handoff or inspected normalized joint map')
        joints = list(expected)
    else:
        joints = list(requested)
    omitted = sorted(set(expected)-set(joints))
    if omitted:
        raise ValueError('Missing arm joints in load recording: '+', '.join(omitted))
    if len(joints) != len(set(joints)):
        raise ValueError('Duplicate recorded joint names')
    sections = []
    for joint in handoff.get('joints', []):
        section = joint.get('section')
        if not section or not handoff.get('section_sensors_generated'):
            continue
        name = joint['name']
        sid = m.site(section['site']).id
        bid = m.body(joint['child']).id
        if int(m.site_bodyid[sid]) != bid or m.body(int(m.body_parentid[bid])).name != joint['parent']:
            raise ValueError('Changed load section body topology: '+name)
        if not np.allclose(m.site_pos[sid], section['position_in_child_m'], atol=1e-12) or not np.allclose(m.site_quat[sid], section['quaternion_wxyz'], atol=1e-12):
            raise ValueError('Changed load section reference frame: '+name)
        if joint.get('mjcf_joint') and int(m.jnt_bodyid[m.joint(joint['mjcf_joint']).id]) != bid:
            raise ValueError('Joint attached to wrong load section body: '+name)
        if joint.get('mjcf_joint'):
            jid = m.joint(joint['mjcf_joint']).id
            kind = mj.mjtJoint.mjJNT_HINGE if joint['type'] == 'revolute' else mj.mjtJoint.mjJNT_SLIDE
            if int(m.jnt_type[jid]) != int(kind):
                raise ValueError('Changed load joint type: '+name)
            if joint.get('axis_child') is not None and not np.allclose(m.jnt_axis[jid], joint['axis_child'], atol=1e-10):
                raise ValueError('Changed load joint axis: '+name)
        for key, kind in [('force_sensor', mj.mjtSensor.mjSENS_FORCE), ('torque_sensor', mj.mjtSensor.mjSENS_TORQUE)]:
            sensor = m.sensor(section[key]).id
            if int(m.sensor_type[sensor]) != int(kind) or int(m.sensor_objid[sensor]) != sid or int(m.sensor_dim[sensor]) != 3:
                raise ValueError('Invalid load section sensor: '+section[key])
        sections.append(dict(joint=name, joint_type=joint['type'], **section))
    coverage = dict(status='complete' if expected else 'unverified_legacy_joint_scope',
                    expected_arm_joints=expected, recorded_joints=joints,
                    extra_joints=sorted(set(joints)-set(expected)) if expected else [],
                    fixed_connections=handoff.get('fixed_joint_names', []),
                    section_status='complete' if sections and len(sections) == len(handoff.get('joints', [])) else 'not_available',
                    section_count=len(sections),
                    source='generator load_handoff' if handoff.get('kind') == 'arm-load-handoff' else 'legacy input')
    return joints, sections, coverage
