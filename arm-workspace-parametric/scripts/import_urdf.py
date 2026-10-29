#!/usr/bin/env python3
"""Convert a named URDF serial path to a workspace request; no mesh/runtime dependencies."""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def convert(text, base_link, tip_link, samples=5000, seed=42):
    if '<!DOCTYPE' in text.upper() or '<!ENTITY' in text.upper():
        raise ValueError('DOCTYPE/entity declarations are not supported')
    robot = ET.fromstring(text)
    links = {link.attrib['name'] for link in robot.findall('link')}
    if base_link not in links or tip_link not in links:
        raise ValueError('base-link and tip-link must name URDF links')
    parents = {}
    for joint in robot.findall('joint'):
        child = joint.find('child').attrib['link']
        if child in parents:
            raise ValueError('multiple parents for URDF link: ' + child)
        parents[child] = joint
    path, visited, current = [], set(), tip_link
    while current != base_link:
        if current in visited or current not in parents:
            raise ValueError('no acyclic directed path from base-link to tip-link')
        visited.add(current)
        joint = parents[current]
        path.append(joint)
        current = joint.find('parent').attrib['link']

    def vec(raw):
        result = [float(x) for x in raw.split()]
        if len(result) != 3 or not all(math.isfinite(x) for x in result):
            raise ValueError('expected finite three-vector')
        return result

    joints = []
    for joint in reversed(path):
        kind = joint.attrib['type']
        if kind not in ('revolute', 'prismatic', 'fixed') or joint.find('mimic') is not None:
            raise ValueError('only non-mimic revolute/prismatic/fixed joints supported')
        origin = joint.find('origin')
        attributes = {} if origin is None else origin.attrib
        row = {'name': joint.attrib['name'], 'type': kind, 'origin': {
            'xyz_m': vec(attributes.get('xyz', '0 0 0')),
            'rpy_rad': vec(attributes.get('rpy', '0 0 0'))}}
        if kind != 'fixed':
            axis, limit = joint.find('axis'), joint.find('limit')
            row['axis'] = vec('1 0 0' if axis is None else axis.attrib['xyz'])
            if math.hypot(*row['axis']) < 1e-12 or limit is None:
                raise ValueError('active joint requires nonzero axis and finite limits')
            row['limits'] = [float(limit.attrib[key]) for key in ('lower', 'upper')]
            if not all(math.isfinite(x) for x in row['limits']) or row['limits'][0] >= row['limits'][1]:
                raise ValueError('invalid joint limits')
        joints.append(row)
    if not joints or not any(j['type'] != 'fixed' for j in joints):
        raise ValueError('path must contain active joints')
    if not 1 <= samples <= 50000 or not 0 <= seed <= 4294967295:
        raise ValueError('samples/seed outside workspace contract')
    return {'schema_version': '1.0', 'mode': 'joint_sampling', 'model': {
        'name': robot.attrib.get('name', 'robot') + ':' + base_link + '->' + tip_link,
        'joints': joints, 'tip': {'xyz_m': [0, 0, 0], 'rpy_rad': [0, 0, 0]}},
        'base_in_world': {'xyz_m': [0, 0, 0], 'rpy_rad': [0, 0, 0]},
        'sampling': {'method': 'random', 'samples': samples, 'seed': seed},
        'metadata': {'base_link': base_link, 'tip_link': tip_link,
                     'scope': 'URDF serial geometry only; meshes, collision and mimic constraints not imported'}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('input', 'output', 'base-link', 'tip-link'):
        parser.add_argument('--' + key, required=True)
    parser.add_argument('--samples', type=int, default=5000)
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()
    source, target = Path(args.input), Path(args.output)
    if source.resolve() == target.resolve():
        raise ValueError('input and output must differ')
    if source.stat().st_size > 10 * 1024 * 1024:
        raise ValueError('URDF exceeds 10 MiB')
    request = convert(source.read_text(encoding='utf-8-sig'), args.base_link, args.tip_link, args.samples, args.seed)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(request, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'ok': True, 'joints': len(request['model']['joints']), 'output': str(target.resolve())}))


if __name__ == '__main__':
    try:
        main()
    except (ValueError, KeyError, OSError, ET.ParseError) as error:
        import sys
        print(json.dumps({'ok': False, 'error': str(error)}), file=sys.stderr)
        sys.exit(1)
