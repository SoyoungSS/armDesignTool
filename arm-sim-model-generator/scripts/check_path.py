"""Check actual recorded joint motion against all generated arm collision primitives."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path, PureWindowsPath

import numpy as np

from arm_sim.path_geometry import Primitive, in_fit, mating_regions, motion_bounds
from arm_sim.resources import hashes, write_json


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8'))


class ArmDistances:
    """Use explicit geom distances, including body pairs excluded from engine contacts."""

    def __init__(self, run, normalized):
        import mujoco
        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(run / 'mujoco' / 'robot.xml'))
        self.data = mujoco.MjData(self.model)
        self.joints = [j for j in normalized['model']['joints'] if j['type'] != 'fixed']
        self.addresses = [self.model.joint(j['name']).qposadr[0] for j in self.joints]
        self.parts = [Primitive(link, p) for link, g in normalized['geometry'].items()
                      for p in g['parts'] if p['collision']]
        self.ids = [self.model.geom(p.geom).id for p in self.parts]
        self.regions = mating_regions(normalized, {p.id: p for p in self.parts})
        owners = np.array([p.link for p in self.parts])
        self.left, self.right = np.where(np.triu(owners[:, None] != owners[None, :], 1))
        self.half = np.array([p.half for p in self.parts])
        lever, self.resolution = motion_bounds(normalized['model']['joints'], self.parts)
        self.weights = np.array([lever[j['name']] if j['type']=='revolute' else 1. for j in self.joints])
        self.tol = max(1e-12, np.finfo(float).eps * 128 * max(1., self.half.max(),
                       np.linalg.norm(normalized['installation']['base_in_world']['xyz_m'])))

    def contacts(self, q):
        self.data.qpos[self.addresses] = q
        self.mj.mj_kinematics(self.model, self.data)
        centers = self.data.geom_xpos[self.ids]
        axes = self.data.geom_xmat[self.ids].reshape(-1, 3, 3)
        spans = (abs(axes) @ self.half[:, :, None])[:, :, 0]
        a, b = self.left, self.right
        keep = np.all(spans[a] + spans[b] - abs(centers[a] - centers[b]) > self.tol, axis=1)
        for i, k in zip(a[keep], b[keep]):
            p1, p2 = self.parts[i], self.parts[k]
            pair = (p1.id, p2.id)
            fit = self.regions.get(pair)
            if fit is not None:
                Ta, Tb = np.eye(4), np.eye(4)
                Ta[:3, :3], Ta[:3, 3] = axes[i], centers[i]
                Tb[:3, :3], Tb[:3, 3] = axes[k], centers[k]
                if in_fit(fit, p1, p2, Ta, Tb, self.tol):
                    continue
            points = np.zeros(6)
            dist = self.mj.mj_geomDistance(self.model, self.data, self.ids[i], self.ids[k],
                                           self.resolution, points)
            if not math.isfinite(dist):
                raise ValueError('nonfinite geometry distance for ' + str(pair))
            if dist < -self.tol:
                yield pair, float(dist), points.reshape(2, 3).tolist()


def check(run, trajectory):
    run = run.resolve()
    if (run / 'latest_attempt.json').is_file():
        recorded = read_json(run / 'latest_attempt.json')['run_directory']
        # Resolve within the delivered copy even if an original run still exists.
        run = run / 'runs' / PureWindowsPath(recorded).name
    manifest = read_json(run / 'report' / 'manifest.json')
    inventory = hashes(run)
    if inventory != manifest['files']:
        raise ValueError('generated model inventory or SHA256 mismatch')
    n = read_json(run / 'report' / 'normalized_model.json')
    checker = ArmDistances(run, n)
    names = [j['name'] for j in checker.joints]
    bounds = np.array([j['limits'] for j in checker.joints])
    findings, range_errors = {}, {}
    rows = checks = segments = 0
    prev = None
    seen_cycles = set()
    max_step = 0.
    with trajectory.open(encoding='utf-8-sig', newline='') as stream:
        reader = csv.DictReader(stream)
        header = reader.fieldnames or []
        required = set(names + ['time_s', 'cycle', 'phase'])
        if len(set(header)) != len(header) or not required.issubset(header):
            raise ValueError('trajectory needs unique columns: ' + ', '.join(sorted(required)))
        for row in reader:
            rows += 1
            cycle, phase = row['cycle'], row['phase']
            time = float(row['time_s'])
            q = np.array([float(row[j]) for j in names])
            if not (phase.strip() and cycle.strip() and math.isfinite(time) and time >= 0 and np.all(np.isfinite(q))):
                raise ValueError(f'invalid sample, row {rows}')
            start, start_time, steps = q, time, 1
            if prev is not None and prev[0] == cycle:
                _, start_time, start = prev
                dt = time - start_time
                if dt <= 0:
                    raise ValueError(f'time must increase inside cycle {cycle}, row {rows}')
                max_step = max(max_step, dt)
                steps = max(1, math.ceil(2 * (abs(q-start) @ checker.weights) / checker.resolution))
                if steps > 1_000_000:
                    raise ValueError(f'row {rows}: more than one million subdivisions needed; check SI units or supply physical steps')
                segments += 1
            else:
                if cycle in seen_cycles:
                    raise ValueError('disjoint rows for the same cycle ID')
                seen_cycles.add(cycle)
            for step in range(1, steps+1):
                t = start_time+(time-start_time)*step/steps
                sample = q if step == steps else start+(q-start)*step/steps
                errors = np.maximum(bounds[:, 0]-sample, sample-bounds[:, 1])
                for i in np.flatnonzero(errors > 1e-9):
                    if names[i] not in range_errors or errors[i] > range_errors[names[i]]['excess']:
                        range_errors[names[i]] = dict(joint=names[i], excess=float(errors[i]), value=float(sample[i]),
                                                       limits=bounds[i].tolist(), cycle=cycle, phase=phase, time_s=t)
                for pair, dist, points in checker.contacts(sample):
                    if pair not in findings or dist < findings[pair]['signed_distance_m']:
                        findings[pair] = dict(parts=list(pair), signed_distance_m=dist, closest_points_world_m=points,
                                              cycle=cycle, phase=phase, time_s=t, csv_row=rows+1, q=dict(zip(names,sample)))
                checks += 1
            prev = cycle, time, q
    if not rows:
        raise ValueError('trajectory is empty')
    return dict(status='failed' if findings or range_errors else 'passed', sample_count=rows,
                segment_checks=segments, checked_poses=checks, interpolated_poses=checks-rows,
                max_recorded_step_s=max_step, relative_motion_resolution_m=checker.resolution,
                model_run=str(run), model_input_sha256=manifest['input_sha256'],
                mjcf_sha256=inventory['mujoco/robot.xml'], trajectory_sha256=hashlib.sha256(trajectory.read_bytes()).hexdigest(),
                mujoco_version=checker.mj.__version__, scope='recorded arm-only joint motion, linearly interpolated within each cycle; finite sampling',
                independent_of_contact_filters=True, joint_limit_violations=list(range_errors.values()),
                findings=list(findings.values()), worst_case=min(findings.values(),key=lambda f:f['signed_distance_m'],default=None))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--trajectory', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        result = check(args.run, args.trajectory)
    except (ImportError, ValueError, OSError, KeyError, TypeError, RuntimeError, OverflowError) as exc:
        result = dict(status='needs_review', error=str(exc))
    write_json(args.output, result)
    print(result['status'], args.output)
    return 0 if result['status']=='passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
