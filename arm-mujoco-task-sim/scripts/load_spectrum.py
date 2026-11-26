"""Stream complete signed joint histories into time-at-load and effort/speed spectra.

No task-specific object names, commanded-load substitutions or fatigue assumptions.
Intervals use their left sample; physical reset boundaries are excluded explicitly.
"""
import argparse
from bisect import bisect_right
import csv
import hashlib
import json
import math
from pathlib import Path


SECTION_SIGNALS = {**{f'force_{a}_N': 'N' for a in 'xyz'},
                   **{f'torque_{a}_Nm': 'N m' for a in 'xyz'}}


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def intervals(path, expected, signals, resets):
    """Yield each measured state and its previous valid same-episode interval."""
    previous = {}
    group, seen = None, set()
    with path.open(encoding='utf-8-sig', newline='') as f:
        for row in csv.DictReader(f):
            name = row['joint']
            if name not in expected:
                raise ValueError(f'{path.name}: unregistered joint {name}')
            t = float(row['time_s']); cycle = int(row['cycle']); phase = row['phase']
            values = {s: float(row[s]) for s in signals}
            if not math.isfinite(t) or not all(math.isfinite(v) for v in values.values()):
                raise ValueError(f'{path.name}: nonfinite state at {name}/{t}')
            key = (t, cycle, phase)
            if group != key:
                if group is not None and seen != expected:
                    raise ValueError(f'{path.name}: incomplete joint sample {group}: {sorted(expected-seen)}')
                group, seen = key, set()
            if name in seen:
                raise ValueError(f'{path.name}: duplicate joint sample {name}/{t}')
            seen.add(name)
            old = previous.get(name)
            dt, reason = 0., None
            if old:
                ot, oc, op, ov = old
                if t <= ot and t not in resets:
                    raise ValueError(f'{path.name}: non-increasing time without reset: {name}/{t}')
                if any(ot < r <= t for r in resets) or t <= ot:
                    reason = 'reset_boundary'
                else:
                    dt = t-ot
            yield name, cycle, phase, values, dt, old, reason
            previous[name] = (t, cycle, phase, values)
    if group is not None and seen != expected:
        raise ValueError(f'{path.name}: incomplete final joint sample: {sorted(expected-seen)}')
    if set(previous) != expected:
        raise ValueError(f'{path.name}: missing joint histories: {sorted(expected-set(previous))}')


def scopes(cycle, phase):
    return [('all', 'all'), ('cycle', str(cycle)), ('phase', phase)]


def edges(lo, hi, bins):
    # A nonzero bracket also gives a meaningful bin for constant/zero histories.
    if hi == lo:
        pad = max(abs(lo)*1e-6, 1e-9)
        lo, hi = lo-pad, hi+pad
    return [lo+(hi-lo)*i/bins for i in range(bins+1)]


def bin_index(bounds, value):
    return min(len(bounds)-2, max(0, bisect_right(bounds, value)-1))


def write_csv(path, fields, rows):
    temp = path.with_suffix(path.suffix+'.tmp')
    with temp.open('w', encoding='utf-8-sig', newline='') as f:
        w = csv.writer(f); w.writerow(fields); w.writerows(rows)
    temp.replace(path)


def _export(run, bins=32):
    if type(bins) is not int or not 1 <= bins <= 256:
        raise ValueError('bins must be an integer in 1..256')
    run = Path(run); data, reports = run/'data', run/'reports'
    reports.mkdir(exist_ok=True)
    meta = read_json(data/'metadata.json')
    joints = {j['name']: j for j in meta['joints']}
    sections = {s['joint']: s for s in meta.get('load_sections', [])}
    resets = []
    if (data/'events.jsonl').is_file():
        with (data/'events.jsonl').open(encoding='utf-8') as f:
            for row in map(json.loads, f):
                if row.get('event', {}).get('kind') in ('episode_start', 'episode_reset'):
                    resets.append(float(row['time_s']))
    tables = [('generalized', data/'joints.csv', set(joints), ['effort', 'velocity', 'power_W'])]
    if sections:
        tables.append(('section', data/'joint_loads.csv', set(sections), list(SECTION_SIGNALS)))
    stats, ranges, excluded = {}, {}, {}
    for kind, path, expected, signals in tables:
        for name, cycle, phase, values, dt, old, reason in intervals(path, expected, signals, resets):
            if reason:
                excluded[(kind, name)] = excluded.get((kind, name), 0)+1
            for signal, value in values.items():
                rk = (kind, name, signal)
                low, high = ranges.get(rk, (value, value))
                ranges[rk] = min(low, value), max(high, value)
                for sk, sv in scopes(cycle, phase):
                    key = (*rk, sk, sv)
                    s = stats.setdefault(key, dict(min=value, max=value, samples=0, duration=0., integral=0., square=0.))
                    s['min'] = min(s['min'], value); s['max'] = max(s['max'], value); s['samples'] += 1
                if dt:
                    # Time belongs to the earlier phase, never to the phase entered at its end.
                    for sk, sv in scopes(old[1], old[2]):
                        s = stats[(*rk, sk, sv)]; v = old[3][signal]
                        s['duration'] += dt; s['integral'] += v*dt; s['square'] += v*v*dt
    bounds = {k: edges(*v, bins) for k, v in ranges.items()}
    hist, speed_hist = {}, {}
    for kind, path, expected, signals in tables:
        for name, cycle, phase, values, dt, old, reason in intervals(path, expected, signals, resets):
            if not dt:
                continue
            for signal, value in old[3].items():
                index = bin_index(bounds[(kind, name, signal)], value)
                for sk, sv in scopes(old[1], old[2]):
                    key = (kind, name, signal, sk, sv, index)
                    hist[key] = hist.get(key, 0.)+dt
            if kind == 'generalized':
                effort = bin_index(bounds[(kind, name, 'effort')], old[3]['effort'])
                speed = bin_index(bounds[(kind, name, 'velocity')], old[3]['velocity'])
                for sk, sv in scopes(old[1], old[2]):
                    key = (name, sk, sv, effort, speed)
                    speed_hist[key] = speed_hist.get(key, 0.)+dt

    def unit(kind, name, signal):
        if kind == 'section': return SECTION_SIGNALS[signal]
        if signal == 'power_W': return 'W'
        if signal == 'effort': return joints[name]['effort_unit']
        return 'rad/s' if joints[name]['type'] == 'hinge' else 'm/s'

    rows = []
    for (kind, name, signal, sk, sv), s in sorted(stats.items()):
        duration = s['duration']
        rows.append([kind, name, signal, unit(kind, name, signal), sk, sv, s['samples'],
                     s['min'], s['max'], max(abs(s['min']), abs(s['max'])), duration,
                     s['integral']/duration if duration else None,
                     math.sqrt(s['square']/duration) if duration else None])
    write_csv(reports/'load_statistics.csv',
              ['kind','joint','signal','unit','scope','scope_id','samples','min','max','peak_abs','duration_s','time_mean','time_rms'], rows)
    rows = []
    for key, s in sorted(stats.items()):
        kind, name, signal, sk, sv = key; b = bounds[(kind, name, signal)]
        for i in range(bins):
            duration = hist.get((*key, i), 0.)
            rows.append([kind,name,signal,unit(kind,name,signal),sk,sv,i,b[i],b[i+1],duration,
                         duration/s['duration'] if s['duration'] else None])
    write_csv(reports/'load_spectrum.csv',
              ['kind','joint','signal','unit','scope','scope_id','bin','lower','upper','duration_s','time_fraction'], rows)
    rows = []
    for (name, sk, sv, ei, vi), duration in sorted(speed_hist.items()):
        eb = bounds[('generalized',name,'effort')]; vb = bounds[('generalized',name,'velocity')]
        total = stats[('generalized',name,'effort',sk,sv)]['duration']
        rows.append([name,sk,sv,unit('generalized',name,'effort'),unit('generalized',name,'velocity'),
                     eb[ei],eb[ei+1],vb[vi],vb[vi+1],duration,duration/total])
    write_csv(reports/'effort_speed_spectrum.csv',
              ['joint','scope','scope_id','effort_unit','speed_unit','effort_lower','effort_upper','speed_lower','speed_upper','duration_s','time_fraction'], rows)
    fingerprints = {}
    for path in [data/'metadata.json', data/'events.jsonl', *[t[1] for t in tables]]:
        if path.is_file():
            h = hashlib.sha256()
            with path.open('rb') as f:
                for chunk in iter(lambda: f.read(1024*1024), b''): h.update(chunk)
            fingerprints[path.relative_to(run).as_posix()] = h.hexdigest()
    summary = dict(schema_version='1.0', status='exported', bins=bins,
                   joint_names=list(joints), section_names=list(sections),
                   coverage=meta.get('load_coverage', {'status':'unverified_legacy_joint_scope'}),
                   section_status='available' if sections else 'not_available',
                   weighting='left sample held over next interval; excludes physical reset boundaries; continuous cycle/phase transitions belong to the left state; terminal sample has zero duration',
                   histogram='signed time-at-load; per-joint global uniform edges shared by all/cycle/phase scopes; final bin includes upper edge',
                   effort_speed='sparse occupied signed effort x signed speed bins; absent bins have zero duration',
                   excluded_intervals=[dict(kind=k[0],joint=k[1],count=v) for k,v in sorted(excluded.items())],
                   source_sha256=fingerprints,
                   task_result=read_json(reports/'result.json') if (reports/'result.json').is_file() else None,
                   limitations=['Finite simulated duty only; no service-life extrapolation or fatigue damage.',
                                'Time-at-load is not rainflow cycle counting.',
                                'No motor-shaft conversion without transmission data.',
                                'Section wrench is not individual bearing load.',
                                'Successful export does not imply successful task or complete requested duty.'])
    (reports/'load_spectrum.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    return summary


def export(run, bins=32):
    reports = Path(run)/'reports'; reports.mkdir(exist_ok=True)
    marker = reports/'load_spectrum.json'
    marker.write_text(json.dumps(dict(status='running')), encoding='utf-8')
    try:
        return _export(run, bins)
    except Exception as exc:
        marker.write_text(json.dumps(dict(status='failed', error=str(exc)), ensure_ascii=False), encoding='utf-8')
        raise


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('run', type=Path); p.add_argument('--bins', type=int, default=32)
    a = p.parse_args(); print(json.dumps(export(a.run, a.bins), ensure_ascii=False, indent=2))
