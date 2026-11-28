"""Build offline all-joint HTML and lossless per-module CSVs from one measured run."""
import argparse
import base64
import csv
import hashlib
from html import escape
import itertools
import json
import math
import os
from pathlib import Path
import re
import tempfile
from urllib.parse import quote
import xml.etree.ElementTree as ET

VERSION = '1.0'
LEAD = ['time_s', 'cycle', 'phase']
STATE = ['position', 'velocity', 'acceleration', 'effort', 'bias', 'passive',
         'constraint', 'applied', 'power_W', 'limit_margin']
SECTION = ([f'force_{a}_N' for a in 'xyz'] + [f'torque_{a}_Nm' for a in 'xyz'] +
           [f'point_world_{a}_m' for a in 'xyz'] + [f'rotation_world_{i}' for i in range(9)])
DRIVE = ['ctrl', 'force', 'length', 'velocity', 'ctrl_at_limit']
FIELDS = (LEAD + ['joint', 'joint_type', 'state_available', 'section_available',
          'position_unit', 'effort_unit'] + STATE + ['position_deg', 'velocity_deg_s',
          'acceleration_deg_s2', 'load_site'] + SECTION + ['actuator_count', 'actuator_names'] +
          ['drive_'+s for s in DRIVE])


def read_json(path, default=None):
    return json.loads(path.read_text(encoding='utf-8')) if path.is_file() else default


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name+'.partial')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    os.replace(temporary, path)


def records(path):
    if path.is_file():
        with path.open(encoding='utf-8-sig', newline='') as stream:
            yield from csv.DictReader(stream)


def number(value):
    if value in ('', None):
        return None
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('Nonfinite measured value')
    return value


def frames(path, identity):
    def key(row):
        time = number(row['time_s'])
        if time is None:
            raise ValueError('Missing simulation time')
        return time, row['cycle'], row['phase']
    for key_value, rows in itertools.groupby(records(path), key):
        group = {}
        for row in rows:
            name = row[identity]
            if name in group:
                raise ValueError(f'Duplicate {identity} at the same sample: {name}')
            group[name] = row
        yield key_value, group


class Aligned:
    def __init__(self, iterator):
        self.iterator = iter(iterator)
        self.next = next(self.iterator, None)

    def take(self, key):
        if self.next is None:
            return {}
        if self.next[0] != key:
            raise ValueError('State/load/actuator samples are not exactly aligned in time, cycle and phase')
        result = self.next[1]
        self.next = next(self.iterator, None)
        return result


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def chart_records(run):
    result = []
    for path in sorted((run/'figures').glob('*.svg')):
        # Only analyze.py's numbered, generated charts are eligible for embedding.
        if not re.fullmatch(r'\d+-[A-Za-z0-9_-]+\.svg', path.name):
            continue
        if not path.resolve().is_relative_to(run):
            raise ValueError('Chart escapes run')
        root = ET.parse(path).getroot()
        title = next(root.iter('{http://www.w3.org/2000/svg}text'), None)
        name, separator, signal = (title.text or '').partition(' | ') if title is not None else ('', '', '')
        if separator:
            result.append(dict(name=name, signal=signal, svg=str(path.relative_to(run)),
                               png=str(path.with_suffix('.png').relative_to(run)) if path.with_suffix('.png').is_file() else None))
    return result


def export(run):
    run = Path(run).resolve()
    reports = run/'reports'
    if not reports.resolve().is_relative_to(run):
        raise ValueError('Report directory escapes run')
    reports.mkdir(exist_ok=True)
    index = reports/'joint_exports.json'
    write_json(index, dict(schema_version=VERSION, status='building'))
    try:
        with tempfile.TemporaryDirectory(prefix='.joint-export-', dir=run) as temporary:
            stage = Path(temporary)
            result = build(run, stage)
            for file in sorted(stage.rglob('*')):
                if file.is_file():
                    target = run/file.relative_to(stage)
                    if not target.resolve().is_relative_to(run):
                        raise ValueError('Export target escapes run')
                    target.parent.mkdir(parents=True, exist_ok=True)
                    os.replace(file, target)
            write_json(index, result)
            return result
    except Exception as exc:
        write_json(index, dict(schema_version=VERSION, status='failed', error=str(exc)))
        (reports/'report.html').write_text('<!doctype html><meta charset="utf-8"><h1>全关节报告整理失败</h1><p>'+escape(str(exc))+'</p>', encoding='utf-8')
        raise


def build(run, stage):
    for relative in ['data', 'figures', 'data/joints.csv', 'data/joint_loads.csv', 'data/actuators.csv', 'data/metadata.json',
                     'reports/load_statistics.csv', 'reports/result.json', 'input/task_contract.json']:
        if not (run/relative).resolve().is_relative_to(run):
            raise ValueError('Export source escapes run')
    meta = read_json(run/'data/metadata.json')
    if not isinstance(meta, dict):
        raise ValueError('Missing recorded metadata')
    joint_meta = {j['name']: j for j in meta.get('joints', [])}
    sections = {s['joint']: s for s in meta.get('load_sections', [])}
    names = list(dict.fromkeys([*meta.get('load_coverage', {}).get('expected_arm_joints', []), *joint_meta, *sections]))
    if not names:
        raise ValueError('No recorded joint scope')
    joint_ids = {j.get('id'): name for name, j in joint_meta.items() if 'id' in j}
    actuator_joint = {a['name']: joint_ids[a['transmission_ids'][0]] for a in meta.get('actuators', [])
                      if a.get('transmission_type') in (0, 1) and a.get('transmission_ids') and a['transmission_ids'][0] in joint_ids}
    active = [name for name in names if name in joint_meta or name not in sections]
    modules = []
    (stage/'data/joint_modules').mkdir(parents=True)
    (stage/'reports').mkdir()
    handles = []

    def writer(relative, fields):
        handle = (stage/relative).open('w', encoding='utf-8-sig', newline='')
        handles.append(handle)
        output = csv.DictWriter(handle, fieldnames=fields, extrasaction='ignore')
        output.writeheader()
        return output

    outputs, drives = {}, {}
    for i, name in enumerate(names):
        stem = f'{i+1:03d}-'+re.sub(r'[^A-Za-z0-9_-]', '_', name)
        info = joint_meta.get(name, sections.get(name, {}))
        entry = dict(joint=name, joint_type=info.get('type', info.get('joint_type', 'unknown')),
                     state_csv=f'data/joint_modules/{stem}_state.csv',
                     actuator_csv=f'data/joint_modules/{stem}_actuators.csv' if name in actuator_joint.values() else None,
                     samples=0, state_samples=0, section_samples=0, actuator_samples=0, actuator_complete_frames=0,
                     expected_actuators=[a for a, joint in actuator_joint.items() if joint == name], peaks={}, ranges={}, charts=[])
        modules.append(entry)
        outputs[name] = writer(entry['state_csv'], FIELDS)
        if entry['actuator_csv']:
            drives[name] = writer(entry['actuator_csv'], LEAD+['actuator']+DRIVE)
    wide_signals = ['position', 'velocity', 'acceleration', 'effort', 'power_W', 'limit_margin']
    wide = writer('data/joint_states_wide.csv', LEAD+[name+'.'+signal for name in active for signal in wide_signals])
    load_frames = Aligned(frames(run/'data/joint_loads.csv', 'joint'))
    drive_frames = Aligned(frames(run/'data/actuators.csv', 'actuator'))
    partial = False
    frame_count = 0
    try:
        for key, states in frames(run/'data/joints.csv', 'joint'):
            frame_count += 1
            if set(states)-set(names):
                raise ValueError('State CSV contains joints outside metadata scope')
            loads, controls = load_frames.take(key), drive_frames.take(key)
            if set(loads)-set(names):
                raise ValueError('Load CSV contains interfaces outside metadata scope')
            lead = {field: next(iter(states.values()))[field] for field in LEAD}
            wide_row = dict(lead)
            by_joint = {name: [] for name in names}
            for actuator, row in controls.items():
                if actuator in actuator_joint:
                    name = actuator_joint[actuator]
                    by_joint[name].append(row)
                    drives[name].writerow(row)
            for entry in modules:
                name = entry['joint']
                state, load = states.get(name, {}), loads.get(name, {})
                partial |= (name in active and not state) or (name in sections and not load)
                controls_complete = all(a in controls for a in entry['expected_actuators'])
                partial |= not controls_complete
                entry['samples'] += 1
                entry['state_samples'] += bool(state)
                entry['section_samples'] += bool(load)
                entry['actuator_samples'] += len(by_joint[name])
                entry['actuator_complete_frames'] += controls_complete
                row = dict(lead, joint=name, joint_type=entry['joint_type'], state_available=int(bool(state)),
                           section_available=int(bool(load)), position_unit=joint_meta.get(name, {}).get('position_unit', ''),
                           effort_unit=joint_meta.get(name, {}).get('effort_unit', ''), load_site=load.get('site', ''),
                           actuator_count=len(by_joint[name]), actuator_names=';'.join(r['actuator'] for r in by_joint[name]))
                row.update({signal: state.get(signal, '') for signal in STATE})
                row.update({signal: load.get(signal, '') for signal in SECTION})
                if entry['joint_type'] == 'hinge':
                    for signal, column in [('position', 'position_deg'), ('velocity', 'velocity_deg_s'), ('acceleration', 'acceleration_deg_s2')]:
                        value = number(state.get(signal))
                        row[column] = math.degrees(value) if value is not None else ''
                if len(by_joint[name]) == 1:
                    row.update({'drive_'+signal: by_joint[name][0].get(signal, '') for signal in DRIVE})
                for signal in STATE+SECTION[:6]:
                    value = number(row[signal])
                    if value is not None:
                        entry['peaks'][signal] = max(entry['peaks'].get(signal, 0.), abs(value))
                        old = entry['ranges'].get(signal, [value, value])
                        entry['ranges'][signal] = [min(old[0], value), max(old[1], value)]
                outputs[name].writerow(row)
                if name in active:
                    wide_row.update({name+'.'+signal: state.get(signal, '') for signal in wide_signals})
            wide.writerow(wide_row)
        if load_frames.next or drive_frames.next:
            raise ValueError('Load/actuator CSV contains samples absent from state CSV')
    finally:
        for handle in handles:
            handle.close()
    partial |= frame_count == 0
    charts = chart_records(run)
    statistics = list(records(run/'reports/load_statistics.csv'))
    for entry in modules:
        entry['charts'] = [c for c in charts if c['name'] == entry['joint']]
        rms = next((s.get('time_rms') for s in statistics if s.get('kind') == 'generalized' and
                    s.get('joint') == entry['joint'] and s.get('signal') == 'effort' and s.get('scope') == 'all'), None)
        entry['effort_time_rms'] = number(rms)
        entry['status'] = ('no_samples' if not frame_count else 'missing_state' if entry['joint'] in active and entry['state_samples'] != frame_count else
                           'missing_section' if entry['joint'] in sections and entry['section_samples'] != frame_count else
                           'missing_actuator' if entry['actuator_complete_frames'] != frame_count else
                           'complete' if entry['section_samples'] else 'state_only')
    summary_fields = ['joint', 'joint_type', 'status', 'samples', 'state_samples', 'section_samples', 'actuator_samples',
                      'position_min', 'position_max', 'peak_velocity', 'peak_acceleration', 'peak_effort', 'effort_time_rms', 'peak_power_W']
    with (stage/'reports/joint_module_summary.csv').open('w', encoding='utf-8-sig', newline='') as stream:
        output = csv.DictWriter(stream, fieldnames=summary_fields)
        output.writeheader()
        for e in modules:
            output.writerow({**{k: e.get(k, '') for k in summary_fields[:7]},
                             'position_min': e['ranges'].get('position', ['', ''])[0], 'position_max': e['ranges'].get('position', ['', ''])[1],
                             **{'peak_'+k: e['peaks'].get(k, '') for k in ['velocity', 'acceleration', 'effort', 'power_W']},
                             'effort_time_rms': e['effort_time_rms']})
    dictionary = dict(schema_version=VERSION, sample=meta.get('sample'), original_metadata='data/metadata.json',
        units=dict(position='position_unit: rad for hinge, m for slide', velocity='rad/s or m/s', acceleration='rad/s^2 or m/s^2',
                   effort='effort_unit: N m for hinge, N for slide', power_W='W; signed effort times velocity',
                   position_deg='deg; hinges only', velocity_deg_s='deg/s; hinges only', acceleration_deg_s2='deg/s^2; hinges only',
                   force='N', torque='N m', point_world='m', rotation_world='dimensionless row-major R_world_site'),
        sections=meta.get('load_section_semantics', {}),
        semantics=['Rows are exactly joined by simulation time, cycle and phase; no resampling or averaging.',
                   'Blank means missing/not applicable, not zero. Fixed interfaces have load data without fabricated motion DOFs.',
                   'effort is qfrc_actuator, not a bearing wrench. constraint is generalized constraint effort, not six-axis interface force.',
                   'drive_* values are raw actuator-space signals, populated only for one directly mapped joint actuator; see each actuator CSV for multiple drives.',
                   'No motor-shaft, gearbox, bearing, temperature, current or life estimates without the corresponding parameters and measurements.',
                   'Charts may be extrema-downsampled; exported CSVs preserve every recorded sample. RMS uses reports/load_statistics.csv time-weighted method.'])
    write_json(stage/'reports/data_dictionary.json', dictionary)
    sources = ['data/joints.csv', 'data/joint_loads.csv', 'data/actuators.csv', 'data/metadata.json',
               'reports/result.json', 'reports/load_statistics.csv', 'input/task_contract.json']
    result = dict(schema_version=VERSION, run_id=run.name, status='partial' if partial else 'exported', frames=frame_count,
                  html='reports/report.html', wide_csv='data/joint_states_wide.csv', summary_csv='reports/joint_module_summary.csv',
                  dictionary='reports/data_dictionary.json', modules=modules, source_coverage=meta.get('load_coverage', {'status':'unverified_legacy_joint_scope'}),
                  sources_sha256={f: digest(run/f) for f in sources if (run/f).is_file()})
    build_html(run, stage, result, meta, dictionary, charts)
    return result


def build_html(run, stage, index, meta, dictionary, charts):
    task = read_json(run/'input/task_contract.json', {})
    result = read_json(run/'reports/result.json', {})
    def show(value):
        return '—' if value is None else escape(str(value))
    def link(file, title):
        return '<a href="'+escape('../'+quote(file, safe='/'))+'">'+escape(title)+'</a>'
    def image(chart):
        encoded = base64.b64encode((run/chart['svg']).read_bytes()).decode('ascii')
        return '<figure><img loading="lazy" alt="'+escape(chart['name']+' '+chart['signal'], quote=True)+'" src="data:image/svg+xml;base64,'+encoded+'"></figure>'
    html = ['<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">',
            '<meta http-equiv="Content-Security-Policy" content="default-src \'none\'; img-src data:; style-src \'unsafe-inline\'; base-uri \'none\'; form-action \'none\'">',
            '<title>全关节状态与载荷报告</title><style>body{font:15px/1.7 system-ui;max-width:1250px;margin:28px auto;padding:0 20px;color:#293139}a{color:#85384c;margin-right:12px}table{border-collapse:collapse;width:100%;font-size:13px}th,td{border:1px solid #ddd;padding:7px;text-align:left}.scroll{overflow:auto}.charts{display:grid;grid-template-columns:repeat(auto-fit,minmax(330px,1fr));gap:12px}figure{margin:0}img{width:100%;height:auto}section{margin:30px 0;border-top:1px solid #ddd}summary{cursor:pointer;font-weight:600}pre{white-space:pre-wrap;overflow-wrap:anywhere}nav{position:sticky;top:0;background:#fff;padding:10px;border:1px solid #ddd}h1,h2{color:#85384c}.note{background:#f6f1f3;padding:12px}</style></head><body>',
            '<h1>全关节状态与载荷报告</h1><p>'+escape(task.get('task_description', '未记录任务描述'))+'</p>',
            '<p>本次运行：'+escape(index['run_id'])+'</p>',
            '<p>任务结果：'+show(result.get('status'))+' · 结束原因：'+show(result.get('reason'))+' · 实际时长 '+show(result.get('duration_s'))+' s · 物理记录 '+str(index['frames'])+' 帧 · 关节/接口 '+str(len(index['modules']))+' 个 · 导出状态 '+index['status']+'</p>',
            '<p>上游关节覆盖：'+show(index['source_coverage'].get('status'))+'。整理成功不自动证明未登记的关节已被记录，覆盖范围以记录元数据和 load_coverage 为准。</p>',
            '<p class="note">本 HTML 的全部曲线已嵌入，可单文件离线查看。完整 CSV 保存在 ZIP 的 data/joint_modules/；解压整个 ZIP 后，下方数据链接可直接打开。CSV 保留全部样本，曲线可能采用保极值降采样。导出成功不等于任务成功，失败或截断工况按原结果保留。</p>',
            '<nav>'+''.join(f'<a href="#joint-{i}">{escape(e["joint"])}</a>' for i, e in enumerate(index['modules']))+'</nav>',
            '<h2>全关节数据</h2><p>'+link(index['wide_csv'], '全关节状态宽表 CSV')+link(index['summary_csv'], '关节模组统计 CSV')+link(index['dictionary'], '字段与单位说明 JSON')+'</p>',
            '<div class="scroll"><table><tr><th>关节/接口</th><th>覆盖</th><th>状态样本</th><th>六维载荷样本</th><th>驱动输出绝对峰值</th><th>驱动输出时间 RMS</th><th>完整数据</th></tr>']
    for e in index['modules']:
        html.append('<tr>'+''.join('<td>'+v+'</td>' for v in [escape(e['joint']), e['status'], str(e['state_samples']), str(e['section_samples']), show(e['peaks'].get('effort')), show(e['effort_time_rms']), link(e['state_csv'], '模组状态 CSV')+(link(e['actuator_csv'], '执行器 CSV') if e['actuator_csv'] else '')])+'</tr>')
    html.append('</table></div><p>旋转关节驱动输出单位 N·m，移动关节为 N；精确单位见各 CSV 的 position_unit、effort_unit。固定连接不生成虚假的角度或驱动输出。</p>')
    for i, entry in enumerate(index['modules']):
        html.append(f'<section id="joint-{i}" data-joint="{escape(entry["joint"], quote=True)}"><h2>{escape(entry["joint"])} · {escape(entry["joint_type"])}</h2><p>'+link(entry['state_csv'], '下载本关节完整状态 CSV')+'</p>')
        motion = [c for c in entry['charts'] if c['signal'] in ['position', 'velocity', 'acceleration', 'effort', 'power_W']]
        load = [c for c in entry['charts'] if c['signal'] not in ['position', 'velocity', 'acceleration', 'effort', 'power_W']]
        html.append('<h3>位置、速度、加速度、驱动输出与功率</h3><div class="charts">'+(''.join(map(image, motion)) or '<p>没有该关节的运动状态曲线。</p>')+'</div>')
        html.append('<details><summary>接口六维载荷曲线</summary><p>瞬时测点坐标系、测点原点、子体朝父体的符号约定；不是轴承内部载荷分配。</p><div class="charts">'+(''.join(map(image, load)) or '<p>未记录接口测点，不能补零替代。</p>')+'</div></details></section>')
    other = [c for c in charts if c['name'] not in {e['joint'] for e in index['modules']}]
    html.append('<section><h2>末端及其他已记录曲线</h2><details><summary>展开查看</summary><div class="charts">'+''.join(map(image, other))+'</div></details></section>')
    files = ['data/joints.csv', 'data/joint_loads.csv', 'data/end_effectors.csv', 'data/actuators.csv', 'data/contacts.csv', 'data/diagnostics.csv',
             'data/events.jsonl', 'reports/load_coverage.json', 'reports/load_statistics.csv', 'reports/load_spectrum.csv', 'reports/load_spectrum.json',
             'reports/effort_speed_spectrum.csv', 'reports/limit_exceedances.csv', 'reports/goal_validation.json', 'reports/result.json']
    html.append('<h2>附加数据与诊断</h2><p>'+''.join(link(file, file) for file in files if (run/file).is_file())+'</p>')
    for title, value in [('实际假设', task.get('assumptions', [])), ('记录与坐标约定', dictionary), ('真实任务结果', result), ('源数据 SHA-256', index['sources_sha256'])]:
        html.append('<details><summary>'+title+'</summary><pre>'+escape(json.dumps(value, ensure_ascii=False, indent=2))+'</pre></details>')
    html.append('</body></html>')
    (stage/'reports/report.html').write_text('\n'.join(html), encoding='utf-8')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    print(json.dumps(export(parser.parse_args().run), ensure_ascii=False))
