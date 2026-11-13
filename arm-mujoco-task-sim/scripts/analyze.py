"""Export individual joint and TCP curve images and performance evidence."""
import argparse
from array import array
import csv
from html import escape
import json
from pathlib import Path
import re

import numpy as np


JOINT_SIGNALS = ["position", "velocity", "acceleration", "effort", "power_W"]
TCP_SIGNALS = ([f"{prefix}_{axis}_{unit}"
                for prefix, unit in (("p", "m"), ("v", "m_s"), ("a", "m_s2"),
                                     ("omega", "rad_s"), ("alpha", "rad_s2")) for axis in "xyz"]
               + [f"quat_{axis}" for axis in "wxyz"])


def samples(path, identity, signals):
    groups = {}
    with path.open(encoding="utf-8-sig", newline="") as f:
        reader = csv.reader(f)
        headers = next(reader)
        columns = [headers.index(k) for k in [identity, "cycle", "time_s", *signals]]
        for row in reader:
            name = row[columns[0]]
            if name not in groups:
                groups[name] = [array("d") for _ in columns[1:]]
            for values, index in zip(groups[name], columns[1:]):
                values.append(float(row[index]))
    return groups


def segment_starts(run):
    path = run/"data/events.jsonl"
    if not path.exists():
        return []
    with path.open(encoding="utf-8") as f:
        return [row["time_s"] for row in map(json.loads, f)
                if row.get("event", {}).get("kind") in ("episode_start", "episode_reset")
                and row["time_s"] > 0]


def extrema(values, bins=850):
    edges = np.linspace(0, len(values), min(bins, len(values))+1, dtype=int)
    indices = {0, len(values)-1}
    for a, b in zip(edges[:-1], edges[1:]):
        indices.update((a+int(np.argmin(values[a:b])), a+int(np.argmax(values[a:b]))))
    return np.array(sorted(indices))


def violations(t, y, limits, cuts):
    if limits is None:
        return []
    lo, hi = limits
    result = []
    edges = np.r_[0, cuts, len(t)]
    for start, stop in zip(edges[:-1], edges[1:]):
        local = y[start:stop]
        for side, value in (("below", lo), ("above", hi)):
            if value is None:
                continue
            mask = local < value if side=="below" else local > value
            changes = np.flatnonzero(np.diff(np.r_[False, mask, False]))
            for a, b in zip(changes[::2], changes[1::2]):
                ix = start+a+int(np.argmax(np.abs(local[a:b]-value)))
                result.append((start+a, start+b-1, ix, side, value, int(b-a)))
    return sorted(result)


def chart(figures, stem, t, y, label, unit, limits, cuts, exceeded, png):
    # Use extrema downsampling for lines, but determine exceedances from every sample.
    x0, x1 = float(t[0]), max(float(t[-1]), float(t[0])+1e-6)
    y0, y1 = float(y.min()), float(y.max())
    if limits:
        bounds = [value for value in limits if value is not None]
        y0, y1 = min(y0, *bounds), max(y1, *bounds)
    pad = max((y1-y0)*.09, abs(y1)*.00001, 1e-6)
    y0 -= pad
    y1 += pad
    x = lambda z: 90+860*(z-x0)/(x1-x0)
    height = lambda z: 285-218*(z-y0)/(y1-y0)
    lines = []
    edges = np.r_[0, cuts, len(t)]
    for a, b in zip(edges[:-1], edges[1:]):
        if b > a:
            ix = a+extrema(y[a:b], max(40, int(850*(b-a)/len(t))))
            lines.append((t[ix], y[ix]))
    xml = ['<svg xmlns="http://www.w3.org/2000/svg" width="990" height="390" viewBox="0 0 990 390">',
           '<rect width="990" height="390" fill="white"/>',
           '<g font-family="sans-serif" fill="#21354d">',
           f'<text x="90" y="30" font-size="20">{escape(label)}</text>',
           f'<text x="90" y="53" font-size="13">{escape(unit)}</text>']
    for v in np.linspace(y0, y1, 5):
        yy = height(v)
        xml.append(f'<path d="M90 {yy:.2f}H950" stroke="#e0e7ef"/>'
                   f'<text x="82" y="{yy+4:.2f}" text-anchor="end" font-size="12">{v:.4g}</text>')
    for v in np.linspace(x0, x1, 6):
        xml.append(f'<text x="{x(v):.2f}" y="307" text-anchor="middle" font-size="12">{v:.4g}</text>')
    for a, b, ix, side, value, count in exceeded:
        xml.append(f'<rect x="{x(t[a]):.2f}" y="67" width="{max(1, x(t[b])-x(t[a])):.2f}" height="218" fill="#e64747" opacity=".14"/>')
    for j in cuts:
        xml.append(f'<path d="M{x(t[j]):.2f} 67V285" stroke="#8b9bad" stroke-dasharray="3 4"/>')
    if limits:
        for value in dict.fromkeys(v for v in limits if v is not None):
            yy = height(value)
            xml.append(f'<path d="M90 {yy:.2f}H950" stroke="#b47312" stroke-dasharray="6 4"/>'
                       f'<text x="944" y="{yy-4:.2f}" text-anchor="end" fill="#885405" font-size="11">{value:.5g}</text>')
    for tx, vy in lines:
        coords = " ".join(f"{x(tt):.2f},{height(yy):.2f}" for tt, yy in zip(tx, vy))
        if len(tx) > 1:
            xml.append(f'<polyline points="{coords}" fill="none" stroke="#155cb9" stroke-width="1.4"/>')
        else:
            xml.append(f'<circle cx="{x(tx[0]):.2f}" cy="{height(vy[0]):.2f}" r="2" fill="#155cb9"/>')
    for _, _, ix, _, _, _ in exceeded:
        xml.append(f'<circle cx="{x(t[ix]):.2f}" cy="{height(y[ix]):.2f}" r="2.8" fill="#bc2b2b"/>')
    note = ("Dashed amber: specified limits. Red: exceedance samples and interval peaks."
            if limits else "No limits supplied for this signal.")
    xml.extend([f'<text x="520" y="335" text-anchor="middle" font-size="13">Simulation time (s)</text>',
                f'<text x="90" y="365" font-size="12">{note}</text>', '</g></svg>'])
    (figures/(stem+".svg")).write_text("\n".join(xml), encoding="utf-8")
    if png:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 3.9), layout="constrained")
        try:
            for a, b, _, _, _, _ in exceeded:
                ax.axvspan(t[a], t[b], color="#e64747", alpha=.14, linewidth=0)
            for j in cuts:
                ax.axvline(t[j], color="#8b9bad", linestyle=":", linewidth=.8)
            for tx, vy in lines:
                ax.plot(tx, vy, color="#155cb9", linewidth=1, marker="." if len(tx)==1 else None)
            if limits:
                for value in dict.fromkeys(v for v in limits if v is not None):
                    ax.axhline(value, color="#b47312", linestyle="--", linewidth=.9)
                    ax.annotate(f"{value:.5g}", (x1, value), xytext=(-5, 3), textcoords="offset points",
                                ha="right", color="#885405", fontsize=8)
            if exceeded:
                indices = [part[2] for part in exceeded]
                ax.scatter(t[indices], y[indices], c="#bc2b2b", s=12, zorder=4)
            ax.set(xlim=(x0,x1), ylim=(y0,y1), xlabel="Simulation time (s)", ylabel=unit)
            ax.set_title(label, loc="left")
            ax.grid(alpha=.20)
            ax.text(.01, .01, note, transform=ax.transAxes, fontsize=7, color="#47556b")
            fig.savefig(figures/(stem+".png"), dpi=145)
        finally:
            plt.close(fig)


def analyze(run, png=None):
    run = Path(run)
    figures, reports = run/"figures", run/"reports"
    figures.mkdir(exist_ok=True)
    reports.mkdir(exist_ok=True)
    if png is None:
        import importlib.util
        png = importlib.util.find_spec("matplotlib") is not None
    if png:
        import matplotlib
        matplotlib.use("Agg")
    limits_file = run/"input/joint_limits_resolved.json"
    if not limits_file.is_file():
        limits_file = run/"input/assets/joint_limits.json"
    if not limits_file.is_file():
        limits_file = reports/"joint_limits.json"
    limits = json.loads(limits_file.read_text(encoding="utf-8"))["joints"] if limits_file.is_file() else {}
    meta = json.loads((run/"data/metadata.json").read_text(encoding="utf-8"))
    kinds = {j["name"]: j["type"] for j in meta["joints"]}
    starts = segment_starts(run)
    stats, links, alerts = [], [], []
    for table, identity, signals in (("joints","joint",JOINT_SIGNALS),
                                      ("end_effectors","site",TCP_SIGNALS)):
        for name, values in sorted(samples(run/"data"/(table+".csv"), identity, signals).items()):
            cycle, t, *series = map(np.asarray, values)
            cuts = np.unique(np.searchsorted(t, starts))
            cuts = cuts[(cuts > 0) & (cuts < len(t))]
            hinge = kinds.get(name)=="hinge"
            for signal, y in zip(signals, series):
                unit = ({"position":"rad" if hinge else "m","velocity":"rad/s" if hinge else "m/s",
                         "acceleration":"rad/s^2" if hinge else "m/s^2","effort":"N m" if hinge else "N",
                         "power_W":"W"}[signal] if table=="joints" else
                        {"p":"m","v":"m/s","a":"m/s^2","omega":"rad/s","alpha":"rad/s^2","quat":"unitless"}
                        [signal.split("_")[0]])
                label = f"{name} | {signal}"
                stem = f"{len(links):03d}-"+re.sub(r"[^A-Za-z0-9_-]", "_", name+"_"+signal)
                spec = limits.get(name, {}).get(signal) if table=="joints" else None
                bounds = (spec.get("lower"), spec.get("upper")) if spec else None
                exceeded = violations(t, y, bounds, cuts)
                chart(figures, stem, t, y, label, unit, bounds, cuts, exceeded, png)
                links.append((label, stem, len(exceeded)))
                for a,b,ix,side,value,count in exceeded:
                    alerts.append((name,signal,unit,int(cycle[ix]),side,value,t[a],t[b],t[ix],y[ix],count,spec["source"]))
                for selected in ["all", *np.unique(cycle).astype(int)]:
                    mask = np.ones(len(t), dtype=bool) if selected=="all" else cycle==selected
                    yy, tt = y[mask], t[mask]
                    adjacent = mask[:-1] & mask[1:]
                    adjacent[np.maximum(0,cuts-1)] = False
                    dt = np.diff(t)[adjacent]
                    duration = float(dt.sum())
                    energy = float(np.sum((y[:-1][adjacent]**2+y[1:][adjacent]**2)*dt/2))
                    rms = float(np.sqrt(energy/duration)) if duration>0 else float(np.sqrt(np.mean(yy**2)))
                    stats.append((table,name,selected,signal,unit,float(min(yy)),float(max(yy)),float(max(abs(yy))),rms,duration))
    for filename, columns, rows in [
        ("statistics.csv", "table name cycle signal unit min max peak_abs time_weighted_rms duration_s", stats),
        ("limit_exceedances.csv", "joint signal unit cycle side limit start_s end_s peak_time_s peak_value samples source", alerts),
    ]:
        with (reports/filename).open("w", encoding="utf-8-sig", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(columns.split())
            writer.writerows(rows)
    c = json.loads((run/"input/task_contract.json").read_text(encoding="utf-8"))
    result = json.loads((reports/"result.json").read_text(encoding="utf-8"))
    html = ('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>仿真曲线与数据</title>'
            '<style>body{font:16px/1.6 system-ui;max-width:1080px;margin:36px auto;padding:0 18px}'
            'img{display:block;width:100%;height:auto}details{margin:.55rem 0}summary{cursor:pointer}'
            '.alert{color:#a32222;font-weight:bold}a{color:#1457a3}</style><h1>仿真曲线与数据</h1>'
            f'<p>{escape(c["task_description"])} · {escape(result["status"])}</p>'
            '<p><a href="result.json">逐轮结果</a> · <a href="statistics.csv">峰值与 RMS</a> · '
            '<a href="limit_exceedances.csv">越限明细</a> · <a href="../data/joints.csv">关节 CSV</a> · '
            '<a href="../data/end_effectors.csv">末端 CSV</a></p>'
            '<p>橙色虚线表示已知限值，红色表示越限区间和峰值。灰色虚线为复位边界。</p>')
    for title, stem, count in sorted(links, key=lambda item: not item[2]):
        alert = f' <span class="alert">越限 {count} 段</span>' if count else ""
        file = stem+(".png" if png else ".svg")
        download = f'<a href="../figures/{stem}.svg">SVG</a>'
        if png:
            download += f' · <a href="../figures/{file}">PNG</a>'
        html += f'<details><summary>{escape(title)}{alert}</summary><img loading="lazy" src="../figures/{file}" alt="{escape(title)}">{download}</details>'
    (reports/"report.html").write_text(html+"</html>", encoding="utf-8")
    return stats


if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path)
    parser.add_argument("--png", action="store_true", default=None)
    args = parser.parse_args()
    analyze(args.run, args.png)
