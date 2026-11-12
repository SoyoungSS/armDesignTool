"""CSV statistics and SVG plots without a plotting dependency."""
import argparse
import csv
from html import escape
import json
from pathlib import Path
import re
import numpy as np


def plot(path,t,y,label):
    lo,hi=float(min(y)),float(max(y)); pad=max((hi-lo)*.08,1e-8);lo-=pad;hi+=pad
    xmin,xmax=float(min(t)),max(float(max(t)),float(min(t))+1e-9)
    selected={0,len(y)-1}
    for a,b in zip(np.linspace(0,len(y),801,dtype=int)[:-1],np.linspace(0,len(y),801,dtype=int)[1:]):
        if b>a: selected.update([a+int(np.argmin(y[a:b])),a+int(np.argmax(y[a:b]))])
    coords=' '.join(f'{80+840*(t[i]-xmin)/(xmax-xmin):.2f},{270-210*(y[i]-lo)/(hi-lo):.2f}' for i in sorted(selected))
    xml=[f'<svg xmlns="http://www.w3.org/2000/svg" width="960" height="330"><rect width="960" height="330" fill="white"/><g font-family="sans-serif" fill="#20324d"><text x="80" y="28" font-size="18">{escape(label)}</text>']
    for f in np.linspace(0,1,5):
        yy=270-210*f;xx=80+840*f
        xml.append(f'<path d="M80 {yy}H920" stroke="#dde3ed"/><text x="73" y="{yy+4}" text-anchor="end" font-size="11">{lo+(hi-lo)*f:.4g}</text><text x="{xx}" y="295" text-anchor="middle" font-size="11">{xmin+(xmax-xmin)*f:.4g}</text>')
    xml.append(f'<polyline points="{coords}" fill="none" stroke="#1956c4" stroke-width="1.4"/><text x="470" y="320">Time (s)</text></g></svg>')
    path.write_text('\n'.join(xml),encoding='utf-8')


def analyze(run):
    run=Path(run); (run/'figures').mkdir(exist_ok=True);(run/'reports').mkdir(exist_ok=True)
    stats=[]; links=[]
    for table,identity,signals in [('joints','joint',['position','velocity','acceleration','effort','power_W']),('end_effectors','site',['p_x_m','p_y_m','p_z_m','v_x_m_s','v_y_m_s','v_z_m_s','a_x_m_s2','a_y_m_s2','a_z_m_s2','omega_x_rad_s','omega_y_rad_s','omega_z_rad_s','alpha_x_rad_s2','alpha_y_rad_s2','alpha_z_rad_s2'])]:
        with (run/'data'/(table+'.csv')).open(encoding='utf-8-sig',newline='') as f: rows=list(csv.DictReader(f))
        for name in sorted({r[identity] for r in rows}):
            group=[r for r in rows if r[identity]==name];t=np.array([float(r['time_s']) for r in group])
            for signal in signals:
                y=np.array([float(r[signal]) for r in group]); unit=''
                if table=='joints':
                    angle=group[0]['type']=='hinge'
                    unit={'position':'rad' if angle else 'm','velocity':'rad/s' if angle else 'm/s','acceleration':'rad/s2' if angle else 'm/s2','effort':'N m' if angle else 'N','power_W':'W'}[signal]
                label=f'{name} | {signal} {unit}'
                filename=f'{len(links):03d}-'+re.sub(r'[^A-Za-z0-9_-]','_',name+'_'+signal)+'.svg'
                plot(run/'figures'/filename,t,y,label);links.append((label,filename))
                for cycle in ['all']+sorted({r['cycle'] for r in group}):
                    mask=np.ones(len(y),dtype=bool) if cycle=='all' else np.array([r['cycle']==cycle for r in group])
                    yy,tt=y[mask],t[mask];duration=float(tt[-1]-tt[0])
                    integral=float(np.trapezoid(yy**2,tt)) if len(tt)>1 else float(yy[0]**2)
                    rms=float(np.sqrt(integral/duration)) if duration>0 else float(abs(yy[0]))
                    stats.append([table,name,cycle,signal,unit,float(min(yy)),float(max(yy)),float(max(abs(yy))),rms,duration])
    with (run/'reports/statistics.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.writer(f);w.writerow(['table','name','cycle','signal','unit','min','max','peak_abs','time_weighted_rms','duration_s']);w.writerows(stats)
    c=json.loads((run/'input/task_contract.json').read_text(encoding='utf-8'))
    result=json.loads((run/'reports/result.json').read_text(encoding='utf-8'))
    html='<html lang="zh-CN"><meta charset="utf-8"><title>任务仿真结果</title><style>body{font:16px/1.8 system-ui;max-width:1000px;margin:40px auto}img{max-width:100%}pre{white-space:pre-wrap}</style><h1>任务仿真结果</h1>'
    html+='<p>任务：'+escape(c['task_description'])+'</p><p>模式：'+escape(c['mode'])+'；策略：'+escape(str(c['strategy']))+'；数据来源：'+escape(str(c['state_source']))+'</p>'
    html+='<p>输出目录：'+escape(str(run.resolve()))+'</p><h2>执行结果</h2><pre>'+escape(json.dumps(result,ensure_ascii=False,indent=2))+'</pre>'
    html+='<h2>实现和约定</h2><p>场景、资产清单、策略和任务适配器完整快照在 input/；用户确认和轮次定义在 input/task_contract.json。瞬时数据在同一仿真时刻求解后、积分前采样，单位、坐标和加速度方法见 data/metadata.json。</p><p>曲线和 CSV 是仿真载荷。寿命、电机轴或轴承载荷需要对应的材料、应力、传动与几何模型；本工具未凭扭矩直接计算寿命。agent 还须补充任务成功证据与模型限制到 reports/implementation.md。</p>'
    html+='<h2>曲线</h2>'+''.join(f'<p><a href="../figures/{fn}">{escape(label)}</a></p>' for label,fn in links)+'</html>'
    (run/'reports/report.html').write_text(html,encoding='utf-8')
    return stats


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run');args=p.parse_args();analyze(args.run)
