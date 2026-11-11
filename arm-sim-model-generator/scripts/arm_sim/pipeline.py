import hashlib, json, uuid, traceback
from pathlib import Path
from .model import read_input, normalize
from .geometry import build_geometry, mass_properties
from .math3d import serializable
from .exporters import urdf, ros1, mjcf, usd
from .validation import run_checks
from .drives import prepare_drives
from .collision import mating_pairs

def write_json(path,obj):
    path.parent.mkdir(parents=True,exist_ok=True); path.write_text(json.dumps(serializable(obj),ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')

def hashes(run):
    return {p.relative_to(run).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(run.rglob('*')) if p.is_file() and p.name not in ['manifest.json','validation.json','report.md']}

def report(n,manifest,checks,run):
    lines=['# 模型生成报告','',f"模型：{n['model']['name']}；等级：{n['model_grade']}。",'', '所有坐标为右手系，m/kg/rad；RPY=Rz Ry Rx。基座安装只应用于 world→base 一次。几何零位与初始关节值分开保存。','', '| 目标 | 生成/检查状态 | 运行时 |','|---|---|---|']
    for k,v in manifest['targets'].items(): lines.append(f"| {k} | {v['status']} | {v['runtime_validation']} |")
    if n.get('appearance_design'):
        lines+=['','## 外形生成','',f"规则：{n['appearance_design']['policy']}。整链比例、肩肘腕模组空间约束、多截面连续外壳、圆肩端盖和分区材质；使用 NumPy 原生生成，没有 CAD 依赖。",'',
                '外壳模板是概念设计推定；每个实体部件闭合，但同一连杆的壳体/端盖可局部相交，未执行 CAD 布尔融合。碰撞使用中央梁盒体与模组圆柱近似，圆滑过渡不等同于精确接触表面。实体重叠不重复计重。详细规则、尺寸与模板见 appearance_design.json 和下方逐连杆记录。','']
    if n.get('joint_layout'):
        lines+=['','## 关节模组布置','',f"规则：{n['joint_layout']['policy']}；状态：{n['joint_layout']['status']}。原始运动学帧保持不变，自转固定壳归上游连杆、输出套筒归下游连杆，模组退让不移动已有质量/质心。",'', '| 组合 | 模板 | 自转退让 m | 摆动侧置 m |','|---|---|---:|---:|']
        for pair in n['joint_layout']['pairs']: lines.append(f"| {' / '.join(pair['joints'])} | {pair['template']} | {pair['retreat_m']:.6g} | {pair['bend_side_offset_m']:.6g} |")
        lines+=['', '独立模组包络检查：'+checks.get('module_packing',{}).get('status','pending')+'。该检查不受仿真碰撞过滤影响，详见 validation.json/module_packing；不等同于全机构可制造性证明。','']
    lines+=['','## 缺项与假设','']+[f'- {x}' for x in n['missing_drive_limits']+n['missing_physics']+n['notes']]
    design=n.get('drive_design',{})
    if design:
        lines+=['','## 驱动参数','',f"策略：{design.get('policy','unresolved')}；状态：{design['status']}；建议物理步长：{design.get('timestep_s','pending')} s。",'',
                'SI 参数供 MJCF / USD 共用。角 kp 单位 Nm/rad、kv 单位 Nm·s/rad；移动轴为 N/m、N·s/m。USD 角增益另按每度换算。固定规则和采样证据见 drive_design.json；没有伪造质量、转子惯量、摩擦或重力补偿。','',
                '| 关节 | 模式 | kp | kv | 力/矩上限 | 来源 kp / kv |', '|---|---|---:|---:|---:|---|']
        for name,r in design.get('joints',{}).items():
            c=r['resolved_SI']
            def value(key): return format(c[key],'.8g') if key in c else '—'
            lines.append(f"| {name} | {c['mode']} | {value('kp')} | {value('kv')} | {value('effort')} | {r.get('kp_source','—')} / {r.get('kv_source','—')} |")
        servo=checks.get('mujoco',{}).get('runtime',{})
        servo=servo.get('servos',{}) if isinstance(servo,dict) else {}
        if servo:
            lines+=['',f"MuJoCo 位置驱动验收：{servo['status']}。以下仅为接触开启的有限姿态结果；静差单独报告，不以稳定代替精度。",'',
                    '| 工况 | 稳定 | 最大尾段峰峰值 | 最大尾段速度 | 最大静差 |', '|---|---|---:|---:|---:|']
            for c in servo.get('cases',[]):
                lines.append(f"| {c['case']} | {c['settled']} | {max(c['tail_peak_to_peak']):.6g} | {max(c['tail_max_speed']):.6g} | {max(map(abs,c['steady_error'])):.6g} |")
            lines+=['','数值分别使用 rad/m、rad/s或m/s；混合关节的逐轴数值和饱和率详见 validation.json。']
    lines+=['','## 装配配合体碰撞排除','',
            '两种仿真格式共用以下拓扑排除，避免相邻配合壳体互相顶开。共点壳体扩展只跨零平移 revolute/fixed 链，遇移动轴或非零偏移即停止；不自动排除其他穿插。真实 CAD 需另行复核。','']
    lines += [f"- {p['body1']} / {p['body2']}：{p['reason']}" for p in n.get('collision_exclusions',[])]
    lines+=['','## 连杆生成与质量来源','']
    for link,g in n['geometry'].items():
        lines += [f'### {link}', '',g['method'], '', '```json',json.dumps({'dimensions':g['dimensions'],'mass':n['mass_properties'].get(link),'mass_proxy':g['mass_proxy']},ensure_ascii=False,indent=2),'```','']
    lines+=['## 验证边界','','Mesh 检查闭合、面朝向、退化面。碰撞为低复杂度盒体/圆柱；干涉筛查为保守 AABB，候选不等于已证实穿透，无候选也不证明整个关节范围无碰撞。',f"当前报告 {len(checks.get('interference',{}).get('candidates',[]))} 个包围盒相交候选；详见 validation.json。",'', '独立 URDF 的相对资源根目录必须设为 URDF 所在目录（或加载前将相对文件名解析到此目录）；验证器复制整个目录后重新解析与加载 STL。安装在 world_fixed 中；加载器不得再施加同一安装变换。','', 'MuJoCo 使用导出目录中的 run.py 启动，自动加载 initial keyframe；自定义加载时调用 mj_resetDataKeyframe(model,data,0)，同时恢复 q、ctrl 和滤波状态。USD robot.usd 以初始姿态装配，引用 robot_zero.usd 保存原始零位框架；两层一起搬移。在 Isaac 中使用 runtime_config.json 的建议物理步长，并以 SI 核对初始关节位置与目标。USD 旋转限位和驱动目标使用 deg，角驱动增益由每 rad 转为每 deg。','', '硬件额定值、控制器参数和静力学保持载荷分别保存；静力学 drives 不当作动态 effort/velocity。','', '## ROS1 Noetic 使用','', '将 ros1/*_description 复制到 catkin_ws/src，安装 robot_state_publisher、joint_state_publisher、joint_state_publisher_gui、rviz；执行：','', '```bash','cd ~/catkin_ws','catkin_make','source devel/setup.bash',f"roslaunch {n['model']['name']}_description display.launch",f"roslaunch {n['model']['name']}_description display.launch gui:=false",'```','', '运行后检查 RobotModel 无资源/TF 错误，滑块逐关节方向、上下限和初始值正确。ROS1/RViz 和 Isaac 未运行时必须保留 pending。','', '## Isaac 导入与待执行验证','', '若安装了指定版本 Isaac Sim，优先用该版本官方 URDF Importer；Fix Base=True、Merge Fixed Joints=False、导入惯量=True、碰撞使用 URDF primitive、保留命名与 tool_frame。以 report/normalized_model.json 核对轴与限位。不要对已含 world_fixed 的描述再次应用安装位姿。','', '本次 robot.usd 由 OpenUSD 离线生成（如该目标成功）；这不代表 Isaac articulation/PhysX 步进已通过。使用同目录 runtime_config.json 及 Skill scripts/isaac_runtime.py 进行实机验收。','']
    (run/'report'/'report.md').write_text('\n'.join(lines),encoding='utf-8')

def generate(input_path,output,targets=None,runtime=False):
    output=Path(output).resolve(); output.mkdir(parents=True,exist_ok=True)
    run=output/'runs'/uuid.uuid4().hex; run.mkdir(parents=True); (run/'report').mkdir()
    attempt={'run_directory':str(run),'status':'running'}; write_json(output/'latest_attempt.json',attempt)
    manifest={'schema_version':'1.0','run_id':run.name,'status':'running','targets':{}}
    try:
        raw=read_input(Path(input_path)); write_json(run/'report'/'input_snapshot.json',raw)
        if targets: raw.setdefault('export',{})['targets']=targets
        n=normalize(raw); n['collision_exclusions']=mating_pairs(n); g=build_geometry(n,run/'meshes'); mass_properties(n,g)
        drive_error=None
        if set(n['export']['targets']) & {'mujoco','isaac'}:
            try: prepare_drives(n)
            except ValueError as e: drive_error=str(e); n['drive_design']={'status':'failed','error':drive_error}
        manifest['input_sha256']=hashlib.sha256(Path(input_path).read_bytes()).hexdigest()
        manifest['naming_map']={j['name']:{'joint':j['name'],'link':j['child']} for j in n['model']['joints']}
        for target in n['export']['targets']:
            st={'generated':False,'static_checks':'pending','runtime_validation':'pending','status':'pending'}; manifest['targets'][target]=st
            if target in ['mujoco','isaac'] and drive_error:
                st.update(status='failed',error=drive_error); continue
            if target in ['urdf','ros1'] and n['missing_drive_limits']:
                st.update(status='failed',error='URDF required effort/velocity missing: '+', '.join(n['missing_drive_limits'])); continue
            if target=='mujoco' and n['missing_physics']:
                st.update(status='pending',reason='missing physics; no invented simulator inertia'); continue
            try:
                path={'urdf':urdf,'ros1':ros1,'mujoco':mjcf,'isaac':usd}[target](n,g,run/({'urdf':'urdf_standalone'}.get(target,target)),run/'meshes')
                st.update(generated=True,status='generated',entry=str(path.relative_to(run)))
            except ImportError as e: st.update(status='pending',reason='optional runtime dependency unavailable: '+str(e))
            except Exception as e: st.update(status='failed',error=str(e))
        checks=run_checks(n,run,manifest['targets'],runtime)
        manifest['status']='failed' if any(v['status']=='failed' for v in manifest['targets'].values()) else 'completed'
        manifest['model_grade']=n['model_grade']; manifest['runtime_versions_requested']=n['export'].get('runtime_versions',{})
        write_json(run/'report'/'normalized_model.json',n); write_json(run/'report'/'validation.json',checks)
        if 'drive_design' in n: write_json(run/'report'/'drive_design.json',n['drive_design'])
        if 'appearance_design' in n: write_json(run/'report'/'appearance_design.json',n['appearance_design'])
        if 'joint_layout' in n: write_json(run/'report'/'joint_layout.json',n['joint_layout'])
        report(n,manifest,checks,run); manifest['files']=hashes(run); write_json(run/'report'/'manifest.json',manifest)
        attempt.update(status=manifest['status'],targets=manifest['targets']); write_json(output/'latest_attempt.json',attempt)
        return attempt
    except Exception as e:
        manifest.update(status='failed',error=str(e)); write_json(run/'report'/'manifest.json',manifest)
        attempt.update(status='failed',error=str(e)); write_json(output/'latest_attempt.json',attempt); return attempt

def validate(output,runtime=False):
    run=Path(output).resolve()
    output_root=None
    if (run/'latest_attempt.json').exists():
        output_root=run
        a=read_input(run/'latest_attempt.json')
        if a['status'] not in ['completed','failed'] or 'error' in a: return {'status':'failed','error':'latest generation failed/incomplete; old results not reused','latest_attempt':a}
        run=Path(a['run_directory'])
    manifest=read_input(run/'report'/'manifest.json'); n=read_input(run/'report'/'normalized_model.json')
    current=hashes(run); expected=manifest['files']
    if current!=expected: return {'status':'failed','error':'resource inventory or SHA256 mismatch','changed':[p for p in set(current)|set(expected) if current.get(p)!=expected.get(p)]}
    checks=run_checks(n,run,manifest['targets'],runtime); write_json(run/'report'/'validation.json',checks)
    manifest['status']='failed' if any(v['status']=='failed' for v in manifest['targets'].values()) else 'completed'
    write_json(run/'report'/'manifest.json',manifest); report(n,manifest,checks,run)
    result={'status':manifest['status'],'run_directory':str(run),'targets':manifest['targets']}
    if output_root: write_json(output_root/'latest_attempt.json',result)
    return result
