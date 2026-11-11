# 输入契约 1.0

坐标约定与 arm-statics-parametric、arm-workspace-parametric 的 input-contract.md 在 2026-11-10 核对。适配器自包含，无需运行它们；外部 Skill 只作为可选 FK 独立测试基准。

## 根字段

`schema_version:"1.0"` 和 `model` 必填；可选 dimensions、physics、installation、poses、drives、export、metadata。JSONC 支持 BOM、行/块注释，不支持尾逗号；拒绝重复键。数值必须有限。名称以 ASCII 字母起始，其余字母/数字/下划线；机器人/ROS 包名转换小写，关节名保持原样。其他字符需先提供命名映射。

## model

`{name,base_link?,joints:[...],tip:{xyz_m,rpy_rad}}`。每个关节 `{name,type,origin:{xyz_m,rpy_rad},axis?,limits?,components?}`；type 为 revolute/prismatic/fixed。joints 为顺序数组；或每行显式给 parent/child，生成器从 base_link（默认 base_link）排序，拒绝分支、断链、回路。1–64 行。顺序模式输出刚体默认 `<joint>_link`。world、tool_frame 为保留连杆名；tip、tool_fixed、world_fixed 为保留关节名。

origin 相对上游输出刚体；`T_child=T_parent T_origin Rot(axis,q)` 或 `T_parent T_origin Trans(axis*q)`。非单位轴归一化并报告，零轴拒绝。fixed 禁止 axis/limits。活动关节必须给有限 limits=[lower,upper]，旋转 rad、移动 m。tip 相对最后输出刚体，导出 tool_frame（URDF fixed frame、MJCF site、USD 子 Xform）。

几何零位为所有活动关节 q=0，可在行程外；初始与验收姿态必须限位内，不把初始 q 折入 origin。明确单位入口：变换 xyz_m/rpy_rad 可换为 xyz_mm/rpy_deg；limits 可换为 limits_deg（仅旋转）/limits_mm（仅移动），不能同时给两种单位。质量 kg、COM m、惯量 kg·m²，无全局猜测单位。

## dimensions（全部可选）

```json
{"base":{"diameter_m":0.12,"axial_width_m":0.04},
 "joints":{"J1":{"diameter_m":0.06,"axial_width_m":0.045}},
 "links":{"J1_link":{"width_m":0.045,"thickness_m":0.03,"max_envelope_m":[0.5,0.12,0.12]}},
 "flange":{"diameter_m":0.045,"axial_width_m":0.008},
 "rules":{"width_ratio":0.16,"min_width_m":0.025,"max_width_m":0.10,"module_ratio":1.35}}
```

尺寸 _m 可用 _mm 替代。max_envelope 是连杆局部 XYZ 总外廓上限；超出失败，不篡改运动学。已知壳体大于短跨度时报告安装冲突。缺尺寸时用整链比例、局部跨度和相邻安装空间共同推导，每项记录 design/estimated 和依据。输入不需要 CAD/Mesh/顶点/面。

外形默认使用 rounded-shell-v2，详见 [外形规则](appearance.md)。可选 `dimensions.appearance={"detail_level":0}` 关闭少量制造细节；默认1。这些细节不参与质量/碰撞。

相邻共点旋转轴支持模组角色和轴向退让：`dimensions.joints.J1={"role":"axial_twist","module_retreat_mm":130}`；角色默认 auto，退让默认推导。`dimensions.appearance.assembly_clearance_mm` 默认2。它们只影响外壳/碰撞部件布置，不改关节 origin；完整约束见 [关节模组规则](joint-layout.md)。

## physics

组件置于原有 model.joints[i].components 或 `physics.components:{link_or_joint_name:[components]}`。两者相加，同一刚体内重名拒绝；模组固定端/运动端保留真实所属刚体。

```json
{"name":"housing","mass_kg":1.2,"com_m":[0.05,0.01,0],"frame":"J1_link",
 "inertia_kg_m2":[[0.01,0,0],[0,0.02,0],[0,0,0.025]],
 "inertia_frame":"component","inertia_rpy_rad":[0,0.2,0]}
```

frame 默认所属输出刚体，可为所属关节名/连杆名；末端部件可用 tip/tool_frame。其他刚体坐标不能隐式使用。COM 随 frame 变换。惯量始终在 COM 处：inertia_frame=link 表示已按所属刚体轴表达；component 表示先按 frame 轴再施加 inertia_rpy_rad（默认0）。先旋转再平行轴合成，检查正质量、对称性、正主惯量和三角不等式。静力学组件缺惯量时保留质量/COM，并报告缺项。

组件 COM 可明确使用 com_mm，负载可用 com_in_tip_mm，惯量表达旋转可用 inertia_rpy_deg，均在入口转换且不得同时给两种单位。阻尼/摩擦也可写 physics.joints={关节名:{damping,friction}}；若 drives 同时给出不同值则报冲突。

附件：`physics.attachments:[{name,frame,mass_kg,com_m,...惯量字段}]`。负载：`physics.payload={mass_kg,com_in_tip_m,...惯量字段}`。质量0不添加；正质量须二选一：`included_in:{link,component}` 指向已有部件而不再计重；或 separate_mass_confirmed=true 确认独立加入。不能因数值相同推断重复。

概念估算须 export.mode=concept 且显式 physics.estimation：

- `{method:"uniform_density",density_kg_m3:800}`：无质量连杆用一个实体代理；已有质量/COM 只估缺失惯量。
- `{method:"known_mass_proxy"}`：只为已知质量组件估惯量，无质量连杆仍报缺项。

代理为主梁盒体或短跨度壳体圆柱，参数见 normalized_model.geometry.mass_proxy，不合并装饰/重叠体积。已知组件质量分布到该代理并平移至给定 COM，属于概念假设。严格模式缺完整物理时仍交付可视化 URDF；MJCF 待补齐，USD 只写可视化场景。

## installation / poses

`installation={base_in_world:{xyz_m,rpy_rad},gravity_world_m_s2:[0,0,-9.81]}`。默认单位安装和该重力，报告默认。URDF world_fixed、MJCF 根 body、USD 世界装配及固定约束只应用一次，不再写首关节。

`poses={initial:{J1:0.2,slide:0.03},acceptance:[{name:"check",q:{J1:-0.3,slide:0.1}}]}`。每项恰好包含所有活动关节，fixed 不得出现；SI 数值或旋转 `{deg:30}`、移动 `{mm:20}`。省略 initial 时用限位内最接近0的姿态，另外保存全0 geometric_zero。

## drives / export

drives 按活动关节名：`{effort,velocity,mode?,kp?,kv?,timeconst?,damping?,friction?,hardware_rating?,controller?,static_holding_load?}`。effort 是导出力/矩上限 N/Nm，velocity 是 m/s 或 rad/s；不会从静态保持载荷/电机额定值猜测。mode 默认 auto，解析为位置驱动；position/velocity 允许省略增益，由版本化物理规则计算；显式 kp/kv 保留。effort 为直接输入，passive 明确禁用驱动，fixed 不接受驱动配置。主动驱动需 effort/velocity 和完整质量属性。timeconst 是位置指令滤波秒数，默认 0.03，显式 0 禁用；USD 将它保存为运行时要求。详见 [驱动规则](drives.md)。hardware_rating/controller/static_holding_load 只作分类来源保留，不代替顶层配置。MuJoCo velocity 是 URDF 限值及速度控制目标范围，不是硬速度约束。

`export={mode:"strict"|"concept",targets:["urdf","ros1","mujoco","isaac"],limit_placeholders:{effort:10,velocity:1},runtime_versions:{ros1:"Noetic",mujoco:"3.3.7",isaac:"5.1.0"}}`。默认 strict/全部目标。strict 缺 effort/velocity 时 URDF/ROS1 失败；concept 只有显式占位配置才填并逐轴标记。MuJoCo 被动模型不需这些值，但必须有完整惯性。请求版本仅作目标记录，实际版本写验证报告。

可选 `export.drive_policy` 覆盖固定算法的已知常数，例如 `{"frequency_hz":1.5,"damping_ratio":1.2,"max_timestep_s":0.001}`；完整字段及约束见驱动规则。不能用策略自动放大 effort、改惯量或覆盖显式增益。旧输入的显式 passive 保持被动，若要启用驱动请改 auto 或 position。

## 现有输入适配

识别静力学 input_mode=standard/simple、工作空间 mode/model。提取 model/tip/components/附件/安装/姿态，不运行无关静力学/IK。静力学 drives 保存 source_statics_drives；wrenches 报告未导入，不伪装为模拟器驱动力。

simple 按原规则：首 origin=base（缺省单位并警告）；后续 origin.xyz 为前段 length×normalized(direction)，origin.rpy 为本段 rpy（缺省0）；末段向量为 tip。默认+X、中点 COM、整模组归当前输出刚体近似均保留。移动/fixed/真实模组拆分需标准输入。旧 gravity_m_s2 在基座系，转为世界重力一次；工作空间 base_in_world 映射 installation。静力学非零 payload 需追加重复计重声明。

源输入快照与归一化结果分开保存。源 Skill 的 normalized_input 不是此处请求格式。
