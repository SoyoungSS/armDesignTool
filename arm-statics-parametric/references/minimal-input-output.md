# 精简输入与输出参考

## 文件选择

- `assets/examples/worksheet-minimal.jsonc`：供人填写，带中文注释。
- `assets/examples/worksheet-minimal.json`：完全相同参数，无注释，供只接受标准 JSON 的系统使用。
- `analyze(input)` 接收已解析对象；文本先经过 `parseRequestText(text)`。JSONC 仅增加注释，不增加第二套计算逻辑。
- 支持 `//`、`/* */` 注释及文件开头 UTF-8 BOM。不支持尾逗号、单引号、表达式及函数。输出文件始终为严格 JSON。

## 必须提供的信息

| 字段 | 内容及单位 |
|---|---|
| schema_version / input_mode | 固定为 `"1.0"` / `"simple"` |
| simple.name | 模型名称 |
| simple.segments | 按基座到末端排列，每个旋转关节一行 |
| name / axis | 唯一关节名、当前局部坐标系转轴，按右手定则 |
| length_m | 当前轴心到下一轴心的段长；末段到末端参考点，m |
| limits_rad | 运动上下限，rad，不能填角度数值 |
| link_mass_kg | 此段连杆质量，kg；明确无质量才填 0 |
| direction | 连杆在当前输出坐标系中的方向。本例必须填 `[0,0,-1]`；省略会默认 +X，模型将不同 |
| joint_mass_kg | 当前模组质量。本例必须保留以构成 3.612 kg 自重；若质量已归入其他刚体，不重复填写 |
| gravity_m_s2 | 重力在基座参考系的向量，m/s² |
| payload.mass_kg / com_in_tip_m | 负载质量 kg 及质心相对末端参考点的坐标 m；无负载明确填 0 |
| poses 或 search | 至少一个非空。poses 中每个 q 必须列全主动关节，rad；search 使用限位搜索 |

`direction`、`joint_mass_kg` 虽是语法可选字段，但对本例物理模型不可省略。不同机械臂需替换轴、方向、长度、限位及质量，不能只换名称。

## 精简时采用的假设

省略 base 表示基座原点及姿态均为零。纯重力下，单纯平移安装原点不改变关节扭矩，但会改变末端绝对位置。若安装姿态不同，必须补 base.rpy_rad 并核对重力坐标系。

省略 link_com_m：连杆质心取几何中点；省略 joint_com_m：整模组质量集中在当前关节输出原点。这是集中质量近似，不是实物 CAD 质心。若有真实质心，应补这两个坐标；若模组固定端和运动端需拆分归属，使用 standard 输入。

本例为选型表连杆 0.710 kg + 模组 2.902 kg = 单臂 3.612 kg，另加 3 kg 负载；保留 J1–J2 间 35 mm 段。所有零位段朝 -Z，不是所有机械臂的通用默认构型。

## 可选信息

- search：删除后仅计算 poses。示例 samples=20000、seed=42；省略高级控制时，elite_count=3、refine_sweeps=8。对比历史搜索结果时也必须对齐这些配置。
- poses：若只做范围搜索可删除；若只做典型姿态校核则保留并删除 search。
- design_factor：示例 1.2，静态输出需求为已检查最大绝对载荷乘此值；不填写默认 1，并产生提示。1.2 只是本例假设。
- drives：进行现有驱动扭矩裕量判定时才需要。未提供时仍能计算静态需求，但 capacity/margin/motor_static_torque_Nm 为 null，status=not_sized。
- base、真实 COM、附件、偏心负载和外载：按实物实际情况补充。输入格式详见 input-contract.md。

## 运行

```bash
node scripts/run.mjs --input assets/examples/worksheet-minimal.jsonc --output /absolute/output/statics_result.json --report-dir /absolute/output
```

网页可直接加载 `.jsonc`；HTTP POST `/api/statics` 支持 application/json 和 application/jsonc，响应仍为 `{ok:true,result:...}`，CLI 主文件直接为 result 对象。

## 输出文件格式

| 文件 | 格式与用途 |
|---|---|
| statics_result.json | 完整严格 JSON，供网页及其他 agent 消费；含输入哈希、模型快照、逐姿态带符号保持载荷、分项贡献、范围搜索临界姿态、逐轴汇总、假设与限制 |
| joint_summary.csv | 一关节一行，便于 Excel 查看；列是 summary 的固定子集，缺失数值导出为空单元格 |
| report.md | 面向人的分析报告：逐轴最大载荷、静态需求、能力状态及假设 |
| standard_request.json | 展开的标准请求，可直接作为下一次计算输入 |
| normalized_model.json | 展开后的内部几何和质量模型，仅模型片段，不能单独直接计算 |

完整 JSON 主要字段：

| 字段 | 含义 |
|---|---|
| engine_version / schema_version | 计算内核和数据格式版本 |
| input_sha256 / normalized_sha256 | 已解析请求及归一化模型哈希；只修改注释不改变计算结果或哈希 |
| joint_names | 数组型力矩结果的顺序 |
| poses[].holding_load | 各关节指定姿态带符号保持载荷，旋转关节 Nm、移动关节 N |
| poses[].components / contributions | 自重、负载、附件、外载分项及详细贡献 |
| search.extrema | 各关节搜索最大绝对载荷及对应 q、符号、搜索阶段 |
| summary[].max_abs_checked | 已检查典型姿态与搜索中最大绝对载荷 |
| summary[].required_output | 输出侧静态设计需求 = max_abs_checked × design_factor |
| summary[].critical_pose | 最大值的来源及完整关节角 q |
| summary[].continuous_available / margin / status | 驱动能力、裕量（available / required − 1）及判定；缺驱动时为 null/null/not_sized |
| summary[].motor_static_torque_Nm | 有传动比与效率时折算的电机静态扭矩；否则 null |
| warnings / scope | 输入估算、搜索未证明全局最大、未校核项目等 |

最大重力扭矩是当前检查得到的最大值，不是已证明的全局极值。保持扭矩符号不能直接用于比较选型容量，应使用绝对值。未输入容量时不能宣称驱动通过。无碰撞、热、动态或结构校核。
