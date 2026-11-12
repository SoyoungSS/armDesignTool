# 机械臂网页分析示例

| 模型 | 公共模型 | 静力学请求 | 可达空间请求 | 轨迹请求 |
|---|---|---|---|---|
| 选型表七轴单臂 | [模型](../examples/models/seven-axis.json) | [平举与搜索](../arm-statics-parametric/assets/examples/worksheet-minimal.json) | [FK 点云](../arm-workspace-parametric/assets/examples/worksheet-workspace.jsonc) | [动态载荷演示](../arm-kinematics-dynamics/assets/examples/worksheet-dynamics.json) |
| 双臂设计方案 CAD 右臂 | [模型](../examples/models/design-right-arm.json) | [平举与搜索](../examples/cases/design-right-arm/statics.json) | [FK 点云](../examples/cases/design-right-arm/workspace.json) | [往返与动态载荷](../examples/cases/design-right-arm/motion.json) |
| Franka FR3 法兰原始几何 | [模型](../examples/models/franka-fr3.json) | 源文件缺少质量和质心 | [原始 FK 点云请求](../arm-workspace-parametric/assets/examples/franka-fr3.workspace.json) | [轨迹与奇异性](../arm-kinematics-dynamics/assets/examples/franka-fr3-motion.json) |
| 七轴仿真输入 | [模型](../examples/models/seven-axis-sim.json) | [初始与验收姿态](../examples/cases/seven-axis-sim/statics.json) | [5,000 点 FK](../examples/cases/seven-axis-sim/workspace.json) | [验收姿态轨迹及动态载荷](../examples/cases/seven-axis-sim/motion.json) |
| Franka FR3 法兰质量预设 | [模型](../examples/models/franka-fr3-flange-preset.json) | [初始与验收姿态](../examples/cases/franka-fr3-flange-preset/statics.json) | [30,000 点 FK](../examples/cases/franka-fr3-flange-preset/workspace.json) | [验收姿态轨迹及动态载荷](../examples/cases/franka-fr3-flange-preset/motion.json) |

## 网页使用

1. 打开「规格与模型」，在「示例模型」下拉框选取选型表、CAD 右臂、七轴仿真输入或 FR3 法兰质量预设，也可导入自定义公共模型 JSON。
2. 选择后会自动应用示例。修改规格后，点击「应用到三个 Skill」更新分析输入，或点击「运行三个 Skill 并生成规格表」直接联算。
3. 在「静力学分析」「可达空间」「轨迹与奇异性」中可按当前公共模型调整各自工况。上表的完整请求保留供独立 CLI 复算。网页选择示例时以公共模型生成默认分析工况。

## 选型表与 CAD 右臂的参数来源

- 选型表按仓库已录入的 `worksheet-minimal.json`，七段长度为 35、50、250、50、163、47、40 mm，整臂运动质量 3.612 kg，示例附加负载 3 kg。网页公共模型采用 YZ 安装面（世界 +X 法向）和 J1 +Y 正轴，保留原零位下垂几何和物理行程。表中独立 CLI 请求保留原选型表坐标约定。动态载荷示例使用记录在请求内的球体惯量估算。
- CAD 右臂来自《纯旋转关节具身双臂平台设计方案文档》的配套 `ArmReference/dual_arm/urdf/dual_arm.urdf`。保留右臂 170 mm 安装偏移、七个主动关节、固定法兰及各运动连杆的 CAD 质量、质心和惯量。CAD 运动质量合计 2.61908817 kg。设计文档的 3 kg 额定负载在此作为法兰原点的演示附加负载，惯量按半径 50 mm 的均匀实心球估算。轨迹限制为演示设置。工作空间请求采用 5,000 个 FK 样本。

## 七轴仿真输入与 FR3 法兰质量预设

- 七轴仿真示例从 `ArmReference/seven_axis_sim_input.json` 转为公共模型，保留 7 个运动刚体共 16.062132 kg 的参考质量、质心和惯量、安装重力及零附加负载。固定底座质量不参与关节载荷，107 mm 法兰变换只应用一次。静力学采用源文件初始姿态、两个验收姿态和限位内搜索。动态轨迹使用这些姿态往返，速度沿用源文件固定导出限值，加速度上限按 2 rad/s² 演示。驱动 effort 不作为静力学连续额定能力。
- 七轴仿真输入与 FR3 原始几何的七轴原点、RPY、轴向、限位及法兰变换逐项一致。FR3 质量预设借用其七段运动刚体参考参数，另在 `fr3_joint8` 的固定法兰上添加 0.10 kg。源文件未提供法兰实体几何，以 `fr3_link8` 原点 [0, 0, 0] m 近似几何中心，以半径 25 mm 的均匀实心球估算惯量。若实物法兰已经计入 J7，应去除重复计重。两套示例均为空载，参考刚体参数与法兰预设未经实物标定。

## FR3 原始几何来源与输入

源文件为用户提供的 franka_fr3.workspace.json，复制时保留原始字节。文件指纹在工作空间 Skill 的 references/franka-fr3-source.json。内置请求自包含，不依赖源文件 metadata 中提及的 URDF 路径。

FR3 有七个主动旋转关节和 fr3_joint8 固定关节；末端是 fr3_link8 法兰，不含夹爪或额外 TCP。保留各关节 origin RPY、轴、限位、固定末端和 30,000 个 FK 样本的设置。文件未提供质量、质心和惯量，因此原始几何请求只能直接用于工作空间及运动学分析。

## 历史数据

单杆、两杆、移动关节、旧七轴概念臂及 CAD/Dexbot 左右臂已移至相应 scripts/tests/fixtures；公共单杆/两杆模型和保持工况移至 shared/tests/fixtures。旧浏览器输入示例移至网站 tests/fixtures。它们仅用于数值回归，不列入网页或公开模型目录。

旧浏览器内核构型实现保留为历史兼容和回归依据，主页面不再提供相应预设，也不将 FR3 几何改写成该旧内核的近似模型。

## 本轮验证

网站 21 项、静力学 64 项、工作空间 22 项、公共契约/分发 10 项通过。包括新增 FR3 文件指纹、独立齐次变换对照、30,000 个姿态的关节限位、位置 IK 重放，以及模式转换保留几何。

实际浏览器已完成 FR3 加载、真实 CLI 计算和三维预览；工作空间结果共 30,000 点，异模型应用按钮保持禁用。体素模式的转换已验证，本轮未运行 FR3 全量体素搜索。
