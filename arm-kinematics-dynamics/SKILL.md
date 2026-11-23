---
name: arm-kinematics-dynamics
description: 为参数化机械臂生成可复现的关节空间随机轨迹，并分析关节轨迹、TCP运动状态和任务相关奇异性。支持五次多项式轨迹、外部时间轨迹和刚体逆动力学载荷计算，无需URDF或网页。
---

# 轨迹状态与奇异性分析

接受 `arm-motion-request` 1.0。启用 `dynamics.enabled=true` 后计算关节输出端动态载荷。输入字段、单位、坐标与限制见 [请求契约](references/input-contract.md)。公开示例仅有选型表七轴单臂和 Franka FR3，位于 assets/examples。运动限制是明确的演示假设，不是 FR3 厂家能力。

## 双默认示例

在本工具目录使用 Node.js 20 或更高版本运行。选型表采用同一套七轴单臂的侧装模型，FR3 采用 Franka ROS 标称模型。两份请求均可单独运行。[选型表七轴单臂](assets/examples/defaults/worksheet.json) · [Franka FR3](assets/examples/defaults/fr3.json)

```bash
node scripts/run.mjs --input assets/examples/defaults/worksheet.json --output outputs/worksheet/result.json --report-dir outputs/worksheet
node scripts/run.mjs --input assets/examples/defaults/fr3.json --output outputs/fr3/result.json --report-dir outputs/fr3
```

两例均生成 5 s 的往返轨迹，包含 1 s 保持段，标称采样间隔 0.02 s。结果包括法兰运动、奇异性、关节动态力矩、峰值与 RMS。选型表按半径 0.04 m 的均匀实心球近似部件惯量，3 kg 负载的球半径为 0.05 m。FR3 采用参考文件给出的逐轴速度限值和演示用 2 rad/s² 加速度上限。

FR3 保留七个活动轴的标称质量、局部质心和惯性张量、基座安装及 0.107 m 法兰偏移，使用无手爪、空载配置。上游公用惯性表采用 Gaz 等人针对 Panda 的辨识结果。[Franka 固定版本惯性来源](https://github.com/frankarobotics/franka_ros/blob/ddd2fffd9de44b02ad15b4bbb2bfa2cec4d60d98/franka_description/robots/common/inertial.yaml) · [上游许可证](references/franka-fr3-LICENSE.txt)

`analyze(input)` 位于 scripts/lib/analyze.mjs。独立分发用仓库 shared/scripts/package-skill.mjs，保持 Skill 与 shared 同级，不依赖网页。JSON 请求内嵌参数模型。仅做运动学时可省略质量与惯量。

同一 CLI 接受 `arm-random-motion-request` 1.0。使用 `--input` 和 `--output` 获取一批可复现的轨迹，每条的 `request` 可提交给原分析入口。程序入口为 `scripts/lib/random-trajectory.mjs` 的 `generateRandomTrajectories(input)`。字段见 [请求契约](references/input-contract.md)。

## 结果解释

- waypoints：按关节 ID 的模型顺序给出数组；每个关键点停车，q/dq/ddq 用解析五次多项式生成。strict 时长不满足限制时仍输出诊断但 trajectory_feasible=false；stretch 延长时长并重新计算。位置关键点超限直接拒绝。
- samples：joint_ids 显式映射外部数组，时刻严格递增；不自动推导缺失速度/加速度。derivatives=estimate 才使用三点非均匀时间局部多项式估计，噪声风险需保留。
- 奇异性按 position 或 pose 的雅可比分析。平移行按 characteristic_length_m 缩放，移动关节列按同一长度缩放。数值秩、最小奇异值、条件数和阈值随结果保存；阈值不是普适安全标准。
- task_underactuated 是任务维度大于关节数，与姿态秩下降区分。几何奇异风险不等于关节轨迹必然不可执行；没有目标末端运动时不判断任务方向跟踪。
- 生成轨迹按解析段极值检查速度/加速度及可选跃度，近限位/奇异性仍仅为采样观察。导入轨迹约束只检查样本，不能认证点间安全。
- 若缺导数或限制，约束状态为 incomplete 或 samples_passed，不擅自判全程通过。

输出主 JSON、trajectory.csv、report.md、request.json，保留输入/模型哈希和假设。CLI执行成功不等于轨迹可行。失败返回非零退出码和结构化错误，主结果在成功完成后原子替换；失败时不要读取旧输出冒充新结果。

FK、雅可比复用 shared/kinematics/serial.mjs；不复制现有工作空间 IK。首版只处理关节路径，不调用笛卡尔路径 IK，动力学支持刚体逆动力学，热、碰撞、柔性和自动绕奇异功能留待后续。


## 动态载荷

在请求中设置 `dynamics.enabled=true`。所有运动部件需要显式质量、质心和绕质心惯性张量，惯量表达在所属关节运动后坐标系中。模型需要显式重力。正质量末端负载还需 `dynamics.payload_inertia_kg_m2`，采用物理 tip 坐标轴、绕负载质心。质量为零的部件可省略惯量，`components: []` 声明无质量刚体。缺失参数返回输入错误。

`assets/examples/worksheet-dynamics.json` 是选型表动力学演示。部件惯量按半径 0.04 m 均匀实心球估算，3 kg 负载按半径 0.05 m 估算。假设记录在 provenance 中，使用真实设计数据时替换这些惯量。当前双默认示例使用选型表侧装版本及 FR3 标称刚体模型。

`scripts/lib/dynamics.mjs` 采用基座坐标表达的递归 Newton–Euler 法，支持旋转、移动、固定关节及多个刚体部件，输出动态力矩/推力和接口载荷。`dynamic-loads.csv` 随 `--report-dir` 导出。功率为关节输出端机械功率。RMS 根据非均匀采样时间，对力矩/推力平方进行梯形积分后计算。峰值与 RMS 需要通过减小采样间隔检查收敛。

当前计算包含刚体惯性、重力和可选的恒定世界系末端外力/力矩。传动损失、摩擦、电机转子惯量、柔性和接触求解需后续补充。载荷时序供后续工况占比、循环次数和载荷谱统计使用。
