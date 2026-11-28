# 通用工况与全关节载荷谱

## 任务与载荷输出相互独立

本技能把机械臂在给定工况下的物理执行转换成设计载荷输入。抓放、擦拭、推压、插装、工具操作、巡检、自由运动、带载保持或混合工序均使用同一运行器、记录器与谱导出器。分拣仅为已有端到端示例，不规定机器人轴数、工具、物体数量、颜色或任务阶段。

先定义本次 duty：初始状态、运动/力目标、环境与接触、持续时间、停留、重复次数、退出条件、失败动作及观测来源。非周期任务使用一个 episode，不制造重复周期；概念模式可根据常规试算委托默认 `cycles=1` 并记录依据；严格模式的 `cycles=1` 仍须来自明确要求。连续工况可在同一 cycle 用不同 phase 表达。

载荷数据有效性与任务成功分别报告。失败、超时、碰撞前的实际记录可以导出，明确截断位置与原因；不能把部分数据称为完整生产节拍，也不能因任务未成功而删除冲击和异常。没有实际仿真就不产生计算载荷。

## 两类载荷

| 数据 | 含义 | 用途与边界 |
|---|---|---|
| joints.csv 的 effort | qfrc_actuator，实际关节广义驱动力矩/力；hinge N·m，slide N | 驱动输出侧需求；不是控制指令，也不是六维轴承反力。被动关节可能为0，必须连同接口载荷解释 |
| joint_loads.csv 的 Fx/Fy/Fz/Mx/My/Mz | 在每个关节子体原点的 MuJoCo force/torque sensor，随子体运动的 site 系，MuJoCo 子体朝父体符号约定 | 刚体树接口载荷，包含下游惯性、接触和外扰的影响；不等于具体轴承分担或柔性结构应力 |

新生成器在每个原始关节（含 fixed）子体原点加入 `load_site_<joint>`，并生成 `load_force_<joint>`、`load_torque_<joint>`。不增加质量、焊接体或假惯量；不把 fixed 伪造成可动 DOF。移动关节的测点随子体移动；若设计截面位于不同位置，必须另建测量 site 并做力臂换算和验证。

接口 CSV 同时保存测点 world 位置和 `R_world_site`，便于后处理：`F_W=R_WS F_S`，`M_W=R_WS M_S`，换参考点 O 时再加 `(p_S-p_O)×F_W`。不能只旋转力矩而漏力臂。metadata 登记符号、单位、传感器与参考点。

## 全关节覆盖

1. 模型生成后读取同 run 的 `report/load_handoff.json`；inspect_generator 会校验其清单哈希并嵌入 model_inspection。
2. 新项目用 `robot_joints: "all"`，按上游主动 DOF 名称展开（含 passive 模式），不把场景工件 freejoint 当机械臂。夹爪如需纳入另列明确名称，或明确合并模型的范围。
3. 显式列出 robot_joints 时也必须包含上游全部活动关节，额外关节在 coverage 单独登记。场景装配后的关节所属 body、接口父子体、传感器类型、测点与方向均核对。
4. `reports/load_coverage.json` 与 metadata 给出活动关节覆盖、固定连接、额外关节及接口传感器可用性。旧包可从 verified normalized_model 恢复活动关节名单，但不能凭空得到六维传感器；需要六维载荷时重新生成或有依据地补测点。
5. 仅有手工名称且无上游名单时标为 `unverified_legacy_joint_scope`，不宣称已经证明“全关节”。没有 TCP 需求可设置 `end_effector_sites: []`。

## 谱导出

`analyze.py RUN` 自动生成载荷谱；无需生成全部图片时可单独运行：

```sh
python scripts/load_spectrum.py "RUN" --bins 32
```

| 文件 | 内容 |
|---|---|
| data/joints.csv | 完整带符号关节运动、驱动力矩/力、功率及广义分量 |
| data/joint_loads.csv | 全部可用连接的六维接口载荷及参考坐标 |
| reports/load_statistics.csv | 按全程、cycle、phase 的峰值、时间均值与 RMS；功率均值×持续时间是有符号机械功 |
| reports/load_spectrum.csv | 各载荷分量的带符号区间、持续时间、时间占比；也含速度与功率谱 |
| reports/effort_speed_spectrum.csv | 关节驱动力矩/力—速度联合驻留谱，保留方向，只列有占用的格子 |
| reports/load_spectrum.json | 导出状态、分箱/时间规则、数据哈希、覆盖范围、任务原始结果与限制 |

时间积分按每个记录值代表直到下一样本的区间（左端保持近似）。末样本参与峰值但没有额外持续时间；非均匀采样按真实 dt 加权。只有物理 reset 事件跨越区间排除并登记数量，不跨跳变积分；连续物理运动的 cycle/phase 切换保留完整时间，区间归前一状态。轮次编号改变本身不代表物理复位，适配器必须记录真实复位事件。只有单样本时 RMS/均值留空。历史 analyze 的 statistics.csv 使用梯形积分，有限采样下可能与该左端近似略有差异，以各自方法说明为准。

分箱默认每个 joint/signal 全程统一等宽32箱（可设1..256），全程/轮次/阶段共用该边界；常值信号使用小的对称包络，末箱包含最大值。缺行、重复关节、非有限值或无 reset 的时间倒退会导致失败；旧成功标记被 failed 取代，不复用旧谱作新结果。多份 CSV 不构成目录事务，必须检查同次 `load_spectrum.json.status=exported`。

这是载荷水平的时间驻留谱，不是雨流循环计数。完整时序作为疲劳/耐久后处理的原始输入保留；材料S-N、传动比/效率、载荷分配、实际班次和任务占比未给出时，不推算寿命或电机轴扭矩。多工况的工序比例须由实际作业计划给定，失败工况与生产占比也分开，不平均各工况的峰值来代替包络。

## 非分拣示例与通用扩展

自由空间周期运动可复制 `assets/workloads/joint_cycle.py` 为项目 `code/task.py`，显式在合同 workload 设置 period_s、settling_s、excursion_by_joint、completion_tolerance_SI，每关节均填 SI 值。该例只支持一关节一直接位置伺服、gear=1；不模拟缺失的驱动能力。phase 为 excitation/settle，碰撞合同也需覆盖这两个阶段。幅度、周期、轮数优先取用户目标；概念模式按[常规试算](concept-trial.md)补齐并记录，严格模式不自动写演示值；应另核对速度、加速度、跟踪误差和输出限值。需要接触力、插装、视觉等控制时实现对应适配器，不强套这个例程。

沿用适配器的 `control/evaluate` 接口；外扰通过已登记的 xfrc_applied、接触或其他物理输入施加，不能逐帧改 qpos 假造工作循环。结构、工具或传感器变化时重新记录模型与合同，不复用旧映射。

## 借鉴来源

参考 robotics-antigravity-plugin 本地审阅版本 `0f640d8` 的 motion-planning、sim2real-gauntlet/preflight.py、perturb.py 与 iteration 的设计思路：任务阶段独立、实测验收、成功判据正负例、工况扰动与命名接口。本实现未引入其 DeviceLayer、Torch 或 Antigravity 运行依赖，也未移植其规划器；几何规划仍通过任务适配器和本工程碰撞合同执行。
