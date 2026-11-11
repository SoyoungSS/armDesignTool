# 确定性驱动规则 serial-pd-v1

用于生成有完整质量属性的 MJCF / USD。实现为 `scripts/arm_sim/drives.py`，与 ROS、MuJoCo、Isaac 无关。相同归一化输入和依赖版本得到相同参数；运行目录 ID 不影响模型。修改算法时更新 policy 版本，不按某次仿真结果暗中调参。

## 模式与物理边界

活动关节默认 `auto`，解析为 `position`。`position` / `velocity` 的缺失增益按本规则补齐，显式 kp/kv 保留。`effort` 为直接力/矩，USD 需要运行时下发 effort；`passive` 无驱动，fixed 无驱动。只给运动学时仍生成外观与可视化模型，不虚构驱动所需的惯量、额定力矩和速度。

effort、velocity 必须来自输入，或概念模式中明确配置的占位值。保持载荷与硬件铭牌只作来源信息，不能自动替代输出限值。不会增加虚假的转子惯量、摩擦、质量或重力补偿来消除抖动。给定的物理阻尼和摩擦独立保存。

## 固定算法

1. 固定采样：初始姿态、全部验收姿态、限位中点、限位内最近零位，再加按活动关节顺序使用质数基数的 16 个 Halton 点（索引 1–16）。无随机数、时间依赖或运行后拟合。
2. 对每个姿态，用全部下游质量、质心、旋转后的 COM 惯量及运动雅可比求 `M = Σ(m JvᵀJv + Jwᵀ Iworld Jw)`、保持重力载荷 `g = -Σ Jvᵀ m gravity_world`。包含 fixed 后的部件和显式附加负载；基座不计入活动关节惯量。
3. 令 `di=max_samples Mii`，`D=diag(di)`，`rho=max_samples λmax(D^(-1/2) M D^(-1/2))`，`bi=rho*di`。使用全链耦合信息，不能只用当前连杆质量。
4. 默认位置增益：`kp=min(max(di*(2π*f)^2, max|gi|/error), effort_fraction*effort/step)`；`step=min(design_step,0.1*joint_range)`。
5. 默认位置阻尼增益：`kv=2*zeta*sqrt(kp*bi)`。不扣除被动物理阻尼，使通用 USD 与 MuJoCo 共用保守的主动阻尼。默认速度模式 `kv=2*zeta*bi*2π*f`。保持显式增益，对显式低阻尼给出复核提示。
6. 对各采样姿态求 `omega=max sqrt(λmax(L^-1 K L^-T))`，其中 `M=L Lᵀ`、`K=diag(position kp)`。推荐 `dt=min(max_timestep,modal_step_limit/omega)`，向下截到 1 ns。小于 1 μs 则失败，要求复核参数；不会改质量掩盖问题。

这些惯量/重力最大值是采样结果，不是整个工作空间的数学上界。强耦合、接触、力矩饱和、显式低阻尼或新增负载仍需实际验证。

## 默认常数与覆盖

可以在 `export.drive_policy` 覆盖以下已知字段。缺省值固定；拒绝未知字段与非正值。

| 字段 | 默认 | 含义 |
|---|---:|---|
| frequency_hz | 1.5 | 基础伺服频率 |
| damping_ratio | 1.2 | 估算阻尼比，至少 1 |
| position_error_rad | 0.035 | 旋转静差设计目标 |
| position_error_m | 0.002 | 移动静差设计目标 |
| design_step_rad | 0.1 | 旋转力矩预算步幅 |
| design_step_m | 0.01 | 移动力预算步幅 |
| proportional_effort_fraction | 0.5 | 比例项力/矩预算，最大 1 |
| max_timestep_s | 0.001 | 步长上限，最大 0.005 |
| modal_step_limit | 0.1 | 模态频率乘步长上限，最大 0.2 |
| command_filter_s | 0.03 | 默认位置指令一阶滤波常数 |

单轴 `timeconst` 可显式为 0 以禁用指令滤波。输入 kp 单位为 Nm/rad 或 N/m，kv 为 Nm·s/rad 或 N·s/m；timeconst 为 s。输出 `report/drive_design.json` 保存请求值、解析值、惯量/重力证据、增益来源、步长和警告。

若达到比例项力矩预算后仍不能满足静差目标，报告预计静差，保留真实 effort 限值；不声称精准保持。采样重力载荷超过 effort 时报告驱动能力不足。位置驱动的 velocity 不是引擎硬速度约束；实际应用应使用连续限速轨迹，并验证负载下速度。

## 格式映射与启动

MuJoCo：position actuator 使用 kp/kv、力矩限制、限位内 ctrlrange 和 `timeconst`；velocity actuator 使用 kv 和速度目标范围；effort 使用 motor。采用 `implicitfast` 与计算的 dt。initial keyframe 同时保存 qpos、ctrl、滤波 activation，防止开始时目标从零跳变。

```bash
python /path/to/run/mujoco/run.py
python /path/to/run/mujoco/run.py --headless --seconds 5
```

这份启动器只依赖 MuJoCo/NumPy，与生成 Skill 分离。自行加载 XML 时，先 `mj_resetDataKeyframe(model,data,0)` 再 `mj_forward`；MuJoCo 通用查看器不一定自动加载 keyframe。

USD：在 angular/linear `UsdPhysics.DriveAPI` 上显式写 force 类型、stiffness、damping、maxForce、targetPosition、targetVelocity。旋转目标由 rad 转 deg；角 kp/kv 乘 `π/180` 转为每度，Nm 上限不变；移动参数保持 SI。入口 robot.usd 的刚体已放到初始姿态，关节原始框架存于相对引用的 robot_zero.usd，不把初始值折入零位定义。

`runtime_config.json` 保存 SI 参数和建议 physics_dt。通用 DriveAPI 无指令滤波属性，Isaac 控制程序需要按 timeconst 实施目标滤波；它也不能完整表达 MuJoCo 的被动关节阻尼/库仑摩擦，已有输入保留并提示待运行时设置。纯 USD 没有宣称与 MuJoCo 接触求解或瞬态完全一致。Isaac GUI 的默认步长可能远大于建议值，必须先设置，再初始化 articulation、核对初始 q 和目标；API 若接收 SI 增益，不得再次乘角度换算系数。

## 装配接触与验收

两个仿真目标使用同一组配合体排除：直接父子刚体，以及沿后续零平移 revolute/fixed 关节连接的共点壳体组；遇到非零偏移或 prismatic 即停止扩展。每对及原因记录在 normalized_model.collision_exclusions。它针对生成的壳体配合近似，不根据仿真穿透结果自动扩大排除，不关闭远端自碰撞。换成真实 CAD 时须重新审核。

`--runtime` 对 MuJoCo 重加载增益、限制、滤波状态和惯量，比较采样 M/g，再进行保持与平滑小幅运动。接触开启结果决定通过；接触关闭仅作诊断。静差、尾段峰峰值、速度、饱和、穿透均保留。USD 重加载核对驱动属性、角度单位、初始/零位和搬移后的相对引用；PhysX 实机测试单列 pending。具体阈值见 [验证说明](validation.md)。

官方单位依据：[OpenUSD DriveAPI](https://openusd.org/release/api/class_usd_physics_drive_a_p_i.html)、[Omniverse USD Physics schema](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/schemas/usdphysics.html)。MuJoCo 参数依据：[XML reference](https://mujoco.readthedocs.io/en/3.3.5/XMLreference.html)。
