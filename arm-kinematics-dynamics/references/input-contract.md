# 轨迹、运动状态与动态载荷输入输出契约

根字段：schema_name="arm-motion-request"、schema_version="1.0"、design_model（公共参数模型）、trajectory；可选 motion_limits、singularity、tcp_offset、dynamics。未知字段拒绝，全部采用 SI 单位。最多16主动关节、20,000个样本；旋转与移动关节分别使用rad与m。

## 生成轨迹

trajectory.kind="waypoints"；start_q 为公共模型主动关节顺序的位置数组。segments 为1～200个 `{id,kind,duration_s,q_target?}`，kind为move/hold/wait；move必填q_target，hold/wait保持前一终点。时长>0；首段起点及所有终点必须满足位置限位；不做角度取模或最短转动假设。

sample_dt_s 范围0.0001～1；每段均匀分割，使实际间隔不超过该值，同时插入速度和加速度极值时刻。重复段边界只保留一条记录，边界属于前一段，下一段从其后继续。输出 segments 描述完整阶段时间，不根据样本数计算保持时间。

timing_policy为strict（默认）或stretch。每段使用 s(u)=10u³−15u⁴+6u⁵，端点速度/加速度为零。各关节位移d的速度峰值为1.875|d|/T，加速度峰值为10/√3·|d|/T²，跃度峰值为60|d|/T³。若允许stretch，按所有关节限制确定最小时长。段间跃度不保证连续。

motion_limits 在生成模式必需，以主动关节稳定id为键，每关节给 velocity_max、acceleration_max（正数），可选jerk_max。演示值不是厂商约束，调用方负责提供实际要求。

## 外部轨迹

trajectory.kind="samples"，joint_ids 与模型主动关节ID集合相同，可以不同序，samples 至少2条，每条 `{time_s,q,dq?,ddq?,phase_id?}`。时间严格递增；dq/ddq必须对全部样本提供或全部缺省；只有ddq而无dq拒绝。

derivatives="none"（默认）保留缺失值为null；"estimate" 至少3点，用局部三点多项式估计缺少的dq，再从dq估计缺少的ddq。估计不滤波且可能放大噪声。即使采样全部通过，点间可行性仍为未知。

## TCP、坐标与奇异性

tcp_offset 可给 xyz_m/rpy_rad，表示模型tip到TCP的固定变换；缺省分析模型tip。内部附加固定变换，不改变源模型哈希。

TCP位姿分别提供基座系与世界系；速度、角速度及加速度字段明确在基座系。朝向为旋转矩阵，不将欧拉角导数当角速度。加速度使用 J*qdd + 沿qd方向的中心差分Jdot*qd，方法在scope记录；不是解析Jdot。

singularity.task为position或pose（默认），characteristic_length_m默认1 m；rank_relative_tolerance默认1e-8（<1）；near_sigma_threshold默认0.05；condition_warning默认100。数值秩同时使用1e-12绝对底限。

对常规行矩阵J，平移行除特征长度，旋转行不变；移动关节列乘特征长度。单边Jacobi SVD避免先计算JJᵀ而平方条件数。输出奇异值取min(任务维数,关节数)个；任务维数大于关节数另标underactuated。秩下降输出singular，条件数为null并标记infinite；其余按sigma/条件数阈值输出near_singular或regular。

奇异区间仅为相邻样本观察区间，不保证连续时间边界；近限位按关节范围5%记录。没有期望末端路径时，不判断受限任务方向或跟踪成功。

## 输出与失败语义

result包含输入快照、哈希、normalized_model（含TCP变换）、joint_ids/names/types、完整轨迹段、samples、events、summary、scope及warnings。

summary.trajectory_feasible：false表示已有约束违规；true仅表示生成轨迹的声明运动约束解析检查通过；null表示输入信息或连续区间证据不足。碰撞、动力学和实际任务完成均未据此认证。

启用 dynamics.enabled 后提供关节动态载荷。默认仅分析运动状态；动力学必须显式启用。

## 关节空间随机生成

CLI 使用同一个 `scripts/run.mjs --input generation.json --output batch.json`。生成请求的 `schema_name` 为 `arm-random-motion-request`，`schema_version` 为 `1.0`。另需提供 `template`（完整的 `arm-motion-request` JSON，包含 `motion_limits`）和 `settings`。默认网页设置对应：

```json
{
  "seed": 42,
  "trajectory_count": 3,
  "move_count": 3,
  "max_step_fraction": 0.1,
  "min_speed_fraction": 0.4,
  "max_speed_fraction": 0.8,
  "min_hold_s": 0,
  "max_hold_s": 0.5
}
```

种子是 0 到 4294967295 的整数。每批 1–10 条轨迹，每条 1–20 个运动段。最大单段位移比例为 0.001–1。速度比例为 0.05–1，决定段时长为 `max(sample_dt_s, 最短允许时长 / 速度比例)`。停留范围为 0–60 秒，填 0 到 0 时跳过停留。起点来自 `start_q`，或按主动关节顺序映射外部轨迹的首个样本。外部轨迹模板的生成采样间隔为 0.02 秒。

输出 `arm-random-motion-result`，包含输入哈希、设置及 `trajectories`。每个条目带有 ID、时长及标准的 `request`，供后续分析。每条最多 20,000 个分析样本。生成使用关节限位及段间位移约束，碰撞、笛卡尔任务和动态载荷仍需另外评估。


## 动态载荷请求与结果

```json
{
  "enabled": true,
  "payload_inertia_kg_m2": [[0.003, 0, 0], [0, 0.003, 0], [0, 0, 0.003]],
  "external_wrench_world": {"force_N": [0, 0, 0], "moment_Nm": [0, 0, 0]}
}
```

以上对象放在请求的 `dynamics` 字段。模型全部关节需显式 `components`，有质量的部件需 `inertia_kg_m2`。惯量采用关节运动后坐标轴、绕部件质心，单位 kg·m²。张量需对称、半正定，并满足主惯量三角不等式。模型必须提供 `gravity_world_m_s2`，允许为零。正质量的 payload 必须提供上述负载惯量，绕负载质心、按物理 tip 坐标轴表达。显式零惯量表示点质量假设，由调用方记录来源。

外载荷为环境施加给机器人的力/力矩，作用点为模型物理 tip 原点，分量在世界系表达，整条轨迹保持不变。默认零外载荷。TCP 分析偏移只改变运动学观察点，负载和外力始终绑定物理 tip。各时刻需要完整 q/dq/ddq，显式估算的导数会保留相应警告。

每个 sample 新增 `effort` 和 `dynamic_loads`，按主动关节顺序排列。每个载荷包含 effort、interface_wrench、axial_force_N、radial_force_N、bending_moment_Nm。旋转关节 effort 单位 Nm，移动关节单位 N，正方向跟随关节轴。interface_wrench 为上游对下游子链的作用，分量在基座系表达，力矩参考点为记录的 `point_m`。轴向力带符号，径向力和弯矩为非负模长。移动关节的参考点随关节平移。

summary.dynamic_joints 提供正负极值、绝对峰值及其时刻和姿态、时间加权 RMS、机械功率绝对峰值，以及轴向力、径向力和弯矩峰值。dynamic-loads.csv 同时导出六维接口载荷和参考点。JSON dynamics 字段记录算法、坐标和统计语义。刚体动力学峰值按采样统计，完整载荷谱还需工况权重与重复次数。

算法参考 [Modern Robotics 的 Newton–Euler 逆动力学教材](https://modernrobotics.northwestern.edu/nu-gm-book-resource/8-3-newton-euler-inverse-dynamics/)。
