# 输入与坐标约定

## 根请求

`schema_version="1.0"`。必填input_mode、payload以及对应model/simple；gravity_m_s2与installation必须且只能提供一个，poses与search至少提供一项。可选attachments、wrenches、drives、design_factor、metadata。

拒绝未知业务字段、非有限数值、布尔数值、重复名称、循环对象。metadata用于来源与自定义说明，其中normalization_assumptions保留简易模型推导假设。

统一SI：m、kg、rad、N、Nm。旋转关节q为rad，移动关节q为m。不猜测或自动转换mm/deg。旧字段gravity_m_s2在基坐标系表达，新installation字段按下述规则转换；不自动设定重力；无负载时显式给payload质量0。

## 安装姿态与重力（内核1.1.0新增）

旧字段 `gravity_m_s2` 始终表示模型基座坐标系 B 中的重力，旧输入仍可计算。
另一种输入方式是：

```json
"installation": {
  "gravity_world_m_s2": [0, 0, -9.81],
  "base_orientation_in_world": {"rpy_rad": [0, 1.5707963267948966, 0]}
}
```

RPY 使用弧度，`R_WB = Rz(yaw) Ry(pitch) Rx(roll)` 将基座向量转换到世界 W。
内核只转换重力 `g_B = R_WBᵀ g_W`，关节轴、质心、附件和局部外载不做第二次安装旋转。
J1局部轴可以是任意方向，不能用J1轴向推断整个安装姿态。

模型里的 `simple.base` 或标准模型第一个 `origin` 仍是模型根相对 B 的变换；它们与 B 相对 W 的 `installation` 分属两层，均可非零。不要把同一个安装旋转重复写入两层。
均匀重力下安装平移不影响关节保持载荷，此接口仅定义安装旋转。

`normalized_input.gravity_m_s2` 总是转换后的基座重力；使用新输入时，`normalized_input.installation` 另外保留世界重力、RPY与 `rotation_base_to_world`。
输出几何 `joint_geometry`、`tip_pose_in_base` 仍在 B 中，显示到世界时乘一次 R_WB。
`toStandardInput()` 保留安装输入，导出时不会同时写出两个互斥的重力字段。
任意世界固定外力仍不属于本次扩展范围；`wrenches` 保持原有局部随动定义。

## 标准模型

model包含name、joints和tip。joints为从基座到末端的串联链，不是树；每行表示一个关节及其输出刚体。

```json
{
  "name":"J1",
  "type":"revolute",
  "origin":{"xyz_m":[0,0,0],"rpy_rad":[0,0,0]},
  "axis":[0,1,0],
  "limits":[-1.57,1.57],
  "components":[{"name":"link","mass_kg":0.2,"com_m":[0.15,0,0]}]
}
```

- origin为相对上游输出刚体的零位安装变换，旋转采用Rz(yaw)Ry(pitch)Rx(roll)。
- revolute子变换为T(origin)Rot(axis,q)，prismatic为T(origin)Trans(axis*q)。axis在安装后的关节坐标系表达。
- com_m在当前输出刚体坐标系表达；该质量只加载当前及全部上游主动关节，不加载下游关节。
- fixed禁止axis/limits，不接受q；其质量传递给上游主动关节。全固定模型拒绝。
- 主动关节必须提供有限limits，下限小于上限。连续转动轴要明确一个有限分析范围。
- components允许显式空数组，表示无质量刚体并警告。组件质量与COM均必填，组件名在刚体内唯一。
- tip为相对最后输出刚体的固定变换，包含xyz_m、rpy_rad。关节名称tip保留。
- 非单位轴显式归一化并警告；模长小于1e-12拒绝。
- 支持任意轴向、关节偏置和非共点结构，不要求DH或URDF。

关节模组固定端应放入上游刚体，运动端放入下游。不能将同一模组在多个刚体重复计重。基座质量可以放在第一个fixed行，不会加载后续主动关节。

## 简易模型

simple包含name、可选base和segments。每段必填name、axis、length_m、limits_rad、link_mass_kg。可选direction、link_com_m、joint_mass_kg、joint_com_m、rpy_rad。

连杆direction默认局部+X，link_com_m默认长度中点，均警告。direction会归一化。joint_mass_kg可选；提供时整体放入当前输出刚体，COM默认当前原点，并警告固定端/运动端近似。提供joint_com_m时必须同时提供joint_mass_kg。

第一关节使用base的xyz_m/rpy_rad，不再接受第一段rpy_rad。base缺省为单位安装并警告。其余关节origin平移为前一段length×direction，旋转为本段rpy_rad（默认0）。最后一段向量作为tip平移，tip旋转为0。

所有link_com_m和joint_com_m都在当前关节输出坐标系表达。需要移动轴、固定中间段或质量拆分时用standard。

toStandardInput返回完整标准请求，保留metadata.normalization_assumptions；不要删除估算来源来隐藏警告。

## 负载与外力

payload={mass_kg,com_in_tip_m}，质量和相对tip的重心偏移均必填。工具已包含在模型中时不能再重复添加。

attachments每项={name,frame,mass_kg,com_m}。frame为关节输出帧名称或tip；上游附件不加载下游关节。

wrenches每项={name,frame,point_m,force_N,moment_Nm}。所有字段必填，向量均在指定frame表达，随该帧旋转。moment_Nm是自由力矩；内核会另外计算力的力臂矩，不要重复添加。

本版本没有基座帧固定外力的独立选项。指定姿态可转换外力表达后计算；范围搜索不能把世界固定力错误地当作随动局部力。未知接触反力不求解。

保持载荷抵消全部已知外载：旋转关节为−a·[(p−o)×F+M]，移动关节为−a·F。重力势能−Σmg·p的广义坐标导数等于纯重力保持载荷。

## 姿态与搜索

poses每项={name,q}，q是全部主动关节名称到数值的映射。缺项、多项或越限拒绝，不自动补0；fixed没有q。

search必填samples（1～200000）、seed（无符号32位整数）。可选elite_count（1～10，默认3）、refine_sweeps（0～40，默认8）。搜索最多16主动关节。

同时限制预估质量部件×关节×计算次数不超过5000万，防止大模型与大搜索预算组合造成不可控计算。超过预算返回analysis_budget输入错误；可减少采样、细化或姿态数。输入嵌套最多64层。

先检查中点、全部下限组合、全部上限组合及指定姿态，再用固定PRNG采样，按每关节最大绝对载荷保留精英池，固定坐标顺序与缩步预算细化。结果包括种子、计算次数、采样最大值、每关节临界姿态。不是全局最优性证明。

范围搜索不检查碰撞，不依赖IK。当前工作空间NPZ缺少关节角，不能把位置点直接当成静力学姿态。

## 驱动与判定

design_factor≥1，默认1并警告；required_output=max_abs_checked×design_factor，只乘一次。

drives按主动旋转关节名称索引：continuous_output为连续输出Nm；continuous_conditions为其适用条件字符串；ratio和efficiency必须同时提供，ratio>0、0<efficiency≤1。

电机需求=required_output/(ratio×efficiency)，仅为声明效率近似下的静态初选，不含动态、自锁、摩擦、抱闸收益。移动关节不支持驱动/丝杠换算。

| status | 解释 |
|---|---|
| not_sized | 没有驱动容量，不能判定合格 |
| exceeds_capacity | 已检查需求超过容量 |
| capacity_conditions_unknown | 数值足够，但连续能力条件未声明 |
| satisfies_checked_cases | 声明条件下，已检查范围满足 |

margin=available/required−1。无容量或需求0时为null；不通过无限大裕量表示。

## 输出与追溯

包含schema_version、engine_version、input_snapshot、normalized_input、joint_names、poses、search、summary、scope、warnings和原始/规范化SHA-256。normalized_input是内部结构，q为数组，不能直接重新提交；用toStandardInput获取可重用请求。

poses.holding_load与joint_names对应；components为robot/payload/attachments/external分项；contributions保存各部件载荷；joint_geometry和tip_pose_in_base供三维显示。

summary包含最大绝对值、临界姿态、设计需求、容量及判定。每关节临界姿态可能不同，不代表整臂所有关节同时最大。

核心不加入时钟、耗时或随机ID。同版本同输入在相同环境要求字节一致；跨JS引擎最后几位采用容差比较。来源改变可改变哈希而不改变物理结果。
