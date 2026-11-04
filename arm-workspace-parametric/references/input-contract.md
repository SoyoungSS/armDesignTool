# 输入与结果契约

## 坐标和模型

`schema_version="1.0"`。长度m、旋转rad，移动关节位移m。世界W与模型基座B分开表达。

`model={name,joints,tip}` 为单条串联链，1～64个 revolute/prismatic/fixed 关节。每行有name、type、origin；主动关节另有axis和有限limits。origin与tip均为 `{xyz_m:[x,y,z],rpy_rad:[roll,pitch,yaw]}`。
旋转采用 `Rz(yaw)Ry(pitch)Rx(roll)`。子变换为 `T_origin Rot(axis,q)` 或 `T_origin Trans(axis*q)`。axis在origin之后的局部坐标中；归一化非单位轴并警告，零向量拒绝。fixed不能带axis/limits。

tip相对最后输出刚体。质量、惯量、重力均不需要，不能将静力学请求直接传入本Skill。关节名称唯一；姿态按名称映射。

`base_in_world={xyz_m,rpy_rad}` 可选，默认单位变换。它把B中的点变到W，不能重复写入首关节origin。
目标位置、目标姿态、剖面和体素统计始终在B中；点云另外返回世界坐标。重力不参与本Skill的几何可达性计算。

## 模式一：joint_sampling

```json
"mode": "joint_sampling",
"sampling": {"method":"random","samples":5000,"seed":42}
```

随机采样总数包含第一个限位内最近零位，其余采用固定LCG序列。每点保留q，完全复现不依赖时钟。
不与原NumPy/cuRobo随机序列逐点一致，也不是笛卡尔空间均匀采样。

网格模式：`{"method":"grid","steps_per_joint":3,"fixed_joints":{"J1":0}}`。
各未固定关节按含端点的linspace枚举一次笛卡尔积，不再填samples。固定关节按名称指定，越限拒绝。

两种采样最多50,000点，点数×链关节数不超过200万。超出预算拒绝，不悄悄截断网格。

## 模式二：pose_reachability

使用 `reachability`，不提供sampling。目标位置二选一：

- `points_m:[[x,y,z],...]`：最多2,000个指定目标点。
- `sphere:{center_m,radius_m,radial_steps,theta_steps,phi_steps}`：径向层1仅外球面；>=2包括球心和均匀半径层。各层角分辨率随半径缩放，最少3×3。纬度按z均匀分层，球心和南北极去重；与参考源码点数不完全相同。

可选 `orientations_rpy_rad:[[r,p,y],...]`，每点最多64个姿态。省略时只检查位置，不能称为姿态覆盖率。

`orientation_frame="base"`（默认）：输入为B中的目标RPY。
`orientation_frame="radial"`：仅用于sphere，工具+Z朝向球心，输入RPY为该参考姿态右侧相乘的扰动；球心采用单位参考姿态并警告。
这是离散姿态集，不是SO(3)均匀采样；重复姿态按重复查询计入分母，用户应避免无意重复。

`ik`可选字段：attempts默认4（1～12），max_iterations默认120（1～300），position_tolerance_m默认0.001，orientation_tolerance_rad默认0.02，damping默认0.03。`seed`默认42。
采用限位约束、多初值、步长限制及线搜索的阻尼雅可比求解；姿态误差为SO(3)对数，处理180°转动。成功需位置误差和可选姿态误差同时满足阈值。

最多16个主动关节；目标数×姿态数×attempts×max_iterations×链关节数不超过2,000万，超出则输入报错。
IK失败只表示当前预算未找到解，不证明不可达；增加预算可能改变成功率。

### 空间可达、采样灵巧与不可达分类（内核1.1.0）

每个目标先执行独立的位置IK，再检查输入的姿态集合；任何成功的姿态解也能证明位置有解。位置解单独保存在 `position_solution` 中，即使姿态全未解，也能重放该位置解。

| category | 定义 | 颜色 |
|---|---|---|
| spatial_reachable | 已有位置解，但灵巧性未评估或指定姿态未全通过 | 蓝 |
| sampled_dexterous | 至少两个不同旋转、全部输入采样姿态找到解 | 绿 |
| partially_dexterous | 至少两个不同旋转，部分输入采样姿态找到解 | 橙 |
| unreachable | 超出保守几何外界，已证明位置不可达 | 红 |
| unresolved | 界内且当前预算未找到位置解 | 灰 |

绿色和橙色均为空间位置可达子集，`summary.spatial_reachable_point_count` 包括蓝+绿+橙；`workspace_counts`五项互斥，其和等于总点数。
旋转矩阵等价的重复RPY（例如yaw=0和2π）只算一个不同姿态，不能用重复项获得灵巧标签。连续任意朝向的严格灵巧工作空间仍未证明，`scope.dexterous_workspace_proven=false`。

几何外界以首关节origin的xyz为球心，半径为后续origin平移范数之和、tip平移范数，以及所有移动关节最大绝对行程之和。三角不等式保证末端不会超出该球界；判定时另留位置容差与数值余量。界内点可能仍不可达，但当前不对其作数学否定。
`geometric_reach_bound`记录此外界；`spatial_status`为reachable/unreachable_proven/unresolved；`dexterity_status`记录姿态测试状态；`unreachable_reason`仅在证明不可达时填写。

IK预算现在包括额外的位置检查。`ik_query_count`保留采样位姿检查数；`position_check_count`记录位置检查数；`numerical_ik_solve_count`为实际调用数，几何提前拒绝时不调用IK。
FK模式只能给出已采到的蓝色位置可达点，不能据未采样区域判断不可达，灵巧数量返回null表示未评估。

## 模式三：dexterity_voxels

使用同一 `model` 和 `base_in_world`。将 `mode` 设为 `dexterity_voxels`，使用 `dexterity`，省略 `sampling` 和 `reachability`。

```json
"dexterity": {
  "voxel_size_m": 0.10,
  "orientations_per_voxel": 20,
  "seed": 42
}
```

- 体素边长默认 0.10 m。中心以基座坐标原点为格点锚点，每轴为 `(索引 + 0.5) × 边长`。只检查保守几何外界内的中心。可选 `bounds_base_m: {"min":[x,y,z],"max":[x,y,z]}` 进一步裁剪任务区。
- 朝向数默认 20，允许 2–100。由 `seed` 生成一组共用的均匀 SO(3) 随机朝向，每个中心按相同顺序测试。以相邻体素同朝向的已知解作为额外初值，失败后用常规多初值 IK。可选 `ik` 与目标模式相同，默认 6 个初值，每个 180 次迭代。
- 批次不限时，完成全部计划位姿后输出结果。网页的预估耗时仅供参考，运行时可手动取消。旧输入的 `time_budget_s` 字段仍可读取，求解时忽略。单批限制 100,000 个包围盒格子、50,000 个候选中心及 1,000,000 个计划位姿检查。
- `dexterity.orientations_wxyz` 记录实际测试的四元数。每点记录 `voxel_index`、中心坐标、`orientation_tested`、`orientation_successes`、`orientation_total`、`successful_orientation_indices` 和一个示例关节解。体素的 `reachability` 为成功数除以计划朝向数。0% 表示测试中没有找到解，几何可达性仍待定。结果中 `dexterity.completed` 为 `true`，`time_budget_s` 为 `null`。
- 网页剖面用基座系 XY、YZ 或 XZ。拖动滑块逐层移动，三维剖面随安装变换转入世界系，可仅显示当前层。CLI 的三个剖面 SVG 使用最靠近 `sections.center_m` 的层，CSV 包含每个中心的测试数和得分。

## 剖面与输出

`sections={center_m,thickness_m,voxel_size_m}`，默认 `[0,0,0]`、0.03m、0.02m。
XY/XZ/YZ剖面选择距相应平面不超过总厚度一半的全部目标点，保留不可达及待定点；二维投影同样包含全部点。网页分类筛选同步作用于三维、投影和剖面，SVG导出带分类图例。

JSON包括input_snapshot、normalized_input、input_sha256、model_sha256、joint_names、points、summary、sections、scope、warnings。

每点保留B/W位置。FK 保留 q。`pose_reachability` 保留每个成功姿态的 q、姿态编号及位置/姿态误差，`reachability` 为成功姿态数除以本点声明的姿态数。仅位置模式的分母为 1，几何不可达目标全部计失败。q 优先为成功姿态解，其次为独立位置解。体素模式的逐点输出见上节。
summary的包围盒只统计已找到解点；全部失败时为null。`sampled_voxel_volume_m3`只表示已占据采样格子的体积，受分辨率与样本数影响，不能当作真实工作空间体积。

核心输出不含实际耗时或随机运行 ID。未触发体素时限时，同版本、同输入、同环境的计算可复现。跨平台浮点运算允许微小容差。网页桥接的 `execution` 包含实际运行 ID、耗时、时间和原始文本哈希。

CLI输出主JSON、points.csv、report.md、request.json，以及三个projection和三个section SVG。主JSON在配套文件完成后原子替换；配套文件不是目录事务。失败必须检查退出码，旧结果可能仍存在。

## URDF转换

```powershell
python scripts/import_urdf.py --input robot.urdf --base-link arm_base_link --tip-link arm_r_link_8 --output outputs/right.json
node scripts/run.mjs --input outputs/right.json --output outputs/right-result.json --report-dir outputs/right-report
```

转换器仅使用Python标准库，提取指定base到tip的有向串联路径；固定段保留，不加载STL、碰撞、惯量或GPU配置。拒绝mimic和无有限限位的continuous关节；应先建立明确的分析范围和等效模型。生成请求仍由内核校验。
