# 零件碰撞验收

在本 skill 目录运行 `python scripts/generate.py validate --output <run或output>`。复核直接读取交付的零件定义、实际 STL 和模型姿态。生成和重验共用 `report/validation.json` 中的 `collision_review`。旧版 run 可照常重验，碰撞新项标为 `not_evaluated`；新版生成后可取得本项证据。

| 字段 | 如何读 |
|---|---|
| `parts` | 稳定 ID 为 `link/part`。提供实际 MJCF geom 名、所属连杆、用途、形状、局部 4×4 位姿及尺寸。盒体用全长宽高，圆柱用半径与全长，单位 m。 |
| `mating_pairs` | 仅列生成器已知的零件配合。目前为同一关节的输出轴与空心定子、滑块与滑轨。定子记录内孔可用半径和轴向范围，滑轨记录局部导向包络和关节行程。复核按实际位姿和区域判断。运行时已有相邻刚体过滤，独立几何检查仍检查这些连杆上的其他零件。 |
| `findings` | 每个异常零件对保留证据最强的一次，包括姿态名称、可复现的关节值和方法。盒体间使用分离轴定理，圆柱组合使用解析投影排除和共同内点检测。圆柱证据不足时记录候选。数值表示盒体平移分离距离或共同内点的内切半径，注意方法与单位。 |
| `named_pose_results` | 分别保留几何零位、初始位及用户验收位的代理相交证据。关节范围外的几何零位用于复核装配，运动插值只连接范围内的端点。 |
| `coverage` | 包含零位、初始位、命名验收位、确定性样本和逐轴限位。零位→初始位→各验收位按关节线性插值，采样密度取决于滑移、下游杠杆臂及最小代理尺寸。每段最多 128 个区间，预算不足会报告请求与实际密度。场景或任务轨迹另行检查。 |
| `contact_surfaces` | 自动核对生成法兰端面中心和四个内部采样点。记录声明点到**实际视觉 STL**的最近距离及到物理盒体或圆柱表面的有符号距离。附加任务工具可在下游场景按同样的物理单位验证。 |

`failed` 表示采样姿态中存在已证实的非预期碰撞代理相交，或已定位的视觉接触点超出所声明的代理容差。生成的 MuJoCo、Isaac 目标验收失败，模型和报告仍保留用于诊断。`needs_review` 表示存在未能确认的几何候选、采样预算不足，或声明点还需与视觉模型核对。`passed` 限于报告给出的姿态、插值分辨率与接触点。历史 `interference.candidates` 是连杆级视觉包围盒初筛；其重叠量不能用作穿透深度。

**声明设计接触面。** 输入顶层可选 `contact_surfaces` 数组。每项给出 `part`（如 `wrist_link/flange`）、`point_link_m`（该连杆局部系中的三维视觉表面点）、`tolerance_m` 和 `source`（图纸、公差或获准测试依据）。可为同一接触面指定多个点。若点偏离视觉表面，先复核坐标系；若点位于视觉表面而代理有明显偏差，调整对应物理表示。外观装饰和交互用触面分别判读。

**可复现样例。** `python -m unittest discover -s scripts/tests -p test_collision_review.py -v` 取生成的定子和输出轴测合法配合，再在同样的两条连杆上加入相交壳体。另测法兰面与缩短的代理，以及实际斜轴滑台的导轨配合和回缩干涉。正例 `assets/two-revolute-driven.json` 提供完整质量、惯量、两轴驱动和初始状态。用 `python scripts/generate.py generate --input assets/two-revolute-driven.json --output output --targets urdf mujoco` 生成，按任务 skill 入口检查这个 output 后可接入任务。`skew-prismatic.json` 会指出肩部梁与滑块干涉，`seven-axis-driven.json` 会报告壳体等零件对。优先复查发生姿态、轴线、部件布局、滑块完整行程与外壳留空，再重新生成。更改质量、限位或任务工况须有相应设计依据。
