# 输出目录、字段与载荷解释

用户指定任务输出根目录；技能源码保持只读。每次正式/试验运行新建目录，失败也保留并标记，不能以旧结果当作新结果。

```text
project/
  task_contract.json              用户原文、确认、模式、轮次、数据要求
  model_inspection.json           上游入口/来源/哈希/验证状态
  source/                        上游完整模型与报告只读副本
  scene/scene.xml, meshes/...     装配后的任务场景
  assets/manifest.json, ...       生成/导入资产、物理参数与来源
  plan/strategy.json, ...         任务状态机、策略、轨迹和参数
  code/task.py, ...               任务适配器、生成器、感知实现
  environment-lock.txt            实际依赖锁定，另记录安装命令
  runs/<UTC时间-唯一id>/
    input/                       合同、场景、资产、规划和代码快照、hash
    data/joints.csv              关节状态和广义载荷（长表）
    data/joint_loads.csv         全部可用关节/fixed接口的六维载荷、参考点和方向
    data/end_effectors.csv       TCP 状态（长表）
    data/actuators.csv           控制量和执行器空间量
    data/contacts.csv            每个接触点的接触量与坐标轴
    data/sensors.csv             所有模型传感器原始输出
    data/diagnostics.csv         动力学平衡、接触数、告警
    data/events.jsonl            阶段、轮次与失败/重试/复位事件
    data/metadata.json           单位、坐标系、映射和采样语义
    data/observations/           任务需要的相机/感知/目标与误差证据
    figures/*.{png,svg}          各关节及各 TCP 的独立时序曲线图
    reports/statistics.csv       整程/逐轮极值、绝对峰值、时间加权 RMS
    reports/load_coverage.json   上游机械臂关节与实际记录/接口测点覆盖
    reports/goal_validation.json 成功判据正负例自检或not_provided
    reports/load_statistics.csv  全程/轮次/阶段的载荷统计
    reports/load_spectrum.csv    带符号载荷水平—时间驻留谱
    reports/effort_speed_spectrum.csv 关节力矩/力—速度联合驻留谱
    reports/load_spectrum.json   本次谱状态、分箱、哈希、范围及任务结果
    reports/limit_exceedances.csv 越限区间、峰值、样本数及限值来源
    reports/result.json          完成状态、实际轮数、逐轮判据、失败原因
    reports/environment.json     实际运行版本和模式
    reports/report.html          单文件离线总报告，内嵌全部关节/接口/TCP 曲线
    data/joint_modules/*_state.csv 每关节模组完整状态与对齐的六维接口载荷
    data/joint_modules/*_actuators.csv 每模组映射到的执行器原始信号
    data/joint_states_wide.csv    全关节同一时刻的状态宽表
    reports/joint_module_summary.csv 每模组覆盖、运动范围、峰值及时间 RMS
    reports/data_dictionary.json 字段单位、坐标、缺测和多执行器语义
    reports/joint_exports.json   导出索引、来源 SHA-256 与覆盖状态
    reports/implementation.md    agent 必须完成的任务实现与结果说明
    reports/validation.json      物理/任务/数值/可视化检查
    logs/                       stdout、stderr、过程记录、截图/录像索引
```

## 数据规则

所有时序使用仿真秒，并附 cycle、phase。默认每物理步记录，运行前估计行数/体积（关节数×步数、接触数量）；很长任务需确认分块/压缩/采样方案。CSV UTF-8，列名稳定，保留正负号。零、缺测和不可计算分开，缺测用空字段并在 metadata 解释，不能填零。

**关节**：name、type、position、velocity、acceleration、effort、bias、passive、constraint、applied、power_W、limit_margin。hinge 单位 rad、rad/s、rad/s²、N·m；slide 单位 m、m/s、m/s²、N。effort 读取 qfrc_actuator 的真实 DOF 地址；功率是 effort×velocity。固定轴写入模型映射说明，不伪造运动 DOF。夹爪如需分析应明确添加到记录名称，不因同场景而默认为机械臂轴。

**末端**：site 在 world 的 xyz、姿态四元数 wxyz（保持符号连续）、线速度/加速度、角速度/角加速度。`v=Jp*qvel`，`a=Jp*qacc+Jp_dot*qvel`；角量同理使用 Jr。加速度为运动学惯性系加速度，不额外叠加重力；IMU 比力是另外的传感器量。若需工具局部坐标，新增带 frame 标识的列并正确变换，不覆盖 world 数据。

**载荷**：qfrc_constraint 是所有约束贡献的广义量，不能叫“关节反力”或直接当轴承载荷。接触 wrench 取 mj_contactForce，记录其局部轴在 world 的矩阵、位置、双方 geom；聚合到工具原点时要带力臂叉积并核对作用方向。力/力矩传感器须登记附着 site、方向、单位与参考点。

需要结构切面、轴承六维载荷时，在确认的截面布置 force/torque sensors 或使用核实过的 RNE 空间载荷，明确作用体、符号、参考点和坐标变换；当前通用工具不会把 qfrc_constraint 自动当成六维反力。xfrc_applied 非零时简单广义平衡检查置空，须先转换外力再作残差校验。

**载荷谱与寿命输入**：保留完整带符号循环、停留、起停、冲击和异常记录，自动导出时间驻留谱与力矩/力—速度联合谱；定义、全关节覆盖、六维参考点和命令见[载荷谱约定](load-spectrum.md)。雨流计数、疲劳损伤、电机轴换算另需相应算法及任务占空比、减速比/效率、材料 S-N 等参数；不能将时间谱当循环谱或将有限几轮成功称为寿命通过。

## 曲线图与限值

用关节与末端 CSV 生成多张可独立查看的 PNG 曲线，可同时保留 SVG。关节覆盖位置、速度、加速度、驱动力矩或力及功率。末端覆盖位置、姿态、线速度/加速度和角速度/加速度。每图写明信号、单位和仿真时间，轮间物理复位处分段。场景图片可补充任务证据。

在 `assets/joint_limits.json` 给出适用的已知限值，例如 `{"joints":{"J2":{"velocity":{"lower":-1,"upper":1,"source":"上游 drive_design.json，rad/s"}}}}`。位置限值可从编译后的 MJCF 读取。驱动力矩或力限值须对应记录的广义量，执行器空间力和关节侧量需要核对传动。缺少适用限值时在图中说明。用全采样判定越限，绘图抽样应保留分桶极值。绘出限值和越限区间及峰值，并输出明细 CSV。

## 实现与验证文档

读 `reports/preflight.json` 核对初始场景、上游装配状态与可用轨迹覆盖。`reports/result.json` 的 `collision`、`collision_path` 和 `collision_failure` 分别给出逐步规则判定、已检查轨迹段和首次失败证据。场景在预检即失败时通过本次 run 的诊断直接定位，尚无物理步时只有初始记录与报告。

`implementation.md` 用用户语言写清：原任务与确认记录；输入模型/质量可信度；资产生成方法与物理参数表；策略和真实观测来源；阶段与轮次定义；安装/启动/再次导出命令；实现方法与数据采样定义；每轮实际结果与失败/重试；检查/收敛结果；主要载荷和曲线链接；准确输出路径；已知限制与后续设计可用/不可用范围。

自动 HTML 提供曲线索引。文档还须给出实际任务判据、结果和查看器验证记录。场景截图或视频如来自回放或离屏复跑，应注明来源及适用的时间倍率。

## 概念试算来源

结果附 assumption_policy、simulation_scope、实际 assumptions 与原始输入；load_proxy 只报告代理工况的物理运行和谱，不称原任务验证通过。默认授权是补齐参数的委托，不能把 Agent 估值标为用户实测。修改描述后新建 run，关联 previous_job_id；失败记录保留。

## 全关节单文件报告与模组 CSV

`analyze.py RUN` 自动调用 `joint_exports.py`。对已经存在的真实记录，可执行 `python scripts/joint_exports.py RUN` 补齐，不再调用 Agent、修改控制器或执行物理仿真。HTML 内嵌全部曲线，不依赖外部图片或联网脚本；完整 CSV 的相对链接需解压整个记录 ZIP 后使用。

每个模组状态表按精确仿真时间、cycle、phase 对齐状态、接口及可映射执行器信号；不插值、不降采样。包含位置、速度、加速度、实际驱动输出、功率及广义分量、限位余量、六维接口载荷、测点和 R_world_site。旋转轴另附角度/角速度/角加速度的度制列；原 SI 列保留。单个可映射执行器时填 drive_*，多执行器时不平均，分别保存在该模组的 actuators CSV。

固定连接只有已记录的接口数据，不伪造运动 DOF。未测温度、电流、减速器内部状态或寿命不作推断。缺测留空并记录覆盖状态；重复或不对齐的样本使整理失败，旧成功标记被替换。`exported` 仅表示已记录数据整理完成，上游全关节范围仍看 load_coverage。原始状态、接口、末端、接触、事件、谱和越限文件继续保留。
