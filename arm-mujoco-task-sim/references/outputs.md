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
    data/end_effectors.csv       TCP 状态（长表）
    data/actuators.csv           控制量和执行器空间量
    data/contacts.csv            每个接触点的接触量与坐标轴
    data/sensors.csv             所有模型传感器原始输出
    data/diagnostics.csv         动力学平衡、接触数、告警
    data/events.jsonl            阶段、轮次与失败/重试/复位事件
    data/metadata.json           单位、坐标系、映射和采样语义
    data/observations/           任务需要的相机/感知/目标与误差证据
    figures/*.svg                各关节及各 TCP 的时序曲线
    reports/statistics.csv       整程/逐轮极值、绝对峰值、时间加权 RMS
    reports/result.json          完成状态、实际轮数、逐轮判据、失败原因
    reports/environment.json     实际运行版本和模式
    reports/report.html          自动数据索引和摘要
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

**寿命输入**：保留完整带符号循环、停留、起停、冲击和异常记录，另输出逐轮峰值/RMS、速度、功率和统计。载荷谱/雨流计数、疲劳损伤、电机轴换算根据用户需要另计算，需确认任务占空比、减速比/效率、材料 S-N 等参数；不能将有限几轮成功直接称为寿命通过。

## 必交文档

`implementation.md` 用用户语言写清：原任务与确认记录；输入模型/质量可信度；资产生成方法与物理参数表；策略和真实观测来源；阶段与轮次定义；安装/启动/再次导出命令；实现方法与数据采样定义；每轮实际结果与失败/重试；检查/收敛结果；主要载荷和曲线链接；准确输出路径；已知限制与后续设计可用/不可用范围。

不要把自动 HTML 的“输出已生成”当作任务完成证明。文档须有实际成功判据和证据，以及当前平台的可视化截图。PNG/PDF/MP4 可选，若生成要标明时间倍率，不能用加速回放截图冒充实时控制验收。
