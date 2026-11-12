# 输入、问题与确认

## 接入上游

接受生成器 output 根目录、`runs/<run_id>`、其 `mujoco/` 目录或明确的 XML。output 根目录通过 `latest_attempt.json` 的 `run_directory`（当前生成器）或 `run_id`（兼容格式）定位本次尝试；若本次失败不得偷用旧成功运行。搬移后绝对路径失效时向用户确认实际 run，不能任挑一个目录。检查 `report/manifest.json` 中 `targets.mujoco`、entry、generated、status、static_checks、runtime_validation 和 files 哈希；报告中的 pending 不是 passed。

当前生成器通常输出 `mujoco/robot.xml`、`mujoco/run.py`、meshes，以及 `report/normalized_model.json`、`drive_design.json`、`validation.json`、`input_snapshot.json`。历史版本可能不同，以实际清单为准，不能靠固定文件名静默回退。`runtime_config.json` 可能仅在 Isaac 目录，不应臆造为 MJCF 必需文件。

读取 normalized_model 的坐标/质量/初始姿态，读取 drive_design 中的驱动模式、增益、能力和来源。用 MuJoCo 编译确认 nq/nv/nu、joint qposadr/dofadr、actuator transmission、tool_frame site 与 keyframe；固定关节不占 DOF。世界安装只施加一次，复制整个资源树保持相对路径。关节侧负载和工具已有质量不得重复加算。

若上游只有可视化模型或质量/惯量占位，说明对载荷分析的影响并询问补齐或批准估算，不得声称工程载荷已经可靠。

## 一次性整理关键问题

在可从模型确定的内容之外，按依赖顺序询问，避免把整张参数表无区别扔给用户：

| 类别 | 必须弄清的内容 |
|---|---|
| 任务 | 操作什么、从哪里到哪里、顺序、容差、接触/抓稳/放置/擦净的判据 |
| 模式 | 官方 WebViewer 或本地原生窗口；当前系统、实时速度需求；是否允许无窗口预验证 |
| 工具 | 夹爪/吸盘/板擦等结构、驱动、安装变换、质量与质心、行程/开口/夹持能力 |
| 环境 | 对象尺寸/形状/位姿分布、质量/惯量、摩擦/柔顺、桌台障碍、地面、工作空间 |
| 策略 | 用户指定的控制/感知方法；未指定时确认所提规划策略；是否允许真值参与控制 |
| 周期 | 一轮开始/结束的定义、轮数、节拍、停留、连续循环或复位、失败重试是否计入轮数 |
| 随机性 | 随机化哪些参数、范围/分布、每轮是否重采样、种子和是否需要多种子试验 |
| 记录 | TCP/site 定义、坐标系、采样频率/时长、额外传感器、轴承/传动/寿命分析所需量 |

对会影响载荷或成功率的未知数不得猜测。对纯视觉装饰等非物理细节，可在用户明确委托后选取，并记录不参与动力学。用户选择已有时记录原答复，不重复确认。轮数没有默认值，不能自动写“三轮”。

## 合同

模板字段为最低要求；assets、strategy、success、cycles 的具体物理定义由 agent 扩展。`confirmation.evidence` 是实际用户答复的摘录或消息定位，不能由 agent 自写“已确认”代替。`unresolved` 非空或必需确认缺失时，脚本拒绝正式运行。

`physics_approval` 说明用户确认的物理参数/估算范围；`asset_manifest`、`strategy_file`、`adapter`、`scene` 是项目内文件。这些文件应写出细节，不能仅有名称。初始 keyframe 可为名称；若没有 keyframe，明确写 `initial_keyframe: null` 并在初始条件中说明如何初始化，不能默认全零。
