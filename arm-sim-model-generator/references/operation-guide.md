# 操作细则

命令和反引号中的相对路径均以本技能根目录为基准；本页的 Markdown 链接相对于本页。仅阅读入口指定的相关章节。示例参数只用于演示，不是用户设计或厂家能力。版本清单和历史验收不代表当前环境已验证。

使用随附生成器完成建模和导出。文件契约与 CLI 不依赖特定模型供应商；执行需要文件读写和 Python 环境。遵循当前平台上位指令及用户的明确要求与授权范围。工具不可用时说明未执行的步骤，不编造运行记录。

## 1. 选择入口与资料

| 用户目标 | 执行入口与必读资料 |
|---|---|
| 自定义机械臂，或适配静力学/工作空间原始请求 | 本文普通生成流程；先读 [输入契约](../references/input-contract.md) |
| 项目的默认选型表七轴臂或 FR3 | 先读 [默认模型与轨迹](../references/default-workflows.md)，使用 `default_model.py`；需要 MuJoCo |
| 复核已有模型包 | 使用下文 `validate`，先确定实际 run；不重新猜测输入 |
| 核对任务中的机械臂轨迹 | 按 [默认模型与轨迹](../references/default-workflows.md) 的 `check_path.py` 约定；工具/工件/桌面接触还需任务场景验收 |

只读取当前任务需要的进一步资料：

- 缺 Python/包或需离线安装：[环境与依赖](../references/environment.md)。相邻任务 Skill 的离线包是可选共享资源；独立分发时可能不存在，不依赖其路径完成核心生成。
- 调整外壳：[外形规则](../references/appearance.md)；共点旋转轴再读 [模组布置](../references/joint-layout.md)。
- 导出 MuJoCo/USD 驱动：[驱动规则](../references/drives.md)。
- 借鉴成熟轴系或惯量参考：[经典机械臂参考](../references/reference-arms.md)。记录型号和来源，不将参考臂额定值当作新臂参数。
- 解释碰撞结果或修复干涉：[碰撞判读](../references/collision-review.md)。
- 运行目标环境验收：[各目标验证](../references/validation.md)。

## 2. 确认输入，保留设计依据

先确定输入文件、输出目录、目标格式及是否要求运行时验证。用户未指定格式时，结合用途选择并说明；命令显式传 `--targets`，避免默认生成全部目标。CLI 名称仅为 `urdf`、`ros1`、`mujoco`、`isaac`，其中 MJCF 对应 `mujoco`，USD 对应 `isaac`。

按输入契约建立单独的 JSON/JSONC 请求。参考 [完整双旋转轴正例](../assets/two-revolute-driven.json)。[worksheet.jsonc](../assets/worksheet.jsonc) 是带密度和驱动占位值的**概念模板**，使用前替换参数并确认概念估算依据。示例不得冒充用户设计。

生成前检查：

- 输入类型：本生成器请求或已支持的原始静力学/工作空间请求。源 Skill 的 `normalized_input`、本生成器的 `normalized_model.json` 和公共 `arm-design-model` 都不能直接当作本生成器请求；先显式映射到输入契约，保留来源。
- 拓扑与坐标：固定基座、单串联链；`origin` 相对父输出刚体，轴在局部关节系。世界安装只应用一次，旋转约定为 `Rz Ry Rx`。初始关节值不折入 `origin`。
- 单位：内部使用 m、kg、rad、N、N·m、kg·m²。仅使用契约支持的 mm/deg 字段，不同时提供同一量的两种单位，不凭数值大小猜单位。
- 姿态与限位：活动关节有限限位；初始/验收姿态恰好包含全部活动关节且限位内，fixed 不含轴、限位或驱动。全零几何姿态可能在行程外，与初始姿态分别保存。
- 质量与负载：COM 所属参考系明确，惯量在 COM 处。负载必须声明已计入哪个组件或确认独立加入；不按质量相等推断重复计重。
- 驱动：`effort`/`velocity` 有来源，不能用静力学保持载荷或电机额定值直接代替输出限值；显式增益保留。

单位、轴系、连接关系或负载归属存在影响正确性的歧义时先澄清。尺寸可按现有规则推导并报告；质量/惯量估算需要明确概念模式和 `physics.estimation`，驱动限值占位需要显式配置。没有依据时记录缺项，交付能力按目标分别说明，不能为让校验通过自动切换概念模式或补造参数。

## 3. 使用同一个解释器生成与复核

以下命令的工作目录是本 Skill 目录 `arm-sim-model-generator/`。`python` 代表已确认的 Python ≥3.10 解释器；按机器实际情况替换为 `python3`、`py -3` 或虚拟环境解释器的路径。输入/输出相对路径均相对此工作目录，含空格时加引号。

先用该解释器检查版本并实际导入 `numpy` 和 `PIL`。依赖以 `scripts/requirements.txt` 为准；缺少时使用项目隔离环境或离线包。USD 生成额外需要 `pxr`；MuJoCo 运行时检查需要 `mujoco`。仅有 OpenUSD 不代表具备 Isaac Sim/PhysX。记录实际环境，不把依赖清单或历史验收记录当作本机执行证据。

下面是可复现的正例。处理真实请求时，将输入和输出替换为本次路径，保留原始用户文件：

```sh
python scripts/generate.py generate --input "assets/two-revolute-driven.json" --output "output/custom-arm" --targets urdf mujoco
python scripts/generate.py validate --output "output/custom-arm"
```

需要 MuJoCo 运行验证且依赖具备时，在生成和复核命令上均追加 `--runtime`。未具备环境时报告运行时待验证。普通生成无需每次运行开发测试套件。

每条命令立即记录退出码和 JSON 输出。无论生成成功或失败，都先读取本次输出目录的 `latest_attempt.json`，取得 `run_directory`；后续链接和报告只使用这个 run。若有顶层 `error`、状态仍为 `running` 或关键报告缺失，按失败/未完成处理，不选择旧的成功目录替代。

可在 `scripts/` 导入 `arm_sim.pipeline.generate(input_path, output, targets=None, runtime=False)` 和 `validate(output, runtime=False)`；API 返回结果同样需要逐目标判读。

## 4. 按证据判断，不合并不同验收范围

读取实际 run 的 `report/manifest.json` 与 `report/validation.json`：

| 证据 | 可得结论与下一步 |
|---|---|
| 命令退出 0 / 整体 `completed` | 流程完成；仍可能有缺依赖、缺物理或待复核目标。继续检查以下字段 |
| `targets.<目标>.generated=true` | 文件已生成；不表示静态或动力学验收通过 |
| `static_checks=passed` | 对应格式与资源静态检查通过；仍需检查装配与碰撞结论 |
| `assembly_validation` / `validation.module_packing` | 装配结果；`needs_review` 或未解析项不能视为通过 |
| `collision_validation` / `validation.collision_review` | `failed` 为已证实问题；`needs_review` 是待复核；`passed` 仅覆盖记录的姿态和采样密度。`not_evaluated` 不算通过 |
| `runtime_validation=passed` | 对应运行检查已通过；结合 `validation` 读取实际项目与范围。MuJoCo 结果不能替代 ROS GUI 或 Isaac/PhysX 验收 |
| 目标 `pending`、`failed`，或字段缺失 | 报告缺项、原因和受影响目标；已生成文件可作诊断或部分交付，不宣称全部通过 |

碰撞候选不等于已证实穿透；AABB 重叠量不当作穿透深度。URDF 静态检查可能通过而碰撞检查失败，两项分开报告。位置驱动稳定不等于定位误差达标，有限姿态/轨迹通过不证明全工作空间无干涉。

失败后按报告定位输入、环境或几何问题。改变用户的轴系、限位、质量、驱动上限、负载或任务范围需要设计依据；不能通过删除碰撞体、扩大碰撞排除、放宽容差或缩小验收范围掩盖失败。外壳尺寸修改也要记录依据；修改后重新生成并读取新的 run。没有可据以修复的信息时，保留诊断产物并说明阻塞原因。

[`skew-prismatic.json`](../assets/skew-prismatic.json) 和 [`seven-axis-driven.json`](../assets/seven-axis-driven.json) 是保留已知干涉的反例，不作为全验收通过的起步模板。

## 5. 交付同一次 run 的结果

交付说明至少包括：

1. 输入文件、实际解释器/可选运行库版本、生成与复核命令及退出码。
2. `run_directory`，本次生成的模型入口、资源包及初始姿态 `preview.png`。
3. `report/input_snapshot.json`、`report/normalized_model.json`、`report/manifest.json`、`report/validation.json`、`report/report.md`，以及本次实际存在的驱动来源报告。
4. 按所请求目标分别列出生成、静态检查、装配、碰撞、运行验证的状态和原因；未执行、待复核、失败明确列出。保留假设、单位、来源和验证范围。

交付目录按资源引用整体搬移。MuJoCo 使用该 run 的 `mujoco/run.py` 恢复 initial keyframe；USD 保留 `robot.usd`、`robot_zero.usd` 和 `runtime_config.json`；URDF 相对资源解析规则见 [各目标验证](../references/validation.md)。`preview.png` 是离线外观图，不是仿真器运行截图。对未生成或不存在的文件不提供成功交付链接。
