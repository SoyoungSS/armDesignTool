# 六个机械臂技能的跨模型使用说明

本次按 [Agent Skills 规范](https://agentskills.io/specification) 与[编写建议](https://agentskills.io/skill-creation/best-practices)统一技能入口。规范要求有效的 `name`、`description` YAML 元数据与 Markdown 正文，不强制固定的正文标题。本工程统一标题是为了减少使用者与模型寻找步骤的成本。

五个计算/仿真技能的说明版本为 `metadata.version: "1.2.0"`，构型选择技能统一知识库版本为 `1.4.0`，此版本表示说明包版本，不改变计算内核版本。原有计算脚本、输入输出契约和用户设计参数继续有效；本次修正文档中的矛盾，不新增计算能力。

## 任务路由

| 用户需要 | 入口 | 关键输入 | 执行条件 |
| --- | --- | --- | --- |
| 从场景初选构型 | [构型选择](../arm-configuration-selection/SKILL.md) | 任务、硬约束、偏好、随包证据 | 文件读取，或由调用方粘贴相关参考正文 |
| 计算静态保持载荷 | [静力学](../arm-statics-parametric/SKILL.md) | 几何、质量/COM、重力、负载、姿态 | Node.js ≥20 + 同级shared |
| 分析可达工作空间 | [工作空间](../arm-workspace-parametric/SKILL.md) | 几何、限位、tip、目标/采样设置 | Node.js ≥20 + 同级shared |
| 轨迹与刚体动态载荷 | [运动与动力学](../arm-kinematics-dynamics/SKILL.md) | 模型、时序、限制；动态载荷另需惯量等 | Node.js ≥20 + 同级shared |
| 生成模型及分项验证 | [模型生成](../arm-sim-model-generator/SKILL.md) | 目标格式及该目标的完整参数请求 | Python ≥3.10；依目标配置依赖 |
| 接触任务执行与时序 | [任务仿真](../arm-mujoco-task-sim/SKILL.md) | 完整模型run、任务合同、场景与控制器 | Python、MuJoCo及所选查看模式依赖 |

技能之间的数据不能直接混传：构型选择记录不是计算请求，内部normalized结果不是原始请求，公共模型需要按目标契约显式映射。只有任务确实需要下游步骤时才继续，不默认执行全链路。

## 接入其他模型

支持Agent Skills的宿主可加载技能目录，按入口的链接读取当前任务需要的参考资料。`agents/openai.yaml` 是可选的界面/调用元数据；核心执行流程不依赖其中字段，也不依赖Codex API。

不支持自动技能发现的宿主，由调用方选择技能并提供SKILL.md正文；仅传目录名或元数据不能代替完整入口。宿主至少应把真实用户输入与技能指令分开，并明确哪些工具可用。

| 模型/宿主能力 | 使用方式 | 可交付范围 |
| --- | --- | --- |
| 能读文件且能运行命令 | 按Workflow执行，每条命令检查退出码，再读本次结果 | 实际执行范围内的结果及证据 |
| 能读文件、不能执行 | 核对契约并准备输入/命令 | 待执行方案；不能给伪造数值或运行记录 |
| 只有文本输入 | 提供入口、所选模式契约、相关参考正文和用户输入 | 需求/证据整理、草案；没有执行证据不能报告已运行 |
| 上下文较小 | 一次传一个阶段/专题，保存阶段记录后继续 | 分步完成；不能把摘要当成未读全文的证据 |

调用方可用下面的交接记录保留进度，缺项写未知：

```text
技能与模式：
原始需求/已确认参数：
单位、坐标、关节ID顺序：
已读取资料及支持内容：
请求/项目路径与模型版本或哈希：
当前步骤、命令、退出码、实际run/结果路径：
已完成检查及适用范围：
未知项、失败项和下一步：
```

该记录用于接续工作，不代替原始模型、合同或结果文件。模型无法获取必读资料时，应停在受影响的结论前，保留可完成的输入整理；资料缺失不意味着自动获得联网或估算授权。

## 迁移文件

构型选择需要完整技能目录及references；模型生成和任务仿真保留自身scripts、assets与references，依赖按目标环境配置。任务仿真的离线包是可选安装资源，不能把其清单当当前环境证据。现有任务接入器要求完整生成器run；裸外部XML需另行导入验证，不能伪造manifest。

三个Node分析技能共用同级shared，不能只复制单个目录。使用现有打包器，在工程根目录运行：

```sh
node shared/scripts/package-skill.mjs arm-statics-parametric /tmp/statics-portable-bundle
node shared/scripts/package-skill.mjs arm-workspace-parametric /tmp/workspace-portable-bundle
node shared/scripts/package-skill.mjs arm-kinematics-dynamics /tmp/motion-portable-bundle
```

目标目录必须尚不存在；其他系统替换为本机可写的新目录。保持 `<bundle>/<skill>/` 与 `<bundle>/shared/` 同级。需要哪项就打包哪项，无需同时加载全部技能。

## 验证范围

本次检查了全部六个入口、参考路径与关键失败分支，实际运行了三个分析CLI和模型生成器URDF正例。验证记录见[本轮报告](../outputs/skills/all-skills-portability-validation.md)。本轮未在不同厂商或不同规模模型上逐一测试，不能保证所有低能力模型表现一致。实际计算依赖工具和完整输入；文档优化不能替代缺失的运行环境，也不赋予额外工程验收能力。
