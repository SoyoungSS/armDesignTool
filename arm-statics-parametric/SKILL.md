---
name: arm-statics-parametric
description: >-
  对参数化串联机械臂计算自重、附件、偏心负载和已知外力产生的静态保持载荷，搜索大载荷姿态并评估已声明的静态驱动能力。用于概念设计、指定姿态校核与静态驱动初选；不替代动态、热保持、碰撞或结构验收。
metadata:
  version: "1.2.0"
---

# 参数化机械臂静力学校核

## Purpose

使用随附确定性内核输出逐轴保持载荷、最大已检查载荷及有条件的静态驱动需求，不由语言模型填写力矩结果。

## When to use

用户需要指定姿态保持载荷、关节范围搜索或静态驱动需求时使用。运动轨迹的惯性载荷不属于本技能。

## Inputs

先读[输入与坐标约定](references/input-contract.md)，按下表选择一种请求；不要混合字段。

| 输入 | 何时选择 | 按需读取 |
| --- | --- | --- |
| standard | 已有逐关节原点、局部轴、质量部件与COM | 输入契约 |
| simple | 仅有连杆长度、方向、质量等概念参数 | [精简输入输出](references/minimal-input-output.md) |
| arm-analysis-request 1.0 | 已有内嵌公共design_model | [公共输入](references/design-model.md) |

必须明确几何、限位、质量/COM、重力、负载质量和COM；无负载显式填0，缺质量不等于0。指定姿态按名称映射；搜索设置与驱动参数依用户要求填写。完整范围见契约。

## Workflow

### Step 1 核对输入

区分硬数据和概念估算，保留来源。重力向量在基座系，或使用installation给世界重力与安装RPY，两者二选一；不能从J1轴推断安装。模组固定端/运动端质量归属不同；安装变换只施加一次。

### Step 2 建立本次请求

按所选契约保存请求，保留原文件；“平举”需转成该轴系的明确姿态。缺关键几何、质量、COM或坐标时集中询问；中点COM等概念估算须显式记录warnings。构型选择记录和normalized_input不能直接当请求。

### Step 3 执行

工作目录为本技能根目录。确认 `node --version` ≥20 且同级shared存在。以下为命令模板：将REQUEST和RUN替换为本次实际路径，路径含空格仍保留引号；RUN使用新的输出目录。

```sh
node scripts/run.mjs --input "REQUEST.json" --output "RUN/result.json" --report-dir "RUN"
```

记录退出码和stdout/stderr。非零退出立即按错误中的path修复输入或环境；不读取旧result当本次成功，不通过改物理参数掩盖错误。主JSON完成后原子替换，但配套文件不是目录事务。

### Step 4 判读与复核

读取本次result的summary、warnings和哈希。按名称对应关节，旋转轴N·m、移动轴N。每轴临界姿态可单点重算；搜索最大值只称“最大已检查”，不称全局最坏值。设计系数只乘一次。

| 状态 | 结论 |
| --- | --- |
| not_sized | 缺驱动能力，不能判定选型 |
| capacity_conditions_unknown | 连续能力适用条件未知 |
| exceeds_capacity | 已检查需求超出声明能力 |
| satisfies_checked_cases | 声明条件下满足已检查工况 |

### Step 5 交付

使用下方结构交付。解释算法或搜索精度时读[验证边界](references/validation.md)；网页/API接入才读[集成接口](references/web-integration.md)。

## Output format

1. 输入与假设：模型/请求、单位、安装、载荷、估算和缺项。
2. 逐轴表：带符号保持载荷、最大绝对值、临界姿态、设计需求与驱动状态。
3. 执行证据：命令、退出码、实际result、joint_summary.csv、report.md、standard_request.json及输入哈希；仅链接本次存在的文件。
4. 范围与下一步：已检查姿态/搜索范围，以及未执行的验证。

## Rules

仅固定基座串联刚体链；闭链、柔性、未知接触反力、浮动基座与被动关节平衡不求解。fixed仅几何锁定，不是有限抱闸能力。静态结果不能证明连续平举30分钟、热、动态、碰撞或结构合格。重力静力学不需要惯性张量。

## Examples

- “看选型表七轴或FR3示例”：读[双默认示例](references/operation-guide.md#双默认示例)，显式选择模型；不要把演示3kg负载带入用户设计。
- “载荷算完了，能长期平举吗”：先核对驱动状态与连续条件，热保持仍待验证。

## Resource loading and fallback

需要 Node.js ≥20、文件读写及随包 `shared/`；不要求网页、URDF、联网或特定模型API。`agents/openai.yaml` 如存在仅用于界面。独立分发必须保留技能与 shared 同级，详见[操作细则](references/operation-guide.md)。

- 能读文件并执行：按Workflow逐步运行，每步保留输入路径、输出路径、退出码和待处理项。
- 能读文件但不能执行：核对请求，提供可运行命令和待验证项，明确“未执行”，不给计算结果。
- 只有文本输入：由调用方提供本入口、相关契约和请求正文；缺资料时列出所缺字段，不猜字段、不假称读取文件。
- 上下文较小：先完成输入核对，再读取当前模式的契约；记录关节ID、单位、坐标、请求路径和检查进度后继续，不一次载入所有示例。
