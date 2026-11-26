---
name: arm-kinematics-dynamics
description: >-
  为参数化串联机械臂生成或导入关节轨迹，分析TCP状态、运动约束、奇异性和可选刚体逆动力学载荷。用于轨迹状态与动态载荷分析、可复现随机关节轨迹生成；不替代碰撞、接触任务仿真、热或寿命验收。
metadata:
  version: "1.2.0"
---

# 轨迹状态、奇异性与动态载荷分析

## Purpose

使用随附轨迹与刚体动力学内核，输出可复现的时序及有范围的可行性结论。

## When to use

用户需要关节轨迹、TCP状态、奇异性或已知运动的动态载荷时使用。笛卡尔路径规划、碰撞规避和接触求解不属于本入口。

## Inputs

先读[请求契约](references/input-contract.md)中所选模式。请求为严格JSON，不将JSONC传入此CLI。公共design_model的字段见[模型契约](../shared/contracts/design-model.schema.json)，含完整结构的示例见[FR3请求](assets/examples/defaults/fr3.json)。SI单位；主动关节数组顺序必须明确，外部轨迹用joint_ids映射。

| 任务 | 请求与关键数据 |
| --- | --- |
| 关键点生成 | arm-motion-request 1.0；design_model、waypoints、motion_limits、时长/采样间隔 |
| 导入轨迹 | 同上；samples、时间和joint_ids，明确导数是否提供/允许估计 |
| 随机生成一批关节轨迹 | arm-random-motion-request 1.0；template与settings，生成后逐条分析 |
| 动态载荷 | motion请求加dynamics.enabled=true；质量、COM、惯量、重力、完整q/dq/ddq及正质量负载惯量 |

仅运动学不要求惯量。缺动力学参数时说明影响，可在用户要求运动学时完成该部分，不能擅自填演示球惯量。

## Workflow

### Step 1 选择模式与分析范围

确认生成、导入或随机批次；确认是否计算动态载荷。保留模型、单位、坐标、关节ID和TCP定义。缺关键输入时集中列出，不默认使用FR3、3kg或演示运动限制。

### Step 2 建立请求

按契约检查限位、时间、导数和运动限制。生成模式的strict只诊断时长不足，stretch会延长时长，保留用户选择。外部轨迹不自动补导数，仅derivatives=estimate才估计并记录噪声风险。动态输入再读[动态载荷](references/operation-guide.md#动态载荷)。

### Step 3 执行

工作目录为本技能根目录；确认Node.js ≥20及同级shared。将REQUEST和RUN替换为本次实际路径，RUN为新输出目录。

```sh
node scripts/run.mjs --input "REQUEST.json" --output "RUN/result.json" --report-dir "RUN"
```

随机生成请求**不传--report-dir**，命令如下（GENERATION替换为批次请求）：

```sh
node scripts/run.mjs --input "GENERATION.json" --output "RUN/batch.json"
```

取trajectories条目内的request分别保存，再以上述分析命令计算。记录退出码和stdout/stderr；失败按path修复，不使用旧结果冒充本次结果。

### Step 4 判读约束和奇异性

读取summary、scope、warnings、events。trajectory_feasible=false表示已有约束违规；true仅表示生成轨迹的声明运动约束解析检查通过；null表示证据不足。CLI成功不等于轨迹可行或任务完成。

奇异性按position/pose任务及声明尺度和阈值解释；task_underactuated与秩下降不同。样本近奇异不代表轨迹必然不可执行；没有期望末端运动不能宣称跟踪成功。外部轨迹即使样本通过也不认证点间安全。

### Step 5 交付与动态统计

动态载荷报告峰值、时间加权RMS、输出端机械功率和接口载荷；需要峰值收敛判断时减小采样间隔比较，不将未比较写成收敛。具体坐标/算法和结果解释见[操作细则](references/operation-guide.md)。

## Output format

1. 输入与假设：模型、轨迹模式、关节映射、TCP、限制/惯量来源及估计项。
2. 结果：运动约束状态、TCP与奇异区间；启用动力学才输出动态峰值/RMS。
3. 证据：命令、退出码、哈希及result.json；分析模式的trajectory.csv、report.md、request.json，动力学实际启用时的dynamic-loads.csv。随机批次仅列实际生成文件。
4. 限制与后续：已检查范围、未知项、未执行验证。

## Rules

不复制另一套FK/动力学公式，由随包内核计算。不做笛卡尔路径IK、碰撞、柔性、热或接触求解。动力学外载荷作用点是物理tip，TCP偏移仅改变观察点。模型支持范围、惯量系和载荷符号以契约为准。主JSON原子替换，失败后仍需防止配套旧文件混用。

## Examples

- “比较选型表臂与FR3往返轨迹”：读[双默认示例](references/operation-guide.md#双默认示例)，演示速度/加速度不是厂家能力。
- “只有位置采样，直接算动力矩”：先补齐导数或取得估计依据及惯量等参数；不能把缺失值填0。

## Resource loading and fallback

需要 Node.js ≥20、文件读写及随包 `shared/`；不要求网页、URDF、联网或特定模型API。`agents/openai.yaml` 如存在仅用于界面。独立分发必须保留技能与 shared 同级，详见[操作细则](references/operation-guide.md)。

- 能读文件并执行：按Workflow逐步运行，每步保留输入路径、输出路径、退出码和待处理项。
- 能读文件但不能执行：核对请求，提供可运行命令和待验证项，明确“未执行”，不给计算结果。
- 只有文本输入：由调用方提供本入口、相关契约和请求正文；缺资料时列出所缺字段，不猜字段、不假称读取文件。
- 上下文较小：先完成输入核对，再读取当前模式的契约；记录关节ID、单位、坐标、请求路径和检查进度后继续，不一次载入所有示例。
