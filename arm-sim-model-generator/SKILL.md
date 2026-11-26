---
name: arm-sim-model-generator
description: >-
  从机械臂参数生成连杆视觉/碰撞网格、预览、URDF、ROS1 RViz包、MuJoCo MJCF或Isaac Sim USD，并分别报告验证状态。用于固定基座单串联刚体链的参数建模、仿真模型交付与全关节载荷测点准备；不处理闭链、柔性或任务控制。
metadata:
  version: "1.3.0"
---

# 机械臂仿真模型生成

## Purpose

使用随附生成器交付可追溯的模型和逐目标验证结果，为通用任务仿真准备完整关节与载荷接口；不把文件生成等同于工程验收。

## When to use

需要参数建模、多格式导出、已有模型包复核或指定机械臂轨迹干涉检查时使用。任务工具/工件/桌面接触另由任务仿真验证。

## Inputs

普通生成先读[输入契约](references/input-contract.md)。需要请求文件、输出目录、目标格式和要求的运行验证范围。固定基座单串联revolute/prismatic/fixed链；几何、安装、限位、初始姿态、质量/COM、惯量及驱动的适用要求按目标核对。

| 用户目标 | 入口与资料 |
| --- | --- |
| 自定义参数或支持的原始静力学/工作空间请求 | 普通generate流程，输入契约 |
| 默认选型表臂/FR3 | [默认模型与轨迹](references/default-workflows.md)，default_model.py，需MuJoCo |
| 已有包复核 | validate，先定位真实run，不重猜输入 |
| 检查记录的机械臂运动 | 默认模型与轨迹中的check_path.py，不替代任务接触检查 |

## Workflow

### Step 1 核对请求

按[输入核对细则](references/operation-guide.md#2-确认输入保留设计依据)检查坐标、单位、质量归属、限位与驱动来源。安装只施加一次；初始关节值不折入origin。缺关键数据先列出；概念质量/惯量估算与驱动占位必须显式配置，不能为通过校验自动补参数。

公共arm-design-model、normalized_input和normalized_model不是本生成器请求；先显式映射。示例仅用于用户选择的演示。

### Step 2 确定目标与环境

CLI目标名仅urdf、ros1、mujoco、isaac；MJCF选mujoco，USD选isaac。命令显式给--targets，不默认生成全部。检查同一Python ≥3.10及numpy/PIL，依赖见scripts/requirements.txt；USD还需pxr，MuJoCo运行检查还需mujoco。缺环境读[环境与依赖](references/environment.md)，安装清单不作为已安装证据。

### Step 3 执行

工作目录为本技能根目录，python替换为已验证解释器。REQUEST与OUTPUT替换为本次实际路径；下面示例目标按实际请求修改。

```sh
python scripts/generate.py generate --input "REQUEST.json" --output "OUTPUT" --targets urdf mujoco
python scripts/generate.py validate --output "OUTPUT"
```

每条命令记录退出码与JSON输出；生成完成后先读OUTPUT/latest_attempt.json取得run_directory。存在顶层error、running或缺关键报告时按失败/未完成处理，不选旧成功run。要求MuJoCo运行检查且依赖具备时，两条命令均加--runtime；否则明确待验证。

### Step 4 分项判读并修复

读实际run的report/manifest.json、validation.json与load_handoff.json，按目标分别检查generated、static_checks、装配、碰撞、runtime_validation；载荷分析还按[全关节测点交接](references/load-handoff.md)核对活动/fixed连接、惯量、驱动与传感器；详细字段和失败处理见[验收判读](references/operation-guide.md#4-按证据判断不合并不同验收范围)。pending、needs_review、not_evaluated、缺失字段均不能写成通过。

退出0只表示流程完成。不能通过删除碰撞体、扩大排除、放宽容差或缩小验收范围掩盖失败；有依据的修复后重新生成，读取新run。缺修复依据时保留诊断并说明缺项。

### Step 5 交付

交付同一次run。模型资源整体搬移，逐目标说明已生成、通过、失败及待验证范围。MJCF逐原始关节生成force/torque传感器和load_handoff，含fixed连接；其余格式不宣称具备MuJoCo传感器。不要将任何分拣工具或场景烘入机械臂本体。目标环境验收时再读[验证方法](references/validation.md)。

## Output format

1. 输入与环境：请求、来源、估算、解释器/依赖、命令与退出码。
2. 产物：run_directory、真实模型入口、资源包和preview.png；预览图不是仿真运行截图。
3. 验证表：每目标生成、静态、装配、碰撞、运行状态及原因。
4. 报告：同run的input_snapshot、normalized_model、manifest、validation、load_handoff和report.md；链接实际存在文件，保留范围与待办。

## Rules

参考臂额定值不是新臂能力，静力学保持载荷也不是驱动限值。有限姿态/轨迹通过不证明全空间无干涉。AABB重叠不等于穿透深度；生成、格式、碰撞和运行状态不能合并。MuJoCo验证不能替代ROS GUI或Isaac/PhysX验收。

## Examples

- “生成双轴URDF与MJCF”：使用[双旋转轴正例](assets/two-revolute-driven.json)试用；真实请求需替换输入，保留所选targets。
- “生成退出0但碰撞needs_review”：报告待复核，不能写成全部验收通过。

## Resource loading and fallback

核心执行需要文件读写与Python，不依赖特定模型API或网页；agents/openai.yaml如存在仅用于界面。可选安装包来自相邻技能，独立分发时可能没有，不能假定固定机器路径存在。

有文件/执行能力时逐步运行并保留请求、目标、run和状态。仅能读文件时核对输入并给命令，说明未执行；纯文本时由调用方提供契约与请求正文，缺资料就列缺项，不编造模型文件、截图或验收。小上下文时一次处理一个目标，保留同一输入和run后再汇总。

调整外壳读[外形](references/appearance.md)，共点轴读[模组布置](references/joint-layout.md)，驱动读[驱动规则](references/drives.md)，借鉴轴系/惯量读[参考臂](references/reference-arms.md)，碰撞解释读[碰撞判读](references/collision-review.md)；不默认加载所有参考。
