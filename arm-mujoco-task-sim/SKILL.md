---
name: arm-mujoco-task-sim
description: >-
  依据实际机械臂模型与通用工况搭建MuJoCo场景，执行运动和接触控制，导出全关节驱动力矩/力、六维接口载荷时序与时间驻留谱。用于循环运动、带载保持、抓放、擦拭、推压、插装等任务的载荷分析；不以分拣模板限制场景，不替代疲劳寿命验收。
metadata:
  version: "1.5.0"
---

# 机械臂通用场景仿真与全关节载荷谱

## Purpose

把给定工作场景或作业循环实现为可复现的MuJoCo物理执行，交付全关节载荷谱、完整带符号时序与任务证据。默认前期概念试算：用户只需描述场景，Agent 补齐常规假设、优先运行，用户再通过描述迭代。明确要求精确参数/验收时切换严格模式。任务定义独立于记录器，分拣仅为示例。

## When to use

需要在自由运动、带载保持、工序循环或工具/物体/环境接触中模拟实际控制响应并记录载荷时使用。单次操作和混合工序同样适用。只需解析静态载荷或已知轨迹逆动力学时，转相应分析技能。

## Inputs

需要可读取的完整生成器模型输出（或其中XML连同报告）、场景描述和输出目录。其他工况、判据、工具/环境参数、策略、轮数与时长，在概念模式按[常规试算与默认假设](references/concept-trial.md)由 Agent 补齐并记录，不因普通缺项反复提问或阻止运行。自由运动不添加无关工件。精确模式再按[任务确认项](references/intake.md)集中补充关键问题。

现有接入脚本要求生成器manifest和资源清单。独立外部XML缺少这些报告时，先说明需补完整run或另行实现导入与验证，不能伪造报告或宣称接入已通过。已有质量或工具不得重复添加。当前记录器支持固定基座串联hinge/slide，可同场加入夹爪和freejoint物体；机器人闭链、球/自由关节、柔性或自定义积分需扩展验证。

## Workflow

### Step 1 检查模型

先执行下方inspect_generator，读model_inspection.json、上游manifest与碰撞/惯量/驱动证据。根目录按latest_attempt定位本次尝试，失败不能偷用旧run。初始keyframe不等于全零；安装只施加一次。读取load_handoff，按[全关节覆盖与谱约定](references/load-spectrum.md)核对活动DOF与固定连接、驱动和接口测点；新项目robot_joints用"all"，不把场景freejoint混入机械臂。旧包缺测点时明确六维载荷不可用。

### Step 2 确定合同与策略

按[策略选择](references/scene-strategy.md)保留用户指定方法。概念模式读取[默认假设](references/concept-trial.md)，自行补齐轮数、节拍、物理参数、策略和判据后直接实现；只对真实执行阻塞停止，普通待补信息仅作后续修改建议。填写[合同模板](assets/task_contract.template.json)，把概念委托和实际假设记入 assumption_authorization/assumptions，不伪造用户逐项确认。严格模式等待关键缺项补齐和方案审阅。载荷工况均定义运动、停留、接触及异常退出。

### Step 3 核对环境

用同一解释器运行environment.py probe，mode按已选web/native/headless替换。通过就复用；缺环境读[安装指南](references/install-api.md)与[离线依赖](references/offline-dependencies.md)，包清单不代表当前已安装。没有用户授权的无窗口试验不能替代所选查看方式。

### Step 4 建立场景并预检

按[适配器](references/adapter.md)实现控制器和观测，按[碰撞合同](references/collision.md)登记阶段接触对象及阈值来源。视觉、碰撞、动力学、传感器的几何和来源分别保留。运行contracts与preflight，读取本次reports/preflight.json、load_coverage.json、goal_validation.json；不通过时按证据修复再检查。复杂放置/插装等成功判据采用[正负例自检](references/adapter.md#成功判据自检)；未提供时保留not_provided，不假称自检通过。

### Step 5 短试跑与正式运行

按[验证方法](references/validation.md)检查接近、接触、退出、速度、饱和、抓稳/滑移和失败处理，范围取已有授权。正式按合同轮数运行，失败轮保留，不静默加轮数或改策略/物理参数。控制器用物理时钟，复位建立新时间段或run。

### Step 6 验收与交付

读取实际run的result.json、逐轮状态和碰撞诊断，再执行analyze自动生成全关节载荷谱。任务失败或中止也保留已记录谱，标注截断，不将导出成功等同于任务成功。按[输出规范](references/outputs.md)交付一个内嵌全关节曲线的离线 HTML、每关节模组完整状态 CSV、全关节宽表、字段单位说明、含限值的曲线和失败证据。analyze 自动整理；已有运行可单独运行 joint_exports.py，无需重新仿真。实际查看模式验证播放、暂停、恢复、Reset后播放、退出和端口释放；无GUI能力则明确此项未验证。

命令模板：skill/project/model/run替换为实际路径，python为同一已验证解释器，示例web替换为所选模式。逐条检查退出码再继续；原始模型保持只读。

```sh
python "SKILL/scripts/inspect_generator.py" "MODEL" --out "PROJECT/model_inspection.json"
python "SKILL/scripts/environment.py" probe --mode web
python "SKILL/scripts/contracts.py" "PROJECT/task_contract.json"
python "SKILL/scripts/run_task.py" --project "PROJECT" --preflight
python "SKILL/scripts/run_task.py" --project "PROJECT"
python "SKILL/scripts/analyze.py" "PROJECT/runs/RUN-ID"
```

## Output format

1. 任务与模型：原要求、concept/strict 模式、task/load_proxy 范围、成功判据、委托/确认依据、策略、实际物理参数、假设和来源。
2. 执行：解释器、实际模式、命令/退出码、run路径、计划与实际轮数。
3. 结果：逐轮成功/失败/未完成、碰撞与限值证据；load_coverage、完整joints/joint_loads时序、load_statistics、load_spectrum、effort_speed_spectrum及metadata实际链接。载荷谱交付细则见[谱约定](references/load-spectrum.md)。
4. 复现与范围：启动/复跑命令、实际查看器验证、未执行检查、implementation.md及后续可用范围。

## Rules

非零退出或缺本次报告时停止依赖该结果的后续步骤，保留诊断，不用旧结果替代。概念模式可在已委托范围内调整未指定参数，并保留实际用值、理由与来源；用户明确指定的任务、质量、摩擦、工具或驱动限制不可擅自改动。精确模式的实质变更需要确认。确认依据不得自行伪造。

qfrc_constraint不能直接叫关节六维反力或轴承载荷；驱动广义力、执行器力、接触wrench需分开解释，坐标/作用点见输出规范。有限轮次成功不证明寿命、实机能力或全工作空间安全。接口六维载荷不等于轴承分担；时间驻留谱不等于雨流计数；没有传动和实际作业占比时不推算电机轴谱或全寿命。

## Examples

- “按工作循环导出所有关节载荷谱”：先读[通用工况与谱](references/load-spectrum.md)，按实际运动/停留/接触构建适配器；自由空间位置伺服可参考assets/workloads/joint_cycle.py，不引入分拣物体。

- “试用两台默认臂分拣”：读[双模型分拣](references/dual-sorting.md)，只有用户选择演示时才沿用其参数与轮次。
- “模型有了，帮我擦桌”：概念模式自动采用常规代理工具/接触假设或明确标记的载荷代理，先试算载荷；用户可再描述工具、力或速度修订。严格验收模式才先确认关键工具和接触参数。

## Resource loading and fallback

需要文件读写、Python与所选模式的MuJoCo依赖；不依赖模型供应商或固定本机目录。agents/openai.yaml如存在仅供界面。详细历史流程和离线库存见[操作细则](references/operation-guide.md)，性能评估才读[同题回归](references/agent-benchmark.md)。

仅能读文件时完成检查/合同草案并给命令，标记未执行；纯文本时由调用方提供合同/模型摘要与相关参考正文，只整理方案和缺项，不伪造仿真数值、资产或截图。小上下文按阶段读取资料，保留项目路径、模型哈希、合同版本、当前run与待办后继续。依赖缺失时可交付准备材料，不能宣称任务仿真完成。
