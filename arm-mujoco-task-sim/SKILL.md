---
name: arm-mujoco-task-sim
description: >-
  接收实际机械臂模型和任务描述，在已有授权及确认范围内搭建MuJoCo场景、实现控制策略、执行任务并导出时序、曲线与结果证据。用于抓放、擦拭、推压等接触任务和载荷分析；不把未执行的仿真、任务成功或寿命写成通过。
metadata:
  version: "1.2.0"
---

# 机械臂任务仿真与载荷导出

## Purpose

把明确的机械臂任务实现为可复现的MuJoCo场景和控制流程，交付逐轮任务证据及实际时序。

## When to use

需要模型与工具/物体/环境接触、控制执行、可视化和记录时使用。只需静态载荷或已知轨迹逆动力学时，不必搭任务仿真。

## Inputs

需要完整生成器模型输出（或其中XML连同报告）、任务目标与成功判据、工具/环境物理参数、策略、查看模式、起止与轮次、输出目录。先读[任务确认项](references/intake.md)，复用已有答复；缺项按影响集中询问，不重问已授权事项。

现有接入脚本要求生成器manifest和资源清单。独立外部XML缺少这些报告时，先说明需补完整run或另行实现导入与验证，不能伪造报告或宣称接入已通过。已有质量或工具不得重复添加。当前记录器支持固定基座串联hinge/slide，可同场加入夹爪和freejoint物体；机器人闭链、球/自由关节、柔性或自定义积分需扩展验证。

## Workflow

### Step 1 检查模型

先执行下方inspect_generator，读model_inspection.json、上游manifest与碰撞/惯量/驱动证据。根目录按latest_attempt定位本次尝试，失败不能偷用旧run。初始keyframe不等于全零；安装只施加一次。

### Step 2 确定合同与策略

按[策略选择](references/scene-strategy.md)保留用户指定方法；有实质选择且未确定时才询问。填写[合同模板](assets/task_contract.template.json)及真实用户确认依据。缺项可先写草案；仅影响这些缺项的正式仿真等待补齐，已确认的内容不重复审批。轮数没有默认值，不自动套三轮。

### Step 3 核对环境

用同一解释器运行environment.py probe，mode按已选web/native/headless替换。通过就复用；缺环境读[安装指南](references/install-api.md)与[离线依赖](references/offline-dependencies.md)，包清单不代表当前已安装。没有用户授权的无窗口试验不能替代所选查看方式。

### Step 4 建立场景并预检

按[适配器](references/adapter.md)实现控制器和观测，按[碰撞合同](references/collision.md)登记阶段接触对象及阈值来源。视觉、碰撞、动力学、传感器的几何和来源分别保留。运行contracts与preflight，读取本次reports/preflight.json；不通过时按证据修复再检查。

### Step 5 短试跑与正式运行

按[验证方法](references/validation.md)检查接近、接触、退出、速度、饱和、抓稳/滑移和失败处理，范围取已有授权。正式按合同轮数运行，失败轮保留，不静默加轮数或改策略/物理参数。控制器用物理时钟，复位建立新时间段或run。

### Step 6 验收与交付

读取实际run的result.json、逐轮状态和碰撞诊断，再执行analyze。按[输出规范](references/outputs.md)交付数据、含限值的曲线和失败证据。实际查看模式验证播放、暂停、恢复、Reset后播放、退出和端口释放；无GUI能力则明确此项未验证。

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

1. 任务与模型：原要求、成功判据、合同/确认依据、策略、物理参数和来源。
2. 执行：解释器、实际模式、命令/退出码、run路径、计划与实际轮数。
3. 结果：逐轮成功/失败/未完成、碰撞与限值证据、曲线和原始CSV/metadata链接。
4. 复现与范围：启动/复跑命令、实际查看器验证、未执行检查、implementation.md及后续可用范围。

## Rules

非零退出或缺本次报告时停止依赖该结果的后续步骤，保留诊断，不用旧结果替代。任务含义、质量、摩擦、工具或驱动能力变化需确认对应变化；沿用已有授权。确认依据不得自行伪造。

qfrc_constraint不能直接叫关节六维反力或轴承载荷；驱动广义力、执行器力、接触wrench需分开解释，坐标/作用点见输出规范。有限轮次成功不证明寿命、实机能力或全工作空间安全。

## Examples

- “试用两台默认臂分拣”：读[双模型分拣](references/dual-sorting.md)，只有用户选择演示时才沿用其参数与轮次。
- “模型有了，帮我擦桌”：先提取缺失工具、摩擦、接触阈值和成功判据，可准备草案，不能编造确认后开跑。

## Resource loading and fallback

需要文件读写、Python与所选模式的MuJoCo依赖；不依赖模型供应商或固定本机目录。agents/openai.yaml如存在仅供界面。详细历史流程和离线库存见[操作细则](references/operation-guide.md)，性能评估才读[同题回归](references/agent-benchmark.md)。

仅能读文件时完成检查/合同草案并给命令，标记未执行；纯文本时由调用方提供合同/模型摘要与相关参考正文，只整理方案和缺项，不伪造仿真数值、资产或截图。小上下文按阶段读取资料，保留项目路径、模型哈希、合同版本、当前run与待办后继续。依赖缺失时可交付准备材料，不能宣称任务仿真完成。
