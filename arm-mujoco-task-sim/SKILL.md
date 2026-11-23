---
name: arm-mujoco-task-sim
description: 接收 arm-sim-model-generator 输出的机械臂模型和自然语言任务描述，经用户确认仿真方式、策略、环境参数与轮次后，生成任务资产、搭建 MuJoCo 场景、实现规划或用户指定控制策略，实时可视化并导出关节、末端与载荷时序数据、曲线和报告。适用于抓取放置、擦拭、推压等机械臂任务仿真与设计载荷分析。
---

# 机械臂任务仿真与载荷导出

要直接试用选型表七轴或 Franka FR3 的三轮桌面抓放，运行[双模型红蓝分拣示例](references/dual-sorting.md)。

输入为模型生成器的一次实际输出和任务描述。命令中的 `<skill>`、`<project>` 替换为实际路径，同一项目使用同一已验证的 Python。

| 步骤 | 执行与继续条件 |
|---|---|
| 1. 接入机械臂 | `python <skill>/scripts/inspect_generator.py <模型输出或run> --out <project>/model_inspection.json`。检查入口、哈希、关节和初始 keyframe。读上游零件级碰撞结论与待复核姿态，保留已知质量、惯量、能力和名称。旧模型会标记碰撞未评估。 |
| 2. 确认任务与路线 | 按[任务确认项](references/intake.md)复用用户已有答复。核对目标、成功判据、查看模式、策略、环境与工具参数、起止条件、节拍、轮数、轮间复位和输出位置。对缺项一次提问。明确指定的方法直接落实；路线有实质差异时按[策略选择与提问示例](references/scene-strategy.md)给出候选并记录用户选择。 |
| 3. 定好运行环境 | 先用已有解释器执行 `python <skill>/scripts/environment.py probe --mode web`（或 native、headless）。通过即复用。Ubuntu 20.04/22.04/24.04 x86-64 缺环境时用同一条离线命令 `python3 <skill>/scripts/install_ubuntu.py --mode web --venv .venv`。Windows 及其他平台见[离线安装与版本指南](references/install-api.md)。Web 与本地原生窗口共用 Python 后端；无窗口批量试验需获得相应授权。 |
| 4. 生成场景和控制器 | 从[合同模板](assets/task_contract.template.json)填入参数、来源和真实确认证据。写入资产清单、策略和[控制适配器](references/adapter.md)。按[碰撞合同](references/collision.md)列出各阶段的接触对象、穿透/力阈值来源和轨迹检查方式。保留视觉、碰撞、动力学与传感器各自的几何和数据来源。 |
| 5. 预检与短试跑 | 运行下方预检命令，读本次 `reports/preflight.json`。按证据修复零件、摆位、路径或阈值来源；修改后重新检查。执行有代表性的接近、接触和退出，检查速度、饱和、抓稳/滑移、接触力及失败处理。[验证方法](references/validation.md) |
| 6. 正式仿真与交付 | 按确认轮数运行，检查 `reports/result.json`、碰撞诊断、逐轮状态及原始时序。按[输出规范](references/outputs.md)交付 CSV、含限值的 PNG 曲线、失败证据和实际 run 路径。用实际场景验证播放、暂停、恢复、Reset 后再播放、退出和端口释放，再交付启动命令。 |

```text
python <skill>/scripts/contracts.py <project>/task_contract.json
python <skill>/scripts/run_task.py --project <project> --preflight
python <skill>/scripts/run_task.py --project <project>
python <skill>/scripts/analyze.py <project>/runs/<run-id>
```

只有用户答复或明确委托的估算才能写入合同。未确认时可准备草案，依赖未确认参数的正式仿真须等待。控制器按物理时钟运行，复位建立新的时间段或 run；指定视觉策略时记录实际图像观测、估计和控制证据。失败轮次保留，涉及任务含义、质量、摩擦、工具或驱动能力变化时重新确认。

记录器支持固定基座串联 hinge/slide，并可与夹爪及带 freejoint 的物体同场。机器人自身有球/自由关节、闭链、柔性或自定义积分时需扩展并验证。关节反力、驱动力、末端接触力分别记录；换算电机载荷或寿命还需传动及材料等参数。

验证其他 agent 的使用效果时用[同题回归与评分](references/agent-benchmark.md)，保留模型标识、用户答复和实际输出。
