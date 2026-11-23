---
name: arm-sim-model-generator
description: 根据机械臂运动学与初步设计参数自动生成逐连杆视觉/碰撞 Mesh、机械臂预览 PNG、独立 URDF、ROS1 RViz 描述包、MuJoCo MJCF 和 Isaac Sim USD。支持固定基座单条串联 revolute/prismatic/fixed 刚体链及静力学/工作空间输入适配；用于概念建模与仿真模型交付，不处理闭链、柔性或网页可视化。
---

# 机械臂仿真模型生成

1. **建立输入。** 按 [输入契约](references/input-contract.md) 将用户数据填入 [JSONC 模板](assets/worksheet.jsonc)。先核对轴、安装变换、类型和连接关系。标准输入和现有静力学/工作空间适配器均从同一套 SI 模型生成几何与导出文件。需要参考成熟构型、关节方向、惯量参考系或碰撞代理时，查 [经典机械臂参考](references/reference-arms.md)，保留型号和来源。新臂的额定力矩、几何尺寸、质量和惯量使用本设计的依据。
2. **建立几何与驱动。** 默认使用 rounded-shell-v2 生成外壳和独立碰撞体。按 [外形规则](references/appearance.md) 调尺寸，共点旋转轴另读 [模组布置](references/joint-layout.md)。保留关节 origin、局部轴和质量归属。MuJoCo/USD 共用 [驱动参数规则](references/drives.md)，按已知能力和质量属性补全增益，尊重用户给定值。概念模式使用的估算须在输入与报告中注明。
3. **生成并复核。** Python ≥3.10，安装 `scripts/requirements.txt`。在本 skill 目录运行下面命令。首条是可直接换输出路径的正例。将输入替换为实际设计并按需选择目标；有 MuJoCo/OpenUSD 时可按 `scripts/requirements-validation.txt` 准备运行环境并追加 `--runtime`。

```sh
python scripts/generate.py generate --input assets/two-revolute-driven.json --output output --targets urdf mujoco
python scripts/generate.py validate --output output
python -m unittest discover -s scripts/tests -p test_collision_review.py -v
```

4. **判断结果。** 读取退出码和 `output/latest_attempt.json` 指向的实际 run。查 `report/manifest.json` 的逐目标状态，再查 `report/validation.json` 的 `collision_review`、`module_packing` 和运行时检查。按 [碰撞判读和修复](references/collision-review.md) 定位零件对、姿态和允许配合。有碰撞失败时回到相应的结构或输入参数；遇到待复核候选时说明范围并检查证据。深度、载荷或误差均须保留单位和来源。
5. **交付。** 链接同一 run 的模型、初始姿态 `preview.png`、归一化输入、驱动来源及报告。零位和初始姿态分别保存；MuJoCo 通过交付目录的 `mujoco/run.py` 恢复 keyframe，USD 搬移时保留零位层和配置。按 [各目标验证](references/validation.md) 列出已检查和仍待目标环境验证的项目。下游任务从 run 接入，碰撞体名称和配合范围见 `collision_review.parts`、`mating_pairs`。

[斜轴滑台](assets/skew-prismatic.json) 和 [七轴驱动](assets/seven-axis-driven.json) 保留原始设计，可用于复现“驱动或格式检查通过，零件级碰撞验收失败”。两者的已知干涉见 [碰撞样例](references/collision-review.md)。

可在 `scripts/` 导入 `arm_sim.pipeline.generate(input_path, output, targets=None, runtime=False)`、`validate(output, runtime=False)`。URDF/ROS1 可在缺少动力学属性时交付可视模型；MJCF 动力学等待质量和惯量补齐。世界安装仅应用一次，旋转使用 `Rz Ry Rx`，关节轴在局部关节系。负载以独立质量或已计入部件标注。
