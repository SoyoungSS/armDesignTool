---
name: arm-sim-model-generator
description: 根据机械臂运动学与初步设计参数自动生成逐连杆视觉/碰撞 Mesh、机械臂预览 PNG、独立 URDF、ROS1 RViz 描述包、MuJoCo MJCF 和 Isaac Sim USD。支持固定基座单条串联 revolute/prismatic/fixed 刚体链及静力学/工作空间输入适配；用于概念建模与仿真模型交付，不处理闭链、柔性或网页可视化。
---

# 机械臂仿真模型生成

1. **建立输入。** 按 [输入契约](references/input-contract.md) 将用户数据填入 [JSONC 模板](assets/worksheet.jsonc)。先核对轴、安装变换、类型和连接关系。标准输入和现有静力学/工作空间适配器均从同一套 SI 模型生成几何与导出文件。需要参考成熟构型、关节方向、惯量参考系或碰撞代理时，按构型选择 UR5e / UR10e、FR3、KUKA KR 6 或 xArm6，查 [经典机械臂参考](references/reference-arms.md)，保留型号和来源。新臂的额定力矩、几何尺寸、质量和惯量使用本设计的依据。
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

项目的两台默认七轴机械臂有独立输入。[原表格空载臂](assets/examples/defaults/worksheet.json)的运动部件共 3.612 kg。静力学的 3 kg 末端负载另有[独立工况](assets/examples/defaults/worksheet-with-3kg.json)。分拣场景使用空载输入，另建实际夹爪和自由物体。[FR3 空载参考臂](assets/examples/defaults/fr3.json)保留参考刚体惯量和轴系，工作模型在肘、腕为概念外壳留出转动空间。

在装有 MuJoCo 的环境中运行默认例子。每条命令都生成独立模型、预览和运行时检查，并复核一段指定运动。读取各自输出目录的 `default_acceptance.json`，其中 `nominal_motion_collision` 对应表格臂的零位至初始位，或 FR3 的初始位至偏置位。`full_range_collision` 单独记录原限位抽查。通用生成命令在宽范围内检出干涉时返回失败；此默认入口按声明的运动范围返回退出码。

```sh
python scripts/default_model.py --arm worksheet --output output/worksheet
python scripts/default_model.py --arm fr3 --output output/fr3
```

这两个默认例子的指定运动、原始限位干涉反例和生成器运行时检查，均已在 MuJoCo 3.3.7、4.0.1 实测。

两台默认臂的初始位通过零件复核，部分极限姿态仍有关节外壳干涉。评估限定任务时保留原始全范围报告。将同一模型实际运行的每个物理步写成含 `time_s,cycle,phase` 和所有活动关节名的 CSV，旋转值用 rad，移动值用 m。在 MuJoCo 环境运行下列命令，报告存到任务目录：

```sh
python scripts/check_path.py --run output --trajectory task/data/arm_path.csv --output task/reports/arm_path_collision.json
```

脚本核对模型资源指纹，按连杆运动距离细分相邻记录，再直接计算跨连杆碰撞体之间的距离。判定范围为本次记录的机械臂轨迹。工具、工件、桌面接触和抓持保持情况由任务场景同步验收。用分拣通过的结果报告对应轨迹，并保留模型报告中的极限位冲突。

可在 `scripts/` 导入 `arm_sim.pipeline.generate(input_path, output, targets=None, runtime=False)`、`validate(output, runtime=False)`。URDF/ROS1 可在缺少动力学属性时交付可视模型；MJCF 动力学等待质量和惯量补齐。世界安装仅应用一次，旋转使用 `Rz Ry Rx`，关节轴在局部关节系。负载以独立质量或已计入部件标注。
