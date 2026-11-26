# 默认七轴模型与任务轨迹

以下命令均在 `arm-sim-model-generator/` 目录运行，`python` 指已验证依赖的解释器；路径与空格处理见 [入口](../SKILL.md)。仅在用户明确选择这两台默认臂时使用默认入口。

项目的两台默认七轴机械臂有独立输入。[原表格空载臂](../assets/examples/defaults/worksheet.json)的运动部件共 3.612 kg。静力学的 3 kg 末端负载另有[独立工况](../assets/examples/defaults/worksheet-with-3kg.json)。分拣场景使用空载输入，另建实际夹爪和自由物体。[FR3 空载参考臂](../assets/examples/defaults/fr3.json)保留参考刚体惯量和轴系，工作模型在肘、腕为概念外壳留出转动空间。

在装有 MuJoCo 的环境中运行默认例子。每条命令都生成独立模型、预览和运行时检查，并复核一段指定运动。读取各自输出目录的 `default_acceptance.json`，其中 `nominal_motion_collision` 对应表格臂的零位至初始位，或 FR3 的初始位至偏置位。`full_range_collision` 单独记录原限位抽查。通用生成命令在宽范围内检出干涉时返回失败；此默认入口按声明的运动范围返回退出码。

```sh
python scripts/default_model.py --arm worksheet --output output/worksheet
python scripts/default_model.py --arm fr3 --output output/fr3
```

这两个默认例子的指定运动、原始限位干涉反例和生成器运行时检查，历史验收记录为 MuJoCo 3.3.7、4.0.1 实测；当前环境仍需重新运行并读取本次报告。

两台默认臂的初始位通过零件复核，部分极限姿态仍有关节外壳干涉。评估限定任务时保留原始全范围报告。将同一模型实际运行的每个物理步写成含 `time_s,cycle,phase` 和所有活动关节名的 CSV，旋转值用 rad，移动值用 m。在 MuJoCo 环境运行下列命令，报告存到任务目录：

```sh
python scripts/check_path.py --run output --trajectory task/data/arm_path.csv --output task/reports/arm_path_collision.json
```

脚本核对模型资源指纹，按连杆运动距离细分相邻记录，再直接计算跨连杆碰撞体之间的距离。判定范围为本次记录的机械臂轨迹。工具、工件、桌面接触和抓持保持情况由任务场景同步验收。用分拣通过的结果报告对应轨迹，并保留模型报告中的极限位冲突。


## 限定范围的结果

`default_acceptance.json` 是默认入口的专项报告，不替代同一次 run 的 `report/manifest.json` 和 `report/validation.json`。分别报告指定运动验收和全范围采样冲突，即使专项命令返回 0 也保留后者。不要把默认入口的成功推广到自定义模型或任务轨迹。
