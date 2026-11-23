# 两台七轴机械臂的桌面分拣示例

在任务仿真 skill 目录运行下列命令。解释器需通过本 skill 的 MuJoCo 环境探测。两条命令各自生成项目和一次新的三轮物理仿真。每轮四件自由运动的物体，由夹垫摩擦提起。红色圆罐和阶梯块放左盘，蓝色方块和双柱组合件放右盘。

```sh
python scripts/dual_sorting.py --arm worksheet --project ./sort-worksheet
python scripts/dual_sorting.py --arm fr3 --project ./sort-fr3
```

只搭场景时添加 `--prepare-only`。启动实时网页后按播放，暂停、复位和重新播放使用同一套控制与逐步碰撞检查。每次复位都保留独立运行记录。两台并行时分别用不同端口。

```sh
python scripts/dual_sorting_web.py --project ./sort-worksheet --port 8911 --open-browser
python scripts/dual_sorting_web.py --project ./sort-fr3 --port 8912 --open-browser
```

### 两个模型和场景

选型表七轴以原安装坐标的侧装机械臂工作，桌面在基座原点下方。FR3 使用厂商描述的关节运动学、质量和质心惯量，以及经过本项目调整的概念外壳。两台均使用生成器完整导出、原始驱动限幅和重力，再在工具法兰上增加 220 g 夹具和两根各 35 g 的手指。夹具转接方向及四件 45–72 g 工件的几何、颜色、尺寸、摆位和抓握参数见生成项目的 `assets/manifest.json`。安装高度和桌面中心按各臂的工作空间分别设置。

内置压缩包保存原生成模型、网格及源清单，准备时检查 SHA256。独立零件检查使用同版生成器检查实现的便携副本。FR3 参数来自固定版本的 Franka ROS 描述，惯量为其公共参数；这两套七轴壳体均是概念外观。要校验已有生成结果，可给准备命令增加 `--model <该参考模型的robot.xml>`。[模型指纹](../assets/dual-sorting/models.json) · [FR3 来源许可证](../assets/dual-sorting/FR3-UPSTREAM-LICENSE.txt)

### 验收和输出

- 每轮按固定种子重新排列四件工件，轻微改变位置和朝向。控制器使用实测位姿、数值逆解、平滑轨迹和原位置驱动。夹持和运输通过力与接触验证。每侧夹垫须在运输各阶段建立接触，允许的连续失载时间为 6 ms。每件提升至少 80 mm，搬运滑移须小于 30 mm。释放高度按当时夹持姿态及工件最低表面计算。
- 逐物体及轮末验收包含正确托盘底接触、中心处于托盘有效内部、平动速度低于 0.025 m/s、角速度低于 0.35 rad/s。检查预期外接触、夹垫和支撑面嵌入，以及每个实际可用的路径段。独立检查读取记录的每个物理步，并按表面移动上界补插值，重新核对包括引擎排除项在内的所有异连杆碰撞代理。
- 在 `runs/<id>/reports/result.json` 查整体状态和每轮每件证据。在 `reports/arm_path_collision.json` 查独立复核的姿态数、范围、模型及轨迹哈希。该结论覆盖本次实际运行的轨迹，两个概念模型在全关节范围均有已知干涉。
- `runs/<id>/input/` 保存场景、代码、资产和任务合同快照。`data/arm_path.csv` 保存每个物理步的七轴关节角；`data/replay.jsonl` 约 20 Hz 并在阶段切换时加帧，每帧包含 `time_s`、`cycle`、`phase` 和全量 `qpos`（含夹爪与四件自由物体）。`data/joints.csv`、`end_effectors.csv`、`contacts.csv` 提供同一时刻的动力学和接触时序，`layouts.json` 保存三轮摆位。可用 `python scripts/analyze.py <项目>/runs/<id>` 输出曲线和逐轴统计。
