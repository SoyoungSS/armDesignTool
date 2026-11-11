---
name: arm-sim-model-generator
description: 根据机械臂运动学与初步设计参数自动生成逐连杆视觉/碰撞 Mesh、独立 URDF、ROS1 RViz 描述包、MuJoCo MJCF 和 Isaac Sim USD。支持固定基座单条串联 revolute/prismatic/fixed 刚体链及静力学/工作空间输入适配；用于概念建模与仿真模型交付，不处理闭链、柔性或网页可视化。
---

# 机械臂仿真模型生成

先读 [输入契约](references/input-contract.md)。从设计表或文字提取结构化参数，缺少不可推导的轴、安装变换、类型、连接关系时列出字段，不能套用样例构型。现有两个机械臂 Skill 的接口保持独立；此处适配器只读取它们的输入，不修改其源码。

## 执行

Python ≥3.10；核心只依赖 NumPy，不依赖 ROS、MuJoCo、Isaac。安装 `scripts/requirements.txt`；可选运行检查安装 `scripts/requirements-validation.txt`。在本 Skill 目录运行：

```powershell
python scripts/generate.py generate --input assets/two-revolute.json --output /absolute/output --targets urdf
python scripts/generate.py generate --input assets/skew-prismatic.json --output /absolute/output --runtime
python scripts/generate.py validate --output /absolute/output --runtime
python -m unittest discover -s scripts/tests -v
```

可填写 [JSONC 模板](assets/worksheet.jsonc)，或用纯运动学 [双旋转轴](assets/two-revolute.json) 与含斜轴、移动轴、固定末端、偏心质心的 [完整样例](assets/skew-prismatic.json)。`--targets urdf ros1 mujoco isaac` 覆盖输入选择；省略则读取 `export.targets`。

[七轴驱动样例](assets/seven-axis-driven.json) 提供整链质量属性、力矩/速度上限、初始和验收姿态，用于检查耦合驱动和重力静差；不代表特定硬件已标定的控制器。

默认采用 `rounded-shell-v2` 外形规则：整机比例 → 连续外壳 → 关节模组布置 → 曲面和材质 → 少量端盖/法兰细节。全部由现有 NumPy 后端生成。需要调整外观时读 [网格与外形规则](references/appearance.md)，优先填写已有 dimensions；不改变关节安装点迎合造型。

相邻旋转关节共点时必须读取 [关节模组布置规则](references/joint-layout.md)：关节轴线交点不等于电机中心。自动识别自转—摆动/同轴组合，沿轴退让固定模组，输出套筒归运动连杆；禁止移动关节 origin 来消除电机重叠。检查 joint_layout.json 和独立于碰撞过滤的 module_packing；无法自动布置的组合保留 needs_review，不得声称机械装配通过。

生成仿真目标时阅读 [驱动参数规则](references/drives.md)。活动关节默认 `auto` 位置伺服；脚本按完整质量属性、力/矩与速度上限计算缺失增益，禁止每次凭经验另选一组参数。用户显式增益优先；`passive` 明确关闭该轴驱动。缺物理或驱动能力时报告缺项，不编造额定值。

每次生成写入新的 `output/runs/<run_id>/`；`latest_attempt.json` 指向本次结果，包括失败结果。必须检查退出码及逐目标状态，不能把旧运行当本次成功。交付时链接实际 run 目录中的模型与报告。`validate` 可接收 output 或具体 run 目录；它核对文件清单和哈希，重新加载资源及位姿。

## 必须保持的约定

- 顺序：读取校验 → SI 归一化 → 零位装配 → 逐连杆网格 → 质量属性 → 导出 → 重加载验证。所有导出复用同一模型和网格，禁止分别重建运动学。
- 右手系，RPY=`Rz Ry Rx`；关节运动在 origin 后，轴在局部关节系。世界安装只应用一次；初始关节值不写回几何零位。姿态使用关节名称。
- Mesh 自动生成，在连杆局部系使用 m。颜色由格式材质表达。碰撞体是单独派生的盒体/圆柱；封闭 STL 也随包提供。不得把装饰件、重叠体积计入密度估算。
- 质量、COM 和 COM 处惯量优先采用给定值。固定端归上游、运动端归下游。随动负载必须确认独立质量或指向已计入部件；不自动重复相加。
- 严格模式缺质量/惯量时仍可交付可视化模型，并逐连杆报告缺项；MJCF 动力学交付待补齐。缺 URDF effort/velocity 则该目标失败。概念模式只有显式配置占位值/估算方法时才使用，并逐项记录。
- 干涉结果是保守候选及有限姿态筛查，不是全域无碰撞证明。模型不得声称 CAD 精度、控制器已调好或硬件安全。

## 目标与验证

各目录搬移时整体携带资源；独立 URDF 目录只能有 URDF 和 meshes。ROS1 默认 Noetic，使用 `package://`、catkin 包、`display.launch`、RViz RobotModel 和关节滑块；`gui:=false` 切换无 GUI 发布器。

MuJoCo 与 USD 共用 `serial-pd-v1` 计算的 SI 驱动参数，均写入力/矩上限。MuJoCo 工具为 site，初始 q/ctrl/滤波状态为 keyframe；使用输出目录的 `mujoco/run.py` 启动，避免零状态突跳。USD `robot.usd` 在初始姿态，引用 `robot_zero.usd` 保存零位和关节框架；搬移时携带两层及 runtime_config.json。角驱动参数转换为 USD 的每度单位。

Isaac 环境可用时优先按目标版本运行官方导入器，保留 fixed joints 与名称、核对安装及工具帧；否则调用内置 OpenUSD 离线导出。有 USD 不能视为 Isaac 运行通过。版本差异、正式导入与实机步骤见 [导出与验证](references/validation.md)。

读取 `report/manifest.json`、`drive_design.json` 和 `validation.json`，按目标分别说明生成、静态检查、运行时检查、待验证或失败。检查接触开启时的保持/小幅运动尾段振幅、速度、饱和及重力静差；测试失败不能用关闭碰撞或增加虚假惯量掩盖。有限姿态通过不能称为全域无抖动。ROS/RViz 还需构建和滑块检查。最后交付模型、归一化输入、质量/尺寸及驱动来源、验证结果以及待执行步骤。

复用脚本：把 `scripts/` 加入 Python 路径，导入 `arm_sim.pipeline.generate(input_path, output, targets=None, runtime=False)`、`validate(output, runtime=False)`；底层 `normalize`、`build_geometry`、`mass_properties` 可分别调用。网页版不属本期。
