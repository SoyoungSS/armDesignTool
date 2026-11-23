# 经典机械臂参考库

五个参考包各含有可直接加载的 URDF、MJCF、USD、视觉网格和生成器输入。数字模型采用下列 ROS 描述的标称关节安装、限位、质量、质心及惯量。壳体和碰撞代理由本生成器按照连杆跨度和概念尺寸创建。用这些模型检查坐标约定、物理量级和格式一致性，应用到具体机器人前另行核对外壳边界、载荷及接触参数。

| 参考臂 | 可直接打开的模型 | 可重算的数据 |
|---|---|---|
| Universal Robots UR5e，串联六轴，输出 `tool0` | [URDF](../assets/reference-arms/ur5e/robot.urdf) · [MJCF](../assets/reference-arms/ur5e/robot.xml) · [USD](../assets/reference-arms/ur5e/robot.usd) | [逐轴输入](../assets/reference-arms/ur5e/input.json) · [两组数值基准](../assets/reference-arms/ur5e/expected.json) |
| Franka Research 3，冗余七轴，输出不含手爪的 `fr3_link8` | [URDF](../assets/reference-arms/franka_fr3/robot.urdf) · [MJCF](../assets/reference-arms/franka_fr3/robot.xml) · [USD](../assets/reference-arms/franka_fr3/robot.usd) | [逐轴输入](../assets/reference-arms/franka_fr3/input.json) · [两组数值基准](../assets/reference-arms/franka_fr3/expected.json) |
| KUKA KR 6 R900-2，工业六轴，观察肩肘载荷和偏置腕 | [URDF](../assets/reference-arms/kuka_kr6_r900_2/robot.urdf) · [MJCF](../assets/reference-arms/kuka_kr6_r900_2/robot.xml) · [USD](../assets/reference-arms/kuka_kr6_r900_2/robot.usd) | [逐轴输入](../assets/reference-arms/kuka_kr6_r900_2/input.json) · [数值基准](../assets/reference-arms/kuka_kr6_r900_2/expected.json) · [碰撞复核](../assets/reference-arms/kuka_kr6_r900_2/collision-review.json) |
| UFACTORY xArm6，紧凑六轴，观察两组共点轴的布置 | [URDF](../assets/reference-arms/ufactory_xarm6/robot.urdf) · [MJCF](../assets/reference-arms/ufactory_xarm6/robot.xml) · [USD](../assets/reference-arms/ufactory_xarm6/robot.usd) | [逐轴输入](../assets/reference-arms/ufactory_xarm6/input.json) · [数值基准](../assets/reference-arms/ufactory_xarm6/expected.json) · [碰撞复核](../assets/reference-arms/ufactory_xarm6/collision-review.json) |
| Universal Robots UR10e，长臂展六轴，可与 UR5e 同姿态对照 | [URDF](../assets/reference-arms/ur10e/robot.urdf) · [MJCF](../assets/reference-arms/ur10e/robot.xml) · [USD](../assets/reference-arms/ur10e/robot.usd) | [逐轴输入](../assets/reference-arms/ur10e/input.json) · [数值基准](../assets/reference-arms/ur10e/expected.json) · [碰撞复核](../assets/reference-arms/ur10e/collision-review.json) |

**使用。** 搬移 URDF、MJCF 时连同同目录的 `meshes/` 复制，USD 已合成为自包含层。MJCF 动态运行前恢复名为 `initial` 的 keyframe，同时恢复关节、控制和执行器激活状态。USD 带 Z 向上坐标、米制单位、刚体、关节、驱动和初始装配姿态。MuJoCo 3.3.7 和 4.0.1 均已实际加载两个格式；USD 已通过 OpenUSD 加载和初始姿态检查，Isaac/PhysX 的实机步进仍需另行验收。

**计算。** 在 skill 目录运行以下命令。第一个命令只需 NumPy，以运动学变换、刚体雅可比和惯量矩阵的中心差分复算五台臂的十个工况。第二个命令另外需要 MuJoCo 和 `usd-core`。容许误差见验证脚本，误差超限会返回非零退出码。

```sh
python scripts/reference_arms.py
python scripts/reference_arms.py --mujoco --usd
python scripts/reference_arms.py --arm kuka_kr6_r900_2 --mujoco --usd
```

`expected.json` 从各自固定版本的 ROS Xacro 展开 URDF **单独导入** MuJoCo 4.0.1 计算。生成数值基准时去掉显示与接触几何、保留惯性和固定坐标系。初始位与偏置位均给出关节序、q（rad）、dq（rad/s）、ddq（rad/s²）、世界至末端的齐次矩阵、保持重力矩（N·m）、完整关节空间惯量矩阵（kg·m²）和逆动力学力矩（N·m）。采用固定基座、地球重力 [0,0,-9.81] m/s²、空载、无接触和无摩擦刚体方程。保持力矩对应正向驱动力抵消重力的符号。动态数值使用非零速度和加速度，包含惯性、科氏/离心和重力项。

例如，UR5e 初始位的第 2 轴保持力矩约 −38.603 N·m，末端位置约 [0.643739, 0.342906, 0.552235] m。FR3 初始位的第 4 轴约 +16.329 N·m，末端位置约 [0.381572, 0.113949, 0.790053] m。比较时使用文件中相同的关节姿态、重力方向及工具帧。输入包含连杆质心、质心惯量的参考坐标系、逐轴速度/努力限值；这些瞬时工况和给定限值用于计算示例，连续驱动额定值需查对应硬件。

以同一个初始关节角、空载和相同重力比较，UR10e 第 2 轴保持力矩约 −78.96 N·m，UR5e 约 −38.60 N·m。这组重力矩差异来自连杆质量及其质心相对关节轴的力臂。每个型号可用 `--arm` 单独复算。

## 碰撞复核

需要用 FR3 运行分拣时，可以从[默认任务输入](../assets/examples/defaults/fr3.json)生成新模型。该输入保留这里的 FR3 运动学及动力学参数，重新设计短连杆外壳，使初始姿态和偏置姿态均通过零件级检查。下方旧几何保留为可复现的碰撞反例。

原有两款参考臂保留所有标称关节限位和真实零件对。UR5e 调整概念肩部布局后，几何零位、初始位及偏置位均通过零件复核；独立搜索时在折叠姿态 `sample_1` 检出底座臂壳与上臂臂壳相交。FR3 例子的数值动力学通过，但初始位仍有短横向连杆的装配问题，例如第 3 连杆碰撞梁和第 4 连杆壳体有共同内部点，证据深度约 4.70 mm。作为动力学和失败诊断例子使用，任务场景应先重新设计相应壳体，再复核。每个报告的 `named_pose_results` 可直接查对应关节配置、零件、证据和结论。[UR5e 碰撞复核](../assets/reference-arms/ur5e/collision-review.json) · [FR3 碰撞复核](../assets/reference-arms/franka_fr3/collision-review.json)

新增三款的初始姿态和偏置姿态均通过零件级碰撞复核。KUKA 的肩肘腕使用逐轴概念壳体尺寸；xArm6 肩部和腕部的轴向电机沿上游退让；UR10e 的肘和腕分别设置壳体直径和轴向宽度。全部尺寸都可在各自输入的 `dimensions` 中检查。三款在扩展采样中均存在干涉，xArm6 与 UR10e 的几何零位也有干涉。任务起点优先读 `initial`，仍须检查实际场景和整条路径。

## 来源和改动

- **UR5e。** Universal Robots 维护的 ROS 2 描述仓库，固定提交 `970bf8aaed1ffa160ca86ab4bb6c04181f7b592d`。使用 `ur_type=ur5e`、默认标称运动学和 ROS 关节范围。世界到惯性基座含 Z 轴 π 旋转，腕部到 `tool0` 的固定变换合并为输入的 tip。共点肩部的基座回转模组取 75 mm 直径，肩部摆动模组取 104 mm，两者轴宽均为 45 mm。上游固定模组在 162.5 mm 安装跨度内退让 100 mm，给侧支承和臂壳留出空间。这些尺寸是概念布局选择。原物理参数对基座质量精度有保留，固定基座质量不参与活动轴保持力矩。[运动学参数](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/970bf8aaed1ffa160ca86ab4bb6c04181f7b592d/config/ur5e/default_kinematics.yaml) · [物理参数](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/970bf8aaed1ffa160ca86ab4bb6c04181f7b592d/config/ur5e/physical_parameters.yaml) · [关节限值](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/970bf8aaed1ffa160ca86ab4bb6c04181f7b592d/config/ur5e/joint_limits.yaml) · [源描述许可证](../assets/reference-arms/ur5e/UPSTREAM-LICENSE.txt)
- **FR3。** Franka Robotics 的 `franka_description`，固定提交 `ddd2fffd9de44b02ad15b4bbb2bfa2cec4d60d98`。采用 `hand=false, gazebo=true` 的刚体数据和该版本的软件限位。上游明确说明公用惯量来自 Gaz 等人针对 Panda 的辨识研究，将其用于 FR3 参考描述时保留这项局限。末端 0.107 m 固定段合并至 tip。[FR3 Xacro](https://github.com/frankarobotics/franka_ros/blob/ddd2fffd9de44b02ad15b4bbb2bfa2cec4d60d98/franka_description/robots/fr3/fr3.urdf.xacro) · [惯性来源与归属](https://github.com/frankarobotics/franka_ros/blob/ddd2fffd9de44b02ad15b4bbb2bfa2cec4d60d98/franka_description/robots/common/inertial.yaml) · [源描述许可证](../assets/reference-arms/franka_fr3/UPSTREAM-LICENSE.txt)

- **KUKA KR 6 R900-2。** 采用 KUKA ROS 社区 AGILUS 描述的关节、惯量和 `tool0` 固定帧，可用于复算该公开模型。硬件参数需对照对应版本的 KUKA 数据表。[固定版本模型](https://github.com/kuka-ros/kuka_robot_descriptions/blob/5ddbc41ce8726fd680bcc19562d78bcabfbdf96d/kuka_agilus_support/urdf/kr6_r900_2.urdf.xacro) · [源许可证](../assets/reference-arms/kuka_kr6_r900_2/UPSTREAM-LICENSE.txt)
- **UFACTORY xArm6。** 采用六轴、无夹爪、`link_eef` 末端与默认 `xarm6_type6_HT_BR2` 惯性配置。该厂商对不同硬件变型提供多套惯性数据，换机型时应核对序列号。[固定版本描述](https://github.com/xArm-Developer/xarm_ros2/tree/ac736c92b02de7ca7e0be6f737d77d46707464d8/xarm_description) · [源许可证](../assets/reference-arms/ufactory_xarm6/UPSTREAM-LICENSE.txt)
- **UR10e。** 与 UR5e 取自同一 Universal Robots 提交，使用 UR10e 的专属物理参数和标称运动学。肘关节的 ±π 为该 ROS 描述的规划范围。
  [物理参数](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/970bf8aaed1ffa160ca86ab4bb6c04181f7b592d/config/ur10e/physical_parameters.yaml) · [源许可证](../assets/reference-arms/ur10e/UPSTREAM-LICENSE.txt)

参考包使用原始数值参数重新组织输入、生成外观、驱动与三个格式。每个输入保存原展开 URDF 的 SHA256。外观不含上游 CAD、商标纹理或手册文件。五个参考包的外观均为概念重建；源文件一致性只验证对应物理方程，碰撞验收应对生成结果和目标任务重新运行。
