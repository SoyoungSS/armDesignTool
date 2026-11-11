# 导出与验收

## 本地检查

运行 generate 后使用 validate；两者返回逐目标 generated、static_checks、runtime_validation、status。status 可为 generated/static_checked/runtime_validated/pending/failed。整体 completed 只说明生成流程完成，必须同时看 pending 目标。每次运行独立目录，latest_attempt 包括失败，不覆盖旧模型。manifest 保存文件清单、SHA256、命名映射、请求版本；validation 保存实际环境、检查数与干涉候选。

测试：`python -m unittest discover -s scripts/tests -v`。核心测试涵盖：缺项、非法轴/限位/惯量、单位、安装一次、显式拓扑排序、简易适配、并行轴合成、共点和短连杆、确定性网格、搬移、损坏资源及失败不复用旧结果。有相邻 workspace Skill 和 Node 时对其真实 FK 函数做独立比对；无环境时显示 skipped，不能计为通过。可选 MuJoCo/OpenUSD 缺失也显式跳过。

验证器检查二进制 STL 长度、非退化面、封闭边和有向体积。URDF 从导出 XML 重新计算每个连杆/末端位姿，比较零位、初始值、验收姿态与四组固定确定姿态；核对限位、轴、惯量、视觉/碰撞局部变换，并在临时目录搬移后加载全部 Mesh。独立 URDF 的资源根为 URDF 所在目录；加载器须支持这个约定或显式设置 resolver。不是所有 URDF 加载器都会自行解析相对文件名。

干涉使用连杆 AABB 宽阶段与碰撞 primitive 内部采样点。solid_overlap_witnesses 表示找到同时严格位于两个碰撞实体内的点，包含相邻链接；关节壳体或导轨配合可能是有意重叠，需人工区分。没有 witness 不证明不相交。候选和证据分别列出，不以外观整体包围盒代替精确碰撞验收。

## ROS1 Noetic / RViz

包中包含 package.xml、CMakeLists.txt、urdf、meshes、launch/display.launch、rviz/display.rviz。默认固定帧为实际 base_link，不是推测的 world；world_fixed 仍保留安装变换。先在装有 Noetic 的主机执行：

```bash
mkdir -p ~/catkin_ws/src
# 将生成的 ros1/<robot>_description 放入 src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch <robot>_description display.launch
# 无滑块窗口：
roslaunch <robot>_description display.launch gui:=false
```

依赖 robot_state_publisher、joint_state_publisher、joint_state_publisher_gui、rviz。检查模型资源无错误、TF 链完整、初始值正确、每个滑块正方向和上下限；观察零位与验收姿态。保存 rosversion -d、构建结果、日志和截图。无 ROS1 桌面时只报告静态检查通过，构建/GUI 操作仍 pending。

## MuJoCo

`--runtime` 实际加载 MjModel，按导出名称设置 q，在全部检查姿态执行 mj_forward，核对每个 body 与 tool_frame site 的位姿、限位、质量、COM、主轴重构惯量。核对 actuator 数量、关节关联、kp/kv、控制/力矩范围、滤波常数和初始 activation；并将各固定采样姿态的整链质量矩阵、重力项与 MuJoCo 比较。恢复 initial keyframe 后运行50个建议步长的步进，检查有限状态与数值警告。

位置驱动还进行初始姿态及前两组验收姿态各4秒保持，以及初始姿态起的小幅五次平滑运动（每轴最多0.05rad/0.005m、最多5%行程，目标速度不超过25%配置速度），随后保持4秒。尾段0.5秒峰峰值阈值为0.002rad/0.0002m，速度阈值0.01rad/s或0.001m/s，99%力矩上限的饱和占比须小于1%；全程实际速度不得超过配置值的105%。保留静差，静差不作为上述振动通过条件。超过50万步预算则 pending，不误报通过。接触开启结果决定通过，关闭接触的结果只用于定位问题。被动、速度和 effort 模式的有限短步进不能称为位置保持验收。

MJCF 使用 angle=radian、inertiafromgeom=false、固定根 body、显式惯性主轴、视觉/碰撞分离；visual geom 不参与碰撞。actuator 按配置输出 position/velocity/motor，未指定模式默认自动位置驱动，显式 passive 无驱动。采用 implicitfast 与策略计算的步长；URDF velocity 不会自动变成 MuJoCo 硬速度限制。使用导出的 `mujoco/run.py` 自动初始化 keyframe。参数计算、装配配合接触排除和局限见 [驱动规则](drives.md)。

## Isaac Sim / USD

默认离线路径需要 usd-core，入口 robot.usd 引用同目录 robot_zero.usd；所有视觉三角形嵌入零位层，不依赖 STL 插件。两层必须一同搬移；runtime_config.json 保存初始 q、驱动、建议步长和关节路径。单位 m、Z-up，完整物理时写 RigidBody/Mass/Collision/Joint/ArticulationRoot API，fixed joints 保留。无质量/惯性时写可视化 USD，不能当成 articulation 仿真模型。

USD 旋转限位/drive target 单位 deg，关节轴用 canonical Z 和两端 localRot 对齐原斜轴。localRot0=originRotation×axisAlignment，localRot1=axisAlignment；不能只旋转一个端点。旋转 drive kp/kv 从每 rad 换算为每 deg，力矩仍 Nm。robot_zero.usd 保存几何零位；robot.usd 只覆盖刚体装配到初始姿态，不改变关节的零位框架；运行时按 SI 初始化 q 并核对目标。OpenUSD 重加载核对两层装配、两端关节框架重建的多姿态 FK、限位、质量和驱动属性、过滤配合体关系、搬移后的引用；不包含 PhysX 引擎验收。Isaac 使用建议 physics_dt，并在运行时目标生成器应用 timeconst 滤波；不要直接用 GUI 默认大步长证明稳定。

目标 Isaac 环境存在时，优先使用该版本官方 URDF importer，并输出到新的导入目录作对比：Fix Base=true、Merge Fixed Joints=false、Import Inertia=true、Distance Scale=1、碰撞使用生成的 primitive、无凭空密度、默认 drive 不覆盖用户模式/增益。导入后应用原始 drives、核对 joint/link 映射和 tool_frame；不要重复施加 URDF world_fixed 的安装。官方导入结果通过重新加载和 articulation 验证后，才可替代离线交付入口。

5.1 的官方 Python 路由是 `isaacsim.asset.importer.urdf` 与 `URDFParseAndImportFile`。6.0 起该命令已移除，使用目标版本 URDFImporter/URDFImporterConfig，禁止跨版本盲跑旧命令。版本不明时先探测本地 runtime，读对应官方文档；不安装庞大的 Isaac 运行时来假装本机原已具备环境。

离线 USD 的 5.1 初步实机脚本：

```bash
<isaac-python> scripts/isaac_runtime.py --run /absolute/run --evidence /absolute/external-check/isaac.json --expected-version 5.1
```

该脚本仅 smoke test：确认 articulation root、DOF 名称、初始值及有限短步进；full_acceptance 保持 pending，需在目标版本核对每个 link/tool 的世界位姿、斜轴正方向、限位、固定关节与驱动效果。依赖未具备时脚本写失败证据，不更新原 manifest 为通过。其他版本先按该版本 API 适配 helper，不声称已验证。

## 官方参考（2026-11-10 核对）

- [MuJoCo XML reference](https://mujoco.readthedocs.io/en/3.3.5/XMLreference.html)：compiler、body/joint/geom、inertial、actuator 和 keyframe。
- [OpenUSD Physics schema](https://openusd.org/dev/api/usd_physics_page_front.html)：质量、刚体与两端关节框架。
- [Isaac 5.1 URDF importer](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/importer_exporter/import_urdf.html)：对应版本官方导入路径。
- [Isaac 6.0 importer migration](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/urdf_mjcf_importer_exporter_pipeline.html)：旧命令移除及新导入 API。

示例的 Isaac 5.1 是明确的接口目标，不代表当前推荐版本，也不代表本机已安装该环境。

## 首版实际验收记录（2026-11-10）

- Skill frontmatter/scaffold 校验通过；14 项 unittest 全部通过，无跳过。
- Windows，隔离 Python 3.11.9、NumPy 2.2.6、MuJoCo 3.3.7、OpenUSD 25.11。系统 Anaconda 环境曾出现可选运行库 DLL 错误，独立 Python 环境可正常运行，未修改全局 Python 配置。
- 双旋转轴纯运动学样例完成自动建网格、URDF、ROS1、可视化 USD；无质量/惯量未伪造动力学参数。
- 斜轴/移动/fixed/偏心 COM 样例及概念密度 JSONC 模板均完成四目标；逐目标资源搬移后重加载通过。完整样例每个相关格式核对40个连杆/末端姿态，MuJoCo 完成50×1ms步且无数值警告。
- 独立试用发现并修复方向向量原地归一化污染跨度的问题；补充端点中点、梁长度和质量代理 COM 回归断言。独立复测额外三姿态：URDF/ROS FK 最大元素误差0，MuJoCo 7.61e-13，USD 两端关节框架重建5.64e-8；梁中心偏差不超过9.32e-10m。
- ROS1 catkin/RViz 滑块、Isaac 官方导入器和 PhysX 实机验收仍 pending；不把离线 USD 或 smoke helper 的存在写成实机通过。

可选离线外观图使用 `python scripts/preview.py --run <run> --output <outside-run>/preview.png`，另需 Pillow。它是静态正交投影，不能代替 RViz/Isaac 运行验收。将预览放在 run 之外避免改变已校验的交付清单。

## 驱动版本实际验收记录（2026-11-11）

- `serial-pd-v1`，26 项 unittest 全部通过，无跳过；涵盖单旋转/单移动轴解析动力学、整链惯量与重力、确定性/质量缩放、显式与部分增益、被动/速度/effort 模式、容量不足提示、步长随刚度变化、同输入 MJCF/USD 字节一致及可移植启动器。Skill 标准格式校验通过。
- Windows 隔离 Python 3.11.9、NumPy 2.2.6、MuJoCo 3.3.7、OpenUSD 25.11；原系统环境未改动。七轴样例实际导出7个位置 actuator 和7组 USD DriveAPI，建议 dt=0.000655783s；21个姿态的 M/g、72个连杆/末端 FK 比对通过。
- 七轴接触开启：3组保持加1组平滑小幅运动均收敛，最大尾段峰峰值约1.46e-6rad，最大尾段速度约5.18e-6rad/s；无数值警告、尾段力矩饱和或配置速度超限。有限工况通过不证明全工作空间稳定。
- 最大实际重力静差约0.09294rad（5.33°），策略已提示部分轴受比例项力矩预算限制。此模型未加重力前馈，不把稳定性通过解释为定位精度通过。
- USD 初始/零位两层各9个位姿、多姿态关节框架、角度换算、驱动属性及搬移后引用通过；Isaac/PhysX 实机仍 pending。ROS1 catkin/RViz 同样仍 pending。
- 独立试用自建旋转+移动双轴输入，自动/显式驱动均通过 MuJoCo 运行及 USD 属性重加载；显式 kp/kv/timeconst 保留，缺 velocity 的负例明确失败。独立解析 M/g 对比误差分别约1.11e-16与0。完整 Skill ZIP 解压到无相邻 Skill 的临时目录后，核心 URDF 生成、四目标生成及 validate 均通过。

## 原生外壳版本验收记录（2026-11-11）

- generator 1.2.0 / rounded-shell-v1；核心仍只有 NumPy，无 CAD/布尔库新增依赖。31项回归全部通过，新增真实闭合曲面、空间约束、共点/斜轴模板、显式尺寸、外廓冲突和细节开关不改变物理/碰撞测试。
- 七轴输出79个 STL 文件（包含逐部件与逐连杆合并副本）闭合、正向且无退化面；USD 重加载48个视觉部件的材质绑定、粗糙度、金属度与单位法线。URDF/ROS1 相对资源、USD 两层引用均经过搬移检查。
- 七轴的给定质量/COM/惯量及驱动参数与上一驱动版本完全一致，接触开启的3组保持和1组小幅运动均通过，仍保留原有重力静差。碰撞代理为低复杂度近似，不能把无代理接触解释为真实外壳全行程无干涉。
- 实际生成 STL 的初始、验收姿态和侧面预览已检查；前后对比使用同一预览器与相机方向。真实 Mesh 的截面、端部过渡和模组尺寸均已变化，不仅是渲染修改。ROS1 GUI 与 Isaac/PhysX 实机仍 pending。

## 共点模组布置验收（2026-11-11）

- generator 1.3.0 / rounded-shell-v2 / axis-module-layout-v1。38项回归通过；新增角色与轴反向、固定/输出端归属、FK/组件不变、真实环形内孔、同轴组合、交叉输出轴间隙、显式 mm 退让与空间不足、多轴未解析状态、独立包络冲突检查。
- 七轴示例自动识别 J1/J2 与 J5/J6；J1 自转模组向上游退让约109.177mm，J5约70.112mm。固定壳分别归 base_link/J1_link 与 J4_link/J5_link，输出件归各自 child；原关节 origin、轴、质量、COM、惯量、驱动、安装及初始/验收姿态与上一版本逐项相同。
- 在8个检查姿态对4个新模组进行48组包络比较；组合内最小投影间隙2mm。检查不读取引擎碰撞排除。原动力学保持/小幅运动通过，不代表全行程机械装配或精准位置保持。
- 三个以上共点旋转轴或不能识别支持结构时 assembly_validation=needs_review，目标保持 pending；投影包络不能分离时目标失败。不能把成功的短步进覆盖成装配已验收。
- 七轴实际 STL 的初始/验收/侧面图已检查；USD 离线重加载通过，ROS1 GUI 和 Isaac/PhysX 仍待目标环境验证。
- 独立试用自建共点自转/摆动双轴输入，三格式部件归属及原始物理输入保持通过；与独立原输入 FK 比较，MuJoCo 四姿态最大矩阵误差约7.48e-13。显式1mm退让按最小78mm要求失败，反向轴通过。修正报告轴字段为 axis_base_zero 并标明 axis_frame，避免把基座系误称世界系。完整 ZIP 解压搬移后，脱离相邻 Skill 的核心 URDF 生成通过。
