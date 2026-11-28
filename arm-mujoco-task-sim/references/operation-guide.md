# 模式补充

默认概念模式按[常规试算](concept-trial.md)补齐未指定参数并优先运行，用户可用描述迭代。下列逐项等待确认的流程适用于严格模式；已有概念委托不重复请求数值确认。

# 操作细则

命令和反引号中的相对路径均以本技能根目录为基准；本页的 Markdown 链接相对于本页。仅阅读入口指定的相关章节。示例参数只用于演示，不是用户设计或厂家能力。版本清单和历史验收不代表当前环境已验证。

## 离线依赖速查

缺少 Python 或包时，先查同一仓库的 `arm-mujoco-task-sim/assets/`。复制到其他机器后按相对路径使用。[离线安装命令、文件校验和系统依赖](../references/offline-dependencies.md)

| 需要的功能或缺少的包 | 随附版本 | 共享安装包位置（相对 `arm-mujoco-task-sim/assets/`） |
|---|---|---|
| Windows x64 / Ubuntu 20.04、22.04、24.04 x86-64 的 Python、pip、venv | CPython 3.12.16 | `python-win_amd64/`、`python-linux-x86_64/`。三个 Ubuntu 版本共用一份 |
| 生成网格、URDF、MJCF 和 PNG（`numpy`、`PIL`） | NumPy 2.5.5、Pillow 13.0.0 | `wheels/<平台>/`，只做模型生成可选 `model` |
| 任务物理仿真、Web 和原生窗口 | mujoco-core / mujoco-studio / mujoco-studio-wasm 4.0.1、websockets 17.4 | `wheels/<平台>/` 和 `wheels/common/` |
| 曲线和报告 | Matplotlib 3.11.2、contourpy 1.4.0、fonttools 4.68.0、kiwisolver 1.5.1、cycler 0.12.1、packaging 26.3、pyparsing 3.3.3、python-dateutil 4.0.0、tzdata 2026.6 | 随任务环境安装，位于 `wheels/<平台>/` 和 `wheels/common/` |
| USD 导出和静态校验（`pxr`） | usd-core 25.11 | `wheels-optional/<平台>/`，选 `usd` |
| Python 相机渲染 | glfw 2.10.0、PyOpenGL 3.1.10 | `wheels-optional/<平台>/` 和 `wheels-optional/common/`，选 `camera` |
| 复现生成器的旧版验证环境 | mujoco 3.3.7、absl-py 2.3.1、etils 1.13.0、fsspec 2025.10.0、importlib-resources 6.5.2、zipp 3.23.0、typing-extensions 4.15.0，另含上述 OpenUSD、渲染和模型基础包 | 在独立环境选 `generator-3.3.7`，从两组 wheel 目录取包 |
| Windows 缺少 MSVCP140 / VCRUNTIME140 系列 DLL | Microsoft Visual C++ x64 14.52.37029.0 | `windows-runtime/vc_redist.x64.exe`，按需安装 |

`<平台>` 在 Windows 上是 `cp312-win_amd64`，在 Ubuntu 上是 `cp312-linux_x86_64`。

## 任务工作流

要直接试用选型表七轴或 Franka FR3 的三轮桌面抓放，运行[双模型红蓝分拣示例](../references/dual-sorting.md)。

输入为模型生成器的一次实际输出和任务描述。命令中的 `<skill>`、`<project>` 替换为实际路径，同一项目使用同一已验证的 Python。

| 步骤 | 执行与继续条件 |
|---|---|
| 1. 接入机械臂 | `python <skill>/scripts/inspect_generator.py <模型输出或run> --out <project>/model_inspection.json`。检查入口、哈希、关节和初始 keyframe。读上游零件级碰撞结论与待复核姿态，保留已知质量、惯量、能力和名称。旧模型会标记碰撞未评估。 |
| 2. 确认任务与路线 | 按[任务确认项](../references/intake.md)复用用户已有答复。核对目标、成功判据、查看模式、策略、环境与工具参数、起止条件、节拍、轮数、轮间复位和输出位置。对缺项一次提问。明确指定的方法直接落实；路线有实质差异时按[策略选择与提问示例](../references/scene-strategy.md)给出候选并记录用户选择。 |
| 3. 定好运行环境 | 先用已有解释器执行 `python <skill>/scripts/environment.py probe --mode web`（或 native、headless）。通过即复用。Ubuntu 20.04/22.04/24.04 x86-64 缺环境时用同一条离线命令 `python3 <skill>/scripts/install_ubuntu.py --mode web --venv .venv`。Windows 及其他平台见[离线安装与版本指南](../references/install-api.md)。Web 与本地原生窗口共用 Python 后端；无窗口批量试验需获得相应授权。 |
| 4. 生成场景和控制器 | 从[合同模板](../assets/task_contract.template.json)填入参数、来源和真实确认证据。写入资产清单、策略和[控制适配器](../references/adapter.md)。按[碰撞合同](../references/collision.md)列出各阶段的接触对象、穿透/力阈值来源和轨迹检查方式。保留视觉、碰撞、动力学与传感器各自的几何和数据来源。 |
| 5. 预检与短试跑 | 运行下方预检命令，读本次 `reports/preflight.json`。按证据修复零件、摆位、路径或阈值来源；修改后重新检查。执行有代表性的接近、接触和退出，检查速度、饱和、抓稳/滑移、接触力及失败处理。[验证方法](../references/validation.md) |
| 6. 正式仿真与交付 | 按确认轮数运行，检查 `reports/result.json`、碰撞诊断、逐轮状态及原始时序。按[输出规范](../references/outputs.md)交付 CSV、含限值的 PNG 曲线、失败证据和实际 run 路径。用实际场景验证播放、暂停、恢复、Reset 后再播放、退出和端口释放，再交付启动命令。 |

```text
python <skill>/scripts/contracts.py <project>/task_contract.json
python <skill>/scripts/run_task.py --project <project> --preflight
python <skill>/scripts/run_task.py --project <project>
python <skill>/scripts/analyze.py <project>/runs/<run-id>
```

只有用户答复或明确委托的估算才能写入合同。未确认时可准备草案，依赖未确认参数的正式仿真须等待。控制器按物理时钟运行，复位建立新的时间段或 run；指定视觉策略时记录实际图像观测、估计和控制证据。失败轮次保留，涉及任务含义、质量、摩擦、工具或驱动能力变化时重新确认。

记录器支持固定基座串联 hinge/slide，并可与夹爪及带 freejoint 的物体同场。机器人自身有球/自由关节、闭链、柔性或自定义积分时需扩展并验证。关节反力、驱动力、末端接触力分别记录；换算电机载荷或寿命还需传动及材料等参数。

验证其他 agent 的使用效果时用[同题回归与评分](../references/agent-benchmark.md)，保留模型标识、用户答复和实际输出。
