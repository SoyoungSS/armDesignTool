# 生成器环境与离线依赖

以下为随附离线包清单，不代表当前解释器已经安装这些版本。缺少 Python 或包时，先查同一仓库的 `arm-mujoco-task-sim/assets/`。复制到其他机器后按相对路径使用。[离线安装命令、文件校验和系统依赖](../../arm-mujoco-task-sim/references/offline-dependencies.md)

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

