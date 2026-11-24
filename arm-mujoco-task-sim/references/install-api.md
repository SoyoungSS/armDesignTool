# 安装、模式与官方接口

## 环境复用与离线安装

先找到项目启动脚本或已有环境锁记录的 Python，用该解释器执行 `python <skill>/scripts/environment.py probe --mode web`，模式可换为 `native` 或 `headless`。探测会执行物理步进、质量矩阵计算、对应查看器组件导入和绘图。若项目已有独立绘图环境，可设置 `ARM_SIM_ANALYSIS_PYTHON`。通过后保存输出中的解释器路径并直接用于预检与正式运行。界面交互在任务场景中另行验收。

### Ubuntu 20.04、22.04、24.04 共用一套离线环境

三版 **Intel/AMD 64 位（`uname -m` 输出 `x86_64`）** 采用同一份 CPython 3.12.16、MuJoCo Studio/Core 4.0.1 与绘图依赖。20.04、22.04 和 24.04 的系统 Python 分别为 3.8、3.10 和 3.12。只需系统提供 `python3`、`tar` 和常规运行库，安装器会校验并展开 skill 自带的可迁移解释器。无需 sudo、在线 pip、额外软件源或替换系统 Python。

从项目目录执行，将 `/path/to/arm-mujoco-task-sim` 换成实际 skill 路径。选一次所需模式，之后可在同一环境切换：

```sh
SKILL=/path/to/arm-mujoco-task-sim
python3 "$SKILL/scripts/install_ubuntu.py" --mode web --venv .venv
.venv/bin/python "$SKILL/scripts/run_task.py" --project . --preflight
# 检查同一环境的无窗口模式
.venv/bin/python "$SKILL/scripts/environment.py" probe --mode headless
```

极简镜像若连 `python3` 都没有，可直接从同一份归档启动。以下命令需要 `tar`、`sha256sum`，解释器目录须保留供该项目的 venv 使用：

```sh
bundle="$SKILL/assets/python-linux-x86_64"
runtime="$HOME/.local/arm-cpython-3.12.16"
(cd "$bundle" && sha256sum -c SHA256SUMS) &&
  mkdir -p "$runtime" &&
  tar -xzf "$bundle"/cpython-*.tar.gz --strip-components=1 -C "$runtime" &&
  "$runtime/bin/python3.12" "$SKILL/scripts/environment.py" install --mode web --venv .venv
```

可迁移解释器首次解包到 `${XDG_CACHE_HOME:-$HOME/.cache}/arm-mujoco-task-sim/`，后续项目复用；已有项目环境仍引用这个目录。拷贝整个 skill 即可断网安装。只传输 Linux 所需的安装材料时，保留 `scripts/`、`assets/python-linux-x86_64/`、`assets/wheels/common/` 和 `assets/wheels/cp312-linux_x86_64/` 的相对位置。启动任务还需项目输入以及所用模板和其他任务资产。Windows 与 Linux 共用纯 Python 和 Web 客户端 wheel。

Linux 基础分拣环境约 96.5 MB，已含解释器。OpenUSD、相机和旧版兼容 wheel 单独存放，见[共享离线安装包](offline-dependencies.md)。在联网机只下载 Ubuntu 版任务 skill 时，可用 Git 的按需检出，然后将检出的 `arm-mujoco-task-sim` 文件夹复制到各台离线机器：

```sh
git clone --depth 1 --filter=blob:none --no-checkout https://github.com/SoyoungSS/armDesignTool.git
git -C armDesignTool sparse-checkout set --no-cone '/arm-mujoco-task-sim/' '!/arm-mujoco-task-sim/assets/wheels/cp312-win_amd64/' '!/arm-mujoco-task-sim/assets/wheels-optional/cp312-win_amd64/' '!/arm-mujoco-task-sim/assets/python-win_amd64/' '!/arm-mujoco-task-sim/assets/windows-runtime/'
git -C armDesignTool checkout
```

三个 Ubuntu 版本的 glibc 基线依次为 2.31、2.35、2.39。已核对随包解释器、Linux wheel 的 SHA256、Python 依赖和 ELF 导入符号。全套二进制最高要求 GLIBC_2.27、GLIBCXX_3.4.22。精简系统需提供标准 C/C++ 运行库、zlib、libcrypt；本地窗口还要有图形会话和可用驱动，Web 需可访问运行器端口的浏览器。ARM64 / Jetson 使用下文的匹配架构打包流程。包版本、兼容标签和解释器发行信息可核对 [Linux 清单](../assets/wheels/cp312-linux_x86_64/bundle.json) · [可迁移 CPython 原版发行](https://github.com/astral-sh/python-build-standalone/releases/tag/20261110) · [MuJoCo Studio 发行文件](https://pypi.org/project/mujoco-studio/4.0.1/#files)。

**断网实测（2026-11-23）。** 在同一 QEMU Linux 6.6.134 内核下分别启动官方 Ubuntu Base 20.04.1、22.04、24.04.3 amd64 用户空间，禁用虚拟网卡。三者均用归档内的 Python 启动 `install_ubuntu.py` 安装同一组 15 个 wheel，通过 `pip check`、MuJoCo 质量矩阵计算及绘图。相同模型各步进 250 次（0.5 s），末角均为 0.14897657 rad，并各自生成 7933 字节的曲线 PNG。三个环境的 Web 和本地查看器均通过模块导入检查。本轮图形测试范围止于导入，浏览器显示和原生窗口仍需在目标桌面试跑。

### Windows x64

**Windows x64、普通 CPython 3.12** 可从 skill 随附的 15 个原版 wheel 离线安装，覆盖 Studio 4.0.1、物理核心、WASM 客户端、NumPy、Matplotlib 和间接依赖。随包提供 CPython 3.12.16（含 venv 和 pip）以及微软 x64 运行库。新机可直接按[共享离线安装说明](offline-dependencies.md)用 PowerShell 启动；原生查看器需要桌面和图形驱动。已有 Python 启动器时，在项目目录执行：

```powershell
py -3.12 <skill>/scripts/environment.py install --mode web --venv .venv
.\.venv\Scripts\python.exe <skill>/scripts/run_task.py --project . --preflight
```

安装器先核对整包文件清单、平台与 SHA256，再用 `--no-index` 从本地安装，最后运行 `pip check` 和模式探测。使用其他模式可在同一环境重新 probe。固定的离线包供复现一次已验证组合，项目也可复用下节列出的其他兼容版本。

没有 `py` 启动器时，直接用已有 64 位 Python 3.12 的可执行文件运行相同命令。首次安装后将探测输出里的 python 路径写入项目启动入口。该环境也已具备上游模型生成和 PNG 预览所需的 NumPy 和 Pillow，生成 MJCF 与任务仿真可以共用它。OpenUSD 导出可从随包的 `wheels-optional` 安装 `usd` 包组。

这套离线包已在 Windows x64 / CPython 3.12 上完成 Web 后端和本地窗口各 200 步试跑。Windows 上 Studio 4.0.1 的默认显示路径可能报 `Target not found`，本运行器对这一组合使用实测可用的 `opengl_software` 本地呈现模式。需要验证其他原生显示后端时可在合同中指定 `native_gfx`。裸机拷贝本 Skill 时保留 `assets/python-win_amd64/` 和 `assets/windows-runtime/`。

### 为其他组合打包

其他系统、架构、Python 或 MuJoCo 版本可在与目标机匹配的联网机器准备离线包（Linux 还应匹配 glibc 基线）。复制整个生成目录，保留 `bundle.json` 和原版 wheel 内的许可证。下面以 Studio 4.0.1 为例：

```text
python <skill>/scripts/environment.py pack --mode web --version 4.0.1 --bundle <新目录>
python <skill>/scripts/environment.py install --mode web --venv <项目环境> --bundle <已复制的目录>
```

纯计算可用 `pack --mode headless` 准备 core 和绘图依赖。涉及视觉识别、相机或额外规划库时，先根据选定路线列明增量依赖与离线资产。断网且无匹配包时，报告缺少的解释器、平台或包名，并给出匹配机器上的准备命令。保留已有可用环境，避免反复重建。

## 版本选择

2026-11-16 核查官方发布说明和 PyPI 文件。按用户选定的模式选择 Python 发行包，并核对系统、CPU 架构和 Python ABI 对应的 wheel。

| MuJoCo 版本 | 官方 WebViewer 任务 | 本地任务及模型检查 |
|---|---|---|
| **4.0.1** | Windows x64、Linux 和 macOS 对应 wheel 可用时优先试用。浏览器客户端由 `mujoco-studio-wasm` 提供，随 Studio 安装。 | NativeViewer 或无窗口任务的首选基线。该补丁修复了 Windows WebViewer 分发，以及 4.0.0 中可能引起错误加速度和发散的稀疏求解问题。 |
| **4.0.0** | Linux/macOS 的 Studio 发布包是可试跑的兼容候选。Windows 的 PyPI 包缺少可用的浏览器客户端。 | 有 NativeViewer 和 core 接口。只在复现旧项目等确有需要时保留，接触与步进结果须重新检查。 |
| **后续 4.x 正式补丁版** | 以发布说明、匹配 wheel 和实际浏览器试跑为准。 | 以 NativeViewer 或无窗口试跑为准，同时检查任务插件和采样 API。 |
| **3.x，例如 3.17.0、3.3.7** | 完整任务需 Studio WebViewer 接口。 | 可用经典 `mujoco.viewer` 加载与检查 MJCF。3.17.0 提供当前录制器所用的 `mj_fullM(m,d,M)` 签名，可作为无窗口试跑候选。3.3.7 是上游模型生成器的校验基线，运行此任务录制器前须适配旧签名。经典查看器运行完整任务需另接控制与记录适配器。 |

优先复用团队已经稳定运行、能通过此次场景试跑的正式版本。新环境先筛选支持所需模式、平台和场景功能且含关键修复的版本，再优先选发布较成熟、同平台复现案例较多或被任务依赖采用的正式补丁版。可参考团队环境锁文件、下游项目的依赖约束及同平台的问题记录。现有官方发布说明和包元数据不足以给 4.0.0 与 4.0.1 排出用户数顺序。选择时避开预发布和已撤回版本。试跑成功后锁定项目环境，记录版本与选择依据。

- [4.0.0 发布说明](https://github.com/google-deepmind/mujoco/releases/tag/4.0.0)
- [4.0.1 发布说明及 Windows WebViewer 修复](https://github.com/google-deepmind/mujoco/releases/tag/4.0.1)
- [Studio 版本与 wheel](https://pypi.org/project/mujoco-studio/#history) · [Core 版本与 wheel](https://pypi.org/project/mujoco-core/#history) · [3.17.0 版本与 wheel](https://pypi.org/project/mujoco/3.17.0/#files)
- [3.17.0 惯量矩阵 API](https://github.com/google-deepmind/mujoco/blob/3.17.0/include/mujoco/mujoco.h)

## 官方资料入口

- [Python 安装及包拆分](https://mujoco.readthedocs.io/en/latest/python.html#installation)
- [官方 WebViewer / MuJoCo Live](https://mujoco.readthedocs.io/en/latest/studio/web_viewers.html)
- [官方 NativeViewer 与各平台启动](https://mujoco.readthedocs.io/en/latest/studio/native_viewers.html)
- [版本说明](https://mujoco.readthedocs.io/en/latest/changelog.html)
- [官方二进制发布](https://github.com/google-deepmind/mujoco/releases)
- [Python 官方源码和示例](https://github.com/google-deepmind/mujoco/tree/main/python/mujoco)
- [模型构建](https://mujoco.readthedocs.io/en/latest/modeling.html)、[MJCF 元素与参数](https://mujoco.readthedocs.io/en/latest/XMLreference.html)
- [函数接口](https://mujoco.readthedocs.io/en/latest/APIreference/APIfunctions.html)、[数据与模型类型](https://mujoco.readthedocs.io/en/latest/APIreference/APItypes.html)
- [物理计算与坐标约定](https://mujoco.readthedocs.io/en/latest/computation/index.html)

## 环境命令

用项目独立环境。Studio 4.0.0/4.0.1 的包元数据要求 Python ≥3.11，官方 CPython 3.11 和 3.12 wheel 覆盖 Windows x64、Linux x64/ARM64（glibc ≥2.27）、macOS ARM64（11+）。用这两个版本的 NativeViewer 或 Filament 时选 **Python ≥3.12**，其渲染模块会导入 `collections.abc.Buffer`。Windows x64 上已用 Python 3.11 和 4.0.1 跑通 WebViewer 后端与任务插件。以下用 Python 3.12 和 4.0.1 演示，按上表替换版本和包名。 [4.0.0 渲染模块](https://github.com/google-deepmind/mujoco/blob/4.0.0/python/mujoco/rendering/filament/renderer.py) · [4.0.1 渲染模块](https://github.com/google-deepmind/mujoco/blob/4.0.1/python/mujoco/rendering/filament/renderer.py) · [Python Buffer 接口](https://docs.python.org/3/library/collections.abc.html#collections.abc.Buffer)

```powershell
py -3.12 -m venv .venv
$mujocoVersion = '4.0.1'
.\.venv\Scripts\python.exe -m pip install --only-binary=:all: "mujoco-studio==$mujocoVersion" "numpy>=2,<3"
.\.venv\Scripts\python.exe -m pip check
.\.venv\Scripts\python.exe -m pip freeze > environment-lock.txt
```

Linux / macOS：

```sh
python3.12 -m venv .venv
mujoco_version=4.0.1
.venv/bin/python -m pip install --only-binary=:all: "mujoco-studio==$mujoco_version" 'numpy>=2,<3'
.venv/bin/python -m pip check
.venv/bin/python -m pip freeze > environment-lock.txt
```

- **WebViewer 或 NativeViewer** 安装所选版本的 `mujoco-studio`，或安装同一版本的完整 `mujoco`。由包解析器安装配套 core 和适用的 WASM 组件。
- **无窗口** 在 4.x 安装所选版本的 `mujoco-core`。若要试跑 3.17.0 或使用经典查看器检查模型，单独创建环境并安装 `mujoco==3.17.0`。3.17.0 支持 Python ≥3.10。
- **渲染和分析** 需要经典 Python 渲染器时选 `mujoco-core[rendering]` 或完整 `mujoco`。按需安装相机、视觉、绘图和视频依赖。

用 `pip check` 和冻结清单核对实际安装的 core、Studio 与 WASM 组件是否匹配。跨 3.x / 4.x 或切换主机架构时使用单独环境。

## 模式对照与启动

| 模式 | 物理/策略运行位置 | 启动和限制 |
|---|---|---|
| 官方 WebViewer | 本地或已配置服务器 Python | `launch_web`，浏览器渲染场景；无需主机原生窗口；本工具绑定 127.0.0.1 |
| 官方 NativeViewer | 本机 Python | `launch_native`，本机窗口；需合适显卡/渲染后端；macOS 非阻塞入口需 `mjpython` |
| 无窗口 | Python | 验证/批量数据使用，先获用户同意；不能称为实时可视化已验证 |
| MuJoCo Live | 浏览器 WASM | 零安装模型查看/内建仿真；不直接执行本包 Python 适配器。如用户要求这种模式，另评估接口能力并确认方案 |

仅检查一个模型能否在官方窗口启动：

```sh
python -m mujoco.studio.viewer --model=scene/scene.xml --gfx=web
python -m mujoco.studio.viewer --model=scene/scene.xml
```

使用经典 Python 查看器时运行 `python -m mujoco.viewer --mjcf=scene/scene.xml`。

这两条只用于查看模型。完整任务使用 `run_task.py --project <project>`，mode 来自已确认合同。原生窗口在 macOS 使用 `mjpython`。运行器默认暂停，官方工具条 Play 开始。Reset 在同一查看器内重建任务控制器、初始物理状态和记录器，生成独立 run，并暂停等待再次播放。Reset 后保留原 run 的记录。历史跳转、交互扰动、外部模型替换和物理选项改动在记录会话中被锁定。

## 任务入口与会话清理

- 为用户提供项目启动入口。Web 默认用 `--port 0 --open-browser`，由官方查看器直接绑定可用端口并打开实际地址，监听限制在本机回环。启动输出实际访问地址和 run 路径。只清理本任务可确认归属的进程。
- 在官方工具条验证 Play、Pause 和 Reset。可在查看器里加入随物理时间推进的滚动关键曲线，例如关节驱动力矩与 TCP 位置。Reset 清空上一 run 的实时曲线。
- Web 可用 `File → Quit` 或启动终端的 Ctrl+C 退出，默认在浏览器断开超过 45 秒后自动退出。退出时封存记录并释放监听。用实际场景完成验收后退出测试会话，确认监听释放。交付后由用户自行启动。
- PNG 曲线需要 Matplotlib。可装入项目环境，或设置 `ARM_SIM_ANALYSIS_PYTHON` 指向已验证的绘图环境。SVG 仅依赖 NumPy。

独立桌面应用也可从官方发布页获取对应系统和架构的包：Windows zip 内 `bin/mujoco_studio.exe`，Linux tar.gz 内 `bin/mujoco_studio`，macOS dmg 的 `MuJoCoStudio.app`。不假定安装独立应用能运行 Python 策略，任务应选对应 Python NativeViewer 链路。

## 探测与故障

先在选定 Python 中检查 `mujoco.__version__`、实际 MJCF 编译、`mj_jacDot` 及 `mj_fullM` 调用，并运行 `scripts/self_test.py` 检查步进和载荷采样。Web 模式可用 `scripts/web_smoke_test.py` 检查服务及任务插件，再在浏览器打开实际场景。Native 模式实际创建窗口。两种界面都检查播放、暂停、Reset 及采样。记录平台、架构、Python、安装命令、解析后的包版本、渲染后端及试跑结果。

4.0.1 官方发布说明修复了 Windows WebViewer 组件分发问题。若使用旧 Anaconda 遇到 MSVC 导入冲突，先诊断 DLL 版本；已确认系统运行库更新时，可仅对当前进程设置 `ARM_SIM_SYSTEM_MSVC=1` 使用工具内预加载，不复制/替换系统 DLL。

Linux 原生直接显示需检查 X11/Wayland 与后端；无显示服务器使用已确认的无窗口/Web 模式，不声称原生窗口已验收。macOS 非阻塞窗口的主线程约束见官方 NativeViewer 文档。网页可视化不等于 Python 相机可渲染，视觉任务须独立测试摄像机观测。没有匹配 wheel 时停止安装并确认兼容 Python/平台或源码构建，不能静默切换版本。

## API 速查

| 接口 | 本流程用途 |
|---|---|
| `MjModel.from_xml_path`、`MjData` | 编译场景与创建状态 |
| `mj_resetDataKeyframe` | 按名称取 id 后恢复初始 q、ctrl、act 状态 |
| `mj_forward`、`mj_step` | 同步求解瞬时量、执行积分 |
| `m.jnt_qposadr`、`m.jnt_dofadr` | 显式关节与 qpos/qvel 地址映射 |
| `mj_jacSite`、`mj_jacDot` | 末端线角速度和加速度 |
| `mj_contactForce` | 每接触点局部 6D 接触量 |
| `mj_fullM(m,d,M)` | 当前录制器使用的密集惯量矩阵接口，旧版需核对签名 |
| `mju_mat2Quat` | 旋转矩阵到 wxyz 四元数 |
| `launch_web.launch` / `launch_native.launch` | 接入同一个任务循环 |
| `App()`、`SimPlugin`、`StepEvent` | 官方界面和任务步进插件 |
| `RequestPauseEvent`、`StepControlSnapshot` | 暂停和速度同步 |

不要同时包含默认 `StepControl` 与自定义步进器；否则会重复积分。`sync` 自身不推进物理也不等待。接口变更时读当前安装源码和对应官方版本文档，修改后重测数据时序。
