# 两个 Skill 共用的离线环境

从仓库根目录执行以下命令。将整个 `arm-mujoco-task-sim` 目录复制到离线机可获得下面所有 Python 安装材料。两台默认机械臂使用同一环境。两个 Skill 的依赖表分别在[任务仿真](../SKILL.md)和[模型生成](../../arm-sim-model-generator/SKILL.md)。

## 从没有 Python 的机器开始

**Windows x64。** 在 PowerShell 中运行。Windows 10/11 的 `tar.exe` 负责展开解释器。脚本校验 Python 归档，默认缓存到当前用户的 `%LOCALAPPDATA%\arm-mujoco-task-sim`，再用该解释器创建项目环境。已装有合适的 CPython 3.12 时，也可直接调用 `scripts/environment.py install`。

```powershell
$skill = (Resolve-Path ./arm-mujoco-task-sim).Path
powershell -NoProfile -ExecutionPolicy Bypass -File "$skill/scripts/install_windows.ps1" -Mode web -Venv .venv
.\.venv\Scripts\python.exe "$skill/scripts/environment.py" probe --mode web
```

如果系统报告缺少 VC++ DLL，核对 `assets/windows-runtime/SHA256SUMS` 与安装程序“数字签名”中的 Microsoft Corporation，再运行 `assets/windows-runtime/vc_redist.x64.exe` 并重试。安装共享系统运行库时 Windows 可能要求管理员权限。可用 `-Cache <目录>` 指定解释器存放位置。项目环境在使用期间应保留其解释器缓存。

**Ubuntu 20.04 / 22.04 / 24.04 x86-64。** 三版共用归档和 wheel。已有发行版自带的 `python3` 时，从仓库根目录运行：

```sh
skill="$PWD/arm-mujoco-task-sim"
python3 "$skill/scripts/install_ubuntu.py" --mode web --venv .venv
.venv/bin/python "$skill/scripts/environment.py" probe --mode web
```

只有 `tar` 和 `sha256sum`、尚未安装 Python 的 Ubuntu 镜像也能从同一份包启动：

```sh
skill="$PWD/arm-mujoco-task-sim"
bundle="$skill/assets/python-linux-x86_64"
runtime="$HOME/.local/arm-cpython-3.12.16"
(cd "$bundle" && sha256sum -c SHA256SUMS) &&
  mkdir -p "$runtime" &&
  tar -xzf "$bundle"/cpython-*.tar.gz --strip-components=1 -C "$runtime" &&
  "$runtime/bin/python3.12" "$skill/scripts/environment.py" install --mode web --venv .venv
```

## 选择包组

下表中 `PY` 代表目标项目环境自己的 Python。Windows 用 `.\.venv\Scripts\python.exe`，Linux 用 `.venv/bin/python`。按需执行 `PY arm-mujoco-task-sim/scripts/install_optional.py <包组>`。安装器检查平台、ABI 和文件 SHA256，pip 只读取本地文件，最后运行 `pip check`。

| 包组 | 何时选 | 装入的内容 |
|---|---|---|
| `model` | 只生成网格、PNG、URDF 和 MJCF | 在空的 3.12 虚拟环境只装 NumPy 和 Pillow。模型动力学运行验证可使用任务环境 |
| `usd` | 生成 USD 或缺少 `pxr` | 在当前环境添加 OpenUSD，并实际创建内存场景 |
| `camera` | 任务需要通过 Python Renderer 取图 | 在已有 MuJoCo 环境添加 GLFW、PyOpenGL，使用目标机的图形后端 |
| `generator-3.3.7` | 复现生成器 `requirements-validation.txt` | 在独立空环境安装固定的 MuJoCo 3.3.7、OpenUSD、模型包及其传递依赖 |

例如，以下命令分别为当前任务环境补充 USD，并创建可独立复现旧版验证的环境：

```powershell
.\.venv\Scripts\python.exe arm-mujoco-task-sim/scripts/install_optional.py usd
.\.venv\Scripts\python.exe -m venv .venv-generator337
.\.venv-generator337\Scripts\python.exe arm-mujoco-task-sim/scripts/install_optional.py generator-3.3.7
```

```sh
.venv/bin/python arm-mujoco-task-sim/scripts/install_optional.py usd
.venv/bin/python -m venv .venv-generator337
.venv-generator337/bin/python arm-mujoco-task-sim/scripts/install_optional.py generator-3.3.7
```

只需模型生成时，可直接用解包后的 `python.exe`（Linux 为 `bin/python3.12`）执行 `-m venv .venv-model`，再以新环境的解释器安装 `model`，需要 USD 时追加 `usd`。Studio 4.x 和 MuJoCo 3.x 使用独立环境，两者提供同名 Python 模块。

## 携带哪些文件

基本任务需要本平台的 Python 目录、`scripts/`、`assets/wheels/<平台>/` 及 `assets/wheels/common/`。可选功能另带 `assets/wheels-optional/bundle.json`、同目录的本平台子目录及 `common/`。使用桌面分拣示例时还需复制任务场景和模型资产。基础包和可选包都保留上游 wheel 内的许可文件。

| 来源和核对方式 | 文件 |
|---|---|
| Astral CPython 同版本发布包 | [Windows SHA256](../assets/python-win_amd64/SHA256SUMS) · [Linux SHA256](../assets/python-linux-x86_64/SHA256SUMS) · [上游发行清单](https://github.com/astral-sh/python-build-standalone/releases/tag/20261110) |
| PyPI 基础 wheel，每个平台 15 个 | [Windows 清单](../assets/wheels/cp312-win_amd64/bundle.json) · [Linux 清单](../assets/wheels/cp312-linux_x86_64/bundle.json) |
| PyPI 可选 wheel，每个平台 10 个，纯 Python 文件跨平台共用 | [文件名、版本与 SHA256](../assets/wheels-optional/bundle.json) · [OpenUSD](https://pypi.org/project/usd-core/25.11/) · [经典 MuJoCo](https://pypi.org/project/mujoco/3.3.7/) |
| Microsoft VC++ x64 安装程序 | [随包 SHA256](../assets/windows-runtime/SHA256SUMS) · [微软发行说明](https://learn.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist) |

## 由目标机器提供的环境

| 使用条件 | 所需环境 |
|---|---|
| Ubuntu 基础环境 | x86-64、glibc ≥2.31、标准 C/C++ 运行库、zlib、libcrypt、tar。按需从对应发行版安装系统库 |
| Web 和原生桌面 | Web 需要浏览器，本地窗口需要桌面会话和显示驱动；Python 相机渲染还需可用的 OpenGL/EGL 等后端 |
| ROS1 RViz / Isaac Sim PhysX | 相应应用及其系统依赖另行部署。OpenUSD 支持 USD 导出和静态验证；Isaac 的物理步进由 Isaac Sim 环境提供 |

Linux 基础任务环境已有三个 Ubuntu 用户空间的断网试跑记录，详见[安装验证](install-api.md)。本次在 Windows 从随包解释器创建新环境，验证了 Web 探测、两轮任务自测、64×64 相机取图、URDF / MJCF / USD 生成及 3.3.7 独立步进。新增 Linux 可选包已核对哈希并完成离线依赖解析，尚未在这三个发行版分别运行新增功能。
