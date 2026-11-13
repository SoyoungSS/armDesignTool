# 安装、模式与官方接口

核实日期：2026-11-12。参考版本 MuJoCo 4.0.1；这是版本基线，不是保证所有未来版本/平台都通过测试。执行时查询官方版本兼容性，记录平台/架构、Python、安装命令、实际包版本、渲染后端、探测与启动结果。先问用户选模式，再创建独立环境，不能擅自安装整套大依赖。

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

本工具用 Python 3.11、MuJoCo 4.0.1、NumPy 2.x 作为验证基线。使用项目目录下的新环境，不覆盖现有 Python/全局包。Windows PowerShell：

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install "mujoco-studio==4.0.1" "numpy>=2,<3"
.\.venv\Scripts\python.exe -m pip check
.\.venv\Scripts\python.exe -m pip freeze > environment-lock.txt
```

Linux / macOS：

```sh
python3 -m venv .venv
.venv/bin/python -m pip install 'mujoco-studio==4.0.1' 'numpy>=2,<3'
.venv/bin/python -m pip check
.venv/bin/python -m pip freeze > environment-lock.txt
```

`mujoco-studio` 安装官方 core 与 wasm 组件；实际依赖以包元数据和 freeze 为准。只做已获准的无窗口计算可装 `mujoco-core==4.0.1`，仅依赖 NumPy；SVG 导出不需 Matplotlib。需要经典渲染器时选择 `mujoco-core[rendering]` 或完整 `mujoco`，不能混用不同版本的 core/studio。视觉感知、优化、PNG、视频的额外依赖按策略单独列出。

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

这两条只用于查看模型。完整任务使用 `run_task.py --project <project>`，mode 来自已确认合同。原生窗口在 macOS 使用 `mjpython`。运行器默认暂停，官方工具条 Play 开始。Reset 在同一查看器内重建任务控制器、初始物理状态和记录器，生成独立 run，并暂停等待再次播放。Reset 后保留原 run 的记录。历史跳转、交互扰动、外部模型替换和物理选项改动在记录会话中被锁定。

## 任务入口与会话清理

- 为用户提供项目启动入口。Web 默认用 `--port 0 --open-browser`，由官方查看器直接绑定可用端口并打开实际地址，监听限制在本机回环。启动输出实际访问地址和 run 路径。只清理本任务可确认归属的进程。
- 在官方工具条验证 Play、Pause 和 Reset。可在查看器里加入随物理时间推进的滚动关键曲线，例如关节驱动力矩与 TCP 位置。Reset 清空上一 run 的实时曲线。
- Web 可用 `File → Quit` 或启动终端的 Ctrl+C 退出，默认在浏览器断开超过 45 秒后自动退出。退出时封存记录并释放监听。用实际场景完成验收后退出测试会话，确认监听释放。交付后由用户自行启动。
- PNG 曲线需要 Matplotlib。可装入项目环境，或设置 `ARM_SIM_ANALYSIS_PYTHON` 指向已验证的绘图环境。SVG 仅依赖 NumPy。

独立桌面应用也可从官方发布页获取对应系统和架构的包：Windows zip 内 `bin/mujoco_studio.exe`，Linux tar.gz 内 `bin/mujoco_studio`，macOS dmg 的 `MuJoCoStudio.app`。不假定安装独立应用能运行 Python 策略，任务应选对应 Python NativeViewer 链路。

## 探测与故障

先在选定 Python 中检查 `mujoco.__version__`、`MjModel` 编译、`mj_jacDot` 可调用。Web 模式额外导入 `mujoco.studio.web.headless_ui/state_payload` 并实际打开场景；Native 模式实际创建窗口。不能用 import 成功替代可视化验收。

4.0.1 官方发布说明修复了 Windows WebViewer 组件分发问题。若使用旧 Anaconda 遇到 MSVC 导入冲突，先诊断 DLL 版本；已确认系统运行库更新时，可仅对当前进程设置 `ARM_SIM_SYSTEM_MSVC=1` 使用工具内预加载，不复制/替换系统 DLL。

Linux 原生直接显示需检查 X11/Wayland 与后端；无显示服务器使用已确认的无窗口/Web 模式，不声称原生窗口已验收。macOS 非阻塞窗口的主线程约束见官方 NativeViewer 文档。网页可视化不等于 Python 相机可渲染，视觉任务须独立测试摄像机观测。没有匹配 wheel 时停止安装并确认兼容 Python/平台或源码构建，不能静默切换版本。

## API 速查（4.0.1）

| 接口 | 本流程用途 |
|---|---|
| `MjModel.from_xml_path`、`MjData` | 编译场景与创建状态 |
| `mj_resetDataKeyframe` | 按名称取 id 后恢复初始 q、ctrl、act 状态 |
| `mj_forward`、`mj_step` | 同步求解瞬时量、执行积分 |
| `m.jnt_qposadr`、`m.jnt_dofadr` | 显式关节与 qpos/qvel 地址映射 |
| `mj_jacSite`、`mj_jacDot` | 末端线角速度和加速度 |
| `mj_contactForce` | 每接触点局部 6D 接触量 |
| `mj_fullM(m,d,M)` | MuJoCo 4.0 的密集惯量矩阵接口 |
| `mju_mat2Quat` | 旋转矩阵到 wxyz 四元数 |
| `launch_web.launch` / `launch_native.launch` | 接入同一个任务循环 |
| `App()`、`SimPlugin`、`StepEvent` | 官方界面和任务步进插件 |
| `RequestPauseEvent`、`StepControlSnapshot` | 暂停和速度同步 |

不要同时包含默认 `StepControl` 与自定义步进器；否则会重复积分。`sync` 自身不推进物理也不等待。接口变更时读当前安装源码和对应官方版本文档，修改后重测数据时序。
