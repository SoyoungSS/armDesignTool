# Linux 部署与外部 Agent 适配

本文对应仓库当前实现。网页不再提供 Python 环境选择或目标环境运行检查开关；统一由服务器管理员配置。默认执行本地确定性 Skill，可切换至外部 Agent 桥接模式。这里的 Agent 桥接协议是本项目接口，不假定某个 codeagent 产品具有特定 SDK 或 CLI。

## 1. 环境与目录

整仓部署，保留 `srs-lab-source/`、`shared/`、`examples/` 及各 `arm-*/` Skill 的相对目录。不要只复制前端 `dist/`。

| 项目 | 当前代码要求 / 用途 |
|---|---|
| Node.js | ≥20；HTTP 服务和静力学、空间、动力学 CLI，无需安装 npm 第三方包 |
| Python | ≥3.10；仿真模型生成 |
| NumPy | ≥1.24、<3；几何、惯量、计算 |
| Pillow | ≥10；生成器依赖，见 requirements.txt |
| usd-core | 当前验证依赖锁定 25.11；生成 Isaac USD，提供 `pxr` |
| MuJoCo | 当前验证依赖锁定 3.3.7；目标运行验证 |
| ROS1 / RViz | 打开导出的 ROS1 包时使用，服务器仅生成文件无需安装 |
| Isaac Sim | 离线生成 USD 无需安装；实际 PhysX/Articulation 验证需另在目标 Isaac 环境运行 |

版本来自仓库 `arm-sim-model-generator/scripts/requirements*.txt`，并非对未来版本兼容性的承诺。可先在目标 Linux 发行版和 CPU 架构上验证这些依赖是否能安装。默认全部导出时需安装 usd-core，否则 USD 可能未生成；下载成功不等于全部目标均成功，检查 ZIP 中 `report/manifest.json`。

在仓库根目录操作（路径示例 `/srv/armDesignTool`）：

```bash
cd /srv/armDesignTool
python3 -m venv .venv-arm-sim
.venv-arm-sim/bin/python -m pip install -r arm-sim-model-generator/scripts/requirements-validation.txt
.venv-arm-sim/bin/python -c 'import sys,numpy,PIL,mujoco; from pxr import Usd; print(sys.version,numpy.__version__)'
```

只生成 URDF/ROS1/MJCF 的基础依赖可安装 `requirements.txt`；当前网页默认同时选择 USD，建议使用上面的完整依赖文件。MuJoCo/Isaac GUI 和 GPU 配置属于目标模拟器部署，不能用离线导出成功替代。

## 2. 服务器配置

提供配置模板：`srs-lab-source/server/linux.env.example`。以下配置由服务进程读取，修改后重启；网页不能覆盖它们。

| 环境变量 | 默认 / 用途 |
|---|---|
| `ARM_SIM_PYTHON` | 仿真 Python 可执行文件的**绝对路径**；生产部署明确指定，指定后不会因缺依赖切换到其他解释器 |
| `ARM_SIM_RUNTIME_CHECKS` | `false`；只有字符串 `true` 开启生成器 `--runtime` 验证，由服务器决定 |
| `ARM_EXECUTION_MODE` | `local` 或 `agent` |
| `ARM_AGENT_COMMAND` | agent 模式下桥接程序的可执行文件，建议绝对路径 |
| `ARM_AGENT_ARGS` | 桥接程序参数的 JSON 字符串数组，不是 shell 命令 |
| `ARM_WEB_HOST` / `PORT` | 建议反向代理部署设为 `127.0.0.1` / `8787` |
| `STATICS_SKILL_DIR` | 静力学 Skill 目录，默认同仓库目录 |
| `WORKSPACE_SKILL_DIR` | 空间 Skill 目录，默认同仓库目录 |
| `MOTION_SKILL_DIR` | 动力学 Skill 目录，默认同仓库目录 |
| `STATICS_TIMEOUT_MS` | 默认 120000；静力学及模组联合计算中单个 Skill 超时 |
| `WORKSPACE_TIMEOUT_MS` | 默认 295000；普通空间分析，体素灵巧空间保留不限时和手动取消 |
| `ARM_DEMO_RESULTS` | 三维模型/场景回放的预计算结果根目录；期望含 `viewer/worksheet/`、`viewer/fr3/` |
| `WEB_ORIGIN` | 可选的已有分析接口跨域来源；网页与服务优先保持同源 |

未设置 `ARM_SIM_PYTHON` 时仍保留开发用自动查找。生成超时当前为 180 秒、ZIP 打包为 60 秒；Agent 桥接开销也计入同一个执行超时。ZIP 用服务器 Python 本地打包，环境探测也在服务器本地执行，不经过 Agent。

```bash
cd /srv/armDesignTool
ARM_SIM_PYTHON=/srv/armDesignTool/.venv-arm-sim/bin/python \
ARM_SIM_RUNTIME_CHECKS=false ARM_EXECUTION_MODE=local \
ARM_WEB_HOST=127.0.0.1 PORT=8787 npm --prefix srs-lab-source start
```

也可使用 systemd：

```ini
[Unit]
Description=Arm design tool
After=network.target

[Service]
Type=simple
User=armtool
WorkingDirectory=/srv/armDesignTool
EnvironmentFile=/etc/arm-design-tool.env
ExecStart=/usr/bin/node /srv/armDesignTool/srs-lab-source/server/index.mjs
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

`ExecStart` 的 Node 路径须替换为服务器实际路径。服务用户需能读取整仓、执行 Python/Agent，并写入系统临时目录；临时模型任务结束后会清理。生产入口可用 Nginx 转发到 127.0.0.1:8787，上传限制至少 2 MiB，普通生成接口的代理读取超时建议至少 300 秒；体素不限时计算需另行调整代理超时。服务当前无内置账号鉴权，多用户或公网入口应由反向代理提供认证和访问控制。

网页保留默认全格式导出、默认等效密度 2000 kg/m³、固定基座等规则；服务器不需要为未填质量属性的固定基座构造惯量。

## 3. 外部 Agent 接入点

`server/execution-adapter.mjs` 的 `executeSkill()` 是统一执行接口，已接入：

- `/api/statics`、`/api/workspace`、`/api/motion` 的独立分析。
- `/api/module-specs` 的三个 Skill 顺序联合计算。
- `/api/simulation-export` 的仿真模型生成。

`GET /api/execution` 返回当前执行模式、桥接协议版本和支持操作，不返回服务器命令路径或凭据。现有业务 HTTP 请求与响应格式不变；Agent 也可作为 HTTP 客户端直接调用这些业务接口。建议选择一种模式，避免把同一个请求递归提交给自身。

### 标准输入输出协议：arm-skill-execution/1

每个任务启动一个桥接进程，不使用 shell。服务器通过 stdin 发送单个 JSON 对象并关闭 stdin：

```json
{
  "protocol": "arm-skill-execution/1",
  "task_id": "server-generated-uuid",
  "operation": "analysis",
  "skill": "arm-statics-parametric",
  "timeout_ms": 120000,
  "invocation": {
    "command": "/usr/bin/node",
    "args": ["scripts/run.mjs", "--input", "/tmp/task/input.json", "--output", "/tmp/task/result.json"],
    "cwd": "/srv/armDesignTool/arm-statics-parametric"
  },
  "artifacts": {
    "input_path": "/tmp/task/input.json",
    "output_path": "/tmp/task/result.json"
  }
}
```

`operation` 为 `analysis`、`simulation.generate` 或下文的 `configuration.select`。仿真生成时 command 为服务器指定的 Python，args 为固定 generate.py 参数，`output_path` 是目录而非 JSON 文件，目录中需有生成器原有的 `latest_attempt.json` 及运行产物。`timeout_ms` 可以为 null，表示沿用允许不限时的任务。

桥接器在完成文件写入后，stdout **仅返回一个 JSON 对象**；日志写 stderr：

```json
{"protocol":"arm-skill-execution/1","task_id":"server-generated-uuid","ok":true}
```

业务失败可返回：

```json
{"protocol":"arm-skill-execution/1","task_id":"server-generated-uuid","ok":false,"error":{"message":"任务失败原因"}}
```

输入校验失败时在 error 中加 `code:"INVALID_INPUT"`，服务器据此保留现有接口的 422 状态；其他 Agent 执行失败按服务错误处理。

成功标记不替代真实计算：必须写出原 Skill 格式的产物，不能用自然语言描述冒充结果。服务器仍读取、校验结果并执行联合计算的模型哈希一致性检查。错误协议、task_id 不一致、非 JSON 输出、进程异常都会报错，**不自动回退本地执行**。

调用命令、工作目录和产物路径均由服务器生成，不允许前端指定执行程序。Agent 凭据由服务器环境或桥接器配置管理，无需传给浏览器。

### 可运行示例与 codeagent 适配位置

仓库提供 `server/adapters/stdio-bridge-example.mjs`，它实现上述协议并转发到原有本地 Skill，用于验证桥接链路；**它不包含真实 codeagent 接入**。

```bash
cd /srv/armDesignTool
ARM_EXECUTION_MODE=agent \
ARM_AGENT_COMMAND=/usr/bin/node \
ARM_AGENT_ARGS='["/srv/armDesignTool/srs-lab-source/server/adapters/stdio-bridge-example.mjs"]' \
ARM_SIM_PYTHON=/srv/armDesignTool/.venv-arm-sim/bin/python \
ARM_WEB_HOST=127.0.0.1 npm --prefix srs-lab-source start
```

接入实际 codeagent 时复制该示例，在 `runProcess()` 调用处替换为对应 SDK、CLI 或内部 RPC。不同产品的名称、认证及调用方式可能不同，待确定具体实现后只需改这个桥接器：

1. 读取任务 JSON，保留 task_id、operation 和输入文件内容。
2. 把任务交给 Agent；要求执行仓库内对应确定性 Skill，保留物理单位、固定基座和估算来源。
3. 等待任务完成，将结果 JSON 或完整生成目录写回指定 output_path。
4. 完成后返回协议响应；禁止先回复成功再后台写文件。

当前为**同机文件协议**。Agent 在容器或远端时，需要共享相同路径挂载，或由桥接器自行上传输入、映射 invocation 路径、下载产物并重写仿真 `latest_attempt.json` 中的 `run_directory` 为服务器可读目录。不要直接把远端路径返回服务器。当前网页等待同步 HTTP 完成，不是持久化任务队列；服务重启会中断请求，长时间远程任务需要后续另加队列和状态接口。

Linux 中超时、取消会杀死本次执行的进程组。桥接器本地子进程应留在同一进程组；远端 SDK 还需自行接入远端任务取消和租约/超时。示例使用 `processGroup:false` 保证子进程随桥接进程一起清理。代码中已有异常、超时、取消和协议校验测试。

## 4. 部署验收

```bash
cd /srv/armDesignTool/srs-lab-source
npm test
curl -fsS http://127.0.0.1:8787/api/execution
```

分别以 local 和 agent 模式运行已有模型，核对分析结果和模型哈希；默认固定基座应无基座惯量缺项，运动刚体缺项仍应按规则估算或报告。分别检查 URDF、ROS1、MuJoCo、USD 的 manifest 状态。开启 `ARM_SIM_RUNTIME_CHECKS=true` 后再检验运行验证状态，离线 USD 生成不代表 Isaac Sim 的实际步进已经验收。

## 5. 机械臂构型选择页面

左侧“机械臂构型选择”不要求先建立具体模型。字段涵盖场景、动作、位姿、工件/工具质量（kg）、作业距离（mm）、周期（s）、部署、环境、控制、硬约束、偏好和既定构型。空白保持未知，不采用示例的 7 轴、3 kg 或 600 mm。

知识库随仓库部署于 `arm-configuration-selection/`，来自已安装的同名 Skill 1.4.0。入口为 SKILL.md，专题资料在 references/。新增接口：

- `GET /api/configuration-selection/health`：Agent 命令和核心知识库是否配置；不表示真实推理能力已验收。
- `POST /api/configuration-selection`：场景需求 JSON → `{ok,result,input,execution}`。每个服务进程同时允许一个构型任务，忙时 503；取消请求会停止桥接进程组。
- `/api/execution` 的 operations 新增 `configuration.select`。

此 Skill 是知识判断流程，**没有可替代推理的本地计算 CLI**。必须配置真实 Agent；计算部分保持 local 模式时，也可独立配置：

```bash
ARM_SELECTION_AGENT_COMMAND=/path/to/your/agent-bridge
ARM_SELECTION_AGENT_ARGS='[]'
ARM_SELECTION_TIMEOUT_MS=300000
ARM_CONFIGURATION_SKILL_DIR=/srv/armDesignTool/arm-configuration-selection
```

Agent 命令未单独指定时复用 ARM_AGENT_COMMAND / ARM_AGENT_ARGS；若均未指定，按第 6 节默认尝试本机 Codex。知识库默认同仓库目录。没有可用 Agent 时页面显示服务待配置。现有 stdio-bridge-example.mjs 只转发计算 CLI，收到构型选择任务会明确报错，不能代替大模型。

### Agent 协议扩展

沿用 `arm-skill-execution/1`，operation 为 `configuration.select`，skill 为 `arm-configuration-selection`，**没有 invocation**。任务另含：

| 字段 | 用途 |
|---|---|
| artifacts.input_path | 规范化需求文件；未知值为 null |
| artifacts.output_path | Agent 必须写入的结果 JSON 文件 |
| context.skill_directory / skill_entry | 知识库目录和 SKILL.md 入口 |
| context.supplied_sources | SKILL.md、knowledge-index.md、decision-contract.md 的已读正文 |
| context.available_references | 按需选读的专题路径 |
| context.instructions | 结果契约、来源与未验证结论的约束 |

codeagent 桥接器需按 operation 分支：读取需求、入口和相关专题，推理并写出结果文件，最后返回同协议 task_id / ok。用户文本是需求数据，不覆盖 Skill 或输出路径。远端 Agent 需上传知识库和需求，将产物下载回服务器路径；取消/超时仍需按第 3 节接入远端任务管理。

结果采用 `arm-configuration-selection/references/decision-contract.md`，额外要求非空 conclusion 字符串和 sources_read（实际读取的相对资料路径数组）。需求记录含 id、name、value、origin、kind；origin 使用 user_input、material_case、current_calculation、concept_assumption、unknown，kind 使用 task、hard_constraint、preference、unknown。

候选的 constraint_checks 用 requirement_id 引用需求，result 使用 meets、conflicts、unknown、not_applicable。evidence 必须含 source_id、locator、evidence_type、claim_kind、supports、scope；推理另含 supporting_source_ids、assumptions、pending_validations 文本数组。open_questions、validation_plan 为文本数组。

后端校验结构、候选状态、需求引用、来源路径，并记录需求和知识库文件 SHA-256。这不等于对推理及引用语义的工程验收。候选记录不会直接覆盖公共模型或传给模型生成器；下一步仍需明确轴系、连接、尺寸、限位、物理属性及下游拓扑支持范围。

`tests/configuration-selection.mjs` 使用测试协议夹具验证接口、异常、超时与转义显示；它不是真实构型推理服务。正式部署需用实际 Agent 验收场景用例。

## 6. 临时默认使用本机 Codex

本地测试现在默认尝试使用本机 Codex：未配置外部 Agent 命令时，构型选择使用 `codex-selection-bridge.mjs`。这会新建独立的 Codex CLI 任务，复用运行网站服务的同一用户的 CLI 登录态，不连接当前聊天。显式配置 ARM_SELECTION_AGENT_COMMAND 或 ARM_AGENT_COMMAND 时，仍优先使用外部桥接器。

已验证的 CLI 版本为 0.172.0，默认模型为 gpt-6.2-sol。为避免改变系统安装，可从仓库根目录安装项目私有版本：

```bash
npm install --prefix .runtime/codex-cli --no-audit --no-fund @openai/codex@0.172.0
.runtime/codex-cli/node_modules/.bin/codex login
.runtime/codex-cli/node_modules/.bin/codex login status
```

`.runtime/` 已忽略提交，不包含在部署源码中；服务器需要自行安装 CLI，并使用服务用户登录。已有登录可直接复用，无需将令牌填写进网页。不要复制或提交认证文件。模型可用性和额度以该登录账号为准。

| 配置 | 用途 |
|---|---|
| ARM_CODEX_CLI | 可选：指定 Codex 可执行文件的绝对路径；默认先查项目 .runtime，再查 PATH |
| ARM_CODEX_MODEL | 默认 gpt-6.2-sol；可指定账号有权使用的模型 |
| ARM_SELECTION_PROVIDER | 未指定外部桥接器时默认 codex；设为 external 可关闭自动 Codex 接入 |
| ARM_SELECTION_TIMEOUT_MS | 未配置时默认 300000（5 分钟）；反向代理应配置更长的读取超时 |

适配器通过官方 `codex exec` 非交互模式运行，使用 `--ignore-user-config`、`--sandbox read-only`、`--ephemeral` 和 `--output-schema`。不加载个人 MCP 配置；认证仍使用 CLI 登录态。模型只读知识库，由调用器将最终 JSON 写到指定任务文件。调用会使用该账号的 Codex 额度。

健康检查确认 CLI 版本和登录状态，缓存 15 秒；不对每次检查发起模型请求。未安装、未登录或版本过旧时，页面显示原因。网页字段和其他计算 Skill 的执行模式不变。更换服务器或服务用户后需重新核对登录权限。

官方说明：[Codex 非交互模式](https://learn.chatgpt.com/docs/non-interactive-mode)、[Codex 模型](https://learn.chatgpt.com/docs/models?surface=cli)。

## 7. 场景描述驱动的仿真

“场景仿真”页面现使用本工程 `arm-mujoco-task-sim/SKILL.md`。旧的分拣预计算回放不再作为场景页面的主体；“机械臂 3D 模型”页面保持独立。

### 网页操作

1. 在“仿真模型生成”中生成包含 MuJoCo 的完整模型；成功后服务器自动登记，保留原模型、manifest、归一化参数、载荷测点与驱动报告。已有旧 ZIP 不会被默认为新模型，请重新生成登记。
2. 场景页选择模型（留空自动使用最近导出的模型），输入一句场景描述，点击“开始场景仿真”。其他信息均可留空。
3. 默认采用概念试算：Agent 按常规补齐未指定的轮数、节拍、运动和环境假设，生成方案后直接准备并运行。缺少真实夹具或接触细节时可采用明确标注的代表性载荷代理；待补问题作为建议保留，不阻断普通试算。
4. 结果页显示本次方案、假设、实际结果和载荷谱。输入新的描述后点击“按新描述重新仿真”，新任务带上前次输入、方案与结果摘要，支持逐步修正。更多约束中可切换严格模式，先补齐关键参数、审阅方案再运行。
5. Agent 只准备本次项目的 scene、assets、plan、controller 和合同。服务器独立执行环境探测、inspect_generator、合同检查、预检、Skill Session 和 analyze；准备或预检失败时概念模式自动反馈给 Agent 修复一次。正式物理运行不以修改判据的方式反复重试。
6. 实际姿态通过 Recorder 采样钩子记录，网页可播放/暂停/定位/复位视角。关节时序、六维接口载荷、载荷谱、图表及诊断可下载，任务失败也保留已经产生的数据。

默认概念轮数为 1，代表周期约 5 秒、稳定时间约 1 秒、最大时长 10 秒（受服务器上限约束），采用低速、小幅平滑运动；具体可行值由 Agent 按模型与用户约束确定。显式输入优先，估算不冒充实测或逐项人工确认。模型不可读取、编译失败或硬约束冲突仍返回诊断；不会跳过碰撞检查或编造成功。

本页面采用 **headless 后台物理计算 + 浏览器实测姿态回放**；不启动官方 WebViewer。最大仿真时长是超时限制，不是成功判据。控制器不得覆写 qpos、qvel、time 或自行 mj_step。任务结果、载荷覆盖和谱导出状态分别记录；代理载荷不能等同于原任务接触验证、实机或寿命验收。具体默认策略见 `arm-mujoco-task-sim/references/concept-trial.md`。

### 服务器配置与依赖

| 配置 | 当前默认与用途 |
|---|---|
| ARM_SCENE_STORAGE | `<repo>/.runtime/scene-simulations`；完整模型和任务记录的持久化目录 |
| ARM_SCENE_SKILL_DIR | 同仓库 arm-mujoco-task-sim |
| ARM_SCENE_PYTHON | 优先使用此解释器，其次 ARM_SIM_PYTHON，再查项目 .runtime/scene-python、.venv-arm-sim，最后 python3 |
| ARM_SCENE_AGENT_COMMAND / ARM_SCENE_AGENT_ARGS | 场景专用外部 Agent 桥接；未指定时复用 ARM_AGENT_COMMAND / ARM_AGENT_ARGS，再默认本机 Codex |
| ARM_SCENE_AGENT_TIMEOUT_MS | 每阶段 Agent 超时，默认 300000 ms |
| ARM_SCENE_RUN_TIMEOUT_MS | 本地实际运行与分析进程超时，默认 300000 ms |
| ARM_SCENE_MAX_SIM_SECONDS | 网页允许单次任务的最大物理时间，默认 120 秒 |

运行环境需 MuJoCo、NumPy、Matplotlib；不会在网页请求期间自动安装依赖。当前项目已用 Skill 自带 Linux 离线包创建私有测试环境，部署时可在仓库根目录复现：

```bash
XDG_CACHE_HOME="$PWD/.runtime/cache" python3 arm-mujoco-task-sim/scripts/install_ubuntu.py \
  --mode headless --venv "$PWD/.runtime/scene-python"
.runtime/scene-python/bin/python arm-mujoco-task-sim/scripts/environment.py probe --mode headless
```

按该 Skill 当前离线包安装的是 CPython 3.12、MuJoCo Core 4.0.1、NumPy 和绘图依赖；不要覆盖已有 MuJoCo 3.x 环境，模型生成和场景任务可使用不同解释器。环境与平台范围以 `arm-mujoco-task-sim/references/install-api.md` 为准。

scene storage 需由服务用户可读写，并在服务升级/重启后保留。当前没有自动保留期清理；管理员应按磁盘容量制定归档/删除策略。服务进程重启时未完成的任务标为 interrupted，不偷用旧结果。运行是异步任务，浏览器每约 1.5 秒查询状态，长任务不占用单次长 HTTP 请求；取消接口会停止当前本地进程组，远端 Agent 的取消仍由外部桥接器适配。

### HTTP 与 Agent 接口

- `GET /api/scene-simulation/health`：场景 Agent/Skill 配置状态。
- `GET /api/scene-simulation/models`：已登记的完整 MuJoCo 模型。
- `POST /api/scene-simulation/plan`：创建异步任务，返回 202 与 job.id；网页传入 `assumption_policy:"concept",auto_run:true` 自动继续执行，严格模式传 `assumption_policy:"strict"`。可选 `previous_job_id` 传递上次任务上下文。
- `GET /api/scene-simulation/jobs/:id`：阶段、方案、缺项、错误及实际结果。
- `POST /api/scene-simulation/jobs/:id/run`：`{"plan_sha256":"已审阅方案哈希"}`，只接受当前完整方案。
- `POST /api/scene-simulation/jobs/:id/cancel`：取消本次任务。
- `GET /api/scene-simulation/jobs/:id/artifact?path=...`：读取本次项目内的受限产物，拒绝越界路径与符号链接逃逸。
- `GET /api/scene-simulation/jobs/:id/archive`：下载完整项目 ZIP，运行中不可下载。

沿用 `arm-skill-execution/1`，新增 **scene.plan**（只读方案）与 **scene.prepare**（项目代码准备）两个 operation，没有 invocation。`context.paths` 提供 Skill、项目、原模型、解释器、inspection、approval 路径，`supplied_sources` 提供核心 Skill 资料；输入/输出仍通过 artifacts 指定。真实 Agent 应按这两个操作适配，不能把 stdio-bridge-example 的 CLI 转发当场景生成器。

scene.plan 结果是 schema_name=arm-scene-plan、schema_version=1.0，含 summary、strategy、cycles、max_time_s、phases（name/action/completion/on_failure）、success_criteria、physics、assumptions、missing_inputs、ready_to_run、limitations、sources_read。另含 simulation_scope（task/load_proxy）、follow_up_questions、blocking_issues。scene.prepare 返回 ready、summary、missing_inputs、simulation_scope、assumptions、limitations、sources_read，并在 project 中写入具体文件；它不执行正式仿真、不写运行报告、不修改 Skill 或上游原模型。服务器依据真实运行请求填入 confirmation；概念自动运行记录 delegated_concept_defaults 委托依据与 assumptions.json，严格模式记录方案审阅依据，然后运行确定性 Skill。普通缺项进入 follow_up_questions，真实不可执行条件进入 blocking_issues；prepare 应读取 approved-task.json 中可能存在的 repair_feedback，仅修复本次项目。

本地测试用协议夹具准备单轴保持控制器，再实际调用 MuJoCo 与本工程记录器，验证正负例成功判据、全关节载荷谱及浏览器回放；此测试不向外部 Agent 发送项目资料。正式 Agent 场景需要在用户明确提供并授权的模型和场景上另行验收。

### 真实 Codex 链路验收记录（2026-11-28）

经用户明确授权，将本工程 Skill 文档、开发用单轴模型和测试描述用于一次真实 Codex 场景试跑。测试使用隔离存储 `.runtime/scene-codex-smoke/`，未使用业务模型。Codex 独立完成了场景计划和任务适配器，服务器随后实际运行本工程 Skill。

- 工况：固定基座单轴 J1 初始角保持，无额外工具、物体、环境接触或随机化。
- 批准范围：一轮；0.2 秒后角误差严格小于 0.02 rad 才成功；0.4 秒为超时上限，保留原模型参数和积分步长。
- 实测：200 个物理步、201 个记录样本，结束时间约 0.2 秒；J1 角误差 0 rad；完成 1 轮，任务与碰撞检查均通过。
- 成功判据自检：初始角达到保持时间为正例；容差边界、超差、保持时间不足为负例，四例均通过。
- 全关节载荷覆盖为 complete，记录 J1；实际六维接口载荷文件已产生；load_spectrum.json 为 exported。
- 实测回放文件包含 6 帧，时间范围 0–0.2 秒，来自本次物理姿态。

可复现测试脚本和结果保存在本机忽略提交的 `.runtime/scene-real-smoke.mjs`、`.runtime/scene-codex-smoke-result.json`。本次证明了单轴保持工况的真实 Agent—Skill—物理执行—产物链路；不扩大为抓放/擦拭/插装等其他任务已通过。浏览器访问权限未获授予，实际播放、暂停等 UI 操作没有在此次联网验收中人工验证；不将回放文件生成等同于 UI 交互验收。

### 一句话自动概念试算验收（2026-11-28）

使用同一开发用单轴合成模型，在隔离存储 `.runtime/scene-concept-smoke/` 中只提交一句低速往复描述与 `auto_run:true`，未填写轮数、时长、控制器或成功判据。真实 Codex 自动选择 1 轮、J1 转出 0.10 rad 后返回、5 s 运动及约 1 s 稳定，并生成具体假设和正负例判据。服务器随后独立预检和物理执行；未要求用户补充信息或再次点击运行。

本次实际完成 6000 个物理步、6001 个样本，约 6 s；完成 1 轮，末端角误差约 0.0000183 rad，路径及运行碰撞检查通过。全关节载荷覆盖完整，载荷谱已导出，实际姿态回放已生成。脚本为 `.runtime/scene-concept-smoke.mjs`，记录为 `.runtime/scene-concept-smoke-result.json`。该记录验证默认自动执行链路，不代表其他真实抓放或接触任务已验收。

## 8. 导出模型与三维查看器的数据联通

模型导出、三维查看与场景仿真共用服务器的完整 MuJoCo 模型登记目录。成功生成 MuJoCo 文件后，即使碰撞验收失败也保留几何和检查报告；三维预览显示检查状态，不把几何可读取等同于仿真验收通过。

“机械臂 3D 模型”页面从 `GET /api/generated-models` 读取已导出列表，优先打开本次新生成的模型；没有新导出时保留有效选择，默认使用最近登记的模型。参考示例列在独立分组，使用旧的 demo-assets 预计算数据。

- `GET /api/generated-models/:id/viewer`：将对应完整输出中的真实 MJCF/网格转换为浏览器几何和初始/几何零位两组姿态。转换使用场景仿真 Python 配置，在服务器本地执行，不调用外部 Agent。
- `GET /api/generated-models/:id/validation`：该模型原始检查报告。
- `GET /api/generated-models/:id/manifest`：该模型原始导出清单。

查看数据缓存在模型登记目录内，按 XML 和 manifest 内容签名复用；同一模型并发请求共用转换任务。模型 ID 和资源路径均受限于登记目录。网页提供模型切换、列表刷新、姿态切换和复位视角，导出完成后记录本次模型 ID，下一次打开三维页自动选择它。

当前浏览器几何转换依赖完整 MuJoCo 输出及可导入 MuJoCo/NumPy 的 Python。仅导出 URDF/ROS1/USD、或旧版只下载 ZIP 且服务器未登记的模型，不会自动出现在此列表，需要生成一次包含 MuJoCo 的完整输出。已经登记的模型无需重填参数或重新生成。

## 9. 全关节数据交付

场景运行结束后 analyze 自动生成内嵌所有关节及末端曲线的 `reports/report.html`，每关节 `data/joint_modules/*_state.csv` 与 `*_actuators.csv`、全关节宽表、模组统计、字段字典和 SHA-256 索引。报告生成器仅使用 Python 标准库，复用已存在的 SVG，不需要 Agent 或再次运行 MuJoCo。

`POST /api/scene-simulation/jobs/:id/exports` 接收空 JSON，从该任务已有实际运行目录补齐数据，返回更新后的 job；运行中的任务拒绝整理，同任务并发请求共用一次整理，整理过程中暂不下载 ZIP。旧任务可在页面点击“整理全关节 HTML 与 CSV”。成功/失败的物理结果保持原值，导出独立记录 exported/partial/failed。
