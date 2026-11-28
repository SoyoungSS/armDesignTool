# SRS Lab — 机械臂网页工作台

本目录提供网页和 Node.js HTTP 服务，接入同仓库独立计算 Skill：

- `arm-statics-parametric`：静力学校核、安装姿态与重力分量、临界姿态及驱动静态需求。
- `arm-workspace-parametric`：FK点云、位置/姿态IK、空间与采样灵巧空间分类、不可达与待定点分析。
- `arm-kinematics-dynamics`：轨迹、奇异性与动态载荷。
- `arm-sim-model-generator`：URDF、ROS1、MuJoCo 和 USD 模型导出。

完整启动、示例、CLI、结果说明和排障以[工程根 README](../README.md)为准。

## 启动

Node.js ≥20，无第三方运行依赖，无需先执行 `npm install`。

```powershell
cd "E:\Robot\armDesignTool\srs-lab-source"
$env:STATICS_SKILL_DIR="E:\Robot\armDesignTool\arm-statics-parametric"
npm.cmd start
```

工作空间 Skill 默认自动定位同仓库目录；放在其他目录时设置 `WORKSPACE_SKILL_DIR`。`PORT`默认8787。保持终端打开，停止按Ctrl+C；更新代码后需要重启服务并刷新网页。

- [静力学页面](http://localhost:8787/?model=worksheet#statics)：载入完整输入、应用网页安装设置、运行Skill并预览临界姿态。
- [可达空间页面](http://localhost:8787/?model=worksheet#workspace)：生成当前模型输入或载入示例，再运行工作空间Skill。
- 初次查看空间分类，可载入“选型表 / 空间与灵巧分类”。蓝色位置可达、绿色采样姿态全通过、橙色部分姿态可达、红色几何不可达、灰色待定。

顶部“运行全流程”调用公共模型的静力学、空间与动力学三个 Skill，并汇总规格表；动力学页面“关节模组分析”使用当前轨迹联合计算，在页面内展示规格指导。执行方式可由服务器设置为本地 CLI 或外部 Agent 桥接。

## 代码与测试

`dist/`是当前直接维护的前端文件，`server/index.mjs`提供静态资源及 `POST /api/statics`、`POST /api/workspace`。没有额外前端构建步骤。

```powershell
npm.cmd test
```

测试包括数学回归、最小DOM/Canvas模拟及真实/模拟CLI接口；不等于真实浏览器视觉验收。

## 详细说明

- [静力学安装姿态与重力](../docs/安装姿态与重力配置.md)
- [工作空间Skill设计与使用](../docs/可达工作空间Skill设计与使用.md)
- [静力学输入契约](../arm-statics-parametric/references/input-contract.md)
- [工作空间输入契约](../arm-workspace-parametric/references/input-contract.md)

`package.json`中的 `srs-lab-v10` / `10.0.0` 和 [V10_INTEGRATION.md](V10_INTEGRATION.md)保留历史命名与集成记录，当前功能和操作以根README及专题文档为准。

## Linux 与 Agent 部署

网页导出的 Python 和运行验证由服务器配置，页面不再提供环境选择。部署环境、systemd 示例、Agent 标准输入输出协议和 codeagent 适配步骤见[Linux 部署与外部 Agent 适配](../docs/deployment/linux-and-agent-integration.md)。配置模板见 `server/linux.env.example`，可运行桥接示例见 `server/adapters/stdio-bridge-example.mjs`。

“机械臂构型选择”使用同仓库 `arm-configuration-selection/` 知识库，由真实 Agent 执行。配置 `ARM_SELECTION_AGENT_COMMAND`（或复用 `ARM_AGENT_COMMAND`），协议和结果结构见部署指南第 5 节。

临时测试未指定外部 Agent 时，默认尝试已登录的本机 Codex。项目私有 CLI 安装、登录与覆盖配置见部署指南第 6 节。

“场景仿真”支持一句场景描述 → Agent 自动补齐常规假设并运行 → 真实 MuJoCo Skill 与载荷记录；可继续输入描述修改，也可在更多约束中选择严格审阅模式。需先在“仿真模型生成”成功导出 MuJoCo 并登记模型。异步任务、场景专用解释器、产物持久化和桥接接口见部署指南第 7 节。

空间分析网页合并为“快速工作范围（FK 点云）”和“空间可达与灵巧分析（IK）”。IK 默认区域扫描，支持指定任务点；两种方式均独立检查位置和朝向，体素朝向成功率不混入位置检查次数。旧 API/CLI mode 名称保留。
