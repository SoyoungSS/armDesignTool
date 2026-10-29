# SRS Lab — 机械臂网页工作台

本目录提供网页和 Node.js HTTP 服务，接入同仓库两个独立计算 Skill：

- `arm-statics-parametric`：静力学校核、安装姿态与重力分量、临界姿态及驱动静态需求。
- `arm-workspace-parametric`：FK点云、位置/姿态IK、空间与采样灵巧空间分类、不可达与待定点分析。

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

顶部“运行全流程”仅运行浏览器概念计算，不自动调用两个独立Skill。旧动力学/选型仍使用网页内核估算；分析页的独立Skill结果不可与之混同。

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
