# SRS Lab V10 — 原网站静力学 Skill 接入

## 修改范围

以用户提供的 V9 在线源码 ZIP 为基线，原样保留 `dist/engine.js`、`dist/scene3d.js`、样例及原有界面结构。`dist/app.js` 中的“静力学分析”页面改为单一外部 Skill 来源，新增 `dist/statics-client.mjs`、`server/index.mjs`。旧静力学报告快照文件保留作历史资源，但 V10 的分析界面不读取它。

- `POST /api/statics` 启动 `STATICS_SKILL_DIR/scripts/run.mjs`，输出唯一的 `statics_result.json`，附运行 ID、版本、时间、耗时、哈希。调用失败返回错误，没有网页后备静力学数值。
- 静力学页面支持输入文本、加载 3.612kg 选型表示例和上传 JSON / JSONC；显示 J1–J7 已检查最大绝对扭矩、输出端需求、能力、裕量及临界姿态；可导出原始 Skill JSON。
- 临界姿态数据 `critical_pose.q` 写入原始站点共享关节数组 `q`，由原 `Arm3D` 渲染器显示，不新增计算引擎或 Three.js 场景；检查七轴与关节限位。
- 模型参数修改或重新计算后，上一份 Skill 结果立即失效。
- 动力学、运动学、可达性、旧概念模组选型等原功能保留。**注意：旧 `SRS.run()` 仍会计算供原动力学和旧选型使用的重力估算，这些不是 Skill 校核结果，不能混同。**要实现所有下游模块都只复用 Skill 静力结果，需再对 `engine.js` 下游数据契约做一次单独改造。

## 启动

Node.js 18+：

```bash
export STATICS_SKILL_DIR=/absolute/path/to/arm-statics-parametric
npm start
# 打开 http://localhost:8787/?model=worksheet#statics
```

Windows PowerShell：

```powershell
$env:STATICS_SKILL_DIR='C:\\path\\to\\arm-statics-parametric'
npm start
```

设置 `PORT` 修改端口；`STATICS_TIMEOUT_MS` 修改 CLI 超时（默认 120 秒）。真实 Skill 必须含 `scripts/run.mjs`，且支持 `--input --output --report-dir`；返回结果需含 `summary[]`、`joint_names[]`、`summary[].critical_pose.q`。启动后可以访问 `GET /api/statics/health` 检查是否配置 Skill。

在静力学分析页，先选取 `WORKSHEET-7R` 构型或以 `?model=worksheet` 打开，然后点击「载入选型表」→「运行静力学 Skill」。对其他几何必须上传真正对应其轴线、质量、质心及限位的 Skill 请求。**本 V10 不自动把简化的旧网站配置推导为 Skill 的完整物理模型，避免悄悄生成不准确的质量/质心。**输入模型与网站三维几何是否一致需用户自行核对；若不一致，姿态预览不代表输入模型的真实构型。

## 测试

`npm test` 包含原数学/渲染交互测试及新增 API + CLI 模拟调用验证。模拟 CLI 仅用于隔离验证网站接口，**不证明真实 Skill 数值正确**。真实 Skill 的数值回归、实际浏览器视觉检查、网站云端部署未在此环境完成。

## 部署

该网站 V9 原来是纯静态站点，V10 增加 Node HTTP 服务运行 Skill CLI；纯静态托管不能运行 Node 子进程。需要将 `server/` 部署至支持 Node 的服务端或容器，静态文件可由同一 Node 服务提供。勿将 `STATICS_SKILL_DIR` 指向互联网上未知或未经审计的代码。生产环境需增加鉴权、请求速率限制和执行并发限制。
