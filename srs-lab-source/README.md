# SRS Lab V10 — 机械臂正向设计与静力学 Skill 集成

基于原 SRS Lab 在线网站 V9（用户提供的 `srs-lab-online-source-v9(1).zip`）进行原位改造。保留原有九个导航页与三维模型操控，静力学分析页面改为独立 `arm-statics-parametric` CLI 的结果展示。

启动命令：`STATICS_SKILL_DIR=/path/to/skill npm start`；访问 `http://localhost:8787/`。运行测试：`npm test`。

完整接入细节与边界见 [V10_INTEGRATION.md](V10_INTEGRATION.md)。

## V11 新增：静力学分析 → 三维临界姿态预览

在静力学 Skill 成功返回 `summary[].critical_pose.q` 后，进入“静力学分析”页面下方的“三维临界姿态预览”。在“最大扭矩姿态”下拉框中选择 J1 至 J7 任一关节，页面显示该姿态的计算来源、最大已检查扭矩、完整七轴角度（度），并直接把返回的弧度角传给 V9/V10 原有 `Arm3D` 模型，无第二套扭矩求解器。上方的“关节静力学结果”表格也可点击“显示姿态”。

**使用注意**：该功能要求 Skill 输入与网页当前三维构型（关节名称、轴、零位、安装变换、杆长、限位）一致；只校验七关节角度及限位，无法自动证明不同模型几何等价。修改规格后旧结果失效，需重新运行 Skill。未提供真实 Skill 时，只有模拟接口测试，不能声称已验证真实搜索数值。

Windows PowerShell（按实际目录调整）：

```powershell
cd "E:\Robot\designTool\srs-lab-source"
$env:STATICS_SKILL_DIR="E:\Robot\designTool\arm-statics-parametric"
npm.cmd start
```

浏览器访问 `http://localhost:8787/?model=worksheet#statics`。
