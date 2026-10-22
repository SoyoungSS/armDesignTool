# RobotArm DesignTool — 机械臂设计与静力学校核工作台

本工程将 **SRS Lab 机械臂设计网站** 和 **`arm-statics-parametric` 静力学校核 Skill** 放在同一个代码仓中管理，使网页成为静力学 Skill 的**参数输入、运行入口、结果展示和三维临界姿态预览界面**。

> **设计原则：** 静力学扭矩、临界姿态搜索及驱动静态选型需求，由独立 Skill 负责计算；网站通过 API 获取结果，不再实现第二套用于“静力学校核”的计算逻辑。网页原有运动学、可达空间、动力学等概念设计功能仍保留。

## 1. 项目作用

- **配置机械臂模型：** 在 SRS Lab 中查看/编辑七自由度机械臂的构型、关节轴、杆长和其他规格。
- **观察与分析：** 使用原有三维机械臂模型、运动学计算、工作空间点云和动力学分析功能。
- **静力学校核：** 网页将 JSON/JSONC 输入发送给后端，由 `arm-statics-parametric` 实际执行重力矩校核和姿态搜索。
- **关节选型参考：** 展示 J1–J7 已检查姿态的最大绝对扭矩、设计系数后的输出侧需求、连续能力、裕量等。
- **三维临界姿态预览（V11）：** 选择某关节最大扭矩对应的姿态，使用 Skill 返回的七个关节角度驱动网页原有三维机械臂。
- **可追溯：** 查看 Skill 运行 ID、计算内核版本、输入哈希，导出结构化结果。

### 整体架构

```text
浏览器：SRS Lab（http://localhost:8787）
   ├─ 设计总览 / 规格与模型 / 运动学 / 可达空间 / 动力学 / 原概念选型
   └─ 静力学分析 → 载入 JSON(C) → 运行 Skill → 扭矩表 / 三维临界姿态预览
                                │ HTTP POST /api/statics
                                ▼
Node.js 网站服务（srs-lab-source/server/index.mjs）
                                │ 启动独立 CLI 进程
                                ▼
arm-statics-parametric/scripts/run.mjs
                                │ 生成 statics_result.json
                                ▼
Node.js 将真实 Skill 结果返回给网页 → 更新扭矩、需求、临界姿态
```

## 2. 工程目录

建议本机目录结构如下（下文所有操作均以此为例）：

```text
E:\Robot\designTool\
├── README.md                         # 本文件：整个工程的使用指南
├── arm-statics-parametric\          # 独立静力学校核 Skill
│   ├── scripts\
│   │   └── run.mjs                   # Skill CLI 入口（必需）
│   └── assets\examples\
│       └── worksheet-minimal.jsonc   # 静力学校核示例输入（以实际文件为准）
└── srs-lab-source\                 # SRS Lab 网站 V11 工程
    ├── package.json                 # npm 启动、测试命令
    ├── server\index.mjs            # HTTP 服务 + Skill 调用桥接
    ├── dist\index.html             # 网页入口
    ├── dist\app.js                 # 主界面、静力学数据显示与交互
    ├── dist\scene3d.js             # 机械臂三维模型
    ├── dist\engine.js              # 原有运动学等概念计算
    ├── dist\statics-client.mjs     # Skill HTTP 客户端
    ├── dist\worksheet-minimal.jsonc# 网页可加载的示例
    └── tests\                      # 网站与 Skill API 自动化测试
```

> 目录是**推荐组织方式**，实际以本地文件为准。两个子工程可以分别版本管理，但本工程建议在根目录统一记录变更及交付版本。

## 3. 运行环境（Windows 10 / 11）

### 3.1 安装 Node.js

从 [Node.js 官网](https://nodejs.org/) 安装当前 LTS 版本（建议 Node.js 20+；项目所需最低版本以运行测试结果为准）。安装时使用默认配置并确保可在终端中运行 `node`。

按 `Win + R`，输入 `powershell` 回车，分别运行：

```powershell
node -v
npm.cmd -v
```

只要两条命令均显示版本号，就说明已安装成功。PowerShell 中优先使用 `npm.cmd`，避免部分电脑出现 `npm.ps1` 执行策略限制。

### 3.2 检查两个工程文件

```powershell
Test-Path "E:\Robot\designTool\srs-lab-source\package.json"
Test-Path "E:\Robot\designTool\arm-statics-parametric\scripts\run.mjs"
```

**两条都应显示 `True`。** 第一条说明找到了网站工程根目录；第二条只说明 Skill 入口存在，是否能正确计算仍需实际运行验证。

## 4. 一键按顺序启动网站和 Skill

> Skill 不是一个需要单独常驻启动的网页服务。**启动网站后端时配置 Skill 路径，网页每次发起静力学分析，后端才运行 Skill CLI。**

在一个新的 PowerShell 窗口中执行：

```powershell
# 步骤 1：进入网站工程根目录
cd "E:\Robot\designTool\srs-lab-source"

# 步骤 2：告诉网站静力学 Skill 在哪里
$env:STATICS_SKILL_DIR="E:\Robot\designTool\arm-statics-parametric"

# 步骤 3：启动本地网站服务
npm.cmd start
```

正常情况下会看到类似输出：

```text
Skill bridge listening on 8787; configured=true
```

然后打开 Edge 或 Chrome：

- **网站主页：** http://localhost:8787/
- **选型表模型的静力学页面：** http://localhost:8787/?model=worksheet#statics
- **后端 Skill 配置检查：** http://localhost:8787/api/statics/health

健康检查正常时通常返回：

```json
{"ok":true,"skillConfigured":true}
```

**注意：** `configured=true` 只代表设置了 Skill 环境变量，**不等于**已经完成真实计算或数值校核。

**保持 PowerShell 窗口打开**，网站才能继续访问。要停止网站，回到该窗口按 `Ctrl + C`。

### 4.1 以后重新打开网站

每次重新打开 PowerShell，都重新执行上面的三条命令。`$env:STATICS_SKILL_DIR` 是当前终端会话内的临时配置，关闭窗口后不会永久保存。

### 4.2 修改端口（可选）

如果 8787 被占用，可在 `npm.cmd start` 前设置：

```powershell
$env:PORT="8888"
npm.cmd start
```

随后访问 http://localhost:8888/ 。未设置时默认端口为 `8787`。

## 5. 如何在网站中执行静力学校核

1. 打开 http://localhost:8787/?model=worksheet#statics 。
2. 进入左侧 **“静力学分析”** 页面。
3. 点击 **“载入选型表”**，或上传/粘贴符合 Skill 契约的 `.json` / `.jsonc` 输入；示例位于 `srs-lab-source/dist/worksheet-minimal.jsonc`。
4. 点击 **“运行静力学 Skill”**。浏览器向本地 `POST /api/statics` 提交输入。
5. 计算成功后，检查 **J1–J7 最大绝对载荷、输出侧需求、连续能力、裕量** 以及 Skill 版本/输入哈希等信息。
6. 在 **“三维临界姿态预览”** 下拉框中选择 **J1～J7 最大扭矩临界姿态**，原有三维模型将应用 Skill 返回的完整七轴姿态角 `summary[].critical_pose.q`。
7. 也可以在关节结果表点击 **“显示姿态”**。切换姿态时不重新计算扭矩，只是切换结果对应的预览角度。
8. 修改模型或输入后应**重新运行** Skill，不能继续把旧结果当作当前模型的校核结论。

### 输入、输出与单位

| 数据 | 示例字段 / 说明 |
| --- | --- |
| 输入模型 | 关节名称、轴方向、杆长、质量、质心、限位、负载、重力等 |
| 指定姿态 | `poses`；角度采用 **rad**（弧度） |
| 搜索配置 | `search`；在关节限位中查找高负载姿态 |
| Skill 返回 | `result.joint_names`、`result.summary[]`、`result.poses[]` 等 |
| 最大扭矩 | `summary[].max_abs_checked`，已检查范围内最大绝对值，单位 N·m |
| 设计需求 | `summary[].required_output`，通常为静态载荷乘设计系数 |
| 临界姿态 | `summary[].critical_pose.q`，完整 J1–J7 角度（rad） |
| 裕量 | `summary[].margin`；没有驱动能力数据时可能是 `null` |

**工程注意：** 最大已检查扭矩不等于数学意义上的全局最大值；静力学校核不能取代动态载荷、碰撞、温升保持、寿命或结构强度验证。

### 三维姿态一致性特别重要

Skill 返回的是 **关节角度**，网页负责根据自己已有的模型显示机械臂。必须保证 Skill 与网页使用的是**同一个机械臂物理模型**（关节顺序、轴方向、安装姿态、零位、几何和限位）。否则即使角度有效，三维图也可能不对应实际被计算的姿态。

## 6. 单独运行和验证 Skill（可选，推荐初次集成时做一次）

若希望绕开网页、直接验证计算内核，可执行：

```powershell
cd "E:\Robot\designTool\arm-statics-parametric"

# 将结果放进 Skill 工程自己的 outputs 文件夹
New-Item -ItemType Directory -Force ".\outputs" | Out-Null

node scripts/run.mjs `
  --input "assets/examples/worksheet-minimal.jsonc" `
  --output "outputs/statics_result.json" `
  --report-dir "outputs"
```

注意：上述命令使用 PowerShell 的反引号 `` ` `` 作为续行符，**反引号必须位于每个续行的最后**，不能在后面加空格。也可以直接使用单行形式：

```powershell
node scripts/run.mjs --input "assets/examples/worksheet-minimal.jsonc" --output "outputs/statics_result.json" --report-dir "outputs"
```

如果你的 Skill 版本中示例文件不在这个位置，改成真实存在的输入文件路径。

正常情况下，Skill 应生成：

```text
outputs/
├── statics_result.json   # 结构化计算结果
├── report.md             # 面向人的报告
├── joint_summary.csv     # 逐关节汇总
├── standard_request.json # 标准化请求
└── normalized_model.json # 展开的内部模型
```

是否生成全部文件以实际 Skill 版本和运行参数为准。可以优先查看 `statics_result.json` 和 `report.md`。

## 7. 运行网站自动化测试

另开一个 PowerShell 窗口：

```powershell
cd "E:\Robot\designTool\srs-lab-source"
npm.cmd test
```

测试应显示各项 `pass`，`fail` 为 0。此命令用于检查网站的原有功能和 Skill API 接口，不等同于真实 Skill 数值回归。

首次真正接通 Skill 后，应使用同一份输入，把**网页结果**与**第 6 节 CLI 结果**逐项对比，尤其对比 `summary[]` 中的最大扭矩、设计需求和临界姿态角。

## 8. 常见问题排查

| 现象 | 原因或处理办法 |
| --- | --- |
| `node` 无法识别 | 未安装 Node.js，或安装后未重新打开 PowerShell |
| `npm.ps1` 执行受限 | 使用 `npm.cmd start` 和 `npm.cmd test` |
| 找不到 `package.json` | 未进入 `E:\Robot\designTool\srs-lab-source` 根目录 |
| 浏览器无法打开 localhost | 检查 PowerShell 服务是否运行；确认访问端口是否正确 |
| `EADDRINUSE` | 端口被占用；关闭旧网站进程或使用 `PORT=8888`（PowerShell 中为 `$env:PORT="8888"`） |
| `configured=false` | 启动前未设置 `STATICS_SKILL_DIR`；停止服务后重新设置并启动 |
| `SKILL_NOT_CONFIGURED` | 服务端没有 Skill 路径配置 |
| `SKILL_EXECUTION_FAILED` | Skill 命令失败或输出格式异常；查看网站 PowerShell 窗口的错误日志，再独立运行第 6 节 CLI 排查 |
| `SKILL_TIMEOUT` | Skill 搜索耗时超过服务超时配置；先减少搜索样本或调整 `STATICS_TIMEOUT_MS`（毫秒） |
| 扭矩结果与历史值不一致 | 核对模型、质量归属、质心、关节轴、零位、重力和搜索配置是否一致 |
| 三维临界姿态不合理 | 核对 Skill 与网页是否采用同一几何模型、关节轴、关节顺序和零位 |
| 修改 Skill 后网页未变 | 确认改动的是真正配置的 Skill 目录，并重新运行分析；必要时重启服务 |

**快速检查当前环境：**

```powershell
Get-Location
node -v
$env:STATICS_SKILL_DIR
Test-Path "E:\Robot\designTool\arm-statics-parametric\scripts\run.mjs"
```

## 9. 工程维护约定

1. **静力学算法唯一入口：** 只修改 `arm-statics-parametric` 中的算法；网站不能再复制一份扭矩计算内核。
2. **保持数据接口兼容：** Skill 更新时优先保留 `result.joint_names`、`summary[]`、`summary[].critical_pose.q` 等字段；必要时同步更新网页适配层和测试。
3. **不要将概念估算混为校核结果：** 原网页运动学、动力学和部分概念选型仍是浏览器侧计算；正式静力学输出必须标识 Skill 来源。
4. **保存可复现输入：** 记录输入 JSON/JSONC、Skill 版本、搜索种子/搜索规模、运行结果与校核报告。
5. **变更后测试：** 改网站运行 `npm.cmd test`；改 Skill 先做 CLI 数值回归，再做网页端到端验证。
6. **避免误提交大文件与临时输出：** 推荐忽略 `node_modules/`、`outputs/`、`tmp/`、`.env` 等本地运行目录/文件（如果使用 Git，可通过 `.gitignore` 管理）。

## 10. 当前集成状态与边界

- 网站：**SRS Lab V11**（在 V9 原站基础上迭代 V10/V11）。
- 静力学：**独立 Skill CLI**，通过 Node.js 服务的 `POST /api/statics` 调用。
- 三维临界姿态：通过 Skill 搜索结果 `summary[].critical_pose.q` 驱动原三维场景。
- 验证：此前网站的接口与模拟 Skill 自动化测试已通过；**实际本机是否完整通过**，仍需按第 5～7 节运行真实 Skill 并比对结果。
- 使用范围：当前为本地学习和工程开发工具。默认没有完整的公网安全防护，**不要直接把本地服务暴露到公网**。

---

**首次使用推荐顺序：** 安装 Node.js → 检查目录 → 按第 4 节启动网站 → 按第 6 节单独运行 Skill → 按第 5 节网页分析 → 按第 7 节测试与对比。
