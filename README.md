# RobotArm DesignTool — 机械臂设计、静力学校核与工作空间分析工作台

本工程将 **SRS Lab 网页工作台** 与两个独立计算 Skill 放在同一仓库中：

- **`arm-statics-parametric`**：保持载荷、关节范围内大载荷姿态搜索及静态驱动需求，支持任意基座安装朝向和重力分量。
- **`arm-workspace-parametric`**：关节采样点云、目标位置/姿态 IK、空间与采样灵巧空间分类、几何不可达与待定点分析。

两个 Skill 均可通过 CLI 独立运行；网页后端启动真实 CLI，返回计算结果、输入哈希和执行来源。计算不依赖大模型在线推理，也不需要 CUDA。

## 1. 功能与计算来源

| 功能 | 当前实现 | 使用边界 |
| --- | --- | --- |
| 规格与模型 | 七轴模板、逐关节轴向、几何、质量、限位、运动参数及安装姿态编辑 | 网页固定七个旋转关节；独立 Skill 支持更通用的串联模型 |
| 运动学 | 浏览器内 FK、6×7 雅可比、位置 IK、交互三维视图 | 网页运动学页的位置 IK 不约束末端朝向 |
| 静力学分析 | 独立静力学 Skill，经 `POST /api/statics` 调用 | 已检查载荷不等于已证明的全局最坏载荷 |
| 可达空间 | 独立工作空间 Skill，经 `POST /api/workspace` 调用 | 区分位置、离散姿态成功率及未解点，不检查碰撞或带载 |
| 动力学、概念选型 | 保留浏览器内核的示例工况与需求计算 | 使用旧内核的重力估算，尚未自动消费静力学 Skill 结果；未匹配真实产品 |
| 报告与复现 | 输入/结果 JSON、CSV、报告、工作空间 SVG、版本及哈希 | 网页总导出与独立 Skill 导出分别保存 |

**顶部“运行全流程”只运行浏览器概念内核，不自动执行两个独立 Skill。** 静力学页和可达空间页需分别点击运行；重新运行网页概念计算会使旧 Skill 结果失效。

当前版本标识：静力学内核 **1.1.0**、工作空间内核 **1.1.0**、浏览器概念内核 **1.4.0**。网站 `package.json` 仍保留历史名称 `srs-lab-v10` / `10.0.0`，不代表两个 Skill 的版本。

## 2. 快速启动（Windows PowerShell）

### 环境与文件检查

需要 **Node.js ≥20**；已有开发验证环境为 Node.js v24.22.1。网站和两个计算 CLI 无第三方运行依赖，首次运行无需 `npm install`。

Python 3 仅用于可选的 URDF 转换及其测试。PyYAML 仅用于 Skill 作者的元数据校验，不是计算运行依赖。

```powershell
node -v
npm.cmd -v
Test-Path "E:\Robot\armDesignTool\srs-lab-source\package.json"
Test-Path "E:\Robot\armDesignTool\arm-statics-parametric\scripts\run.mjs"
Test-Path "E:\Robot\armDesignTool\arm-workspace-parametric\scripts\run.mjs"
```

三个 `Test-Path` 均应返回 `True`。下文以 `E:\Robot\armDesignTool` 为工程位置；若实际位置不同，请统一替换路径。

### 启动网站与两个 Skill 的调用入口

```powershell
cd "E:\Robot\armDesignTool\srs-lab-source"
$env:STATICS_SKILL_DIR="E:\Robot\armDesignTool\arm-statics-parametric"
npm.cmd start
```

**务必进入 `srs-lab-source` 再运行 `npm.cmd start`。仓库根目录没有 `package.json`。** PowerShell 中使用 `npm.cmd` 可避开部分机器对 `npm.ps1` 的执行策略限制。

工作空间 Skill 默认自动定位同仓库的 `arm-workspace-parametric`，无需额外配置。两个 Skill 不需要另开终端常驻启动，每次网页提交分析时由后端运行 CLI。

正常启动日志类似：

```text
Skill bridge listening on 8787; configured=true
```

这里的 `configured=true` **只表示静力学路径环境变量非空**，不证明路径正确、计算成功或工作空间配置正常。

| 入口 | 地址 |
| --- | --- |
| 网站主页 | [设计总览](http://localhost:8787/) |
| 选型表静力学 | [静力学分析](http://localhost:8787/?model=worksheet#statics) |
| 选型表工作空间 | [可达空间分析](http://localhost:8787/?model=worksheet#workspace) |
| 静力学配置检查 | [statics health](http://localhost:8787/api/statics/health) |
| 工作空间入口检查 | [workspace health](http://localhost:8787/api/workspace/health) |

健康检查成功返回 `{"ok":true,"skillConfigured":true}`。静力学检查只判断环境变量；工作空间检查确认 CLI 入口可读取。两者都不能替代真实计算验证。

保持终端打开。停止服务按 **Ctrl+C**。更新代码后应停止旧服务、重新 `npm.cmd start`，再刷新网页；新的 PowerShell 窗口需重新设置环境变量。

## 3. 整体架构与目录

```text
浏览器 SRS Lab
  ├─ 规格 / 运动学 / 示例动力学 / 旧概念选型 → 浏览器 engine.js
  ├─ 静力学分析 → POST /api/statics   → 独立静力学 CLI → 载荷、临界姿态
  └─ 可达空间   → POST /api/workspace → 独立工作空间 CLI → 点云、分类、成功解
                          │
               Node HTTP 服务：srs-lab-source/server/index.mjs
               每次请求独立临时目录、真实子进程、结果与执行元信息
```

```text
armDesignTool/
├─ README.md
├─ docs/                                 工程设计与专题使用说明
├─ arm-statics-parametric/
│  ├─ SKILL.md                           静力学 Skill 入口说明
│  ├─ scripts/run.mjs                    独立 CLI
│  ├─ scripts/lib/statics.mjs             静力学、安装重力、搜索及判定
│  ├─ assets/examples/                   普通输入、安装姿态、左右臂等样例
│  └─ references/                        坐标契约、Schema、集成与验证说明
├─ arm-workspace-parametric/
│  ├─ SKILL.md                           工作空间 Skill 入口说明
│  ├─ scripts/run.mjs                    独立 CLI
│  ├─ scripts/import_urdf.py              可选 URDF 串联路径转换器
│  ├─ scripts/lib/kinematics.mjs          计算与网页预览共用的几何 FK
│  ├─ scripts/lib/workspace.mjs           采样、IK、分类与统计
│  ├─ scripts/lib/exports.mjs             分类颜色、CSV、报告及 SVG
│  ├─ assets/examples/                   选型表、分类演示、Dexbot 左右臂
│  └─ references/                        输入契约、参考源码分析与来源哈希
└─ srs-lab-source/
   ├─ package.json                       网站启动与测试命令
   ├─ server/index.mjs                   两个 Skill 的 HTTP/CLI 桥接
   ├─ dist/app.js                        页面、状态与结果失效管理
   ├─ dist/engine.js                     浏览器概念计算内核
   ├─ dist/scene3d.js                    网页机械臂三维交互
   ├─ dist/statics-client.mjs             静力学调用与结果适配
   ├─ dist/workspace-client.mjs           工作空间输入生成、调用与姿态重放
   ├─ dist/workspace-scene.js             独立模型点云及杆链预览
   └─ tests/                             数学、页面及真实/模拟 API 测试
```

网站 `dist/` 是当前直接维护、运行的前端代码，仓库没有另一个需要构建的前端源目录。

## 4. 静力学校核：安装、重力与临界姿态

### 网页操作

1. 打开[选型表静力学页面](http://localhost:8787/?model=worksheet#statics)。
2. 在“基座安装与重力”选择正装、倒装、侧装，或编辑自定义 RPY 和世界重力。预设会自动重算；自定义数值编辑后点击顶部“运行全流程”。
3. 点击“载入选型表”，或上传/粘贴符合静力学契约的 JSON/JSONC。
4. 点击“应用网页安装设置到输入”，使上传请求采用当前安装角及世界重力。该操作只更新安装条件，不替你补齐或验证机械臂全部几何、质量和质心；JSONC 会格式化为严格 JSON，注释不保留。
5. 点击“运行静力学 Skill”，检查逐关节载荷、设计输出需求、能力、裕量、输入哈希和运行来源。
6. 在“三维临界姿态预览”选择关节，或点击结果表“显示姿态”，重放该关节最大已检查载荷对应的完整七轴角度。

网页校验结果的基座重力，结果包含安装矩阵时也检查朝向；不一致时拒绝加载。安装一致并不证明几何等价，仍需确认关节名称/顺序、轴向、安装变换、零位、质量归属、质心和限位一致。

### 重力输入二选一

J1 的局部轴向与整臂安装朝向独立，不能通过 J1 轴向推断重力。根请求中的重力部分可以使用旧格式，直接给出**基座系 B**的向量：

```json
"gravity_m_s2": [0, 0, -9.8]
```

也可使用新格式，给出**世界系 W**的重力和 B 相对 W 的安装朝向：

```json
"installation": {
  "gravity_world_m_s2": [0, 0, -9.8],
  "base_orientation_in_world": {
    "rpy_rad": [0.5235987755982988, 0.7853981633974483, 0]
  }
}
```

以上是请求中的字段片段，不是完整请求。两种格式**必须且只能提供一种**。RPY 单位为 rad，示例对应 Roll 30°、Pitch 45°、Yaw 0°。

旋转约定为 `R_WB = Rz(yaw) Ry(pitch) Rx(roll)`；内核转换 `g_B = R_WBᵀ g_W`。该示例的基座重力约为 `[6.929646, -3.464823, -6.001250] m/s²`。

各关节局部轴、几何和质心不因安装角而重复旋转。`simple.base` 或首关节 `origin` 是模型根相对 B 的变换，与 B 相对 W 的 `installation` 分属两层；不要重复填写同一个安装旋转。

### 结果含义

| 字段/状态 | 解释 |
| --- | --- |
| `summary[].max_abs_checked` | 指定姿态及搜索中最大已检查绝对保持载荷；旋转轴 N·m，移动轴 N |
| `summary[].required_output` | 最大已检查载荷 × 设计系数，系数只乘一次 |
| `summary[].critical_pose.q` | 该关节临界姿态的全部主动关节角/位移，不代表所有关节同时最大 |
| `summary[].margin` | 连续能力 / 设计需求 − 1；缺容量或需求为零时为 null |
| `not_sized` | 未提供连续能力，尚不能判定选型 |
| `exceeds_capacity` | 已检查需求超过声明能力 |
| `capacity_conditions_unknown` | 数值能力足够，但连续工作适用条件未声明 |
| `satisfies_checked_cases` | 在声明条件下满足已检查工况 |

完整 `status` 和驱动适用条件可查看 Skill JSON/CLI 报告，不能仅凭网页上的数值裕量判断无条件合格。

## 5. 工作空间：位置、灵巧空间与不可达点

### 两种分析模式

| 模式 | 计算方法 | 能回答的问题 |
| --- | --- | --- |
| `joint_sampling` | 限位内随机或网格关节姿态 → FK → 末端点云 | 哪些位置已由采样关节姿态实际到达 |
| `pose_reachability` | 指定点/分层球体目标 → 独立位置 IK → 可选的多个朝向 IK | 位置是否找到解、哪些采样朝向找到解、哪些点几何不可达或仍待定 |

网页工作空间输入可以由当前模型自动生成，无需质量、质心和重力。独立 Skill 支持旋转、移动及固定关节串联几何；前端预览与计算使用同一份几何 FK，不复制采样或 IK 求解器。

### 网页操作

1. 打开[选型表可达空间页面](http://localhost:8787/?model=worksheet#workspace)。
2. 选择“关节采样 / FK 点云”或“空间 / 灵巧 / 不可达分类（IK）”，点击“从当前网页模型生成输入”。几何、轴向、限位、肩高与安装角一并转换。
3. 或在“参考模型”中选择示例，再点击“载入示例”；也可上传 JSON/JSONC。初次查看分类建议选 **“选型表 / 空间与灵巧分类”**。
4. 点击“运行工作空间 Skill”，查看分类数量、世界点云、基座系投影和剖面。
5. 使用“显示分类”筛选点集；输入点编号、选择成功解，点击“显示所选姿态”。
6. 只有实际关节名称、几何、轴向、限位和安装变换均与当前网页模型一致时，才能“应用到网页七轴模型”。异模型仍可在独立视图中预览，不会强行套用 SRS 外形。
7. 下载完整 Skill JSON、点云 CSV 或分析报告。CLI 另提供六张带分类图例的投影/剖面 SVG。

### 分类定义

| 颜色 | `category` | 含义 |
| --- | --- | --- |
| 蓝 | `spatial_reachable` | 已有位置解，但灵巧性未评估或指定朝向未全通过 |
| 绿 | `sampled_dexterous` | 至少两个不同的采样朝向全部找到解，属于采样灵巧空间近似 |
| 橙 | `partially_dexterous` | 部分采样朝向找到解 |
| 红 | `unreachable` | 超出保守几何外界，已证明位置不可达 |
| 灰 | `unresolved` | 界内且当前预算未找到位置解，保持待定 |

**空间位置可达总数 = 蓝 + 绿 + 橙。** 五类互斥，但“空间可达”这个总集合包含灵巧点，不能把它们再重复相加。

绿色不证明连续任意朝向均可达。单一朝向或等价的重复旋转不能获得灵巧标签；75% 成功率也不能称为全部姿态可达。位置有解而指定朝向全未解时，仍属于蓝色位置可达点。

红色只使用三角不等式保守外界作证明：首关节原点为球心，后续平移长度、末端偏移及移动关节最大行程构成半径。界内未收敛不能直接判为不可达。

**FK 模式只能产生已采到的可达点，不能把点云空洞当作不可达区域，灵巧数量显示“未评估”。** 要查看红色/灰色目标及灵巧分类，请使用 IK 模式。

### 结果与预算

- 每点记录 B/W 位置、分类、位置状态、姿态测试状态、成功姿态解及误差；仅位置有解时，也保存可重放的 `position_solution`。
- `summary.workspace_counts` 为五类互斥计数；`spatial_reachable_point_count` 是位置可达总数；`sampled_dexterous_point_count` 未评估时为 null。
- 采样姿态成功率只针对输入的离散目标集合。`sampled_voxel_volume_m3` 是已占据采样格子的体积，不是真实工作空间体积。
- FK 最多 50,000 点，并限制点数 × 链关节数；IK 最多 16 个主动关节，并限制位置检查、姿态、初值与迭代的组合预算。超过预算会拒绝请求，应先降低分辨率。

## 6. 参考输入与模型一致性

两个 Skill 的请求格式不同；虽然都使用 `schema_version="1.0"`，**静力学请求不能直接提交给工作空间 Skill，反之亦然**。

| 用途 | 示例 |
| --- | --- |
| 静力学：七轴选型表、基座重力输入 | [worksheet-minimal.jsonc](arm-statics-parametric/assets/examples/worksheet-minimal.jsonc) |
| 静力学：世界重力＋安装角，带详细注释 | [worksheet-installation.jsonc](arm-statics-parametric/assets/examples/worksheet-installation.jsonc) |
| 静力学：两杆侧装 | [side-mounted-two-link.json](arm-statics-parametric/assets/examples/side-mounted-two-link.json) |
| 工作空间：选型表 FK 点云 | [worksheet-workspace.jsonc](arm-workspace-parametric/assets/examples/worksheet-workspace.jsonc) |
| 工作空间：球体位置/姿态 IK | [worksheet-reachability.jsonc](arm-workspace-parametric/assets/examples/worksheet-reachability.jsonc) |
| 工作空间：空间/灵巧/不可达分类演示 | [worksheet-classification.jsonc](arm-workspace-parametric/assets/examples/worksheet-classification.jsonc) |
| 工作空间：原 Dexbot URDF 左右臂快照 | [右臂](arm-workspace-parametric/assets/examples/dexbot-right-arm.json)、[左臂](arm-workspace-parametric/assets/examples/dexbot-left-arm.json) |

分类演示有4个目标：目标0为两个采样朝向全通过，目标1为位置可达但指定朝向均未解，目标2超出0.635m几何外界，目标3在界内暂未找到位置解，分别呈绿、蓝、红、灰色。

统一计算单位为 m、kg、rad、N、N·m；网页部分字段显示 mm、度，输入转换由界面负责。JSONC 支持 `//`、`/* */` 注释和 UTF-8 BOM，不支持尾逗号。

静力学以 `installation` 表达世界重力和安装朝向；工作空间以 `base_in_world={xyz_m,rpy_rad}` 表达 B 到 W 的位姿，不需要重力。两者都保留模型的局部坐标定义。

**Dexbot 与 WORKSHEET 不是同一套几何。** 原 URDF 肩部三轴共点，左右肩偏移为 ±0.17m，肩到末端600mm；选型表为七段35/50/250/50/163/47/40mm，J1到末端635mm。

新工作空间 Skill 参考 Dexbot 分析思路，使用独立 CPU 求解，并非 cuRobo 等价移植。随机序列、球体采样、径向参考姿态定义、IK 算法及容差均可能不同；相同参数不保证点云或可达率一致。此前独立 Python FK 对照不等于已完成 cuRobo A/B 验证，详见[参考源码分析](arm-workspace-parametric/references/dexbot-source-review.md)。

## 7. 独立 CLI 与 URDF 转换

CLI 命令不要求网站已启动。以下均使用单行命令，避免 PowerShell 续行符误用。

### 静力学校核

```powershell
cd "E:\Robot\armDesignTool\arm-statics-parametric"
node scripts/run.mjs --input assets/examples/worksheet-installation.jsonc --output outputs/statics_result.json --report-dir outputs/statics
```

生成主 JSON，以及 `report.md`、`joint_summary.csv`、`standard_request.json`、`normalized_model.json`。原始输入或标准请求可复算；`normalized_input` 是内部快照，不能直接当作请求提交。

### 工作空间分类

```powershell
cd "E:\Robot\armDesignTool\arm-workspace-parametric"
node scripts/run.mjs --input assets/examples/worksheet-classification.jsonc --output outputs/workspace_result.json --report-dir outputs/classification
```

生成主 JSON，以及 `points.csv`、`report.md`、`request.json`、`projection-xy/xz/yz.svg` 和 `section-xy/xz/yz.svg`，即三张投影、三张剖面。

两个 CLI 都会创建所需输出目录。主 JSON 在计算和配套导出完成后原子替换，配套文件不是整个目录的事务；**失败时必须检查退出码，不能把仍存在的旧文件当作新结果**。工作空间 `outputs/` 已加入该 Skill 自身的 `.gitignore`。

### 可选 URDF 转换

有实际 URDF 时，可将指定 base→tip 的串联路径转成工作空间请求。将下面的 `robot.urdf` 和 link 名称替换为实际值：

```powershell
cd "E:\Robot\armDesignTool\arm-workspace-parametric"
python scripts/import_urdf.py --input robot.urdf --base-link arm_base_link --tip-link arm_r_link_8 --output outputs/robot-input.json
node scripts/run.mjs --input outputs/robot-input.json --output outputs/robot-result.json --report-dir outputs/robot-report
```

转换器只使用 Python 标准库，不需要 STL 或 cuRobo。保留路径上的固定关节，不导入碰撞或动力学信息；不支持 mimic、无有限分析范围的 continuous 关节及闭链模型。

## 8. 配置、接口与导出区别

| 环境变量 | 默认/作用 |
| --- | --- |
| `STATICS_SKILL_DIR` | 静力学 Skill 目录，必须显式设置 |
| `WORKSPACE_SKILL_DIR` | 可选；默认定位同仓库 `arm-workspace-parametric` |
| `PORT` | 8787 |
| `STATICS_TIMEOUT_MS` | 静力学 CLI 超时，默认120000ms |
| `WORKSPACE_TIMEOUT_MS` | 工作空间 CLI 超时，默认120000ms |
| `WEB_ORIGIN` | 可选跨域响应允许来源，不是鉴权配置 |

例如修改端口：

```powershell
$env:PORT="8888"
npm.cmd start
```

随后将页面地址中的8787改成8888。更换工作空间目录时，在启动前设置：

```powershell
$env:WORKSPACE_SKILL_DIR="E:\Robot\armDesignTool\arm-workspace-parametric"
```

两个 POST 接口均接收完整请求文本，支持 `application/json` / `application/jsonc`，输入上限2MiB。工作空间桥接最多2个并发任务，忙碌返回 `WORKSPACE_BUSY`，超时返回 `WORKSPACE_TIMEOUT`；静力学桥接未实现相同的并发限制。

网站桥接与静力学独立参考服务不同：`arm-statics-parametric/scripts/server.mjs` 的默认8765端口不是主工作台入口，其并发、错误状态和输入大小限制不能套用到8787桥接服务。

| 导出入口 | 内容 |
| --- | --- |
| 网页顶部总导出 | 浏览器概念计算结果、规格CSV、HTML报告或网页配置，不包含独立 Skill 合并结果 |
| 静力学页下载 | Skill原始 `result`，包含输入、安装条件、搜索与判定；不包含外层 `execution` |
| 工作空间页下载 | 包含 `result` 和 `execution` 的完整响应，保留运行ID等信息 |
| 独立 CLI | 各自的主结果JSON与配套分析文件 |

后端每次请求使用独立临时目录，返回结果后清理，不长期保存报告。需要复现时，请自行保留请求、结果、代码版本/提交号及执行信息。

## 9. 测试与验证范围

分别运行：

```powershell
cd "E:\Robot\armDesignTool\srs-lab-source"
npm.cmd test

cd "E:\Robot\armDesignTool\arm-statics-parametric"
node --test scripts/tests/*.test.mjs

cd "E:\Robot\armDesignTool\arm-workspace-parametric"
node --test scripts/tests/*.test.mjs
python -m unittest discover -s scripts/tests -p "test_*.py"
```

已有开发验证记录为：网站 **16项**、静力学 **64项**、工作空间 **17项**、URDF转换 **3项**测试通过。运行器的一项测试可能包含多条断言；这些数字不是每次启动时自动检查的结果。

验证包括真实 HTTP→CLI 与直接内核结果一致性、静力学安装旋转等价性、临界姿态复算、工作空间模型转换/成功解重放、分类判定、重复朝向处理、预算拒绝和失败无回退。Dexbot左右臂20组FK与独立Python齐次变换对照通过。

**尚未完成真实浏览器视觉与完整端到端交互验收，也未完成原 cuRobo 的同目标集对照运行。** Node中的页面/Canvas模拟测试不能代替这两项验证。

## 10. 常见问题

| 现象 | 检查与处理 |
| --- | --- |
| `ENOENT ... armDesignTool\package.json` | 当前在仓库根目录；先 `cd E:\Robot\armDesignTool\srs-lab-source` 再启动 |
| `npm.ps1` 执行受限 | 改用 `npm.cmd start` / `npm.cmd test` |
| `EADDRINUSE` | 旧服务未停止或端口被占用；停止自己的旧服务，或设置 `PORT` |
| `configured=false` / `SKILL_NOT_CONFIGURED` | 在启动服务的同一终端设置正确的 `STATICS_SKILL_DIR`；不要沿用旧的 `E:\Robot\designTool` 路径 |
| 静力学 `configured=true` 仍调用失败 | 该标志不检查实际目录；确认 `scripts/run.mjs` 存在，再独立运行 CLI |
| `SKILL_EXECUTION_FAILED` | 查服务日志和CLI错误；静力学超时在日志中可见 `SKILL_TIMEOUT`，对外可能仍返回通用执行失败 |
| `WORKSPACE_ASSET_UNAVAILABLE` / `WORKSPACE_EXECUTION_FAILED` | 检查工作空间目录、CLI和共享模块；必要时显式设置 `WORKSPACE_SKILL_DIR` |
| `WORKSPACE_TIMEOUT` / `WORKSPACE_BUSY` | 降低采样分辨率、姿态数量或IK预算；等待任务完成，必要时调整超时 |
| 安装/重力不一致，静力结果被拒绝 | 先重算网页，再“应用网页安装设置到输入”；仍需自行核对物理模型 |
| 参数修改后无法查看新结果 | 先运行顶部全流程更新网页参数，再分别重新运行两个Skill；旧结果已失效 |
| FK图里没有不可达点，灵巧性未评估 | 正常；FK只能展示已采到的可达点。切换IK模式或载入分类示例 |
| 灰色点很多 | 表示预算内未找到位置解，不证明不可达；检查模型和目标，再合理增加预算 |
| 绿色点不能称作严格灵巧工作空间 | 绿色只验证输入的离散朝向；需要更充分的姿态覆盖分析或其他证明 |
| 工作空间“应用到网页七轴模型”不可用 | Skill模型与当前网页模型不一致，或所选点没有解；可继续在独立视图观察 |
| 更新代码后仍是旧界面/接口 | Ctrl+C停止旧服务，重新启动并刷新页面 |

## 11. 边界、维护与详细文档

本工程面向固定基座串联机械臂的概念设计和已声明工况分析。静力学不替代动态、热保持、寿命、抱闸或结构验收；工作空间不替代碰撞、带载可达性和完整边界证明。目前没有完整的产品目录匹配、参数优化回写、任务持久化队列或多用户鉴权，不作为直接公网部署的成品服务。

维护时保持独立 Skill 为对应分析页的结果来源。网页旧内核仍为概念动力学/选型计算重力和采样估计，不能混同于新Skill校核；改动契约时同步检查前端适配、示例及测试。不要提交大量输出、临时目录或配置密钥。

- [工程设计说明](docs/工程设计说明.md)：首次整体梳理及实现边界；后续升级以专题文档为准。
- [安装姿态与重力配置](docs/安装姿态与重力配置.md)：静力学坐标转换、网页操作及追溯。
- [可达工作空间Skill设计与使用](docs/可达工作空间Skill设计与使用.md)：独立工作空间实现与分类演示。
- [静力学Skill说明](arm-statics-parametric/SKILL.md)与[输入契约](arm-statics-parametric/references/input-contract.md)。
- [工作空间Skill说明](arm-workspace-parametric/SKILL.md)与[输入契约](arm-workspace-parametric/references/input-contract.md)。
- [Dexbot参考源码分析](arm-workspace-parametric/references/dexbot-source-review.md)：原工程与当前CPU实现的差异、模型来源及指纹。
