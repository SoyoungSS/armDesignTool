---
name: arm-statics-parametric
description: 不依赖URDF的参数化机械臂静力学校核。支持标准输入和简易连杆输入，计算串联旋转、移动及固定关节链的自重、附件、偏心负载与已知外力产生的保持载荷，搜索关节范围内大载荷姿态，输出逐轴结果与驱动静态需求。用于机械臂概念设计、典型姿态校核、静态驱动初选和网页接入；不替代动力学、热保持、碰撞或结构强度验收。
---

# 参数化机械臂静力学校核

先阅读 [输入与坐标约定](references/input-contract.md)。网页接入时阅读 [集成接口](references/web-integration.md)；解释精度时阅读 [验证与边界](references/validation.md)。

## 输入选择

- 使用 standard 接收逐关节原点、安装旋转、局部轴、限位、质量部件和局部质心。支持1～64个串联 revolute/prismatic/fixed 关节；搜索至多16主动关节。
- 使用 simple 接收每段轴向、长度、方向、质量和限位。支持1～32个旋转关节，展开后使用同一标准内核。
- 重力输入二选一：`gravity_m_s2` 为模型基座坐标系中的重力；或用 `installation` 给出世界重力与基座相对世界的 RPY（rad），内核计算 `g_B=R_WBᵀg_W`。两者不能同时给出；不能由J1轴向推断安装朝向。
- 显式给出重力向量、负载质量及负载质心。无负载时填0。不要将缺失质量或COM默认为0。
- 接收构型选择Skill的参数，不要求URDF。缺真实COM时可以采用简易输入的中点估算，但必须保留warnings，不能称为CAD精度。
- 将模组固定端、运动端质量分配到正确刚体。简易输入整模组放入当前输出刚体只是概念近似。
- 使用关节名称映射角度，禁止依赖隐含的左右臂数组顺序。
- 保留旧版arm-statics；本版本契约不同，不把新JSON直接传给旧脚本。

注释输入可用 `assets/examples/worksheet-minimal.jsonc`；严格 JSON 对照为同名 `.json`。CLI、参考网页和 HTTP 统一通过 `scripts/lib/input-text.mjs` 解析，支持 `//` 与 `/* */` 注释及 UTF-8 BOM，不支持尾逗号。`analyze(input)` 仍接收普通对象，输出始终为严格 JSON。详细示例见 [精简输入与输出](references/minimal-input-output.md)。

## 执行

在技能目录使用Node.js≥20，推荐22/24，无需安装npm依赖：

```bash
node scripts/run.mjs --input assets/examples/simple-two-link.json --output /absolute/output/statics_result.json --report-dir /absolute/output/report
```

将示例替换成实际请求。失败返回非零退出码与结构化错误；不得把仍存在的旧结果解释为新请求成功。主JSON在计算及配套导出完成后原子替换，配套文件不是整目录事务。

## 工作步骤

1. 核对几何、质量、COM坐标系、安装重力及负载偏置。区分表格和CAD模型版本，不混用质量分布。
2. 选择standard或simple。把“平举”等工况转成明确关节角，不假设所有构型零位都是前平举。
3. 调用确定性内核，禁止由语言模型手填力矩结果。
4. 计算指定姿态及可选固定种子搜索。每关节保存临界姿态并单点复核；不要声称已证明全局最坏值。
5. 展示带符号保持载荷、最大绝对值、分项贡献和设计系数。旋转单位Nm，移动单位N。
6. 缺驱动参数时解释为not_sized；连续能力条件不明时为capacity_conditions_unknown。只有satisfies_checked_cases可解释为声明条件下已检查范围满足。
7. 明确热、碰撞、动态和结构未检查；静态需求不证明无抱闸持续平举30分钟已达标。
8. 按用户要求保存分析交付物。技能代码通过Skills工作区Git保存，不另复制到Library。

## 网页接口

- Node与浏览器均导入 `scripts/lib/statics.mjs` 的 `analyze(input)`，禁止复制第二套公式。
- 使用 `assets/web/adapter.mjs` 的 `analyzeInWorker` 隔离搜索，支持取消与超时。
- 使用 `toStandardInput(input)` 生成可重新计算的标准请求并继承估算假设。结果中的normalized_input是内部快照，不是请求格式。
- 执行 `node scripts/server.mjs --port 8765` 打开参考页面及POST /api/statics。现有网页映射表单与结果字段即可，无需重建网站。
- 用references/schemas/request.schema.json做预校验；内核再次检查名称引用、限位、非法数值等。

## 示例与测试

- assets/examples/simple-two-link.json 和standard-two-link.json：等价物理模型。
- assets/examples/seven-axis-concept.json：3.612kg概念集中质量，不等同CAD。
- assets/examples/project-right-arm.json 和project-left-arm.json：项目参数快照，无运行时URDF依赖。
- assets/examples/prismatic.json：移动关节输出力。
- assets/examples/side-mounted-two-link.json：世界重力与侧装姿态输入；报告保留世界/基座重力、安装RPY及旋转矩阵。
- 执行 `node --test scripts/tests/*.test.mjs`。不要将未运行的浏览器或服务检查写成已通过。

只支持固定基座串联刚体链。闭链、柔性、浮动基座、未知接触反力与被动关节平衡不求解。fixed代表几何锁定，不模拟有限容量抱闸。重力静力学不要求惯性张量。
