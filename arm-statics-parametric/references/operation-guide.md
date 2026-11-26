# 操作细则

命令和反引号中的相对路径均以本技能根目录为基准；本页的 Markdown 链接相对于本页。仅阅读入口指定的相关章节。示例参数只用于演示，不是用户设计或厂家能力。版本清单和历史验收不代表当前环境已验证。

先阅读 [输入与坐标约定](../references/input-contract.md)。网页接入时阅读 [集成接口](../references/web-integration.md)；解释精度时阅读 [验证与边界](../references/validation.md)。

## 输入选择

- 使用 standard 接收逐关节原点、安装旋转、局部轴、限位、质量部件和局部质心。支持1～64个串联 revolute/prismatic/fixed 关节；搜索至多16主动关节。
- 使用 simple 接收每段轴向、长度、方向、质量和限位。支持1～32个旋转关节，展开后使用同一标准内核。
- 重力输入二选一：`gravity_m_s2` 为模型基座坐标系中的重力；或用 `installation` 给出世界重力与基座相对世界的 RPY（rad），内核计算 `g_B=R_WBᵀg_W`。两者不能同时给出；不能由J1轴向推断安装朝向。
- 显式给出重力向量、负载质量及负载质心。无负载时填0。不要将缺失质量或COM默认为0。
- 接收由构型选择记录显式映射并补齐后的参数请求，不要求URDF。缺真实COM时可以采用简易输入的中点估算，但必须保留warnings，不能称为CAD精度。
- 将模组固定端、运动端质量分配到正确刚体。简易输入整模组放入当前输出刚体只是概念近似。
- 使用关节名称映射角度，禁止依赖隐含的左右臂数组顺序。
- 保留旧版arm-statics；本版本契约不同，不把新JSON直接传给旧脚本。

注释输入可用 `assets/examples/worksheet-minimal.jsonc`；严格 JSON 对照为同名 `.json`。CLI、参考网页和 HTTP 统一通过 `scripts/lib/input-text.mjs` 解析，支持 `//` 与 `/* */` 注释及 UTF-8 BOM，不支持尾逗号。`analyze(input)` 仍接收普通对象，输出始终为严格 JSON。详细示例见 [精简输入与输出](../references/minimal-input-output.md)。

## 双默认示例

在本工具目录使用 Node.js 20 或更高版本运行。选型表采用同一套七轴单臂的侧装模型，FR3 采用 Franka ROS 标称模型。两份请求均可单独运行。[选型表七轴单臂](../assets/examples/defaults/worksheet.json) · [Franka FR3](../assets/examples/defaults/fr3.json)

```bash
node scripts/run.mjs --input assets/examples/defaults/worksheet.json --output outputs/worksheet/result.json --report-dir outputs/worksheet
node scripts/run.mjs --input assets/examples/defaults/fr3.json --output outputs/fr3/result.json --report-dir outputs/fr3
```

两例以种子 42 搜索 20,000 个姿态并计算指定姿态的七轴保持力矩。选型表含 3 kg 负载和前、侧平举工况。FR3 采用重力 9.81 m/s²，在初始和偏置姿态检查空载法兰。

FR3 保留七个活动轴的标称质量、局部质心和惯性张量、基座安装及 0.107 m 法兰偏移，使用无手爪、空载配置。上游公用惯性表采用 Gaz 等人针对 Panda 的辨识结果。[Franka 固定版本惯性来源](https://github.com/frankarobotics/franka_ros/blob/ddd2fffd9de44b02ad15b4bbb2bfa2cec4d60d98/franka_description/robots/common/inertial.yaml) · [上游许可证](../references/franka-fr3-LICENSE.txt)

将示例替换成实际请求。失败返回非零退出码与结构化错误；不得把仍存在的旧结果解释为新请求成功。主JSON在计算及配套导出完成后原子替换，配套文件不是整目录事务。

## 工作步骤

1. 核对几何、质量、COM坐标系、安装重力及负载偏置。区分表格和CAD模型版本，不混用质量分布。
2. 选择standard或simple。把“平举”等工况转成明确关节角，不假设所有构型零位都是前平举。
3. 调用确定性内核，禁止由语言模型手填力矩结果。
4. 计算指定姿态及可选固定种子搜索。每关节保存临界姿态并单点复核；不要声称已证明全局最坏值。
5. 展示带符号保持载荷、最大绝对值、分项贡献和设计系数。旋转单位Nm，移动单位N。
6. 缺驱动参数时解释为not_sized；连续能力条件不明时为capacity_conditions_unknown。只有satisfies_checked_cases可解释为声明条件下已检查范围满足。
7. 明确热、碰撞、动态和结构未检查；静态需求不证明无抱闸持续平举30分钟已达标。
8. 按用户要求保存分析交付物。保留技能源码与本次分析交付物的各自位置，不依赖特定平台工作区或Library目录。

## 网页接口

- Node与浏览器均导入 `scripts/lib/statics.mjs` 的 `analyze(input)`，禁止复制第二套公式。
- 使用 `assets/web/adapter.mjs` 的 `analyzeInWorker` 隔离搜索，支持取消与超时。
- 使用 `toStandardInput(input)` 生成可重新计算的标准请求并继承估算假设。结果中的normalized_input是内部快照，不是请求格式。
- 执行 `node scripts/server.mjs --port 8765` 打开参考页面及POST /api/statics。现有网页映射表单与结果字段即可，无需重建网站。
- 用references/schemas/request.schema.json做预校验；内核再次检查名称引用、限位、非法数值等。

## 示例与测试

- 旧版简易格式、安装参数和轴向约定可通过 worksheet-minimal.json/jsonc 与 worksheet-installation.jsonc 重放。单杆、两杆和移动关节等回归数据见 scripts/tests/fixtures。
- 执行 `node --test scripts/tests/*.test.mjs`。不要将未运行的浏览器或服务检查写成已通过。

只支持固定基座串联刚体链。闭链、柔性、浮动基座、未知接触反力与被动关节平衡不求解。fixed代表几何锁定，不模拟有限容量抱闸。重力静力学不要求惯性张量。


## 公共参数模型与独立分发（P0/P1）

CLI 与 `analyze(input)` 另外接受 `schema_name: "arm-analysis-request"`、`schema_version: "1.0"` 的公共请求，design_model 内嵌参数模型，options 保留本 Skill 的分析设置。无需网页、CAD 或 URDF。完整示例为 `assets/examples/design-seven-axis.json`；读取 [公共输入说明](../references/design-model.md) 后再构造这种请求。旧格式继续支持。

独立执行仍使用本 Skill 的 `scripts/run.mjs --input ... --output ... --report-dir ...`。共享代码位于同包根的 `shared/`，单独分发时使用仓库 `shared/scripts/package-skill.mjs` 生成 bundle；保留 skill 与 shared 同级结构，不依赖网页源码或原始仓库路径。
