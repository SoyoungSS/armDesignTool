# 可达工作空间 Skill：设计与使用

本次在工程新增 `arm-workspace-parametric/`，与静力学Skill并列。网页“可达空间”页通过 `POST /api/workspace` 启动独立CLI，计算结果、点云、投影、剖面及关节解均来自该Skill。

内核1.1.0增加空间分类：蓝色为空间位置可达，绿色为已采样姿态全通过的灵巧空间近似，橙色为部分采样姿态可达，红色为超出保守几何外界的已证明不可达点，灰色为界内暂未找到位置解的待定点。绿色不等于连续所有朝向都已证明可达。

## 参考来源与能力

参考用户提供的Dexbot_ana.zip：采用其“关节采样FK点云”和“球内位置×姿态IK成功率”的分析思路。新Skill不安装cuRobo/CUDA，用原生Node实现确定性CPU计算。左右臂参数直接读取压缩包内URDF形成单独示例，没有执行压缩包代码。

支持：

- 任意局部轴、安装偏置、旋转/移动/固定关节串联模型。
- 关节随机采样、全网格、按关节名称固定部分姿态。
- 指定目标点、分层球体或外球面采样。
- 位置IK、离散末端姿态IK、朝向球心的参考姿态扰动。
- 基座/世界坐标点云、三投影与三剖面、采样体素统计。
- 逐点关节解和误差保存、模型匹配后的网页七轴姿态应用。
- JSON/CSV/Markdown/SVG导出、输入/模型哈希与运行来源。

不检查碰撞、自碰撞、驱动载荷、热、完整边界或双臂协同。IK失败表示预算内未找到解；采样姿态成功率不等于连续姿态覆盖率，采样体素体积不等于真实工作空间体积。

## 启动与网页操作

已有服务需要先按Ctrl+C停止，再重新启动，才能加载新接口：

```powershell
cd E:\Robot\armDesignTool\srs-lab-source
$env:STATICS_SKILL_DIR = 'E:\Robot\armDesignTool\arm-statics-parametric'
npm.cmd start
```

工作空间Skill默认自动定位同仓库 `arm-workspace-parametric`，无需额外设置环境变量。若放在其他目录，可设置 `WORKSPACE_SKILL_DIR`；`WORKSPACE_TIMEOUT_MS`默认120000ms。

访问 `http://localhost:8787/?model=worksheet#workspace`。服务检查为 `GET /api/workspace/health`，与静力学的配置独立。

1. 选择“关节采样 / FK点云”或“球体位置与姿态 / IK可达率”。
2. 点击“从当前网页模型生成输入”，几何、轴向、限位、肩高及安装角将一起转换；无需填写质量或重力。
3. 如需更换采样预算、目标或姿态，在JSON文本中修改后运行Skill。
4. 也可以载入选型表或Dexbot左右臂示例，或上传JSON/JSONC。
5. 查看点云、三投影及三剖面；输入点编号，选择已找到的解并显示姿态。
6. 仅当模型几何、轴、限位、安装角和关节名称均匹配时，“应用到网页七轴模型”可用。
7. 从工作空间页导出完整Skill结果、点云CSV或报告。顶部旧全流程导出仍是网页概念结果，未合并独立Skill结果。

查看三类空间时，请选择IK模式，或在参考模型中载入“选型表 / 空间与灵巧分类”。通过“显示分类”可筛选全部空间可达点（蓝+绿+橙）、仅灵巧点、部分姿态点、已证明不可达点或待定点。
仅做FK点云采样时无法判断不可达区域，灵巧空间显示“未评估”。

带注释样例：[worksheet-classification.jsonc](../arm-workspace-parametric/assets/examples/worksheet-classification.jsonc)。当前固定输入产生4个目标：目标0为两个采样朝向全通过；目标1为位置可达但指定朝向均未解；目标2超出0.635m几何外界；目标3在界内未找到位置解，保持待定。它们分别呈绿、蓝、红、灰色。

其他模型可以在独立工作空间视图中正确显示点云和杆链，不会强行套用当前SRS展示模型。模型/输入修改或网页重新计算后，旧结果失效；运行中改变输入会丢弃过期响应。调用失败时不会回退显示网页旧点云。

## 独立运行

```powershell
cd E:\Robot\armDesignTool\arm-workspace-parametric
node scripts/run.mjs --input assets/examples/worksheet-workspace.jsonc --output outputs/workspace_result.json --report-dir outputs
node scripts/run.mjs --input assets/examples/worksheet-reachability.jsonc --output outputs/reachability_result.json --report-dir outputs/ik
node scripts/run.mjs --input assets/examples/dexbot-right-arm.json --output outputs/right_result.json --report-dir outputs/right
```

生成主JSON、points.csv、report.md、request.json、projection-xy/xz/yz.svg及section-xy/xz/yz.svg。
结果目录已加入Skill自身的.gitignore，避免把大量点云误提交。

URDF转换可使用Python标准库脚本：

```powershell
python scripts/import_urdf.py --input robot.urdf --base-link arm_base_link --tip-link arm_r_link_8 --output outputs/robot-input.json
```

它只提取指定串联路径，不需要STL，也不导入碰撞与动力学信息。完整契约见[输入说明](../arm-workspace-parametric/references/input-contract.md)。

## 实现结构

```text
arm-workspace-parametric/
  SKILL.md / agents/openai.yaml
  scripts/lib/kinematics.mjs   共享FK、Jacobian、SO(3)误差与线性求解
  scripts/lib/workspace.mjs    输入检查、采样、IK、成功率与统计
  scripts/lib/exports.mjs      CSV、报告、投影/剖面SVG
  scripts/lib/input-text.mjs   独立JSONC解析，无运行时跨Skill依赖
  scripts/run.mjs              CLI与原子结果写入
  scripts/import_urdf.py       标准库URDF串联路径转换
  scripts/tests/               数学、模型、边界、CLI和转换测试
  assets/examples/             选型表FK/IK及Dexbot左右臂
  references/                 契约与源码分析、来源哈希

srs-lab-source/
  server/index.mjs             /api/workspace、默认发现、双并发上限与超时
  dist/workspace-client.mjs    请求生成、执行来源检查、共享FK姿态重放
  dist/workspace-scene.js      世界点云及任意串联模型杆链预览
  dist/app.js                 可达空间页及失效管理
```

前端预览加载Skill同一份几何FK模块，不复制采样或IK算法。后端使用独立临时目录、固定CLI参数、结果结构检查及清理；每次请求最多2MiB，最多2个并发工作空间任务。没有持久化队列和多用户鉴权。

## 验证与范围

- 新Skill的17项Node测试覆盖解析几何、随机/网格、姿态重放、安装变换、雅可比差分、180°姿态误差、IK成功/失败、球采样去重、预算拒绝、CLI确定性和失败保护。
- Dexbot左右臂共20组姿态与独立Python 4×4变换结果对照，位置和旋转矩阵误差小于1e-9。
- 网页16项Node测试覆盖原功能、新模块、所有网页模板/自定义轴的模型转换、真实CLI/API一致性、错误无回退，以及页面控件、姿态应用、异模型拒绝和过期结果丢弃。
- URDF转换器另有3项Python标准库测试。
- 当前未完成真实浏览器视觉验收；页面测试使用最小DOM和Canvas模拟，不等同于浏览器端到端验收。

参考项目依赖GPU、随机序列与IK求解器均不同，不能将本Skill的成功率与历史cuRobo报告直接视为同一次实验结果。原项目细节和几何差异见[源码分析](../arm-workspace-parametric/references/dexbot-source-review.md)。
