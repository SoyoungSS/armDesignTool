---
name: arm-workspace-parametric
description: 参数化串联机械臂工作空间分析，支持 FK 点云、目标位姿 IK、按朝向成功率着色的灵巧空间体素图及可移动剖面。区分几何不可达和给定求解设置下未找到解。
---

# 机械臂可达工作空间分析

先读 [输入与结果契约](references/input-contract.md)。参考来源和与GPU原项目的差异见 [Dexbot源码分析](references/dexbot-source-review.md)。

- 使用标准串联几何输入，不要求质量、质心或重力，也不把缺失CAD参数补成现有样例。
- `joint_sampling` 计算限位内随机/网格姿态的末端点云；`pose_reachability` 检查指定点或球壳点及可选姿态。两者含义不同。
- `dexterity_voxels` 在基座系规则体素中心测试共用的均匀随机朝向，按成功比例着色。FR3 全空间初扫可用 **10 cm、20 个朝向**。分析会测试全部计划位姿，网页可手动取消。若需更细分辨率，可用 `bounds_base_m` 限定任务区域。
- 用关节名称映射姿态。关节轴在安装后的局部坐标系，长度m、角度rad。基座到世界变换单独给出，目标与结果基坐标几何均在B中。
- IK失败表示声明预算内未找到解，不证明不可达。姿态成功率仅针对输入采样集合，不是连续姿态空间比例或体积。
- 位置IK与姿态IK分开检查；位置有解而指定朝向无解仍计为空间可达。至少两个不同旋转全部找到解，才显示“采样灵巧空间”，单姿态或周期重复姿态不作为灵巧判据。
- 只有超出三角不等式保守几何外界的目标才标为不可达；界内未解点为待定。FK采样无法判定未采样区域，不能据点云空洞声称不可达。
- 保存成功解、误差、输入哈希和内核版本，重放前复核关节限位与模型一致性。
- 不检查碰撞、自碰撞、负载扭矩、热或双臂协同；左右臂单独运行不等于双臂协同验证。

Node.js >=20，无第三方运行依赖：

```powershell
node scripts/run.mjs --input assets/examples/worksheet-workspace.jsonc --output outputs/workspace_result.json --report-dir outputs
node --test scripts/tests/*.test.mjs
```

CLI导出JSON、点云CSV、Markdown报告及XY/XZ/YZ投影SVG。网页后端调用相同CLI；失败不得显示旧网页点云作为新Skill结果。新输入可参考 `franka-fr3.workspace.json` 和 `worksheet-reachability.jsonc`。历史 Dexbot 数据已移入测试夹具。

三类空间及未解点的带注释演示见 `assets/examples/worksheet-classification.jsonc`。分类颜色、筛选、JSON/CSV字段与报告使用统一定义。


## 公共参数模型与独立分发（P0/P1）

CLI 与 `analyze(input)` 另外接受 `schema_name: "arm-analysis-request"`、`schema_version: "1.0"` 的公共请求，design_model 内嵌参数模型，options 保留本 Skill 的分析设置。无需网页、CAD 或 URDF。完整示例为 `assets/examples/design-seven-axis.json`；读取 [公共输入说明](references/design-model.md) 后再构造这种请求。旧格式继续支持。

独立执行仍使用本 Skill 的 `scripts/run.mjs --input ... --output ... --report-dir ...`。共享代码位于同包根的 `shared/`，单独分发时使用仓库 `shared/scripts/package-skill.mjs` 生成 bundle；保留 skill 与 shared 同级结构，不依赖网页源码或原始仓库路径。


## 当前示例

公开模型仅有选型表七轴单臂和 Franka FR3。FR3 输入为 `assets/examples/franka-fr3.workspace.json`，完整保留用户提供的 origin RPY、七个主动关节限位和末端固定关节。其末端为 fr3_link8 法兰，不含夹爪/TCP；无质量/惯量，不能据此做静力学或动力学。来源指纹见 `references/franka-fr3-source.json`。
