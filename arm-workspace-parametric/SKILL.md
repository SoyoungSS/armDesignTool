---
name: arm-workspace-parametric
description: 参数化串联机械臂工作空间分析，区分空间位置可达、采样灵巧空间、部分姿态可达、几何不可达与未解点。支持FK点云、目标位姿IK、投影剖面及网页联动；不证明连续全姿态灵巧性或碰撞、带载可达性。
---

# 机械臂可达工作空间分析

先读 [输入与结果契约](references/input-contract.md)。参考来源和与GPU原项目的差异见 [Dexbot源码分析](references/dexbot-source-review.md)。

- 使用标准串联几何输入，不要求质量、质心或重力，也不把缺失CAD参数补成现有样例。
- `joint_sampling` 计算限位内随机/网格姿态的末端点云；`pose_reachability` 检查指定点或球壳点及可选姿态。两者含义不同。
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

CLI导出JSON、点云CSV、Markdown报告及XY/XZ/YZ投影SVG。网页后端调用相同CLI；失败不得显示旧网页点云作为新Skill结果。新输入可参考 `dexbot-right-arm.json`、`dexbot-left-arm.json` 和 `worksheet-reachability.jsonc`。

三类空间及未解点的带注释演示见 `assets/examples/worksheet-classification.jsonc`。分类颜色、筛选、JSON/CSV字段与报告使用统一定义。
