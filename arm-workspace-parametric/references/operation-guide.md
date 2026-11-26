# 操作细则

命令和反引号中的相对路径均以本技能根目录为基准；本页的 Markdown 链接相对于本页。仅阅读入口指定的相关章节。示例参数只用于演示，不是用户设计或厂家能力。版本清单和历史验收不代表当前环境已验证。

先读 [输入与结果契约](../references/input-contract.md)。参考来源和与GPU原项目的差异见 [Dexbot源码分析](../references/dexbot-source-review.md)。

- 使用标准串联几何输入，不要求质量、质心或重力，也不把缺失CAD参数补成现有样例。
- `joint_sampling` 计算限位内随机/网格姿态的末端点云；`pose_reachability` 检查指定点或球壳点及可选姿态。两者含义不同。
- `dexterity_voxels` 在基座系规则体素中心测试共用的均匀随机朝向，按成功比例着色。FR3 全空间初扫可用 **10 cm、20 个朝向**。分析会测试全部计划位姿，网页可手动取消。若需更细分辨率，可用 `bounds_base_m` 限定任务区域。
- 用关节名称映射姿态。关节轴在安装后的局部坐标系，长度m、角度rad。基座到世界变换单独给出，目标与结果基坐标几何均在B中。
- IK失败表示声明预算内未找到解，不证明不可达。姿态成功率仅针对输入采样集合，不是连续姿态空间比例或体积。
- 位置IK与姿态IK分开检查；位置有解而指定朝向无解仍计为空间可达。至少两个不同旋转全部找到解，才显示“采样灵巧空间”，单姿态或周期重复姿态不作为灵巧判据。
- 只有超出三角不等式保守几何外界的目标才标为不可达；界内未解点为待定。FK采样无法判定未采样区域，不能据点云空洞声称不可达。
- 保存成功解、误差、输入哈希和内核版本，重放前复核关节限位与模型一致性。
- 不检查碰撞、自碰撞、负载扭矩、热或双臂协同；左右臂单独运行不等于双臂协同验证。

## 双默认示例

在本工具目录使用 Node.js 20 或更高版本运行。选型表采用同一套七轴单臂的侧装模型，FR3 采用 Franka ROS 标称模型。两份请求均可单独运行。[选型表七轴单臂](../assets/examples/defaults/worksheet.json) · [Franka FR3](../assets/examples/defaults/fr3.json)

```bash
node scripts/run.mjs --input assets/examples/defaults/worksheet.json --output outputs/worksheet/result.json --report-dir outputs/worksheet
node scripts/run.mjs --input assets/examples/defaults/fr3.json --output outputs/fr3/result.json --report-dir outputs/fr3
```

两例分别用种子 42 在七轴关节限位内抽取 30,000 组姿态，导出基座系及世界系法兰点云、三个方向的投影图和 CSV。选型表采用侧装基座，FR3 采用标准直立安装。

FR3 保留七个活动轴的标称质量、局部质心和惯性张量、基座安装及 0.107 m 法兰偏移，使用无手爪、空载配置。上游公用惯性表采用 Gaz 等人针对 Panda 的辨识结果。[Franka 固定版本惯性来源](https://github.com/frankarobotics/franka_ros/blob/ddd2fffd9de44b02ad15b4bbb2bfa2cec4d60d98/franka_description/robots/common/inertial.yaml) · [上游许可证](../references/franka-fr3-LICENSE.txt)

CLI 导出结果 JSON、点云 CSV、Markdown 报告和 XY/XZ/YZ 投影 SVG。运行 `node --test scripts/tests/*.test.mjs` 可复核内核。

三类空间及未解点的带注释演示见 `assets/examples/worksheet-classification.jsonc`。分类颜色、筛选、JSON/CSV字段与报告使用统一定义。


## 公共参数模型与独立分发（P0/P1）

CLI 与 `analyze(input)` 另外接受 `schema_name: "arm-analysis-request"`、`schema_version: "1.0"` 的公共请求，design_model 内嵌参数模型，options 保留本 Skill 的分析设置。无需网页、CAD 或 URDF。完整示例为 `assets/examples/design-seven-axis.json`；读取 [公共输入说明](../references/design-model.md) 后再构造这种请求。旧格式继续支持。

独立执行仍使用本 Skill 的 `scripts/run.mjs --input ... --output ... --report-dir ...`。共享代码位于同包根的 `shared/`，单独分发时使用仓库 `shared/scripts/package-skill.mjs` 生成 bundle；保留 skill 与 shared 同级结构，不依赖网页源码或原始仓库路径。


## 历史几何输入

早期导入的 `assets/examples/franka-fr3.workspace.json` 保留原始用户几何、末端固定关节和旧软件限位，适用于重放该版本结果。其原文件指纹见 [历史来源](../references/franka-fr3-source.json)。
