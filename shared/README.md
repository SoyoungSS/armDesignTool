# P0/P1 公共契约与适配

这不是新的 Skill，也不要求网页或 URDF。`contracts/` 定义公共对象和无依赖校验；`model/` 负责分级模型审计及现有两个 Skill 的转换；`kinematics/serial.mjs` 是浏览器与 Node 共用的几何计算。

## 分级输入

`arm-design-model` 1.0 直接描述参数化串联链。关节 origin 相对前一输出刚体；轴定义在 origin 安装后的局部系；components 的 COM 与质心惯量定义在当前输出刚体系。惯量单位 kg·m²，矩阵关于 COM，不能填关于关节轴的惯量。

- geometry：关节 ID/名称、类型、origin、轴/限位、tip、来源；不要求质量。
- mass_properties：每个关节显式给 components；`[]` 表示明确无质量；部件包含 mass_kg、com_m、provenance。静力学另要求明确世界重力及 payload（无负载填零）。
- inertial_properties：所有正质量部件再给对称、物理可实现的 inertia_kg_m2。单杆/两杆样例采用理想细杆，七轴样例故意不伪造惯量。

payload 当前表示末端附加点质量，com_in_tip_m 是相对 tip 的位置；需要负载转动惯量的后续动力学契约将另行扩展，当前能力审计只覆盖关节 components。

base_in_world 缺省为单位位姿。工作空间保留平移和旋转；静力学只取旋转变换世界重力，世界平移不影响均匀重力下的关节保持载荷。未提供重力、负载或部件信息时静力学拒绝运行，不默认补零。

## 同一模型用于两个 Skill

```json
{
  "schema_name": "arm-analysis-request",
  "schema_version": "1.0",
  "design_model": { "说明": "填写 examples/models 中的完整模型对象，不是路径" },
  "options": { "说明": "各 Skill 自己的分析设置，不允许覆盖模型字段" }
}
```

完整可执行示例位于两个 Skill 的 `assets/examples/design-seven-axis.json`。原有请求格式和 CLI 参数继续支持。

静力学 options 支持 poses/search/attachments/wrenches/drives/design_factor；工作空间支持 mode/sampling/reachability/dexterity/sections。不要混用。模型 ID 与名称的映射、来源、假设和模型哈希通过结果 design_model_info/design_model_sha256 保留，原始公共请求保存在 input_snapshot。

## 校验与 Schema

```bash
node --test shared/tests/*.test.mjs
node shared/contracts/export-schemas.mjs
```

JSON Schema 2020-12 文件由 schemas.mjs 导出，测试防止定义漂移。依赖-free 运行时校验器只使用本项目声明的 Schema 子集；不是通用 JSON Schema 引擎。惯量物理性、ID 唯一、轴/限位、时序等跨字段约束由运行时补充。case/trajectory/joint-trace/run-manifest 是 P0 数据契约，不代表轨迹、仿真或导出 Skill 已实现。

## 脱离仓库运行与分发

因为共用代码位于 Skill 目录外，不能只复制单个 Skill 目录。使用打包器生成可搬迁目录：

```bash
node shared/scripts/package-skill.mjs arm-statics-parametric /tmp/statics-bundle
cd /tmp
node /tmp/statics-bundle/arm-statics-parametric/scripts/run.mjs --input /tmp/statics-bundle/arm-statics-parametric/assets/examples/design-seven-axis.json --output /tmp/statics-result.json --report-dir /tmp/statics-report
```

工作空间将名称替换为 arm-workspace-parametric；轨迹 Skill 使用 arm-kinematics-dynamics（示例路径改为 assets/examples/franka-fr3-motion.json）。目标 bundle 目录必须尚不存在。分发时保留 `<bundle>/<skill>/` 与 `<bundle>/shared/` 的相对位置；整体可放到任意目录，不需要网页、Node 依赖安装或原仓库。打包时包含共享契约和模型/几何模块。

轨迹第一阶段请求为 `arm-motion-request` 1.0，Schema 见 contracts/motion-request.schema.json。其轨迹生成与状态分析已实现，和原 P0 仅定义的未来载荷契约区分。
