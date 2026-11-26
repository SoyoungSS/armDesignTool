# 需求与选择结果约定

统一使用按设计主题组织的知识库。支持自然语言咨询和自研方案交接，不要求初选用户先提供JSON或完整机械臂模型。下面是需辨认的信息，不是每次必须问完的表单。

## 输入

| 信息 | 记录方式与作用 |
| --- | --- |
| 任务与角色 | 搬运、接触、遥操、运动辅助、充电及必须动作；区分操作臂与运动辅助上肢 |
| 任务位姿 | 分接近/操作/退出记录目标点、工具姿态、哪些转动可自由、进出路径、单位与坐标；只知区域时照实记录 |
| 质量与负载 | 工件、夹爪、相机、六维力分别列质量与已知作用位置；缺项用未知，不把净工件当总载荷 |
| 动作与工况 | 屈肘/翻腕范围、速度、加速度、保持时长和接触条件 |
| 部署 | 固定、轮式、双足、双臂、导轨，安装、包络、自重限制 |
| 控制与工程 | 位置/力矩接口、力感知、拖动、可用模组、维护与成本要求 |
| 既定选择 | 用户指定的轴数、机构、驱动和参照；哪些可以修改 |

每项区分用户确认、材料案例、本次计算、概念假设、未知。概念假设需有用途和依据，不用假设制造硬约束通过。仅用户给出的偏好或明确要求的权重可以进入偏好比较。

若用户只说家用机械臂，优先澄清具体动作、载荷和操作/安装区域；可先说明候选分支，不直接选定7轴型号。无需为初选提前索要完整轴表。

## 默认交付

先给需求摘要，分任务、硬约束、偏好与未知项，并说明处于概念初选或已有几何的方案复核。候选比较包含：

| 字段 | 内容 |
| --- | --- |
| 候选与状态 | 完整组合名称，candidate/conditional/excluded/insufficient_evidence之一 |
| 整臂几何 | F机构族、真实独立自由度、轴线/支链/形状描述；没有轴系时不宣称严格SRS |
| 肘腕 | 局部自由度、机构、耦合与范围；不填入未选定参考臂的限位 |
| 驱动和控制 | 旋转/直线、减速与远传、电机位置、传感与接口 |
| 部署 | 安装、固定/移动、单臂/双臂，与裸臂能力分开 |
| 理由与代价 | 对应需求的支持、已知冲突、未证实条件分别写 |
| 参考实现 | 商品、项目方案、研究样机、专利或仅路线分析 |
| 证据与验证 | 来源编号及章节/附件位置、证据类型、下一步计算 |

随后给排除/待定理由与下一步。排除具体设计须指出冲突对象和依据，不扩大到所有同类机构。材料不足时保留条件候选；知识库内无支持候选不证明其他路线不存在。

状态解释：candidate只是列入比较，不表示验收通过；conditional需满足列出的条件；excluded为明确约束冲突；insufficient_evidence为关键判断缺证据。总体可标shortlist、needs_requirements或no_supported_candidate，均不等于动力学、碰撞或热验收通过。

## 结构化交接

需要机器记录时使用以下概念格式。它不是其他分析技能的有效请求，也不是arm-design-model。

```json
{
  "schema_name": "arm-configuration-selection",
  "schema_version": "1.0",
  "outcome": "needs_requirements",
  "requirements": [
    {"id": "R1", "name": "payload", "value": null, "unit": "kg", "origin": "unknown", "kind": "hard_constraint"}
  ],
  "candidates": [],
  "open_questions": ["工件及末端工具分别有多重？"],
  "validation_plan": []
}
```

有候选时每项至少有id、status、configuration、fit_reasons、tradeoffs、constraint_checks、evidence、pending_validations。configuration分开记录arm_dof、axis_layout、elbow、wrist、drive_layout、sensing_control、deployment；缺项用null。

constraint_checks引用需求id，结果用meets/conflicts/unknown/not_applicable，保留证据与作用范围。参考实现符合自身规格，不证明新设计符合用户规格。

每条evidence记录source_id、locator、evidence_type、supports、scope。evidence_type统一使用report、research、textbook、vendor_document、patent、engineering_inference、user_input、current_calculation；不按资料提供方设置类型。claim_kind为fact或inference。推理记录supporting_source_ids、assumptions及pending_validations；本次计算另需真实输入/模型、方法、结果文件与覆盖范围，未运行不能用current_calculation。

来源编号只用于追溯和解释支持范围，不决定该条资料能否进入候选比较。

## 容易混淆的边界

- 要求腕部±90°时，可以排除照搬本项目2-SPS-U设计，不据此否定所有并联腕。联合姿态域与坐标仍需核对。
- 用户3kg、600mm仅与参考尺度相近，不直接复制模组、臂重或带载速度。
- 仅给场景而要求模型时，先说明几何缺项；不导出虚构的下游请求。
- 精细力控同时比较感知、接口、刚度与机构，不只回答混合方案。
