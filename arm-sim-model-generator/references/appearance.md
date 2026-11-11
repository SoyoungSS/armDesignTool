# 原生外形规则 rounded-shell-v2

依赖与原版相同：核心只用 Python/NumPy，不使用 CadQuery、Blender、CAD 文件或生成式 3D 服务。相同输入和依赖版本得到相同 Mesh；固定公式与模板位于 scripts/arm_sim/surfaces.py，装配在 geometry.py。不能用好看的图片替代实际逐连杆 Mesh。

## 优先级与确定规则

1. 整机比例：尺度 S 为各 origin 平移长度与 tip 平移长度之和，至少 0.12m。第 i 个连杆默认宽度为 clamp(max(0.085*S*(1-0.44*i/N), local_span*width_ratio), min_width, max_width)，N 为关节数；默认厚度为宽度的0.82倍。零跨度关节不再缩成最小模组。显式尺寸优先，来源逐项记录。
2. 安装空间：默认模组直径为连杆宽度乘 module_ratio；非基座再受最近两侧非零安装距离的90%约束，查找跨过共点帧。该规则是空间启发，不是实际电机尺寸计算；用户明确的直径保留，冲突报告。基座适当放大，模组轴宽默认不超过直径的0.72倍。
3. 连续外壳：使用48点超椭圆截面、多轴向截面和收分/鼓起曲线构成闭合实体。长连杆主体直接跨接原安装位置，去除外露细颈。截面短轴尽量对齐上游关节轴在梁法平面上的投影，避免斜轴时任意扭转。薄圆肩与端部收分都写入真实顶点。
4. 模板：基座为 pedestal；移动轴为 linear_carriage / linear_support；固定轴为 fixed_adapter；末端为 tool_wrist。旋转轴按相邻轴平行度与偏移选择 axial_spindle、offset_parallel、elbow_shell、coaxial_wrist、cross_axis_wrist。共点组合先按 [模组布置规则](joint-layout.md) 分离固定壳与输出件；未解析的交叉腕保留概念壳体并提示待复核，偏置/肘部使用端部扩大的多截面梁。模板主要决定壳体截面、端部过渡与材质区域，不声称复原了内部传动或完整叉架机构。共点组合会截短上游外壳以露出轴向分界，不改变运动学安装点。
5. 表面材质：浅灰主体、深色盖板、金属法兰，默认很少使用强调色。USD 写入平滑顶点法线及 UsdPreviewSurface 的颜色/粗糙度/金属度；MuJoCo 使用其材质高光参数近似；URDF 使用 RGBA。STL 保留几何，材质属于目标格式，不能承诺三个引擎光照一致。
6. 有限细节：双侧端盖、金属中心盖、基座安装圈和末端4个简化紧固件。不生成真实螺纹/内部齿轮。`dimensions.appearance.detail_level` 为1默认；为0去除中心盖、安装圈和紧固件。细节不参与碰撞或密度计算。

各字段和单位仍见 [输入契约](input-contract.md)。没有外观字段也自动使用新规则；输入不必增加 CAD、Mesh 或品牌名称。

## 物理与几何边界

- 连杆/工具的原始坐标、关节轴、安装、初始姿态保持原定义。跨关节的壳体不焊成一个刚体。
- 同一连杆由若干闭合部件组成；主体梁自身连续，端盖/壳体之间可以有覆盖连接。没有 CAD 布尔并集，合并 STL 不保证是单一无自交实体，不能把其总体积直接当成质量。
- 碰撞是独立低面数近似：主体中央盒体、模组圆柱、导轨盒体。`collision_size` 可不同于外壳包络；圆滑端部由模组近似覆盖，不代表精确外表面接触。两种仿真目标共用这些代理，接触排除仍仅按已有安装拓扑规则，不能为了外观通过而任意关闭远端碰撞。
- 已提供的质量/COM/惯量和驱动能力保持原值。显式启用密度或质量代理的概念模型会随着新默认尺寸重新估算质量/惯量，并记录来源；不会把端盖/重叠体积加进去。
- 外廓上限包含实际生成顶点，超出即失败。检查初始/验收姿态和确定性采样的干涉候选；近似代理与有限姿态不能证明连续全行程无碰撞，真实外壳需设计复核。

## 检查和预览

生成后查看 report/appearance_design.json、normalized_model.json.geometry 和 report.md。验证闭合边、退化面、面朝向、原 FK、尺寸来源、材质绑定、物理属性及接触开启的驱动表现。

```bash
python scripts/generate.py generate --input assets/seven-axis-driven.json --output /absolute/output --runtime
python scripts/preview.py --run /absolute/output/runs/<run_id> --output /absolute/preview.png
```

预览还支持 `--view iso|front|side` 和 `--pose <验收姿态名>`。预览使用已有可选 Pillow，直接读取生成的 STL；改进了曲面插值着色，不更改 Mesh。预览放在 run 外，避免修改已校验清单。最终在实际目标加载检查；USD 离线成功不代表 Isaac/PhysX 已验收。
