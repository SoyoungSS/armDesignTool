# 当前公开机械臂示例

更新日期：2026-11-09。公开模型仅保留两个；同一模型的分析工况和请求格式不是新的机械臂。

| 模型 | 参数模型 | 独立 Skill 请求 | 能力 |
|---|---|---|---|
| 选型表七轴单臂 | examples/models/seven-axis.json | 两个 Skill 的 assets/examples/design-seven-axis.json；原 worksheet 系列工况 | 主网页模型、静力学、几何与工作空间 |
| Franka FR3 | examples/models/franka-fr3.json | arm-workspace-parametric/assets/examples/franka-fr3.workspace.json | FK、位置/姿态 IK、体素灵巧空间、独立几何预览 |

## FR3 来源与输入

源文件为用户提供的 franka_fr3.workspace.json，复制时保留原始字节。文件指纹在工作空间 Skill 的 references/franka-fr3-source.json。内置请求自包含，不依赖源文件 metadata 中提及的 URDF 路径。

FR3 有七个主动旋转关节和 fr3_joint8 固定关节；末端是 fr3_link8 法兰，不含夹爪或额外 TCP。保留各关节 origin RPY、轴、限位、固定末端和 30,000 个 FK 样本的设置。文件未提供质量、质心和惯量，因此没有 FR3 静力学/动力学示例。

## 网页使用

网页默认主模型为选型表。进入“可达空间”，在参考模型下拉框选择选型表或 Franka FR3，点击“载入示例”，再运行。切换分析方式只转换采样/目标设置，保留当前模型和来源。

FR3 不强行应用到主网页选型表七轴模型；使用独立视图预览。更新后需重启服务并 Ctrl+Shift+R。旧主模型配置在 localStorage 的 srs-config-before-example-cleanup 中备份，再回到选型表默认；选型表用户配置保留。

## 历史数据

单杆、两杆、移动关节、旧七轴概念臂及 CAD/Dexbot 左右臂已移至相应 scripts/tests/fixtures；公共单杆/两杆模型和保持工况移至 shared/tests/fixtures。旧浏览器输入示例移至网站 tests/fixtures。它们仅用于数值回归，不列入网页或公开模型目录。

旧浏览器内核构型实现保留为历史兼容和回归依据，主页面不再提供相应预设，也不将 FR3 几何改写成该旧内核的近似模型。

## 本轮验证

网站 21 项、静力学 64 项、工作空间 22 项、公共契约/分发 10 项通过。包括新增 FR3 文件指纹、独立齐次变换对照、30,000 个姿态的关节限位、位置 IK 重放，以及模式转换保留几何。

实际浏览器已完成 FR3 加载、真实 CLI 计算和三维预览；工作空间结果共 30,000 点，异模型应用按钮保持禁用。体素模式的转换已验证，本轮未运行 FR3 全量体素搜索。
