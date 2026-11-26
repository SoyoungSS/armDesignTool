---
name: arm-workspace-parametric
description: >-
  分析参数化串联机械臂的FK采样点云、目标位置/姿态IK和离散朝向灵巧空间体素，输出成功解与可复现报告。用于工作范围和任务位姿覆盖分析；区分几何不可达与预算内未找到解，不检查碰撞或带载能力。
metadata:
  version: "1.2.0"
---

# 机械臂可达工作空间分析

## Purpose

用随附几何内核回答哪些位置/位姿找到解，保留模型、求解预算和结果边界。

## When to use

需要点云、指定目标可达性或离散朝向覆盖时使用；质量、驱动和任务接触不是本技能的计算目标。

## Inputs

先读[输入与结果契约](references/input-contract.md)。需要串联几何、局部关节轴、有限限位、tip及所选模式设置。长度m、角度rad；目标在基座系B，base_in_world独立表示安装，不能重复写入首关节。不需要质量、COM或重力。

| 用户问题 | mode | 需要的设置 |
| --- | --- | --- |
| 看限位内采到的末端位置 | joint_sampling | 随机/网格、数量及种子/固定轴 |
| 检查目标点及可选朝向 | pose_reachability | 目标点/球壳、可选朝向、IK预算与容差 |
| 按离散朝向成功率看体素 | dexterity_voxels | 分辨率、朝向数、任务区域与IK预算 |

原生请求使用契约；内嵌公共模型请求先读[公共输入](references/design-model.md)。URDF串联路径转换按输入契约末节执行，不能直接把静力学请求交给本CLI。

## Workflow

### Step 1 确定问题与坐标

从目标选择表中选一种模式，保留用户指定模式。目标坐标、轴系或单位不明时集中询问；没有真实几何时只整理输入，不能改用示例冒充。

### Step 2 建立请求并核对预算

按契约构造请求，关节名映射姿态，确认限位、tip、安装、采样范围和朝向集合。先核对点数/迭代预算；过大时说明影响并建议缩小区域或分批，不静默截断。只有用户选择默认演示时才使用FR3初扫10cm/20朝向。

### Step 3 执行

工作目录为本技能根目录。确认Node.js ≥20、同级shared存在。将REQUEST和RUN替换为实际请求和本次新目录，运行：

```sh
node scripts/run.mjs --input "REQUEST.json" --output "RUN/result.json" --report-dir "RUN"
```

记录退出码与stdout/stderr。非零退出时按错误path检查输入/预算/依赖，保留失败记录；旧结果不作为新请求结果。

### Step 4 分类并核对证据

读取本次summary、scope、warnings及成功解。位置IK与姿态IK分开；按关节名、限位和模型版本重放成功解。使用以下结论范围：

| 证据/分类 | 可以说什么 |
| --- | --- |
| FK点云 | 已采到的位置可达；空洞不能证明不可达 |
| spatial_reachable | 位置已有解；姿态覆盖不一定满足 |
| sampled_dexterous / partially_dexterous | 至少两个不同旋转的采样朝向全部/部分找到解 |
| unreachable | 超出保守几何外界，位置不可达 |
| unresolved / 体素0% | 给定预算/测试中未找到解，不能称不可达 |

### Step 5 交付

给出模式、范围、分类和代表解；按下方结构保存结果。研究来源或原GPU项目差异时再读[源码分析](references/dexbot-source-review.md)。

## Output format

1. 输入与设置：模型、安装/目标坐标、模式、预算、种子、容差。
2. 结果：成功与未解数量、适用分类/朝向成功比例、代表解及误差。
3. 证据：命令、退出码、输入/模型哈希、result.json、points.csv、report.md、request.json和projection/section SVG实际路径。
4. 限制与下一步：采样/求解边界、未知项和未执行检查。

## Rules

IK失败不证明不可达；离散姿态成功率不代表连续SO(3)覆盖率。重复旋转不能获得灵巧标签；采样体积不是真实空间体积。只支持固定基座串联几何，不检查碰撞、自碰撞、负载、热或双臂协同。主JSON原子替换不代表全部配套文件有目录事务。

## Examples

- “看FR3点云”：使用[双默认示例](references/operation-guide.md#双默认示例)，先选FR3，不额外启动体素全空间求解。
- “目标IK失败，是否够不到”：区分unreachable与unresolved，报告预算；位置有解但朝向无解也要分别说明。

## Resource loading and fallback

需要 Node.js ≥20、文件读写及随包 `shared/`；不要求网页、URDF、联网或特定模型API。`agents/openai.yaml` 如存在仅用于界面。独立分发必须保留技能与 shared 同级，详见[操作细则](references/operation-guide.md)。

- 能读文件并执行：按Workflow逐步运行，每步保留输入路径、输出路径、退出码和待处理项。
- 能读文件但不能执行：核对请求，提供可运行命令和待验证项，明确“未执行”，不给计算结果。
- 只有文本输入：由调用方提供本入口、相关契约和请求正文；缺资料时列出所缺字段，不猜字段、不假称读取文件。
- 上下文较小：先完成输入核对，再读取当前模式的契约；记录关节ID、单位、坐标、请求路径和检查进度后继续，不一次载入所有示例。
