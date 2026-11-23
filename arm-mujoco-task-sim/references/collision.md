# 从装配复核到任务接触验收

先读 `model_inspection.json` 的 `collision_review`。用 `parts[].mjcf_geom` 对照场景中的机械臂碰撞体，查阅装配例外、待复核姿态和最差工况。上游失败时修正机械臂。旧项目的 `not_evaluated` 表示应在任务场景补做检查。

## 给每个任务阶段定义接触

新合同使用 `schema_version: "1.1"`。从初始状态开始，按控制器实际返回的 phase 列出该阶段允许和必须接触的几何体，未列出的接触默认禁止。使用碰撞 geom 名称。角色可将多个 geom 分组，同一角色对的 `require` 表示其中至少一对有实际接触。双侧夹持须为左右指垫各写一条。允许支撑面的接触时应包含桌面、托盘、地面所支撑的实际物体。

下面是双指夹持样例的一段合同，数值仅演示字段单位。实际任务应替换来源及阈值，并补齐每一个阶段。

```json
{
  "collision_policy": {
    "default": "forbid",
    "required_phases": ["carry"],
    "roles": {"left_pad": ["left_pad_collision"], "right_pad": ["right_pad_collision"]},
    "phases": {
      "initial": [
        {"a":"part","b":"table","mode":"allow","max_depth_m":0.0003,"source":"样例支撑面容差"}
      ],
      "carry": [
        {"a":"left_pad","b":"part","mode":"require","max_depth_m":0.001,"min_force_N":0.2,"grace_s":0.006,"source":"样例夹垫压缩和保持判据"},
        {"a":"right_pad","b":"part","mode":"require","max_depth_m":0.001,"min_force_N":0.2,"grace_s":0.006,"source":"同一夹持试验"}
      ]
    },
    "barriers": [{"wall":"table","moving":"part","axis":"z"}]
  },
  "collision_path_mode": "planned"
}
```

- `allow` 允许该阶段的接触，`require` 还要求建立并维持接触，`forbid` 显式指定禁止接触的部件对。两条规则展开后不可覆盖同一对几何体。显式规则会检查场景碰撞掩码、父子过滤和排除项。监测范围是刚性几何体。
- 新合同的 `allow`、`require` 需要 `max_depth_m` 和 `source`。按工具柔顺性、装配公差或已确认试验确定值。可选 `max_force_N` 为同一几何对的压缩法向力限值，`max_violation_s` 为超深/超力允许持续的时间。`grace_s` 允许建立或重新获得必需接触的时间，默认立即判定。长行程内只允许指垫接触工件时，指背、壳体和腕部仍受禁止规则约束。
- `required_phases` 填写每轮都必须执行的阶段。运行器在完成时检查阶段覆盖，按条件进入的恢复分支留在普通阶段表。需要正净空的禁止接触对可设置带来源的 `min_clearance_m`，用于机械臂与工装的接近预警和失败判定。
- 薄壁检查的 `wall` 是盒形碰撞体，`axis` 是其局部法向轴。检查 `moving` 几何中心在相邻样本之间是否穿过该面的有限范围。弯曲障碍、边缘擦碰和快速旋转仍需轨迹细分及实际接触检查。
- 为关键表面填写可选的 `collision_geometry`，例如 `{"table":{"type":"box","size_m":[0.4,0.3,0.01],"initial_center_m":[0,0,0.49],"tolerance_m":0.0001,"source":"样例工装尺寸图"}}`。尺寸按 MJCF 定义，盒体用半长，圆柱和胶囊用半径及半长。中心为初始化后的世界坐标。填写在合同顶层。检查点为控制器初始化后和每次重新创建 Session 时。

## 将规划器的实际插值交给预检

| 合同的 `collision_path_mode` | 适配器接口 |
|---|---|
| `planned` | 控制器实现 `collision_plan(model, data)`，按次序返回实际计划段。每段为字典，包含 `phase`、可选 `cycle`、可调用的 `sample_qpos(u)` 和 `travel_bound_m`。 |
| `online` | 在每次非终止 `control` 的返回值中提供 `collision_segments`，使用与实际指令一致的短时预测。默认使用当轮和当前阶段。 |
| `feedback_only` | 控制仅给出力或反馈、没有可用位置预测时，在 `collision_path_reason` 说明原因。报告记录已检查轨迹段数和运行期逐步覆盖。 |

`sample_qpos(u)` 对归一化参数 0 到 1 返回完整的 `model.nq` 维姿态，包括抓持物体的运动以及所有活动关节。首段从实测姿态开始，以后各段承接上一段终点，采样函数只读规划状态。用路径实际的速度曲线估计每单位 u 的最大表面相对位移上界，包含转动半径、相对运动和插值峰值。可传 `resolution_m` 进一步缩小采样间隔。每段的 `collision_policy.max_path_samples` 预算默认 65536；超限时报告诊断，检查单位和路径分段，再按需要提高预算。控制器改线后，在下发新指令前交付新的预测段。预检在独立物理状态中检查几何，运行时按真实接触验收力和持续时间。

## 检查与定位

1. `python scripts/run_task.py --project <project> --preflight` 执行初始场景和可用的预规划段。读取返回的 run 路径和 `reports/preflight.json`。在线控制的预测段在控制指令产生时才可检查。
2. 正式运行时 Web、本地和无窗口模式共用相同监测。首次异常见 `reports/result.json` 的 `collision_failure` 和 `data/events.jsonl`。查看阶段、对象、姿态、实测值及来源。在确有任务依据时修改规则，几何错误优先改摆位、路径或碰撞代理。
3. 历史合同可照原命令复跑，缺少规则时显示 `not_evaluated`。对比时同时查看任务判定、碰撞状态、已检查轨迹段数和上游装配结论。
4. `python scripts/collision_regression.py --output <新目录>` 可生成并运行隔离的桌面和薄壁正反例。
