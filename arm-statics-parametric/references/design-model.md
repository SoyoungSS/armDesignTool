# 公共设计模型请求

公共请求将模型与分析设置分开。格式：`{schema_name:"arm-analysis-request",schema_version:"1.0",design_model:完整模型对象,options:分析设置}`。design_model 不是文件路径，完整格式见 assets/examples/design-*.json。

静力学需要每关节 components、部件质量和局部 COM、世界重力、显式 payload；不需要惯量。options 允许 poses、search、attachments、wrenches、drives、design_factor。poses 和 search 至少提供一个。

模型 joints 按串联顺序给出，稳定 id 与唯一 name 分开保存；origin、axis、tip 单位 m/rad。base_in_world 缺省单位位姿。质量来源与估算假设放在 provenance.source/assumptions，不用零替代缺失数据。

结果 input_snapshot 保留原公共请求；design_model_sha256 标识完整模型；design_model_info 包含来源、假设、ID 映射及已具备的属性。公共适配后的原生请求仍可使用旧 analyze/CLI，标准化导出亦可复算。

独立调用示例（在包含 shared 同级目录的 bundle 中）：

```bash
node scripts/run.mjs --input assets/examples/design-seven-axis.json --output /tmp/result.json --report-dir /tmp/report
```

运行时没有 npm 第三方依赖。失败返回非零退出码与结构化 INVALID_INPUT/字段位置，不回退到示例结果。网页不是必需环境。
