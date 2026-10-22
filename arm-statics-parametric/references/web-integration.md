# 网页与命令行集成

## 共用内核

无第三方运行依赖，Node.js≥20（推荐22/24）。浏览器支持ES模块、模块Worker、TextEncoder及structured clone。

```js
import {analyze,toStandardInput} from './scripts/lib/statics.mjs';
const result=analyze(request);
const standardRequest=toStandardInput(request);
```

references/schemas/request.schema.json用于表单预校验，内核仍检查名称引用、限位和非法数字。不要在计算前四舍五入。

## 浏览器

保留scripts/lib/和assets/web/相对目录：

```js
import {analyzeInWorker} from './assets/web/adapter.mjs';
const controller=new AbortController();
const result=await analyzeInWorker(request,{
  timeoutMs:120000,signal:controller.signal
});
// controller.abort() 取消并终止Worker
```

每次请求独立Worker并释放资源，避免阻塞主界面。显示格式化数值，完整JSON保留原双精度；三维图使用真实joint_geometry和tip_pose_in_base，展示错开不能写回模型。

参考页不是已修改的机械臂工作台。接入已有工具时映射现有表单与结果字段；没有源码时不声称已完成工作台改造。

## 本地参考页面与API

```bash
node scripts/server.mjs --port 8765
```

打开http://127.0.0.1:8765/，可选示例、上传JSON、计算、取消和下载结果，无需npm install。

POST /api/statics，Content-Type: application/json，请求为完整JSON。成功200返回{ok:true,result}；输入错误422返回{ok:false,error:{code,path,message}}。其他状态：JSON错误400、格式错误415、超过1MiB为413、忙碌503、计算超时408。

服务仅监听回环地址，每次计算独立Node Worker，最多2并发，120秒超时。浏览器跨源请求默认拒绝；现有服务端可直接导入内核或用同源代理。本参考服务不包含生产鉴权与多租户管理。

## CLI与导出

```bash
node scripts/run.mjs --input assets/examples/project-right-arm.json --output /absolute/result.json --report-dir /absolute/reports
```

生成主JSON、joint_summary.csv、report.md、normalized_model.json和standard_request.json。标准请求可复算；模型部分供后续动力学、URDF导出使用。

主JSON及每个配套文件分别原子替换，不是整目录事务。检查退出码；失败后旧文件可能仍存在。并发任务使用不同输出目录。

## 展示规则

区分Nm与N、带符号载荷与最大绝对值；显示设计系数是否已乘。not_sized和capacity_conditions_unknown不能显示绿色“合格”。保留估算COM及集中质量警告；不把完成搜索解释为碰撞、温升或结构合格。

## 带注释输入

CLI、HTTP 与参考网页使用 `scripts/lib/input-text.mjs` 的 `parseRequestText(text)`，支持严格 JSON 或带 `//`、`/* */` 注释的 JSONC（无尾逗号）。HTTP 接收 application/json 或 application/jsonc。对自行集成的网页，先解析文本，再把对象传给 analyze/analyzeInWorker；结果仍为严格 JSON。示例见 assets/examples/worksheet-minimal.jsonc。
