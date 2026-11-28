const $=id=>document.getElementById(id);
const esc=value=>String(value??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const card=(title,body)=>`<section class="card"><div class="card-head"><h2>${title}</h2></div><div class="card-body">${body}</div></section>`;
const list=items=>items?.length?`<ul>${items.map(item=>`<li>${esc(item)}</li>`).join('')}</ul>`:'<p>暂无记录</p>';
const state={draft:{},health:null,models:[],job:null,message:'写一句场景描述即可开始。未指定参数由 Agent 按常规假设补齐，之后可继续描述修改。',modified:false,loading:false,exporting:false,error:null};
const busy=()=>['planning','preparing','checking','running','exporting'].includes(state.job?.status);
const fields=[['description','场景描述与动作顺序','描述操作对象、从哪里到哪里、接近/操作/退出过程'],['success_criteria','成功判据与失败退出','例如保持误差、放置稳定性、允许接触与失败时的停止条件'],['tool_environment','工具、环境与物理参数','工具质量/安装、工件尺寸/质量、摩擦、桌台/障碍、坐标与单位；已有模型参数不用重复添加'],['strategy','控制/规划策略要求','指定策略、观测来源，是否允许仿真真值；未定时由 Agent 提出可审阅方案'],['randomization','随机性要求','哪些参数随机、范围/分布；不随机也请说明'],['additional_notes','其他要求或补充答复','可把 Agent 上次提出的问题及你的回答填在这里']];
const labels={planning:'生成场景方案中',ready:'方案待审阅',needs_input:'需要补充信息',preparing:'准备场景与控制器',checking:'执行预检',running:'物理仿真中',exporting:'生成回放中',completed:'任务通过',simulation_failed:'任务未通过',failed:'执行失败',cancelled:'已取消',interrupted:'任务已中断'};
let timer=null,hooks={},checking=false;
const artifactURL=(job,path)=>`/api/scene-simulation/jobs/${job.id}/artifact?path=${encodeURIComponent(path)}`;
function planHTML(job){
  const plan=job.plan;if(!plan)return '';
  return card('本次采用的方案与假设',`<p class="selection-prose">${esc(plan.summary)}</p><p>轮数：${plan.cycles??'待明确'} · 最大仿真时长：${plan.max_time_s??'待明确'} s</p><h3>任务阶段</h3><div class="table-scroll"><table><thead><tr><th>阶段</th><th>动作</th><th>完成依据</th><th>异常退出</th></tr></thead><tbody>${plan.phases.map(phase=>`<tr>${['name','action','completion','on_failure'].map(key=>`<td>${esc(phase[key])}</td>`).join('')}</tr>`).join('')}</tbody></table></div><h3>策略</h3><p>${esc(plan.strategy)}</p><h3>成功判据</h3>${list(plan.success_criteria)}<h3>物理参数与估算范围</h3>${list([...plan.physics,...plan.assumptions])}${plan.missing_inputs.length?`<h3>当前执行阻塞</h3>${list(plan.missing_inputs)}`:''}<details><summary>范围与知识库来源</summary>${list(plan.limitations)}${list(plan.sources_read)}</details>${plan.follow_up_questions?.length?`<details><summary>后续可细化的信息（不影响本次常规试算）</summary>${list(plan.follow_up_questions)}</details>`:''}${plan.ready_to_run&&job.status==='ready'?`<p>点击运行表示按上方方案确认轮数、时长、策略、物理参数与估算范围，并授权后台计算与载荷记录。</p><div class="actions-row"><button class="button primary" id="sceneRun" ${state.modified?'disabled':''}>按此方案运行仿真</button></div>`:''}`);
}
export function jointExportsHTML(job){
  const result=job.result;if(!result?.run_path)return '';
  const index=result.joint_exports;
  const file=(path,label)=>`<a class="button primary" href="${artifactURL(job,result.run_path+'/'+path)}" download>${esc(label)}</a>`;
  const format=value=>value==null?'—':Number(value).toPrecision(6);
  const build=`<button class="button primary" id="sceneExports" ${state.exporting?'disabled':''}>${state.exporting?'正在整理全部关节…':index?.modules?'重新整理已有记录':'整理全关节 HTML 与 CSV'}</button>`;
  if(!index?.modules)return card('全关节报告与 CSV',`<p>已记录关节：${esc(result.load_coverage?.recorded_joints?.join('、')??'请查看覆盖报告')}。使用已有实际记录生成总报告，无需再次调用 Agent 或重新仿真。</p>${index?.error?`<p>${esc(index.error)}</p>`:''}${build}`);
  return card('全关节报告与 CSV',`<p>${index.modules.length} 个关节/接口 · ${index.frames} 帧 · 导出状态：${esc(index.status)}。HTML 内嵌全部曲线，可离线查看；CSV 保留全部记录样本。</p><div class="actions-row">${file(index.html,'下载全关节 HTML 总报告')}${file(index.wide_csv,'下载全关节状态宽表 CSV')}${file(index.summary_csv,'下载关节模组统计 CSV')}${file(index.dictionary,'下载字段与单位说明')}</div><div class="table-scroll"><table><thead><tr><th>关节模组</th><th>状态样本</th><th>六维载荷样本</th><th>驱动输出绝对峰值</th><th>数据覆盖</th><th>独立 CSV</th></tr></thead><tbody>${index.modules.map(entry=>`<tr><td>${esc(entry.joint)}</td><td>${entry.state_samples}</td><td>${entry.section_samples}</td><td>${format(entry.peaks?.effort)} ${entry.joint_type==='hinge'?'N·m':entry.joint_type==='slide'?'N':''}</td><td>${esc(({complete:'完整',state_only:'仅状态，接口未提供',missing_state:'状态缺失',missing_section:'接口载荷缺失',missing_actuator:'执行器记录缺失',no_samples:'无记录'})[entry.status]??entry.status)}</td><td>${file(entry.state_csv,entry.joint+' 模组状态')}${entry.actuator_csv?file(entry.actuator_csv,'执行器原始状态'):''}</td></tr>`).join('')}</tbody></table></div><p>状态 CSV 包含位置、速度、加速度、驱动输出、功率、六维接口载荷及参考坐标。未记录或不适用的量留空；固定连接不伪造运动自由度。载荷谱、越限、末端和接触数据仍保留在完整 ZIP 中。</p>${index.modules.map(entry=>`<details><summary>${esc(entry.joint)} · 状态曲线</summary>${entry.charts.filter(chart=>chart.png&&['position','velocity','acceleration','effort','power_W'].includes(chart.signal)).map(chart=>`<img loading="lazy" src="${artifactURL(job,result.run_path+'/'+chart.png)}" alt="${esc(entry.joint+' '+chart.signal)}" style="width:100%;height:auto">`).join('')||'<p>此接口没有独立运动曲线，详见 HTML 与 CSV。</p>'}</details>`).join('')}<details><summary>重新整理导出</summary>${build}</details>`);
}
function resultHTML(job){
  const r=job.result;if(!r)return '';
  const measured=r.measured;const proxy=r.simulation_scope==='load_proxy',concept=r.assumption_policy==='concept';
  const important=(r.artifacts??[]).filter(file=>(!r.run_path||file.path.startsWith(r.run_path+'/'))&&/reports\/(result|load_coverage|preflight|goal_validation|load_statistics|load_spectrum|effort_speed_spectrum|limit_exceedances)|data\/(joints|joint_loads|metadata)/.test(file.path));
  return card('实际执行结果与载荷记录',`${measured?`<p><strong>${measured.status==='passed'?(proxy?'本次代表性载荷代理运行完成':concept?'本次概念工况试算完成':'本次任务通过'):'本次工况未通过或未完成'}</strong> · 实际时长 ${esc(measured.duration_s)} s · ${esc(measured.samples)} 个记录样本</p><p>结束原因：${esc(measured.reason)} · 实际完成轮数：${esc(measured.evaluation?.cycles_completed??'未记录')}</p><details><summary>逐轮判定、碰撞与原始结果</summary><pre class="matrix">${esc(JSON.stringify(measured,null,2))}</pre></details><p>载荷谱：${r.load_spectrum?.status==='exported'?'本次实际记录已导出':'请检查载荷谱状态或诊断记录'}。${r.replay_error?'回放生成失败：'+esc(r.replay_error):''}</p>`:'<p>未完成实际运行，请查看错误信息与诊断文件。</p>'}${concept?`<p>概念试算${proxy?' · 当前为载荷代理，原场景的真实接触/抓取等动作尚未验证':''}。可直接修改上方描述重新计算。</p><details open><summary>实际采用的常规假设</summary>${list(r.assumptions)}</details>`:''}${list(r.limitations)}${r.preparation_notes?.length?`<details><summary>Agent 准备阶段说明（执行前）</summary>${list(r.preparation_notes)}</details>`:''}<div class="actions-row"><a class="button primary" href="/api/scene-simulation/jobs/${job.id}/archive">下载本次完整记录 ZIP</a></div><details open><summary>数据与诊断下载</summary><ul>${(important.length?important:r.artifacts??[]).slice(0,35).map(file=>`<li><a href="${artifactURL(job,file.path)}" download>${esc(file.path)}</a></li>`).join('')}</ul></details>`)+jointExportsHTML(job)+(r.replay_available?window.ResultViewer.taskReplayHTML():'');
}
function html(){
  const job=state.job,locked=busy()||state.exporting,health=state.health,policy=state.draft.assumption_policy??'concept';
  const field=([key,label,placeholder])=>`<label class="selection-field"><span>${label}</span><textarea name="${key}" rows="${key==='description'?4:3}" maxlength="12000" placeholder="${esc(placeholder)}">${esc(state.draft[key]??'')}</textarea></label>`;
  const status=job?.status==='completed'?(job.result?.simulation_scope==='load_proxy'?'载荷代理已完成':job.result?.assumption_policy==='concept'?'概念试算已完成':'任务通过'):labels[job?.status]??job?.status;
  return card('场景描述与仿真任务',`<p>描述要做什么，Agent 调用本工程场景仿真 Skill，按常规假设补齐并直接试算载荷谱。场景和模型报告会交给当前配置的 Agent 处理。</p><p role="status">${esc(health?.message??'正在连接场景仿真服务…')}</p><form id="sceneForm"><fieldset class="selection-form" ${locked?'disabled':''}><label class="selection-field"><span>仿真机械臂</span><select name="model_id"><option value="">${state.models.length?'自动使用最近导出的模型':'请先生成包含 MuJoCo 的模型'}</option>${state.models.map(model=>`<option value="${model.id}" ${state.draft.model_id===model.id?'selected':''}>${esc(model.name)}</option>`).join('')}</select></label><div class="actions-row"><button class="button" type="button" id="sceneModels">刷新模型</button><button class="button" type="button" id="sceneGenerateModel">打开仿真模型生成</button></div><div class="selection-fields" style="grid-template-columns:1fr">${field(fields[0])}</div><p>例如“在桌面上往返搬运小物体，先按常规轻载试算”。默认从一轮、低速代表工况开始，实际参数与假设随结果展示。</p><details><summary>更多约束与精确模式（可选）</summary><div class="selection-fields">${fields.slice(1).map(field).join('')}</div><div class="form-grid"><label class="selection-field"><span>轮数（留空由 Agent 补齐）</span><input name="cycles" type="number" min="1" max="1000" step="1" value="${esc(state.draft.cycles??'')}"></label><label class="selection-field"><span>最大仿真时长 / s（留空自动试算）</span><input name="max_time_s" type="number" min="0.001" step="any" value="${esc(state.draft.max_time_s??'')}"></label><label class="selection-field"><span>执行方式</span><select name="assumption_policy"><option value="concept" ${policy==='concept'?'selected':''}>常规概念试算，自动补齐并运行</option><option value="strict" ${policy==='strict'?'selected':''}>精确模式，先补齐并确认方案</option></select></label></div></details><p>后台物理计算＋网页实测回放。无法精细还原的工具/接触会标注为载荷代理；真实模型、驱动和碰撞检查仍保留。</p><div class="actions-row"><button class="button primary" id="sceneSubmit" type="submit" ${!health?.configured?'disabled':''}>${locked?'任务处理中…':policy==='strict'?'生成待确认方案':job?'按新描述重新仿真':'开始场景仿真'}</button></div></fieldset></form><p id="sceneMessage" role="status">${esc(state.message)}</p>${state.modified&&job?'<p>新描述将结合上次场景与参数重新试算，例如“速度减半”“负载改为 1 kg”。</p>':''}${state.error?`<p class="note">${esc(state.error)}</p>`:''}`)+
    (job?card('当前任务状态',`<p><strong>${esc(status)}</strong> · ${esc(job.stage)}</p>${job.error?`<p>${esc(job.error)}</p>`:''}<ol>${(job.events??[]).slice(-6).map(event=>`<li>${esc(event.message)}</li>`).join('')}</ol>${busy()?'<button class="button" id="sceneCancel">取消当前任务</button>':''}<button class="button" id="sceneReconnect">刷新任务状态</button>`)+planHTML(job)+resultHTML(job):'');
}
function render(){if(location.hash!=='#sorting')return;window.ResultViewer?.closeViewer();$('content').innerHTML=html();bind();}
function readDraft(){const form=$('sceneForm');if(form)for(const field of form.elements)if(field.name)state.draft[field.name]=field.value;}
async function services(){
  if(checking)return;checking=true;
  try{const [h,m]=await Promise.all([fetch('/api/scene-simulation/health'),fetch('/api/scene-simulation/models')]);if(!h.ok||!m.ok)throw Error('场景服务未连接，请确认后端已更新。');state.health=await h.json();state.models=(await m.json()).models;state.error=null;}
  catch(err){state.error=err.message;state.health={configured:false,message:err.message};}
  finally{checking=false;render();}
}
async function request(url,body){const response=await fetch(url,{method:'POST',headers:{'content-type':'application/json'},body:JSON.stringify(body)});const data=await response.json();if(!response.ok||!data.ok)throw Error(data.detail??data.error);return data.job;}
function remember(){try{sessionStorage.setItem('srs-scene-job',state.job.id);}catch{}}
function schedule(){clearTimeout(timer);if(busy())timer=setTimeout(refreshJob,1500);}
async function refreshJob(){
  if(!state.job)return;
  try{const response=await fetch(`/api/scene-simulation/jobs/${state.job.id}`);const data=await response.json();if(!response.ok||!data.ok)throw Error(data.detail??data.error);state.job=data.job;state.error=null;}
  catch(err){state.error=err.message;}
  render();schedule();
}
function bind(nextHooks){
  if(nextHooks)hooks=nextHooks;const form=$('sceneForm');if(!form)return;
  if(!state.health&&!checking)services();
  if(!state.job&&!state.loading){try{const id=sessionStorage.getItem('srs-scene-job');if(id){state.job={id};state.loading=true;refreshJob().then(()=>{state.loading=false;if(!state.modified&&state.job.input){state.draft={...state.job.input};render();}});}}catch{}}
  form.oninput=()=>{readDraft();state.modified=true;const submit=$('sceneSubmit');if(submit)submit.textContent=state.draft.assumption_policy==='strict'?'生成待确认方案':state.job?'按新描述重新仿真':'开始场景仿真';const button=$('sceneRun');if(button)button.disabled=true;};
  $('sceneModels').onclick=()=>{readDraft();services();};
  $('sceneGenerateModel').onclick=()=>{readDraft();hooks.go?.('simulation');};
  form.onsubmit=async event=>{
    event.preventDefault();if(busy()||state.exporting)return;readDraft();
    if(!state.draft.description?.trim()){$('sceneMessage').textContent='请先描述场景和任务动作。';return;}
    const input={...state.draft,assumption_policy:state.draft.assumption_policy??'concept',previous_job_id:state.job?.id??null};input.auto_run=input.assumption_policy==='concept';for(const key of ['cycles','max_time_s'])input[key]=input[key]?.trim()?Number(input[key]):null;
    state.error=null;state.message='正在提交，Agent 将补齐常规假设并试算；可随时取消。';
    try{state.job=await request('/api/scene-simulation/plan',input);state.modified=false;remember();render();schedule();}catch(err){state.error=err.message;render();}
  };
  if($('sceneRun'))$('sceneRun').onclick=async()=>{try{state.job=await request(`/api/scene-simulation/jobs/${state.job.id}/run`,{plan_sha256:state.job.plan_sha256});state.error=null;render();schedule();}catch(err){state.error=err.message;render();}};
  if($('sceneCancel'))$('sceneCancel').onclick=async()=>{try{state.job=await request(`/api/scene-simulation/jobs/${state.job.id}/cancel`,{});render();schedule();}catch(err){state.error=err.message;render();}};
  if($('sceneReconnect'))$('sceneReconnect').onclick=refreshJob;
  if($('sceneExports'))$('sceneExports').onclick=async()=>{
    if(state.exporting)return;readDraft();state.exporting=true;state.error=null;render();
    try{state.job=await request(`/api/scene-simulation/jobs/${state.job.id}/exports`,{});state.message='已按本次实际记录整理全部关节，无需重新仿真。';}
    catch(err){state.error=err.message;}finally{state.exporting=false;render();}
  };
  if(state.job?.result?.replay_available)window.ResultViewer.mountTaskReplay(artifactURL(state.job,'viewer.json'));
}
export const SceneSimulationPage={html,bind};
