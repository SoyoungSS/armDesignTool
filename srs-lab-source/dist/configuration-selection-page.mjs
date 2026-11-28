const esc=value=>String(value??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const $=id=>document.getElementById(id);
const card=(title,body)=>`<section class="card"><div class="card-head"><h2>${title}</h2></div><div class="card-body">${body}</div></section>`;
const state={draft:{},data:null,busy:false,message:'填写已知需求即可，暂不确定的项目可留空。',health:null,checking:false};
const textFields=[
  ['scenario','应用场景','例如：狭小工位内的零件取放，需绕过夹具'],
  ['task','具体任务与必须动作','描述接近、操作、退出过程，以及是否有接触、插装或翻转'],
  ['task_poses','目标位置、工具姿态与进出路径','哪些方向必须保持，哪些转动可自由；已知区域也可描述'],
  ['motion_conditions','动作与工况','速度、加速度、屈肘/翻腕范围、保持时间及接触条件'],
  ['environment','空间包络与环境','工位尺寸、障碍、洁净、防护、人机共域及自重限制'],
  ['control_requirements','驱动、感知与控制要求','位置/力矩接口、力感知、拖动、远传或可用模组'],
  ['hard_constraints','硬约束','必须满足的限制；没有确认的内容请留空'],
  ['preferences','设计偏好','成本、维护、轻量化、灵活性等偏好'],
  ['existing_configuration','既定方案或参照','已指定的轴数、机构、驱动；哪些允许调整'],
];
const numbers=[['workpiece_mass_kg','工件质量 / kg'],['tool_mass_kg','工具与附加设备质量 / kg'],['reach_mm','目标作业距离 / mm'],['cycle_time_s','任务周期 / s']];
const installations=[['','未知 / 待确定'],['fixed_floor','固定地面安装'],['fixed_wall','固定侧装'],['fixed_ceiling','固定倒装'],['rail','导轨'],['mobile','移动平台'],['dual_arm','双臂部署'],['other','其他（在场景中说明）']];
const display=value=>value===null||value===undefined||value===''?'未知':typeof value==='object'?JSON.stringify(value):String(value);
const list=items=>items.length?`<ul>${items.map(item=>`<li>${esc(item)}</li>`).join('')}</ul>`:'<p>暂无记录</p>';
const statusLabels={candidate:'列入比较',conditional:'条件候选',excluded:'明确冲突',insufficient_evidence:'证据不足'};
const checkLabels={meets:'有证据符合',conflicts:'明确冲突',unknown:'未知',not_applicable:'不适用'};
const originLabels={user_confirmed:'用户提供',user_input:'用户提供',material_case:'参考案例',current_calculation:'本次计算',concept_assumption:'概念假设',unknown:'未知'};
const kindLabels={hard_constraint:'硬约束',preference:'偏好',unknown:'未知',task:'任务'};
const dimensions=[['arm_dof','独立自由度'],['axis_layout','机构与轴线'],['elbow','肘部'],['wrist','腕部'],['drive_layout','驱动布置'],['sensing_control','感知与控制'],['deployment','部署']];
export function resultHTML(data){
  const r=data.result;
  const requirements=r.requirements.map(row=>`<tr><td>${esc(row.id)}</td><td>${esc(row.name)}</td><td>${esc(display(row.value))} ${esc(row.unit??'')}</td><td>${esc(kindLabels[row.kind]??row.kind)}</td><td>${esc(originLabels[row.origin]??row.origin)}</td></tr>`).join('');
  const rows=r.candidates.map(c=>`<tr><td>${esc(c.name??c.id)}</td><td>${esc(statusLabels[c.status])}</td><td>${esc(display(c.configuration.arm_dof))}</td><td>${esc(display(c.configuration.axis_layout))}</td><td>${esc(c.fit_reasons.join('；'))}</td><td>${esc(c.tradeoffs.join('；'))}</td></tr>`).join('');
  return card('需求与未知项',`<div class="table-scroll"><table><thead><tr><th>编号</th><th>需求</th><th>已知值</th><th>性质</th><th>来源</th></tr></thead><tbody>${requirements}</tbody></table></div><h3>待补充信息</h3>${list(r.open_questions)}`)+
    card('候选比较',rows?`<p>候选状态用于概念初选，不表示工程验收通过。</p><div class="table-scroll"><table><thead><tr><th>候选</th><th>状态</th><th>自由度</th><th>机构与轴线</th><th>适用理由</th><th>代价与限制</th></tr></thead><tbody>${rows}</tbody></table></div>${r.candidates.map(c=>`<details class="selection-candidate"><summary>${esc(c.name??c.id)} · 构型、约束与依据</summary><dl class="data-list">${dimensions.map(([key,label])=>`<div><dt>${label}</dt><dd>${esc(display(c.configuration[key]))}</dd></div>`).join('')}</dl><h3>约束核对</h3>${list(c.constraint_checks.map(check=>`${check.requirement_id} · ${checkLabels[check.result]}${check.reason?' · '+check.reason:''}`))}<h3>证据与适用范围</h3>${list(c.evidence.map(e=>`${e.source_id} · ${e.locator} · ${e.claim_kind==='inference'?'工程推理':'事实'} / ${e.evidence_type}；支持：${e.supports}；范围：${e.scope}${e.claim_kind==='inference'?'；假设：'+e.assumptions.join('；')+'；待验证：'+e.pending_validations.join('；'):''}`))}<h3>待验证项</h3>${list(c.pending_validations)}</details>`).join('')}`:'<p>当前没有可支持的候选，请先补充上方关键需求。</p>')+
    card('选择结论与下一步',`<p><strong>${esc({shortlist:'形成候选清单',needs_requirements:'需要补充需求',no_supported_candidate:'暂无可支持候选'}[r.outcome])}</strong></p><p class="selection-prose">${esc(r.conclusion)}</p><h3>验证计划</h3>${list(r.validation_plan)}<div class="actions-row"><button class="button primary" id="selectionJSON">下载选择记录 JSON</button></div><details><summary>知识库追溯</summary>${list(r.sources_read)}<p>本次记录时间：${esc(data.execution?.executed_at??'')}。知识库文件哈希随 JSON 导出。</p></details>`);
}
function field([key,label,placeholder]){
  return `<label class="selection-field"><span>${label}</span><textarea name="${key}" rows="3" maxlength="8000" placeholder="${esc(placeholder)}">${esc(state.draft[key]??'')}</textarea></label>`;
}
function html(){
  const health=state.health;
  return card('场景需求',`<p>从任务动作出发比较机构、自由度、肘腕、驱动与部署。空白项保持未知，无需提前准备完整轴表或 DH 参数。</p><p id="selectionAgentState" role="status">${health?health.configured?(health.message??'构型选择 Agent 已配置'):esc(health.message):'正在检查构型选择服务…'}</p><form id="selectionForm"><fieldset ${state.busy?'disabled':''} class="selection-form"><div class="selection-fields">${textFields.slice(0,3).map(field).join('')}</div><div class="form-grid">${numbers.map(([key,label])=>`<label class="selection-field"><span>${label}</span><input name="${key}" type="number" min="${key==='reach_mm'||key==='cycle_time_s'?'0.000001':'0'}" step="any" placeholder="未知可留空" value="${esc(state.draft[key]??'')}"></label>`).join('')}<label class="selection-field"><span>安装与部署</span><select name="installation">${installations.map(([key,label])=>`<option value="${key}" ${state.draft.installation===key?'selected':''}>${label}</option>`).join('')}</select></label><label class="selection-field"><span>坐标与尺寸说明</span><input name="coordinate_frame" type="text" maxlength="8000" placeholder="例如：基座坐标，XYZ 单位 mm" value="${esc(state.draft.coordinate_frame??'')}"></label></div><details open><summary>动作、工程约束与设计偏好</summary><div class="selection-fields">${textFields.slice(3).map(field).join('')}</div></details><div class="actions-row"><button class="button primary" type="submit" ${state.busy||!health?.configured?'disabled':''}>${state.busy?'Agent 正在分析…':'开始构型选择'}</button><button class="button" type="button" id="selectionHealth">重新连接服务</button></div></fieldset></form>${state.busy?'<div class="actions-row"><button class="button" id="selectionCancel">取消分析</button></div>':''}<p id="selectionMessage" role="status">${esc(state.message)}</p>`)+`<div id="selectionResults">${state.data?resultHTML(state.data):''}</div>`;
}
function render(){if(location.hash!=='#configuration')return;const el=$('content');if(el){el.innerHTML=html();bind();}}
let controller;
async function health(){
  if(state.checking)return;
  state.checking=true;
  try{const response=await fetch('/api/configuration-selection/health');if(!response.ok)throw Error();state.health=await response.json();}
  catch{state.health={configured:false,message:'无法连接构型选择服务，请稍后重新连接。'};}
  finally{state.checking=false;render();}
}
function bind(){
  const form=$('selectionForm');if(!form)return;
  if(!state.health&&!state.checking)health();
  const read=()=>{
    const draft={};
    for(const element of form.elements){if(element.name)draft[element.name]=element.value;}
    state.draft=draft;
  };
  form.oninput=()=>{read();state.data=null;state.message='需求已修改，可按当前输入重新分析。';if($('selectionMessage'))$('selectionMessage').textContent=state.message;if($('selectionResults'))$('selectionResults').innerHTML='';};
  $('selectionHealth').onclick=()=>{read();health();};
  form.onsubmit=async event=>{
    event.preventDefault();if(state.busy)return;
    read();const request={...state.draft};
    if(!request.scenario?.trim()&&!request.task?.trim()){$('selectionMessage').textContent='请至少填写应用场景或具体任务。';return;}
    for(const [key] of numbers)request[key]=request[key]?.trim()?Number(request[key]):null;
    controller=new AbortController();state.busy=true;state.data=null;state.message='Agent 正在读取构型知识库并核对需求，完成后在本页显示结果。';render();
    try{
      const response=await fetch('/api/configuration-selection',{method:'POST',headers:{'content-type':'application/json'},body:JSON.stringify(request),signal:controller.signal});
      const data=await response.json();if(!response.ok||!data.ok)throw Error(data.detail??data.error??'分析失败');
      state.data=data;state.message='构型初选已完成，请查看候选条件、来源和待验证项。';
    }catch(error){state.message=controller.signal.aborted?'已取消构型选择。':error.message;}
    finally{state.busy=false;controller=null;render();}
  };
  if($('selectionCancel'))$('selectionCancel').onclick=()=>controller?.abort();
  if($('selectionJSON'))$('selectionJSON').onclick=()=>{
    const url=URL.createObjectURL(new Blob([JSON.stringify(state.data,null,2)],{type:'application/json'}));
    const a=document.createElement('a');a.href=url;a.download='arm-configuration-selection.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
  };
}
export const ConfigurationSelectionPage={html,bind};
