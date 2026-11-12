const $=id=>document.getElementById(id);
const esc=value=>String(value).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const card=(title,body)=>`<section class="card"><div class="card-head"><h2>${title}</h2></div><div class="card-body">${body}</div></section>`;
const fixed2=value=>Number(value).toFixed(2);
const zero=[0,0,0];
const state={model:null,edits:{},busy:false,message:'正在读取选型表模型…',spec:null,applied:null};
const toDeg=value=>value*180/Math.PI;
const toRad=value=>value*Math.PI/180;
const mm=values=>values.map(value=>value*1000);
const meters=values=>values.map(value=>value/1000);
const skillLabels={statics:'静力学校核',workspace:'可达空间',motion:'运动学与动力学'};
const examples=[
  ['worksheet','选型表七轴单臂'],
  ['design-right-arm','双臂设计方案 CAD 右臂'],
  ['seven-axis-sim','七轴仿真输入'],
  ['franka-fr3','Franka FR3 法兰质量预设'],
];
let hooks;

function loadModel(model){
  hooks.clear();
  state.model=model;
  state.edits={};
  state.spec=null;
  state.applied=null;
}
async function loadExample(key=examples[0][0]){
  const response=await fetch('/model-examples/'+key+'.json');
  if(!response.ok)throw Error('示例模型无法载入');
  loadModel(await response.json());
  state.example=key;
  state.message='已载入示例模型。';
}
const value=(key,original)=>state.edits[key]??(Array.isArray(original)?original.map(x=>Number.isFinite(x)?Number(x.toFixed(6)):x).join(', '):String(original));
const input=(key,original,label,type='text')=>
  `<input type="${type}" ${type==='number'?'step="any"':''} data-module-field="${key}" aria-label="${esc(label)}" value="${esc(value(key,original))}" ${state.busy?'disabled':''}>`;
const field=(key,original,label,type)=>`<label class="module-field"><span>${label}</span>${input(key,original,label,type)}</label>`;

function bodyProperties(joint){
  if(!joint.components)return {mass:'',com:zero};
  const mass=joint.components.reduce((sum,c)=>sum+c.mass_kg,0);
  const com=mass?zero.map((_,axis)=>joint.components.reduce((sum,c)=>sum+c.com_m[axis]*c.mass_kg,0)/mass):zero;
  return {mass,com};
}
function specsHTML(){
  const model=state.model,disabled=state.busy?'disabled':'';
  const options=`<div class="actions-row"><label for="moduleExample">示例模型</label>
    <select id="moduleExample" ${disabled}><option value="">选择示例</option>${examples.map(([key,label])=>`<option value="${key}" ${state.example===key?'selected':''}>${label}</option>`).join('')}</select>
    <label for="moduleFile">导入模型 JSON</label><input id="moduleFile" type="file" accept=".json,application/json" ${disabled}></div>`;
  if(!model)return card('公共模型',options+`<p id="moduleStatus" role="status">${esc(state.message)}</p>`);
  const rows=model.joints.map((joint,i)=>{
    const key='j'+i,{mass,com}=bodyProperties(joint),rotary=joint.type==='revolute';
    const range=rotary?joint.limits.map(toDeg):joint.limits?.map(x=>x*1000);
    return `<tr><td>${esc(joint.name)}</td><td>${{revolute:'旋转',prismatic:'移动',fixed:'固定'}[joint.type]}</td>
      <td>${input(key+'origin',mm(joint.origin.xyz_m),joint.name+' 原点 XYZ / mm')}</td>
      <td>${input(key+'rpy',joint.origin.rpy_rad.map(toDeg),joint.name+' 安装 RPY / °')}</td>
      <td>${joint.axis?input(key+'axis',joint.axis,joint.name+' 轴向 XYZ'):'—'}</td>
      <td>${range?input(key+'range',range,joint.name+' 限位 '+(rotary?'°':'mm')):'—'}</td>
      <td>${input(key+'mass',mass,joint.name+' 运动体质量 / kg','number')}</td>
      <td>${input(key+'com',mm(com),joint.name+' 质心 XYZ / mm')}</td></tr>`;
  }).join('');
  return card('三个 Skill 共用的模型',`${options}<div class="form-grid module-form">
    ${field('name',model.name,'模型名称')}
    ${field('tip',mm(model.tip.xyz_m),'末端 XYZ / mm')}
    ${field('tipRPY',model.tip.rpy_rad.map(toDeg),'末端 RPY / °')}
    ${field('baseXYZ',mm(model.base_in_world?.xyz_m??zero),'基座 XYZ / mm')}
    ${field('gravity',model.gravity_world_m_s2??[0,0,-9.8],'世界重力 XYZ / m/s²')}
    ${field('mount',(model.base_in_world?.rpy_rad??zero).map(toDeg),'安装 RPY / °')}
    ${field('payload',model.payload?.mass_kg??0,'末端负载 / kg','number')}
    ${field('payloadCOM',mm(model.payload?.com_in_tip_m??zero),'负载质心 XYZ / mm')}
    </div>
    <p>向量用逗号分隔。原点相对上一级关节，质心相对该关节运动后的坐标系。旋转限位用度，移动限位用毫米。</p>
    <div class="table-scroll module-joints"><table><thead><tr><th>关节</th><th>类型</th><th>原点 XYZ / mm</th><th>安装 RPY / °</th><th>轴向 XYZ</th><th>限位下界、上界</th><th>运动体质量 / kg</th><th>质心 XYZ / mm</th></tr></thead><tbody>${rows}</tbody></table></div>
    <details><summary>概念计算假设</summary><p>缺少部件惯量时按半径 40 mm 的均匀实心球估算，负载采用 50 mm。修改某行质量或质心后，该行按一个等效运动体计算。可导入带完整部件属性的模型。</p>
    <p>规格表使用中位往返轨迹，旋转关节默认 1 rad/s、2 rad/s²，移动关节默认 0.2 m/s、0.5 m/s²。固定安装、关节坐标系旋转及部件属性从模型保留。缺省重力为世界系 Z 向下 9.8 m/s²。</p></details>
    <div class="actions-row">
      <button class="button" id="moduleSave" ${disabled}>下载模型 JSON</button>
      <button class="button" id="moduleApply" ${disabled}>应用到三个 Skill</button>
      <button class="button primary" id="moduleRun" ${disabled}>运行三个 Skill 并生成规格表</button>
    </div><p id="moduleStatus" role="status">${esc(state.message)}</p>`);
}
function numeric(key,label){
  const raw=state.edits[key].trim(),number=Number(raw);
  if(!raw||!Number.isFinite(number)||number<0)throw Error(label+'须填写非负数字');
  return number;
}
function vector(key,length,label){
  const values=state.edits[key].trim().split(/[,，\s]+/);
  if(values.length!==length||values.some(x=>!x||!Number.isFinite(Number(x))))throw Error(label+'须填写 '+length+' 个数字');
  return values.map(Number);
}
const edited=key=>Object.hasOwn(state.edits,key);
function buildModel(){
  const model=structuredClone(state.model);
  if(edited('name'))model.name=state.edits.name.trim();
  if(edited('tip'))model.tip.xyz_m=meters(vector('tip',3,'末端 XYZ'));
  if(edited('tipRPY'))model.tip.rpy_rad=vector('tipRPY',3,'末端 RPY').map(toRad);
  if(edited('gravity'))model.gravity_world_m_s2=vector('gravity',3,'世界重力');
  if(edited('mount')||edited('baseXYZ'))model.base_in_world={
    xyz_m:edited('baseXYZ')?meters(vector('baseXYZ',3,'基座 XYZ')):model.base_in_world?.xyz_m??zero,
    rpy_rad:edited('mount')?vector('mount',3,'安装 RPY').map(toRad):model.base_in_world?.rpy_rad??zero,
  };
  if(edited('payload')||edited('payloadCOM')){
    model.payload??={mass_kg:0,com_in_tip_m:zero,provenance:{source:'网页输入',assumptions:[]}};
    if(edited('payload'))model.payload.mass_kg=numeric('payload','负载质量');
    if(edited('payloadCOM'))model.payload.com_in_tip_m=meters(vector('payloadCOM',3,'负载质心'));
    model.payload.provenance={source:'网页输入',assumptions:[]};
  }
  for(const [i,joint] of model.joints.entries()){
    const key='j'+i;
    if(edited(key+'origin'))joint.origin.xyz_m=meters(vector(key+'origin',3,joint.name+' 原点'));
    if(edited(key+'rpy'))joint.origin.rpy_rad=vector(key+'rpy',3,joint.name+' 安装 RPY').map(toRad);
    if(edited(key+'axis'))joint.axis=vector(key+'axis',3,joint.name+' 轴向');
    if(edited(key+'range')){
      const limits=vector(key+'range',2,joint.name+' 限位');
      joint.limits=joint.type==='revolute'?limits.map(toRad):meters(limits);
    }
    if(edited(key+'mass')||edited(key+'com')){
      const body=bodyProperties(joint);
      const mass=edited(key+'mass')?numeric(key+'mass',joint.name+' 质量'):body.mass;
      if(mass==='')throw Error('请填写 '+joint.name+' 的运动体质量');
      const com=edited(key+'com')?meters(vector(key+'com',3,joint.name+' 质心')):body.com;
      joint.components=mass===0?[]:[{name:'等效运动体',mass_kg:mass,com_m:com,provenance:{source:'网页输入',assumptions:[]}}];
    }
  }
  return model;
}
function specificationsHTML(){
  const progress=state.applied?`<p>${Object.entries(skillLabels).map(([key,label])=>label+' '+(state.applied.results[key]?'已完成':'待计算')).join(' · ')}</p>`:'';
  if(!state.spec)return card('关节模组概念规格',`<p>完成三个 Skill 的同模型计算后，这里会显示规格表和导出按钮。</p>${progress}
    <div class="actions-row"><button class="button" data-go="specs">打开规格与模型</button>
    <button class="button primary" id="moduleRun" ${state.busy?'disabled':''}>运行三个 Skill 并生成规格表</button></div>
    <p id="moduleStatus" role="status">${esc(state.message)}</p>`);
  const spec=state.spec;
  const rows=spec.joints.map(j=>`<tr><td>${esc(j.name)}</td><td>${j.range.map(fixed2).join(' ～ ')} ${j.range_unit}</td>
    <td>${fixed2(j.holding_effort_abs)} ${j.effort_unit}</td><td>${fixed2(j.dynamic_peak_abs)} ${j.effort_unit}</td>
    <td>${fixed2(j.dynamic_rms)} ${j.effort_unit}</td><td>${fixed2(j.checked_effort_abs)} ${j.effort_unit}</td>
    <td>${fixed2(j.speed_peak_abs)} ${j.speed_unit}</td><td>${fixed2(j.effort_peak_90pct_window.duration_s)}</td><td>${fixed2(j.axial_force_peak_abs_N)}</td>
    <td>${fixed2(j.radial_force_peak_N)}</td><td>${fixed2(j.bending_moment_peak_Nm)}</td></tr>`).join('');
  const bounds=spec.workspace.bounds_base_m;
  const groupRows=spec.groups.map(g=>`<tr><td>${g.joints.join(' / ')}</td><td>${fixed2(g.holding_effort_abs)}</td><td>${fixed2(g.dynamic_peak_abs)}</td><td>${fixed2(g.checked_effort_abs)}</td><td>${fixed2(g.speed_peak_abs)}</td></tr>`).join('');
  return card('关节模组概念规格',`<p><strong>${esc(spec.model_name)}</strong> · ${fixed2(spec.cycle.duration_s)} s 轨迹 · 可达空间分析 ${spec.workspace.sample_count} 点</p>
    <p>${bounds?'基座系采样范围 / m · '+['X','Y','Z'].map((axis,i)=>axis+' '+fixed2(bounds.min[i])+' ～ '+fixed2(bounds.max[i])).join(' · '):'本次工作空间分析未获得可达位置。'}</p>
    <div class="table-scroll"><table><thead><tr><th>关节</th><th>设计行程</th><th>静态保持</th><th>动态峰值</th><th>轨迹 RMS</th><th>已查最大需求</th><th>轨迹最高速度</th><th>峰值附近 / s</th><th>轴向力 / N</th><th>径向力 / N</th><th>弯矩 / N·m</th></tr></thead><tbody>${rows}</tbody></table></div>
    <p>峰值附近时长统计本次峰值所在的连续区间，阈值为峰值的 90%。${spec.cycle.constraint_status==='verified'?'':' 轨迹约束状态：'+esc(spec.cycle.constraint_status)+'。'}</p>
    ${groupRows?`<h3>七轴分组参考</h3><div class="table-scroll"><table><thead><tr><th>组</th><th>静态保持 / Nm</th><th>动态峰值 / Nm</th><th>已查最大需求 / Nm</th><th>轨迹最高转速 / rpm</th></tr></thead><tbody>${groupRows}</tbody></table></div>`:''}
    <div class="actions-row"><button class="button primary" id="moduleCSV">下载规格表 CSV</button>
    <button class="button" id="moduleJSON">下载规格表 JSON</button><button class="button" data-go="specs">修改模型</button></div>
    <details><summary>工况假设与结果来源</summary><ul>${spec.assumptions.map(x=>'<li>'+esc(x)+'</li>').join('')}</ul>
    <ul>${Object.entries(spec.sources).map(([key,source])=>'<li>'+skillLabels[key]+' · '+esc(source.engine_version)+' · '+esc(source.input_sha256)+'</li>').join('')}</ul></details>`);
}
function csv(spec){
  const encode=value=>'"'+String(value).replaceAll('"','""')+'"';
  const header=['关节','行程下界','行程上界','行程单位','运动体质量 kg','静态保持','动态峰值','轨迹 RMS','已查最大需求','力矩或推力单位','轨迹最高速度','速度单位','峰值附近 s','轴向力 N','径向力 N','弯矩 Nm'];
  const rows=spec.joints.map(j=>[
    /^[=+\-@\t\r\n]/.test(j.name)?"'"+j.name:j.name,...j.range,j.range_unit,j.body_mass_kg,j.holding_effort_abs,j.dynamic_peak_abs,
    j.dynamic_rms,j.checked_effort_abs,j.effort_unit,j.speed_peak_abs,j.speed_unit,j.effort_peak_90pct_window.duration_s,j.axial_force_peak_abs_N,j.radial_force_peak_N,j.bending_moment_peak_Nm,
  ]);
  return '\ufeff'+[header,...rows].map(row=>row.map(encode).join(',')).join('\r\n')+'\r\n';
}
async function action(mode,load){
  if(state.busy)return;
  state.busy=true;
  state.message=mode==='run'?'正在运行静力学、可达空间及运动学与动力学 Skill…':'正在验证模型…';
  if(mode==='run'){state.spec=null;state.applied=null;}
  if(mode==='run'&&!['#specs','#sizing'].includes(location.hash))hooks.go('sizing');
  else hooks.render();
  let showSizing=false;
  try{
    if(load)await load();
    else if(!state.model)await loadExample();
    if(mode==='run')state.message='正在运行静力学、可达空间及运动学与动力学 Skill…';
    const response=await fetch('/api/module-specs',{method:'POST',headers:{'content-type':'application/json'},
      body:JSON.stringify({design_model:buildModel(),prepare_only:mode!=='run'})});
    const data=await response.json();
    if(!response.ok||!data.ok)throw Error((skillLabels[data.stage]?skillLabels[data.stage]+' · ':'')+(data.detail??data.error));
    if(mode==='download'){
      hooks.download('arm-design-model.json',JSON.stringify(data.requests.motion.design_model,null,2));
      state.message='模型 JSON 已生成。';
    }else{
      hooks.apply(data);
      state.applied={model:data.requests.motion.design_model,hash:data.modelHash,results:data.results??{}};
      state.spec=data.specifications??null;
      state.message=state.spec?'规格表已生成。':'已填入三个 Skill，可在各页单独运行。';
      showSizing=!!state.spec;
    }
  }catch(error){state.message=error.message;}
  finally{state.busy=false;if(showSizing)hooks.go('sizing');else hooks.render();}
}
function bind(nextHooks){
  hooks=nextHooks;
  if(!state.model&&!state.busy&&state.message==='正在读取选型表模型…'){
    queueMicrotask(()=>{if(!state.model&&!state.busy)action('apply');});
  }
  if($('moduleExample'))$('moduleExample').onchange=event=>{
    const key=event.target.value;
    if(examples.some(([id])=>id===key))return action('apply',()=>loadExample(key));
  };
  if($('moduleFile'))$('moduleFile').onchange=async event=>{
    const file=event.target.files[0];
    if(!file)return;
    return action('apply',async()=>{
      if(file.size>2*1024*1024)throw Error('模型文件不可超过 2 MiB');
      const raw=JSON.parse(await file.text());
      const model=raw.design_model??raw;
      if(model?.schema_name!=='arm-design-model'||!Array.isArray(model.joints)||!model.joints.length||model.joints.length>64||
        !Array.isArray(model.tip?.xyz_m)||!model.tip.xyz_m.every(Number.isFinite)||!Array.isArray(model.tip.rpy_rad)||
        (model.base_in_world&&!Array.isArray(model.base_in_world.rpy_rad))||
        (model.payload&&!Array.isArray(model.payload.com_in_tip_m))||model.joints.some(joint=>!Array.isArray(joint?.origin?.xyz_m)||!Array.isArray(joint.origin.rpy_rad)||
        !['revolute','prismatic','fixed'].includes(joint.type)||(joint.type!=='fixed'&&(!Array.isArray(joint.axis)||!Array.isArray(joint.limits)))||
        (joint.components!==undefined&&(!Array.isArray(joint.components)||joint.components.some(c=>!Array.isArray(c?.com_m)))))){
        throw Error('请选择有效的 arm-design-model JSON');
      }
      loadModel(model);
      state.example='';
    });
  };
  document.querySelectorAll('[data-module-field]').forEach(control=>control.oninput=()=>{
    if(state.applied)hooks.clear();
    state.edits[control.dataset.moduleField]=control.value;
    state.spec=null;
    state.applied=null;
    state.message='模型已修改，请重新应用或计算。';
    $('moduleStatus').textContent=state.message;
  });
  if($('moduleSave'))$('moduleSave').onclick=()=>action('download');
  if($('moduleApply'))$('moduleApply').onclick=()=>action('apply');
  if($('moduleRun'))$('moduleRun').onclick=()=>action('run');
  if($('moduleCSV'))$('moduleCSV').onclick=()=>hooks.download('arm-module-requirements.csv',csv(state.spec),'text/csv;charset=utf-8');
  if($('moduleJSON'))$('moduleJSON').onclick=()=>hooks.download('arm-module-requirements.json',JSON.stringify(state.spec,null,2));
}
function record(skill,result){
  const applied=state.applied;
  if(!applied)return;
  if(result?.design_model_sha256===applied.hash)applied.results[skill]=result;
  else delete applied.results[skill];
  state.spec=null;
  if(Object.keys(applied.results).length===3){
    try{
      state.spec=combineResults(applied.model,applied.hash,applied.results);
      state.message='规格表已生成。';
    }catch(error){state.message=error.message;}
  }else state.message=result&&result.design_model_sha256!==applied.hash
    ?skillLabels[skill]+'结果使用了其他模型，请重新应用公共模型。'
    :'等待三个 Skill 完成同模型计算。';
}
function provenanceHTML(){
  const model=state.model;
  if(!model)return card('公共模型来源','<p>请在「规格与模型」选择或导入模型。</p>');
  const provenance=[model.provenance,model.payload?.provenance,...model.joints.flatMap(joint=>(joint.components??[]).map(component=>component.provenance))].filter(Boolean);
  const sources=[...new Set(provenance.map(item=>item.source))];
  const assumptions=[...new Set(provenance.flatMap(item=>item.assumptions??[]))];
  return card('公共模型来源',`<p>${esc(value('name',model.name))}</p>
    <ul>${[...sources,...assumptions].map(text=>'<li>'+esc(text)+'</li>').join('')}</ul>
    <button class="button" data-go="specs">打开规格与模型</button>`);
}
export const ModuleDesignPage={modelName:()=>state.model?value('name',state.model.name):null,specsHTML,specificationsHTML,provenanceHTML,bind,record,run:()=>action('run'),save:()=>action('download'),
  exportSpecifications:kind=>{
    if(!state.spec)return false;
    if(kind==='csv')hooks.download('arm-module-requirements.csv',csv(state.spec),'text/csv;charset=utf-8');
    else hooks.download('arm-module-requirements.json',JSON.stringify(state.spec,null,2));
    return true;
  }};
import {combineResults} from './module-specifications.mjs';

