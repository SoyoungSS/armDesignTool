const $=id=>document.getElementById(id);
const esc=value=>String(value).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const card=(title,body)=>`<section class="card"><div class="card-head"><h2>${title}</h2></div><div class="card-body">${body}</div></section>`;
const fixed2=value=>Number(value).toFixed(2);
const zero=[0,0,0];
const state={model:null,edits:{},busy:false,message:'正在读取选型表模型…',spec:null,applied:null,analysisActive:false};
const toDeg=value=>value*180/Math.PI;
const toRad=value=>value*Math.PI/180;
const mm=values=>values.map(value=>value*1000);
const meters=values=>values.map(value=>value/1000);
const skillLabels={statics:'静力学校核',workspace:'可达空间',motion:'运动学与动力学'};
const examples=[
  ['worksheet','选型表七轴单臂'],
  ['fr3','Franka FR3'],
  ['design-right-arm','双臂设计方案 CAD 右臂'],
  ['seven-axis-sim','七轴仿真输入'],
  ['franka-fr3','Franka FR3 法兰质量预设'],
];
const simulationFormats={urdf:'URDF',ros1:'ROS1 / RViz',mujoco:'MuJoCo MJCF',isaac:'Isaac Sim USD'};
const simulationStatus={generated:'已生成',static_checked:'静态检查通过',runtime_validated:'运行时检查通过',pending:'待补齐或验证',failed:'失败'};
const simulation={busy:false,message:'',robotName:'robot_arm',targets:Object.keys(simulationFormats),limits:new Map(),density:2000};
let hooks;

function loadModel(model){
  hooks.clear();
  simulation.limits.clear();
  simulation.density=2000;
  state.model=model;
  state.baseRows=(model.base_components??[]).slice();
  state.edits={};
  state.spec=null;
  state.applied=null;
  state.analysisActive=false;
}
async function loadExample(key=examples[0][0]){
  const response=await fetch('/model-examples/'+key+'.json');
  if(!response.ok)throw Error('示例模型无法载入');
  loadModel(await response.json());
  state.example=key;
  state.message='已载入示例模型。';
}
const value=(key,original)=>state.edits[key]??(Array.isArray(original)?original.map(x=>Number.isFinite(x)?Number(x.toFixed(6)):x).join(', '):String(original));
const input=(key,original,label,type='text',descriptionId)=>
  `<input type="${type}" ${type==='number'?'step="any"':''} data-module-field="${key}" aria-label="${esc(label)}" ${descriptionId?`aria-describedby="${descriptionId}"`:''} value="${esc(value(key,original))}" ${state.busy?'disabled':''}>`;
const field=(key,original,label,type,help)=>{
  const descriptionId=help?'help-'+key:undefined;
  return `<label class="module-field"><span>${label}</span>${input(key,original,label,type,descriptionId)}${help?`<small class="module-help" id="${descriptionId}">${help}</small>`:''}</label>`;
};
const jointColumns=[
  ['origin','原点 XYZ / mm','关节零位原点相对上游刚体原点的偏移，用上游刚体的轴表达。首关节相对基座填写。'],
  ['rpy','安装 RPY / °','关节零位坐标系相对上游刚体的固定旋转，依次填 Roll、Pitch、Yaw。'],
  ['axis','轴向 XYZ','在安装旋转后的关节局部系中填写非零方向向量。例如 0, 1, 0 表示局部 +Y；旋转正向遵循右手定则。'],
  ['range','限位下界、上界','从几何零位量起的机械行程，先填下界、再填上界。旋转关节用度，移动关节用毫米。'],
  ['mass','运动体质量 / kg','填写随本关节输出端运动的部件总质量。模组固定侧归上游刚体，下游部件逐级填写，避免重复计重。'],
  ['com','质心 XYZ / mm','填写本行部件的合成质心，从本关节运动后的输出刚体原点量起，沿该刚体的局部轴表达。'],
];

function bodyProperties(joint){
  if(!joint.components)return {mass:'',com:zero};
  const mass=joint.components.reduce((sum,c)=>sum+c.mass_kg,0);
  const com=mass?zero.map((_,axis)=>joint.components.reduce((sum,c)=>sum+c.com_m[axis]*c.mass_kg,0)/mass):zero;
  return {mass,com};
}
const inertiaFields=[['Ixx',0,0],['Iyy',1,1],['Izz',2,2],['Ixy',0,1],['Ixz',0,2],['Iyz',1,2]];
const coordinates=['X','Y','Z'];
function basePropertiesHTML(disabled){
  const rows=state.baseRows.map((component,index)=>{
    const key='baseComponent'+index+'-';
    if(edited(key+'removed'))return '';
    return `<fieldset class="base-component"><legend>基座部件 ${index+1}</legend>
      <div class="form-grid module-form">
        ${field(key+'name',component.name??'','部件名称','text','基座内唯一的标识，例如底板或安装支架。')}
        ${field(key+'mass',component.mass_kg??'','质量 / kg','number','固定部件的质量，例如 630 g 填 0.63。')}
        ${coordinates.map((axis,i)=>field(key+'com'+i,component.com_m?component.com_m[i]*1000:'','质心 '+axis+' / mm','number','从基座原点沿 '+axis+' 轴到该部件质心的有符号距离。')).join('')}
      </div>
      <div class="form-grid module-form">
        ${inertiaFields.map(([label,row,column])=>field(key+label,component.inertia_kg_m2?.[row]?.[column]??'',label+' / kg·m²','number',row===column?'绕穿过质心、平行于基座 '+coordinates[row]+' 轴的转动惯量。':coordinates[row]+coordinates[column]+' 非对角分量，可为正、负或零。')).join('')}
      </div>
      <button class="button" type="button" data-remove-base="${index}" ${disabled}>移除此部件</button>
    </fieldset>`;
  }).join('');
  return `<p><strong>基座约束：固定在世界坐标系</strong>。基座不参与运动，位置和安装朝向由上方基座 XYZ、安装 RPY 设置。无需填写基座质量或为其设置密度即可按固定基座导出。</p>
    <details><summary>高级：基座质量属性（可选，默认无需填写）</summary>
    <p>仅在需要保留基座实际质量属性时填写；留空时向仿真 Skill 声明固定基座并省略其质量属性，仍保留支座几何，不使用密度补齐基座。填写属性也不会解除固定约束，其质量不计入活动关节载荷。质心位置从基座原点量起，惯量取绕部件质心、沿基座轴表达的值。惯量六项可一起留空；填写时请核对 CAD 报告的参考点、单位和惯性积符号。</p>
    ${rows}<button class="button" type="button" id="addBaseComponent" ${disabled}>添加基座部件</button>
    </details>`;
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
      <td>${input(key+'origin',mm(joint.origin.xyz_m),joint.name+' 原点 XYZ / mm','text','joint-help-origin')}</td>
      <td>${input(key+'rpy',joint.origin.rpy_rad.map(toDeg),joint.name+' 安装 RPY / °','text','joint-help-rpy')}</td>
      <td>${joint.axis?input(key+'axis',joint.axis,joint.name+' 轴向 XYZ','text','joint-help-axis'):'—'}</td>
      <td>${range?input(key+'range',range,joint.name+' 限位 '+(rotary?'°':'mm'),'text','joint-help-range'):'—'}</td>
      <td>${input(key+'mass',mass,joint.name+' 运动体质量 / kg','number','joint-help-mass')}</td>
      <td>${input(key+'com',mm(com),joint.name+' 质心 XYZ / mm','text','joint-help-com')}</td></tr>`;
  }).join('');
  return card('公共机械臂模型',`${options}<div class="form-grid module-form">
    ${field('name',model.name,'模型名称','text','用于识别当前模型，也会写入分析结果和模型 JSON。')}
    ${field('tip',mm(model.tip.xyz_m),'末端 XYZ / mm','text','从最后一个输出刚体原点到末端工具坐标系原点的偏移，沿该刚体的局部轴填写。')}
    ${field('tipRPY',model.tip.rpy_rad.map(toDeg),'末端 RPY / °','text','工具坐标系相对最后一个输出刚体的朝向，决定 TCP 姿态和负载质心坐标轴。')}
    ${field('baseXYZ',mm(model.base_in_world?.xyz_m??zero),'基座 XYZ / mm','text','基座原点相对于世界原点的位置，沿世界 X、Y、Z 轴填写。')}
    ${field('gravity',model.gravity_world_m_s2??[0,0,-9.8],'世界重力 XYZ / m/s²','text','沿世界轴填写重力加速度。世界 Z 向上时可填 0, 0, -9.81，侧装通过安装 RPY 表达。')}
    ${field('mount',(model.base_in_world?.rpy_rad??zero).map(toDeg),'安装 RPY / °','text','基座相对世界的朝向。例如 0, 90, 0 使基座 +Z 指向世界 +X。')}
    ${field('payload',model.payload?.mass_kg??0,'末端负载 / kg','number','填写附加在工具端的质量。已计入连杆的部件无需重复填写；空载填 0。')}
    ${field('payloadCOM',mm(model.payload?.com_in_tip_m??zero),'负载质心 XYZ / mm','text','从工具坐标系原点到附加负载质心的偏移，沿工具坐标轴填写。质心在工具原点时填 0, 0, 0。')}
    </div>
    <p>向量按 XYZ 顺序、用逗号分隔。所有 RPY 按 Roll、Pitch、Yaw 的顺序输入，旋转矩阵为 Rz(yaw) · Ry(pitch) · Rx(roll)。</p>
    <details open><summary>关节表填写说明</summary>
      <dl class="joint-field-help">${jointColumns.map(([key,label,help])=>`<div><dt>${label}</dt><dd id="joint-help-${key}">${help}</dd></div>`).join('')}</dl>
    </details>
    <div class="table-scroll module-joints"><table><thead><tr><th>关节</th><th>类型</th>${jointColumns.map(([,label])=>`<th>${label}</th>`).join('')}</tr></thead><tbody>${rows}</tbody></table></div>
    ${basePropertiesHTML(disabled)}
    <details><summary>概念计算假设</summary><p>缺少部件惯量时按半径 40 mm 的均匀实心球估算，负载采用 50 mm。修改某行质量或质心后，该行按一个等效运动体计算。可导入带完整部件属性的模型。</p>
    <p>规格表使用中位往返轨迹，旋转关节默认 1 rad/s、2 rad/s²，移动关节默认 0.2 m/s、0.5 m/s²。固定安装、关节坐标系旋转及部件属性从模型保留。缺省重力为世界系 Z 向下 9.8 m/s²。</p></details>
    <div class="actions-row">
      <button class="button" id="moduleSave" ${disabled}>下载模型 JSON</button>
      <button class="button primary" id="moduleApply" ${disabled}>应用到所有 Skill</button>
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
  if(state.baseRows.length||model.base_components){
    model.base_components=state.baseRows.flatMap((original,index)=>{
      const key='baseComponent'+index+'-';
      if(edited(key+'removed'))return [];
      const name=value(key+'name',original.name??'').trim();
      const mass=value(key+'mass',original.mass_kg??'').trim();
      const com=coordinates.map((_,i)=>value(key+'com'+i,original.com_m?original.com_m[i]*1000:'').trim());
      const inertia=inertiaFields.map(([label,row,column])=>value(key+label,original.inertia_kg_m2?.[row]?.[column]??'').trim());
      if(![name,mass,...com,...inertia].some(Boolean))return [];
      if(!Object.keys(state.edits).some(field=>field.startsWith(key)))return [original];
      if(!mass||!Number.isFinite(Number(mass))||Number(mass)<=0||com.some(part=>!part||!Number.isFinite(Number(part)))){
        throw Error(`基座部件 ${index+1} 须填写正质量和三个质心坐标`);
      }
      const component={name:name||'base_component_'+(index+1),mass_kg:Number(mass),
        com_m:com.map(part=>Number(part)/1000),provenance:{source:'网页输入',assumptions:[]}};
      if(inertia.some(Boolean)){
        if(inertia.some(part=>!part||!Number.isFinite(Number(part))))throw Error(`基座部件 ${index+1} 须填写六项惯量分量`);
        const [xx,yy,zz,xy,xz,yz]=inertia.map(Number);
        component.inertia_kg_m2=[[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]];
      }
      return [component];
    });
  }
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
function specificationsHTML(inline=false){
  const progress=state.applied?`<p>${Object.entries(skillLabels).map(([key,label])=>label+' '+(state.applied.results[key]?'已完成':'待计算')).join(' · ')}</p>`:'';
  if(!state.spec)return card('关节模组概念规格',`<p>完成三个 Skill 的同模型计算后，这里会显示规格表和导出按钮。</p>${progress}
    <div class="actions-row"><button class="button" data-go="specs">打开规格与模型</button><button class="button" data-go="motion">返回动力学分析</button>
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
    <h3>关节模组规格指导</h3><p>以静态保持与动态峰值的较大值作为已查输出端最大需求，以轨迹 RMS 评估连续转矩需求，以最高速度确定输出转速；同时核对行程、轴向力、径向力及弯矩。选用减速器和电机时结合速比、效率、安全系数与热工况换算，不能将采样结果直接当作产品额定值。</p>
    <p>峰值附近时长统计本次峰值所在的连续区间，阈值为峰值的 90%。${spec.cycle.constraint_status==='verified'?'':' 轨迹约束状态：'+esc(spec.cycle.constraint_status)+'。'}</p>
    ${groupRows?`<h3>七轴分组参考</h3><div class="table-scroll"><table><thead><tr><th>组</th><th>静态保持 / Nm</th><th>动态峰值 / Nm</th><th>已查最大需求 / Nm</th><th>轨迹最高转速 / rpm</th></tr></thead><tbody>${groupRows}</tbody></table></div>`:''}
    <div class="actions-row"><button class="button primary" id="moduleCSV">下载规格表 CSV</button>
    <button class="button" id="moduleJSON">下载规格表 JSON</button>${inline?'':'<button class="button" data-go="specs">修改模型</button><button class="button" data-go="motion">返回动力学分析</button>'}</div>
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
async function action(mode,load,motionRequest){
  if(state.busy)return;
  const inline=!!motionRequest;
  if(inline)state.analysisActive=true;
  state.busy=true;
  state.message=mode==='run'?'正在运行静力学、可达空间及运动学与动力学 Skill…':'正在验证模型…';
  if(mode==='run'){state.spec=null;state.applied=null;}
  if(mode==='run'&&!inline&&!['#specs','#sizing'].includes(location.hash))hooks.go('sizing');
  else hooks.render();
  let showSizing=false;
  try{
    if(load)await load();
    else if(!state.model)await loadExample();
    if(mode==='run')state.message='正在运行静力学、可达空间及运动学与动力学 Skill…';
    const response=await fetch('/api/module-specs',{method:'POST',headers:{'content-type':'application/json'},
      body:JSON.stringify({design_model:buildModel(),prepare_only:mode!=='run',...(motionRequest?{motion_request:motionRequest}:{})})});
    const data=await response.json();
    if(!response.ok||!data.ok)throw Error((skillLabels[data.stage]?skillLabels[data.stage]+' · ':'')+(data.detail??data.error));
    if(mode==='download'){
      hooks.download('arm-design-model.json',JSON.stringify(data.requests.motion.design_model,null,2));
      state.message='模型 JSON 已生成。';
    }else{
      hooks.apply(data);
      state.applied={model:data.requests.motion.design_model,hash:data.modelHash,results:data.results??{}};
      state.spec=data.specifications??null;
      state.message=state.spec?'规格表已生成。':'公共模型已应用到所有 Skill，可在对应页面继续操作。';
      showSizing=!!state.spec&&!inline;
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
      if(model?.schema_name!=='arm-design-model'||(model.base_components!==undefined&&(!Array.isArray(model.base_components)||model.base_components.some(component=>!component||typeof component!=='object'||Array.isArray(component))))||!Array.isArray(model.joints)||!model.joints.length||model.joints.length>64||
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
  const modelEdited=()=>{
    if(state.applied)hooks.clear();
    state.spec=null;
    state.applied=null;
    state.message='模型已修改，请重新应用或计算。';
    $('moduleStatus').textContent=state.message;
  };
  document.querySelectorAll('[data-module-field]').forEach(control=>control.oninput=()=>{
    state.edits[control.dataset.moduleField]=control.value;
    modelEdited();
  });
  if($('addBaseComponent'))$('addBaseComponent').onclick=()=>{
    state.baseRows.push({});
    hooks.render();
  };
  document.querySelectorAll('[data-remove-base]').forEach(button=>button.onclick=()=>{
    state.edits['baseComponent'+button.dataset.removeBase+'-removed']=true;
    modelEdited();
    hooks.render();
  });
  if($('simulationExport'))bindSimulation($('simulationExport'));
  if($('moduleSave'))$('moduleSave').onclick=()=>action('download');
  if($('moduleApply'))$('moduleApply').onclick=()=>action('apply');
  if($('moduleRun'))$('moduleRun').onclick=()=>action('run');
  bindSpecifications();
}
function bindSpecifications(){
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
function simulationHTML(){
  if(!state.model)return card('导出仿真模型','<p>请先在「规格与模型」选择或导入模型。</p><button class="button" data-go="specs">打开规格与模型</button>');
  const disabled=simulation.busy||state.busy?'disabled':'';
  const active=state.model.joints.filter(joint=>joint.type!=='fixed');
  const rows=active.map(({id,name,type,simulation_limits},index)=>{
    const limits=simulation.limits.get(id);
    const limitField=(field,label,unit)=>{
      const fromModel=simulation_limits?.[field];
      return `<td><input name="${field}-${index}" type="number" step="any" min="0"
        ${fromModel===undefined?'required':'readonly'} value="${esc(fromModel??limits?.[field]??'')}" aria-label="${esc(name)} ${label}"> ${unit}</td>`;
    };
    return `<tr><td>${esc(name)}</td>
      ${limitField('effort','输出上限',type==='revolute'?'N·m':'N')}
      ${limitField('velocity','速度上限',type==='revolute'?'rad/s':'m/s')}</tr>`;
  }).join('');
  return card('导出仿真模型',`<div class="card-body">
    <p>当前模型 <strong>${esc(value('name',state.model.name))}</strong>
    <button class="button" data-go="specs">编辑规格与模型</button></p>
    <p><strong>基座：固定安装</strong>。保留公共模型中的安装位置与朝向；未填写基座质量属性时，直接省略基座质量、质心和惯量，无需为基座填写估算密度。运动刚体的质量属性仍需提供或估算。</p>
    <form id="simulationExport"><fieldset ${disabled}>
      <div class="actions-row">${Object.entries(simulationFormats).map(([target,label])=>
        `<label><input type="checkbox" name="target" value="${target}" ${simulation.targets.includes(target)?'checked':''}> ${label}</label>`).join('')}</div>
      <label>导出模型名 <input name="robotName" required maxlength="64" pattern="[A-Za-z][A-Za-z0-9_]*" value="${esc(simulation.robotName)}"></label>
      <h3>关节导出限值</h3><p>模型中已有的上限会自动带入，请补填空白项。</p>
      <div class="table-scroll"><table><thead><tr><th>关节</th><th>力矩或力上限</th><th>速度上限</th></tr></thead><tbody>${rows}</tbody></table></div>
      <p><strong>质量属性：自动补齐</strong>。默认使用几何代理估算缺失属性；已有质量、质心和惯量优先保留。仅缺惯量时按已知质量估算；运动刚体缺少质量部件时按等效密度 ${esc(simulation.density)} kg/m³ 估算（默认 2000）。未填写属性的固定基座不参与估算。</p>
      <details><summary>高级：调整估算密度与查看估算说明</summary>
        <label>等效估算密度 / kg/m³ <input type="number" name="density" min="0.000001" step="any" value="${esc(simulation.density)}" required></label>
        <p>默认 2000 kg/m³ 是前期概念估算假设，可按实际设计修改。仅在整个运动刚体缺少质量部件时，使用「密度 × 代理实体体积」估算质量；已有质量不受密度影响。若代理体积为 0.001 m³、目标质量为 2 kg，等效密度填 2000。</p>
        <p>「规格与模型 → 基座质量属性」可填写固定基座实际质量、质心和惯量。基座留空则省略其质量属性；已填基座质量但缺惯量时，按已知质量和代理几何补齐惯量。固定约束始终保留。</p>
        <p>估算只用于本次仿真导出，不回写公共模型。模型中的正质量末端负载作为附加质量导出，估算来源会保留在导出报告中。</p>
      </details>
      <div class="actions-row">
        <button type="submit" class="button primary">${simulation.busy?'正在生成…':'生成并下载 ZIP'}</button>
      </div>
    </fieldset></form><p id="simulationStatus" role="status">${esc(simulation.message)}</p>
    </div>`);
}
function bindSimulation(form){
  const fields=form.elements;
  const active=state.model.joints.filter(joint=>joint.type!=='fixed');
  const readSettings=()=>{
    simulation.robotName=fields.robotName.value;
    simulation.targets=Array.from(form.querySelectorAll('input[name="target"]:checked'),input=>input.value);
    simulation.density=fields.density.value;
    for(const [index,joint] of active.entries()){
      const limits={};
      for(const field of ['effort','velocity']){
        const input=fields[field+'-'+index];
        if(!input.readOnly)limits[field]=input.value;
      }
      simulation.limits.set(joint.id,limits);
    }
  };
  form.oninput=readSettings;
  form.onsubmit=async event=>{
    event.preventDefault();
    if(simulation.busy||state.busy)return;
    readSettings();
    simulation.busy=true;
    simulation.message='正在调用仿真模型生成 Skill…';
    hooks.render();
    try{
      const design_model=buildModel();
      const limits=Object.fromEntries([...simulation.limits].map(([id,limit])=>[
        id,Object.fromEntries(Object.entries(limit).map(([field,value])=>[field,Number(value)])),
      ]));
      const response=await fetch('/api/simulation-export',{method:'POST',headers:{'content-type':'application/json'},
        body:JSON.stringify({design_model,robot_name:simulation.robotName,targets:simulation.targets,limits,
          density_kg_m3:Number(simulation.density)})});
      if(!response.ok){
        const error=await response.json();
        throw Error(error.detail??error.error);
      }
      const targets=JSON.parse(response.headers.get('x-simulation-targets'));
      const sceneModelId=response.headers.get('x-scene-model-id');
      if(sceneModelId){try{sessionStorage.setItem('srs-latest-generated-model',sceneModelId);}catch{}}
      const filename=response.headers.get('content-disposition')?.match(/filename="([^"]+)"/)?.[1]??'arm-simulation.zip';
      hooks.download(filename,await response.blob(),'application/zip');
      const statuses=Object.entries(targets).map(([target,status])=>simulationFormats[target]+' '+(simulationStatus[status]??status));
      simulation.message=design_model.name+' · 已下载 ZIP。'+statuses.join('；')+'。'+(sceneModelId?'模型已联通「机械臂 3D 模型」与「场景仿真」。':'')+'详细状态见 report/manifest.json。';
    }catch(error){simulation.message=error.message;}
    finally{simulation.busy=false;hooks.render();}
  };
}
function provenanceHTML(){
  const model=state.model;
  if(!model)return card('公共模型来源','<p>请在「规格与模型」选择或导入模型。</p>');
  const provenance=[model.provenance,model.payload?.provenance,...(model.base_components??[]).map(component=>component.provenance),...model.joints.flatMap(joint=>(joint.components??[]).map(component=>component.provenance))].filter(Boolean);
  const sources=[...new Set(provenance.map(item=>item.source))];
  const assumptions=[...new Set(provenance.flatMap(item=>item.assumptions??[]))];
  return card('公共模型来源',`<p>${esc(value('name',model.name))}</p>
    <ul>${[...sources,...assumptions].map(text=>'<li>'+esc(text)+'</li>').join('')}</ul>
    <button class="button" data-go="specs">打开规格与模型</button>`);
}
export const ModuleDesignPage={modelName:()=>state.model?value('name',state.model.name):null,specsHTML,specificationsHTML,provenanceHTML,simulationHTML,bind,bindSpecifications,analysisHTML:()=>state.analysisActive?(state.busy?card('关节模组分析','<p role="status">正在联合计算静力学、可达空间与动态载荷，请稍候…</p>'):state.spec?specificationsHTML(true):card('关节模组分析',`<p role="status">${esc(state.message)}</p>`)):'',analysisBusy:()=>state.busy,record,run:()=>action('run'),analyze:request=>action('run',null,request),save:()=>action('download'),
  exportSpecifications:kind=>{
    if(!state.spec)return false;
    if(kind==='csv')hooks.download('arm-module-requirements.csv',csv(state.spec),'text/csv;charset=utf-8');
    else hooks.download('arm-module-requirements.json',JSON.stringify(state.spec,null,2));
    return true;
  }};
import {combineResults} from './module-specifications.mjs';

