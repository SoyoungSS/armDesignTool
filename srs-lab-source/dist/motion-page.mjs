import {runMotion,previewMotion} from './motion-client.mjs';
import {dynamicsCSV} from './motion-export.mjs';
const esc=s=>String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const num=(x,n=3)=>x===null||x===undefined?'未提供':Number(x).toFixed(n);
const $=id=>document.getElementById(id);
const state={input:'',data:null,busy:false,message:'请先在「规格与模型」应用模型。',index:0,field:'q',joint:0,token:0,playing:false};
const randomFields=[
  ['seed','随机种子',42,0,4294967295,1],
  ['trajectory_count','轨迹条数',3,1,10,1],
  ['move_count','每条运动段数',3,1,20,1],
  ['max_step_fraction','单段最大位移 / 关节行程 %',10,.1,100,'any'],
  ['min_speed_fraction','速度下限 / %',40,5,100,'any'],
  ['max_speed_fraction','速度上限 / %',80,5,100,'any'],
  ['min_hold_s','停留时间下限 / s',0,0,60,'any'],
  ['max_hold_s','停留时间上限 / s',.5,0,60,'any'],
];
const randomSettings=Object.fromEntries(randomFields.map(([key,,value])=>[key,value]));
let batch=null,selectedTrajectory=0;
let scene=null,timer=null,controller=null;
const card=(title,body)=>`<section class="card"><div class="card-head"><h2>${title}</h2></div><div class="card-body">${body}</div></section>`;
const invalidate=()=>{window.ModuleDesignPage?.record('motion',null);state.token++;state.data=null;stop();};
function stop(){if(timer)clearInterval(timer);timer=null;state.playing=false;}
function download(name,text,type='application/json'){const a=document.createElement('a'),url=URL.createObjectURL(new Blob([text],{type}));a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}
function randomHTML(){
  const fields=randomFields.map(([key,label,,min,max,step])=>`<div class="field"><label for="random-${key}">${label}</label><input id="random-${key}" type="number" value="${randomSettings[key]??''}" min="${min}" max="${max}" step="${step}" required></div>`).join('');
  const disabled=state.busy?'disabled':'';
  const selection=batch?`<label for="motionRandomSelect">生成的轨迹（种子 ${batch.settings.seed}）</label>
    <select id="motionRandomSelect" ${disabled}>${batch.trajectories.map((item,i)=>`<option value="${i}" ${selectedTrajectory===i?'selected':''}>第 ${i+1} 条 · ${num(item.duration_s,2)} s</option>`).join('')}</select>
    <button type="button" class="button" id="motionRandomSave">下载生成批次 JSON</button>`:'';
  return `<form id="motionRandomForm">
    <h3>关节空间随机生成</h3>
    <p>沿用公共模型、轨迹起点和逐关节运动限制。速度百分比用于调整整段节奏，每个关键点停稳。</p>
    <div class="form-grid motion-random-fields">${fields}</div>
    <div class="actions-row"><button class="button" type="submit" ${disabled}>生成随机轨迹</button>${selection}</div>
  </form>`;
}
function useRandomTrajectory(index){
  invalidate();
  selectedTrajectory=index;
  state.input=JSON.stringify(batch.trajectories[index].request,null,2);
  state.message=`已载入第 ${index+1} 条随机轨迹，共 ${batch.trajectories.length} 条。可编辑 JSON 或运行轨迹分析。`;
}
function dynamicHTML(result){
  if(!result.scope.dynamics_computed)return '';
  const rows=result.summary.dynamic_joints.map((joint,i)=>`<tr>
    <td>${esc(result.joint_names[i])}</td><td>${num(joint.effort_peak_abs)} ${joint.effort_unit}</td>
    <td>${num(joint.effort_rms)} ${joint.effort_unit}</td><td>${num(joint.power_peak_abs_W)}</td>
    <td>${num(joint.axial_force_peak_abs_N)}</td><td>${num(joint.radial_force_peak_N)}</td>
    <td>${num(joint.bending_moment_peak_Nm)}</td>
    <td><button class="button" data-motion-peak="${i}">${num(joint.peak_time_s)} s</button></td></tr>`).join('');
  return card('动态载荷与关节接口',`<p>刚体逆动力学输出关节输出端需求。接口力和力矩采用基座坐标轴，作用点为随关节运动的原点，方向为上游对下游子链的作用。峰值和 RMS 根据采样时序统计。</p>
    <div class="table-scroll"><table><thead><tr><th>关节</th><th>力矩 / 推力峰值</th><th>RMS</th><th>功率峰值 / W</th><th>轴向力峰值 / N</th><th>径向力峰值 / N</th><th>弯矩峰值 / Nm</th><th>定位力矩 / 推力峰值</th></tr></thead><tbody>${rows}</tbody></table></div>
    <p>演示模型的惯量来源与估算假设见下方完整结果说明。实际选型需补充传动效率、摩擦、转子惯量及模组额定能力。</p>
    <div class="actions-row"><button class="button" id="motionLoadsCSV">下载动态载荷 CSV</button></div>`);
}
function chart(result){
  const samples=result.samples,j=state.joint,key=state.field,values=samples.map(s=>s[key]?.[j]??null),finite=values.filter(x=>x!==null);
  if(!finite.length)return '<p>该记录未提供此导数。可在请求中显式启用估计，估计会保留质量标记。</p>';
  const lo=Math.min(0,...finite),hi=Math.max(0,...finite),span=Math.max(hi-lo,1e-9),start=samples[0].time_s,duration=Math.max(samples.at(-1).time_s-start,1e-9);
  const x=t=>45+(t-start)/duration*700,y=v=>185-(v-lo)/span*150;
  const path=samples.filter((_,i)=>values[i]!==null).map((s,i)=>`${i?'L':'M'}${x(s.time_s).toFixed(2)},${y(s[key][j]).toFixed(2)}`).join(' ');
  return `<svg viewBox="0 0 800 230" role="img" aria-label="关节状态时间曲线" style="width:100%;background:#f7f8f9"><line x1="45" y1="185" x2="745" y2="185" stroke="#9ba5ae"/><path d="${path}" fill="none" stroke="#84384c" stroke-width="2"/><line x1="${x(samples[state.index].time_s)}" y1="25" x2="${x(samples[state.index].time_s)}" y2="185" stroke="#258064"/><text x="45" y="215">${num(start)} s</text><text x="660" y="215">${num(samples.at(-1).time_s)} s</text><text x="45" y="18">${esc(result.joint_names[j])} · ${esc(key)} / ${key==='effort'?(result.joint_types[j]==='revolute'?'Nm':'N'):(result.joint_types[j]==='revolute'?'rad':'m')}${key==='dq'?'/s':key==='ddq'?'/s²':''} · ${num(lo)} ～ ${num(hi)}</text></svg>`;
}
function resultHTML(){
  const r=state.data.result,s=r.summary;
  const constraint={verified:'声明运动约束通过',violated:'存在约束违规',incomplete:'约束信息不足',samples_passed:'样本检查通过，点间未知'}[s.constraint_status];
  return `<div class="kpis"><div class="kpi"><div class="label">轨迹时长</div><strong>${num(s.duration_s,2)}</strong><span>s</span></div><div class="kpi"><div class="label">计算样本</div><strong>${s.sample_count}</strong></div><div class="kpi"><div class="label">奇异 / 接近奇异样本</div><strong>${s.singular_sample_count} / ${s.near_singular_sample_count}</strong></div><div class="kpi"><div class="label">最小奇异值（已尺度化）</div><strong>${num(s.minimum_sigma,4)}</strong></div></div>`+
  card('轨迹状态与几何预览',`<p><b>${esc(r.normalized_model.name)}</b> · ${esc(constraint)}。本页结果独立于主设计模型，不把 FR3 姿态应用到选型表。${r.scope.dynamics_computed?'已计算动态载荷。':'可启用动态载荷计算。'}</p><div class="actions-row"><button class="button" id="motionPlay">播放 / 暂停</button><label for="motionTime">时间游标</label><input id="motionTime" type="range" min="0" max="${r.samples.length-1}" step="1" value="${state.index}" style="width:min(350px,50vw)"><output id="motionTimeLabel"></output></div><p id="motionSampleInfo" role="status"></p><div class="viewport"><canvas id="motionScene" aria-label="轨迹机械臂与TCP路径"></canvas></div><div class="actions-row"><label for="motionJoint">关节</label><select id="motionJoint">${r.joint_names.map((name,i)=>`<option value="${i}" ${i===state.joint?'selected':''}>${esc(name)}</option>`).join('')}</select><label for="motionField">状态量</label><select id="motionField">${[['q','位置'],['dq','速度'],['ddq','加速度'],...(r.scope.dynamics_computed?[['effort','力矩 / 推力']]:[])].map(([k,l])=>`<option value="${k}" ${k===state.field?'selected':''}>${l}</option>`).join('')}</select></div><div id="motionChart">${chart(r)}</div>`)+
  dynamicHTML(r)+card('风险事件与导出',`<p>奇异任务：${esc(r.singularity_settings.task)}，特征长度 ${num(r.singularity_settings.characteristic_length_m)} m；近奇异阈值 ${r.singularity_settings.near_sigma_threshold}。风险区间为采样观察，不能视作精确边界。</p>${s.task_underactuated?'<p>任务维度超过主动关节数，存在结构性欠驱动。</p>':''}<div class="actions-row">${r.events.slice(0,80).map((e,i)=>`<button class="button" data-motion-event="${i}">${esc(e.type)} · ${num(e.time_s??e.start_s??0)} s</button>`).join('')||'<p>未记录风险事件；不代表碰撞或实际任务已验证。</p>'}</div><small>展示前80条事件，完整记录见 JSON。</small><div class="actions-row"><button class="button" id="motionJSON">下载完整结果 JSON</button><button class="button" id="motionCSV">下载关节时序 CSV</button></div><p>${r.warnings.map(esc).join('<br>')}</p>`);
}
function html(){if(!state.input)return card('轨迹与动态载荷', '<p>请先应用公共模型。</p><button class="button" data-go="specs">打开规格与模型</button>');let input;try{input=JSON.parse(state.input);}catch{}const generated=input?.trajectory?.kind==='waypoints';return card('独立轨迹、奇异性与动态载荷 Skill',`<p>关节空间关键点停靠轨迹、外部轨迹、TCP 状态和奇异性；FK/雅可比复用公共库。可启用刚体动态载荷计算。示例运动限制和演示惯量带有明确假设。</p><div class="actions-row"><button class="button primary" id="motionRun" ${state.busy?'disabled':''}>${state.busy?'处理中…':'运行轨迹分析'}</button>${state.busy?'<button class="button" id="motionCancel">取消</button>':''}</div><div class="actions-row"><label><input id="motionDynamics" type="checkbox" ${input?.dynamics?.enabled?'checked':''}> 计算动态载荷（需质量、质心及惯量）</label></div><div class="actions-row"><label for="motionDuration">每段运动时长 / s</label><input id="motionDuration" type="number" min="0.01" step="0.1" value="${esc(Array.isArray(input?.trajectory?.segments)?input.trajectory.segments.find(s=>s?.kind==='move')?.duration_s??2:2)}" ${generated?'':'disabled'} style="width:90px"><label for="motionDt">采样间隔 / s</label><input id="motionDt" type="number" min="0.0001" step="0.01" value="${esc(input?.trajectory?.sample_dt_s??.02)}" ${generated?'':'disabled'} style="width:90px"><label for="motionTask">奇异性任务</label><select id="motionTask"><option value="pose" ${input?.singularity?.task!=='position'?'selected':''}>完整位姿</option><option value="position" ${input?.singularity?.task==='position'?'selected':''}>仅位置</option></select></div>${randomHTML()}<p>几何和质量参数在「规格与模型」编辑。此处可编辑关节轨迹工况。</p><textarea id="motionSource" aria-label="关节轨迹工况 JSON" spellcheck="false" style="width:100%;min-height:220px;padding:12px;margin-top:12px;font:12px/1.5 monospace">${esc(JSON.stringify(input?.trajectory,null,2))}</textarea><p id="motionMessage" role="status">${esc(state.message)}</p>`)+(state.data?'<div id="motionResults">'+resultHTML()+'</div>':'');}
function render(){const content=$('content');if(!content||location.hash!=='#motion')return;stop();scene=null;content.innerHTML=html();bind();}
function update(){
  if(!state.data||!$('motionScene')){stop();return;}const r=state.data.result,s=r.samples[state.index];$('motionTime').value=state.index;$('motionTimeLabel').textContent=num(s.time_s)+' s / '+s.phase_id;
  const status={regular:'常规',near_singular:'接近奇异',singular:'奇异'}[s.singularity.status];$('motionSampleInfo').textContent=`${status}；最小奇异值 ${num(s.singularity.sigma_min,5)}；秩 ${s.singularity.effective_rank}/${s.singularity.task_dimension}；近限位关节：${s.near_limit_joint_ids.join('、')||'无'}；${r.trajectory.derivatives}`;
  const preview=previewMotion(r,state.index);if(!scene){const stride=Math.max(1,Math.ceil(r.samples.length/200)),fitPositions=r.samples.flatMap((_,i)=>i%stride===0||i===r.samples.length-1?previewMotion(r,i).nodes_world_m:[]);scene=new window.WorkspaceScene($('motionScene'),{points:r.samples.map((s,i)=>({id:i,position_world_m:s.tcp_world.position_m})),summary:{}},preview,{fitPositions});}else{scene.preview=preview;scene.render();}
  $('motionChart').innerHTML=chart(r);
}
function bind(){
  if(!$('motionSource'))return;
  for(const [key] of randomFields){
    const field=$('random-'+key);
    field.oninput=()=>{randomSettings[key]=Number.isFinite(field.valueAsNumber)?field.valueAsNumber:null;};
  }
  $('motionRandomForm').onsubmit=async event=>{
    event.preventDefault();
    if(state.busy)return;
    let template;
    try{template=JSON.parse(state.input);template.trajectory=JSON.parse($('motionSource').value);}catch{$('motionMessage').textContent='请先载入示例或输入有效的轨迹 JSON';return;}
    const settings=Object.fromEntries(Object.entries(randomSettings).map(([key,value])=>[key,key.endsWith('_fraction')?value/100:value]));
    state.busy=true;
    const token=++state.token;
    controller=new AbortController();
    state.message='正在生成随机轨迹…';
    render();
    try{
      const {result}=await runMotion({schema_name:'arm-random-motion-request',schema_version:'1.0',template,settings},{
        signal:controller.signal,expected:'arm-random-motion-result',
      });
      if(token===state.token){
        batch=result;
        useRandomTrajectory(0);
      }else state.message='输入已修改，请重新生成';
    }catch(error){state.message=controller.signal.aborted?'已取消':error.message;}
    finally{state.busy=false;controller=null;render();}
  };
  if(batch){
    $('motionRandomSelect').onchange=event=>{useRandomTrajectory(Number(event.target.value));render();};
    $('motionRandomSave').onclick=()=>download('motion-random-batch.json',JSON.stringify(batch,null,2));
  }
  const edit=fn=>{
    try{
      const input=JSON.parse(state.input);
      input.trajectory=JSON.parse($('motionSource').value);
      fn(input);
      state.input=JSON.stringify(input,null,2);
      invalidate();
      batch=null;
      state.message='设置已修改，请重新分析。';
      render();
    }catch(e){$('motionMessage').textContent=e.message;}
  };
  $('motionSource').oninput=()=>{
    invalidate();
    $('motionResults')?.remove();
    const input=JSON.parse(state.input);
    try{
      input.trajectory=JSON.parse($('motionSource').value);
      state.input=JSON.stringify(input,null,2);
      $('motionMessage').textContent='轨迹已修改，请重新分析。';
    }catch{$('motionMessage').textContent='请输入有效的轨迹 JSON。';}
  };
  $('motionDynamics').onchange=event=>edit(input=>{input.dynamics={...input.dynamics,enabled:event.target.checked};});
  $('motionDuration').onchange=e=>edit(input=>input.trajectory.segments.filter(s=>s.kind==='move').forEach(s=>s.duration_s=e.target.valueAsNumber));
  $('motionDt').onchange=e=>edit(input=>input.trajectory.sample_dt_s=e.target.valueAsNumber);
  $('motionTask').onchange=e=>edit(input=>{input.singularity={...input.singularity,task:e.target.value};});
  $('motionRun').onclick=async()=>{if(state.busy)return;try{const input=JSON.parse(state.input);input.trajectory=JSON.parse($('motionSource').value);state.input=JSON.stringify(input,null,2);}catch{$('motionMessage').textContent='请输入有效的轨迹 JSON';return;}stop();window.ModuleDesignPage?.record('motion',null);state.data=null;state.busy=true;const token=++state.token,raw=state.input;controller=new AbortController();state.message='独立 CLI 正在分析…';render();try{const data=await runMotion(raw,{signal:controller.signal});if(token!==state.token||state.input!==raw){state.message='输入已改变，丢弃过期结果';}else{state.data=data;window.ModuleDesignPage?.record('motion',data.result);state.index=0;state.joint=0;state.field='q';state.message='Skill 执行完成 · '+data.result.summary.constraint_status;}}catch(e){state.message=controller.signal.aborted?'已取消':e.message;}finally{state.busy=false;controller=null;render();}};
  if($('motionCancel'))$('motionCancel').onclick=()=>controller?.abort();
  if(state.data){
    const r=state.data.result;scene=null;update();
    $('motionTime').oninput=e=>{state.index=+e.target.value;update();};
    $('motionJoint').onchange=e=>{state.joint=+e.target.value;update();};$('motionField').onchange=e=>{state.field=e.target.value;update();};
    $('motionPlay').onclick=()=>{if(timer){stop();return;}if(state.index===r.samples.length-1)state.index=0;state.playing=true;const wall=performance.now(),start=r.samples[state.index].time_s;timer=setInterval(()=>{if(!document.body.contains($('motionScene'))||location.hash!=='#motion'){stop();return;}const t=start+(performance.now()-wall)/1000;while(state.index<r.samples.length-1&&r.samples[state.index+1].time_s<=t)state.index++;update();if(t>=r.samples.at(-1).time_s)stop();},30);};
    document.querySelectorAll('[data-motion-event]').forEach(b=>b.onclick=()=>{stop();const e=r.events[+b.dataset.motionEvent],time=e.time_s??e.critical_time_s??e.start_s??0;state.index=r.samples.reduce((best,s,i)=>Math.abs(s.time_s-time)<Math.abs(r.samples[best].time_s-time)?i:best,0);update();});
    if(r.scope.dynamics_computed){
      $('motionLoadsCSV').onclick=()=>download('dynamic-loads.csv',dynamicsCSV(r),'text/csv');
      document.querySelectorAll('[data-motion-peak]').forEach(button=>button.onclick=()=>{
        stop();
        state.joint=Number(button.dataset.motionPeak);
        const time=r.summary.dynamic_joints[state.joint].peak_time_s;
        state.index=r.samples.findIndex(sample=>sample.time_s===time);
        state.field='effort';
        $('motionField').value='effort';
        $('motionJoint').value=String(state.joint);
        update();
      });
    }
    $('motionJSON').onclick=()=>download('motion-result.json',JSON.stringify(state.data,null,2));
    $('motionCSV').onclick=()=>{const rows=[['time_s','phase_id','joint_id','q','dq','ddq','sigma_min','singularity_status']];for(const s of r.samples)r.joint_ids.forEach((id,i)=>rows.push([s.time_s,s.phase_id,id,s.q[i],s.dq?.[i]??'',s.ddq?.[i]??'',s.singularity.sigma_min,s.singularity.status]));download('motion-trajectory.csv',rows.map(row=>row.map(v=>'"'+String(v).replaceAll('"','""')+'"').join(',')).join('\n'),'text/csv');};
  }
}
window.addEventListener('hashchange',()=>{if(location.hash!=='#motion')stop();});window.addEventListener('resize',()=>{if(scene&&$('motionScene'))scene.render();});
export const MotionPage={html,bind,applyRequest(request,data){
  invalidate();
  batch=null;
  state.input=request?JSON.stringify(request,null,2):'';
  state.data=data??null;
  state.field='q';
  state.index=0;
  state.joint=0;
  state.message=data?'已载入同模型的联算结果。':request?'已应用公共模型，可运行轨迹分析。':'请先在「规格与模型」应用模型。';
}};
