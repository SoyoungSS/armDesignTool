const labels={passed:'通过',needs_review:'待复核',failed:'未通过',not_evaluated:'未检查'};
export function generatedModelHTML(){return `<section class="card run-view" id="run-view"><div class="card-head"><label>已导出的机械臂 <select data-model disabled><option>正在读取模型列表…</option></select></label><div class="actions-row" style="margin:0"><button class="button" data-model-refresh>刷新模型列表</button><button class="button" data-reset disabled>复位视角</button></div></div><div class="run-canvas" aria-label="已导出机械臂三维模型"></div><div class="card-body"><p class="run-status" role="status">正在读取服务器保存的模型…</p><div class="actions-row" data-controls hidden><label>姿态 <select data-pose><option value="0">初始姿态</option><option value="1">几何零位</option></select></label></div><small>拖动旋转 · 滚轮缩放 · 右键平移。预览读取实际导出几何；模型检查状态单独展示。</small><div data-evidence role="status"></div><div class="actions-row" data-links></div></div></section>`;}
export function chooseGeneratedModel(models,{requested,lastExport,current}={}){
  if(lastExport&&models.some(model=>model.id===lastExport))return lastExport;
  if(['demo:worksheet','demo:fr3'].includes(requested))return requested;
  for(const id of [requested,current])if(id&&models.some(model=>model.id===id))return id;
  if(models.length)return models[0].id;
  return 'demo:worksheet';
}
export async function mountGeneratedModel({createScene,closeViewer,setDispose}){
  closeViewer();const el=document.getElementById('run-view');if(!el)return;
  const find=s=>el.querySelector(s),select=find('[data-model]'),controller=new AbortController();
  let view,loading=null,models=[],current=null;
  setDispose(()=>{controller.abort();loading?.abort();view?.dispose();});
  function link(label,url){const a=document.createElement('a');a.textContent=label;a.href=url;a.target='_blank';a.rel='noopener';a.className='button';find('[data-links]').append(a);}
  function option(value,label){const o=document.createElement('option');o.value=value;o.textContent=label;return o;}
  async function load(value){
    loading?.abort();const request=new AbortController();loading=request;view?.dispose();view=null;
    find('.run-canvas').replaceChildren();find('[data-links]').replaceChildren();find('[data-evidence]').textContent='';find('[data-controls]').hidden=true;find('[data-reset]').disabled=true;
    current=value;select.value=value;
    const demo=value.startsWith('demo:'),model=models.find(item=>item.id===value),demoId=value.slice(5);
    const name=demo?(demoId==='fr3'?'Franka FR3（参考示例）':'选型表七轴单臂（参考示例）'):model?.name;
    find('.run-status').textContent=demo?'正在读取参考示例…':'正在准备所选导出模型的三维预览…';
    try{
      const response=await fetch(demo?`/demo-assets/${demoId}/model.json`:`/api/generated-models/${value}/viewer`,{signal:request.signal});
      if(!response.ok){let detail;try{const error=await response.json();detail=error.detail??error.error;}catch{}throw Error(demo?'参考示例数据尚未部署；可先在“仿真模型生成”生成自己的 MuJoCo 模型。':detail||'无法读取已导出模型');}
      const spec=await response.json();if(controller.signal.aborted||request.signal.aborted)return;
      if(!spec.frames?.length||!spec.geoms?.length)throw Error('导出数据缺少可显示的几何或姿态。');
      view=createScene(find('.run-canvas'),spec);const pose=find('[data-pose]');
      pose.replaceChildren();for(let i=0;i<spec.frames.length;i++)pose.append(option(String(i),i===0?'初始姿态':i===1?'几何零位':`姿态 ${i+1}`));
      const show=()=>{
        const index=Number(pose.value)||0;view.show(index);
        if(spec.collision){const status=spec.collision.poses?.[index?'geometric_zero':'initial'];find('[data-evidence]').textContent=`当前姿态零件检查：${labels[status]??status??'未记录'}；扩展范围碰撞检查：${labels[spec.collision.overall]??spec.collision.overall??'未记录'}。`;}
        else find('[data-evidence]').textContent='未提供碰撞检查记录。三维预览不等于仿真或碰撞验收通过。';
      };
      pose.value='0';pose.onchange=show;show();find('[data-controls]').hidden=false;find('[data-reset]').disabled=false;find('[data-reset]').onclick=()=>view?.reset();
      find('.run-status').textContent=`${name} · ${spec.joints?.length??'—'} 个关节 · ${spec.geoms.length} 个实际导出几何体${model?.mujoco_status==='failed'?' · 模型检查未通过，请查看报告':''}`;
      const project=document.getElementById('projectLabel');if(project)project.textContent=name;
      link('模型检查报告',demo?`/demo-assets/${demoId}/validation.json`:`/api/generated-models/${value}/validation`);
      if(!demo)link('导出清单',`/api/generated-models/${value}/manifest`);
      const url=new URL(location.href);url.searchParams.set('model',value);url.searchParams.delete('arm');history.replaceState(null,'',url);
      if(!demo){try{sessionStorage.setItem('srs-viewer-model',value);}catch{}}
      return true;
    }catch(error){if(!controller.signal.aborted&&!request.signal.aborted)find('.run-status').textContent='无法显示：'+error.message;}
  }
  async function refresh(){
    select.disabled=true;
    try{
      const response=await fetch('/api/generated-models',{signal:controller.signal});if(!response.ok)throw Error('模型列表读取失败，请确认服务已更新');
      models=(await response.json()).models;if(controller.signal.aborted)return;
      let lastExport,saved;try{lastExport=sessionStorage.getItem('srs-latest-generated-model');saved=sessionStorage.getItem('srs-viewer-model');}catch{}
      const requested=new URL(location.href).searchParams.get('model')??saved;
      const next=chooseGeneratedModel(models,{requested,lastExport,current});
      select.replaceChildren();if(models.length){const group=document.createElement('optgroup');group.label='已导出的模型';for(const model of models)group.append(option(model.id,`${model.name} · ${model.created_at.slice(0,19).replace('T',' ')}`));select.append(group);}
      const demos=document.createElement('optgroup');demos.label='参考示例（独立预计算文件）';demos.append(option('demo:worksheet','选型表七轴单臂（参考示例）'),option('demo:fr3','Franka FR3（参考示例）'));select.append(demos);select.disabled=false;
      await load(next);if(lastExport===next&&find('[data-controls]').hidden===false){try{sessionStorage.removeItem('srs-latest-generated-model');}catch{}}
    }catch(error){if(!controller.signal.aborted)find('.run-status').textContent=error.message;}
  }
  select.onchange=()=>load(select.value);find('[data-model-refresh]').onclick=refresh;
  await refresh();
}
