import * as THREE from 'three';
import {generatedModelHTML,mountGeneratedModel} from './generated-model-view.mjs';
import { OrbitControls } from './vendor/three/OrbitControls.js';

const arms = {worksheet:'选型表七轴单臂', fr3:'Franka FR3'};
let selected = new URLSearchParams(location.search).get('arm') === 'fr3' ? 'fr3' : 'worksheet';
let disposeActive = null;
const phaseLabels = {initial:'初始化', approach:'接近', descend:'下降抓取', grip:'夹持', close:'闭合', grasp:'抓取', lift:'抬升', carry:'搬运', transfer:'搬运', lower:'下降放置', release:'松爪', retreat:'回撤', settle:'稳定性检查', return_home:'返回初始姿态', end_dwell:'轮末稳定性检查', home:'返回', done:'完成'};
const objectLabels = {round_can:'红色圆柱',stepped_block:'红色阶梯块',block:'蓝色方块',twin_posts:'蓝色双柱件'};

function geometry(g, meshes) {
  const [x,y,z] = g.size;
  if (g.kind === 'mesh') {
    const m = meshes[g.mesh];
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position',new THREE.Float32BufferAttribute(m.vertices,3));
    geo.setIndex(m.faces);
    geo.computeVertexNormals();
    return geo;
  }
  if (g.kind === 'plane') return new THREE.PlaneGeometry(2*(x || 1.4),2*(y || 1.4));
  if (g.kind === 'box') return new THREE.BoxGeometry(2*x,2*y,2*z);
  if (g.kind === 'sphere') return new THREE.SphereGeometry(x,32,20);
  if (g.kind === 'ellipsoid') return new THREE.SphereGeometry(1,32,20).scale(x,y,z);
  if (g.kind === 'capsule') return new THREE.CapsuleGeometry(x,2*y,8,24).rotateX(Math.PI/2);
  return new THREE.CylinderGeometry(x,x,2*y,32).rotateX(Math.PI/2);
}

function setQuaternion(target, q, offset=0) {
  target.set(q[offset+1],q[offset+2],q[offset+3],q[offset]);
}

function createScene(viewport, spec) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0xf1f4f4);
  const camera = new THREE.PerspectiveCamera(35,1,.001,30);
  camera.up.set(0,0,1);
  const renderer = new THREE.WebGLRenderer({antialias:true});
  renderer.setPixelRatio(Math.min(devicePixelRatio,2));
  viewport.appendChild(renderer.domElement);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = false;
  const draw = () => renderer.render(scene,camera);
  controls.addEventListener('change',draw);
  scene.add(new THREE.HemisphereLight(0xffffff,0xc1cbd0,2));
  const key = new THREE.DirectionalLight(0xffffff,1.8);
  key.position.set(2,-3,5);
  scene.add(key);
  const bodies = Array.from({length:spec.frames[0].bodies.length/7}, () => {
    const body = new THREE.Group();
    scene.add(body);
    return body;
  });
  const meshes = [];
  for (const g of spec.geoms) {
    const [r,green,b,a] = g.rgba;
    const material = new THREE.MeshStandardMaterial({color:new THREE.Color().setRGB(r,green,b,THREE.SRGBColorSpace),roughness:.57,metalness:.13,side:THREE.DoubleSide,transparent:a<1,opacity:a});
    const mesh = new THREE.Mesh(geometry(g,spec.meshes),material);
    mesh.position.fromArray(g.pos);
    setQuaternion(mesh.quaternion,g.quat);
    bodies[g.body].add(mesh);
    meshes.push(mesh);
  }
  function show(index) {
    const values = spec.frames[index].bodies;
    bodies.forEach((body,i) => {
      body.position.fromArray(values,7*i);
      setQuaternion(body.quaternion,values,7*i+3);
    });
    draw();
  }
  show(0);
  const bounds = new THREE.Box3();
  for (let i=0;i<meshes.length;i++) {
    if(spec.geoms[i].kind !== 'plane') bounds.expandByObject(meshes[i]);
  }
  const center = bounds.getCenter(new THREE.Vector3());
  const size = bounds.getSize(new THREE.Vector3());
  const span = Math.max(size.length(),.2);
  const grid = new THREE.GridHelper(span*2,24,0xa9b6bb,0xd2dadc);
  grid.rotation.x = Math.PI/2;
  grid.position.set(center.x,center.y,bounds.min.z-.004);
  if (!spec.geoms.some(g=>g.kind==='plane')) scene.add(grid);
  function reset() {
    controls.target.copy(center);
    const direction = new THREE.Vector3(1,-.62,.55).normalize();
    const right = new THREE.Vector3().crossVectors(camera.up,direction).normalize();
    const up = new THREE.Vector3().crossVectors(direction,right);
    const slope = Math.tan(THREE.MathUtils.degToRad(camera.fov/2));
    let distance = 0;
    for (const x of [-.5,.5]) for (const y of [-.5,.5]) for (const z of [-.5,.5]) {
      const corner = new THREE.Vector3(size.x*x,size.y*y,size.z*z);
      const projected = Math.max(Math.abs(corner.dot(up)),Math.abs(corner.dot(right))/camera.aspect);
      distance = Math.max(distance,corner.dot(direction)+projected/slope);
    }
    camera.position.copy(center).addScaledVector(direction,distance*1.06);
    camera.near = Math.max(.0005,span/2000);
    camera.far = span*60;
    camera.updateProjectionMatrix();
    controls.minDistance = span*.2;
    controls.maxDistance = span*7;
    controls.update();
    draw();
  }
  function resize() {
    camera.aspect = viewport.clientWidth/Math.max(1,viewport.clientHeight);
    camera.updateProjectionMatrix();
    renderer.setSize(viewport.clientWidth,viewport.clientHeight,false);
    draw();
  }
  resize();
  reset();
  const observer = new ResizeObserver(resize);
  observer.observe(viewport);
  return {show,reset,dispose(){
    observer.disconnect();
    controls.dispose();
    meshes.forEach(m=>{m.geometry.dispose();m.material.dispose();});
    grid.geometry.dispose(); grid.material.dispose();
    renderer.dispose();renderer.forceContextLoss();renderer.domElement.remove();
  }};
}

export function viewerHTML(task) {
  if(!task)return generatedModelHTML();
  return `<section class="card run-view" id="run-view">
    <div class="card-head"><label>机械臂 <select data-arm>${Object.entries(arms).map(([id,label])=>`<option value="${id}" ${id===selected?'selected':''}>${label}</option>`).join('')}</select></label>
    <button type="button" class="button" data-reset>复位视角</button></div>
    <div class="run-canvas" aria-label="${task?'桌面分拣回放':'机械臂三维网格'}"></div>
    <div class="card-body"><div class="run-status" role="status">正在读取本地结果…</div>
    <div class="actions-row" data-controls hidden>${task?`
      <button class="button primary" data-play>播放</button><label>轮次 <select data-round></select></label>
      <label>速度 <select data-speed><option value="1">1×</option><option value="2" selected>2×</option><option value="4">4×</option></select></label>
      <input aria-label="仿真时间" type="range" min="0" step="1" value="0" data-seek><output data-clock></output>`:
      '<label>姿态 <select data-pose><option value="0">初始姿态</option><option value="1">几何零位</option></select></label>'}
    </div><small>拖动旋转 · 滚轮缩放 · 右键拖动平移</small>
    <div class="actions-row" data-links></div><div data-evidence></div></div></section>`;
}

export function closeViewer() {
  disposeActive?.();
  disposeActive = null;
}

export async function mountViewer(task) {
  if(!task)return mountGeneratedModel({createScene,closeViewer,setDispose:value=>{disposeActive=value;}});
  closeViewer();
  const el = document.getElementById('run-view');
  if (!el) return;
  const find = s=>el.querySelector(s);
  const label=document.getElementById('projectLabel');
  if(label)label.textContent=arms[selected];
  const controller = new AbortController();
  let view, animation;
  disposeActive = ()=>{controller.abort();cancelAnimationFrame(animation);view?.dispose();};
  find('[data-arm]').onchange = e=>{
    selected = e.target.value;
    const url = new URL(location.href);url.searchParams.set('arm',selected);history.replaceState(null,'',url);
    el.outerHTML=viewerHTML(task);mountViewer(task);
  };
  function link(label,url) {
    const a = document.createElement('a');
    a.textContent=label;a.href=url;a.target='_blank';a.rel='noopener';a.className='button';
    find('[data-links]').append(a);
  }
  try {
    const base='/demo-assets/'+selected+'/';
    const response = await fetch(base+(task?'sorting':'model')+'.json',{signal:controller.signal});
    if (!response.ok) throw Error('此机型尚无可读取的本地'+(task?'分拣回放':'模型输出'));
    const spec=await response.json();
    if(controller.signal.aborted) return;
    view=createScene(find('.run-canvas'),spec);
    const frames=spec.frames;
    find('[data-controls]').hidden=false;
    find('[data-reset]').onclick=view.reset;
    if(!task) {
      const labels={passed:'通过', needs_review:'待复核', failed:'发现干涉'};
      const pose = index=>{
        view.show(index);
        if(spec.collision){
          const status=spec.collision.poses[index?'geometric_zero':'initial'];
          find('[data-evidence]').textContent='所选姿态的零件检查：'+(labels[status]??status)+'。扩展关节范围采样：'+(labels[spec.collision.overall]??spec.collision.overall)+'。';
        }
      };
      find('[data-pose]').onchange=e=>pose(Number(e.target.value));
      pose(0);
      find('.run-status').textContent=arms[selected]+' · '+spec.joints.length+' 个关节 · 实际导出模型';
      link('模型检查报告',base+'validation.json');
      link('初始姿态图片',base+'preview.png');
      return;
    }
    const verdict=spec.result.status==='passed'?'通过':'未通过';
    const rounds=[...new Set(frames.map(f=>f.cycle))];
    const round=find('[data-round]'),seek=find('[data-seek]'),play=find('[data-play]');
    for (const n of rounds) round.add(new Option('第 '+n+' 轮',n));
    seek.max=frames.length-1;
    find('.run-status').textContent=arms[selected]+' · 分拣验收'+verdict+' · 已完成 '+(spec.result.evaluation?.cycles_completed??0)+' 轮';
    link('逐轮验收结果',base+'result.json');
    const evidence=document.createElement('table');
    const heading=evidence.createTHead().insertRow();
    for(const text of ['轮次','红色进入左盘','蓝色进入右盘','验收']){
      const cell=document.createElement('th');cell.textContent=text;heading.append(cell);
    }
    const body=evidence.createTBody();
    for(const cycle of spec.result.evaluation?.per_cycle??[]){
      const row=body.insertRow();
      const placed=(color,tray)=>(cycle.final_positions??[]).filter(o=>o.color===color&&o.tray===tray&&o.placed).length;
      for(const value of [cycle.cycle,placed('red','left')+' / 2',placed('blue','right')+' / 2',cycle.success?'通过':'未通过']){
        row.insertCell().textContent=value;
      }
    }
    find('[data-evidence]').append(evidence);
    let i=0,playing=false,previous=0,elapsed=0;
    function show(n) {
      i=n;view.show(i);seek.value=i;round.value=frames[i].cycle;
      const key=frames[i].phase;
      const phase=Object.keys(phaseLabels).find(p=>key===p||key.startsWith(p+'_'));
      const object=phase && objectLabels[key.slice(phase.length+1)];
      find('[data-clock]').textContent=[frames[i].t.toFixed(1)+' s',phaseLabels[phase]??key,object].filter(Boolean).join(' · ');
    }
    function pause(){playing=false;play.textContent='播放';cancelAnimationFrame(animation);}
    function tick(now){
      elapsed+=Math.min(.25,(now-previous)/1000)*Number(find('[data-speed]').value);
      previous=now;
      let next=i;
      while(next+1<frames.length){
        const gap=Math.max(0,frames[next+1].t-frames[next].t);
        if(gap>elapsed)break;
        elapsed-=gap;next++;
      }
      if(next!==i)show(next);
      if(i===frames.length-1){pause();return;}
      animation=requestAnimationFrame(tick);
    }
    play.onclick=()=>{if(playing){pause();return;}if(i===frames.length-1)show(0);
      playing=true;previous=performance.now();elapsed=0;play.textContent='暂停';animation=requestAnimationFrame(tick);};
    seek.oninput=()=>{elapsed=0;show(Number(seek.value));};
    round.onchange=()=>{elapsed=0;show(frames.findIndex(f=>f.cycle===Number(round.value)));};
    show(0);
    const ready=await fetch(base+'live.json',{signal:controller.signal});
    if(ready.ok){
      const info=await ready.json();
      const u=new URL(info.url);
      if(!controller.signal.aborted && u.protocol==='http:' && ['127.0.0.1','localhost'].includes(u.hostname))link('打开实时 MuJoCo',u.href);
    }
  }catch(error){
    if(!controller.signal.aborted)find('.run-status').textContent='无法显示：'+error.message;
  }
}


export function taskReplayHTML(){return `<section class="card" id="scene-replay"><div class="card-head"><h2>本次实测三维回放</h2><button class="button" data-replay-reset>复位视角</button></div><div class="run-canvas" aria-label="当前任务实测姿态回放"></div><div class="card-body"><p data-replay-status role="status">正在载入实际记录…</p><div class="actions-row"><button class="button primary" data-replay-play disabled>播放</button><input type="range" data-replay-seek min="0" step="1" value="0" aria-label="实测时间游标"><output data-replay-clock></output></div><small>拖动旋转 · 滚轮缩放 · 右键平移。回放基于实际物理采样，不是实时查看器验收。</small></div></section>`;}
export async function mountTaskReplay(url){
  closeViewer();const el=document.getElementById('scene-replay');if(!el)return;
  const find=selector=>el.querySelector(selector),controller=new AbortController();let view,animation,index=0,playing=false,previous=0,elapsed=0;
  disposeActive=()=>{controller.abort();cancelAnimationFrame(animation);view?.dispose();};
  try{
    const response=await fetch(url,{signal:controller.signal});if(!response.ok)throw Error('无法读取本次回放');
    const spec=await response.json();if(controller.signal.aborted)return;
    if(!spec.frames?.length)throw Error('本次未记录到可回放姿态');
    view=createScene(find('.run-canvas'),spec);const frames=spec.frames,play=find('[data-replay-play]'),seek=find('[data-replay-seek]');
    seek.max=frames.length-1;play.disabled=false;
    find('[data-replay-reset]').onclick=view.reset;
    function show(i){index=i;view.show(i);seek.value=i;find('[data-replay-clock]').textContent=`${frames[i].t.toFixed(2)} s · 第 ${frames[i].cycle} 轮 · ${frames[i].phase}`;}
    function pause(){playing=false;play.textContent='播放';cancelAnimationFrame(animation);}
    function tick(now){elapsed+=(now-previous)/1000;previous=now;let next=index;while(next+1<frames.length&&frames[next+1].t-frames[next].t<=elapsed){elapsed-=Math.max(0,frames[next+1].t-frames[next].t);next++;}if(next!==index)show(next);if(index===frames.length-1){pause();return;}animation=requestAnimationFrame(tick);}
    play.onclick=()=>{if(playing){pause();return;}if(index===frames.length-1)show(0);playing=true;elapsed=0;previous=performance.now();play.textContent='暂停';animation=requestAnimationFrame(tick);};
    seek.oninput=()=>{pause();show(Number(seek.value));};
    find('[data-replay-status]').textContent=`${frames.length} 个实际采样姿态 · ${spec.result?.status==='passed'?'任务通过':'任务未通过或未完成，保留实际记录'}`;show(0);
  }catch(error){if(!controller.signal.aborted)find('[data-replay-status]').textContent=error.message;}
}
