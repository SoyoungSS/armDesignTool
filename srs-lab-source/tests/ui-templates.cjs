// Template and event-handler smoke test with a minimal DOM; not visual/browser QA.
const vm=require('node:vm'),fs=require('node:fs'),assert=require('node:assert/strict');let nodes={};
const noop=()=>{},ctx2=new Proxy({},{get:(t,k)=>t[k]||noop,set:(t,k,v)=>(t[k]=v,true)});
function node(id){let html='';return {id,style:{},className:'',classList:{toggle:noop},disabled:false,textContent:'',onclick:null,value:'',dataset:{},get innerHTML(){return html;},set innerHTML(v){html=v;if(id==='content'){for(const k of Object.keys(nodes))if(!['content','heading','eyebrow','subtitle','projectLabel','run','export','state','message','nav','exports','closeExport'].includes(k))delete nodes[k];for(const m of v.matchAll(/id="([^"]+)"/g))nodes[m[1]]=node(m[1]);}},getBoundingClientRect:()=>({left:0,top:0,width:700,height:520}),getContext:()=>ctx2,setPointerCapture:noop,showModal:noop,close:noop};}
['content','heading','eyebrow','subtitle','projectLabel','run','export','state','message','nav','exports','closeExport'].forEach(k=>nodes[k]=node(k));const sandbox={URLSearchParams,console,structuredClone,Math,Date,JSON,Number,Promise,Infinity,setTimeout,URL,Blob,devicePixelRatio:1,requestAnimationFrame:cb=>cb(),history:{replaceState:noop},location:{hash:''},localStorage:{getItem:()=>null,setItem:noop},document:{querySelector:s=>nodes[s.slice(1)]||null,querySelectorAll:()=>[],body:{contains:()=>true},createElement:()=>({click:noop})},window:{addEventListener:noop,scrollTo:noop,devicePixelRatio:1}};sandbox.globalThis=sandbox;vm.createContext(sandbox);vm.runInContext(fs.readFileSync('dist/engine.js','utf8'),sandbox);vm.runInContext(fs.readFileSync('dist/scene3d.js','utf8'),sandbox);vm.runInContext(fs.readFileSync('dist/worksheet-critical-poses.js','utf8'),sandbox);vm.runInContext(fs.readFileSync('dist/app.js','utf8'),sandbox);
(async()=>{await new Promise(r=>setTimeout(r,20));for(const tab of ['overview','specs','kinematics','workspace','statics','dynamics','sizing','skills','notes']){vm.runInContext(`tab='${tab}';render()`,sandbox);assert.ok(nodes.content.innerHTML.length>500,tab);assert.ok(!nodes.content.innerHTML.includes('NaN'),tab);assert.ok(!nodes.content.innerHTML.includes('undefined'),tab);if(tab==='specs'){assert.match(nodes.content.innerHTML,/id="f-reach"[^>]*value="600"/);assert.match(nodes.content.innerHTML,/id="f-speed"[^>]*value="60"/);assert.match(nodes.content.innerHTML,/id="f-rotorInertia"[^>]*value="0.00002"/);}}vm.runInContext("tab='kinematics';render()",sandbox);
const before=vm.runInContext('q.slice()',sandbox),camBefore=vm.runInContext('sceneState.yaw',sandbox),joint=vm.runInContext('sceneState.hitJoints[3]',sandbox),cv=nodes.scene,event=(x,y)=>({clientX:x,clientY:y,pointerId:1,button:0,preventDefault:noop});cv.onpointerdown(event(joint.x,joint.y));assert.equal(vm.runInContext('sceneState.drag.type',sandbox),'joint');cv.onpointermove(event(joint.x+20,joint.y));cv.onpointerup();const after=vm.runInContext('q.slice()',sandbox);assert.ok(Math.abs(after[3]-before[3])>.1);for(let j=0;j<7;j++)if(j!==3)assert.equal(after[j],before[j]);assert.equal(vm.runInContext('sceneState.yaw',sandbox),camBefore);assert.equal(nodes.q3.value,after[3]*180/Math.PI);assert.equal(nodes.sceneAngle.textContent,(after[3]*180/Math.PI).toFixed(1)+'°');
const beforeCamera=after.slice();cv.onpointerdown(event(2,60));assert.equal(vm.runInContext('sceneState.drag.type',sandbox),'camera');cv.onpointermove(event(32,75));cv.onpointerup();assert.notEqual(vm.runInContext('sceneState.yaw',sandbox),camBefore);assert.equal(JSON.stringify(vm.runInContext('q',sandbox)),JSON.stringify(beforeCamera));
vm.runInContext('sceneState.change(3,999)',sandbox);assert.equal(vm.runInContext('q[3]',sandbox),130*Math.PI/180);vm.runInContext('sceneState.choose(3);sceneState.change(3,.2)',sandbox);
// A drag on the projected rotation ring must change the selected angle via its 3D axis plane.
const ring=vm.runInContext('sceneState.ring.map(p=>({...p}))',sandbox),hits=vm.runInContext('sceneState.hitJoints',sandbox);let k=ring.findIndex((p,i)=>i>2&&i<80&&hits.every(h=>Math.hypot(h.x-p.x,h.y-p.y)>Math.max(15,h.radius+7)+2));assert.ok(k>=0);const ringQ=vm.runInContext('q[3]',sandbox);cv.onpointerdown(event(ring[k].x,ring[k].y));assert.equal(vm.runInContext('sceneState.drag.mode',sandbox),'ring');cv.onpointermove(event(ring[k+6].x,ring[k+6].y));cv.onpointerup();assert.ok(Math.abs(vm.runInContext('q[3]',sandbox)-ringQ-6*2*Math.PI/96)<1e-6);
const zoom=vm.runInContext('sceneState.zoom',sandbox);cv.onwheel({deltaY:-100,preventDefault:noop});assert.ok(vm.runInContext('sceneState.zoom',sandbox)>zoom);
// The spaced 3D display chain must be a rigid serial chain: upstream nodes stay fixed.
const c=vm.runInContext('config',sandbox),q0=Array(7).fill(0),F=sandbox.Arm3DMath.displayFK(c,q0),V=sandbox.Arm3DMath.V;for(let j=0;j<7;j++){assert.ok(V.norm(V.sub(F.nodes[j+1],F.nodes[j]))>0);const pert=q0.slice();pert[j]=.4;const G=sandbox.Arm3DMath.displayFK(c,pert);for(let i=0;i<=j;i++)assert.ok(V.norm(V.sub(F.nodes[i],G.nodes[i]))<1e-10);}
await vm.runInContext("tab='specs';render();switchConfiguration('SRS-7R-YZX')",sandbox);assert.equal(vm.runInContext('config.configuration',sandbox),'SRS-7R-YZX');assert.equal(vm.runInContext('result.input.configuration',sandbox),'SRS-7R-YZX');assert.equal(vm.runInContext('result.engine_version',sandbox),sandbox.SRS.version);assert.ok(nodes.content.innerHTML.includes('类人肩部'));assert.ok(nodes.content.innerHTML.includes('J1 / Y'));assert.ok(nodes.content.innerHTML.includes('J2 / Z'));vm.runInContext("tab='kinematics';render();sceneState.choose(0)",sandbox);assert.ok(nodes.content.innerHTML.includes('J1 · Y 轴'));const newAxis=vm.runInContext('sceneState.f.axes[0]',sandbox);assert.equal(JSON.stringify(newAxis),JSON.stringify([0,1,0]));const g0=vm.runInContext('SRS.gravity(config,q)[0]',sandbox);assert.ok(Math.abs(g0)>1);const newJoint=vm.runInContext('sceneState.hitJoints[0]',sandbox),newCV=nodes.scene;newCV.onpointerdown(event(newJoint.x,newJoint.y));assert.equal(vm.runInContext('sceneState.drag.j',sandbox),0);newCV.onpointermove(event(newJoint.x+15,newJoint.y));newCV.onpointerup();assert.ok(vm.runInContext('q[0]',sandbox)>0);assert.equal(nodes.q0.value,vm.runInContext('q[0]*180/Math.PI',sandbox));await vm.runInContext("switchConfiguration('SRS-7R')",sandbox);assert.equal(vm.runInContext('result.input.configuration',sandbox),'SRS-7R');
await vm.runInContext("tab='specs';render();switchConfiguration('SRS-7R-XYZ')",sandbox);assert.ok(nodes.content.innerHTML.includes('J1 / X'));assert.ok(nodes.content.innerHTML.includes('J2 / Y'));assert.ok(nodes.content.innerHTML.includes('J3 / Z'));assert.ok(nodes.content.innerHTML.includes('data-axis-preset="6"'));assert.ok(nodes.content.innerHTML.includes('data-axis-component="6,2"'));
vm.runInContext('setJointAxis(0,[0,0,-3])',sandbox);assert.equal(vm.runInContext('dirty',sandbox),true);assert.equal(nodes.axisPreset0.value,'-Z');assert.ok(nodes.axisLabel0.textContent.includes('-Z'));await vm.runInContext('calculate()',sandbox);assert.equal(JSON.stringify(vm.runInContext('result.model.axes[0]',sandbox)),JSON.stringify([0,0,-1]));assert.equal(JSON.stringify(vm.runInContext('config.jointAxes[0]',sandbox)),JSON.stringify([0,0,-1]));
vm.runInContext('setJointAxis(0,[0,0,0])',sandbox);await vm.runInContext('calculate()',sandbox);assert.equal(vm.runInContext('dirty',sandbox),true);assert.ok(nodes.message.textContent.includes('非零'));vm.runInContext('setJointAxis(0,[1,1,0])',sandbox);await vm.runInContext('calculate()',sandbox);assert.ok(Math.abs(vm.runInContext('result.model.axes[0][0]',sandbox)-Math.SQRT1_2)<1e-10);vm.runInContext("tab='kinematics';render()",sandbox);const customDisplay=vm.runInContext('sceneState.f.axes[0]',sandbox);assert.ok(Math.abs(customDisplay[0]-Math.SQRT1_2)<1e-10);assert.ok(Math.abs(customDisplay[1]-Math.SQRT1_2)<1e-10);await vm.runInContext("switchConfiguration('SRS-7R-XYZ')",sandbox);assert.equal(vm.runInContext('config.jointAxes',sandbox),null);assert.equal(JSON.stringify(vm.runInContext('result.model.axes[0]',sandbox)),JSON.stringify([1,0,0]));
await vm.runInContext("tab='overview';switchConfiguration('WORKSHEET-7R')",sandbox);assert.equal(vm.runInContext('config.selfMass',sandbox),3.612);for(const t of ['overview','specs','kinematics','workspace','statics','dynamics','sizing','notes']){vm.runInContext(`tab='${t}';render()`,sandbox);assert.ok(!nodes.content.innerHTML.includes('NaN'),t);assert.ok(!nodes.content.innerHTML.includes('undefined'),t);}assert.equal(vm.runInContext('config.linkVectors.length',sandbox),7);
vm.runInContext("tab='statics';render()",sandbox);
assert.ok(nodes.content.innerHTML.includes('独立静力学校核 Skill'));
assert.ok(!nodes.content.innerHTML.includes('采样重力矩包络'));
assert.equal(vm.runInContext('skillCurrent()',sandbox),false);
const fakeView={joints:Array.from({length:7},(_,i)=>({name:'J'+(i+1),maxTorqueNm:i+1,requiredNm:(i+1)*1.2,capacityNm:null,margin:null,criticalPose:{q:Object.fromEntries(Array.from({length:7},(_,k)=>['J'+(k+1),0]))}})),engineVersion:'test-only',inputSha256:'test',runId:'test',executedAt:'test',warnings:[]};
sandbox.window.SkillClient={poseToJointAngles:(pose,names)=>names.map(n=>pose.q[n])};
sandbox.testSkillData={view:fakeView,payload:{result:{joint_names:Array.from({length:7},(_,i)=>'J'+(i+1))},execution:{execution_mode:'skill_cli',elapsed_ms:10}}};
vm.runInContext('skillData=testSkillData;skillRunSignature=skillKey();render()',sandbox);
assert.ok(nodes.content.innerHTML.includes('J7'));
assert.ok(nodes.content.innerHTML.includes('三维临界姿态预览'));
assert.ok(nodes.content.innerHTML.includes('id="criticalPosePick"'));
assert.ok(nodes.content.innerHTML.includes('J7 · 最大 |τ| 7.000 N·m'));
vm.runInContext("selectCriticalPose('J1')",sandbox);
assert.ok(nodes.content.innerHTML.includes('七轴角度'));
assert.ok(nodes.content.innerHTML.includes('id="criticalApply"')); 
assert.equal(vm.runInContext('q[0]',sandbox),0);
assert.equal(vm.runInContext('selectedJoint',sandbox),0);
vm.runInContext('config.payload=4;markDirty()',sandbox);
assert.equal(vm.runInContext('skillCurrent()',sandbox),false);
assert.equal(vm.runInContext('skillData',sandbox),null);
// Installation UI invalidates results and rotates the world-facing camera basis.
await vm.runInContext("tab='specs';calculate()",sandbox);
assert.ok(nodes.content.innerHTML.includes('id="installationPreset"'));
assert.ok(nodes.content.innerHTML.includes('id="mount2"'));
const camera=vm.runInContext('sceneState.camera(config,700,520)',sandbox);
await nodes.installationPreset.onchange({target:{value:'side'}});
assert.ok(Math.abs(vm.runInContext('config.installation.base_orientation_in_world.rpy_rad[1]',sandbox)-Math.PI/2)<1e-12);
assert.equal(vm.runInContext('skillData',sandbox),null);
const rotated=vm.runInContext('sceneState.camera(config,700,520)',sandbox);
assert.ok(Math.abs(camera.right[0]-rotated.right[0])>.1);
vm.runInContext("setInstallationValue('angle',0,NaN)",sandbox);
assert.equal(vm.runInContext('dirty',sandbox),true);
assert.ok(nodes.installationGravity.textContent.includes('无效'));
vm.runInContext("setInstallationValue('angle',0,.2)",sandbox);
await vm.runInContext('calculate()',sandbox);
assert.equal(vm.runInContext('dirty',sandbox),false);
// A numerically successful CLI response must still match the current installation.
vm.runInContext("tab='statics';render()",sandbox);
nodes.skillSource.value='{"test":true}';
sandbox.window.SkillClient.runStaticsSkill=async()=>({result:{joint_names:Array.from({length:7},(_,i)=>'J'+(i+1)),normalized_input:{gravity_m_s2:[123,0,0]}},execution:{execution_mode:'skill_cli'}});
sandbox.window.SkillClient.makeStaticsViewModel=()=>fakeView;
await vm.runInContext('runSkill()',sandbox);
assert.equal(vm.runInContext('skillData',sandbox),null);
assert.ok(vm.runInContext('skillMessage',sandbox).includes('不一致'));
// Editing input while a request runs prevents its completed result being displayed.
nodes.skillSource.value='{"test":true}';
sandbox.window.SkillClient.runStaticsSkill=async()=>{vm.runInContext("skillInput='new input'",sandbox);return {result:{joint_names:Array.from({length:7},(_,i)=>'J'+(i+1))},execution:{execution_mode:'skill_cli'}};};
await vm.runInContext('runSkill()',sandbox);
assert.equal(vm.runInContext('skillData',sandbox),null);
assert.ok(vm.runInContext('skillMessage',sandbox).includes('过期'));
console.log('PASS: original views, scene controls, Skill results and critical joint binding, invalidation after model changes; original historical-snapshot assertions replaced.');})();
