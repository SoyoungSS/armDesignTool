import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile,mkdtemp,writeFile,rm} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import {join} from 'node:path';
import {pathToFileURL,fileURLToPath} from 'node:url';
import {spawn} from 'node:child_process';
import {once} from 'node:events';
import {createRequire} from 'node:module';
import vm from 'node:vm';
import {analyze} from '../../arm-workspace-parametric/scripts/lib/workspace.mjs';
import {forward} from '../../arm-workspace-parametric/scripts/lib/kinematics.mjs';
const E=createRequire(import.meta.url)('../dist/engine.js');
const close=(a,b)=>assert.ok(Math.abs(a-b)<1e-9,`${a} != ${b}`);
const root=new URL('../../arm-workspace-parametric/',import.meta.url);
const dir=await mkdtemp(join(tmpdir(),'workspace-client-test-'));
for(const name of ['kinematics','exports','input-text','voxel-grid'])await writeFile(join(dir,`workspace-${name}.mjs`),await readFile(name==='kinematics'?new URL('../../shared/kinematics/serial.mjs',import.meta.url):new URL(`scripts/lib/${name}.mjs`,root)));
await writeFile(join(dir,'workspace-client.mjs'),await readFile(new URL('../dist/workspace-client.mjs',import.meta.url)));
const client=await import(pathToFileURL(join(dir,'workspace-client.mjs')));

try{
  await test('unified workspace controls expose two entries and accept region or task inputs without changing the model',async()=>{
    const html=client.workspaceModeControls('dexterity_voxels');
    const main=html.match(/<select id="workspaceMode"[^>]*>(.*?)<\/select>/s)[1];
    assert.equal((main.match(/<option /g)??[]).length,2);
    assert.match(main,/空间可达与灵巧分析/);assert.match(html,/区域扫描（默认）/);assert.match(html,/指定任务点/);
    const c=E.defaults('WORKSHEET-7R'),source=client.fromWebConfig(c,E.model(c),E.installationInfo(c));
    const region=client.applyWorkspaceIKControls(client.asWorkspaceMode(source,'dexterity_voxels',{voxel_size_m:.2,orientations_per_voxel:2}),{min:'-0.2, -0.2, 0',max:'0.2, 0.2, 0.4',seed:'12',ik:'{"attempts":1}'});
    assert.deepEqual(region.dexterity.bounds_base_m,{min:[-.2,-.2,0],max:[.2,.2,.4]});assert.equal(region.dexterity.seed,12);assert.deepEqual(region.model,source.model);
    const task=client.applyWorkspaceIKControls(client.asWorkspaceMode(region,'pose_reachability'),{points:'0.3, 0, 0.5\n0.4, 0, 0.5',orientations:'0, 90, 0',seed:'42',ik:'{}'});
    assert.deepEqual(task.reachability.points_m,[[.3,0,.5],[.4,0,.5]]);assert.deepEqual(task.reachability.orientations_rpy_rad,[[0,Math.PI/2,0]]);assert.equal(task.reachability.sphere,undefined);
    assert.match(client.workspaceIKControls(task,'pose_reachability'),/末端朝向 RPY/);
    const positionOnly=client.applyWorkspaceIKControls(task,{points:'0.3, 0, 0.5',orientations:''});assert.equal(positionOnly.reachability.orientations_rpy_rad,undefined);
    assert.throws(()=>client.applyWorkspaceIKControls(region,{min:'0,0,0',max:''}),/同时填写/);
    assert.throws(()=>client.applyWorkspaceIKControls(region,{min:'1,0,0',max:'0,1,1'}),/最小值/);
    assert.throws(()=>client.applyWorkspaceIKControls(task,{points:'0,0',orientations:''}),/三个有限数值/);
  });
  await test('workspace page handlers switch from FK to default region IK and submit task points on the same model',async()=>{
    const nodes={},submitted=[];
    const c=E.defaults('WORKSHEET-7R'),input=client.fromWebConfig(c,E.model(c),E.installationInfo(c));
    const context=vm.createContext({SRS:E,structuredClone,console,AbortController,setInterval,clearInterval,
      localStorage:{getItem:()=>null,setItem(){}},document:{querySelector:id=>nodes[id]??null,querySelectorAll:()=>[]},
      window:{ModuleDesignPage:{record(){}},WorkspaceClient:{...client,runWorkspaceSkill:async raw=>{const request=JSON.parse(raw);submitted.push(request);return {result:analyze(request),execution:{elapsed_ms:1}};}}},fixture:JSON.stringify(input)});
    const source=await readFile(new URL('../dist/app.js',import.meta.url),'utf8');
    vm.runInContext(source.slice(0,source.indexOf("$('#nav').onclick="))+"\nrender=()=>{};message=text=>{globalThis.lastError=text;};workspaceInput=fixture;",context);
    nodes['#workspaceMode']={};nodes['#workspaceIKMode']={};nodes['#workspaceRun']={};
    vm.runInContext('bindWorkspacePage()',context);
    nodes['#workspaceMode'].onchange({target:{value:'ik'}});
    assert.equal(vm.runInContext('workspaceMode',context),'dexterity_voxels');
    nodes['#workspaceSource']={value:vm.runInContext('workspaceInput',context)};
    nodes['#workspaceBoundsMin']={value:'-0.1,-0.1,0'};nodes['#workspaceBoundsMax']={value:'0.1,0.1,0.2'};
    nodes['#workspaceIKOptions']={value:'{"attempts":1,"max_iterations":2}'};nodes['#workspaceIKSeed']={value:'42'};
    vm.runInContext('voxelSettings={voxel_size_m:.2,orientations_per_voxel:2};',context);
    await nodes['#workspaceRun'].onclick();
    assert.equal(submitted.length,1,context.lastError);assert.equal(submitted[0].mode,'dexterity_voxels');assert.deepEqual(submitted[0].model,input.model);
    nodes['#workspaceIKMode'].onchange({target:{value:'pose_reachability'}});
    nodes['#workspaceSource'].value=vm.runInContext('workspaceInput',context);
    nodes['#workspaceTaskPoints']={value:'0.3,0,0.5'};nodes['#workspaceTaskOrientations']={value:''};
    await nodes['#workspaceRun'].onclick();
    assert.equal(submitted.length,2,context.lastError);assert.deepEqual(submitted[1].reachability.points_m,[[.3,0,.5]]);assert.equal(submitted[1].reachability.orientations_rpy_rad,undefined);
    assert.deepEqual(submitted[1].model,input.model);
    assert.match(vm.runInContext('workspaceView()',context),/空间可达与朝向覆盖/);
  });
  await test('common requests retain model and provenance when switching voxel mode',async()=>{
    const input=JSON.parse(await readFile(new URL('../../arm-workspace-parametric/scripts/tests/fixtures/design-single-link.json',import.meta.url),'utf8'));
    const next=client.asDexterityInput(input,{voxel_size_m:.5,orientations_per_voxel:2});
    assert.equal(next.options.mode,'dexterity_voxels');assert.deepEqual(next.design_model,input.design_model);assert.equal(next.options.sampling,undefined);
    assert.ok(client.estimateDexterity(next).centers>0);
  });
  await test('FR3 mode switching preserves complete geometry and resets only analysis options',async()=>{
    const input=JSON.parse(await readFile(new URL('../../arm-workspace-parametric/assets/examples/franka-fr3.workspace.json',import.meta.url),'utf8'));
    for(const mode of ['joint_sampling','pose_reachability','dexterity_voxels']){
      const request=client.asWorkspaceMode(input,mode,{voxel_size_m:.2,orientations_per_voxel:2});
      assert.deepEqual(request.model,input.model);assert.deepEqual(request.metadata,input.metadata);assert.equal(request.mode,mode);
      if(mode==='pose_reachability'){assert.equal(request.sampling,undefined);assert.equal(request.reachability.points_m.length,1);}
      if(mode==='dexterity_voxels'){assert.equal(request.sampling,undefined);assert.equal(request.dexterity.orientations_per_voxel,2);}
    }
  });
  await test('workspace client maps all web templates/custom axes and installation to identical FK; replays q',()=>{
    for(const variant of E.configurations()){
      const c=E.defaults(variant.id);c.installation={gravity_world_m_s2:[0,0,-9.8],base_orientation_in_world:{rpy_rad:[.3,-.4,.8]}};c.samples=100;
      c.jointAxes=E.model(c).axes.map(a=>a.map(x=>-x));
      const input=client.fromWebConfig(c,E.model(c),E.installationInfo(c)),result=analyze(input);
      for(let i=0;i<result.points.length;i+=7){const p=result.points[i],f=E.fk(c,result.joint_names.map(j=>p.q[j]));f.p.forEach((v,k)=>close(v,p.position_base_m[k]));const preview=client.previewPose(result,i);preview.position_base_m.forEach((v,k)=>close(v,p.position_base_m[k]));preview.position_world_m.forEach((v,k)=>close(v,p.position_world_m[k]));}
      assert.equal(client.parseInput('// comment\n'+client.formatInput(input)).mode,'joint_sampling');
    }
  });
  await test('workspace client replays position-only fallback when all requested orientations fail',()=>{
    const c={...E.defaults('WORKSHEET-7R'),samples:100},input=client.fromWebConfig(c,E.model(c),E.installationInfo(c));
    const position=forward(input.model,[0,0,0,0,0,0,0]).position_m;
    delete input.sampling;input.mode='pose_reachability';input.reachability={points_m:[position],orientations_rpy_rad:[[Math.PI,0,0]],ik:{attempts:1,max_iterations:1}};
    const result=analyze(input);assert.equal(result.points[0].spatial_status,'reachable');assert.equal(result.points[0].solutions.length,0);
    const preview=client.previewPose(result,0);preview.position_base_m.forEach((v,k)=>close(v,position[k]));
    assert.equal(client.pointVisible(result.points[0],'spatial'),true);
    assert.equal(client.pointVisible(result.points[0],'sampled_dexterous'),false);
  });
  const script=`import {createServer} from 'node:net';const s=createServer();s.listen(0,'127.0.0.1',()=>{process.env.PORT=String(s.address().port);s.close(async()=>{await import('./server/index.mjs');console.log('URL=http://127.0.0.1:'+process.env.PORT);});});`;
  const env={...process.env};delete env.STATICS_SKILL_DIR;delete env.WORKSPACE_SKILL_DIR;
  const server=spawn(process.execPath,['--input-type=module','-e',script],{cwd:new URL('../',import.meta.url),env,stdio:['ignore','pipe','pipe']});
  try{
    const url=await new Promise((resolve,reject)=>{let text='';const timer=setTimeout(()=>reject(Error('startup timeout')),10000);server.stdout.on('data',b=>{text+=b;const m=text.match(/URL=(http:\/\/\S+)/);if(m){clearTimeout(timer);resolve(m[1]);}});server.once('error',e=>{clearTimeout(timer);reject(e);});});
    await test('workspace server health/assets work independently of statics env configuration',async()=>{
      assert.equal((await fetch(url+'/api/workspace/health')).status,200);
      assert.equal((await fetch(url+'/api/statics/health')).status,503);
      for(const asset of ['/shared/kinematics/serial.mjs','/workspace-client.mjs','/workspace-scene.js','/workspace-kinematics.mjs','/workspace-exports.mjs','/workspace-input-text.mjs','/workspace-voxel-grid.mjs','/workspace-example.jsonc','/workspace-reachability.jsonc','/workspace-classification.jsonc','/workspace-fr3.json'])assert.equal((await fetch(url+asset)).status,200,asset);
      assert.equal((await fetch(url+'/workspace-dexbot-right.json')).status,404);
      assert.equal((await fetch(url+'/workspace-dexbot-left.json')).status,404);
      assert.equal((await fetch(url+'/workspace-../../server/index.mjs')).status,404);
    });
    await test('workspace real CLI result and provenance equal direct analysis for FK and IK',async()=>{
      const c={...E.defaults('WORKSHEET-7R'),samples:100};
      const input=client.fromWebConfig(c,E.model(c),E.installationInfo(c));
      for(const mode of ['joint_sampling','pose_reachability']){
        const request=structuredClone(input);request.mode=mode;
        if(mode==='pose_reachability'){delete request.sampling;request.reachability={points_m:[forward(request.model,[0,0,0,0,0,0,0]).position_m,[10,0,0]],ik:{attempts:2,max_iterations:30}};}
        const response=await fetch(url+'/api/workspace',{method:'POST',headers:{'content-type':'application/jsonc'},body:'/* test */'+JSON.stringify(request)});assert.equal(response.status,200);
        const data=await response.json();assert.deepEqual(data.result,analyze(request));assert.equal(data.execution.execution_mode,'skill_cli');assert.equal(data.execution.skill_name,'arm-workspace-parametric');assert.ok(data.execution.raw_input_sha256);assert.ok(data.execution.request_id);
      }
    });
    await test('workspace errors do not return fallback point clouds',async()=>{
      assert.equal((await fetch(url+'/api/workspace',{method:'POST',body:'{}'})).status,415);
      for(const body of ['{}','/* bad */ {']){const response=await fetch(url+'/api/workspace',{method:'POST',headers:{'content-type':'application/jsonc'},body});assert.equal(response.status,502);const data=await response.json();assert.equal(data.ok,false);assert.equal(data.result,undefined);}
    });
    await test('busy requests identify the occupied slots and aborted requests release them',async()=>{
      const c=E.defaults('WORKSHEET-7R');
      const input=client.fromWebConfig(c,E.model(c),E.installationInfo(c),'dexterity_voxels');
      input.dexterity.voxel_size_m=.05;
      const request=(body,signal)=>fetch(url+'/api/workspace',{method:'POST',headers:{'content-type':'application/json'},body:JSON.stringify(body),signal});
      const controllers=[new AbortController(),new AbortController()];
      const pending=controllers.map(c=>request(input,c.signal).catch(error=>error));
      async function waitForActiveJobs(count){
        const deadline=Date.now()+5000;
        while(Date.now()<deadline){
          const health=await (await fetch(url+'/api/workspace/health')).json();
          if(health.activeJobs===count)return;
          await new Promise(resolve=>setTimeout(resolve,25));
        }
        assert.fail(`Expected ${count} active workspace jobs`);
      }
      try{
        await waitForActiveJobs(2);
        const busy=await request(input);
        assert.equal(busy.status,503);
        const body=await busy.json();
        assert.equal(body.error,'WORKSPACE_BUSY');
        assert.ok(body.request_id);
        controllers[0].abort();
        await waitForActiveJobs(1);
        const fast={...client.fromWebConfig(c,E.model(c),E.installationInfo(c)),sampling:{method:'random',samples:2,seed:42}};
        assert.equal((await request(fast)).status,200);
      }finally{
        for(const c of controllers)c.abort();
        await Promise.all(pending);
        await waitForActiveJobs(0);
      }
    });
  }finally{server.kill();await once(server,'exit');}
}finally{await rm(dir,{recursive:true,force:true});}
