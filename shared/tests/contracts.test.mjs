import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile,mkdtemp,rm} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import {join} from 'node:path';
import {fileURLToPath} from 'node:url';
import {spawnSync} from 'node:child_process';
import {schemas} from '../contracts/schemas.mjs';
import {validateContract} from '../contracts/validate.mjs';
import {validateDesignModel} from '../model/design-model.mjs';
import {toStaticsRequest,toWorkspaceRequest} from '../model/adapters.mjs';
import {analyze as statics,toStandardInput} from '../../arm-statics-parametric/scripts/lib/statics.mjs';
import {analyze as workspace} from '../../arm-workspace-parametric/scripts/lib/workspace.mjs';
import {forward} from '../kinematics/serial.mjs';
const root=new URL('../../',import.meta.url);
const json=async path=>JSON.parse(await readFile(new URL(path,root),'utf8'));
const models=Object.fromEntries(await Promise.all(['single-link','two-link','seven-axis'].map(async name=>[name,await json(`examples/models/${name}.json`)])));
const request=(m,options)=>({schema_name:'arm-analysis-request',schema_version:'1.0',design_model:structuredClone(m),options});
const pose=m=>({poses:[{name:'reference',q:Object.fromEntries(m.joints.filter(j=>j.type!=='fixed').map(j=>[j.name,0]))}]});
const sampling={mode:'joint_sampling',sampling:{method:'random',samples:32,seed:42}};
test('exported JSON Schemas match executable dependency-free definitions',async()=>{
  for(const [name,schema] of Object.entries(schemas)){const copy=JSON.parse(await readFile(new URL(`../contracts/${name}.schema.json`,import.meta.url)));delete copy.$schema;assert.deepEqual(copy,schema);}
});
test('capability gates: geometry-only runs workspace; mass enables statics; missing inertia does not block statics',()=>{
  const m=structuredClone(models['single-link']);delete m.payload;delete m.gravity_world_m_s2;m.joints.forEach(j=>delete j.components);
  assert.equal(validateDesignModel(m).capabilities.geometry,true);assert.equal(workspace(request(m,sampling)).points.length,32);
  assert.throws(()=>statics(request(m,pose(m))),e=>e.code==='INVALID_INPUT'&&e.path.includes('components'));
  const massive=structuredClone(models['single-link']);delete massive.joints[0].components[0].inertia_kg_m2;
  assert.equal(statics(request(massive,pose(massive))).summary.length,1);
  assert.throws(()=>validateDesignModel(massive,{require:'inertial_properties'}),/inertia/);
  assert.equal(validateDesignModel(models['single-link'],{require:'inertial_properties'}).capabilities.inertial_properties,true);
});
test('single/two link analytic holding efforts and FK validate adapter semantics',()=>{
  const a=statics(request(models['single-link'],pose(models['single-link']))),b=statics(request(models['two-link'],pose(models['two-link'])));
  assert.ok(Math.abs(a.poses[0].holding_load[0]-24.525)<1e-10);
  assert.ok(Math.abs(b.poses[0].holding_load[0]-58.86)<1e-10);
  assert.ok(Math.abs(b.poses[0].holding_load[1]-24.525)<1e-10);
  assert.deepEqual(forward(toWorkspaceRequest(request(models['two-link'],sampling)).model,[0,0]).position_m,[2,0,0]);
});
test('all three common parameter models preserve legacy calculation and input provenance',async()=>{
  for(const [name,m] of Object.entries(models)){
    const s=request(m,pose(m)),w=request(m,sampling),legacyS=toStaticsRequest(s),legacyW=toWorkspaceRequest(w),before=structuredClone(s);
    assert.deepEqual(statics(s).summary,statics(legacyS).summary);
    assert.deepEqual(statics(s).poses,statics(toStandardInput(s)).poses);
    assert.deepEqual(workspace(w).points,workspace(legacyW).points);
    assert.deepEqual(statics(s).input_snapshot,s);assert.deepEqual(s,before);
    assert.equal(statics(s).design_model_sha256,workspace(w).design_model_sha256);
    assert.deepEqual(statics(s).design_model_info.joint_ids,m.joints.map(j=>({id:j.id,name:j.name})));
    if(name==='seven-axis'){assert.equal(statics(s).design_model_info.capabilities.inertial_properties,false);assert.ok(statics(s).warnings.some(x=>x.includes('inertia intentionally absent')));}
  }
});
test('installation rotates gravity and world positions once, without changing geometry',()=>{
  const m=structuredClone(models['single-link']);m.base_in_world={xyz_m:[2,3,4],rpy_rad:[0,0,Math.PI/2]};
  assert.ok(Math.abs(statics(request(m,pose(m))).poses[0].holding_load[0])<1e-9);
  const r=workspace(request(m,{mode:'joint_sampling',sampling:{method:'random',samples:1,fixed_joints:{J1:0}}}));
  [2,4,4].forEach((x,i)=>assert.ok(Math.abs(r.points[0].position_world_m[i]-x)<1e-9));
});
test('reject ambiguous, missing, physically invalid and non-JSON model data',()=>{
  const change=fn=>{const m=structuredClone(models['two-link']);fn(m);assert.throws(()=>validateDesignModel(m),e=>e.code==='INVALID_INPUT');};
  change(m=>m.joints[1].id=m.joints[0].id);change(m=>m.joints[0].axis=[0,0,0]);change(m=>m.joints[0].extra=1);
  change(m=>m.joints[0].components[0].inertia_kg_m2=[[1,0,0],[0,1,0],[0,0,3]]);
  change(m=>m.joints[0].components[0].inertia_kg_m2=[[1,2,0],[2,1,0],[0,0,1]]);
  change(m=>m.joints[0].origin.xyz_m[0]=NaN);
  const s=request(models['single-link'],{...pose(models['single-link']),model:{}});assert.throws(()=>toStaticsRequest(s),/model-owned/);
  const noPayload=request(models['single-link'],pose(models['single-link']));delete noPayload.design_model.payload;assert.throws(()=>toStaticsRequest(noPayload),/payload/);
});
test('P0 trajectory/case/trace/manifest contracts enforce IDs, time and missing-value semantics',async()=>{
  validateContract('case',await json('examples/cases/single-joint-hold.json'));
  const provenance={source:'test',assumptions:[]};
  const t={schema_name:'arm-trajectory',schema_version:'1.0',joint_ids:['J1'],joint_types:['revolute'],samples:[{time_s:0,q:[0],dq:[0],ddq:[0],phase_id:'hold'}],provenance};validateContract('trajectory',t);
  assert.throws(()=>validateContract('trajectory',{...t,samples:[...t.samples,...t.samples]}),/increase/);
  const trace={schema_name:'arm-joint-trace',schema_version:'1.0',joint_ids:['J1'],samples:[{time_s:0,joint_id:'J1',phase_id:'hold',q:0,dq:0,ddq:null,effort:null,effort_semantics:'unavailable',effort_unit:'Nm',interface_wrench:null,quality_flags:['missing effort']}],provenance};validateContract('joint-trace',trace);
  assert.throws(()=>validateContract('joint-trace',{...trace,samples:[{...trace.samples[0],effort_semantics:'drive_output'}]}),/unavailable/);
  validateContract('run-manifest',{schema_name:'arm-run-manifest',schema_version:'1.0',run_id:'test',skill:'statics',engine_version:'1',code_revision:'test',source_backend:'cpu',status:'completed',input_sha256:'a'.repeat(64),model_sha256:'b'.repeat(64),assumptions:[],artifacts:[]});
});
test('relocatable bundles run both CLIs outside repository with reports and shared resources',async()=>{
  const temp=await mkdtemp(join(tmpdir(),'arm-bundle-'));
  try{for(const skill of ['arm-statics-parametric','arm-workspace-parametric']){
    const bundle=join(temp,skill),build=spawnSync(process.execPath,[fileURLToPath(new URL('../scripts/package-skill.mjs',import.meta.url)),skill,bundle],{cwd:tmpdir(),encoding:'utf8'});assert.equal(build.status,0,build.stderr);
    const run=spawnSync(process.execPath,[join(bundle,skill,'scripts/run.mjs'),'--input',join(bundle,skill,'assets/examples/design-two-link.json'),'--output',join(bundle,'result.json'),'--report-dir',join(bundle,'reports')],{cwd:tmpdir(),encoding:'utf8'});assert.equal(run.status,0,run.stderr);
    const output=JSON.parse(await readFile(join(bundle,'result.json'),'utf8'));assert.equal(output.joint_names.length,2);assert.ok(output.design_model_sha256);
  }}finally{await rm(temp,{recursive:true,force:true});}
});

test('web installation adapter updates common model without adding legacy root fields',async()=>{
  const temp=await mkdtemp(join(tmpdir(),'arm-install-client-'));
  try{
    const {writeFile}=await import('node:fs/promises');
    await writeFile(join(temp,'statics-client.mjs'),await readFile(new URL('../../srs-lab-source/dist/statics-client.mjs',import.meta.url)));
    await writeFile(join(temp,'skill-input-text.mjs'),await readFile(new URL('../../arm-statics-parametric/scripts/lib/input-text.mjs',import.meta.url)));
    const {applyInstallationToText}=await import((await import('node:url')).pathToFileURL(join(temp,'statics-client.mjs')));
    const input=request(models['single-link'],pose(models['single-link']));input.design_model.base_in_world.xyz_m=[1,2,3];
    const mount={gravity_world_m_s2:[0,0,-9.8],base_orientation_in_world:{rpy_rad:[.2,.3,.4]}};
    const result=JSON.parse(await applyInstallationToText(JSON.stringify(input),mount));
    assert.deepEqual(result.design_model.base_in_world,{xyz_m:[1,2,3],rpy_rad:[.2,.3,.4]});
    assert.deepEqual(result.design_model.gravity_world_m_s2,mount.gravity_world_m_s2);
    assert.equal(result.installation,undefined);assert.equal(statics(result).summary.length,1);
    const legacy=JSON.parse(await applyInstallationToText(JSON.stringify(toStaticsRequest(input)),mount));assert.deepEqual(legacy.installation,mount);
  }finally{await rm(temp,{recursive:true,force:true});}
});
