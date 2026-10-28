import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile,writeFile,mkdtemp,readdir,rm} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import {join} from 'node:path';
import {spawn,spawnSync} from 'node:child_process';
import {fileURLToPath} from 'node:url';
import {analyze,toStandardInput,canonicalJSON} from '../lib/statics.mjs';
const root=fileURLToPath(new URL('../../',import.meta.url));
const example=async name=>JSON.parse(await readFile(join(root,'assets','examples',name+'.json'),'utf8'));
const close=(a,b,t=1e-9)=>assert.ok(Math.abs(a-b)<t,`${a} != ${b}`);

test('all bundled examples compute finite outputs and normalize roundtrip',async()=>{
  for(const name of ['simple-two-link','side-mounted-two-link','standard-two-link','seven-axis-concept','project-right-arm','project-left-arm','prismatic']){
    const input=await example(name),r=analyze(input),s=toStandardInput(input),round=analyze(s);
    assert.deepEqual(r.poses,round.poses);assert.deepEqual(r.summary,round.summary);
    assert.deepEqual(s,toStandardInput(s));assert.ok(!JSON.stringify(r).includes('NaN'));
    assert.equal(analyze(input).input_sha256,r.input_sha256);
  }
});
test('project poses agree with independent established calculations',async()=>{
  const expected={
    'project-right-arm':[23.298447735137707,0,-.03348423806273433,10.083138993248095,-.012154680735817727,-2.6713349518499196,0],
    'project-left-arm':[-23.301046159153373,0,.03346659795306163,-10.0842494904139,.012137040804636342,2.6714750376297247,0]
  };
  for(const[name,values]of Object.entries(expected)){const input=await example(name);delete input.search;const r=analyze(input);values.forEach((v,i)=>close(r.poses[0].holding_load[i],v));}
});
test('CLI writes atomic deterministic result and preserves old result on input failure',async()=>{
  const dir=await mkdtemp(join(tmpdir(),'statics-cli-'));
  try{
    const input=join(root,'assets','examples','simple-two-link.json'),output=join(dir,'result.json'),reports=join(dir,'reports');
    const run=()=>spawnSync(process.execPath,[join(root,'scripts','run.mjs'),'--input',input,'--output',output,'--report-dir',reports],{encoding:'utf8'});
    const a=run();assert.equal(a.status,0,a.stderr);const result=await readFile(output,'utf8');assert.deepEqual(JSON.parse(result),analyze(await example('simple-two-link')));
    assert.equal(run().status,0);assert.equal(await readFile(output,'utf8'),result);
    const standard=JSON.parse(await readFile(join(reports,'standard_request.json'),'utf8'));assert.deepEqual(analyze(standard).poses,JSON.parse(result).poses);
    const bad=join(dir,'bad.json');await writeFile(bad,'{"schema_version":"1.0"}');
    const fail=spawnSync(process.execPath,[join(root,'scripts','run.mjs'),'--input',bad,'--output',output],{encoding:'utf8'});assert.notEqual(fail.status,0);assert.equal(JSON.parse(fail.stderr).ok,false);assert.equal(await readFile(output,'utf8'),result);
    assert.ok(!(await readdir(dir)).some(n=>n.includes('.tmp-')));
    assert.notEqual(spawnSync(process.execPath,[join(root,'scripts','run.mjs'),'--input',bad,'--output',bad]).status,0);
  }finally{await rm(dir,{recursive:true,force:true});}
});
test('HTTP Worker interface equals direct engine and rejects invalid requests',async()=>{
  const proc=spawn(process.execPath,[join(root,'scripts','server.mjs'),'--port','0'],{stdio:['ignore','pipe','pipe']});
  try{
    const url=await new Promise((resolve,reject)=>{let text='';const timeout=setTimeout(()=>reject(Error('server startup timeout')),10000);proc.stdout.on('data',chunk=>{text+=chunk;if(text.includes('\n')){clearTimeout(timeout);resolve(JSON.parse(text.split('\n')[0]).url);}});proc.once('error',reject);proc.once('exit',code=>{clearTimeout(timeout);reject(Error('server exited '+code));});});
    const input=await example('side-mounted-two-link');const response=await fetch(url+'api/statics',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(input)});assert.equal(response.status,200);assert.deepEqual((await response.json()).result,analyze(input));
    const commented=await fetch(url+'api/statics',{method:'POST',headers:{'Content-Type':'application/jsonc'},body:'/* annotated */'+JSON.stringify(input)+'// end'});assert.equal(commented.status,200);assert.deepEqual((await commented.json()).result,analyze(input));
    const standard=await fetch(url+'api/statics',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(toStandardInput(input))});assert.deepEqual((await standard.json()).result.poses,analyze(input).poses);
    assert.equal((await fetch(url+'api/statics',{method:'POST',headers:{'Content-Type':'application/json'},body:'{bad'})).status,400);
    assert.equal((await fetch(url+'api/statics',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'})).status,422);
    assert.equal((await fetch(url+'api/statics',{method:'POST',body:'{}'})).status,415);
    assert.equal((await fetch(url,{headers:{Origin:'https://unrelated.example'}})).status,403);
    assert.equal((await fetch(url)).status,200);assert.equal((await fetch(url+'web/worker.mjs')).headers.get('content-type'),'text/javascript; charset=utf-8');
    assert.equal((await fetch(url+'api/statics',{method:'POST',headers:{'Content-Type':'application/json'},body:' '.repeat(1024*1024+1)})).status,413);
  }finally{proc.kill('SIGTERM');}
});
test('large computational requests are rejected before execution',async()=>{
  const input=await example('project-right-arm');input.search={samples:200000,seed:1,elite_count:10,refine_sweeps:40};
  for(const j of input.model.joints)j.components=Array.from({length:100},(_,i)=>({name:'mass'+i,mass_kg:.01,com_m:[0,0,0]}));
  assert.throws(()=>analyze(input),e=>e.code==='INVALID_INPUT'&&e.path==='analysis_budget');
});
test('prototype-sensitive joint names remain data, not object mutation',async()=>{
  const r=await example('prismatic');r.model.joints[0].name='__proto__';r.poses[0].q=JSON.parse('{"__proto__":0.5}');r.attachments=[{name:'extra',frame:'__proto__',mass_kg:1,com_m:[0,0,0]}];const a=analyze(r);close(a.poses[0].holding_load[0],58.86);assert.equal({}.mass_kg,undefined);
});
