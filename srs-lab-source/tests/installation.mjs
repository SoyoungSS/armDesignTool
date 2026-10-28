import test from 'node:test';
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';
import {readFile} from 'node:fs/promises';
import {spawn} from 'node:child_process';
import {once} from 'node:events';
import {fileURLToPath} from 'node:url';
import {analyze,normalizeInput} from '../../arm-statics-parametric/scripts/lib/statics.mjs';
const require=createRequire(import.meta.url),E=require('../dist/engine.js');
const close=(a,b,t=1e-8)=>assert.ok(Math.abs(a-b)<t,`${a} != ${b}`);
const input=JSON.parse(await readFile(new URL('../../arm-statics-parametric/assets/examples/worksheet-minimal.json',import.meta.url),'utf8'));

test('installation: worksheet web statics and rest dynamics match Skill under arbitrary mount and custom gravity',()=>{
  const q=[.3,-.5,.4,.8,-.4,.3,.2];
  for(const angles of [[0,0,0],[Math.PI,0,0],[0,Math.PI/2,0],[.3,-.8,1.2]]){
    const c=E.defaults('WORKSHEET-7R');c.installation={gravity_world_m_s2:[.3,-.7,-9.8],base_orientation_in_world:{rpy_rad:angles}};
    const r=structuredClone(input);delete r.gravity_m_s2;delete r.search;r.installation=c.installation;r.poses=[{name:'test',q:Object.fromEntries(q.map((v,j)=>['J'+(j+1),v]))}];
    const expected=analyze(r).poses[0].holding_load;
    E.gravity(c,q).forEach((v,j)=>close(v,expected[j]));
    E.inverseDynamics(c,q,Array(7).fill(0),Array(7).fill(0)).forEach((v,j)=>close(v,expected[j]));
    const h=1e-6,g=E.installationInfo(c).gravity_m_s2;
    const potential=q=>E.fk(c,q).bodies.reduce((sum,b)=>sum-b.m*b.p.reduce((s,x,k)=>s+x*g[k],0),0);
    q.forEach((_,j)=>{const a=q.slice(),b=q.slice();a[j]+=h;b[j]-=h;close((potential(a)-potential(b))/(2*h),expected[j],2e-7);});
    // Rigid mounting changes gravity, not the base-frame inertia matrix.
    assert.deepEqual(E.massMatrix(c,q),E.massMatrix(E.defaults('WORKSHEET-7R'),q));
  }
  assert.throws(()=>E.validate({...E.defaults(),installation:{gravity_world_m_s2:[0,0,-9.8],base_orientation_in_world:{rpy_rad:[0,NaN,0]}}}));
});

test('installation: real website CLI bridge preserves installation, parser and deterministic result',async()=>{
  const script=`import {createServer} from 'node:net';const s=createServer();s.listen(0,'127.0.0.1',()=>{process.env.PORT=String(s.address().port);s.close(async()=>{await import('./server/index.mjs');console.log('TEST_URL=http://127.0.0.1:'+process.env.PORT);});});`;
  const server=spawn(process.execPath,['--input-type=module','-e',script],{cwd:new URL('../',import.meta.url),env:{...process.env,STATICS_SKILL_DIR:fileURLToPath(new URL('../../arm-statics-parametric/',import.meta.url))},stdio:['ignore','pipe','pipe']});
  try{
    const url=await new Promise((resolve,reject)=>{let text='';const timer=setTimeout(()=>reject(Error('startup timeout')),10000);server.stdout.on('data',c=>{text+=c;const m=text.match(/TEST_URL=(http:\/\/[^\s]+)/);if(m){clearTimeout(timer);resolve(m[1]);}});server.on('error',reject);});
    const parser=await fetch(url+'/skill-input-text.mjs');assert.equal(parser.status,200);assert.match(await parser.text(),/parseRequestText/);
    const r=structuredClone(input);delete r.gravity_m_s2;r.search={samples:100,seed:42};r.installation={gravity_world_m_s2:[0,0,-9.8],base_orientation_in_world:{rpy_rad:[.3,.8,-.2]}};
    const response=await fetch(url+'/api/statics',{method:'POST',headers:{'content-type':'application/jsonc'},body:'/* mounted arm */\n'+JSON.stringify(r)});
    assert.equal(response.status,200);const data=await response.json();assert.deepEqual(data.result,analyze(r));assert.equal(data.execution.execution_mode,'skill_cli');
    assert.deepEqual(data.result.normalized_input.installation,normalizeInput(r).installation);
  }finally{server.kill();await once(server,'exit');}
});
