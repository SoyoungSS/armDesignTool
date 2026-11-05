import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {spawn} from 'node:child_process';
import {fileURLToPath} from 'node:url';
import {analyze as statics} from '../../arm-statics-parametric/scripts/lib/statics.mjs';
import {analyze as workspace} from '../../arm-workspace-parametric/scripts/lib/workspace.mjs';
const script=`import {createServer} from 'node:net';const s=createServer();s.listen(0,'127.0.0.1',()=>{process.env.PORT=String(s.address().port);s.close(async()=>{await import('./server/index.mjs');console.log('URL=http://127.0.0.1:'+process.env.PORT);});});`;
const server=spawn(process.execPath,['--input-type=module','-e',script],{cwd:new URL('../',import.meta.url),env:{...process.env,STATICS_SKILL_DIR:fileURLToPath(new URL('../../arm-statics-parametric',import.meta.url)),WORKSPACE_SKILL_DIR:fileURLToPath(new URL('../../arm-workspace-parametric',import.meta.url))},stdio:['ignore','pipe','pipe']});
try{
  const url=await new Promise((resolve,reject)=>{let output='';const timer=setTimeout(()=>reject(Error('startup timeout')),10000);server.stdout.on('data',b=>{output+=b;const match=output.match(/URL=(http:\/\/\S+)/);if(match){clearTimeout(timer);resolve(match[1]);}});server.on('error',e=>{clearTimeout(timer);reject(e);});server.on('exit',code=>{clearTimeout(timer);reject(Error('server exit '+code));});});
  await test('common parameter requests reach both real CLIs through unchanged web API',async()=>{
    for(const [skill,endpoint,analyze] of [['arm-statics-parametric','statics',statics],['arm-workspace-parametric','workspace',workspace]]){
      const request=JSON.parse(await readFile(new URL(`../../${skill}/assets/examples/design-two-link.json`,import.meta.url),'utf8'));
      const response=await fetch(url+'/api/'+endpoint,{method:'POST',headers:{'content-type':'application/jsonc'},body:'// common parameter request\n'+JSON.stringify(request)});
      assert.equal(response.status,200);const body=await response.json();assert.deepEqual(body.result,analyze(request));assert.equal(body.execution.execution_mode,'skill_cli');
    }
  });
  await test('shared browser geometry resolves through compatibility re-export and allowlist',async()=>{
    const entry=new URL('/workspace-kinematics.mjs',url),text=await (await fetch(entry)).text();const relative=text.match(/from '([^']+)'/)[1];
    const response=await fetch(new URL(relative,entry));assert.equal(response.status,200);assert.match(await response.text(),/export function forward/);
    assert.equal((await fetch(url+'/shared/contracts/schemas.mjs')).status,404);
    assert.equal((await fetch(url+'/shared/kinematics/other.mjs')).status,404);
  });
}finally{server.kill('SIGTERM');}
