import test from 'node:test';
import assert from 'node:assert/strict';
import {mkdtemp,mkdir,writeFile,rm} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import path from 'node:path';
import {spawn} from 'node:child_process';
import {once} from 'node:events';

const mockDir=await mkdtemp(path.join(tmpdir(),'statics-cli-test-'));
await mkdir(path.join(mockDir,'scripts'));
await writeFile(path.join(mockDir,'scripts/run.mjs'),`import {readFile,writeFile} from 'node:fs/promises';
const args=process.argv, input=args[args.indexOf('--input')+1], output=args[args.indexOf('--output')+1];
const data=await readFile(input,'utf8'); if(!data.includes('test-request'))process.exit(9);
const result={schema_version:'1.0',engine_version:'mock-1',input_sha256:'from-skill',joint_names:['J1','J2','J3','J4','J5','J6','J7'],summary:Array.from({length:7},(_,i)=>({name:'J'+(i+1),max_abs_checked:10+i,required_output:(10+i)*1.2,critical_pose:{q:Object.fromEntries(Array.from({length:7},(_,j)=>['J'+(j+1),0]))}}))};
await writeFile(output,JSON.stringify(result));`);
const port=19000+Math.floor(Math.random()*3000);
let proc;
async function request(url,opts={}) {return fetch('http://127.0.0.1:'+port+url,opts)}
async function ready(){for(let i=0;i<100;i++){try{const r=await request('/api/statics/health');if(r.ok)return;}catch{}await new Promise(r=>setTimeout(r,40));}throw Error('Server did not start')}
try{
 proc=spawn(process.execPath,['server/index.mjs'],{cwd:process.cwd(),env:{...process.env,PORT:String(port),STATICS_SKILL_DIR:mockDir},stdio:'ignore'});
 await ready();
 await test('V10 original HTML, JS, CSS and JSONC served',async()=>{
   for(const [url,mime] of [['/','text/html'],['/app.js','text/javascript'],['/scene3d.js','text/javascript'],['/style.css','text/css'],['/worksheet-minimal.jsonc','application/jsonc']]) {const r=await request(url);assert.equal(r.status,200,url);assert.match(r.headers.get('content-type'),new RegExp(mime));}
 });
 await test('External CLI called and returns provenance',async()=>{
   const r=await request('/api/statics',{method:'POST',headers:{'content-type':'application/jsonc'},body:'{"name":"test-request"}'});assert.equal(r.status,200);const d=await r.json();assert.equal(d.execution.execution_mode,'skill_cli');assert.equal(d.result.engine_version,'mock-1');assert.equal(d.result.summary.length,7);assert.equal(d.result.input_sha256,'from-skill');
 });
 await test('Bad input does not fall back to old browser statics',async()=>{
   const r=await request('/api/statics',{method:'POST',headers:{'content-type':'application/json'},body:'{}'});assert.equal(r.status,502);const d=await r.json();assert.equal(d.ok,false);
 });
 await test('Rejects unknown asset path',async()=>assert.equal((await request('/../server/index.mjs')).status,404));
}finally{proc?.kill();await rm(mockDir,{recursive:true,force:true});}
