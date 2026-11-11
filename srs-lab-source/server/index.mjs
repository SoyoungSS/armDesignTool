import http from 'node:http';
import { spawn } from 'node:child_process';
import { mkdtemp, writeFile, readFile, rm } from 'node:fs/promises';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { randomUUID, createHash } from 'node:crypto';
import { prepareRequests } from './module-specs.mjs';
import { combineResults } from '../dist/module-specifications.mjs';
import { hash } from '../../arm-kinematics-dynamics/scripts/lib/analyze.mjs';
import { parseRequestText as parseStaticsText } from '../../arm-statics-parametric/scripts/lib/input-text.mjs';

const PORT = Number(process.env.PORT || 8787);
const WEB_DIR = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../dist');
const SKILL_DIR = process.env.STATICS_SKILL_DIR;
const BUNDLED_STATICS_DIR=path.resolve(WEB_DIR,'../../arm-statics-parametric');
const WORKSPACE_DIR = process.env.WORKSPACE_SKILL_DIR || path.resolve(WEB_DIR,'../../arm-workspace-parametric');
const MOTION_DIR=process.env.MOTION_SKILL_DIR||path.resolve(WEB_DIR,'../../arm-kinematics-dynamics');
let motionActive=0;
let workspaceActive=0;
let moduleSpecsActive=false;
const MAX_INPUT_BYTES = 2 * 1024 * 1024;
const TIMEOUT_MS = Number(process.env.STATICS_TIMEOUT_MS || 120000);
const CORS_ORIGIN = process.env.WEB_ORIGIN || ''; // empty: same origin only

function respond(res, status, value, requestId) {
  res.writeHead(status, {
    'content-type': 'application/json; charset=utf-8',
    'cache-control': 'no-store', 'x-request-id': requestId,
    ...(CORS_ORIGIN ? {'access-control-allow-origin': CORS_ORIGIN, 'vary': 'Origin'} : {})
  });
  res.end(JSON.stringify(value));
}
function readBody(req) {
  return new Promise((resolve, reject) => {
    let text = '', bytes = 0;
    req.on('data', b => { bytes += b.length; if (bytes > MAX_INPUT_BYTES) { reject(new Error('INPUT_TOO_LARGE')); req.destroy(); } else text += b; });
    req.on('end', () => resolve(text)); req.on('error', reject);
  });
}
function execute(skillDir, input, output, reportDir, timeoutMs=TIMEOUT_MS, signal) {
  return new Promise((resolve, reject) => {
    signal?.throwIfAborted();
    // Run fixed CLI arguments without shell interpolation; computation belongs to the selected Skill.
    const args = ['scripts/run.mjs', '--input', input, '--output', output];
    if (reportDir) args.push('--report-dir', reportDir);
    const child = spawn(process.execPath, args, {
      cwd: skillDir, env: {...process.env}, stdio: ['ignore','pipe','pipe']
    });
    let out = '', err = '', timedOut = false;
    const timer = timeoutMs===null?null:setTimeout(() => { timedOut = true; child.kill('SIGKILL'); }, timeoutMs);
    const cancel=()=>child.kill('SIGKILL');
    signal?.addEventListener('abort',cancel,{once:true});
    const cleanup=()=>{clearTimeout(timer);signal?.removeEventListener('abort',cancel);};
    child.stdout.on('data', b => { out = (out + b).slice(-8192); });
    child.stderr.on('data', b => { err = (err + b).slice(-8192); });
    child.on('error', e => { cleanup(); reject(e); });
    child.on('close', code => {
      cleanup();
      if (signal?.aborted) reject(signal.reason);
      else if (timedOut) reject(new Error('SKILL_TIMEOUT'));
      else if (code !== 0) {
        const failure=new Error(`SKILL_FAILED_${code}: ${err || out}`);
        try{const detail=JSON.parse(err.trim());if(detail.error?.code==='INVALID_INPUT')failure.inputDetail=detail.error.message;}catch{}
        reject(failure);
      }
      else resolve();
    });
  });
}
const server = http.createServer(async (req, res) => {
  const requestId = randomUUID(), url = new URL(req.url, `http://${req.headers.host || 'localhost'}`);
  if (req.method === 'GET' && !url.pathname.startsWith('/api/')) {
    // Fixed allowlist; load the same bundled geometry version as the selected workspace Skill.
    if(url.pathname==='/shared/kinematics/serial.mjs'){
      try{const data=await readFile(path.resolve(WORKSPACE_DIR,'../shared/kinematics/serial.mjs'));res.writeHead(200,{'content-type':'text/javascript; charset=utf-8','cache-control':'no-store','x-content-type-options':'nosniff'});res.end(data);}catch{respond(res,503,{ok:false,error:'SHARED_GEOMETRY_UNAVAILABLE'},requestId);}return;
    }
    const motionAssets=new Map([['/motion-worksheet.json','assets/examples/worksheet-motion.json'],['/motion-fr3.json','assets/examples/franka-fr3-motion.json'],['/motion-worksheet-dynamics.json','assets/examples/worksheet-dynamics.json'],['/motion-export.mjs','scripts/lib/export.mjs']]);
    if(motionAssets.has(url.pathname)){
      try{const data=await readFile(path.join(MOTION_DIR,motionAssets.get(url.pathname)));res.writeHead(200,{'content-type':url.pathname.endsWith('.mjs')?'text/javascript; charset=utf-8':'application/json; charset=utf-8','cache-control':'no-store'});res.end(data);}catch{respond(res,503,{ok:false,error:'MOTION_ASSET_UNAVAILABLE'},requestId);}return;
    }
    const workspaceAssets=new Map([['/workspace-kinematics.mjs','scripts/lib/kinematics.mjs'],['/workspace-exports.mjs','scripts/lib/exports.mjs'],['/workspace-input-text.mjs','scripts/lib/input-text.mjs'],['/workspace-example.jsonc','assets/examples/worksheet-workspace.jsonc'],['/workspace-reachability.jsonc','assets/examples/worksheet-reachability.jsonc'],['/workspace-classification.jsonc','assets/examples/worksheet-classification.jsonc'],['/workspace-fr3.json','assets/examples/franka-fr3.workspace.json']]);
    workspaceAssets.set('/workspace-voxel-grid.mjs','scripts/lib/voxel-grid.mjs');
    if(workspaceAssets.has(url.pathname)){
      try{const data=await readFile(path.join(WORKSPACE_DIR,workspaceAssets.get(url.pathname)));res.writeHead(200,{'content-type':url.pathname.endsWith('.mjs')?'text/javascript; charset=utf-8':'application/json; charset=utf-8','cache-control':'no-store','x-content-type-options':'nosniff'});res.end(data);}catch{respond(res,503,{ok:false,error:'WORKSPACE_ASSET_UNAVAILABLE'},requestId);}return;
    }
    if(url.pathname === '/skill-input-text.mjs') {
      try {
        const source = await readFile(path.resolve(SKILL_DIR || path.resolve(WEB_DIR,'../../arm-statics-parametric'),'scripts/lib/input-text.mjs'));
        res.writeHead(200,{'content-type':'text/javascript; charset=utf-8','cache-control':'no-store','x-content-type-options':'nosniff'});res.end(source);
      } catch { respond(res,503,{ok:false,error:'SKILL_PARSER_UNAVAILABLE'},requestId); }
      return;
    }
    const whitelist = new Map([['/','index.html'],['/index.html','index.html'],['/app.js','app.js'],['/engine.js','engine.js'],['/scene3d.js','scene3d.js'],['/worksheet-critical-poses.js','worksheet-critical-poses.js'],['/statics-client.mjs','statics-client.mjs'],['/workspace-client.mjs','workspace-client.mjs'],['/workspace-scene.js','workspace-scene.js'],['/motion-page.mjs','motion-page.mjs'],['/motion-client.mjs','motion-client.mjs'],['/style.css','style.css'],['/worksheet-minimal.jsonc','worksheet-minimal.jsonc'],['/worksheet-minimal.json','worksheet-minimal.json']]);
    whitelist.set('/module-design-page.mjs','module-design-page.mjs');
    whitelist.set('/module-specifications.mjs','module-specifications.mjs');
    const filename = whitelist.get(url.pathname);
    if (!filename) { respond(res,404,{ok:false,error:'NOT_FOUND'},requestId); return; }
    try {
      const data = await readFile(path.join(WEB_DIR,filename));
      const type=filename.endsWith('.html')?'text/html; charset=utf-8':filename.endsWith('.css')?'text/css; charset=utf-8':filename.endsWith('.json')?'application/json':filename.endsWith('.jsonc')?'application/jsonc':'text/javascript; charset=utf-8';
      res.writeHead(200,{'content-type':type,'cache-control':'no-store','x-content-type-options':'nosniff'});res.end(data);
    } catch { respond(res,500,{ok:false,error:'WEB_ASSET_ERROR'},requestId); }return;
  }
  if (req.method === 'OPTIONS' && CORS_ORIGIN && ['/api/statics','/api/workspace','/api/motion','/api/module-specs'].includes(url.pathname)) {
    res.writeHead(204, {'access-control-allow-origin': CORS_ORIGIN, 'access-control-allow-methods': 'POST, OPTIONS', 'access-control-allow-headers': 'content-type', 'vary': 'Origin'}); res.end(); return;
  }
  if(req.method==='POST'&&url.pathname==='/api/module-specs'){
    if((req.headers['content-type']||'').split(';')[0].trim()!=='application/json'){
      respond(res,415,{ok:false,error:'UNSUPPORTED_MEDIA_TYPE'},requestId);return;
    }
    if(moduleSpecsActive){respond(res,503,{ok:false,error:'已有规格表任务在运行'},requestId);return;}
    moduleSpecsActive=true;
    const controller=new AbortController();
    res.once('close',()=>{if(!res.writableEnded)controller.abort();});
    let workDir,stage='model';
    try{
      let input;
      try{input=JSON.parse(await readBody(req));}catch{respond(res,400,{ok:false,error:'请输入有效 JSON'},requestId);return;}
      const requests=prepareRequests(input?.design_model);
      const modelHash=hash(requests.motion.design_model);
      if(input.prepare_only===true){respond(res,200,{ok:true,requests,modelHash},requestId);return;}
      workDir=await mkdtemp(path.join(tmpdir(),'arm-module-specs-'));
      const results={},execution={};
      const directories={statics:SKILL_DIR||BUNDLED_STATICS_DIR,workspace:WORKSPACE_DIR,motion:MOTION_DIR};
      for(const [skill,directory] of Object.entries(directories)){
        stage=skill;
        const inputFile=path.join(workDir,skill+'-request.json');
        const outputFile=path.join(workDir,skill+'-result.json');
        await writeFile(inputFile,JSON.stringify(requests[skill]));
        const started=Date.now();
        await execute(directory,inputFile,outputFile,undefined,TIMEOUT_MS,controller.signal);
        results[skill]=JSON.parse(await readFile(outputFile,'utf8'));
        execution[skill]={request_id:requestId,executed_at:new Date().toISOString(),elapsed_ms:Date.now()-started,
          execution_mode:'skill_cli',input_sha256:results[skill].input_sha256,engine_version:results[skill].engine_version};
      }
      stage='specifications';
      const specifications=combineResults(requests.motion.design_model,modelHash,results);
      respond(res,200,{ok:true,requests,modelHash,results,execution,specifications},requestId);
    }catch(error){
      if(!controller.signal.aborted)respond(res,error.code==='INVALID_INPUT'||error.inputDetail?422:502,
        {ok:false,error:'MODULE_SPECS_FAILED',stage,detail:error.inputDetail??error.message},requestId);
    }finally{
      moduleSpecsActive=false;
      if(workDir)await rm(workDir,{recursive:true,force:true}).catch(()=>{});
    }
    return;
  }
  if(req.method==='GET'&&url.pathname==='/api/motion/health'){
    let configured=false;try{await readFile(path.join(MOTION_DIR,'scripts/run.mjs'));configured=true;}catch{}
    respond(res,configured?200:503,{ok:configured,activeJobs:motionActive,maxConcurrentJobs:2},requestId);return;
  }
  if(req.method==='POST'&&url.pathname==='/api/motion'){
    if((req.headers['content-type']||'').split(';')[0].trim()!=='application/json'){respond(res,415,{ok:false,error:'UNSUPPORTED_MEDIA_TYPE'},requestId);return;}
    if(motionActive>=2){respond(res,503,{ok:false,error:'MOTION_BUSY'},requestId);return;}
    motionActive++;let workDir;const controller=new AbortController();res.once('close',()=>{if(!res.writableEnded)controller.abort();});
    try{
      const raw=await readBody(req);if(!raw.trim())throw Error('EMPTY_INPUT');
      workDir=await mkdtemp(path.join(tmpdir(),'arm-motion-'));const input=path.join(workDir,'request.json'),output=path.join(workDir,'result.json');await writeFile(input,raw);const start=Date.now();
      await execute(MOTION_DIR,input,output,undefined,120000,controller.signal);
      const result=JSON.parse(await readFile(output,'utf8'));
      const valid=result.schema_name==='arm-motion-result'&&Array.isArray(result.samples)
        ||result.schema_name==='arm-random-motion-result'&&Array.isArray(result.trajectories);
      if(!valid||!result.summary)throw Error('INVALID_MOTION_OUTPUT');
      respond(res,200,{ok:true,result,execution:{execution_mode:'skill_cli',skill_name:'arm-kinematics-dynamics',engine_version:result.engine_version,request_id:requestId,elapsed_ms:Date.now()-start,input_sha256:result.input_sha256}},requestId);
    }catch(e){if(!controller.signal.aborted)respond(res,e.inputDetail?422:e.message==='EMPTY_INPUT'?400:502,{ok:false,error:e.inputDetail?'INVALID_INPUT':'MOTION_EXECUTION_FAILED',detail:e.inputDetail??e.message,request_id:requestId},requestId);}
    finally{motionActive--;if(workDir)await rm(workDir,{recursive:true,force:true}).catch(()=>{});}return;
  }
  if(req.method==='GET'&&url.pathname==='/api/workspace/health'){
    let configured=false;try{await readFile(path.join(WORKSPACE_DIR,'scripts/run.mjs'));configured=true;}catch{}
    respond(res,configured?200:503,{ok:configured,skillConfigured:configured,activeJobs:workspaceActive,maxConcurrentJobs:2},requestId);return;
  }
  if(req.method==='POST'&&url.pathname==='/api/workspace'){
    const type=(req.headers['content-type']||'').split(';')[0].trim();
    if(!['application/json','application/jsonc'].includes(type)){respond(res,415,{ok:false,error:'UNSUPPORTED_MEDIA_TYPE'},requestId);return;}
    if(workspaceActive>=2){respond(res,503,{ok:false,error:'WORKSPACE_BUSY',detail:'已有两个工作空间任务在运行。请等待完成，或在运行任务的页面点击「取消计算」。',request_id:requestId},requestId);return;}
    const controller=new AbortController(),signal=controller.signal;
    res.once('close',()=>{if(!res.writableEnded)controller.abort();});
    workspaceActive++;let workDir;
    try{
      const raw=await readBody(req);if(!raw.trim())throw Error('EMPTY_INPUT');
      let mode;
      try{
        const parserURL=pathToFileURL(path.join(WORKSPACE_DIR,'scripts/lib/input-text.mjs'));
        const {parseRequestText}=await import(parserURL.href);
        const parsed=parseRequestText(raw);
        mode=parsed?.schema_name==='arm-analysis-request'?parsed.options?.mode:parsed?.mode;
      }catch{/* Let the CLI report input or configuration errors. */}
      workDir=await mkdtemp(path.join(tmpdir(),'arm-workspace-'));
      const input=path.join(workDir,'request.jsonc'),output=path.join(workDir,'workspace_result.json');
      await writeFile(input,raw,'utf8');const start=Date.now();
      const timeout=mode==='dexterity_voxels'?null:Number(process.env.WORKSPACE_TIMEOUT_MS||295000);
      await execute(WORKSPACE_DIR,input,output,workDir,timeout,signal);
      const result=JSON.parse(await readFile(output,'utf8'));
      if(!Array.isArray(result.points)||!Array.isArray(result.joint_names)||!result.summary||!result.normalized_input?.model)throw Error('INVALID_SKILL_OUTPUT');
      const execution={request_id:requestId,executed_at:new Date().toISOString(),elapsed_ms:Date.now()-start,execution_mode:'skill_cli',skill_name:'arm-workspace-parametric',skill_entrypoint:'scripts/run.mjs',engine_version:result.engine_version,input_sha256:result.input_sha256,raw_input_sha256:createHash('sha256').update(raw).digest('hex')};
      respond(res,200,{ok:true,result,execution},requestId);
    }catch(e){if(!signal.aborted){const isInput=['EMPTY_INPUT','INPUT_TOO_LARGE'].includes(e.message);respond(res,isInput?400:e.message==='SKILL_TIMEOUT'?504:502,{ok:false,error:isInput?e.message:e.message==='SKILL_TIMEOUT'?'WORKSPACE_TIMEOUT':'WORKSPACE_EXECUTION_FAILED',detail:e.inputDetail,request_id:requestId},requestId);console.error(JSON.stringify({event:'workspace_error',request_id:requestId,message:String(e.message).slice(0,500)}));}}
    finally{workspaceActive--;if(workDir)await rm(workDir,{recursive:true,force:true}).catch(()=>{});}
    return;
  }
  if (req.method === 'GET' && url.pathname === '/api/statics/health') {
    respond(res, SKILL_DIR ? 200 : 503, {ok: Boolean(SKILL_DIR), skillConfigured: Boolean(SKILL_DIR)}, requestId); return;
  }
  if (req.method !== 'POST' || url.pathname !== '/api/statics') { respond(res,404,{ok:false,error:'NOT_FOUND'},requestId); return; }
  const type = (req.headers['content-type'] || '').split(';')[0].trim();
  if (!['application/json','application/jsonc'].includes(type)) { respond(res,415,{ok:false,error:'UNSUPPORTED_MEDIA_TYPE'},requestId); return; }
  let workDir;
  try {
    const raw = await readBody(req);
    if (!raw.trim()) throw new Error('EMPTY_INPUT');
    let directory=SKILL_DIR;
    if(!directory){
      try{if(parseStaticsText(raw)?.schema_name==='arm-analysis-request')directory=BUNDLED_STATICS_DIR;}catch{}
    }
    if(!directory){respond(res,503,{ok:false,error:'SKILL_NOT_CONFIGURED'},requestId);return;}
    workDir = await mkdtemp(path.join(tmpdir(), 'arm-statics-'));
    const input = path.join(workDir, type === 'application/jsonc' ? 'request.jsonc' : 'request.json');
    const output = path.join(workDir, 'statics_result.json');
    await writeFile(input, raw, 'utf8');
    const start = Date.now();
    await execute(directory, input, output, workDir);
    const result = JSON.parse(await readFile(output, 'utf8'));
    if (!Array.isArray(result.summary) || !Array.isArray(result.joint_names)) throw new Error('INVALID_SKILL_OUTPUT');
    const meta = {
      request_id: requestId, executed_at: new Date().toISOString(), elapsed_ms: Date.now()-start,
      execution_mode: 'skill_cli', skill_entrypoint: 'scripts/run.mjs',
      raw_input_sha256: createHash('sha256').update(raw).digest('hex'),
      engine_version: result.engine_version ?? null, input_sha256: result.input_sha256 ?? null
    };
    console.info(JSON.stringify({event:'skill_finished',...meta}));
    respond(res,200,{ok:true,result,execution:meta},requestId);
  } catch(e) {
    console.error(JSON.stringify({event:'skill_error',request_id:requestId,message:String(e.message).slice(0,500)}));
    const isInputError = ['EMPTY_INPUT','INPUT_TOO_LARGE'].includes(e.message);
    respond(res,isInputError ? 400 : 502,{ok:false,error:isInputError ? e.message : 'SKILL_EXECUTION_FAILED',request_id:requestId},requestId);
  } finally { if (workDir) await rm(workDir,{recursive:true,force:true}).catch(()=>{}); }
});
server.listen(PORT, () => console.log(`Skill bridge listening on ${PORT}; configured=${Boolean(SKILL_DIR)}`));
