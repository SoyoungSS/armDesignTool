import http from 'node:http';
import { spawn } from 'node:child_process';
import { mkdtemp, writeFile, readFile, rm } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { randomUUID, createHash } from 'node:crypto';

const PORT = Number(process.env.PORT || 8787);
const WEB_DIR = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../dist');
const SKILL_DIR = process.env.STATICS_SKILL_DIR;
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
function execute(skillDir, input, output, reportDir) {
  return new Promise((resolve, reject) => {
    // No shell interpolation. The Skill is the ONLY component permitted to compute statics.
    const child = spawn(process.execPath, ['scripts/run.mjs', '--input', input, '--output', output, '--report-dir', reportDir], {
      cwd: skillDir, env: {...process.env}, stdio: ['ignore','pipe','pipe']
    });
    let out = '', err = '', timedOut = false;
    const timer = setTimeout(() => { timedOut = true; child.kill('SIGKILL'); }, TIMEOUT_MS);
    child.stdout.on('data', b => { out = (out + b).slice(-8192); });
    child.stderr.on('data', b => { err = (err + b).slice(-8192); });
    child.on('error', e => { clearTimeout(timer); reject(e); });
    child.on('close', code => {
      clearTimeout(timer);
      if (timedOut) reject(new Error('SKILL_TIMEOUT'));
      else if (code !== 0) reject(new Error(`SKILL_FAILED_${code}: ${err || out}`));
      else resolve();
    });
  });
}
const server = http.createServer(async (req, res) => {
  const requestId = randomUUID(), url = new URL(req.url, `http://${req.headers.host || 'localhost'}`);
  if (req.method === 'GET' && !url.pathname.startsWith('/api/')) {
    const whitelist = new Map([['/','index.html'],['/index.html','index.html'],['/app.js','app.js'],['/engine.js','engine.js'],['/scene3d.js','scene3d.js'],['/worksheet-critical-poses.js','worksheet-critical-poses.js'],['/statics-client.mjs','statics-client.mjs'],['/style.css','style.css'],['/worksheet-minimal.jsonc','worksheet-minimal.jsonc'],['/worksheet-minimal.json','worksheet-minimal.json']]);
    const filename = whitelist.get(url.pathname);
    if (!filename) { respond(res,404,{ok:false,error:'NOT_FOUND'},requestId); return; }
    try {
      const data = await readFile(path.join(WEB_DIR,filename));
      const type=filename.endsWith('.html')?'text/html; charset=utf-8':filename.endsWith('.css')?'text/css; charset=utf-8':filename.endsWith('.json')?'application/json':filename.endsWith('.jsonc')?'application/jsonc':'text/javascript; charset=utf-8';
      res.writeHead(200,{'content-type':type,'cache-control':'no-store','x-content-type-options':'nosniff'});res.end(data);
    } catch { respond(res,500,{ok:false,error:'WEB_ASSET_ERROR'},requestId); }return;
  }
  if (req.method === 'OPTIONS' && CORS_ORIGIN && url.pathname === '/api/statics') {
    res.writeHead(204, {'access-control-allow-origin': CORS_ORIGIN, 'access-control-allow-methods': 'POST, OPTIONS', 'access-control-allow-headers': 'content-type', 'vary': 'Origin'}); res.end(); return;
  }
  if (req.method === 'GET' && url.pathname === '/api/statics/health') {
    respond(res, SKILL_DIR ? 200 : 503, {ok: Boolean(SKILL_DIR), skillConfigured: Boolean(SKILL_DIR)}, requestId); return;
  }
  if (req.method !== 'POST' || url.pathname !== '/api/statics') { respond(res,404,{ok:false,error:'NOT_FOUND'},requestId); return; }
  if (!SKILL_DIR) { respond(res,503,{ok:false,error:'SKILL_NOT_CONFIGURED'},requestId); return; }
  const type = (req.headers['content-type'] || '').split(';')[0].trim();
  if (!['application/json','application/jsonc'].includes(type)) { respond(res,415,{ok:false,error:'UNSUPPORTED_MEDIA_TYPE'},requestId); return; }
  let workDir;
  try {
    const raw = await readBody(req);
    if (!raw.trim()) throw new Error('EMPTY_INPUT');
    workDir = await mkdtemp(path.join(tmpdir(), 'arm-statics-'));
    const input = path.join(workDir, type === 'application/jsonc' ? 'request.jsonc' : 'request.json');
    const output = path.join(workDir, 'statics_result.json');
    await writeFile(input, raw, 'utf8');
    const start = Date.now();
    await execute(SKILL_DIR, input, output, workDir);
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
