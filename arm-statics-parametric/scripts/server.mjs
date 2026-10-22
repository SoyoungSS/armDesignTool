#!/usr/bin/env node
import http from 'node:http';
import {parseRequestText} from './lib/input-text.mjs';
import {readFile} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import {resolve,extname,sep} from 'node:path';
import {Worker} from 'node:worker_threads';
const root=fileURLToPath(new URL('../',import.meta.url));
const portArg=process.argv.indexOf('--port');const port=portArg<0?8765:Number(process.argv[portArg+1]);
if(!Number.isInteger(port)||port<0||port>65535)throw Error('invalid --port');
let active=0;
const json=(res,status,data)=>{res.writeHead(status,{'Content-Type':'application/json; charset=utf-8','Cache-Control':'no-store'});res.end(JSON.stringify(data));};
const server=http.createServer(async(req,res)=>{
  const origin=req.headers.origin;
  if(origin&&origin!==`http://127.0.0.1:${server.address().port}`&&origin!==`http://localhost:${server.address().port}`){json(res,403,{ok:false,error:{code:'ORIGIN_DENIED'}});return;}
  if(req.method==='POST'&&req.url==='/api/statics'){
    if(!String(req.headers['content-type']??'').startsWith('application/json')){json(res,415,{ok:false,error:{code:'JSON_REQUIRED'}});return;}
    if(active>=2){json(res,503,{ok:false,error:{code:'BUSY'}});return;}
    let length=0,body='',oversize=false;req.setEncoding('utf8');
    req.on('data',chunk=>{length+=Buffer.byteLength(chunk);if(length>1024*1024){oversize=true;body='';}else if(!oversize)body+=chunk;});
    req.on('end',()=>{
      if(oversize){json(res,413,{ok:false,error:{code:'BODY_TOO_LARGE'}});return;}
      let input;try{input=parseRequestText(body);}catch{json(res,400,{ok:false,error:{code:'INVALID_JSON'}});return;}
      if(active>=2){json(res,503,{ok:false,error:{code:'BUSY'}});return;}
      active++;const worker=new Worker(new URL('./node-worker.mjs',import.meta.url));let done=false;
      const end=(status,data)=>{if(done)return;done=true;clearTimeout(timer);worker.terminate();active--;if(!res.destroyed)json(res,status,data);};
      const timer=setTimeout(()=>end(408,{ok:false,error:{code:'TIMEOUT',message:'Reduce search budget'}}),120000);
      res.on('close',()=>{if(!res.writableEnded)end(499,{ok:false,error:{code:'CANCELLED'}});});
      worker.on('message',m=>end(m.ok?200:422,m));worker.on('error',e=>end(500,{ok:false,error:{code:'WORKER_ERROR',message:e.message}}));worker.on('exit',code=>{if(!done)end(500,{ok:false,error:{code:'WORKER_EXIT',message:String(code)}});});worker.postMessage(input);
    });return;
  }
  if(req.method!=='GET'){json(res,405,{ok:false,error:{code:'METHOD_NOT_ALLOWED'}});return;}
  try{
    const pathname=decodeURIComponent(new URL(req.url,'http://localhost').pathname);
    if(pathname==='/'){res.writeHead(302,{Location:'/assets/web/index.html'});res.end();return;}
    // Preserve existing integration URLs while using standard Skill directories.
    const aliases=[['/web/','/assets/web/'],['/engine/','/scripts/lib/'],['/examples/','/assets/examples/'],['/schemas/','/references/schemas/']];
    let mapped=pathname;
    for(const [prefix,target] of aliases)if(mapped.startsWith(prefix)){mapped=target+mapped.slice(prefix.length);break;}
    const relative=mapped.slice(1),path=resolve(root,relative);
    if(!path.startsWith(resolve(root)+sep)){json(res,403,{ok:false,error:{code:'FORBIDDEN'}});return;}
    const mime={'.html':'text/html','.mjs':'text/javascript','.json':'application/json','.jsonc':'application/jsonc','.md':'text/plain'}[extname(path)]??'application/octet-stream';
    const data=await readFile(path);res.writeHead(200,{'Content-Type':mime+'; charset=utf-8','Cache-Control':'no-store','X-Content-Type-Options':'nosniff'});res.end(data);
  }catch{json(res,404,{ok:false,error:{code:'NOT_FOUND'}});}
});
server.headersTimeout=15000;server.requestTimeout=30000;
server.listen(port,'127.0.0.1',()=>process.stdout.write(JSON.stringify({url:`http://127.0.0.1:${server.address().port}/`,api:'/api/statics'})+'\n'));
