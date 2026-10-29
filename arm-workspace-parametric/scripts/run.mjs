#!/usr/bin/env node
import {readFile,writeFile,mkdir,rename,rm} from 'node:fs/promises';
import {resolve,dirname,join} from 'node:path';
import {randomUUID} from 'node:crypto';
import {parseRequestText} from './lib/input-text.mjs';
import {analyze} from './lib/workspace.mjs';
import {pointsCSV,reportMarkdown,projectionSVG} from './lib/exports.mjs';
async function atomic(file,text){await mkdir(dirname(file),{recursive:true});const tmp=file+'.tmp-'+randomUUID();try{await writeFile(tmp,text,'utf8');await rename(tmp,file);}finally{await rm(tmp,{force:true});}}
async function main(){
  const args=process.argv.slice(2),options={};for(let i=0;i<args.length;i++){if(!['--input','--output','--report-dir'].includes(args[i])||!args[i+1]||args[i+1].startsWith('--'))throw Error('Usage: run.mjs --input request.json[c] --output result.json [--report-dir reports]');options[args[i].slice(2)]=args[++i];}
  if(!options.input||!options.output)throw Error('--input and --output required');if(resolve(options.input)===resolve(options.output))throw Error('Input and output must differ');
  const input=parseRequestText(await readFile(options.input,'utf8')),result=analyze(input),dir=options['report-dir'];
  if(dir){await atomic(join(dir,'points.csv'),pointsCSV(result));await atomic(join(dir,'report.md'),reportMarkdown(result));await atomic(join(dir,'request.json'),JSON.stringify(input,null,2)+'\n');for(const plane of ['xy','xz','yz']){await atomic(join(dir,`projection-${plane}.svg`),projectionSVG(result,plane));await atomic(join(dir,`section-${plane}.svg`),projectionSVG(result,plane,true));}}
  await atomic(options.output,JSON.stringify(result)+'\n');process.stdout.write(JSON.stringify({ok:true,engine_version:result.engine_version,input_sha256:result.input_sha256,points:result.points.length,output:resolve(options.output)})+'\n');
}
main().catch(e=>{process.stderr.write(JSON.stringify({ok:false,error:{code:e.code??'EXECUTION_ERROR',path:e.path??null,message:e.message}})+'\n');process.exitCode=1;});
