#!/usr/bin/env node
import {readFile,writeFile,mkdir,rename,rm} from 'node:fs/promises';
import {resolve,dirname,join} from 'node:path';
import {randomUUID} from 'node:crypto';
import {invalid} from '../../shared/contracts/validate.mjs';
import {analyze,report,csv} from './lib/analyze.mjs';
import {dynamicsCSV} from './lib/export.mjs';
import {generateRandomTrajectories} from './lib/random-trajectory.mjs';
async function atomic(file,text){await mkdir(dirname(file),{recursive:true});const temp=file+'.tmp-'+randomUUID();try{await writeFile(temp,text);await rename(temp,file);}finally{await rm(temp,{force:true});}}
try{
  const args=process.argv.slice(2),opts={};for(let i=0;i<args.length;i++){if(!['--input','--output','--report-dir'].includes(args[i])||!args[i+1]||args[i+1].startsWith('--'))throw Error('Usage: run.mjs --input request.json --output result.json [--report-dir dir]');opts[args[i].slice(2)]=args[++i];}
  if(!opts.input||!opts.output||resolve(opts.input)===resolve(opts.output))throw Error('Distinct --input and --output required');
  let input;try{input=JSON.parse(await readFile(opts.input,'utf8'));}catch(e){if(e instanceof SyntaxError)invalid('$','invalid JSON');throw e;}
  const random=input?.schema_name==='arm-random-motion-request';
  if(random&&opts['report-dir'])invalid('$','--report-dir requires a motion analysis request');
  const r=random?generateRandomTrajectories(input):analyze(input);
  if(opts['report-dir']&&r.scope.dynamics_computed)await atomic(join(opts['report-dir'],'dynamic-loads.csv'),dynamicsCSV(r));
  if(opts['report-dir']){await atomic(join(opts['report-dir'],'trajectory.csv'),csv(r));await atomic(join(opts['report-dir'],'report.md'),report(r));await atomic(join(opts['report-dir'],'request.json'),JSON.stringify(r.input_snapshot,null,2)+'\n');}
  await atomic(opts.output,JSON.stringify(r,null,2)+'\n');process.stdout.write(JSON.stringify({ok:true,engine_version:r.engine_version,input_sha256:r.input_sha256,summary:r.summary,output:resolve(opts.output)})+'\n');
}catch(e){process.stderr.write(JSON.stringify({ok:false,error:{code:e.code??'EXECUTION_ERROR',path:e.path??null,message:e.message}})+'\n');process.exitCode=1;}
