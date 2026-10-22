#!/usr/bin/env node
import { readFile, writeFile, mkdir, rename, rm } from 'node:fs/promises';
import { resolve, dirname, join } from 'node:path';
import { randomUUID } from 'node:crypto';
import { parseRequestText } from './lib/input-text.mjs';
import { analyze, summaryCSV, reportMarkdown, toStandardInput } from './lib/statics.mjs';

async function atomic(path,text) {
  await mkdir(dirname(path),{recursive:true});const tmp=path+'.tmp-'+randomUUID();
  try{await writeFile(tmp,text,'utf8');await rename(tmp,path);}finally{await rm(tmp,{force:true});}
}
async function main() {
  const args=process.argv.slice(2);let input,output,reportDir;
  for(let i=0;i<args.length;i++){
    if(args[i]==='--input')input=args[++i];else if(args[i]==='--output')output=args[++i];else if(args[i]==='--report-dir')reportDir=args[++i];
    else throw new Error('Usage: node scripts/run.mjs --input request.json --output result.json [--report-dir output-directory]');
  }
  if(!input||!output)throw new Error('--input and --output are required');
  if(resolve(input)===resolve(output))throw new Error('input and output must differ');
  const data=parseRequestText(await readFile(input,'utf8')),result=analyze(data);
  // All analysis completes before replacing a previously successful result.
  if(reportDir){await atomic(join(reportDir,'joint_summary.csv'),summaryCSV(result));await atomic(join(reportDir,'report.md'),reportMarkdown(result));await atomic(join(reportDir,'normalized_model.json'),JSON.stringify(result.normalized_input.model,null,2)+'\n');await atomic(join(reportDir,'standard_request.json'),JSON.stringify(toStandardInput(data),null,2)+'\n');}
  await atomic(output,JSON.stringify(result,null,2)+'\n');
  process.stdout.write(JSON.stringify({ok:true,engine_version:result.engine_version,input_sha256:result.input_sha256,output:resolve(output),joints:result.summary.length})+'\n');
}
main().catch(e=>{process.stderr.write(JSON.stringify({ok:false,error:{code:e.code==='INVALID_INPUT'?e.code:'EXECUTION_ERROR',path:e.path??null,message:e.message}})+'\n');process.exitCode=1;});
