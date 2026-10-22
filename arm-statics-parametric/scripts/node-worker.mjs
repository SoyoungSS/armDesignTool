import {parentPort} from 'node:worker_threads';
import {analyze} from './lib/statics.mjs';
parentPort.on('message',input=>{
  try{parentPort.postMessage({ok:true,result:analyze(input)});}
  catch(e){parentPort.postMessage({ok:false,error:{code:e.code??'ANALYSIS_ERROR',path:e.path??null,message:e.message}});}
});
