import { analyze } from '../../scripts/lib/statics.mjs';
self.onmessage=event=>{
  const {id,input}=event.data;
  try{self.postMessage({id,ok:true,result:analyze(input)});}
  catch(e){self.postMessage({id,ok:false,error:{code:e.code??'ANALYSIS_ERROR',path:e.path??null,message:e.message}});}
};
