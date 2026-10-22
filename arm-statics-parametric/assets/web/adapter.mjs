/* Import this adapter in an existing website. One isolated Worker per request. */
export function analyzeInWorker(input,{timeoutMs=120000,signal}={}) {
  return new Promise((resolve,reject)=>{
    if(signal?.aborted){reject(new Error('Analysis cancelled'));return;}
    const worker=new Worker(new URL('./worker.mjs',import.meta.url),{type:'module'});
    const cleanup=()=>{clearTimeout(timer);signal?.removeEventListener('abort',abort);worker.terminate();};
    const abort=()=>{cleanup();reject(new Error('Analysis cancelled'));};
    const timer=setTimeout(()=>{cleanup();reject(new Error('Analysis timed out; reduce search budget'));},timeoutMs);
    signal?.addEventListener('abort',abort,{once:true});
    worker.onerror=e=>{cleanup();reject(new Error(e.message));};
    worker.onmessage=e=>{cleanup();if(e.data.ok)resolve(e.data.result);else{const err=new Error(e.data.error.message);Object.assign(err,e.data.error);reject(err);}};
    try{worker.postMessage({id:1,input});}catch(e){cleanup();reject(e);}
  });
}
