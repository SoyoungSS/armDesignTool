import {spawn} from 'node:child_process';
import {randomUUID} from 'node:crypto';

export const AGENT_PROTOCOL='arm-skill-execution/1';
export function executionConfiguration(env=process.env){
  const mode=env.ARM_EXECUTION_MODE||'local';
  if(!['local','agent'].includes(mode))throw Error('ARM_EXECUTION_MODE must be local or agent');
  let args=[];
  try{args=JSON.parse(env.ARM_AGENT_ARGS||'[]');}catch{throw Error('ARM_AGENT_ARGS must be a JSON string array');}
  if(!Array.isArray(args)||args.some(value=>typeof value!=='string'))throw Error('ARM_AGENT_ARGS must be a JSON string array');
  if(mode==='agent'&&!env.ARM_AGENT_COMMAND)throw Error('Agent mode requires ARM_AGENT_COMMAND');
  return {mode,command:env.ARM_AGENT_COMMAND,args};
}

// Both the local runner and agent bridge receive commands exclusively from server code.
// No shell parsing. A process group lets cancellation stop a bridge and its local children.
export function runProcess(command,args,{cwd,signal,timeoutMs=120000,stdin,maxBuffer=2*1024*1024,processGroup=true,env=process.env}={}){
  return new Promise((resolve,reject)=>{
    signal?.throwIfAborted();
    const grouped=processGroup&&process.platform!=='win32';
    const child=spawn(command,args,{cwd,env:{...process.env,...env},detached:grouped,stdio:['pipe','pipe','pipe']});
    let stdout='',stderr='',failure;
    const kill=()=>{try{if(grouped)process.kill(-child.pid,'SIGKILL');else child.kill('SIGKILL');}catch{}};
    const cancel=()=>{failure=signal.reason??Error('SKILL_CANCELLED');kill();};
    const timer=timeoutMs===null?null:setTimeout(()=>{failure=Error('SKILL_TIMEOUT');kill();},timeoutMs);
    signal?.addEventListener('abort',cancel,{once:true});
    const cleanup=()=>{clearTimeout(timer);signal?.removeEventListener('abort',cancel);};
    const collect=key=>chunk=>{
      if(key==='stdout')stdout+=chunk;else stderr+=chunk;
      if(Buffer.byteLength(stdout)+Buffer.byteLength(stderr)>maxBuffer){failure=Error('SKILL_OUTPUT_TOO_LARGE');kill();}
    };
    child.stdout.on('data',collect('stdout'));child.stderr.on('data',collect('stderr'));
    child.stdin.on('error',()=>{});
    child.on('error',error=>{cleanup();reject(error);});
    child.on('close',code=>{
      cleanup();
      if(failure){reject(failure);return;}
      if(code!==0){
        const error=Error(`SKILL_FAILED_${code}: ${(stderr||stdout).slice(-8192)}`);
        Object.assign(error,{stdout,stderr,code});
        try{const detail=JSON.parse(stderr.trim());if(detail.error?.code==='INVALID_INPUT')error.inputDetail=detail.error.message;}catch{}
        reject(error);return;
      }
      resolve({stdout,stderr});
    });
    child.stdin.end(stdin);
    if(signal?.aborted)cancel();
  });
}

export async function executeSkill({operation,skill,command,args,cwd,inputPath,outputPath,timeoutMs=120000,signal},env=process.env){
  const config=executionConfiguration(env);
  if(config.mode==='local')return runProcess(command,args,{cwd,timeoutMs,signal,env});
  return executeAgentTask({operation,skill,cwd,inputPath,outputPath,timeoutMs,signal,
    invocation:{command,args,cwd}},env);
}

export async function executeAgentTask({operation,skill,cwd,inputPath,outputPath,timeoutMs=120000,signal,invocation,context},env=process.env){
  const config=executionConfiguration({...env,ARM_EXECUTION_MODE:'agent'});
  const task={protocol:AGENT_PROTOCOL,task_id:randomUUID(),operation,skill,timeout_ms:timeoutMs,
    ...(invocation?{invocation}:{}),...(context?{context}:{}),artifacts:{input_path:inputPath,output_path:outputPath}};
  const result=await runProcess(config.command,config.args,{cwd,timeoutMs,signal,stdin:JSON.stringify(task)+'\n',env});
  let reply;
  try{reply=JSON.parse(result.stdout);}catch{throw Error('AGENT_INVALID_RESPONSE: expected a JSON response');}
  if(reply.protocol!==AGENT_PROTOCOL||reply.task_id!==task.task_id)throw Error('AGENT_INVALID_RESPONSE: protocol or task_id mismatch');
  if(reply.ok!==true){
    const error=Error('AGENT_EXECUTION_FAILED: '+String(reply.error?.message??reply.error??'unknown failure').slice(0,2000));
    if(reply.error?.code==='INVALID_INPUT')error.inputDetail=String(reply.error.message??'Invalid input');
    throw error;
  }
  // Generated artifacts must still pass the existing callers' parsing/hash checks.
  return result;
}
