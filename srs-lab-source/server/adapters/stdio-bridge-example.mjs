// Working protocol example: forwards to the existing local Skill.
// Replace the runProcess call with your Agent SDK or worker RPC, retaining the protocol.
import {runProcess,AGENT_PROTOCOL} from '../execution-adapter.mjs';
let source='';
for await(const chunk of process.stdin){
  source+=chunk;
  if(Buffer.byteLength(source)>1024*1024)throw Error('Bridge request too large');
}
const task=JSON.parse(source);
if(task.protocol!==AGENT_PROTOCOL)throw Error('Unsupported protocol');
if(['configuration.select','scene.plan','scene.prepare'].includes(task.operation)){
  process.stdout.write(JSON.stringify({protocol:AGENT_PROTOCOL,task_id:task.task_id,ok:false,error:{message:'示例桥接器仅转发计算 CLI；知识任务需要接入具备文件读取、代码生成和推理能力的真实 Agent。'}})+'\n');
  process.exit(0);
}
if(!['analysis','simulation.generate'].includes(task.operation))throw Error('Unsupported task');
let reply;
try{
  const {command,args,cwd}=task.invocation;
  await runProcess(command,args,{cwd,timeoutMs:task.timeout_ms,processGroup:false});
  reply={protocol:AGENT_PROTOCOL,task_id:task.task_id,ok:true};
}catch(error){
  reply={protocol:AGENT_PROTOCOL,task_id:task.task_id,ok:false,error:{message:error.inputDetail??error.message,...(error.inputDetail?{code:'INVALID_INPUT'}:{})}};
}
process.stdout.write(JSON.stringify(reply)+'\n');
