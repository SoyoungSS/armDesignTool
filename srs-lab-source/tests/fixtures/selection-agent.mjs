// Test-only Agent protocol fixture; it does not perform engineering selection.
import {readFile,writeFile} from 'node:fs/promises';
let source='';for await(const chunk of process.stdin)source+=chunk;
const task=JSON.parse(source);
if(task.protocol!=='arm-skill-execution/1'||task.operation!=='configuration.select'||task.invocation)throw Error('Invalid selection task');
const input=JSON.parse(await readFile(task.artifacts.input_path));
if(input.scenario==='slow')await new Promise(resolve=>setTimeout(resolve,10000));
if(task.context.supplied_sources.length!==3)throw Error('Missing skill context');
const result={schema_name:'arm-configuration-selection',schema_version:'1.0',outcome:'needs_requirements',
  requirements:[{id:'R1',name:'应用场景',value:input.scenario,unit:null,origin:'user_input',kind:'task'}],candidates:[],
  open_questions:['工具与工件各有多重？'],conclusion:'测试协议记录：需要补充动作与载荷信息。',validation_plan:['明确接近、操作、退出的工具位姿。'],
  sources_read:['SKILL.md','references/knowledge-index.md','references/decision-contract.md']};
if(input.scenario==='bad-output')result.schema_name='wrong';
await writeFile(task.artifacts.output_path,JSON.stringify(result));
process.stdout.write(JSON.stringify({protocol:task.protocol,task_id:task.task_id,ok:true}));
