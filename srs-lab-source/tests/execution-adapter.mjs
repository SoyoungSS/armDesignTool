import test from 'node:test';
import assert from 'node:assert/strict';
import {mkdtemp,readFile,rm} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {executeSkill,executionConfiguration,runProcess} from '../server/execution-adapter.mjs';
const bridge=fileURLToPath(new URL('../server/adapters/stdio-bridge-example.mjs',import.meta.url));
const agent={ARM_EXECUTION_MODE:'agent',ARM_AGENT_COMMAND:process.execPath,ARM_AGENT_ARGS:JSON.stringify([bridge])};

test('local and Agent bridge execute identical artifact contracts',async()=>{
  const directory=await mkdtemp(path.join(tmpdir(),'arm-adapter-test-'));
  try{
    for(const env of [{},agent]){
      const output=path.join(directory,'result.json');
      const args=['--input-type=module','-e',"import {writeFile} from 'node:fs/promises';await writeFile(process.argv[1],JSON.stringify({ok:true}));",output];
      await executeSkill({operation:'analysis',skill:'test-skill',command:process.execPath,args,cwd:directory,inputPath:path.join(directory,'input.json'),outputPath:output},env);
      assert.deepEqual(JSON.parse(await readFile(output)),{ok:true});
      await rm(output);
    }
  }finally{await rm(directory,{recursive:true,force:true});}
});
test('invalid adapters, protocol mismatches and Agent failures surface as errors',async()=>{
  assert.throws(()=>executionConfiguration({ARM_EXECUTION_MODE:'other'}));
  assert.throws(()=>executionConfiguration({ARM_EXECUTION_MODE:'agent'}));
  assert.throws(()=>executionConfiguration({...agent,ARM_AGENT_ARGS:'bad'}));
  const task={operation:'analysis',skill:'test',command:process.execPath,args:['-e','process.exit(1)'],cwd:process.cwd(),timeoutMs:1000};
  await assert.rejects(executeSkill(task,agent),/AGENT_EXECUTION_FAILED/);
  for(const output of ['not-json',JSON.stringify({ok:true,protocol:'wrong',task_id:'wrong'})]){
    await assert.rejects(executeSkill(task,{...agent,ARM_AGENT_ARGS:JSON.stringify(['-e',`console.log(${JSON.stringify(output)})`])}),/AGENT_INVALID_RESPONSE/);
  }
});
test('timeout, cancellation and output limits stop execution',async()=>{
  await assert.rejects(runProcess(process.execPath,['-e','setInterval(()=>{},1000)'],{timeoutMs:50}),/SKILL_TIMEOUT/);
  const controller=new AbortController();
  const pending=runProcess(process.execPath,['-e','setInterval(()=>{},1000)'],{signal:controller.signal});
  controller.abort(Error('cancel-test'));
  await assert.rejects(pending,/cancel-test/);
  await assert.rejects(runProcess(process.execPath,['-e',"console.log('x'.repeat(10000))"],{maxBuffer:100}),/SKILL_OUTPUT_TOO_LARGE/);
});
