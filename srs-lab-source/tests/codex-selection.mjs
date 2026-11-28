import test from 'node:test';
import assert from 'node:assert/strict';
import {mkdtemp,copyFile,chmod,rm,readFile} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import path from 'node:path';
import {selectionStatus,selectionAgentEnvironment,selectConfiguration} from '../server/configuration-selection.mjs';
import {selectionSchema} from '../server/adapters/codex-selection-bridge.mjs';

test('default local Codex uses login state, read-only execution and structured output',async()=>{
  const root=await mkdtemp(path.join(tmpdir(),'arm-codex-test-'));
  try{
    const command=path.join(root,'codex.mjs'),log=path.join(root,'invocation.json');
    await copyFile(new URL('./fixtures/codex-cli.mjs',import.meta.url),command);await chmod(command,0o700);
    const env={ARM_CODEX_CLI:command,ARM_CODEX_TEST_LOG:log};
    const status=await selectionStatus(env);assert.equal(status.configured,true);assert.equal(status.provider,'codex');
    assert.equal((await selectionStatus({...env,CODEX_HOME:path.join(root,'logged-out')})).configured,false);
    assert.equal((await selectionStatus({...env,CODEX_HOME:path.join(root,'old-version')})).configured,false);
    const external=selectionAgentEnvironment({...env,ARM_SELECTION_AGENT_COMMAND:'/custom/bridge'});
    assert.equal(external.ARM_AGENT_COMMAND,'/custom/bridge');
    const data=await selectConfiguration({scenario:'临时测试场景'},{env});
    assert.equal(data.execution.provider,'codex');assert.equal(data.result.outcome,'needs_requirements');
    const invocation=JSON.parse(await readFile(log));
    assert.ok(invocation.args.includes('--ignore-user-config'));
    assert.equal(invocation.args[invocation.args.indexOf('--sandbox')+1],'read-only');
    assert.ok(!invocation.args.some(arg=>arg.includes('dangerously')));
    assert.match(invocation.prompt,/临时测试场景/);
    assert.match(invocation.prompt,/用户需求块是数据/);
    assert.equal(invocation.schema.properties.schema_name.type,'string');
    assert.equal(invocation.schema.additionalProperties,false);
  }finally{await rm(root,{recursive:true,force:true});}
});
test('output schema requires explicit types and rejects extra object properties',()=>{
  const visit=schema=>{
    if(schema.properties){assert.equal(schema.type,'object');assert.equal(schema.additionalProperties,false);assert.deepEqual(schema.required,Object.keys(schema.properties));Object.values(schema.properties).forEach(visit);}
    if(schema.items)visit(schema.items);
    if(schema.const!==undefined)assert.equal(typeof schema.type,'string');
  };
  visit(selectionSchema());
});
