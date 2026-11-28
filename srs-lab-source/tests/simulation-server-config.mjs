import test from 'node:test';
import assert from 'node:assert/strict';
import {mkdtemp,writeFile,readFile,chmod,rm} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {exportSimulation} from '../server/simulation-export.mjs';
import {prepareRequests} from '../server/module-specs.mjs';

test('server owns Python/runtime configuration in local and Agent export modes',async()=>{
  const directory=await mkdtemp(path.join(tmpdir(),'arm-server-config-'));
  const command=path.join(directory,'python-wrapper'),log=path.join(directory,'args.json');
  const variables=['ARM_SIM_PYTHON','ARM_SIM_RUNTIME_CHECKS','ARM_EXECUTION_MODE','ARM_AGENT_COMMAND','ARM_AGENT_ARGS'];
  const previous=Object.fromEntries(variables.map(key=>[key,process.env[key]]));
  try{
    // A deterministic generator stand-in tests server policy, bridge invocation and real ZIP packaging.
    await writeFile(command,`#!/usr/bin/env python3
import json,sys,os
from pathlib import Path
if sys.argv[1]=='-c':
 print(json.dumps(dict(python='3.11.0',numpy='2.0.0',ready=True,usd=True,mujoco=True)))
elif sys.argv[1]=='-m':
 os.execvp('python3',['python3']+sys.argv[1:])
else:
 Path(${JSON.stringify(log)}).write_text(json.dumps(sys.argv[1:]))
 output=Path(sys.argv[sys.argv.index('--output')+1]);run=output/'runs'/'test';run.mkdir(parents=True)
 (run/'model.txt').write_text('test artifact')
 (output/'latest_attempt.json').write_text(json.dumps(dict(run_directory=str(run),targets=dict(urdf=dict(generated=True,status='generated')))))
 print('{}')
`);
    await chmod(command,0o700);
    const raw=JSON.parse(await readFile(new URL('../../examples/models/seven-axis.json',import.meta.url)));
    const model=prepareRequests(raw).motion.design_model;
    const limits=Object.fromEntries(model.joints.filter(j=>j.type!=='fixed').map(j=>[j.id,{effort:100,velocity:1}]));
    process.env.ARM_SIM_PYTHON=command;
    for(const mode of ['local','agent']){
      process.env.ARM_EXECUTION_MODE=mode;
      process.env.ARM_AGENT_COMMAND=process.execPath;
      process.env.ARM_AGENT_ARGS=JSON.stringify([fileURLToPath(new URL('../server/adapters/stdio-bridge-example.mjs',import.meta.url))]);
      for(const runtime of [false,true]){
        process.env.ARM_SIM_RUNTIME_CHECKS=String(runtime);
        const result=await exportSimulation({design_model:model,robot_name:'ServerConfig',targets:['urdf'],limits,
          density_kg_m3:2000,python_environment:'browser-must-not-select',runtime:!runtime});
        assert.equal(result.archive.subarray(0,2).toString(),'PK');
        const args=JSON.parse(await readFile(log));
        assert.equal(args.includes('--runtime'),runtime);
      }
    }
  }finally{
    for(const [key,value] of Object.entries(previous)){if(value===undefined)delete process.env[key];else process.env[key]=value;}
    await rm(directory,{recursive:true,force:true});
  }
});
