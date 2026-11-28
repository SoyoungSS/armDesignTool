#!/usr/bin/env node
// Test-only CLI: no model or network calls.
import {readFile,writeFile} from 'node:fs/promises';
const args=process.argv.slice(2);
if(args[0]==='--version'){
  console.log(process.env.CODEX_HOME?.endsWith('old-version')?'codex-cli 0.114.0':'codex-cli 0.172.0');
}else if(args[0]==='login'){
  if(process.env.CODEX_HOME?.endsWith('logged-out'))process.exitCode=1;
  else console.log('Logged in using ChatGPT');
}else{
  let prompt='';for await(const chunk of process.stdin)prompt+=chunk;
  const output=args[args.indexOf('--output-last-message')+1];
  const schema=JSON.parse(await readFile(args[args.indexOf('--output-schema')+1]));
  if(process.env.ARM_CODEX_TEST_LOG)await writeFile(process.env.ARM_CODEX_TEST_LOG,JSON.stringify({args,prompt,schema}));
  const result={schema_name:'arm-configuration-selection',schema_version:'1.0',outcome:'needs_requirements',conclusion:'测试结果：需要补充工具位姿。',
    requirements:[],candidates:[],open_questions:['需要哪些工具姿态？'],validation_plan:['明确任务位姿'],sources_read:['SKILL.md','references/knowledge-index.md','references/decision-contract.md']};
  await writeFile(output,JSON.stringify(result));
  console.log(JSON.stringify(result));
}
