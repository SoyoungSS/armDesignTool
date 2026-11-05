#!/usr/bin/env node
// Build a relocatable bundle: <output>/<skill>/ plus <output>/shared/.
import {cp,mkdir,realpath} from 'node:fs/promises';
import {resolve,sep} from 'node:path';
import {fileURLToPath} from 'node:url';
const [name,destination]=process.argv.slice(2),root=await realpath(fileURLToPath(new URL('../../',import.meta.url)));
if(!['arm-statics-parametric','arm-workspace-parametric'].includes(name)||!destination)throw Error('Usage: node package-skill.mjs <skill-name> <new-output-directory>');
const output=resolve(destination);
if(output===root||output.startsWith(root+sep))throw Error('Output must be outside source repository');
await mkdir(output); // Refuse to overwrite an existing bundle.
await cp(resolve(root,name),resolve(output,name),{recursive:true});
for(const dir of ['contracts','model','kinematics'])await cp(resolve(root,'shared',dir),resolve(output,'shared',dir),{recursive:true});
process.stdout.write(JSON.stringify({ok:true,skill:name,bundle:output,entrypoint:resolve(output,name,'scripts/run.mjs')})+'\n');
