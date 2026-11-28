import {mkdir,cp,readFile,writeFile,readdir,realpath} from 'node:fs/promises';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {randomUUID} from 'node:crypto';
export const validId=id=>typeof id==='string'&&/^[a-f0-9-]{36}$/.test(id);
export const sceneRoot=(env=process.env)=>path.resolve(env.ARM_SCENE_STORAGE||fileURLToPath(new URL('../../.runtime/scene-simulations',import.meta.url)));
export async function writeJSON(file,data){await mkdir(path.dirname(file),{recursive:true});const temporary=file+'.partial';await writeFile(temporary,JSON.stringify(data,null,2));await import('node:fs/promises').then(fs=>fs.rename(temporary,file));}
export async function containedFile(root,relative){
  if(typeof relative!=='string'||path.isAbsolute(relative)||relative.includes('\\'))throw Error('Invalid artifact path');
  const base=await realpath(root),resolved=await realpath(path.resolve(root,relative));
  const rel=path.relative(base,resolved);if(!rel||rel.startsWith('..')||path.isAbsolute(rel))throw Error('Artifact outside job directory');
  return resolved;
}
export async function registerSceneModel(run,{name},env=process.env){
  const manifest=JSON.parse(await readFile(path.join(run,'report/manifest.json'),'utf8'));
  if(!manifest.targets?.mujoco?.generated)return null;
  await containedFile(run,manifest.targets.mujoco.entry);
  const id=randomUUID(),directory=path.join(sceneRoot(env),'models',id);
  await mkdir(directory,{recursive:true});await cp(run,path.join(directory,'model'),{recursive:true});
  const normalized=JSON.parse(await readFile(path.join(run,'report/normalized_model.json'),'utf8'));
  const metadata={id,name,created_at:new Date().toISOString(),model_grade:normalized.model_grade??'unknown',
    mujoco_status:manifest.targets.mujoco.status,notes:normalized.notes??[],entry:manifest.targets.mujoco.entry};
  await writeJSON(path.join(directory,'metadata.json'),metadata);return metadata;
}
export async function listSceneModels(env=process.env){
  const root=path.join(sceneRoot(env),'models');let entries=[];try{entries=await readdir(root);}catch{return [];}
  const results=[];
  for(const id of entries.filter(validId)){try{results.push(JSON.parse(await readFile(path.join(root,id,'metadata.json'),'utf8')));}catch{}}
  return results.sort((a,b)=>b.created_at.localeCompare(a.created_at));
}
export async function sceneModel(id,env=process.env){
  if(!validId(id))throw Error('请选择已生成的 MuJoCo 模型。');
  const directory=path.join(sceneRoot(env),'models',id),metadata=JSON.parse(await readFile(path.join(directory,'metadata.json'),'utf8'));
  await containedFile(path.join(directory,'model'),metadata.entry);
  return {metadata,directory:path.join(directory,'model')};
}
