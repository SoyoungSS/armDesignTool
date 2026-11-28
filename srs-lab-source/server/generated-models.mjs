import {readFile} from 'node:fs/promises';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {createHash} from 'node:crypto';
import {sceneModel,listSceneModels,containedFile,writeJSON} from './scene-storage.mjs';
import {scenePython} from './scene-simulation.mjs';
import {runProcess} from './execution-adapter.mjs';
const converter=fileURLToPath(new URL('./prepare-viewer.py',import.meta.url));
const building=new Map();
const json=file=>readFile(file,'utf8').then(JSON.parse);
export async function generatedModelList(env=process.env){return (await listSceneModels(env)).map(model=>({...model,viewer_url:`/api/generated-models/${model.id}/viewer`,report_url:`/api/generated-models/${model.id}/validation`}));}
export async function generatedModelReport(id,report,env=process.env){
  if(!['validation','manifest'].includes(report))throw Error('Unsupported model report');
  const model=await sceneModel(id,env);return json(await containedFile(model.directory,`report/${report}.json`));
}
export async function generatedModelViewer(id,env=process.env){
  const model=await sceneModel(id,env),scene=await containedFile(model.directory,model.metadata.entry);
  const key=model.directory;
  if(building.has(key))return building.get(key);
  const promise=(async()=>{
    const directory=path.dirname(model.directory),output=path.join(directory,'viewer.json'),cache=path.join(directory,'viewer-cache.json');
    const signature=createHash('sha256').update(await readFile(scene)).update(await readFile(path.join(model.directory,'report/manifest.json'))).digest('hex');
    try{if((await json(cache)).signature===signature){const saved=await json(output);if(saved.frames?.length&&saved.geoms?.length)return saved;}}catch{}
    try{await runProcess(scenePython(env),[converter,'--scene',scene,'--output',output],{cwd:model.directory,timeoutMs:60000,env});}
    catch(error){const detail=String(error.stderr??error.message).split('\n').filter(Boolean).at(-1);throw Error('无法准备模型三维预览：'+String(detail??'请检查服务器 MuJoCo 环境').slice(0,500));}
    const view=await json(output);
    if(!view.frames?.length||!view.geoms?.length)throw Error('模型缺少可显示的几何或姿态。');
    view.model_id=model.metadata.id;view.model_name=model.metadata.name;view.export_status=model.metadata.mujoco_status;
    try{
      const validation=await generatedModelReport(id,'validation',env),review=validation.collision_review;
      if(review)view.collision={overall:review.status,poses:Object.fromEntries(Object.entries(review.named_pose_results??{}).map(([name,evidence])=>[name,evidence.status]))};
    }catch{}
    await writeJSON(output,view);await writeJSON(cache,{signature});return json(output);
  })();
  building.set(key,promise);try{return await promise;}finally{building.delete(key);}
}
