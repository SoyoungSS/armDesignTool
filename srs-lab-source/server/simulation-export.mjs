import {execFile} from 'node:child_process';
import {mkdtemp,readFile,readdir,rm,writeFile} from 'node:fs/promises';
import {homedir,tmpdir} from 'node:os';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {promisify} from 'node:util';
import {invalid} from '../../shared/contracts/validate.mjs';
import {validateDesignModel,collectAssumptions} from '../../shared/model/design-model.mjs';

const execute=promisify(execFile);
const skillDir=fileURLToPath(new URL('../../arm-sim-model-generator/',import.meta.url));
const supportedTargets=new Set(['urdf','ros1','mujoco','isaac']);
const validName=/^[A-Za-z][A-Za-z0-9_]{0,63}$/;
const reservedNames=new Set(['tip','tool_fixed','world_fixed','tool_frame','world','base_link']);
const positive=value=>typeof value==='number'&&Number.isFinite(value)&&value>0;

export function simulationInput({design_model:model,robot_name,targets,limits,density_kg_m3}={}){
  validateDesignModel(model);
  if(typeof robot_name!=='string'||!validName.test(robot_name))invalid('robot_name','模型文件名须以英文字母开头，且仅使用英文字母、数字和下划线');
  if(!Array.isArray(targets)||!targets.length||targets.length>4||new Set(targets).size!==targets.length||targets.some(x=>!supportedTargets.has(x))){
    invalid('targets','请选择有效的导出格式');
  }
  if(density_kg_m3!==undefined&&!positive(density_kg_m3))invalid('density_kg_m3','估算密度须为正数');

  const names=new Set(model.joints.map(joint=>joint.name));
  const mapping=Object.create(null);
  const drives={};
  const exportComponents=(components=[])=>components.filter(component=>component.mass_kg>0).map(component=>({
    name:component.name,mass_kg:component.mass_kg,com_m:component.com_m,
    ...(component.inertia_kg_m2?{inertia_kg_m2:component.inertia_kg_m2,inertia_frame:'link'}:{}),
  }));
  const joints=model.joints.map((joint,index)=>{
    let name=joint.name;
    if(!validName.test(name)||reservedNames.has(name)){
      name='Joint_'+(index+1);
      while(names.has(name))name+='_';
      names.add(name);
    }
    mapping[joint.id]=name;
    const {type,origin,axis,limits:range,components}=joint;
    const output={name,type,origin,components:exportComponents(components)};
    if(type!=='fixed'){
      const supplied=Object.hasOwn(limits??{},joint.id)?limits[joint.id]:null;
      const effort=joint.simulation_limits?.effort??supplied?.effort;
      const velocity=joint.simulation_limits?.velocity??supplied?.velocity;
      if(!positive(effort)||!positive(velocity))invalid('limits.'+joint.id,joint.name+' 的输出与速度上限须为正数');
      drives[name]={effort,velocity};
      Object.assign(output,{axis,limits:range});
    }
    return output;
  });
  const payload=model.payload?.mass_kg>0?{
    mass_kg:model.payload.mass_kg,com_in_tip_m:model.payload.com_in_tip_m,separate_mass_confirmed:true,
  }:undefined;
  return {
    schema_version:'1.0',model:{name:robot_name,joints,tip:model.tip},drives,
    installation:{base_in_world:model.base_in_world,gravity_world_m_s2:model.gravity_world_m_s2},
    physics:{payload,components:{base_link:exportComponents(model.base_components)},...(density_kg_m3===undefined?{}:{estimation:{method:'uniform_density',density_kg_m3}})},
    export:{mode:density_kg_m3===undefined?'strict':'concept',targets},
    metadata:{normalization_assumptions:[
      ...collectAssumptions(model),
      'Source web model: '+model.name,
      'Joint ID to exported name: '+JSON.stringify(mapping),
      ...(payload?['The web model payload is an additional tip mass.']:[]),
    ]},
  };
}

function pythonCandidates(){
  const windows=process.platform==='win32';
  const executable=windows?'python.exe':'python3';
  const local=path.resolve(skillDir,'..','.venv-arm-sim',windows?'Scripts':'bin',executable);
  const bundled=path.join(process.env.XDG_CACHE_HOME||path.join(homedir(),'.cache'),
    'codex-runtimes','codex-primary-runtime','dependencies','python',windows?'':'bin',executable);
  return [
    ['configured','服务器指定的 Python',process.env.ARM_SIM_PYTHON],
    ['project','项目虚拟环境 (.venv-arm-sim)',local],
    ['system','系统 Python',executable],
    ['bundled','Codex 自带 Python',bundled],
  ].filter(([, , command])=>command);
}

const dependencyProbe=`
import json, sys
result = dict(python=sys.version.split()[0], numpy=None, usd=False, mujoco=False, ready=False)
try:
    import numpy
    from numpy.lib import NumpyVersion
    result['numpy'] = numpy.__version__
    result['ready'] = sys.version_info >= (3, 10) and NumpyVersion("1.24.0") <= NumpyVersion(numpy.__version__) < "3.0.0"
except Exception:
    pass
if result['ready']:
    try:
        from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Gf, Sdf
        result['usd'] = True
    except Exception:
        pass
    try:
        import mujoco
        result['mujoco'] = True
    except Exception:
        pass
print(json.dumps(result))
`;

async function probePython(command){
  try{
    const {stdout}=await execute(command,['-c',dependencyProbe],{cwd:skillDir,timeout:10000,maxBuffer:64*1024});
    return JSON.parse(stdout);
  }catch{
    return {python:null,numpy:null,usd:false,mujoco:false,ready:false};
  }
}

let pendingEnvironmentCheck;
export async function simulationEnvironments(){
  if(!pendingEnvironmentCheck){
    pendingEnvironmentCheck=Promise.all(pythonCandidates().map(async ([id,label,command])=>({
      id,label,...await probePython(command),
    }))).finally(()=>{pendingEnvironmentCheck=undefined;});
  }
  return pendingEnvironmentCheck;
}

async function simulationPython(selection,targets,runtime){
  const candidates=new Map(pythonCandidates().map(([id,,command])=>[id,command]));
  if(selection!=='auto'&&!candidates.has(selection)){
    invalid('python_environment','请选择服务器提供的 Python 环境。');
  }
  const environments=(await simulationEnvironments()).filter(environment=>environment.ready);
  const needsUsd=targets.includes('isaac');
  const needsMujoco=targets.includes('mujoco')&&runtime;
  const selected=selection==='auto'
    ?environments.find(environment=>(!needsUsd||environment.usd)&&(!needsMujoco||environment.mujoco))
      ??(needsUsd?environments.find(environment=>environment.usd):undefined)??environments[0]
    :environments.find(environment=>environment.id===selection);
  if(!selected){
    invalid('python_environment','所选环境缺少 Python ≥3.10 或兼容的 NumPy。请按导出页说明安装依赖，再重新检查环境。');
  }
  if(needsUsd&&!selected.usd&&targets.length===1){
    invalid('python_environment','所选环境无法加载 pxr。请安装 usd-core 或选择支持 USD 的 Python 环境。');
  }
  return candidates.get(selected.id);
}

export async function exportSimulation(request){
  const input=simulationInput(request);
  const python=await simulationPython(request.python_environment??'auto',input.export.targets,request.runtime===true);
  const work=await mkdtemp(path.join(tmpdir(),'arm-sim-web-'));
  try{
    const source=path.join(work,'input.json'),output=path.join(work,'output');
    await writeFile(source,JSON.stringify(input));
    const args=['scripts/generate.py','generate','--input',source,'--output',output];
    if(request.runtime===true)args.push('--runtime');
    try{
      await execute(python,args,{cwd:skillDir,timeout:180000,maxBuffer:2*1024*1024});
    }catch(error){
      if(error.code==='ENOENT'||String(error.stderr).includes("No module named 'numpy'")){
        throw Error('仿真服务需要 Python 和 NumPy。请配置 ARM_SIM_PYTHON，并安装 Skill 的 requirements.txt。');
      }
      if(error.killed)throw Error('生成超时，请减少导出目标或关闭运行时检查后重试。');
      // A failed target can coexist with a successful export in the same run.
      if(!error.stdout)throw Error('仿真生成进程未返回结果。');
    }
    const attempt=JSON.parse(await readFile(path.join(output,'latest_attempt.json'),'utf8'));
    const targets=attempt.targets??{};
    if(!Object.values(targets).some(item=>item.generated)){
      invalid('export',attempt.error??Object.entries(targets).map(([key,item])=>key+' '+(item.error??item.reason??item.status)).join('；'));
    }
    const run=attempt.run_directory;
    const archive=path.join(work,'model.zip');
    const entries=await readdir(run);
    await execute(python,['-m','zipfile','-c',archive,...entries],{cwd:run,timeout:60000});
    return {
      archive:await readFile(archive),
      filename:input.model.name.toLowerCase()+'-simulation.zip',
      targets:Object.fromEntries(Object.entries(targets).map(([key,item])=>[key,item.status])),
    };
  }finally{
    await rm(work,{recursive:true,force:true});
  }
}
