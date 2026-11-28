import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {spawnSync} from 'node:child_process';
import {fileURLToPath} from 'node:url';
import {simulationInput} from '../server/simulation-export.mjs';
import {prepareRequests} from '../server/module-specs.mjs';

const source=JSON.parse(await readFile(new URL('../../examples/models/seven-axis.json',import.meta.url)));
const model=prepareRequests(source).motion.design_model;
model.payload.mass_kg=0;
const limits=Object.fromEntries(model.joints.filter(j=>j.type!=='fixed').map(j=>[j.id,{effort:100,velocity:1}]));
function input(design_model=model,extras={}){
  return simulationInput({design_model,robot_name:'FixedBaseTest',targets:['urdf','mujoco'],limits,...extras});
}
function inspect(request){
  const script=`import json,sys,tempfile
from pathlib import Path
sys.path.insert(0,sys.argv[1])
from arm_sim.model import normalize
from arm_sim.geometry import build_geometry,mass_properties
from arm_sim.exporters import urdf,mjcf
from arm_sim.drives import prepare_drives
raw=json.load(sys.stdin)
n=normalize(raw)
with tempfile.TemporaryDirectory() as tmp:
 g=build_geometry(n,Path(tmp)/'meshes')
 mass_properties(n,g)
 result={'missing':n['missing_physics'],'masses':{k:v['mass_kg'] for k,v in n['mass_properties'].items()}}
 if not n['missing_physics']:
  prepare_drives(n)
  urdf_path=urdf(n,g,Path(tmp)/'urdf',Path(tmp)/'meshes')
  mjcf_path=mjcf(n,g,Path(tmp)/'mujoco',Path(tmp)/'meshes')
  import xml.etree.ElementTree as ET
  robot=ET.parse(urdf_path).getroot()
  assert robot.find("joint[@name='world_fixed']").get('type')=='fixed'
  scene=ET.parse(mjcf_path).getroot()
  base=scene.find("worldbody/body[@name='base_link']")
  assert base is not None and base.find('freejoint') is None and base.find('joint') is None
  result['generated']=['urdf','mujoco']
 print(json.dumps(result))`;
  const run=spawnSync('python3',['-c',script,fileURLToPath(new URL('../../arm-sim-model-generator/scripts',import.meta.url))],{input:JSON.stringify(request),encoding:'utf8',timeout:30000});
  assert.equal(run.status,0,run.stderr);
  return JSON.parse(run.stdout);
}

test('empty fixed base exports without density or base inertia, including when estimation is enabled',()=>{
  for(const extra of [{},{density_kg_m3:2000}]){
    const request=input({...model,base_components:[]},extra);
    assert.equal(request.physics.fixed_base_massless,true);
    assert.deepEqual(request.installation.base_in_world,model.base_in_world);
    const result=inspect(request);
    assert.deepEqual(result.missing,[]);
    assert.equal(result.masses.base_link,undefined);
    assert.equal(Object.keys(result.masses).length,model.joints.length);
    assert.deepEqual(result.generated,['urdf','mujoco']);
  }
});

test('supplied base properties are preserved and moving-body missing data remains required',()=>{
  const component={name:'base_plate',mass_kg:2,com_m:[0,0,0],inertia_kg_m2:[[.01,0,0],[0,.01,0],[0,0,.01]],provenance:{source:'test',assumptions:[]}};
  const supplied=input({...model,base_components:[component]});
  assert.equal(supplied.physics.fixed_base_massless,undefined);
  assert.equal(inspect(supplied).masses.base_link,2);
  const incomplete=structuredClone(model);
  incomplete.joints[0].components=[];
  const result=inspect(input({...incomplete,base_components:[]}));
  assert.ok(result.missing.some(name=>name.includes('mass, COM, inertia')));
  assert.ok(!result.missing.some(name=>name.startsWith('base_link:')));
});
