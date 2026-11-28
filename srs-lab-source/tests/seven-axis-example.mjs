import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {validateDesignModel} from '../../shared/model/design-model.mjs';
import {simulationInput} from '../server/simulation-export.mjs';
const read=async file=>JSON.parse(await readFile(new URL('../../'+file,import.meta.url),'utf8'));
const model=await read('examples/models/seven-axis-sim.json');
const request=design_model=>simulationInput({design_model,robot_name:'seven_axis_example',targets:['urdf','mujoco'],density_kg_m3:2000});

test('seven-axis example preserves its checked geometry through web export and all case poses fit the working limits',async()=>{
  validateDesignModel(model,{require:'inertial_properties'});
  assert.deepEqual(model.joints[5].limits,[.5445,2.6]);
  const output=request(model);
  assert.deepEqual(output.dimensions,model.simulation_dimensions);
  assert.equal(output.dimensions.joints.J1.module_retreat_mm,140);
  assert.equal(output.dimensions.joints.J5.module_retreat_mm,110);
  for(const kind of ['workspace','statics','motion']){
    const input=await read('examples/cases/seven-axis-sim/'+kind+'.json');
    assert.deepEqual(input.design_model,model);
    const poses=kind==='statics'?input.options.poses.map(p=>model.joints.map(j=>p.q[j.name])):kind==='motion'?[input.trajectory.start_q,...input.trajectory.segments.map(s=>s.q_target)]:[];
    for(const q of poses)q.forEach((value,i)=>assert.ok(value>=model.joints[i].limits[0]&&value<=model.joints[i].limits[1],kind+' '+model.joints[i].id));
  }
});

test('geometry dimensions follow stable IDs when export names change and reject invalid references',()=>{
  const changed=structuredClone(model);changed.joints[2].name='第三轴';
  const output=request(changed);
  assert.deepEqual(output.dimensions.joints.Joint_3,model.simulation_dimensions.joints.J3);
  assert.deepEqual(output.dimensions.links.Joint_3_link,model.simulation_dimensions.links.J3_link);
  assert.ok(!output.dimensions.links.J3_link);
  const bad=structuredClone(model);bad.simulation_dimensions.joints.unknown={diameter_mm:30};
  assert.throws(()=>request(bad),/unknown joint ID/);
  const negative=structuredClone(model);negative.simulation_dimensions.links.J3_link.width_mm=-1;
  assert.throws(()=>request(negative),/outside allowed range/);
});
