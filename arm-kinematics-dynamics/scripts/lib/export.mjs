export function dynamicsCSV(result){
  const rows=[['time_s','phase_id','joint_id','q','dq','ddq','effort','effort_unit','power_W',
    'Fx_N','Fy_N','Fz_N','Mx_Nm','My_Nm','Mz_Nm','point_x_m','point_y_m','point_z_m',
    'axial_force_N','radial_force_N','bending_moment_Nm']];
  for(const sample of result.samples){
    sample.dynamic_loads.forEach((load,i)=>{
      const wrench=load.interface_wrench;
      rows.push([sample.time_s,sample.phase_id,result.joint_ids[i],sample.q[i],sample.dq[i],sample.ddq[i],
        load.effort,result.joint_types[i]==='revolute'?'Nm':'N',load.effort*sample.dq[i],
        ...wrench.force_N,...wrench.moment_Nm,...wrench.point_m,
        load.axial_force_N,load.radial_force_N,load.bending_moment_Nm]);
    });
  }
  return rows.map(row=>row.map(value=>'"'+String(value).replaceAll('"','""')+'"').join(',')).join('\n')+'\n';
}
