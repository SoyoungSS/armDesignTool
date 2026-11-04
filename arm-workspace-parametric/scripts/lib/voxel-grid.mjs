export function conservativeReachBound(model){
  // Fixed translations retain their lengths under rotation. Prismatic travel adds its maximum magnitude.
  const center=model.joints[0].origin.xyz_m.slice();
  const radius=Math.hypot(...model.tip.xyz_m)+model.joints.reduce((sum,j,i)=>sum+
    (i?Math.hypot(...j.origin.xyz_m):0)+(j.type==='prismatic'?Math.max(...j.limits.map(Math.abs)):0),0);
  return {center_base_m:center,radius_m:radius,method:'triangle_inequality_outer_bound'};
}

export function voxelCenters(bound,size,region,tolerance){
  const radius=bound.radius_m+tolerance+1e-9,center=bound.center_base_m;
  const lower=center.map((v,k)=>Math.ceil(Math.max(v-radius,region?.min[k]??-Infinity)/size-.5-1e-10));
  const upper=center.map((v,k)=>Math.floor(Math.min(v+radius,region?.max[k]??Infinity)/size-.5+1e-10));
  const gridCount=lower.reduce((n,v,k)=>n*Math.max(0,upper[k]-v+1),1);
  if(gridCount>100000)throw Error('grid exceeds 100,000 box cells; increase voxel size or restrict bounds_base_m');
  const cells=[];
  for(let i=lower[0];i<=upper[0];i++)for(let j=lower[1];j<=upper[1];j++)for(let k=lower[2];k<=upper[2];k++){
    const index=[i,j,k],position=index.map(v=>(v+.5)*size);
    if(Math.hypot(...position.map((v,a)=>v-center[a]))<=radius)cells.push({index,position});
  }
  if(!cells.length)throw Error('no voxel centers in the requested region and geometric bound');
  return cells;
}

export function voxelPlan(model,settings){
  const cells=voxelCenters(conservativeReachBound(model),settings.voxel_size_m,settings.bounds_base_m,settings.ik?.position_tolerance_m??.001);
  const checks=cells.length*settings.orientations_per_voxel;
  return {cells,checks,within_limit:cells.length<=50000&&checks<=1000000};
}
