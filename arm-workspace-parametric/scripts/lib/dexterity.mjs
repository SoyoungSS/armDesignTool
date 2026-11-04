function sampleRotation(random) {
  // A uniform unit quaternion induces a uniform rotation in SO(3).
  const u=random(), a=2*Math.PI*random(), b=2*Math.PI*random();
  const x=Math.sqrt(1-u)*Math.sin(a), y=Math.sqrt(1-u)*Math.cos(a);
  const z=Math.sqrt(u)*Math.sin(b), w=Math.sqrt(u)*Math.cos(b);
  return {quaternion:[w,x,y,z],matrix:[
    [1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
    [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
    [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]
  ]};
}

export function analyzeDexterity({options,cells,world,orientationRandom,solve}) {
  const orientations=Array.from({length:options.orientations_per_voxel},()=>sampleRotation(orientationRandom));
  const cached=new Map();
  const points=cells.map(({index,position},id)=>{
    const neighbors=index.map((_,axis)=>cached.get(index.map((v,k)=>v-(axis===k?1:0)).join(',')));
    const seeds=[], successfulOrientations=[];
    let example=null;
    for(const [k,rotation] of orientations.entries()){
      const seed=neighbors.find(neighbor=>neighbor?.[k])?.[k];
      const solution=solve(position,rotation.matrix,seed);
      if(!solution.success)continue;
      seeds[k]=solution.q;
      successfulOrientations.push(k);
      example??={orientation_index:k,...solution};
    }
    const successes=successfulOrientations.length;
    if(successes)cached.set(index.join(','),seeds);
    return {id,voxel_index:index,position_base_m:position,position_world_m:world(position),
      orientation_total:orientations.length,orientation_tested:orientations.length,orientation_successes:successes,
      successful_orientation_indices:successfulOrientations,reachability:successes/orientations.length,
      q:example?.q??null,solutions:example?[example]:[],
      spatial_status:successes?'reachable':'unresolved',
      category:successes===orientations.length?'sampled_dexterous':successes?'partially_dexterous':'unresolved'};
  });
  return {points,calls:points.length*orientations.length,detail:{voxel_size_m:options.voxel_size_m,
    orientations_wxyz:orientations.map(r=>r.quaternion),time_budget_s:null,
    completed_voxels:points.length,incomplete_voxels:0,completed:true}};
}
