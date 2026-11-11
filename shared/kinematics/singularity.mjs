// Small dense one-sided Jacobi SVD. Input is conventional row-major task Jacobian.
export function singularValues(matrix){
  const rows=matrix.length,cols=matrix[0]?.length??0;
  if(!rows||!cols||matrix.some(r=>r.length!==cols||r.some(x=>!Number.isFinite(x))))throw Error('Invalid finite matrix');
  // Orthogonalize columns of J^T; avoid squaring the condition number via J J^T.
  const a=Array.from({length:cols},(_,i)=>matrix.map(r=>r[i]));
  let converged=false;
  for(let sweep=0;sweep<100;sweep++){
    let changed=false;
    for(let p=0;p<rows;p++)for(let q=p+1;q<rows;q++){
      let aa=0,bb=0,ab=0;for(const r of a){aa+=r[p]**2;bb+=r[q]**2;ab+=r[p]*r[q];}
      if(aa===0||bb===0||Math.abs(ab)<=1e-14*Math.sqrt(aa*bb))continue;
      const z=(bb-aa)/(2*ab),t=(z>=0?1:-1)/(Math.abs(z)+Math.hypot(1,z)),c=1/Math.hypot(1,t),s=c*t;
      for(const r of a){const x=r[p],y=r[q];r[p]=c*x-s*y;r[q]=s*x+c*y;}changed=true;
    }
    if(!changed){converged=true;break;}
  }
  if(!converged)throw Error('SVD failed to converge');
  return Array.from({length:rows},(_,i)=>Math.hypot(...a.map(r=>r[i]))).sort((x,y)=>y-x).slice(0,Math.min(rows,cols));
}
export function singularity(jacobianColumns,jointTypes,settings){
  const dims=settings.task==='position'?3:6,L=settings.characteristic_length_m;
  const matrix=Array.from({length:dims},(_,i)=>jacobianColumns.map((col,j)=>col[i]*(jointTypes[j]==='prismatic'?L:1)/(i<3?L:1)));
  const values=singularValues(matrix),max=values[0],min=values.at(-1),tol=Math.max(1e-12,max*settings.rank_relative_tolerance);
  const rank=values.filter(x=>x>tol).length,rankLoss=rank<Math.min(dims,jointTypes.length),infinite=rankLoss||min===0;
  const condition=infinite?null:max/min;
  return {singular_values:values,sigma_min:min,effective_rank:rank,task_dimension:dims,maximum_rank:Math.min(dims,jointTypes.length),task_underactuated:jointTypes.length<dims,
    condition_number:condition,condition_number_infinite:infinite,status:rankLoss?'singular':min<settings.near_sigma_threshold||condition>settings.condition_warning?'near_singular':'regular'};
}
