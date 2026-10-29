// Geometry-only serial-chain FK/Jacobian. Browser preview and CPU analysis share this file.
export const add=(a,b)=>a.map((v,i)=>v+b[i]);
export const sub=(a,b)=>a.map((v,i)=>v-b[i]);
export const scale=(a,s)=>a.map(v=>v*s);
export const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
export const norm=a=>Math.hypot(...a);
export const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
export const eye=()=>[[1,0,0],[0,1,0],[0,0,1]];
export const mv=(R,v)=>R.map(row=>dot(row,v));
export const transpose=R=>R[0].map((_,i)=>R.map(row=>row[i]));
export const mm=(A,B)=>A.map(row=>[0,1,2].map(j=>row.reduce((s,v,k)=>s+v*B[k][j],0)));
export function rot(a,t){const [x,y,z]=a,c=Math.cos(t),s=Math.sin(t),v=1-c;return [[c+x*x*v,x*y*v-z*s,x*z*v+y*s],[y*x*v+z*s,c+y*y*v,y*z*v-x*s],[z*x*v-y*s,z*y*v+x*s,c+z*z*v]];}
export const rpy=([r,p,y])=>mm(mm(rot([0,0,1],y),rot([0,1,0],p)),rot([1,0,0],r));
export function forward(model,q){
  let p=[0,0,0],R=eye(),k=0;const joints=[],nodes=[[0,0,0]];
  for(const j of model.joints){
    p=add(p,mv(R,j.origin.xyz_m));R=mm(R,rpy(j.origin.rpy_rad));nodes.push(p.slice());
    if(j.type!=='fixed'){
      const axis=mv(R,j.axis);joints.push({name:j.name,type:j.type,axis,origin:p.slice()});
      if(j.type==='revolute')R=mm(R,rot(j.axis,q[k]));else p=add(p,scale(axis,q[k]));
      k++;
    }
  }
  p=add(p,mv(R,model.tip.xyz_m));R=mm(R,rpy(model.tip.rpy_rad));nodes.push(p.slice());
  return {position_m:p,rotation:R,joints,nodes};
}
export function jacobian(f,orientation=false){
  return f.joints.map(j=>{
    const linear=j.type==='revolute'?cross(j.axis,sub(f.position_m,j.origin)):j.axis;
    return orientation?[...linear,...(j.type==='revolute'?j.axis:[0,0,0])]:linear;
  });
}
// SO(3) logarithm in base coordinates; robust at 180 degrees (skew alone vanishes).
export function orientationError(target,current){
  const R=mm(target,transpose(current)),c=Math.max(-1,Math.min(1,(R[0][0]+R[1][1]+R[2][2]-1)/2)),angle=Math.acos(c);
  const skew=[R[2][1]-R[1][2],R[0][2]-R[2][0],R[1][0]-R[0][1]];
  if(angle<1e-7)return scale(skew,.5);
  if(Math.PI-angle<1e-5){
    const diagonal=[R[0][0],R[1][1],R[2][2]],i=diagonal.indexOf(Math.max(...diagonal)),a=[0,0,0];
    a[i]=Math.sqrt(Math.max(0,(diagonal[i]+1)/2));
    for(let j=0;j<3;j++)if(i!==j)a[j]=(R[i][j]+R[j][i])/(4*a[i]);
    const sign=dot(a,skew)<0?-1:1;return scale(a,sign*angle/norm(a));
  }
  return scale(skew,angle/(2*Math.sin(angle)));
}
export function solveLinear(A,b){
  const m=A.map((row,i)=>[...row,b[i]]),N=b.length;
  for(let k=0;k<N;k++){
    let pivot=k;for(let i=k+1;i<N;i++)if(Math.abs(m[i][k])>Math.abs(m[pivot][k]))pivot=i;
    if(Math.abs(m[pivot][k])<1e-18)return null;
    [m[k],m[pivot]]=[m[pivot],m[k]];const v=m[k][k];for(let j=k;j<=N;j++)m[k][j]/=v;
    for(let i=0;i<N;i++)if(i!==k){const s=m[i][k];for(let j=k;j<=N;j++)m[i][j]-=s*m[k][j];}
  }
  return m.map(row=>row[N]);
}
