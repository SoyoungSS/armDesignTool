import {validateContract,invalid} from '../contracts/validate.mjs';
const unique=(values,path)=>{if(new Set(values).size!==values.length)invalid(path,'duplicate identifier');};
function psd(A,path){
  const scale=Math.max(...A.flat().map(Math.abs),1e-30),a=A.map(r=>r.map(x=>x/scale)),eps=1e-10;
  for(let i=0;i<3;i++)if(a[i][i]<-eps)invalid(path,'matrix must be positive semidefinite');
  for(let i=0;i<3;i++)for(let j=i+1;j<3;j++)if(a[i][i]*a[j][j]-a[i][j]**2<-eps)invalid(path,'matrix must be positive semidefinite');
  const det=a[0][0]*(a[1][1]*a[2][2]-a[1][2]*a[2][1])-a[0][1]*(a[1][0]*a[2][2]-a[1][2]*a[2][0])+a[0][2]*(a[1][0]*a[2][1]-a[1][1]*a[2][0]);
  if(det<-eps)invalid(path,'matrix must be positive semidefinite');
}
export function validateInertia(I,mass,path){
  const scale=Math.max(...I.flat().map(Math.abs),1e-30);
  for(let a=0;a<3;a++)for(let b=0;b<3;b++)if(Math.abs(I[a][b]-I[b][a])>1e-10*scale)invalid(path,'symmetric COM inertia required');
  psd(I,path);
  const halfTrace=(I[0][0]+I[1][1]+I[2][2])/2;
  psd(I.map((r,a)=>r.map((x,b)=>(a===b?halfTrace:0)-x)),path+' (principal-moment triangle inequality)');
  if(mass===0&&I.some(r=>r.some(x=>x!==0)))invalid(path,'massless component must have zero inertia');
}
export function validateDesignModel(model,{require:required='geometry'}={}){
  validateContract('design-model',model);
  unique(model.joints.map(j=>j.id),'$.joints.id');unique(model.joints.map(j=>j.name),'$.joints.name');
  const baseComponents=model.base_components??[];
  unique(baseComponents.map(component=>component.name),'$.base_components');
  for(const [i,component] of baseComponents.entries()){
    if(component.inertia_kg_m2)validateInertia(component.inertia_kg_m2,component.mass_kg,`$.base_components[${i}].inertia_kg_m2`);
  }
  const missingMass=[],missingInertia=[];
  if(!model.joints.some(j=>j.type!=='fixed'))invalid('$.joints','at least one active joint required');
  for(const [i,j] of model.joints.entries()){
    const p=`$.joints[${i}]`;
    if(j.name==='tip')invalid(p+'.name','tip is a reserved frame');
    if(j.type==='fixed'){if('axis'in j||'limits'in j)invalid(p,'fixed joint must omit axis/limits');if('simulation_limits'in j)invalid(p+'.simulation_limits','fixed joint has no drive');}
    else {if(!j.axis||Math.hypot(...j.axis)<1e-12)invalid(p+'.axis','nonzero axis required');if(!j.limits||j.limits[0]>=j.limits[1])invalid(p+'.limits','increasing limits required');}
    if(!j.components)missingMass.push(p+'.components');
    unique((j.components??[]).map(c=>c.name),p+'.components');
    for(const [k,c] of (j.components??[]).entries()){
      const cp=`${p}.components[${k}].inertia_kg_m2`,I=c.inertia_kg_m2;
      if(!I){if(c.mass_kg>0)missingInertia.push(cp);continue;}
      validateInertia(I,c.mass_kg,cp);
    }
  }
  if(!['geometry','mass_properties','inertial_properties'].includes(required))throw Error('Unknown capability: '+required);
  if(required!=='geometry'&&missingMass.length)invalid(missingMass[0],'explicit components required; [] declares a massless body');
  if(required==='inertial_properties'&&missingInertia.length)invalid(missingInertia[0],'COM inertia required for massive component');
  return {capabilities:{geometry:true,mass_properties:missingMass.length===0,inertial_properties:missingMass.length===0&&missingInertia.length===0},missing:{mass_properties:missingMass,inertial_properties:missingInertia},assumptions:collectAssumptions(model)};
}
export function collectAssumptions(m){return [...new Set([...(m.provenance?.assumptions??[]),...m.joints.flatMap(j=>(j.components??[]).flatMap(c=>c.provenance.assumptions)),...(m.base_components??[]).flatMap(c=>c.provenance.assumptions),...(m.payload?.provenance.assumptions??[])])];}
export function geometryModel(m){return {name:m.name,joints:m.joints.map(j=>({name:j.name,type:j.type,origin:structuredClone(j.origin),...(j.type==='fixed'?{}:{axis:j.axis.map(x=>x/Math.hypot(...j.axis)),limits:[...j.limits]})})),tip:structuredClone(m.tip)};}
