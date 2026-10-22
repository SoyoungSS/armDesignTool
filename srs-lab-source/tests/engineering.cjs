const assert=require('node:assert/strict'),E=require('../dist/engine.js');
const c=E.defaults(),zero=Array(7).fill(0),close=(a,b,t=1e-6)=>assert.ok(Math.abs(a-b)<t,`${a} != ${b}`);
// Analytic payload-only horizontal lever arm, including wrist tool offset.
const load={...c,selfMass:1e-12,friction:0,viscous:0};const g=E.gravity(load,zero);close(g[1],-3*9.80665*.6);close(g[3],-3*9.80665*.3);close(g[5],-3*9.80665*.087);close(g[0],0);close(g[6],0);
const q=[.5,-.7,.3,1.1,-.4,.2,.5],f=E.fk(c,q);close(f.p.length,3);close(E.fk(c,zero).p[0],.6);
const J=E.jacAt(f,f.p),h=1e-6;for(let j=0;j<7;j++){let p=q.slice(),n=q.slice();p[j]+=h;n[j]-=h;const A=E.fk(c,p),B=E.fk(c,n);for(let k=0;k<3;k++)close((A.p[k]-B.p[k])/(2*h),J[j][k],1e-7);}
// Potential gradient must equal gravity support torque.
const potential=q=>E.fk(c,q).bodies.reduce((s,b)=>s+b.m*9.80665*b.p[2],0),G=E.gravity(c,q);for(let j=0;j<7;j++){let p=q.slice(),n=q.slice();p[j]+=h;n[j]-=h;close((potential(p)-potential(n))/(2*h),G[j],1e-6);}
const M=E.massMatrix(c,q);for(let i=0;i<7;i++)for(let j=0;j<7;j++)close(M[i][j],M[j][i],1e-9);let L=Array.from({length:7},()=>Array(7).fill(0));for(let i=0;i<7;i++)for(let j=0;j<=i;j++){let v=M[i][j];for(let k=0;k<j;k++)v-=L[i][k]*L[j][k];if(i===j){assert.ok(v>0);L[i][j]=Math.sqrt(v);}else L[i][j]=v/L[j][j];}
E.inverseDynamics(c,q,zero,zero).forEach((v,j)=>close(v,G[j],1e-8));
// Velocity product consistency: qd^T C qd = 1/2 qd^T Mdot qd.
const noFriction={...c,viscous:0,friction:0},v=[.3,.4,-.2,.1,.5,-.3,.2],C=E.inverseDynamics(noFriction,q,v,zero).map((t,j)=>t-G[j]);const Ma=E.massMatrix(c,q.map((x,j)=>x+h*v[j])),Mb=E.massMatrix(c,q.map((x,j)=>x-h*v[j]));let rhs=0;for(let i=0;i<7;i++)for(let j=0;j<7;j++)rhs+=.5*v[i]*v[j]*(Ma[i][j]-Mb[i][j])/(2*h);close(v.reduce((s,x,j)=>s+x*C[j],0),rhs,1e-6);
const target=[.35,.15,.25],ik=E.ik(c,target);assert.ok(ik.converged);assert.ok(ik.error<.0001);const bad=E.ik(c,[2,0,0]);assert.equal(bad.converged,false);
assert.throws(()=>E.validate({...c,reach:.8}));assert.throws(()=>E.validate({...c,efficiency:0}));assert.throws(()=>E.validate({...c,inertias:Array(7).fill([10,1,1])}));
const small={...c,samples:300},a=E.run(small),b=E.run(small);assert.deepEqual(a.points,b.points);assert.deepEqual(a.joints,b.joints);a.dynamics.series.forEach(s=>s.qd.forEach(x=>assert.ok(Math.abs(x)<=E.rad(c.speed)+1e-8)));a.joints.forEach(j=>{assert.ok(j.outputPeak>=j.staticPeak*c.safety-1e-8);close(j.motorRpm,800);assert.ok(Object.values(j).every(v=>typeof v!=='number'||Number.isFinite(v)));});
console.log('PASS: analytic lever arm, FK/Jacobian, gravity potential, mass-matrix SPD, inverse-dynamics rest/energy, IK, validation, reproducibility and sizing bounds.');
