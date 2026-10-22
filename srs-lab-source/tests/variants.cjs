const assert=require('node:assert/strict'),E=require('../dist/engine.js'),fs=require('node:fs'),vm=require('node:vm');
const close=(a,b,t=1e-7)=>assert.ok(Math.abs(a-b)<t,`${a} != ${b}`),gold=require('./fixtures/original-zyx.json');
const old=E.run(gold.input);assert.deepEqual(E.fk(gold.input,gold.q).p,gold.position);assert.deepEqual(E.gravity(gold.input,gold.q),gold.gravity);assert.deepEqual(E.massMatrix(gold.input,gold.q),gold.M);assert.deepEqual(old.statics.peak,gold.staticPeak);assert.deepEqual(old.dynamics.peak,gold.dynamicPeak);assert.deepEqual(old.dynamics.rms,gold.rms);assert.deepEqual(old.joints,gold.joints);assert.deepEqual(old.points.slice(0,8),gold.points);
const sandbox={SRS:E};sandbox.globalThis=sandbox;vm.createContext(sandbox);vm.runInContext(fs.readFileSync('dist/scene3d.js','utf8'),sandbox);
for(const variant of E.configurations()){
 const c=E.defaults(variant.id),z=Array(7).fill(0),q=[.3,-.5,.4,.8,-.4,.3,.2],f=E.fk(c,q),J=E.jacAt(f,f.p),h=1e-6;
 assert.deepEqual(E.model(c).axisLabels,variant.axisLabels);assert.deepEqual(c.limits,variant.defaultLimits);assert.equal(E.run({...c,samples:100}).model.type,variant.id);
 const display=sandbox.Arm3DMath.displayFK(c,q);for(let j=0;j<7;j++)for(let k=0;k<3;k++)close(display.axes[j][k],f.axes[j][k]);
 const potential=q=>E.fk(c,q).bodies.reduce((s,b)=>s+b.m*(c.gravityMagnitude??9.80665)*b.p[2],0),G=E.gravity(c,q);for(let j=0;j<7;j++){const a=q.slice(),b=q.slice();a[j]+=h;b[j]-=h;const pa=E.fk(c,a).p,pb=E.fk(c,b).p;for(let k=0;k<3;k++)close((pa[k]-pb[k])/(2*h),J[j][k]);close((potential(a)-potential(b))/(2*h),G[j],1e-6);}
 E.inverseDynamics(c,q,z,z).forEach((v,j)=>close(v,G[j]));
 const M=E.massMatrix(c,q),L=Array.from({length:7},()=>Array(7).fill(0));for(let i=0;i<7;i++)for(let j=0;j<=i;j++){close(M[i][j],M[j][i]);let v=M[i][j];for(let k=0;k<j;k++)v-=L[i][k]*L[j][k];if(i===j){assert.ok(v>0);L[i][j]=Math.sqrt(v);}else L[i][j]=v/L[j][j];}
 assert.ok(E.ik(c,[.35,.15,.25]).converged);
}
const c=E.defaults('SRS-7R-YZX'),zero=Array(7).fill(0),payloadOnly={...c,selfMass:1e-12};assert.deepEqual(E.model(c).axes[0],[0,1,0]);assert.deepEqual(E.model(c).axes[1],[0,0,1]);close(E.gravity(payloadOnly,zero)[0],-3*9.80665*.6);close(E.gravity(c,zero)[1],0);const raised=zero.slice();raised[0]=-Math.PI/2;const p=E.fk(c,raised).p;close(p[0],0);close(p[1],0);close(p[2],c.baseHeight+c.reach);
const r=E.run({...c,samples:200});assert.equal(r.engine_version,E.version);assert.notDeepEqual(r.points.slice(1,8),old.points.slice(1,8));assert.throws(()=>E.model({...c,configuration:'unknown'}));
console.log('PASS: original ZYX numerical regression; YZX forward kinematics, Jacobian, gravity, inertia, inverse dynamics, IK, displayed axes, configuration-specific limits and version.');
