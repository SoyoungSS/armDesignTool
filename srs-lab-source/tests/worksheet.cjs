const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),E=require('../dist/engine.js');
(async()=>{
 const reference=JSON.parse(fs.readFileSync('tests/fixtures/worksheet-statics-reference.json','utf8'));
 const analyze=request=>({poses:[reference.poses.find(p=>JSON.stringify(p.q)===JSON.stringify(request.poses[0].q))]});
 const input=JSON.parse(fs.readFileSync('dist/worksheet-minimal.json','utf8')),c=E.defaults('WORKSHEET-7R');
 const near=(a,b,t=1e-9)=>assert.ok(Math.abs(a-b)<t,`${a} != ${b}`);
 const sandbox={SRS:E};sandbox.globalThis=sandbox;vm.createContext(sandbox);vm.runInContext(fs.readFileSync('dist/scene3d.js','utf8'),sandbox);
 near(c.selfMass,3.612);near(E.model(c).masses.reduce((a,b)=>a+b,0),3.612);near(c.payload,3);near(c.gravityMagnitude,9.8);
 const q0=Array(7).fill(0),f0=E.fk(c,q0);near(f0.p[2],-.635);near(Math.hypot(...f0.p.map((v,k)=>v-f0.origins[1][k])),.6);
 const vectors=E.model(c).axes;assert.deepEqual(vectors,input.simple.segments.map(s=>s.axis));
 const poses=[...input.poses,...Array.from({length:12},(_,i)=>({name:'range_'+i,q:Object.fromEntries(input.simple.segments.map((s,j)=>[s.name,s.limits_rad[0]+(s.limits_rad[1]-s.limits_rad[0])*((i*13+j*7)%37)/37]))}))];
 for(const pose of poses){const request=structuredClone(input);delete request.search;request.poses=[pose];const exact=analyze(request).poses[0];const q=Object.values(pose.q),f=E.fk(c,q),g=E.gravity(c,q),display=sandbox.Arm3DMath.displayFK(c,q);g.forEach((v,j)=>near(v,exact.holding_load[j]));f.p.forEach((v,k)=>near(v,exact.tip_pose_in_base.position_m[k]));for(let j=0;j<7;j++){f.origins[j].forEach((v,k)=>near(v,display.origins[j][k]));f.axes[j].forEach((v,k)=>near(v,display.axes[j][k]));}}
 near(E.gravity(c,[Math.PI/2,0,0,0,0,0,0])[0],25.1925268);
 near(E.gravity(c,[0,-Math.PI/2,0,0,0,0,0])[1],-23.2713838);
 const sampled=E.sample({...c,samples:100});near(sampled.statics.peak[0],25.1925268);near(sampled.statics.peak[1],23.2713838);
 assert.throws(()=>E.validate({...c,linkVectors:[[0,0,0]]}));assert.throws(()=>E.validate({...c,gravityMagnitude:NaN}));
 console.log('PASS: worksheet preset matches statics skill at 14 poses; seven displayed axis centers equal FK; mass/limits/600mm span and known torques; malformed vectors rejected.');
})();
