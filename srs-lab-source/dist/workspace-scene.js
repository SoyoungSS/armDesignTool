/* World-coordinate point cloud and successful-pose preview; no kinematic solver. */
(function(root){'use strict';
// Draw a shaded conceptual assembly directly from the replayed joint and TCP frames.
function drawSolidArm(ctx,preview,span,project,view){
 const {V,basis}=root.Arm3DMath;
 const faces=[];
 const light=V.unit(V.add(view,[-.55,-.25,.8]));
 const shadeFace=(vertices,normal,color)=>{
  if(V.dot(normal,view)<=0)return;
  const points=vertices.map(project);
  const depth=points.reduce((sum,p)=>sum+p.depth,0)/points.length;
  const shade=.57+.43*Math.max(0,V.dot(normal,light));
  faces.push({points,depth,color:color.map(channel=>Math.round(channel*shade))});
 };
 function turnedPart(origin,axis,profile,color){
  const [u,v]=basis(axis),steps=24;
  const rings=profile.map(([offset,radius])=>Array.from({length:steps},(_,k)=>{
   const angle=k*2*Math.PI/steps;
   return V.add(V.add(origin,V.mul(axis,offset)),V.add(V.mul(u,radius*Math.cos(angle)),V.mul(v,radius*Math.sin(angle))));
  }));
  for(let i=1;i<rings.length;i++){
   for(let k=0;k<steps;k++){
    const next=(k+1)%steps;
    const a=rings[i-1][k],b=rings[i-1][next],c=rings[i][next],d=rings[i][k];
    const normal=V.unit(V.cross(V.sub(b,a),V.sub(d,a)));
    shadeFace([a,b,c,d],normal,color);
   }
  }
  shadeFace(rings[0],V.mul(axis,-1),color);
  shadeFace(rings.at(-1),axis,color);
 }
 const silver=[222,231,236],steel=[133,156,174],dark=[65,83,99],accent=[227,156,70];
 const unit=span*.04;
 const base=preview.base_frame;
 turnedPart(base.origin_world_m,base.axes_world[2],[[-.75*unit,1.5*unit],[-.5*unit,1.65*unit],[0,1.55*unit]],dark);
 turnedPart(base.origin_world_m,base.axes_world[2],[[0,1.05*unit],[.25*unit,unit]],steel);
 const nodes=preview.nodes_world_m;
 for(let i=1;i<nodes.length;i++){
  const delta=V.sub(nodes[i],nodes[i-1]),length=V.norm(delta);
  if(length<1e-9)continue;
  const axis=V.unit(delta),radius=Math.min(unit*.67,length*.24);
  turnedPart(nodes[i-1],axis,[[0,radius*.74],[length*.16,radius],[length*.84,radius*.86],[length,radius*.68]],silver);
 }
 for(const joint of preview.joints_world){
  const color=joint.selected?accent:dark;
  turnedPart(joint.origin,joint.axis,[[-unit*.75,unit*.75],[-unit*.52,unit],[unit*.52,unit],[unit*.75,unit*.75]],color);
  for(const side of [-1,1]){
   const cap=V.add(joint.origin,V.mul(joint.axis,side*unit*.77));
   turnedPart(cap,joint.axis,[[-unit*.04,unit*.62],[unit*.04,unit*.62]],joint.selected?accent:steel);
  }
 }
 turnedPart(preview.position_world_m,preview.tip_axes_world[2],[[-unit*.38,unit*.8],[0,unit*.8]],steel);
 faces.sort((a,b)=>a.depth-b.depth);
 for(const {points,color} of faces){
  ctx.beginPath();
  ctx.moveTo(points[0].x,points[0].y);
  for(const p of points.slice(1))ctx.lineTo(p.x,p.y);
  ctx.closePath();
  ctx.fillStyle=`rgb(${color.join(',')})`;
  ctx.strokeStyle=ctx.fillStyle;
  ctx.lineWidth=.45;
  ctx.fill();
  ctx.stroke();
 }
}
class WorkspaceScene{
 constructor(canvas,result,preview,options={}){
  this.options=options;this.cv=canvas;this.result=result;this.preview=preview;
  this.yaw=.7;this.pitch=.45;this.zoom=1;
  let drag=null;
  const hitJoint=event=>{
   const {left,top}=canvas.getBoundingClientRect();
   const x=event.clientX-left,y=event.clientY-top;
   return this.jointHits?.find(point=>Math.hypot(x-point.x,y-point.y)<=point.radius);
  };
  canvas.onpointerdown=event=>{
   if(event.button!==0)return;
   const joint=hitJoint(event);
   drag={x:event.clientX,y:event.clientY,joint:joint?.name,id:event.pointerId};
   canvas.setPointerCapture?.(event.pointerId);
  };
  canvas.onpointermove=event=>{
   if(!drag){
    if(this.options.dragJoint)canvas.style.cursor=hitJoint(event)?'ew-resize':'grab';
    return;
   }
   if(event.pointerId!==drag.id)return;
   const dx=event.clientX-drag.x,dy=event.clientY-drag.y;
   if(drag.joint){
    this.preview=this.options.dragJoint(drag.joint,(dx-dy*.5)*.012);
   }else{
    this.yaw+=dx*.008;
    this.pitch=Math.max(-1.4,Math.min(1.4,this.pitch+dy*.008));
   }
   drag.x=event.clientX;drag.y=event.clientY;
   this.render();
  };
  canvas.onpointerup=canvas.onpointercancel=canvas.onlostpointercapture=()=>{drag=null;};
  canvas.onwheel=event=>{
   event.preventDefault();
   this.zoom=Math.max(.3,Math.min(4,this.zoom*Math.exp(-event.deltaY*.001)));
   this.render();
  };
  this.render();
 }
 setSection(section,only){this.section=section;this.sectionOnly=only;this.render();}
 render(){
  const cv=this.cv,rect=cv.getBoundingClientRect(),W=rect.width,H=rect.height;if(!W||!H)return;const dpr=root.devicePixelRatio||1;cv.width=W*dpr;cv.height=H*dpr;const ctx=cv.getContext('2d');ctx.scale(dpr,dpr);ctx.clearRect(0,0,W,H);
  const data=this.result.points,positions=[...data.map(p=>p.position_world_m),...(this.options.fitPositions??[])],lo=[0,1,2].map(k=>Math.min(...positions.map(p=>p[k]))),hi=[0,1,2].map(k=>Math.max(...positions.map(p=>p[k]))),center=lo.map((v,k)=>(v+hi[k])/2),span=Math.max(.1,...hi.map((v,k)=>v-lo[k]));
  const cy=Math.cos(this.yaw),sy=Math.sin(this.yaw),cp=Math.cos(this.pitch),sp=Math.sin(this.pitch),factor=Math.min(W,H)*.65*this.zoom/span;
  const project=p=>{const [x,y,z]=p.map((v,k)=>v-center[k]);return {x:W/2+factor*(cy*x-sy*y),y:H/2-factor*(-sy*sp*x-cy*sp*y+cp*z),depth:sy*cp*x+cy*cp*y+sp*z};};
  if(this.options.dragJoint){
   const radius=Math.max(12,span*.04*factor);
   this.jointHits=this.preview.joints_world.map(joint=>({
    ...project(joint.origin),name:joint.name,radius,
   })).sort((a,b)=>b.depth-a.depth);
  }
  if(this.section){
   const plane=this.section.corners_world_m.map(project);
   ctx.beginPath();ctx.moveTo(plane[0].x,plane[0].y);
   for(const p of plane.slice(1))ctx.lineTo(p.x,p.y);
   ctx.lineTo(plane[0].x,plane[0].y);ctx.fillStyle='rgba(45,112,147,.13)';ctx.fill();
   ctx.strokeStyle='#417491';ctx.lineWidth=1;ctx.stroke();
  }
  const source=this.sectionOnly?this.section.points:data;
  const step=Math.max(1,Math.ceil(source.length/5000));
  const visible=source.filter(p=>!this.options.pointVisible||this.options.pointVisible(p,this.options.filter??'all')).filter((_,i)=>i%step===0).map(p=>({...project(p.position_world_m),color:this.options.pointColor?.(p)??'#2c6faf'})).sort((a,b)=>a.depth-b.depth);
  const voxelPixels=this.result.dexterity?Math.max(3,this.result.dexterity.voxel_size_m*factor*.57):0;
  for(const p of visible){
   ctx.fillStyle=p.color;
   if(voxelPixels)ctx.fillRect(p.x-voxelPixels/2,p.y-voxelPixels/2,voxelPixels,voxelPixels);
   else{ctx.beginPath();ctx.arc(p.x,p.y,2.2,0,Math.PI*2);ctx.fill();}
  }
  const line=(a,b,color,width)=>{a=project(a);b=project(b);ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.strokeStyle=color;ctx.lineWidth=width;ctx.stroke();};
  for(const [k,color]of [[0,'#a1354f'],[1,'#258064'],[2,'#245bb3']]){const end=[0,0,0];end[k]=span*.25;line([0,0,0],end,color,2);const p=project(end);ctx.fillStyle=color;ctx.fillText('W'+['X','Y','Z'][k],p.x+4,p.y);}
  if(this.preview){
   const {base_frame:base,nodes_world_m:nodes,position_world_m:flange}=this.preview;
   if(this.options.solidLinks){
    const view=[sy*cp,cy*cp,sp];
    drawSolidArm(ctx,this.preview,span,project,view);
   }else{
   const radius=span*.055;
   const basePoint=(x,y,z)=>base.origin_world_m.map((v,k)=>v+x*base.axes_world[0][k]+y*base.axes_world[1][k]+z*base.axes_world[2][k]);
   const corners=[[-radius,-radius],[radius,-radius],[radius,radius],[-radius,radius]];
   const plate=corners.map(([x,y])=>project(basePoint(x,y,0)));
   ctx.beginPath();ctx.moveTo(plate[0].x,plate[0].y);
   for(const p of plate.slice(1))ctx.lineTo(p.x,p.y);
   ctx.lineTo(plate[0].x,plate[0].y);ctx.fillStyle='#d5dce2';ctx.fill();ctx.strokeStyle='#425563';ctx.lineWidth=2;ctx.stroke();
   const baseLabel=project(basePoint(-radius,-radius,0));
   ctx.fillStyle='#34495b';ctx.fillText('基座 B',baseLabel.x-32,baseLabel.y+15);
   const axisStart=basePoint(-radius,-radius,0),axisEnd=basePoint(-radius,-radius,span*.1);
   line(axisStart,axisEnd,'#34495b',2);
   const axisLabel=project(axisEnd);ctx.fillText('B+Z',axisLabel.x-27,axisLabel.y-3);

   for(let i=1;i<nodes.length;i++)line(nodes[i-1],nodes[i],i===1?'#647887':'#215d88',3);
   for(const node of nodes.slice(1,-1)){const p=project(node);ctx.beginPath();ctx.arc(p.x,p.y,4,0,Math.PI*2);ctx.fillStyle='#215d88';ctx.fill();}
   }
   if(this.preview.tip_axes_world){
    const colors=['#a1354f','#258064','#245bb3'];
    this.preview.tip_axes_world.forEach((axis,i)=>{
     const end=flange.map((v,k)=>v+axis[k]*span*.14);
     line(flange,end,colors[i],2);
     const p=project(end);
     ctx.fillStyle=colors[i];ctx.fillText('TCP '+['X','Y','Z'][i],p.x+3,p.y);
    });
   }
   const tip=project(flange);ctx.beginPath();ctx.arc(tip.x,tip.y,5,0,Math.PI*2);ctx.fillStyle='#b65b17';ctx.fill();
   ctx.fillText(this.preview.tip_axes_world?'TCP':'法兰',tip.x+7,tip.y+4);
   if(this.preview.selected_joint){
    const joint=this.preview.selected_joint,p=project(joint.origin_world_m);
    ctx.beginPath();ctx.arc(p.x,p.y,7,0,Math.PI*2);
    ctx.strokeStyle='#b65b17';ctx.lineWidth=3;ctx.stroke();
    ctx.fillStyle='#8d400e';ctx.fillText(joint.name,p.x+10,p.y-7);
   }
  }
  ctx.fillStyle='#425563';ctx.font='12px sans-serif';ctx.fillText('世界坐标 / m · 拖动旋转 · 滚轮缩放',15,24);
 }
}
root.WorkspaceScene=WorkspaceScene;
})(globalThis);
