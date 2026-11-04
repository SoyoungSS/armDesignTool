/* World-coordinate point cloud and successful-pose preview; no kinematic solver. */
(function(root){'use strict';
class WorkspaceScene{
 constructor(canvas,result,preview,options={}){this.options=options;this.cv=canvas;this.result=result;this.preview=preview;this.yaw=.7;this.pitch=.45;this.zoom=1;let drag=null;canvas.onpointerdown=e=>{if(e.button!==0)return;drag=[e.clientX,e.clientY];canvas.setPointerCapture?.(e.pointerId);};canvas.onpointermove=e=>{if(!drag)return;this.yaw+=(e.clientX-drag[0])*.008;this.pitch=Math.max(-1.4,Math.min(1.4,this.pitch+(e.clientY-drag[1])*.008));drag=[e.clientX,e.clientY];this.render();};canvas.onpointerup=canvas.onpointercancel=()=>{drag=null;};canvas.onwheel=e=>{e.preventDefault();this.zoom=Math.max(.3,Math.min(4,this.zoom*Math.exp(-e.deltaY*.001)));this.render();};this.render();}
 setSection(section,only){this.section=section;this.sectionOnly=only;this.render();}
 render(){
  const cv=this.cv,rect=cv.getBoundingClientRect(),W=rect.width,H=rect.height;if(!W||!H)return;const dpr=root.devicePixelRatio||1;cv.width=W*dpr;cv.height=H*dpr;const ctx=cv.getContext('2d');ctx.scale(dpr,dpr);ctx.clearRect(0,0,W,H);
  const data=this.result.points,positions=data.map(p=>p.position_world_m),lo=[0,1,2].map(k=>Math.min(...positions.map(p=>p[k]))),hi=[0,1,2].map(k=>Math.max(...positions.map(p=>p[k]))),center=lo.map((v,k)=>(v+hi[k])/2),span=Math.max(.1,...hi.map((v,k)=>v-lo[k]));
  const cy=Math.cos(this.yaw),sy=Math.sin(this.yaw),cp=Math.cos(this.pitch),sp=Math.sin(this.pitch),factor=Math.min(W,H)*.65*this.zoom/span;
  const project=p=>{const [x,y,z]=p.map((v,k)=>v-center[k]);return {x:W/2+factor*(cy*x-sy*y),y:H/2-factor*(-sy*sp*x-cy*sp*y+cp*z),depth:sy*cp*x+cy*cp*y+sp*z};};
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
   const tip=project(flange);ctx.beginPath();ctx.arc(tip.x,tip.y,5,0,Math.PI*2);ctx.fillStyle='#b65b17';ctx.fill();
   ctx.fillText('法兰',tip.x+7,tip.y+4);
  }
  ctx.fillStyle='#425563';ctx.font='12px sans-serif';ctx.fillText('世界坐标 / m · 拖动旋转 · 滚轮缩放',15,24);
 }
}
root.WorkspaceScene=WorkspaceScene;
})(globalThis);
