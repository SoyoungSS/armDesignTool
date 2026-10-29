const escape=s=>String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
export const WORKSPACE_CATEGORIES={
 spatial_reachable:{label:'空间位置可达（灵巧性未验证或姿态未全通过）',color:'#2c6faf'},
 sampled_dexterous:{label:'灵巧空间（已采样姿态全通过）',color:'#18804b'},
 partially_dexterous:{label:'部分采样姿态可达',color:'#d88a19'},
 unreachable:{label:'不可达（超出几何外界）',color:'#c4383b'},
 unresolved:{label:'未找到位置解（待定）',color:'#929aa4'}
};
export function pointCategory(point){return point.category??(point.q?'spatial_reachable':'unresolved');}
export function pointColor(point){return WORKSPACE_CATEGORIES[pointCategory(point)]?.color??'#929aa4';}
export function pointVisible(point,filter='all'){return filter==='all'||(filter==='spatial'?point.spatial_status==='reachable'||(point.spatial_status===undefined&&!!point.q):pointCategory(point)===filter);}
export function projectionSVG(result,plane='xy',section=false,filter='all'){
  const axes={xy:[0,1],xz:[0,2],yz:[1,2]}[plane];if(!axes)throw Error('Unknown plane');
  const ids=section?new Set(result.sections[plane]):null;
  const points=result.points.filter(p=>(!ids||ids.has(p.id))&&pointVisible(p,filter));
  const all=result.points.map(p=>p.position_base_m),lo=axes.map(k=>Math.min(...all.map(p=>p[k]))),hi=axes.map(k=>Math.max(...all.map(p=>p[k]))),span=Math.max(...hi.map((v,k)=>v-lo[k]),.001),cx=(hi[0]+lo[0])/2,cy=(hi[1]+lo[1])/2;
  const x=p=>210+(p[axes[0]]-cx)*320/span,y=p=>210-(p[axes[1]]-cy)*320/span;
  const step=Math.max(1,Math.ceil(points.length/5000));
  const circles=points.filter((_,i)=>i%step===0).map(p=>`<circle cx="${x(p.position_base_m).toFixed(2)}" cy="${y(p.position_base_m).toFixed(2)}" r="2.3" fill="${pointColor(p)}" opacity="0.7"/>`).join('');
  const legend=Object.values(WORKSPACE_CATEGORIES).map((v,i)=>`<rect x="18" y="${446+i*17}" width="9" height="9" fill="${v.color}"/><text x="34" y="${455+i*17}" font-size="10" fill="#303840">${escape(v.label)}</text>`).join('');
  return `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 420 540" role="img" aria-label="${plane.toUpperCase()} ${section?'section':'projection'}"><rect width="420" height="540" fill="#fafbf9"/><text x="20" y="25" fill="#303840" font-size="14">${escape(plane.toUpperCase())} ${section?'剖面':'投影'} / B / m</text><path d="M40 40 V380 H390" fill="none" stroke="#9aa5ad"/>${circles}<text x="40" y="405" font-size="11">${plane[0].toUpperCase()} [${lo[0].toFixed(3)}, ${hi[0].toFixed(3)}] m</text><text x="40" y="423" font-size="11">${plane[1].toUpperCase()} [${lo[1].toFixed(3)}, ${hi[1].toFixed(3)}] m; ${points.length} points</text>${legend}</svg>`;
}
export function pointsCSV(result){
  const quote=v=>'"'+String(v??'').replaceAll('"','""')+'"';
  return 'id,x_base_m,y_base_m,z_base_m,x_world_m,y_world_m,z_world_m,orientation_success_rate,category,spatial_status,dexterity_status,unreachable_reason,q_json,solutions_json,position_solution_json\n'+result.points.map(p=>[p.id,...p.position_base_m,...p.position_world_m,p.reachability,p.category,p.spatial_status,p.dexterity_status,p.unreachable_reason,JSON.stringify(p.q),JSON.stringify(p.solutions??[]),JSON.stringify(p.position_solution??null)].map(quote).join(',')).join('\n')+'\n';
}
export function reportMarkdown(result){
  const s=result.summary;
  const classes=Object.entries(WORKSPACE_CATEGORIES).map(([key,value])=>`- ${value.label}：${s.workspace_counts?.[key]??0} 点`).join('\n');
  return `# 机械臂可达工作空间分析\n\n模型：${escape(result.normalized_input.model.name)}\n\n内核：${result.engine_version}；模式：${result.mode}\n\n输入 SHA-256：${result.input_sha256}\n\n模型 SHA-256：${result.model_sha256}\n\n- 采样点数：${s.point_count}\n- 有已找到解的点数：${s.reachable_point_count}\n- 采样位姿检查数：${s.ik_query_count??'不适用'}\n- 已采样姿态成功率：${s.sampled_orientation_success_rate===null?'不适用':(s.sampled_orientation_success_rate*100).toFixed(3)+'%'}\n- 基座系已找到解点包围盒 / m：${JSON.stringify(s.bounds_base_m)}\n- 世界系已找到解点包围盒 / m：${JSON.stringify(s.bounds_world_m)}\n- 已占据采样体素数：${s.occupied_voxels}\n- 已占据采样体素体积 / m³：${s.sampled_voxel_volume_m3}（依赖分辨率，不是真实工作空间体积）\n\n## 空间可达性分类\n\n${classes}\n\n空间位置可达总数（含灵巧与部分姿态点）：${s.spatial_reachable_point_count??s.reachable_point_count}。灵巧性是否已采样检查：${result.scope.dexterity_evaluated?'是':'否'}。连续全姿态灵巧工作空间未证明；单一或重复姿态不作为灵巧空间判据。\n\n几何外界：${JSON.stringify(result.geometric_reach_bound??null)}。只有目标超出此外界才标记“不可达”；界内IK失败保持待定。位置可达但指定姿态均未找到解的点仍属于空间位置可达。\n\n## 范围与解释\n\n目标位置和切片在基座系B中；世界坐标由base_in_world转换。IK失败只表示预算内未找到解。成功率针对离散姿态集合；未指定姿态时只是位置IK成功率。关节随机采样不是笛卡尔均匀采样；投影重叠不保证空间内部可达。\n\n${result.warnings.map(w=>'- '+w).join('\n')}\n`;
}
