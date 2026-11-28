import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {prepareRequests} from '../server/module-specs.mjs';
import {ModuleDesignPage} from '../dist/module-design-page.mjs';

test('module analysis renders inline without navigating, including errors and exports',async()=>{
  const model=JSON.parse(await readFile(new URL('../../examples/models/seven-axis.json',import.meta.url)));
  const requests=prepareRequests(model),nodes={},navigations=[],downloads=[];
  const original={document:globalThis.document,location:globalThis.location,fetch:globalThis.fetch};
  globalThis.document={getElementById:id=>nodes[id]??null,querySelectorAll:()=>[]};
  globalThis.location={hash:'#motion'};
  let release;
  globalThis.fetch=async(url,options)=>{
    if(!options)return {ok:true,json:async()=>model};
    const input=JSON.parse(options.body);
    if(input.prepare_only)return {ok:true,json:async()=>({ok:true,requests,modelHash:'shared'})};
    assert.equal(input.motion_request.trajectory.kind,'waypoints');
    return new Promise(resolve=>{release=()=>resolve({ok:true,json:async()=>({
      ok:true,requests,modelHash:'shared',results:{},specifications:{model_name:model.name,
        cycle:{duration_s:5,constraint_status:'verified'},workspace:{sample_count:1000},
        joints:[],groups:[],assumptions:[],sources:{}},
    })});});
  };
  try{
    ModuleDesignPage.bind({render(){},go:page=>navigations.push(page),clear(){},apply(){},download:(...args)=>downloads.push(args)},'motion');
    await new Promise(resolve=>setImmediate(resolve));
    const exportHTML=ModuleDesignPage.simulationHTML();
    assert.doesNotMatch(exportHTML,/pythonEnvironment|checkEnvironment|name="runtime"|name="estimate"/);
    assert.match(exportHTML,/name="density"[^>]*value="2000"/);
    assert.equal((exportHTML.match(/name="target"[^>]*checked/g)||[]).length,4);
    const pending=ModuleDesignPage.analyze(requests.motion);
    assert.equal(ModuleDesignPage.analysisBusy(),true);
    assert.match(ModuleDesignPage.analysisHTML(),/正在联合计算/);
    assert.doesNotMatch(ModuleDesignPage.analysisHTML(),/moduleRun/);
    assert.deepEqual(navigations,[]);
    release();await pending;
    assert.equal(ModuleDesignPage.analysisBusy(),false);
    const html=ModuleDesignPage.analysisHTML();
    assert.match(html,/关节模组规格指导/);
    assert.match(html,/moduleCSV/);
    assert.doesNotMatch(html,/moduleRun|data-go/);
    assert.deepEqual(navigations,[]);
    nodes.moduleCSV={};nodes.moduleJSON={};
    ModuleDesignPage.bindSpecifications();
    nodes.moduleCSV.onclick();nodes.moduleJSON.onclick();
    assert.equal(downloads.length,2);
    globalThis.fetch=async()=>({ok:false,json:async()=>({error:'测试计算失败'})});
    await ModuleDesignPage.analyze(requests.motion);
    assert.match(ModuleDesignPage.analysisHTML(),/测试计算失败/);
    assert.doesNotMatch(ModuleDesignPage.analysisHTML(),/moduleRun/);
    assert.deepEqual(navigations,[]);
  }finally{
    for(const [key,value] of Object.entries(original)){
      if(value===undefined)delete globalThis[key];else globalThis[key]=value;
    }
  }
});
