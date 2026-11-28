import test from 'node:test';
import assert from 'node:assert/strict';
import {chooseGeneratedModel,mountGeneratedModel} from '../dist/generated-model-view.mjs';

test('viewer prefers new exports and preserves explicit or valid existing selections',()=>{
  const models=[{id:'new'},{id:'old'}];
  assert.equal(chooseGeneratedModel(models),'new');
  assert.equal(chooseGeneratedModel(models,{requested:'old'}),'old');
  assert.equal(chooseGeneratedModel(models,{requested:'old',lastExport:'new'}),'new');
  assert.equal(chooseGeneratedModel(models,{requested:'missing'}),'new');
  assert.equal(chooseGeneratedModel(models,{requested:'demo:fr3'}),'demo:fr3');
  assert.equal(chooseGeneratedModel([]),'demo:worksheet');
});

test('viewer binds exported model data, switching and initial/zero poses',async()=>{
  const keys=['document','location','history','fetch','sessionStorage'];
  const previous=Object.fromEntries(keys.map(key=>[key,globalThis[key]]));
  const node=()=>({children:[],textContent:'',value:'',append(...children){this.children.push(...children);},replaceChildren(...children){this.children=children;}});
  const selectors=Object.fromEntries(['[data-model]','[data-model-refresh]','[data-reset]','.run-canvas','[data-links]','[data-evidence]','[data-controls]','.run-status','[data-pose]'].map(key=>[key,node()]));
  const root={querySelector:key=>selectors[key]},project=node(),store=new Map([['srs-latest-generated-model','new']]);
  const models=[{id:'new',name:'New export',created_at:'2026-11-28T00:00:00Z',mujoco_status:'failed'},{id:'old',name:'Older export',created_at:'2026-11-27T00:00:00Z'}];
  const spec={frames:[{bodies:[]},{bodies:[]}],joints:[{name:'J1'}],geoms:[{}],collision:{overall:'failed',poses:{initial:'failed',geometric_zero:'needs_review'}}};
  const requests=[],shown=[];let disposed=0,cleanup;
  globalThis.document={getElementById:id=>id==='run-view'?root:project,createElement:node};
  globalThis.location={href:'http://localhost/#models3d'};
  globalThis.history={replaceState(_a,_b,url){globalThis.location.href=String(url);}};
  globalThis.sessionStorage={getItem:key=>store.get(key)??null,setItem:(key,value)=>store.set(key,value),removeItem:key=>store.delete(key)};
  globalThis.fetch=async url=>{requests.push(url);return {ok:true,json:async()=>url==='/api/generated-models'?{models}:spec};};
  try{
    await mountGeneratedModel({createScene(){return {show:index=>shown.push(index),reset(){},dispose(){disposed++;}};},closeViewer(){},setDispose:value=>{cleanup=value;}});
    assert.ok(requests.includes('/api/generated-models/new/viewer'));
    assert.ok(!requests.some(url=>url.startsWith('/demo-assets')));
    assert.match(selectors['.run-status'].textContent,/New export/);
    assert.match(selectors['.run-status'].textContent,/检查未通过/);
    assert.equal(selectors['[data-controls]'].hidden,false);
    assert.equal(selectors['[data-reset]'].disabled,false);
    selectors['[data-pose]'].value='1';selectors['[data-pose]'].onchange();
    assert.equal(shown.at(-1),1);
    selectors['[data-model]'].value='old';await selectors['[data-model]'].onchange();
    assert.ok(requests.includes('/api/generated-models/old/viewer'));
    assert.equal(project.textContent,'Older export');
    cleanup();assert.equal(disposed,2);
  }finally{
    for(const [key,value] of Object.entries(previous)){if(value===undefined)delete globalThis[key];else globalThis[key]=value;}
  }
});
