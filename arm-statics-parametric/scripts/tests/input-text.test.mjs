import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile,writeFile,mkdtemp,rm} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import {join} from 'node:path';
import {spawnSync} from 'node:child_process';
import {fileURLToPath} from 'node:url';
import {parseRequestText} from '../lib/input-text.mjs';
import {analyze} from '../lib/statics.mjs';
const root=fileURLToPath(new URL('../../',import.meta.url));
test('JSONC comments preserve strings, escapes, BOM and CRLF',()=>{
  const expected={url:'https://example.org/a//b',block:'/* text */',quote:'a"//b',slash:'\\'};
  assert.deepEqual(parseRequestText('\uFEFF/*中文*/\r\n'+JSON.stringify(expected)+'// end'),expected);
  assert.deepEqual(parseRequestText('{ // comment\r\n "a": /* multiline\n */ [1,2]}'),{a:[1,2]});
});
test('JSONC rejects malformed input without evaluating code',()=>{
  for(const text of ['{/* unterminated','{"a":1,}','{"a":1/*x*/2}','{a:1}','{"a":"unterminated}','(()=>42)()'])assert.throws(()=>parseRequestText(text),e=>e.code==='INVALID_INPUT'&&e.path==='input_text');
  assert.throws(()=>parseRequestText(null),e=>e.code==='INVALID_INPUT');
});
test('annotated worksheet and strict JSON produce identical requests and deterministic physics',async()=>{
  const a=parseRequestText(await readFile(join(root,'assets/examples/worksheet-minimal.jsonc'),'utf8'));
  const b=JSON.parse(await readFile(join(root,'assets/examples/worksheet-minimal.json'),'utf8'));
  assert.deepEqual(a,b);delete a.search;delete b.search;
  const r=analyze(a);assert.deepEqual(r,analyze(b));assert.deepEqual(r,analyze(a));
  assert.ok(Math.abs(r.poses[0].holding_load[0]-25.1925268)<1e-10);
  assert.ok(Math.abs(r.poses[1].holding_load[1]+23.2713838)<1e-10);
});
test('CLI accepts JSONC and reports invalid text as structured input error',async()=>{
  const dir=await mkdtemp(join(tmpdir(),'statics-jsonc-'));
  try{
    const args=[join(root,'scripts/run.mjs'),'--input',join(root,'assets/examples/worksheet-minimal.jsonc'),'--output',join(dir,'result.json')];
    const run=spawnSync(process.execPath,args,{encoding:'utf8'});assert.equal(run.status,0,run.stderr);
    const actual=JSON.parse(await readFile(join(dir,'result.json'),'utf8'));
    const input=JSON.parse(await readFile(join(root,'assets/examples/worksheet-minimal.json'),'utf8'));
    assert.deepEqual(actual,analyze(input));
    const bad=join(dir,'bad.jsonc');await writeFile(bad,'{/* unterminated');
    const fail=spawnSync(process.execPath,[join(root,'scripts/run.mjs'),'--input',bad,'--output',join(dir,'result.json')],{encoding:'utf8'});
    assert.notEqual(fail.status,0);assert.equal(JSON.parse(fail.stderr).error.code,'INVALID_INPUT');assert.equal(JSON.parse(fail.stderr).error.path,'input_text');
    assert.deepEqual(JSON.parse(await readFile(join(dir,'result.json'),'utf8')),actual);
  }finally{await rm(dir,{recursive:true,force:true});}
});
