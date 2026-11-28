import {accessSync,constants} from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {runProcess} from './execution-adapter.mjs';
const local=fileURLToPath(new URL('../../.runtime/codex-cli/node_modules/.bin/codex',import.meta.url));
export const codexBridge=fileURLToPath(new URL('./adapters/codex-selection-bridge.mjs',import.meta.url));
export function findCodex(env=process.env){
  const candidates=env.ARM_CODEX_CLI?[env.ARM_CODEX_CLI]:[local,...(env.PATH||'').split(path.delimiter).filter(Boolean).map(directory=>path.join(directory,'codex'))];
  return candidates.find(command=>{try{accessSync(command,constants.X_OK);return true;}catch{return false;}})??null;
}
const checks=new Map();
export async function codexReadiness(env=process.env){
  const command=findCodex(env);
  if(!command)return {configured:false,provider:'codex',message:'未找到本机 Codex CLI，请安装或配置服务器的 Codex 路径。'};
  const key=JSON.stringify([command,env.CODEX_HOME,env.ARM_CODEX_MODEL]);
  const cached=checks.get(key);if(cached&&Date.now()-cached.time<15000)return cached.promise;
  const promise=(async()=>{
    try{
      const version=await runProcess(command,['--version'],{timeoutMs:5000,env});
      const match=(version.stdout+version.stderr).match(/codex-cli\s+(\d+)\.(\d+)\.(\d+)/);
      if(!match||Number(match[1])===0&&Number(match[2])<172)return {configured:false,provider:'codex',message:'本机 Codex CLI 版本过旧，请升级至 0.172.0 或更新版本。'};
      await runProcess(command,['login','status'],{timeoutMs:5000,env});
      return {configured:true,provider:'codex',model:env.ARM_CODEX_MODEL||'gpt-6.2-sol',message:'已连接本机 Codex，可临时进行构型选择。'};
    }catch{return {configured:false,provider:'codex',message:'本机 Codex 尚未登录或无法启动，请在服务器运行 Codex 登录并检查运行权限。'};}
  })();
  checks.set(key,{time:Date.now(),promise});return promise;
}
