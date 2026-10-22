// JSONC text boundary shared by CLI, HTTP and browser; no evaluation.
export function parseRequestText(text) {
  if (typeof text !== 'string') return fail('Input must be text');
  let clean='', state='normal';
  for(let i=0;i<text.length;i++){
    const c=text[i],n=text[i+1];
    if(state==='string'){
      clean+=c;
      if(c==='\\'){if(i+1<text.length)clean+=text[++i];}
      else if(c==='"')state='normal';
    }else if(state==='line'){
      clean+=c==='\n'||c==='\r'?c:' ';
      if(c==='\n'||c==='\r')state='normal';
    }else if(state==='block'){
      if(c==='*'&&n==='/'){clean+='  ';i++;state='normal';}
      else clean+=c==='\n'||c==='\r'?c:' ';
    }else if(c==='"'){clean+=c;state='string';}
    else if(c==='/'&&n==='/'){clean+='  ';i++;state='line';}
    else if(c==='/'&&n==='*'){clean+='  ';i++;state='block';}
    else clean+=i===0&&c==='\uFEFF'?' ':c;
  }
  if(state==='block')return fail('Unterminated block comment');
  try{return JSON.parse(clean);}catch(e){return fail(e.message);}
}
function fail(message){const e=new Error(message);e.code='INVALID_INPUT';e.path='input_text';throw e;}
