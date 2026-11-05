import {writeFile} from 'node:fs/promises';
import {schemas} from './schemas.mjs';
for(const [name,schema] of Object.entries(schemas))await writeFile(new URL(`./${name}.schema.json`,import.meta.url),JSON.stringify({$schema:'https://json-schema.org/draft/2020-12/schema',...schema},null,2)+'\n');
