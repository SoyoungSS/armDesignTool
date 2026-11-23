"""Shared artifact fingerprints and JSON result writer."""
import hashlib
import json
from pathlib import Path

from .math3d import serializable


def write_json(path,obj):
    path.parent.mkdir(parents=True,exist_ok=True); path.write_text(json.dumps(serializable(obj),ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')

def hashes(run):
    return {p.relative_to(run).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(run.rglob('*')) if p.is_file() and p.name not in ['manifest.json','validation.json','report.md']}
