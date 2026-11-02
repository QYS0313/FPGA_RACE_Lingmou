"""Extract the user-supplied HTML's data and licensed Three.js without executing it."""
import argparse
import base64
import gzip
import hashlib
import json
from pathlib import Path
import re

ROOT=Path(__file__).resolve().parents[1]
def import_model(source):
    raw=source.read_bytes();html=raw.decode('utf-8-sig')
    module=re.search(r"const THREE=await import\('data:text/javascript;base64,([A-Za-z0-9+/=]+)'\)",html)
    payload=re.search(r'const data=(.*?);\s*\nconst scene=',html,re.S)
    if not module or not payload:raise ValueError('Expected the supplied embedded Three.js assembly format')
    data=json.loads(payload[1])
    if not data.get('meshes') or not data.get('groups'):raise ValueError('No assembly meshes')
    for mesh in data['meshes']:
        if len(mesh['positions'])%3 or len(mesh['indices'])%3:raise ValueError('Invalid geometry arrays')
        if max(mesh['indices'])>=len(mesh['positions'])//3:raise ValueError('Invalid vertex index')
    assets=ROOT/'Web/assets';assets.mkdir(parents=True,exist_ok=True)
    models=ROOT/'Models';models.mkdir(exist_ok=True)
    (models/'assembly2_original.html').write_bytes(raw)
    library=base64.b64decode(module[1],validate=True)
    if b'Three.js Authors' not in library[:200]:raise ValueError('Three.js license header missing')
    (assets/'three.module.js').write_bytes(library)
    encoded=json.dumps(data,ensure_ascii=False,separators=(',',':')).encode('utf-8')
    (assets/'assembly.json').write_bytes(encoded)
    for name in ('assembly.json','three.module.js'):
        f=assets/name;(assets/(name+'.gz')).write_bytes(gzip.compress(f.read_bytes(),compresslevel=9,mtime=0))
    manifest=dict(source_name=source.name,source_sha256=hashlib.sha256(raw).hexdigest(),
                  data_sha256=hashlib.sha256(encoded).hexdigest(),mesh_count=len(data['meshes']),
                  triangle_count=sum(len(m['indices'])//3 for m in data['meshes']),groups=data['groups'],
                  joints=dict(pan_pivot=[-19.579,0,-108.022],tilt_pivot=[35.055,3.098,37.831],
                              base=[3,8],pan=[4,5,7,10],tilt=[2,6,9,11,12,13,14,62,63,64]))
    (assets/'assembly_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    return manifest
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);args=p.parse_args()
    result=import_model(args.source)
    print(json.dumps({k:v for k,v in result.items() if k not in ('groups','joints')},ensure_ascii=True))
