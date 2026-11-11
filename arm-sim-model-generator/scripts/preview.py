"""Optional offline orthographic preview (pip install Pillow); not a web viewer."""
import argparse, math, sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from arm_sim.model import read_input
from arm_sim.math3d import forward
from arm_sim.geometry import read_stl
from arm_sim.surfaces import normals

def render(run,output,pose='initial',view='iso'):
    n=read_input(run/'report'/'normalized_model.json')
    q=n['poses'].get(pose) if pose in ['initial','geometric_zero'] else next((p['q'] for p in n['poses']['acceptance'] if p['name']==pose),None)
    if q is None: raise ValueError('unknown pose '+pose)
    poses=forward(n,q,world=False)
    # Look along [1.2,-1.6,1.0], with Z displayed upward.
    eye=np.array({'iso':[1.2,-1.6,1.0],'front':[0.,-1.,.12],'side':[1.,0.,.12]}[view]); eye/=np.linalg.norm(eye); right=np.cross([0,0,1],eye); right/=np.linalg.norm(right); up=np.cross(eye,right); camera=np.array([right,up,eye])
    entries=[]; bounds=[]
    light_dir=np.array([.2,-.4,.89]); light_dir/=np.linalg.norm(light_dir)
    for link,g in n['geometry'].items():
        T=poses[link]
        for p in g['parts']:
            tri=read_stl(run/'meshes'/p['mesh']); world=tri@T[:3,:3].T+T[:3,3]; projected=world@camera.T; bounds.append(projected.reshape(-1,3))
            verts,indices=np.unique(world.reshape(-1,3),axis=0,return_inverse=True)
            vn=normals(verts,indices.reshape(-1,3))[indices].reshape(-1,3,3)
            for w,v,nn in zip(world,projected,vn):
                normal=np.cross(w[1]-w[0],w[2]-w[0]); normal/=np.linalg.norm(normal)
                lighting=.48+.52*np.maximum(0,nn@light_dir)
                colors=255*lighting[:,None]*np.array(p['color'][:3]); entries.append((v[:,2].mean(),v,colors))
    V=np.concatenate(bounds); low=V[:,:2].min(0); high=V[:,:2].max(0); center=(low+high)/2
    scale=min(1100/max(high[0]-low[0],.01),550/max(high[1]-low[1],.01))
    pixels=np.empty((800,1300,3),dtype=np.uint8); pixels[:]=[242,245,247]; depth=np.full((800,1300),-np.inf)
    for _,v,color in entries:
        xy=(v[:,:2]-center)*scale; xy[:,0]+=650; xy[:,1]=430-xy[:,1]
        x0=max(0,int(np.floor(xy[:,0].min()))); x1=min(1299,int(np.ceil(xy[:,0].max())))
        y0=max(0,int(np.floor(xy[:,1].min()))); y1=min(799,int(np.ceil(xy[:,1].max())))
        if x0>x1 or y0>y1: continue
        a,b,c=xy; den=(b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
        if abs(den)<1e-9: continue
        xx,yy=np.meshgrid(np.arange(x0,x1+1)+.5,np.arange(y0,y1+1)+.5)
        w0=((b[1]-c[1])*(xx-c[0])+(c[0]-b[0])*(yy-c[1]))/den
        w1=((c[1]-a[1])*(xx-c[0])+(a[0]-c[0])*(yy-c[1]))/den; w2=1-w0-w1
        z=w0*v[0,2]+w1*v[1,2]+w2*v[2,2]; tile=depth[y0:y1+1,x0:x1+1]
        mask=(w0>=-1e-9)&(w1>=-1e-9)&(w2>=-1e-9)&(z>tile)
        tile[mask]=z[mask]
        shaded=w0[:,:,None]*color[0]+w1[:,:,None]*color[1]+w2[:,:,None]*color[2]
        pixels[y0:y1+1,x0:x1+1][mask]=np.clip(shaded[mask],0,255).astype(np.uint8)
    img=Image.fromarray(pixels); draw=ImageDraw.Draw(img)
    def font(size):
        try: return ImageFont.truetype('arial.ttf',size)
        except OSError: return ImageFont.load_default()
    draw.text((35,25),n['model']['name']+' | '+pose+' | generated meshes (m)',fill=(35,45,55),font=font(26))
    draw.text((35,65),'Generated link-local meshes | neutral shells / dark covers / metal flanges',fill=(75,85,95),font=font(20))
    draw.text((35,750),'Concept geometry; collision and physics provenance are in the report.',fill=(75,85,95),font=font(18))
    output.parent.mkdir(parents=True,exist_ok=True); img.save(output)

if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('--run',type=Path,required=True); p.add_argument('--output',type=Path,required=True); p.add_argument('--pose',default='initial',help='initial, geometric_zero or an acceptance pose name'); p.add_argument('--view',choices=['iso','front','side'],default='iso'); a=p.parse_args(); render(a.run,a.output,a.pose,a.view)
