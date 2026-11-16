import pickle, numpy as np
from PIL import Image, ImageDraw
def tris_of(polys,color,shift=(0,0,0)):
    T=[];C=[]
    for p in polys:
        v=np.array(p.v)+shift
        for k in range(1,len(v)-1): T.append([v[0],v[k],v[k+1]]); C.append(color)
    return np.array(T),np.array(C,float)
def render(groups,fname,azim=-35,elev=20,W=900,H=1000,title=None):
    T=np.concatenate([g[0] for g in groups]); C=np.concatenate([g[1] for g in groups])
    a,e=np.radians(azim),np.radians(elev)
    # camera: rotate world so view dir along -z_cam
    Rz=np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
    th=-(np.pi/2-e); Rx=np.array([[1,0,0],[0,np.cos(th),-np.sin(th)],[0,np.sin(th),np.cos(th)]])
    R=Rx@Rz
    P=T@R.T  # x right, y up, z toward viewer
    n=np.cross(T[:,1]-T[:,0],T[:,2]-T[:,0]); nl=np.linalg.norm(n,axis=1,keepdims=True); n=n/np.maximum(nl,1e-12)
    nc=n@R.T
    L=np.array([0.3,0.5,0.8]); L/=np.linalg.norm(L)
    shade=0.35+0.65*np.abs(nc@L)
    mn=P.reshape(-1,3).min(0); mx=P.reshape(-1,3).max(0)
    s=min((W-80)/(mx[0]-mn[0]),(H-120)/(mx[1]-mn[1]))
    X=(P[:,:,0]-mn[0])*s+40+((W-80)-(mx[0]-mn[0])*s)/2; Y=H-((P[:,:,1]-mn[1])*s+40+((H-120)-(mx[1]-mn[1])*s)/2); Z=P[:,:,2]
    zb=np.full((H,W),-1e9); img=np.ones((H,W,3))
    for i in range(len(T)):
        x=X[i];y=Y[i];z=Z[i]
        x0=max(int(np.floor(x.min())),0);x1=min(int(np.ceil(x.max())),W-1)
        y0=max(int(np.floor(y.min())),0);y1=min(int(np.ceil(y.max())),H-1)
        if x1<x0 or y1<y0: continue
        gx,gy=np.meshgrid(np.arange(x0,x1+1)+0.5,np.arange(y0,y1+1)+0.5)
        d=(y[1]-y[2])*(x[0]-x[2])+(x[2]-x[1])*(y[0]-y[2])
        if abs(d)<1e-9: continue
        l0=((y[1]-y[2])*(gx-x[2])+(x[2]-x[1])*(gy-y[2]))/d
        l1=((y[2]-y[0])*(gx-x[2])+(x[0]-x[2])*(gy-y[2]))/d
        l2=1-l0-l1
        m=(l0>=-1e-3)&(l1>=-1e-3)&(l2>=-1e-3)
        if not m.any(): continue
        zz=l0*z[0]+l1*z[1]+l2*z[2]
        sub=zb[y0:y1+1,x0:x1+1]; upd=m&(zz>sub)
        sub[upd]=zz[upd]; img[y0:y1+1,x0:x1+1][upd]=C[i]*shade[i]
    im=Image.fromarray((img*255).astype(np.uint8))
    if title: ImageDraw.Draw(im).text((20,15),title,fill=(40,40,40))
    im.save(fname)
