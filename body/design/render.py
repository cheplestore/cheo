import pickle, sys, numpy as np, matplotlib
matplotlib.use('Agg'); sys.path.insert(0,'/home/claude/leg'); sys.path.insert(0,'/tmp/cad')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import body as B
d = pickle.load(open('body.pkl','rb'))
legs = pickle.load(open('/home/claude/leg/solids.pkl','rb'))
Lg = np.array([0.5,-0.6,0.65]); Lg/=np.linalg.norm(Lg)
def polys_of(s): return [np.array(p.v) for p in (s.polys if hasattr(s,'polys') else s)]
def norm(P):
    n=np.cross(P[1]-P[0],P[2]-P[0]); l=np.linalg.norm(n); return n/l if l>0 else n
def coll(polys,color,mirror=False,shift=(0,0,0)):
    P=[ (p*np.array([1,-1,1]) if mirror else p)+shift for p in polys]
    N=np.array([norm(p)*( [1,-1,1] if mirror else 1) for p in polys])
    if mirror: N=-N
    sh=0.4+0.6*np.clip(N@Lg,0,1)
    return [(color[0]*s,color[1]*s,color[2]*s,1) for s in sh], P
Y=(0.98,0.80,0.25); W=(0.92,0.92,0.95); G=(0.55,0.58,0.62)
def view(items,fname,elev,azim,title,zoom=1.0):
    fig=plt.figure(figsize=(8,10)); ax=fig.add_subplot(111,projection='3d'); allp=[]
    cols=[]
    for polys,col,mir,sh in items:
        c,P=coll(polys,col,mir,sh); allp+=P; cols+=c
    ax.add_collection3d(Poly3DCollection(allp,facecolors=cols,edgecolors='none'))
    V=np.concatenate(allp); mn,mx=V.min(0),V.max(0); c0=(mn+mx)/2; r=(mx-mn).max()/2*zoom
    ax.set_xlim(c0[0]-r,c0[0]+r); ax.set_ylim(c0[1]-r,c0[1]+r); ax.set_zlim(c0[2]-r,c0[2]+r)
    ax.set_box_aspect((1,1,1)); ax.view_init(elev,azim); ax.axis('off'); fig.suptitle(title,fontsize=13)
    plt.tight_layout(); plt.savefig(fname,dpi=100); plt.close()
leg_items=[]
for n,s in legs.items():
    if n=='pelvis_test_mount': continue
    col = W if n in ('thigh','shin','foot') else Y
    leg_items += [(polys_of(s),col,False,(0,0,0)), (polys_of(s),col,True,(0,0,0))]
body_items=[(polys_of(d['pelvis']),G,False,(0,0,0)),(polys_of(d['torso']),Y,False,(0,0,0)),(polys_of(d['head']),W,False,(0,0,0)),
            (polys_of(d['arm_L']),W,False,(0,0,0)),(polys_of(d['arm_R']),W,False,(0,0,0))]
view(leg_items+body_items,'cheo_body_front.png',12,-35,'Cheo biped - front')
view(body_items,'cheo_body_back_open.png',22,145,'Back, covers off: battery bay (bottom), deck, chest',zoom=0.75)
