import sys; sys.path.insert(0,'/home/claude/leg'); sys.path.insert(0,'/home/claude/body')
import pickle, numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon
import body as B
exec(open('verify2.py').read().split("fig,axs")[0].split("P = pickle")[1].join(["P = pickle",""]) if False else "")
P = pickle.load(open('body.pkl','rb'))
def section(polys, axis, c):
    segs=[]
    for p in polys:
        v=np.array(p.v)
        for i in range(1,len(v)-1):
            tri=v[[0,i,i+1]]; d=tri[:,axis]-c; pts=[]
            for a,b in ((0,1),(1,2),(2,0)):
                if (d[a]>0)!=(d[b]>0): t=d[a]/(d[a]-d[b]); pts.append(tri[a]+t*(tri[b]-tri[a]))
            if len(pts)==2: segs.append(pts)
    return segs
def draw(ax, n, axis, c, uv, col, lw=1):
    for a,b in section(P[n], axis, c): ax.plot([a[uv[0]],b[uv[0]]],[a[uv[1]],b[uv[1]]],color=col,lw=lw)
fig,axs=plt.subplots(1,3,figsize=(21,8))
ax=axs[0]; draw(ax,'neck_tower',2,B.TW_Z0+B.TW_BASE+8,(0,1),'#e8182b',1.2); draw(ax,'neck_tower',2,B.TW_Z0+1.5,(0,1),'#999',0.8); draw(ax,'torso',2,B.Z_TOP1-1.2,(0,1),'#2a78d6',0.8)
ax.add_patch(Rectangle((B.SGX0,-B.SG_W/2),B.SG_L,B.SG_W,color='#2a78d6',alpha=0.35,label='SG90'))
for th,c in ((0,'#1baf7a'),(90,'#eda100'),(-90,'#7d5bd6')):
    r=np.radians(th); Rm=np.array([[np.cos(r),-np.sin(r)],[np.sin(r),np.cos(r)]])
    q=np.array([[B.HX0+3,-24],[-9.4,-24],[-9.4,24],[B.HX0+3,24]])@Rm.T
    ax.add_patch(Polygon(q,fill=False,ec=c,lw=2,ls='--',label=f'head floor opening, head turned {th}'))
    hq=np.array([[B.HX0,-B.HW],[B.HX1,-B.HW],[B.HX1,B.HW],[B.HX0,B.HW]])@Rm.T
    if th==0: ax.add_patch(Polygon(hq,fill=False,ec='#888',lw=1,ls=':',label='head outline'))
ax.set_aspect('equal'); ax.set_xlim(-40,40); ax.set_ylim(-40,40); ax.legend(fontsize=8,loc='lower right'); ax.grid(color='#eee')
ax.set_title('Neck box from above: red = box + cradle, blue = torso top openings')
ax=axs[1]; draw(ax,'torso',1,12.0,(0,2),'#52514e'); draw(ax,'neck_tower',1,12.0,(0,2),'#e8182b',1.2); draw(ax,'head',1,12.0,(0,2),'#2a78d6')
ax.set_aspect('equal'); ax.set_xlim(-35,35); ax.set_ylim(80,205); ax.grid(color='#eee'); ax.set_title('Side cut at y = 12 (through the wire path)')
ax.annotate('', xy=(0,60+30), xytext=(-12,198), arrowprops=dict(arrowstyle='->',color='#e8182b',lw=3, connectionstyle='arc3,rad=0.25'))
ax.text(-33,96,'wires: head -> neck box -> chest -> S3',color='#e8182b',fontsize=11)
ax=axs[2]; draw(ax,'torso',0,0.0,(1,2),'#52514e'); draw(ax,'neck_tower',0,0.0,(1,2),'#e8182b',1.2); draw(ax,'head',0,-12.0,(1,2),'#2a78d6')
ax.set_aspect('equal'); ax.set_xlim(-55,55); ax.set_ylim(80,205); ax.grid(color='#eee'); ax.set_title('Front cut (x = 0 body, x = -12 head)')
plt.tight_layout(); plt.savefig('verify_neck.png',dpi=65)
print('box x', round(B.NB_X0,1), round(B.NB_X1,1), 'y +-', B.NB_HW, ' box top', round(B.HEAD_Z0-1.5,1), ' head floor', B.HEAD_Z0)
