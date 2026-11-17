import sys; sys.path.insert(0,'/home/claude/leg'); sys.path.insert(0,'/home/claude/body')
import pickle, numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import body as B, legmodel as L
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
def draw(ax, names, axis, c, uv, col='#52514e'):
    for n in names:
        for a,b in section(P[n], axis, c): ax.plot([a[uv[0]],b[uv[0]]],[a[uv[1]],b[uv[1]]],color=col,lw=0.9)
R = lambda ax,x0,y0,w,h,c,lab=None,a=0.55: ax.add_patch(Rectangle((x0,y0),w,h,color=c,alpha=a,label=lab))
fig,axs=plt.subplots(2,2,figsize=(16,16))
# 1: side cut y=0 through neck + head + hat
ax=axs[0,0]; draw(ax,['torso','neck_tower','head','camera_hat'],1,0.3,(0,2))
R(ax,B.SGX0,B.SG_CASE_Z0,B.SG_L,22.7,'#2a78d6','SG90'); R(ax,-2.5,B.SG_CASE_Z0+22.7,5,B.SG_HORN_TOP-22.7,'#2a78d6')
R(ax,B.HX1-2-B.SCR_STACK,B.SCR_Z0,B.SCR_STACK,B.SCR_H,'#1baf7a','screen')
R(ax,B.CB_X0,B.HAT_Z0,1.6,50.2,'#222222','camera board'); R(ax,B.CB_X1,B.CAM_ZC-3.95,5.1,7.9,'#eb6834','camera'); R(ax,B.CB_X1,B.HAT_Z0+1,6.4,7,'#eda100','white plug')
R(ax,B.MIC_X-7.2,B.HZ1-2-1.5-1.6,14.4,1.6,'#7d5bd6','mic')
R(ax,B.AMP_X0+0.3,B.Z_TOP0-B.AMP_H-0.5,B.AMP_D,B.AMP_H,'#d6409f','amp'); R(ax,B.IX1-B.SPK_D,B.Z_L3+3,B.SPK_D,B.SPK_H,'#444','speaker')
ax.set_xlim(-35,35); ax.set_ylim(80,255); ax.set_aspect('equal'); ax.legend(loc='upper left',fontsize=9); ax.set_title('Side cut through the middle'); ax.grid(color='#eee')
# 2: front cut through the neck (x = 0) : tower legs, wire slots, head floor
ax=axs[0,1]; draw(ax,['torso','neck_tower','head','camera_hat'],0,-1.0,(1,2))
R(ax,-B.SG_W/2,B.SG_CASE_Z0,B.SG_W,22.7,'#2a78d6','SG90'); R(ax,-B.SCR_W/2,B.SCR_Z0,B.SCR_W,B.SCR_H,'#1baf7a','screen (behind)',0.15)
R(ax,-15.35,B.HAT_Z0,30.7,50.2,'#222222','camera board',0.25)
ax.set_xlim(-55,55); ax.set_ylim(80,255); ax.set_aspect('equal'); ax.legend(loc='upper left',fontsize=9); ax.set_title('Cut across (x = -1): neck snap legs + wire slots'); ax.grid(color='#eee')
# 3: deck tray from above with boards
ax=axs[1,0]; draw(ax,['torso'],2,B.Z_DECK1+5,(0,1)); draw(ax,['deck_tray'],2,B.Z_DECK1+B.TRAY_T+2.0,(0,1),'#e8182b'); draw(ax,['torso'],2,B.Z_DECK1-1.5,(0,1),'#bbbbbb')
R(ax,B.S3_X0,B.S3_Y0,B.S3_W,B.S3_L,'#222','ESP32-S3',0.3); R(ax,B.PCA_X0,-B.PCA_L/2,B.PCA_W,B.PCA_L,'#2a78d6','PCA9685',0.3)
for y in B.MINI_YS: R(ax,B.MINI_X0,y,B.MINI_L,B.MINI_T,'#1baf7a','Mini560' if y==B.MINI_YS[0] else None,0.5)
ax.set_xlim(-35,35); ax.set_ylim(-52,52); ax.set_aspect('equal'); ax.legend(loc='upper right',fontsize=9); ax.set_title('Deck from above (red = tray holders, grey = deck slots)'); ax.grid(color='#eee')
# 4: chest from above at the amp / arm level
ax=axs[1,1]; z=B.ARM_S-5; draw(ax,['torso'],2,z,(0,1))
R(ax,B.AMP_X0+0.3,-B.AMP_W/2,B.AMP_D,B.AMP_W,'#d6409f','amp'); R(ax,B.IX1-B.SPK_D,-B.SPK_W/2,B.SPK_D,B.SPK_W,'#444','speaker')
for sd in (1,-1): R(ax,-6.2,min(sd*21.2,sd*43.7),12.4,22.5,'#111','arm servo' if sd>0 else None,0.4)
ax.set_xlim(-35,35); ax.set_ylim(-52,52); ax.set_aspect('equal'); ax.legend(loc='upper right',fontsize=9); ax.set_title(f'Chest from above (z={z:.0f}): amp bay, speaker, arm servos'); ax.grid(color='#eee')
plt.tight_layout(); plt.savefig('verify_v2.png',dpi=70)
