import sys; sys.path.insert(0,'.')
import numpy as np, sim
from sim import *
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MPoly

def peak(r):
    jn={}
    for x in r:
        for (s,j),v in x['tq'].items(): jn[j]=max(jn.get(j,0),v)
    return jn
# Plan A as drawn (hips 26 mm, no lean -> feet flat)
sim.hipY=0.026
rA,MA=run(CFG_A,shift=0.0)
rA2,_=run(CFG_A,shift=0.026)                 # A trying to lean with hip roll -> foot tilts
tiltA=max(x['tilt'] for x in rA2)
# Plan B: ankle roll, hips 18 mm, feet 26 mm apart from center
sim.hipY=0.018
rB,MB=run(CFG_B,footY=0.026)
rBfast,_=run(CFG_B,footY=0.026,T=0.7)
pA,pB=peak(rA),peak(rB)

# ---- push test (standing still on both feet) ----
def push(cfg,footY):
    zp=L_hip+(Lt+Ls)*np.cos(np.radians(15))+cfg['gap']+cfg['h_ar']
    P=np.array([0.0,0,zp]); legs={}
    for s_,sg in (('L',1),('R',-1)):
        sole=np.array([0,sg*footY,0.0]); L=leg_ik(P+np.array([0,sg*sim.hipY,0]),sole,cfg); L['sole']=sole; legs[s_]=L
    ms=masses(P,legs,0,cfg); m=np.array([x[1] for x in ms]); p=np.array([x[2] for x in ms])
    com=(m[:,None]*p).sum(0)/m.sum(); M=m.sum()
    poly=hull(np.vstack([foot_poly(legs['L']['sole']),foot_poly(legs['R']['sole'])]))
    xs,ys=poly[:,0],poly[:,1]
    d={'forward':xs.max()-com[0],'backward':com[0]-xs.min(),'sideways':ys.max()-com[1]}
    return com,M,{k:M*G*v/com[2] for k,v in d.items()},d
comB,MBs,pushB,distB=push(CFG_B,0.026)

# ---- fall estimate ----
head_h=comB[2]+0.200-0.05+0.05
v=np.sqrt(2*G*(comB[2]+0.2))                   # head speed when he tips over (rough)
F_fall=0.115*v**2/(2*0.004)                     # head+neck stopping in ~4 mm

# ---------------- REPORT FIGURE ----------------
plt.rcParams.update({'font.size':11,'text.color':'#dfe8ff','axes.labelcolor':'#cfe0ff','xtick.color':'#9fb3d6','ytick.color':'#9fb3d6',
                     'axes.edgecolor':'#3a4a70','axes.facecolor':'#0f1630','figure.facecolor':'#0a0f22'})
fig=plt.figure(figsize=(16,11.5))
fig.suptitle("Cheo Biped: walking simulation check",fontsize=24,fontweight="bold",color="#e8f4ff",y=0.995)
fig.text(0.5,0.925,f"6 slow steps, 20 mm each, 1.6 s per step · estimated weight {MB*1000:.0f} g · MG90S @5V: stall {STALL:.2f} N·m, comfortable {COMFY:.2f} N·m",
         ha='center',color='#8fa3c7',fontsize=12)

# 1 top view
ax=fig.add_axes([0.04,0.50,0.44,0.40]); ax.set_title("Top view: footprints and balance point (ZMP)",color='#e8f4ff',fontsize=14)
drawn=set()
for r,col,lab in ((rB,'#39d98a','Plan B balance path'),):
    for x in r[::20]:
        f=x['f']
        for s_ in ('L','R'):
            so=f['legs'][s_]['sole']
            if so[2]<0.0005:
                key=(round(so[0],3),round(so[1],3))
                if key not in drawn:
                    drawn.add(key); ax.add_patch(MPoly(foot_poly(so)*1000,closed=True,fc='#26345c',ec='#5f7bb0',lw=1))
z=np.array([x['zmp'] for x in rB])*1000; ax.plot(z[:,0],z[:,1],color='#39d98a',lw=2.2,label='Plan B balance point (stays on a foot)')
zA=np.array([x['zmp'] for x in rA])*1000; ax.plot(zA[:,0],zA[:,1],color='#ff5a6e',lw=2,ls='--',label='Plan A balance point')
bad=np.array([x['zmp'] for x in rA if x['margin']<0])*1000
if len(bad): ax.scatter(bad[::8,0],bad[::8,1],s=14,color='#ff5a6e',zorder=5,label='Plan A: off the foot = tips over')
ax.set_xlabel("forward (mm)"); ax.set_ylabel("left / right (mm)"); ax.set_aspect('equal')
ax.set_xlim(-40,110); ax.set_ylim(-60,60); ax.legend(loc='lower right',fontsize=9,facecolor='#0f1630',edgecolor='#3a4a70',labelcolor='#dfe8ff')
ax.grid(color='#1d2a4a')

# 2 margin
ax=fig.add_axes([0.55,0.50,0.42,0.40]); ax.set_title("Balance margin (how far the balance point is inside the foot)",color='#e8f4ff',fontsize=14)
t=[x['t'] for x in rB]
ax.plot(t,[x['margin']*1000 for x in rB],color='#39d98a',lw=2,label='Plan B, 1.6 s steps')
ax.plot([x['t'] for x in rBfast],[x['margin']*1000 for x in rBfast],color='#fcd34d',lw=1.5,label='Plan B, 0.7 s steps (too fast)')
ax.plot([x['t'] for x in rA],[x['margin']*1000 for x in rA],color='#ff5a6e',lw=2,ls='--',label='Plan A')
ax.axhline(0,color='#ff8a8a',lw=1); ax.text(0.1,-2.5,"below 0 = falls over",color='#ff8a8a',fontsize=10,va='top')
ax.set_xlabel("time (s)"); ax.set_ylabel("mm"); ax.set_ylim(-35,45); ax.grid(color='#1d2a4a')
ax.legend(loc='upper right',fontsize=9,facecolor='#0f1630',edgecolor='#3a4a70',labelcolor='#dfe8ff')

# 3 torques
ax=fig.add_axes([0.06,0.07,0.40,0.34]); ax.set_title("Hardest pull on each servo while walking",color='#e8f4ff',fontsize=14)
names=['hip roll','hip pitch','knee','ankle pitch','ankle roll']
xa=np.arange(len(names))
va=[pA.get(n,0) for n in names]; vb=[pB.get(n,0) for n in names]
ax.bar(xa-0.2,va,0.38,color='#ff5a6e',label='Plan A (hips 26 mm)')
ax.bar(xa+0.2,vb,0.38,color='#39d98a',label='Plan B (hips 18 mm)')
ax.axhline(STALL,color='#ff8a8a',ls='--'); ax.text(4.45,STALL+0.004,'stall (servo gives up)',color='#ff8a8a',ha='right',fontsize=9)
ax.axhline(COMFY,color='#fcd34d',ls='--'); ax.text(4.45,COMFY+0.004,'comfortable',color='#fcd34d',ha='right',fontsize=9)
ax.set_xticks(xa); ax.set_xticklabels(names); ax.set_ylabel("N·m"); ax.set_ylim(0,0.21); ax.grid(axis='y',color='#1d2a4a')
ax.text(4.2,0.012,"Plan A has\nno ankle roll",color='#ff8a8a',ha='center',fontsize=8)
ax.legend(loc='upper center',fontsize=9,facecolor='#0f1630',edgecolor='#3a4a70',labelcolor='#dfe8ff')

# 4 summary
ax=fig.add_axes([0.52,0.05,0.46,0.38]); ax.axis('off')
lines=[("RESULTS",'#3cf',15,'bold'),
 (f"Plan A (4 servos per leg): FALLS. When he lifts a foot, his balance point is {abs(min(x['margin'] for x in rA))*1000:.0f} mm",'#ff8a8a',11,'normal'),
 (f"   off the other foot. Leaning with the hip instead tips that foot {tiltA:.0f}° onto its edge.",'#ff8a8a',11,'normal'),
 (f"Plan B (5 per leg, + ankle roll): STAYS UP with {min(x['margin'] for x in rB)*1000:.0f} mm to spare at 1.6 s per step.",'#39d98a',11,'normal'),
 ("   Faster than ~1 s per step he starts to tip; 0.7 s is right at the edge.",'#cfe0ff',11,'normal'),
 ("",None,6,'normal'),
 ("WEAKEST POINTS (most likely to fail first)",'#3cf',15,'bold'),
 (f"1. Hip roll servos: {pB['hip roll']:.2f} N·m = {100*pB['hip roll']/STALL:.0f}% of stall (they hold the whole body sideways).",'#fcd34d',11,'normal'),
 (f"    With hips 26 mm apart (Plan A) it is {100*pA['hip roll']/STALL:.0f}%: keep the hip servos close together.",'#cfe0ff',11,'normal'),
 (f"2. Falls: his head hits at about {v:.1f} m/s, roughly {F_fall:.0f} N on the head hinge and screen.",'#fcd34d',11,'normal'),
 ("    Thin printed hinge pins can snap. Use M3 screws as pins + let him go limp when falling.",'#cfe0ff',11,'normal'),
 ("3. Printed legs: fine. Even at servo stall the stress is ~6 MPa vs ~20 MPa for PLA",'#cfe0ff',11,'normal'),
 ("    (print brackets flat so the layers don't peel apart).",'#cfe0ff',11,'normal'),
 ("",None,6,'normal'),
 ("PUSH TEST (standing still)",'#3cf',15,'bold'),
 (f"Falls over if pushed with about: forward {pushB['forward']:.1f} N, backward {pushB['backward']:.1f} N, sideways {pushB['sideways']:.1f} N",'#cfe0ff',11,'normal'),
 (f"(1 N = a 100 g weight). Battery pack on his back moves his balance {abs(comB[0])*1000:.0f} mm backward.",'#cfe0ff',11,'normal')]
y=0.98
for txt,c,sz,w in lines:
    if c: ax.text(0,y,txt,color=c,fontsize=sz,fontweight=w,va='top',transform=ax.transAxes)
    y-=0.058 if sz>6 else 0.02
fig.savefig('cheo_sim_report.png',dpi=100)
print("A margin",min(x['margin'] for x in rA),"tilt",tiltA,"B margin",min(x['margin'] for x in rB))
print("peaks A",pA); print("peaks B",pB); print("push",pushB,"com",comB,"M",MBs,"v",v,"F",F_fall)
import pickle; pickle.dump(rB,open('rB.pkl','wb'))
