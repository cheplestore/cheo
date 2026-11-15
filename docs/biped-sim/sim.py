import numpy as np
G=9.81
# ---------------- robot model (meters, kg) ----------------
SERVO=0.0134                       # MG90S mass
STALL=0.183                        # MG90S stall torque @5V (1.87 kg*cm)
COMFY=0.09                         # ~50% of stall: comfortable all-day holding
hipY=0.026; L_hip=0.020; Lt=0.045; Ls=0.045
foot_len=0.070; foot_w=0.045; foot_x0=-0.030   # foot from 30 mm behind ankle to 40 mm in front
upper=[  # name, mass, offset from pelvis (x fwd, y left, z up)
 ("head",0.045,(0.000,0,0.200)),("neck",0.070,(0.000,0,0.140)),
 ("body+electronics",0.110,(0.005,0,0.050)),("2x18650 pack",0.114,(-0.040,0,0.050))]
def make_cfg(name, ankle_roll, arm_servos):
    return dict(name=name, ankle_roll=ankle_roll, arm_m=0.016+arm_servos*SERVO,
                gap=0.018 if ankle_roll else 0.022, h_ar=0.018 if ankle_roll else 0.0)
CFG_A=make_cfg("Plan A: 4 servos/leg (no ankle roll), 2/arm", False, 2)
CFG_B=make_cfg("Plan B: 5 servos/leg (+ankle roll), 1/arm", True, 1)

def leg_ik(H, sole, cfg):
    """H = hip-roll axis point, sole = foot sole center. returns points + angles"""
    A = sole + np.array([0,0,cfg['h_ar']])          # ankle-roll axis (or sole if none)
    d = A - H
    phi = np.arctan2(d[1], -d[2])                   # leg roll (sideways tilt)
    u = np.array([0, d[1], d[2]]); u/=np.linalg.norm(u)   # tilted "down" direction
    e = np.array([1.0,0,0])
    Hp = H + u*L_hip                                 # hip-pitch axis
    D = np.linalg.norm([d[1],d[2]]) - L_hip - cfg['gap']
    X = d[0]
    r = np.hypot(X, D); r=min(r, Lt+Ls-1e-6)
    cosk=(Lt**2+Ls**2-r**2)/(2*Lt*Ls); knee=np.pi-np.arccos(np.clip(cosk,-1,1))
    a=np.arctan2(X,D); b=np.arccos(np.clip((Lt**2+r**2-Ls**2)/(2*Lt*r),-1,1))
    th=a+b                                            # thigh angle from tilted vertical (knee forward)
    K = Hp + e*Lt*np.sin(th) + u*Lt*np.cos(th)
    Ap = Hp + e*X + u*D                               # ankle-pitch axis
    return dict(H=H,Hp=Hp,K=K,Ap=Ap,A=A,phi=phi,knee=knee,hip=th,ankle=-(th-knee))

def masses(P, legs, arm_swing, cfg):
    """list of (name, mass, pos, tag) tag=('L'|'R', chain index) for leg parts"""
    out=[(n,m,P+np.array(o),None) for n,m,o in upper]
    for s,sg in (('L',1),('R',-1)):
        sh=P+np.array([0,sg*0.068,0.085]); ang=arm_swing*(-sg)
        out.append((f"arm{s}",cfg['arm_m'],sh+np.array([0.035*np.sin(ang),0,-0.035*np.cos(ang)]),None))
        L=legs[s]
        parts=[(SERVO,L['H']),(SERVO,L['Hp']),(0.008,(L['Hp']+L['K'])/2),(SERVO,L['K']),
               (0.008,(L['K']+L['Ap'])/2),(SERVO,L['Ap'])]
        if cfg['ankle_roll']: parts.append((SERVO,L['A']))
        parts.append((0.015,L['sole']+np.array([0.005,0,0.004])))
        for i,(m,p) in enumerate(parts): out.append((f"leg{s}{i}",m,np.array(p,float),(s,i)))
    return out

def foot_poly(sole):
    x0=sole[0]+foot_x0; x1=x0+foot_len; y0=sole[1]-foot_w/2; y1=y0+foot_w
    return np.array([[x0,y0],[x1,y0],[x1,y1],[x0,y1]])

def hull(pts):
    pts=sorted(map(tuple,pts))
    def cr(o,a,b): return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lo=[];up=[]
    for p in pts:
        while len(lo)>=2 and cr(lo[-2],lo[-1],p)<=0: lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up)>=2 and cr(up[-2],up[-1],p)<=0: up.pop()
        up.append(p)
    return np.array(lo[:-1]+up[:-1])

def margin(pt, poly):
    """signed distance from pt to polygon edge (+ inside)"""
    n=len(poly); best=1e9; inside=True
    for i in range(n):
        a=poly[i]; b=poly[(i+1)%n]; e=b-a; nrm=np.array([-e[1],e[0]])/np.linalg.norm(e)  # inward for CCW
        dist=np.dot(pt-a,nrm); best=min(best,dist)
    return best

# ---------------- gait ----------------
def smooth(q): q=np.clip(q,0,1); return q*q*(3-2*q)

def run(cfg, step=0.020, lift=0.015, T=1.6, nsteps=6, shift=None, dt=0.005, crouch=15, footY=None):
    if footY is None: footY=hipY
    zp = L_hip + (Lt+Ls)*np.cos(np.radians(crouch)) + cfg['gap'] + cfg['h_ar']
    if shift is None: shift = footY if cfg['ankle_roll'] else 0.0
    t=np.arange(0,nsteps*T,dt)
    fx={'L':0.0,'R':0.0}; fy={'L':footY,'R':-footY}
    frames=[]
    cur=-1
    for ti in t:
        n=int(ti//T); ph=(ti%T)/T
        swing='L' if n%2==0 else 'R'; stance='R' if swing=='L' else 'L'
        if n!=cur:                                   # new step: remember start/target
            cur=n; start=fx[swing]; target=fx[stance]+step
            if n>0: fx[other]=prev_target
        other=swing; prev_target=target
        sgn = 1 if stance=='L' else -1
        q=smooth((ph-0.3)/0.4)
        sx=start+(target-start)*q
        sz=lift*np.sin(np.pi*np.clip((ph-0.3)/0.4,0,1)) if 0.3<ph<0.7 else 0.0
        soles={stance:np.array([fx[stance],fy[stance],0.0]), swing:np.array([sx,fy[swing],sz])}
        mid0=(start+fx[stance])/2; mid1=(target+fx[stance])/2; over=fx[stance]+0.008
        if ph<0.3: px=mid0+(over-mid0)*smooth(ph/0.3)
        elif ph<0.7: px=over
        else: px=over+(mid1-over)*smooth((ph-0.7)/0.3)
        py=shift*sgn*np.sin(np.pi*ph)
        P=np.array([px,py,zp])
        legs={}
        for s_ in ('L','R'):
            H=P+np.array([0,hipY if s_=='L' else -hipY,0])
            L=leg_ik(H,soles[s_],cfg); L['sole']=soles[s_]; legs[s_]=L
        arm=0.35*np.sin(np.pi*ph)*sgn
        frames.append(dict(t=ti,P=P,legs=legs,swing=swing,stance=stance,ph=ph,
                           ms=masses(P,legs,arm,cfg),single=sz>0.0005))
    # accelerations by finite differences
    pos=np.array([[m[2] for m in f['ms']] for f in frames])   # (T, N, 3)
    acc=np.gradient(np.gradient(pos,dt,axis=0),dt,axis=0)
    mass=np.array([m[1] for m in frames[0]['ms']])
    res=[]
    for k,f in enumerate(frames):
        p=pos[k]; a=acc[k]
        com=(mass[:,None]*p).sum(0)/mass.sum()
        den=(mass*(a[:,2]+G)).sum()
        zmp=np.array([(mass*(p[:,0]*(a[:,2]+G)-p[:,2]*a[:,0])).sum()/den,
                      (mass*(p[:,1]*(a[:,2]+G)-p[:,2]*a[:,1])).sum()/den])
        st=f['legs'][f['stance']]['sole']; sw=f['legs'][f['swing']]['sole']
        poly=foot_poly(st) if f['single'] else hull(np.vstack([foot_poly(st),foot_poly(sw)]))
        tilt=np.degrees(abs(f['legs'][f['stance']]['phi'])) if not cfg['ankle_roll'] else 0.0
        # ---- joint torques ----
        F=mass[:,None]*(np.array([0,0,-G])-a)                  # load each mass puts on the robot
        # weight share of each leg (by where the ZMP sits between the feet)
        if f['single']: share={f['stance']:1.0,f['swing']:0.0}
        else:
            yl=f['legs']['L']['sole'][1]; yr=f['legs']['R']['sole'][1]
            wl=np.clip((zmp[1]-yr)/(yl-yr),0,1); share={'L':wl,'R':1-wl}
        tq={}
        for s_ in ('L','R'):
            L=f['legs'][s_]
            joints=[('hip roll',L['H'],np.array([1,0,0]),0),('hip pitch',L['Hp'],np.array([0,1,0]),1),
                    ('knee',L['K'],np.array([0,1,0]),3),('ankle pitch',L['Ap'],np.array([0,1,0]),5)]
            if cfg['ankle_roll']: joints.append(('ankle roll',L['A'],np.array([1,0,0]),6))
            for jn,J,ax,ci in joints:
                # load from everything "above" this joint (weighted by this leg's share)
                above=np.zeros(3); below=np.zeros(3)
                for i,(nm,m,pp,tag) in enumerate(f['ms']):
                    r=np.cross(p[i]-J,F[i])
                    if tag is not None and tag[0]==s_:
                        if tag[1]>ci: below+=r              # parts of this leg below the joint
                        else: above+=r
                    else: above+=r
                T_=abs(np.dot(ax, share[s_]*above + (1-share[s_])*(-below) if share[s_]>0 else -below))
                tq[(s_,jn)]=T_
        res.append(dict(t=f['t'],com=com,zmp=zmp,poly=poly,margin=margin(zmp,poly),tilt=tilt,tq=tq,f=f))
    return res,mass.sum()
