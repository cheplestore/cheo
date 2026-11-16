import numpy as np, pickle, time
import legmodel as L
from vox import Grid, rot

parts = L.build_parts()
# servo bodies + shoes live in the link that holds them
links = {n: [] for n in L.CHAIN}
for n, pr in parts.items(): links[n].append((n, pr))
for sv in L.SERVOS:
    links[L.SERVO_HOME[sv.name]].append((sv.name + ' servo', L.servo_body(sv)))
    links[L.SERVO_HOME[sv.name]].append((sv.name + ' shoe', L.shoe(sv)))
t0 = time.time()
grids = {}
for ln, items in links.items():
    for nm, pr in items: grids[nm] = (ln, Grid(pr))
print("voxelized in %.1fs" % (time.time() - t0), {k: len(g.pts) for k, (l, g) in grids.items()})
# sanity: nothing touches at the straight pose (except intended contacts)
pickle.dump(grids, open('grids.pkl', 'wb'))

def mirror_y(P): Q = P.copy(); Q[:, 1] *= -1; return Q

def sweep(ji, direction, other_leg=False, step=1.0):
    name, sv, k = L.JOINTS[ji]
    above = [n for i, n in enumerate(L.CHAIN) if i < k]
    below = [n for i, n in enumerate(L.CHAIN) if i >= k]
    mov = [(nm, g) for nm, (ln, g) in grids.items() if ln in below]
    sta = [(nm, g) for nm, (ln, g) in grids.items() if ln in above]
    o, a = sv.axis()
    movP = np.vstack([g.pts for nm, g in mov]); movN = np.concatenate([[nm] * len(g.pts) for nm, g in mov])
    if other_leg:   # the right leg (mirror) standing straight, everything below the pelvis
        rl = [(nm, g) for nm, (ln, g) in grids.items() if ln != 'pelvis_test_mount']
    ang = 0.0
    while ang < 180:
        ang += step
        R = rot(a, np.radians(ang * direction))
        P = (movP - o) @ R.T + o
        for nm, g in sta:
            h = g.hit(P)
            if h.any(): return ang, nm, movN[h][0]
        if other_leg:
            Pm = mirror_y(P)   # check my moved points against the mirrored (right) leg = mirror my points into left-leg space
            for nm, g in rl:
                h = g.hit(Pm)
                if h.any(): return ang, 'other leg: ' + nm, movN[h][0]
    return 180.0, None, None

res = {}
for ji, (name, sv, k) in enumerate(L.JOINTS):
    for d in (1, -1):
        r = sweep(ji, d); res[(name, d)] = r
        print(f"{name:12s} dir {d:+d}: stops at {r[0]:5.0f} deg  ({r[2]} hits {r[1]})")
r = sweep(0, -1, other_leg=True); res[('hip roll', 'inward_other_leg')] = r
print("hip roll inward with the other leg there:", r)
pickle.dump(res, open('rom.pkl', 'wb'))
