import numpy as np, time, sys
import legmodel as L
from legmodel import OBox, Cyl, Prism
import csg

def prim_solid(p):
    if isinstance(p, OBox):
        h = p.h; s = csg.box(-h[0], h[0], -h[1], h[1], -h[2], h[2])
        R = p.R; c = p.c
    elif isinstance(p, Prism):
        s = csg.prism('z', [tuple(q) for q in p.pts], p.w0, p.w1)
        R = p.R; c = p.o
    else:
        a = p.a; e1 = np.cross(a, [1, 0, 0] if abs(a[0]) < .9 else [0, 1, 0]); e1 /= np.linalg.norm(e1); e2 = np.cross(a, e1)
        R = np.column_stack([e1, e2, a]); c = p.c
        seg = 40 if p.r > 3 else 20
        s = csg.prism('z', csg.circle(0, 0, p.r, seg), -p.hl, p.hl)
    return s.transform(lambda v: tuple(c + R @ np.array(v)))

def part_solid(prims):
    pos = csg.union_all([prim_solid(p) for s, p in prims if s > 0])
    for s, p in prims:
        if s < 0: pos = pos - prim_solid(p)
    return pos

def place(solid, R=np.eye(3), mirror=False):
    """rotate for printing, mirror for the right leg, then sit it on the bed (z=0) centered"""
    M = np.diag([1, -1, 1]) if mirror else np.eye(3)
    s = solid.transform(lambda v: tuple(R @ (M @ np.array(v))))
    if mirror:   # mirroring flips faces inside-out: reverse vertex order
        s = csg.Solid([csg.Poly(p.v[::-1]) for p in s.polys])
    b = s.bbox(); off = np.array([-(b[0] + b[1]) / 2, -(b[2] + b[3]) / 2, -b[4]])
    return s.transform(lambda v: tuple(np.array(v) + off))

def Rx(d): a = np.radians(d); return np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
def Ry(d): a = np.radians(d); return np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])
PRINT_ROT = {'pelvis_test_mount': Rx(180), 'hip_yoke': np.eye(3), 'thigh': Rx(-90), 'shin': np.eye(3),
             'ankle_yoke': np.eye(3), 'foot': np.eye(3)}
if __name__ == '__main__':
    import os, pickle; os.makedirs('stl', exist_ok=True)
    parts = L.build_parts(); solids = {}
    for n, pr in parts.items():
        t = time.time(); s = part_solid(pr); solids[n] = s
        csg.write_stl_clean(place(s, PRINT_ROT[n]), f'stl/L_{n}.stl', n)
        if n != 'pelvis_test_mount': csg.write_stl_clean(place(s, PRINT_ROT[n], mirror=True), f'stl/R_{n}.stl', 'R_' + n)
        print(n, len(s.polys), 'polys', '%.1fs' % (time.time() - t)); sys.stdout.flush()
    for fl in (2.5, 3.0, 3.5):
        s = part_solid(L.shoe(L.P1, fl))
        Rp = L.P1.R.T     # back into the servo's own frame -> floor flat on the bed
        csg.write_stl_clean(place(s.transform(lambda v: tuple(Rp @ (np.array(v) - L.P1.P)))), f'stl/shoe_{fl}mm.stl', 'shoe')
    pickle.dump(solids, open('solids.pkl', 'wb'))
