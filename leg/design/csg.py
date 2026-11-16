# Minimal BSP CSG (port of csg.js) + convex prism primitives + STL export
import math, struct, sys
sys.setrecursionlimit(1000000)
EPS = 1e-5

def sub(a, b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
def dot(a, b): return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
def cross(a, b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def lerp(a, b, t): return (a[0]+(b[0]-a[0])*t, a[1]+(b[1]-a[1])*t, a[2]+(b[2]-a[2])*t)
def unit(a):
    l = math.sqrt(dot(a, a)); return (a[0]/l, a[1]/l, a[2]/l)

class Plane:
    __slots__ = ("n", "w")
    def __init__(s, n, w): s.n = n; s.w = w
    @staticmethod
    def from_pts(a, b, c):
        n = unit(cross(sub(b, a), sub(c, a))); return Plane(n, dot(n, a))
    def flipped(s): return Plane((-s.n[0], -s.n[1], -s.n[2]), -s.w)
    def split(s, poly, cf, cb, f, b):
        COP, FR, BK, SP = 0, 1, 2, 3
        ptype = 0; types = []
        for v in poly.v:
            t = dot(s.n, v) - s.w
            ty = BK if t < -EPS else (FR if t > EPS else COP)
            ptype |= ty; types.append(ty)
        if ptype == COP:
            (cf if dot(s.n, poly.p.n) > 0 else cb).append(poly)
        elif ptype == FR: f.append(poly)
        elif ptype == BK: b.append(poly)
        else:
            fv, bv = [], []
            n = len(poly.v)
            for i in range(n):
                j = (i+1) % n
                ti, tj = types[i], types[j]; vi, vj = poly.v[i], poly.v[j]
                if ti != BK: fv.append(vi)
                if ti != FR: bv.append(vi)
                if (ti | tj) == SP:
                    t = (s.w - dot(s.n, vi)) / dot(s.n, sub(vj, vi))
                    v = lerp(vi, vj, t); fv.append(v); bv.append(v)
            if len(fv) >= 3: f.append(Poly(fv, poly.p))
            if len(bv) >= 3: b.append(Poly(bv, poly.p))

class Poly:
    __slots__ = ("v", "p")
    def __init__(s, v, p=None):
        s.v = v; s.p = p if p is not None else Plane.from_pts(v[0], v[1], v[2])
    def flipped(s): return Poly(s.v[::-1], s.p.flipped())

class Node:
    __slots__ = ("plane", "front", "back", "polys")
    def __init__(s, polys=None):
        s.plane = None; s.front = None; s.back = None; s.polys = []
        if polys: s.build(polys)
    def invert(s):
        stack = [s]
        while stack:
            n = stack.pop()
            n.polys = [p.flipped() for p in n.polys]
            if n.plane: n.plane = n.plane.flipped()
            n.front, n.back = n.back, n.front
            if n.front: stack.append(n.front)
            if n.back: stack.append(n.back)
    def clip_polys(s, polys):
        if not s.plane: return list(polys)
        f, b = [], []
        for p in polys: s.plane.split(p, f, b, f, b)
        if s.front: f = s.front.clip_polys(f)
        b = s.back.clip_polys(b) if s.back else []
        return f + b
    def clip_to(s, bsp):
        stack = [s]
        while stack:
            n = stack.pop()
            n.polys = bsp.clip_polys(n.polys)
            if n.front: stack.append(n.front)
            if n.back: stack.append(n.back)
    def all_polys(s):
        out = []; stack = [s]
        while stack:
            n = stack.pop(); out.extend(n.polys)
            if n.front: stack.append(n.front)
            if n.back: stack.append(n.back)
        return out
    def build(s, polys):
        work = [(s, polys)]
        while work:
            n, ps = work.pop()
            if not ps: continue
            if not n.plane: n.plane = ps[0].p
            f, b = [], []
            for p in ps: n.plane.split(p, n.polys, n.polys, f, b)
            if f:
                if not n.front: n.front = Node()
                work.append((n.front, f))
            if b:
                if not n.back: n.back = Node()
                work.append((n.back, b))

class Solid:
    def __init__(s, polys): s.polys = polys
    def bbox(s):
        xs = [v for p in s.polys for v in p.v]
        return (min(v[0] for v in xs), max(v[0] for v in xs), min(v[1] for v in xs),
                max(v[1] for v in xs), min(v[2] for v in xs), max(v[2] for v in xs))
    def __add__(a, b):
        if not a.polys: return b
        if not b.polys: return a
        if not overlap(a.bbox(), b.bbox()): return Solid(a.polys + b.polys)
        A, B = Node(a.polys), Node(b.polys)
        A.clip_to(B); B.clip_to(A); B.invert(); B.clip_to(A); B.invert()
        A.build(B.all_polys()); return Solid(A.all_polys())
    def __sub__(a, b):
        if not b.polys or not overlap(a.bbox(), b.bbox()): return a
        A, B = Node(a.polys), Node(b.polys)
        A.invert(); A.clip_to(B); B.clip_to(A); B.invert(); B.clip_to(A); B.invert()
        A.build(B.all_polys()); A.invert(); return Solid(A.all_polys())
    def transform(s, f):
        out = []
        for p in s.polys:
            vs = [f(v) for v in p.v]
            nx = ny = nz = 0.0
            for i in range(len(vs)):
                a, b = vs[i], vs[(i+1) % len(vs)]
                nx += (a[1]-b[1])*(a[2]+b[2]); ny += (a[2]-b[2])*(a[0]+b[0]); nz += (a[0]-b[0])*(a[1]+b[1])
            l = math.sqrt(nx*nx+ny*ny+nz*nz)
            if l < 1e-12: continue
            n = (nx/l, ny/l, nz/l)
            out.append(Poly(vs, Plane(n, dot(n, vs[0]))))
        return Solid(out)

def overlap(a, b, m=0.01):
    return not (a[1] < b[0]-m or b[1] < a[0]-m or a[3] < b[2]-m or b[3] < a[2]-m or a[5] < b[4]-m or b[5] < a[4]-m)

def union_all(lst):
    out = Solid([])
    for s in lst: out = out + s
    return out

# ---------- primitives: convex prism along an axis ----------
def prism(axis, pts2d, a0, a1):
    """pts2d: convex polygon (u,v). axis 'z': (u,v)=(x,y); 'y': (x,z); 'x': (y,z)"""
    def P(u, v, a):
        if axis == 'z': return (u, v, a)
        if axis == 'y': return (u, a, v)
        return (a, u, v)
    bot = [P(u, v, a0) for u, v in pts2d]
    top = [P(u, v, a1) for u, v in pts2d]
    faces = [bot[::-1], top]
    n = len(pts2d)
    for i in range(n):
        j = (i+1) % n
        faces.append([bot[i], bot[j], top[j], top[i]])
    return convex_from_faces(faces)

def convex_from_faces(faces):
    allv = [v for f in faces for v in f]
    c = (sum(v[0] for v in allv)/len(allv), sum(v[1] for v in allv)/len(allv), sum(v[2] for v in allv)/len(allv))
    polys = []
    for f in faces:
        # dedupe consecutive
        g = [f[0]]
        for v in f[1:]:
            if max(abs(v[k]-g[-1][k]) for k in range(3)) > 1e-9: g.append(v)
        if max(abs(g[0][k]-g[-1][k]) for k in range(3)) < 1e-9: g.pop()
        if len(g) < 3: continue
        p = Poly(g)
        fc = (sum(v[0] for v in g)/len(g), sum(v[1] for v in g)/len(g), sum(v[2] for v in g)/len(g))
        if dot(p.p.n, sub(fc, c)) < 0: p = p.flipped()
        polys.append(p)
    return Solid(polys)

def rrect(u0, u1, v0, v1, r=0.0, seg=8):
    if r <= 0: return [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]
    pts = []
    for (cu, cv, a0) in [(u1-r, v0+r, -90), (u1-r, v1-r, 0), (u0+r, v1-r, 90), (u0+r, v0+r, 180)]:
        for k in range(seg+1):
            a = math.radians(a0 + 90*k/seg)
            pts.append((cu + r*math.cos(a), cv + r*math.sin(a)))
    return pts

def circle(cu, cv, r, seg=24):
    return [(cu + r*math.cos(2*math.pi*k/seg), cv + r*math.sin(2*math.pi*k/seg)) for k in range(seg)]

def stadium_up(u0, u1, v0, vc, seg=16):
    """rect u0..u1, v0..vc, with semicircle on top centered (mid, vc)"""
    r = (u1-u0)/2; cu = (u0+u1)/2
    pts = [(u0, v0), (u1, v0)]
    for k in range(seg+1):
        a = math.pi*k/seg
        pts.append((cu + r*math.cos(a), vc + r*math.sin(a)))
    return pts

def stadium_down(u0, u1, v1, vc, seg=16):
    r = (u1-u0)/2; cu = (u0+u1)/2
    pts = [(u1, v1), (u0, v1)]
    for k in range(seg+1):
        a = math.pi + math.pi*k/seg
        pts.append((cu + r*math.cos(a), vc + r*math.sin(a)))
    return pts

def diamond(cu, cv, h):
    return [(cu+h, cv), (cu, cv+h), (cu-h, cv), (cu, cv-h)]

def box(x0, x1, y0, y1, z0, z1, r=0, axis='z'):
    if axis == 'z': return prism('z', rrect(x0, x1, y0, y1, r), z0, z1)
    if axis == 'y': return prism('y', rrect(x0, x1, z0, z1, r), y0, y1)
    return prism('x', rrect(y0, y1, z0, z1, r), x0, x1)

def frustum_y(x0, x1, z0, z1, ya, yb, grow):
    """rect at y=ya grown by 'grow', rect at y=yb exact"""
    A = [(x0-grow, ya, z0-grow), (x1+grow, ya, z0-grow), (x1+grow, ya, z1+grow), (x0-grow, ya, z1+grow)]
    B = [(x0, yb, z0), (x1, yb, z0), (x1, yb, z1), (x0, yb, z1)]
    faces = [A, B[::-1]]
    for i in range(4):
        j = (i+1) % 4
        faces.append([A[i], A[j], B[j], B[i]])
    return convex_from_faces(faces)

def write_stl(solid, path, name="part"):
    tris = []
    for p in solid.polys:
        for k in range(1, len(p.v)-1):
            tris.append((p.p.n, p.v[0], p.v[k], p.v[k+1]))
    with open(path, "wb") as f:
        f.write(name.encode().ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for n, a, b, c in tris:
            f.write(struct.pack("<12fH", *n, *a, *b, *c, 0))
    return len(tris)

def write_stl_clean(solid, path, name="part", tol=1e-4):
    """Fix T-junctions (insert vertices lying on edges), then fan from centroid."""
    import numpy as np
    from collections import defaultdict
    polys = []
    for p in solid.polys:
        vs = [tuple(round(c/tol)*tol for c in v) for v in p.v]
        g = []
        for v in vs:
            if not g or v != g[-1]: g.append(v)
        if len(g) > 1 and g[0] == g[-1]: g.pop()
        if len(g) >= 3: polys.append((g, p.p.n))
    verts = sorted({v for g, _ in polys for v in g})
    V = np.array(verts)
    cell = 2.0
    grid = defaultdict(list)
    for i, v in enumerate(verts):
        grid[(int(v[0]//cell), int(v[1]//cell), int(v[2]//cell))].append(i)
    def on_edge(a, b):
        a_ = np.array(a); b_ = np.array(b); d = b_ - a_; L2 = d.dot(d)
        lo = np.minimum(a_, b_); hi = np.maximum(a_, b_)
        cands = set()
        for gx in range(int(lo[0]//cell), int(hi[0]//cell)+1):
            for gy in range(int(lo[1]//cell), int(hi[1]//cell)+1):
                for gz in range(int(lo[2]//cell), int(hi[2]//cell)+1):
                    cands.update(grid.get((gx, gy, gz), ()))
        res = []
        for i in cands:
            w = V[i] - a_; tt = w.dot(d)/L2
            if tt <= 1e-6 or tt >= 1-1e-6: continue
            if np.linalg.norm(w - tt*d) < 2e-4: res.append((tt, verts[i]))
        res.sort(); return [v for _, v in res]
    tris = []
    for g, n in polys:
        full = []
        for i in range(len(g)):
            a, b = g[i], g[(i+1) % len(g)]
            full.append(a); full.extend(on_edge(a, b))
        c = tuple(sum(v[k] for v in full)/len(full) for k in range(3))
        for i in range(len(full)):
            tris.append((n, c, full[i], full[(i+1) % len(full)]))
    with open(path, "wb") as f:
        f.write(name.encode().ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for n, a, b, c in tris:
            f.write(struct.pack("<12fH", *n, *a, *b, *c, 0))
    return len(tris)
