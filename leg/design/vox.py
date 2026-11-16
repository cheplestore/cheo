import numpy as np
H = 0.4   # test-cube size (mm)

def part_bounds(prims):
    pts = np.vstack([p.corners() for s, p in prims if s > 0])
    return pts.min(0) - H, pts.max(0) + H

def inside(prims, P):
    pos = np.zeros(len(P), bool); neg = np.zeros(len(P), bool)
    for s, p in prims:
        if s > 0: pos |= p.contains(P)
        else: neg |= p.contains(P)
    return pos & ~neg

class Grid:
    """occupancy grid of a part (in its straight-leg position)"""
    def __init__(s, prims):
        lo, hi = part_bounds(prims)
        s.lo = lo; n = np.ceil((hi - lo) / H).astype(int) + 1; s.n = n
        ax = [lo[i] + H * (np.arange(n[i]) + 0.5) for i in range(3)]
        X, Y, Z = np.meshgrid(*ax, indexing='ij')
        P = np.column_stack([X.ravel(), Y.ravel(), Z.ravel()])
        occ = np.zeros(len(P), bool)
        for k in range(0, len(P), 400000): occ[k:k + 400000] = inside(prims, P[k:k + 400000])
        s.occ = occ.reshape(n); s.pts = P[occ]
    def hit(s, P):
        idx = np.floor((P - s.lo) / H).astype(int)
        ok = np.all((idx >= 0) & (idx < s.n), axis=1)
        out = np.zeros(len(P), bool)
        i = idx[ok]; out[ok] = s.occ[i[:, 0], i[:, 1], i[:, 2]]
        return out

def rot(axis, ang):
    a = np.array(axis, float); a /= np.linalg.norm(a); c, s = np.cos(ang), np.sin(ang)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) * c + s * K + (1 - c) * np.outer(a, a)
