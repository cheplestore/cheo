"""Cheo quadruped, hexagon enclosed body (MG90S).  Two parts: DECK (the floor, legs hang under it) and DOME (walls + roof, no screws).
Sizes in mm. x forward, y left, z up. Run:  python3 quad_hex.py   (options: r=72 x0=56 yq=24.4 clr=0.15)"""
import sys, math, time
import numpy as np, csg
import legmodel as L
from build import part_solid

R, X0, YQ, CLR_FIT = 72.0, 56.0, 24.4, 0.15      # hexagon radius (corner to centre), leg plane x, hip-roll axis y, fit gap
for a in sys.argv[1:]:
    k, v = a.split("="); v = float(v)
    if k == "r": R = v
    if k == "x0": X0 = v
    if k == "yq": YQ = v
    if k == "clr": CLR_FIT = v
WT, DECK_T, GROOVE_D, INNER_H, ROOF = 2.4, 4.0, 2.0, 28.0, 3.0
ZD0 = L.R1.w([L.TAB_L1, 0, 0])[2]               # underside of the deck (top of the servo rings)
ZD1 = ZD0 + DECK_T                               # top of the deck

def hexpts(r): return [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a))) for a in (30, 90, 150, 210, 270, 330)]
def hexprism(r, z0, z1): return csg.prism('z', hexpts(r), z0, z1)
def move(s, dx=0, dy=0, dz=0): return s.transform(lambda v: (v[0] + dx, v[1] + dy, v[2] + dz))
def rewind(s): return csg.Solid([csg.Poly(p.v[::-1]) for p in s.polys])
def mirror_y(s): return rewind(s.transform(lambda v: (v[0], -v[1], v[2])))
def rot180(s): return s.transform(lambda v: (-v[0], -v[1], v[2]))
def flip_for_print(s):                           # turn upside down and sit on the bed
    f = rewind(s.transform(lambda v: (v[0], v[1], -v[2]))); b = f.bbox(); return move(f, 0, 0, -b[4])

t0 = time.time()
# ---------- DECK ----------
ring = part_solid(L.ring(L.R1))                  # servo ring, left leg frame (roll axis y = HIP_Y, leg plane x = 0)
hole = csg.box(-21, -13, L.HIP_Y - 6, L.HIP_Y + 6, ZD0 - 2, ZD1 + 1, 1.5)        # cable hole beside each ring
def corners(s):
    fl = move(s, X0, YQ - L.HIP_Y); fr = move(mirror_y(s), X0, -(YQ - L.HIP_Y))
    return [fl, fr, rot180(fr), rot180(fl)]      # front left, front right, rear left, rear right
deck = hexprism(R + 3, ZD0, ZD1)
groove = hexprism(R + CLR_FIT, ZD1 - GROOVE_D, ZD1 + 1) - hexprism(R - WT - CLR_FIT, ZD1 - GROOVE_D - 1, ZD1 + 2)
deck = deck - groove
for h in corners(hole): deck = deck - h
for r_ in corners(ring): deck = deck + r_
# ---------- DOME ----------
z0, z1 = ZD1 - GROOVE_D, ZD1 + INNER_H + ROOF
dome = hexprism(R, z0, z1) - hexprism(R - WT, z0 - 1, z1 - ROOF)
fx = R * math.cos(math.radians(30))              # rear flat is at x = -fx
dome = dome - csg.box(-fx - 2, -fx + WT + 2, -6.5, 6.5, ZD1 + 4, ZD1 + 12, 1.5)        # USB window
dome = dome - csg.box(-fx - 2, -fx + WT + 2, 16, 28, ZD1 + 4, ZD1 + 11, 1.5)           # power switch window
for px in (-24.2, 24.2):                         # four posts from the roof keep a 2-cell holder (about 82 x 45) in place
    for py in (-43.2, 43.2):
        dome = dome + csg.box(px - 1.5, px + 1.5, py - 1.5, py + 1.5, ZD1 + 1.0, z1 - ROOF + 0.5)
for name, s in (("quad_deck", deck), ("quad_dome", dome)):
    p = flip_for_print(s); b = p.bbox()
    csg.write_stl_clean(p, f"stl/{name}.stl", name)
    print(name, "polys", len(p.polys), "size x %.0f y %.0f z %.0f" % (b[1] - b[0], b[3] - b[2], b[5] - b[4]))
print("done %.1fs" % (time.time() - t0))
