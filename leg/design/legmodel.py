"""Cheo biped: one 5-servo leg (left). All sizes in mm. World: X forward, Y left (outward), Z up.
Every part is a list of (+1/-1, primitive) placed at the straight-leg pose."""
import numpy as np, math

# ---------------- MG90S (measured + datasheet) ----------------
CASE_L0, CASE_L1 = -6.0, 16.8      # case along x_s; shaft (joint axis) at x_s = 0, 6 mm from the short end
CASE_HW = 5.95                      # half of 11.9 mm thickness
CASE_TOP = 22.5                     # case top (z_s), shaft side up
TAB_L0, TAB_L1 = -11.0, 21.8        # mounting tabs, 32.8 mm tip to tip
TAB_Z = 16.0                        # bottom of mounting tabs (servo rests here)
HORN_TOP = 33.8                     # top of the horn (tab bottom -> horn top = 17.8 mm)
# single-arm horn (measured): hub 7.2 mm, arm 13.9 mm wide at the hub -> 4.5 mm at the tip, 16.4 mm long, 2.0 mm thick
HORN_HUB_D, HORN_W0, HORN_W1, HORN_LEN, HORN_T = 7.2, 13.9, 4.5, 16.4, 2.0
POCKET_CLR = 0.4
POCKET_DEPTH = 2.4
ARM_EXTRA = 2.0    # arm part of the pocket made 2 mm longer (user fit test)
CLR = 0.2                           # fit clearance per side

# ---------------- design values ----------------
RING_W = 2.5        # wall around the servo case
RING_HW = CASE_HW + CLR + RING_W    # 8.65
PL = 3.5            # bracket plate thickness
SHOE = 3.0          # shoe floor under the servo (2.5 / 3.0 / 3.5 mm versions fine-tune the fit)
GAP = 0.8           # gap between shoe and bracket plate
PIV0, PIV1 = -(SHOE + GAP + PL), -(SHOE + GAP)        # pivot plate z_s range
HRN0 = HORN_TOP - 1.6; HRN1 = HRN0 + 4.0               # horn plate z_s range (horn sits in a 1.8 mm pocket)
BR0, BR1 = 15.0, 20.0                                  # bridge distance from the joint axis
PHW = 9.0                                              # plate / bridge half width
HIP_Y = 10.0        # hip-roll axis 10 mm from the robot's center (20 mm apart)
LEG_Y = 26.0        # leg center line 26 mm from the robot's center

def unit(v): v = np.array(v, float); return v / np.linalg.norm(v)

class OBox:
    def __init__(s, c, R, h): s.c = np.array(c, float); s.R = np.array(R, float); s.h = np.array(h, float)
    def contains(s, P):
        L = (P - s.c) @ s.R
        return np.all(np.abs(L) <= s.h + 1e-9, axis=1)
    def corners(s):
        out = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1): out.append(s.c + s.R @ (s.h * [sx, sy, sz]))
        return np.array(out)

class Cyl:
    def __init__(s, c, a, r, hl): s.c = np.array(c, float); s.a = unit(a); s.r = r; s.hl = hl
    def contains(s, P):
        d = P - s.c; t = d @ s.a
        rad = np.linalg.norm(d - np.outer(t, s.a), axis=1)
        return (np.abs(t) <= s.hl + 1e-9) & (rad <= s.r + 1e-9)
    def corners(s):
        e1 = unit(np.cross(s.a, [1, 0, 0] if abs(s.a[0]) < .9 else [0, 1, 0])); e2 = np.cross(s.a, e1)
        out = []
        for t in (-s.hl, s.hl):
            for u in (-1, 1):
                for v in (-1, 1): out.append(s.c + s.a * t + s.r * (u * e1 + v * e2))
        return np.array(out)

class Prism:
    """convex polygon (u,v) extruded along w, in a frame (origin o, columns R = u,v,w)"""
    def __init__(s, o, R, pts, w0, w1):
        s.o = np.array(o, float); s.R = np.array(R, float); s.pts = np.array(pts, float); s.w0 = w0; s.w1 = w1
        n = len(s.pts); s.edges = []
        cx, cy = s.pts.mean(0)
        for i in range(n):
            a = s.pts[i]; b = s.pts[(i + 1) % n]; e = b - a; nrm = np.array([e[1], -e[0]])
            if np.dot(nrm, [cx - a[0], cy - a[1]]) > 0: nrm = -nrm      # outward normal
            s.edges.append((a, nrm))
    def contains(s, P):
        L = (P - s.o) @ s.R
        ok = (L[:, 2] >= s.w0 - 1e-9) & (L[:, 2] <= s.w1 + 1e-9)
        for a, nrm in s.edges: ok &= ((L[:, :2] - a) @ nrm) <= 1e-9
        return ok
    def corners(s):
        return np.array([s.o + s.R @ np.array([p[0], p[1], w]) for p in s.pts for w in (s.w0, s.w1)])

class Servo:
    """servo frame: z_s = shaft (horn side +), x_s = toward the long end, origin = shaft axis on the case bottom"""
    def __init__(s, name, P, zs, xs):
        s.name = name; s.P = np.array(P, float); s.zs = unit(zs); s.xs = unit(xs)
        s.ys = np.cross(s.zs, s.xs); s.R = np.column_stack([s.xs, s.ys, s.zs])
    def w(s, p): return s.P + s.R @ np.array(p, float)
    def box(s, x0, x1, y0, y1, z0, z1, R=None):
        R = s.R if R is None else R
        c = np.array([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
        return OBox(s.P + s.R @ c if R is s.R else None, R, [(x1 - x0) / 2, (y1 - y0) / 2, (z1 - z0) / 2])
    def cyl(s, x, y, z0, z1, r):
        return Cyl(s.w([x, y, (z0 + z1) / 2]), s.zs, r, (z1 - z0) / 2)
    def axis(s): return s.w([0, 0, 0]), s.zs

def obox_local(sv, Rl, u0, u1, v0, v1, w0, w1):
    """box in a bracket frame Rl (columns u,v,w in servo coords)"""
    c = Rl @ np.array([(u0 + u1) / 2, (v0 + v1) / 2, (w0 + w1) / 2])
    return OBox(sv.P + sv.R @ c, sv.R @ Rl, [(u1 - u0) / 2, (v1 - v0) / 2, (w1 - w0) / 2])

# ---------------- building blocks ----------------
def servo_body(sv):
    return [(1, sv.box(CASE_L0, CASE_L1, -CASE_HW, CASE_HW, 0, CASE_TOP)),
            (1, sv.box(TAB_L0, TAB_L1, -CASE_HW, CASE_HW, TAB_Z, TAB_Z + 2.5)),
            (1, sv.cyl(0, 0, CASE_TOP, 27.0, CASE_HW)),
            (1, sv.box(2, 12, -4, 4, CASE_TOP, 26.0))]

PIN_R = 1.5        # pivot pin radius (3 mm pin)
PIN_IN = 1.8       # how far the pin goes into the shoe

def shoe(sv, floor=SHOE):
    """cap that slides onto the servo's bottom from the long end; the pivot screw threads into it"""
    w = 1.2
    return [(1, sv.box(CASE_L0 - CLR - w, CASE_L1 + CLR, -CASE_HW - CLR - w, CASE_HW + CLR + w, -floor, 3.0)),
            (-1, sv.box(CASE_L0 - CLR, CASE_L1 + CLR + 1, -CASE_HW - CLR, CASE_HW + CLR, 0, 3.5)),
            (-1, sv.cyl(0, 0, -floor - 1, 1, PIN_R + 0.3))]  # the bracket's pivot pin turns in this hole

def ring(sv, x1=TAB_L1 + 0.5):
    return [(1, sv.box(TAB_L0, x1, -RING_HW, RING_HW, 4.0, TAB_Z)),
            (-1, sv.box(CASE_L0 - CLR, CASE_L1 + CLR, -CASE_HW - CLR, CASE_HW + CLR, 3.0, TAB_Z + 1)),
            (-1, sv.cyl(-8.6, 0, 3.0, TAB_Z + 1, 0.9)),           # M2 tab screws (or the screws that come with the servo)
            (-1, sv.cyl(19.4, 0, 3.0, TAB_Z + 1, 0.9))]

def bracket(sv, d, vmin=-PHW, vmax=PHW, rd=PHW):
    """U-bracket that rides on this servo: horn plate + pivot plate + bridge at distance BR0..BR1 along d.
    vmin/vmax: plate width each side of the axis (narrower on the side facing the other leg), rd: disk radius"""
    u = np.array([d[0], d[1], 0.0]); u /= np.linalg.norm(u)
    w = np.array([0, 0, 1.0]); v = np.cross(w, u)
    Rl = np.column_stack([u, v, w])
    out = []
    for z0, z1 in ((PIV0, PIV1), (HRN0, HRN1)):
        out.append((1, sv.cyl(0, 0, z0, z1, rd)))
        out.append((1, obox_local(sv, Rl, 0, BR1, vmin, vmax, z0, z1)))
    out.append((1, obox_local(sv, Rl, BR0, BR1, vmin, vmax, PIV0, HRN1)))
    # printed pivot pin: snaps into the hole in the shoe (no screw needed)
    out.append((1, sv.cyl(0, 0, PIV1 - 0.5, -SHOE + PIN_IN - 0.6, PIN_R)))
    out.append((1, sv.cyl(0, 0, -SHOE + PIN_IN - 0.6, -SHOE + PIN_IN, PIN_R - 0.4)))   # rounded-off tip so it clicks in
    out.append((-1, sv.cyl(0, 0, HRN0 - 1, HRN1 + 1, 3.0)))                  # reach the horn screw
    # horn pocket: the hub + the triangle-shaped arm, pointing toward the bridge
    pz0, pz1 = HRN0 - 0.3, HRN0 + POCKET_DEPTH
    out.append((-1, sv.cyl(0, 0, pz0, pz1, HORN_HUB_D / 2 + POCKET_CLR)))
    hw0 = HORN_W0 / 2 + POCKET_CLR; hw1 = HORN_W1 / 2 + POCKET_CLR; L = HORN_LEN + POCKET_CLR + ARM_EXTRA
    out.append((-1, Prism(sv.P, sv.R @ Rl, [(0, -hw0), (L, -hw1), (L, hw1), (0, hw0)], pz0, pz1)))
    for r in (10.0, 13.5):                                                   # horn screws
        p = Rl @ np.array([r, 0, 0])
        out.append((-1, sv.cyl(p[0], p[1], HRN0 - 1, HRN1 + 1, 0.75)))
    return out

def wbox(x0, x1, y0, y1, z0, z1):
    return OBox([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2], np.eye(3), [(x1 - x0) / 2, (y1 - y0) / 2, (z1 - z0) / 2])

# ---------------- the leg ----------------
SPAN_MID = (PIV0 + HRN1) / 2
PY = LEG_Y - SPAN_MID                     # case-bottom plane for the pitch servos (Y)
PX = -SPAN_MID                            # case-bottom plane for the roll servos (X)
Z_R1 = 0.0
Z_P1 = Z_R1 - (BR0 + BR1)                 # hip roll -> hip pitch (bridges shared)
Z_K = Z_P1 - TAB_L1 - TAB_L1              # thigh: hip-pitch ring meets knee ring
Z_A1 = Z_K - BR1 - TAB_L1                 # shin
Z_R2 = Z_A1 - (BR0 + BR1)                 # ankle yoke
FOOT_TOP = Z_R2 - (TAB_L1 + 0.5) + 0.3     # ankle-roll servo stands upright, long end down into the foot
FOOT_BOT = FOOT_TOP - 3.0

R1 = Servo("hip roll",    [PX, HIP_Y, Z_R1], [1, 0, 0], [0, 0, 1])
P1 = Servo("hip pitch",   [0, PY, Z_P1],     [0, 1, 0], [0, 0, -1])
K  = Servo("knee",        [0, PY, Z_K],      [0, 1, 0], [0, 0, 1])
A1 = Servo("ankle pitch", [0, PY, Z_A1],     [0, 1, 0], [0, 0, 1])
R2 = Servo("ankle roll",  [PX, LEG_Y, Z_R2], [1, 0, 0], [0, 0, -1])

def build_parts():
    P = {}
    mount = ring(R1) + [(1, wbox(-30, 25, -5, 30, R1.w([TAB_L1, 0, 0])[2], R1.w([TAB_L1, 0, 0])[2] + 4)),
                        (-1, Cyl([-22, 12, 0], [0, 0, 1], 1.7, 60)), (-1, Cyl([17, 12, 0], [0, 0, 1], 1.7, 60))]
    P['pelvis_test_mount'] = mount
    P['hip_yoke'] = bracket(R1, (-1, 0), vmin=-8.4, rd=8.4) + bracket(P1, (-1, 0))
    P['thigh'] = ring(P1) + ring(K)
    P['shin'] = bracket(K, (-1, 0)) + ring(A1, x1=TAB_L1 + 1.0)
    P['ankle_yoke'] = bracket(A1, (-1, 0)) + bracket(R2, (-1, 0))
    rx = (R2.w([0, 0, 4])[0], R2.w([0, 0, TAB_Z])[0])
    P['foot'] = ring(R2) + [(1, wbox(-30, 40, LEG_Y - 22.5, LEG_Y + 22.5, FOOT_BOT, FOOT_TOP)),
                            # side braces from the ring down to the foot (stiffer ankle)
                            (1, wbox(min(rx) - 4, min(rx), LEG_Y - RING_HW, LEG_Y + RING_HW, FOOT_TOP - 0.2, FOOT_TOP + 10)),
                            (1, wbox(max(rx), max(rx) + 4, LEG_Y - RING_HW, LEG_Y + RING_HW, FOOT_TOP - 0.2, FOOT_TOP + 10))]
    return P

# which link each servo body + shoe belongs to
SERVO_HOME = {'hip roll': 'pelvis_test_mount', 'hip pitch': 'thigh', 'knee': 'thigh', 'ankle pitch': 'shin', 'ankle roll': 'foot'}
SERVOS = [R1, P1, K, A1, R2]
CHAIN = ['pelvis_test_mount', 'hip_yoke', 'thigh', 'shin', 'ankle_yoke', 'foot']
# joint i connects CHAIN[i] (above) and CHAIN[i+1] (below); axis = that servo's shaft
JOINTS = [('hip roll', R1, 1), ('hip pitch', P1, 2), ('knee', K, 3), ('ankle pitch', A1, 4), ('ankle roll', R2, 5)]
