"""Cheo biped: pelvis, torso, head, arms and covers. All sizes in mm.
World (same as the leg): X forward, Y left, Z up, hip-roll axis at z = 0."""
import sys, math, numpy as np
sys.path.insert(0, '/home/claude/leg')
import csg
from csg import box, prism, circle, union_all
import legmodel as L
from build import part_solid, place, Rx, Ry

# ---------------- measured parts ----------------
BAT_L, BAT_W, BAT_H = 91.5, 44.0, 22.3          # 2x18650 holder (switch + wire on one long face -> faces the door)
SPK_W, SPK_D, SPK_H = 43.7, 15.1, 30.8          # speaker: face 43.7 x 30.8, 15.1 deep
AMP_W, AMP_D, AMP_H = 42.6, 18.8, 24.0          # amp board + plugs
SCR_W, SCR_H, SCR_D, SCR_STACK = 55.5, 34.6, 11.9, 3.8
CAM_BW, CAM_BL, CAM_BH = 50.2, 30.7, 9.8        # camera board
CAM_SQ, CAM_T = 7.9, 5.1                        # camera module on its ribbon
S3_L, S3_W = 63.3, 25.4
# SG90 (standard sizes): case 22.5 x 11.8, shaft 5.9 mm from one end, tabs 32.2 tip to tip
SG_L, SG_W, SG_SHAFT = 22.5, 11.8, 5.9
SG_TAB_SPAN, SG_TAB_Z0, SG_TAB_T = 32.2, 15.9, 2.5
SG_HORN_TOP = 31.5                               # case bottom -> top of the horn

t = 2.0                                          # wall
# ---------------- torso box ----------------
X0, X1 = -32.0, 30.0                             # back, front (outer)
YW = 50.0                                        # half width (outer)
Z_PEL0, Z_PEL1 = 21.8, 25.8                      # pelvis plate (on top of the hip-roll servo tabs)
Z0 = Z_PEL1                                      # torso bottom
Z_DECK0 = Z0 + BAT_H + 2.1                       # battery bay height 24.4
Z_DECK1 = Z_DECK0 + 3.2                        # deck (thicker at the back: the door's top edge hooks up into it)
Z_L3 = Z_DECK1 + 26.0                            # boards + plugs zone
Z_TOP0 = Z_L3 + AMP_H - 5.2 + 0.7 + 18.4         # under the top plate (SG90 hangs above the amp)
Z_TOP1 = Z_TOP0 + 2.5
IX0, IX1, IY = X0 + t, X1 - t, YW - t
SLOT_X0, SLOT_X1 = X0 + 0.8, X0 + 2.7            # back-cover plane (1.6 mm covers)
# ---------------- neck ----------------
SG_CASE_Z0 = Z_TOP0 - (SG_TAB_Z0 + SG_TAB_T)     # SG90 hangs under the top plate, tabs pressed up against it
HEAD_Z0 = SG_CASE_Z0 + SG_HORN_TOP + 0.2         # head floor bottom sits on the horn
COL_R0, COL_R1, COL_H = 13.0, 15.0, 6.0          # collar on the torso top (cut where the SG90 pokes through)
SK_R0, SK_R1 = COL_R1 + 0.5, COL_R1 + 2.5        # head skirt around the collar
# ---------------- head ----------------
HX0, HX1, HW = -21.0, 20.0, 31.0                 # back, front, half width
HFL = 3.0                                        # head floor
HZ0, HZ1 = HEAD_Z0, HEAD_Z0 + 55.0              # camera board stands behind the screen (50.2 tall)
SCR_Z0 = HZ0 + HFL + 1.0                          # screen board bottom
CB_Z0 = HZ0 + HFL - 1.0                          # camera board bottom edge (stands in a 1 mm floor groove)
CAM_ZC = CB_Z0 + 39.9                             # lens centre: 39.9 mm up from the board's USB end (measured)
CB_FRONT = HX1 - 2.0 - SCR_STACK - 1.0 - 6.4      # board's chip side: white plug (6.4) clears the screen's back
CB_BACK = CB_FRONT - 1.6
ht = 2.0
MIC_X, MIC_Z = -8.0, (HEAD_Z0 + 3.0 + 1.0) + 22.0   # behind the screen, right side
# ---------------- arms ----------------
ARM_S = Z_TOP0 - 12.0                             # shoulder axis height
ARM_P_Y = 21.2                                    # MG90S case bottom (its horn ends up 5.8 mm outside the wall)
def arm_servo(side):
    s = 1 if side == 'L' else -1
    return L.Servo('arm' + side, [0, s * ARM_P_Y, ARM_S], [0, s, 0], [0, 0, -1])

def rbox(x0, x1, y0, y1, z0, z1, r):
    return box(x0, x1, y0, y1, z0, z1, r=r)

# ======================================================================
def pelvis():
    """floor of the body + the two hip-roll servo rings (legs hang from these)"""
    s = rbox(X0, X1, -YW, YW, Z_PEL0 + 1.0, Z_PEL1, 4.0) + box(-12, 4, -20, 20, Z_PEL0, Z_PEL0 + 1.01)   # 3 mm plate, 4 mm over the hip servos
    for side in (1, -1):
        rg = part_solid([(sg, p) for sg, p in L.ring(L.R1)])
        if side < 0:
            rg = rg.transform(lambda v: (v[0], -v[1], v[2]))
            rg = csg.Solid([csg.Poly(p.v[::-1]) for p in rg.polys])
        s = s + rg
    cut = []
    # leg servo cables come up through here (front of the battery)
    cut.append(rbox(17, 26, 4, 18, Z_PEL0 - 1, Z_PEL1 + 1, 2))
    cut.append(rbox(17, 26, -18, -4, Z_PEL0 - 1, Z_PEL1 + 1, 2))
    # groove the battery door's bottom edge stands in
    cut.append(box(SLOT_X0 - 0.1, SLOT_X1 + 0.1, -IY + 0.2, IY - 0.2, Z_PEL1 - 1.2, Z_PEL1 + 1))
    # 4 screw holes into the torso's front bosses
    for (x, y) in BOSSES:
        cut.append(prism('z', circle(x, y, 1.2, 16), Z_PEL0 - 1, Z_PEL1 + 1))
        cut.append(prism('z', circle(x, y, 2.3, 16), Z_PEL0 - 1, Z_PEL0 + 1.6))
    # weight-saving windows (the battery sits across them)
    for sd in (1, -1):
        cut.append(rbox(-24, 12, min(sd * 23, sd * 43), max(sd * 23, sd * 43), Z_PEL0 - 1, Z_PEL1 + 1, 4))
    cut.append(rbox(-24, -14, -16, 16, Z_PEL0 - 1, Z_PEL1 + 1, 3))
    return s - union_all(cut)

BOSSES = [(24.5, 44.0), (24.5, -44.0), (24.5, 30.0), (24.5, -30.0)]

def torso():
    s = rbox(X0, X1, -YW, YW, Z0, Z_TOP1, 4.0)
    cut = [box(IX0 - 5, IX1, -IY, IY, Z0 - 1, Z_TOP0)]               # hollow, open back + bottom
    # back-cover grooves in the side walls (upper cover slides down from the top)
    cut.append(box(SLOT_X0, SLOT_X1, -IY - 0.9, IY + 0.9, Z_DECK1 - 0.01, Z_TOP1 + 1))
    cut.append(box(X0 - 1, SLOT_X1, -IY - 0.9, IY + 0.9, Z_TOP0 - 0.01, Z_TOP1 + 1))  # top plate stops before the cover
    # weight-saving windows in the battery bay sides (the battery is held by the floor, deck and door)
    cut.append(box(-22, 10, -YW - 1, YW + 1, Z0 + 5, Z_DECK0 - 4, r=4, axis='y'))
    # speaker grille (front, upper chest)
    zc = Z_L3 + 3 + SPK_H / 2
    for k in range(-3, 4):
        cut.append(box(IX1 - 1, X1 + 1, -SPK_W / 2 + 5, SPK_W / 2 - 5, zc + k * 3.8 - 0.9, zc + k * 3.8 + 0.9, r=0.9, axis='x'))
    # USB-C access to the S3 (left wall, level 2) and a vent row (right wall)
    cut.append(box(-12, 4, IY - 1, YW + 1, Z_DECK1 + 3, Z_DECK1 + 13, r=2, axis='y'))
    for k in range(4):
        cut.append(box(-20 + k * 9, -14 + k * 9, -YW - 1, -IY + 1, Z_DECK1 + 8, Z_DECK1 + 20, r=1.5, axis='y'))
    # shoulder holes for the arm servos' gear boss
    for sd in 'LR':
        cut.append(part_solid([(1, arm_servo(sd).cyl(0, 0, IY - ARM_P_Y - 1.5, YW - ARM_P_Y + 1, 6.6))]))
    # neck: SG90 hole + its 2 tab screw pilots, cable holes inside the collar
    sgx0, sgx1 = -(SG_L - SG_SHAFT), SG_SHAFT                 # long end toward the back
    cut.append(box(sgx0 - 0.3, sgx1 + 0.3, -SG_W / 2 - 0.3, SG_W / 2 + 0.3, Z_TOP0 - 1, Z_TOP1 + COL_H + 1))
    tab_c = ((sgx0 + sgx1) / 2 - (SG_TAB_SPAN / 2 - 2.4), (sgx0 + sgx1) / 2 + (SG_TAB_SPAN / 2 - 2.4))
    for x in tab_c: cut.append(prism('z', circle(x, 0, 0.8, 12), Z_TOP0 - 1, Z_TOP1 + 0.01 - 0.6))
    for side in (1, -1):
        cut.append(rbox(-6, 6, side * 9.5 - 2.5, side * 9.5 + 2.5, Z_TOP0 - 1, Z_TOP1 + COL_H + 1, 2))
    s = s - union_all(cut)
    add = []
    # deck between the battery bay and the electronics
    deck = box(X0, IX1 + 0.01, -IY - 0.01, IY + 0.01, Z_DECK0, Z_DECK1)
    dcut = [rbox(15, 25, -30, 30, Z_DECK0 - 1, Z_DECK1 + 1, 2),                    # leg cables up to the PCA9685
            rbox(SLOT_X1 + 1.5, SLOT_X1 + 7.5, 28, 44, Z_DECK0 - 1, Z_DECK1 + 1, 2),  # battery wire up
            box(SLOT_X0 - 0.1, SLOT_X1 + 0.1, -IY + 0.2, IY - 0.2, Z_DECK0 - 1, Z_DECK0 + 2.6)]  # door's top edge goes up in here
    add.append(deck - union_all(dcut))
    # top plate already part of the shell; collar for the head's skirt
    col = prism('z', circle(0, 0, COL_R1, 64), Z_TOP1 - 0.01, Z_TOP1 + COL_H) - prism('z', circle(0, 0, COL_R0, 64), Z_TOP1 - 1, Z_TOP1 + COL_H + 1)
    col = col - box(sgx0 - 0.3, sgx1 + 0.3, -SG_W / 2 - 0.3, SG_W / 2 + 0.3, Z_TOP1 - 1, Z_TOP1 + COL_H + 1)
    col = col - union_all([rbox(-6, 6, sd * 9.5 - 2.5, sd * 9.5 + 2.5, Z_TOP1 - 1, Z_TOP1 + COL_H + 1, 2) for sd in (1, -1)])
    add.append(col)
    # front screw bosses (pelvis screws up into these)
    for (x, y) in BOSSES:
        add.append(prism('z', circle(x, y, 3.2, 20), Z0, Z0 + 10) + box(x, IX1 + 0.01, y - 3.2, y + 3.2, Z0, Z0 + 10)
                   - prism('z', circle(x, y, 0.85, 12), Z0 - 1, Z0 + 9))
    # speaker cradle: two side ribs + a ledge
    zc0 = Z_L3 + 3
    for side in (1, -1):
        y0 = side * (SPK_W / 2 + 0.3)
        add.append(box(IX1 - SPK_D - 0.5, IX1 + 0.01, min(y0, y0 + side * 1.6), max(y0, y0 + side * 1.6), zc0, zc0 + SPK_H))
    add.append(box(IX1 - SPK_D - 0.5, IX1 + 0.01, -SPK_W / 2 - 1.9, SPK_W / 2 + 1.9, zc0 - 1.6, zc0))
    # arm servo pads: the MG90S slides in from inside, tabs pressed against the pad
    for side in ('L', 'R'):
        sv = arm_servo(side); sg = 1 if side == 'L' else -1
        tab_out = 16.0 + 2.5                       # tab top (z_s) -> pad starts here
        pad = [(1, sv.box(-11.0 - (Z_TOP0 - (ARM_S + 11.0)) - 0.01, L.TAB_L1 + 0.5, -L.RING_HW, L.RING_HW, tab_out, IY - ARM_P_Y + 0.01)),
               (-1, sv.box(L.CASE_L0 - L.CLR, L.CASE_L1 + L.CLR, -L.CASE_HW - L.CLR, L.CASE_HW + L.CLR, tab_out - 1, IY - ARM_P_Y + 1)),
               (-1, sv.cyl(-8.6, 0, tab_out - 1, tab_out + 8, 0.8)), (-1, sv.cyl(19.4, 0, tab_out - 1, tab_out + 8, 0.8))]
        add.append(part_solid(pad))
    return s + union_all(add)

def head():
    s = rbox(HX0, HX1, -HW, HW, HZ0, HZ1, 5.0)
    cut = [box(HX0 - 1, HX1 - ht, -HW + ht, HW - ht, HZ0 + HFL, HZ1 - ht)]          # hollow, open back
    cut.append(box(HX0 + 0.8, HX0 + 2.7, -HW + ht - 1.0, HW - ht + 1.0, HZ0 + HFL - 1.0, HZ1 + 1))   # back cover grooves + top slot
    # screen window 38.5 x 30.5 with a bevel
    zc = SCR_Z0 + SCR_H / 2
    cut.append(box(HX1 - ht - 1, HX1 + 1, -38.5 / 2, 38.5 / 2, zc - 30.5 / 2, zc + 30.5 / 2, r=1.0, axis='x'))
    # camera lens hole in the forehead
    wtop = zc + 30.5 / 2 + 1.5                    # keep a 1.5 mm bar above the screen window
    cut.append(box(HX1 - ht - 1, HX1 - ht + 1.0, -6.0, 6.0, wtop, CAM_ZC + 4.6, r=2.0, axis='x'))
    cut.append(box(HX1 - ht + 0.99, HX1 + 1, -6.8, 6.8, wtop, CAM_ZC + 5.3, r=2.0, axis='x'))
    # camera board: groove in the floor for its bottom edge + room for its USB-C plug
    cut.append(box(CB_BACK - 0.25, CB_FRONT + 0.25, -15.65, 15.65, CB_Z0, HZ0 + HFL + 1))
    cut.append(box(CB_BACK - 0.25, CB_FRONT + 3.8, -5.0, 5.0, CB_Z0, HZ0 + HFL + 1))
    # mic hole in the right side of the head (INMP441 taped inside, its sound hole over this)
    cut.append(box(MIC_X - 1.0, MIC_X + 1.0, -HW - 1, -HW + ht + 1, MIC_Z - 1.0, MIC_Z + 1.0, r=0.9, axis='y'))
    # floor: horn screw access + cables
    cut.append(prism('z', circle(0, 0, 1.6, 16), HZ0 - 1, HZ0 + HFL + 1))
    for side in (1, -1):
        cut.append(rbox(-6, 6, side * 9.5 - 2.5, side * 9.5 + 2.5, HZ0 - 1, HZ0 + HFL + 1, 2))
    s = s - union_all(cut)
    add = []
    # skirt around the torso collar
    sk = prism('z', circle(0, 0, SK_R1, 64), HZ0 - (COL_H + 1.5), HZ0 + 0.01) - prism('z', circle(0, 0, SK_R0, 64), HZ0 - COL_H - 3, HZ0 + 1)
    add.append(sk)
    # screen: stops behind the PCB (it is pushed against the front wall) + side guides
    # little snap ridges just behind the screen board (it clicks past them, then can't slide back)
    bx = HX1 - ht - SCR_STACK - 0.2 - 0.7
    for sd in (1, -1):   # bottom ridges only outside the camera board (its white plug sits in the middle)
        y0, y1 = sorted((sd * 16.5, sd * 26.0))
        add.append(prism('y', [(bx - 0.7, SCR_Z0), (bx, SCR_Z0 + 0.6), (bx + 0.7, SCR_Z0)], y0, y1))
    for side in (1, -1):
        y0, y1 = sorted((side * (CAM_BW / 2 - 3), side * (HW - ht)))
        add.append(prism('y', [(bx - 0.7, SCR_Z0 + SCR_H + 0.41), (bx + 0.7, SCR_Z0 + SCR_H + 0.41), (bx, SCR_Z0 + SCR_H - 0.2)], y0, y1))
    add.append(box(HX1 - ht - SCR_STACK - 0.6, HX1 - ht + 0.01, -SCR_W / 2 - 1.5, SCR_W / 2 + 1.5, HZ0 + HFL - 0.01, SCR_Z0))
    for sd in (1, -1):   # wider ledge only at the sides, under the ridges
        y0, y1 = sorted((sd * 16.5, sd * (SCR_W / 2 + 1.5)))
        add.append(box(HX1 - ht - SCR_STACK - 2, HX1 - ht - SCR_STACK - 0.59, y0, y1, HZ0 + HFL - 0.01, SCR_Z0))
    # shelf the camera board lies on (above the screen)
    add.append(box(HX0 + 3.0, HX1 - ht + 0.01, -HW + ht - 0.01, -CAM_BW / 2 + 3, SCR_Z0 + SCR_H + 0.4, SCR_Z0 + SCR_H + 1.4))
    add.append(box(HX0 + 3.0, HX1 - ht + 0.01, CAM_BW / 2 - 3, HW - ht + 0.01, SCR_Z0 + SCR_H + 0.4, SCR_Z0 + SCR_H + 1.4))
    # (no ribs behind the camera board: they blocked the screen sliding in. Hot glue holds the board.)
    # horn pad: solid floor under the horn for its little screws
    return s + union_all(add)

def arm(side):
    sv = arm_servo(side)
    # in the servo frame: z_s = out from the body, x_s = down
    pz0, pz1 = L.HRN0 - 0.3, L.HRN0 - 0.3 + 6.0
    prims = [(1, sv.cyl(0, 0, pz0, pz1, 10.0)),
             (1, sv.box(0, 62, -8.5, 8.5, pz0, pz1)),
             (1, sv.cyl(62, 0, pz0, pz1, 9.5)),                     # hand
             (-1, sv.cyl(0, 0, pz0 - 1, pz1 + 1, 3.0))]               # 6 mm hole: horn screw goes in from outside, head sits on the horn (like the legs)
    # horn pocket (same triangle as the legs, pointing down the arm)
    hw0 = L.HORN_W0 / 2 + L.POCKET_CLR; hw1 = L.HORN_W1 / 2 + L.POCKET_CLR; Ln = L.HORN_LEN + L.POCKET_CLR + L.ARM_EXTRA
    prims.append((-1, sv.cyl(0, 0, pz0 - 1, L.HRN0 + L.POCKET_DEPTH, L.HORN_HUB_D / 2 + L.POCKET_CLR)))
    prims.append((-1, L.Prism(sv.P, sv.R, [(0, -hw0), (Ln, -hw1), (Ln, hw1), (0, hw0)], pz0 - 1, L.HRN0 + L.POCKET_DEPTH)))
    return part_solid(prims)

def door():
    """battery door: bottom edge stands in the pelvis groove, top clip snaps into the deck groove"""
    # fitted: bottom edge 1.2 mm down in the pelvis groove, top edge 1.2 mm up in the deck slot.
    # to open: lift it 1.3 mm (the deck slot is 2.6 deep), swing the bottom out, pull down.
    w = 2 * IY - 0.8; h = (Z_DECK0 + 1.2) - (Z_PEL1 - 1.2) - 0.1
    p = rbox(0, w, 0, h, 0, 1.6, 0.8)
    p = p - rbox(w / 2 - 22, w / 2 + 22, 5, 17, -1, 3, 3)          # window for the on/off switch
    p = p + box(w / 2 - 12, w / 2 + 12, 1.5, 3.5, 1.59, 3.4)      # finger pull (outside face)
    return p, w, h

def back_cover():
    w = 2 * IY + 1.8 - 0.4; h = Z_TOP1 - Z_DECK1
    p = rbox(0, w, 0, h, 0, 1.6, 1.5)
    p = p + box(w / 2 - 10, w / 2 + 10, h - 1.2, h + 0.8, 0, 3.2)   # pull tab at the top
    v = union_all([rbox(w / 2 - 30, w / 2 + 30, 10 + k * 6, 12.5 + k * 6, -1, 3, 1.2) for k in range(4)])
    return p - v, w, h

def head_cover():
    w = 2 * (HW - ht) + 2.0 - 0.4; h = HZ1 - (HZ0 + HFL - 1.0)
    p = rbox(0, w, 0, h, 0, 1.6, 1.5)
    p = p + box(w / 2 - 8, w / 2 + 8, h - 1.2, h + 0.6, 0, 3.2)
    v = union_all([rbox(w / 2 - 16, w / 2 + 16, 8 + k * 5, 10 + k * 5, -1, 3, 0.9) for k in range(4)])
    return p - v, w, h

if __name__ == '__main__':
    import os, pickle, time, threading
    threading.stack_size(512 * 1024 * 1024); sys.setrecursionlimit(1000000)
    def run():
        os.makedirs('stl', exist_ok=True)
        out = {}
        for name, fn in (('pelvis', pelvis), ('torso', torso), ('head', head)):
            t0 = time.time(); out[name] = fn(); print(name, len(out[name].polys), '%.1fs' % (time.time() - t0)); sys.stdout.flush()
        out['arm_L'] = arm('L'); out['arm_R'] = arm('R')
        d, dw, dh = door(); b, bw, bh = back_cover(); hc, hw, hh = head_cover()
        out['door'] = d; out['back_cover'] = b; out['head_cover'] = hc
        pickle.dump({k: v.polys for k, v in out.items()}, open('body.pkl', 'wb'))
        csg.write_stl_clean(place(out['pelvis'], Rx(180)), 'stl/pelvis.stl', 'pelvis')
        csg.write_stl_clean(place(out['torso'], Ry(90)), 'stl/torso.stl', 'torso')        # lies on its chest
        csg.write_stl_clean(place(out['head'], Ry(90)), 'stl/head.stl', 'head')            # lies on its face
        for sd in 'LR':
            sv = arm_servo(sd); Rp = sv.R.T
            a = out['arm_' + sd].transform(lambda v: tuple(Rp @ (np.array(v) - sv.P)))
            csg.write_stl_clean(place(a, Rx(180)), f'stl/arm_{sd}.stl', 'arm')
        for n, (p, w, h) in (('battery_door', (d, dw, dh)), ('back_cover', (b, bw, bh)), ('head_cover', (hc, hw, hh))):
            csg.write_stl_clean(place(p), f'stl/{n}.stl', n)
        dims = dict(torso=(X1 - X0, 2 * YW, Z_TOP1 - Z0), head=(HX1 - HX0, 2 * HW, HZ1 - HZ0), top_of_head=HZ1,
                    Z_DECK0=Z_DECK0, Z_L3=Z_L3, Z_TOP0=Z_TOP0, ARM_S=ARM_S, HEAD_Z0=HEAD_Z0, SG_CASE_Z0=SG_CASE_Z0)
        print(dims)
    th = threading.Thread(target=run); th.start(); th.join()
