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
# ---------------- neck tower (separate part, snaps onto the torso top) ----------------
TW_BASE = 3.0                                    # tower base plate
TW_Z0 = Z_TOP1                                   # sits on the torso top
SG_CASE_Z0 = TW_Z0 + TW_BASE                     # SG90 stands on the base, shaft up at (0, 0)
SGX0, SGX1 = -(SG_L - SG_SHAFT), SG_SHAFT        # case x range (long end toward the back)
TAB_X0, TAB_X1 = (SGX0 + SGX1) / 2 - SG_TAB_SPAN / 2, (SGX0 + SGX1) / 2 + SG_TAB_SPAN / 2
HEAD_Z0 = SG_CASE_Z0 + SG_HORN_TOP + 0.2         # head floor sits on the horn
# ---------------- head ----------------
HX0, HX1, HW = -19.5, 17.5, 31.0                 # back, front, half width
HFL = 4.0                                        # head floor (horn socket underneath)
HZ0, HZ1 = HEAD_Z0, HEAD_Z0 + 46.0              # tall enough for the screen to slide in under the mic rails
SCR_Z0 = HZ0 + HFL + 1.0                          # screen board bottom (on a 1 mm ledge)
DISP_DY, DISP_DZ = 2.0, 0.85                     # lit area sits right/up of the board centre (photo)
DISP_W, DISP_H = 35.0, 28.0                      # lit area of the 1.8" ST7735
ht = 2.0
MIC_X, MIC_Y = -8.5, 0.0                         # INMP441 under the ceiling, back half
# ---------------- camera hat ----------------
HAT_X0, HAT_X1, HAT_HW = 1.5, 15.0, 17.5
CB_X0 = HAT_X0 + 2.0 + 0.5                       # camera board back face
CB_X1 = CB_X0 + 1.6
HAT_Z0 = HZ1                                     # hat stands on the head top
CAM_ZC = HAT_Z0 + 39.9                           # lens centre (39.9 mm up from the board's USB end)
HAT_Z1 = HAT_Z0 + 50.2 + 0.5 + 2.0
# ---------------- arms ----------------
ARM_S = Z_TOP0 - 12.0                             # shoulder axis height
ARM_P_Y = 21.2                                    # MG90S case bottom (its horn ends up 5.8 mm outside the wall)
def arm_servo(side):
    s = 1 if side == 'L' else -1
    return L.Servo('arm' + side, [0, s * ARM_P_Y, ARM_S], [0, s, 0], [0, 0, -1])

def rbox(x0, x1, y0, y1, z0, z1, r):
    return box(x0, x1, y0, y1, z0, z1, r=r)

# ======================================================================
PIN_HOLE = 1.65                                  # printed snap pins (3 mm) hold the pelvis on: no screws, no glue
BOSSES = [(24.5, 44.0), (24.5, -44.0), (24.5, 30.0), (24.5, -30.0)]

def bump(x0, x1, y0, y1, z0, z1):
    """little crush bump (press-fit holders)"""
    return box(x0, x1, y0, y1, z0, z1)

def pelvis():
    """floor of the body + the two hip-roll servo rings (legs hang from these)"""
    s = rbox(X0, X1, -YW, YW, Z_PEL0 + 1.0, Z_PEL1, 4.0) + box(-12, 4, -20, 20, Z_PEL0, Z_PEL0 + 1.01)
    for side in (1, -1):
        rg = part_solid([(sg, p) for sg, p in L.ring(L.R1)])
        if side < 0:
            rg = rg.transform(lambda v: (v[0], -v[1], v[2]))
            rg = csg.Solid([csg.Poly(p.v[::-1]) for p in rg.polys])
        s = s + rg
    cut = [rbox(17, 26, 4, 18, Z_PEL0 - 1, Z_PEL1 + 1, 2), rbox(17, 26, -18, -4, Z_PEL0 - 1, Z_PEL1 + 1, 2),   # leg cables up
           box(SLOT_X0 - 0.1, SLOT_X1 + 0.1, -IY + 0.2, IY - 0.2, Z_PEL1 - 1.2, Z_PEL1 + 1)]                    # door groove
    for (x, y) in BOSSES:
        cut.append(prism('z', circle(x, y, PIN_HOLE, 20), Z_PEL0 - 1, Z_PEL1 + 1))
        cut.append(prism('z', circle(x, y, 3.3, 24), Z_PEL0 - 1, Z_PEL0 + 1.0 + 1.4))     # pin head sits flush
    for sd in (1, -1):
        cut.append(rbox(-24, 12, min(sd * 23, sd * 43), max(sd * 23, sd * 43), Z_PEL0 - 1, Z_PEL1 + 1, 4))
    cut.append(rbox(-24, -14, -16, 16, Z_PEL0 - 1, Z_PEL1 + 1, 3))
    return s - union_all(cut)

# deck layout (level 2): ESP32-S3 at the back (USB-C to the left wall), PCA9685 in front, 2 Mini560 on the right
S3_X0, S3_Y1 = SLOT_X1 + 0.5 + 1.4 + 0.3, 44.0                        # S3 lies upside down (pins up), USB end at +y
S3_X1, S3_Y0 = S3_X0 + S3_W, S3_Y1 - S3_L
PCA_L, PCA_W = 62.5, 25.4
PCA_X0 = S3_X1 + 2.0; PCA_X1 = PCA_X0 + PCA_W
MINI_L, MINI_H, MINI_T = 32.6, 17.7, 5.1
MINI_YS = (-46.6, -39.5)                          # two Mini560 standing on their long edge
MINI_X0 = -7.2
TRAY_T = 1.6
LEGSLOT = (4.0, 22.0, 34.0, 45.0)                # leg cables come up here (deck + tray)
BATSLOT = (-27.8, -12.0, -46.0, -36.0)           # battery wire comes up here
AMP_X0, AMP_X1 = SLOT_X1 + 0.4, SLOT_X1 + 0.4 + AMP_D + 0.6
NECK_LEG_Y = 9.2                                 # neck tower snap legs
WIRE_Y0, WIRE_Y1 = 13.0, 19.2                    # head wire slots in the torso top (each side)

def torso():
    s = rbox(X0, X1, -YW, YW, Z0, Z_TOP1, 4.0)
    cut = [box(IX0 - 5, IX1, -IY, IY, Z0 - 1, Z_TOP0)]               # hollow, open back + bottom
    cut.append(box(SLOT_X0, SLOT_X1, -IY - 0.9, IY + 0.9, Z_DECK1 - 0.01, Z_TOP1 + 1))
    cut.append(box(X0 - 1, SLOT_X1, -IY - 0.9, IY + 0.9, Z_TOP0 - 0.01, Z_TOP1 + 1))
    cut.append(box(-22, 10, -YW - 1, YW + 1, Z0 + 5, Z_DECK0 - 4, r=4, axis='y'))
    # weight-saving windows: chest sides (front + back of the arm pads), deck level, top plate corners
    for xa, xb in ((-25.0, -13.0), (13.0, 25.0)):
        cut.append(box(xa, xb, -YW - 1, YW + 1, Z_L3 + 4, Z_TOP0 - 6, r=4, axis='y'))
    cut.append(box(2.0, 22.0, IY - 1, YW + 1, Z_DECK1 + 6, Z_DECK1 + 19, r=4, axis='y'))      # left, beside the PCA9685
    cut.append(box(-22.0, 0.0, -YW - 1, -IY + 1, Z_DECK1 + 6, Z_DECK1 + 19, r=4, axis='y'))   # right, beside the Mini560s
    for sd in (1, -1):
        y0, y1 = sorted((sd * 22.0, sd * 37.0))
        cut.append(rbox(12.0, 25.0, y0, y1, Z_TOP0 - 1, Z_TOP1 + 1, 3))
    # shallow panel on the front of the battery bay (wall 1.2 mm there)
    cut.append(box(X1 - 0.8, X1 + 1, -36, 36, Z0 + 4, Z_DECK0 - 3, r=3, axis='x'))
    zc = Z_L3 + 3 + SPK_H / 2
    for k in range(-3, 4):
        cut.append(box(IX1 - 1, X1 + 1, -SPK_W / 2 + 5, SPK_W / 2 - 5, zc + k * 3.8 - 0.9, zc + k * 3.8 + 0.9, r=0.9, axis='x'))
    # USB-C of the S3 (it lies upside down on the tray, so the plug is low)
    ux = (S3_X0 + S3_X1) / 2
    cut.append(box(ux - 7.5, ux + 7.5, IY - 1, YW + 1, Z_DECK1 + 0.5, Z_DECK1 + 11.5, r=2.5, axis='y'))
    for k in range(4):
        cut.append(box(-20 + k * 9, -14 + k * 9, -YW - 1, -IY + 1, Z_DECK1 + 22, Z_DECK1 + 25, r=1.2, axis='y'))
    for sd in 'LR':
        cut.append(part_solid([(1, arm_servo(sd).cyl(0, 0, IY - ARM_P_Y - 1.5, YW - ARM_P_Y + 1, 6.6))]))
    # top plate: neck tower snap slots, SG90 cable hole, 2 big head-wire slots
    for sd in (1, -1):
        y0, y1 = sorted((sd * WIRE_Y0, sd * WIRE_Y1))
        cut.append(rbox(-8, 8, y0, y1, Z_TOP0 - 1, Z_TOP1 + 1, 2.5))
    cut.append(rbox(TAB_X0 + 0.4, SGX0 + 0.6, -4.0, 4.0, Z_TOP0 - 1, Z_TOP1 + 1, 1.5))
    s = s - union_all(cut)
    add = []
    deck = box(X0, IX1 + 0.01, -IY - 0.01, IY + 0.01, Z_DECK0, Z_DECK1)
    dcut = [rbox(LEGSLOT[0], LEGSLOT[1], LEGSLOT[2], LEGSLOT[3], Z_DECK0 - 1, Z_DECK1 + 1, 2),
            rbox(BATSLOT[0], BATSLOT[1], BATSLOT[2], BATSLOT[3], Z_DECK0 - 1, Z_DECK1 + 1, 2),
            box(SLOT_X0 - 0.1, SLOT_X1 + 0.1, -IY + 0.2, IY - 0.2, Z_DECK0 - 1, Z_DECK0 + 2.6),
            rbox(-23.0, -4.0, -28.0, 28.0, Z_DECK0 - 1, Z_DECK1 + 1, 4),     # windows: the tray carries the boards
            rbox(2.0, 24.0, -28.0, 28.0, Z_DECK0 - 1, Z_DECK1 + 1, 4)]
    add.append(deck - union_all(dcut))
    for (x, y) in BOSSES:     # pins go up through these
        add.append(prism('z', circle(x, y, 3.4, 20), Z0, Z0 + 10) + box(x, IX1 + 0.01, y - 3.4, y + 3.4, Z0, Z0 + 10)
                   - prism('z', circle(x, y, PIN_HOLE, 20), Z0 - 1, Z0 + 11))
    # speaker cradle: side ribs with crush bumps + a ledge (speaker presses in, no glue)
    zc0 = Z_L3 + 3
    for side in (1, -1):
        y0 = side * (SPK_W / 2 + 0.3)
        add.append(box(IX1 - SPK_D - 0.5, IX1 + 0.01, min(y0, y0 + side * 1.6), max(y0, y0 + side * 1.6), zc0, zc0 + SPK_H))
        yb0, yb1 = sorted((side * (SPK_W / 2 - 0.05), side * (SPK_W / 2 + 0.31)))
        for zb in (zc0 + 6, zc0 + SPK_H - 6):
            add.append(bump(IX1 - SPK_D + 1, IX1 - 3, yb0, yb1, zb - 1.5, zb + 1.5))
    add.append(box(IX1 - SPK_D - 0.5, IX1 + 0.01, -SPK_W / 2 - 1.9, SPK_W / 2 + 1.9, zc0 - 1.6, zc0))
    # amp bay: hangs under the top plate at the back, open front + back, crush bumps on the sides
    az0 = Z_TOP0 - AMP_H - 3.0                      # 3 mm over the amp for the neck servo cable + amp wires
    for side in (1, -1):
        y0, y1 = sorted((side * (AMP_W / 2 + 0.3), side * (AMP_W / 2 + 1.9)))
        add.append(box(AMP_X0, AMP_X1, y0, y1, az0 - 1.6, Z_TOP0 + 0.01))
        yb0, yb1 = sorted((side * (AMP_W / 2 - 0.05), side * (AMP_W / 2 + 0.31)))
        add.append(bump(AMP_X0 + 3, AMP_X1 - 3, yb0, yb1, az0 + 4, az0 + 8))
    add.append(box(AMP_X0, AMP_X1, -AMP_W / 2 - 1.9, AMP_W / 2 + 1.9, az0 - 1.6, az0))
    # 2 snap legs up through the neck tower's base (tower clicks down onto them)
    for sd in (1, -1):
        y0, y1 = sorted((sd * NECK_LEG_Y, sd * (NECK_LEG_Y + 1.6)))
        bt = Z_TOP1 + TW_BASE
        add.append(box(-4.0, 4.0, y0, y1, Z_TOP1 - 0.01, bt + 1.6))
        yb = sd * (NECK_LEG_Y + 1.6)
        add.append(prism('x', [(yb, bt + 0.15), (yb + sd * 0.7, bt + 0.15), (yb, bt + 1.6)], -4.0, 4.0))
    for side in ('L', 'R'):
        sv = arm_servo(side)
        tab_out = 16.0 + 2.5
        pad = [(1, sv.box(-11.0 - (Z_TOP0 - (ARM_S + 11.0)) - 0.01, L.TAB_L1 + 0.5, -L.RING_HW, L.RING_HW, tab_out, IY - ARM_P_Y + 0.01)),
               (-1, sv.box(L.CASE_L0 - L.CLR, L.CASE_L1 + L.CLR, -L.CASE_HW - L.CLR, L.CASE_HW + L.CLR, tab_out - 1, IY - ARM_P_Y + 1)),
               (-1, sv.cyl(-8.6, 0, tab_out - 1, tab_out + 8, 0.8)), (-1, sv.cyl(19.4, 0, tab_out - 1, tab_out + 8, 0.8))]
        add.append(part_solid(pad))
    return s + union_all(add)

def deck_tray():
    """press-fit tray on the deck: holds the S3, PCA9685 and both Mini560s without glue or screws"""
    tx0, tx1, ty = SLOT_X1 + 0.5, IX1 - 0.3, IY - 0.3
    z0 = Z_DECK1; z1 = z0 + TRAY_T
    s = box(tx0, tx1, -ty, ty, z0, z1)
    cut = [rbox(LEGSLOT[0], LEGSLOT[1], LEGSLOT[2], LEGSLOT[3], z0 - 1, z1 + 1, 2),
           rbox(BATSLOT[0], BATSLOT[1], BATSLOT[2], BATSLOT[3], z0 - 1, z1 + 1, 2),
           rbox(S3_X0 + 5, S3_X1 - 5, S3_Y0 + 8, S3_Y1 - 8, z0 - 1, z1 + 1, 3),        # lighten under the boards
           rbox(PCA_X0 + 5, PCA_X1 - 5, -PCA_L / 2 + 8, PCA_L / 2 - 8, z0 - 1, z1 + 1, 3)]
    s = s - union_all(cut)
    add = []
    for sd in (1, -1):        # side crush bumps against the torso walls
        y0, y1 = sorted((sd * (ty - 0.01), sd * (ty + 0.45)))
        for x in (tx0 + 4, 4.0, tx1 - 6):
            add.append(bump(x - 3, x + 3, y0, y1, z0, z1))
    def corners(x0, x1, y0, y1, pad_h, wall_h):
        """4 corner pads (board sits on them) + 4 corner L-walls with crush bumps"""
        out = []
        for cx, sx in ((x0, -1), (x1, 1)):
            for cy, sy in ((y0, -1), (y1, 1)):
                px0, px1 = sorted((cx, cx - sx * 4.5)); py0, py1 = sorted((cy, cy - sy * 4.5))
                out.append(box(px0, px1, py0, py1, z1 - 0.01, z1 + pad_h))
                wx0, wx1 = sorted((cx + sx * 0.25, cx + sx * 1.65)); wy0, wy1 = sorted((cy - sy * 6, cy + sy * 1.65))
                out.append(box(wx0, wx1, wy0, wy1, z1 - 0.01, z1 + pad_h + wall_h))
                wx0, wx1 = sorted((cx - sx * 6, cx + sx * 1.65)); wy0, wy1 = sorted((cy + sy * 0.25, cy + sy * 1.65))
                out.append(box(wx0, wx1, wy0, wy1, z1 - 0.01, z1 + pad_h + wall_h))
                bx0, bx1 = sorted((cx + sx * 0.25, cx - sx * 0.1)); 
                out.append(bump(bx0, bx1, min(cy - sy * 4.5, cy - sy * 2.5), max(cy - sy * 4.5, cy - sy * 2.5), z1 + pad_h + 0.4, z1 + pad_h + 1.4))
        return out
    # S3 upside down (its chip is underneath): 3.6 mm pads so the chip clears the tray
    add += corners(S3_X0, S3_X1, S3_Y0, S3_Y1, 3.6, 2.4)
    add += corners(PCA_X0, PCA_X1, -PCA_L / 2, PCA_L / 2, 3.0, 2.4)
    # Mini560 pockets (each board stands on its long edge, press fit)
    for y in MINI_YS:
        y0, y1 = y - 0.25, y + MINI_T + 0.25
        add.append(box(MINI_X0 - 1.6, MINI_X0 + MINI_L + 1.6, y0 - 1.6, y1 + 1.6, z1 - 0.01, z1 + 5.0)
                   - box(MINI_X0 - 0.25, MINI_X0 + MINI_L + 0.25, y0, y1, z1, z1 + 6))
        add.append(bump(MINI_X0 + 10, MINI_X0 + 20, y1 - 0.35, y1 + 0.01, z1 + 1, z1 + 4))
    return s + union_all(add)

def neck_tower():
    """SG90 stands in this, snap hooks hold its tabs (no screws). Snaps onto the torso top."""
    b0, b1 = TW_Z0, TW_Z0 + TW_BASE
    s = rbox(TAB_X0 - 4.0, TAB_X1 + 4.0, -12.0, 12.0, b0, b1, 4.0)
    blk = box(TAB_X0 - 0.2, TAB_X1 + 0.2, -SG_W / 2 - 1.8, SG_W / 2 + 1.8, b1 - 0.01, b1 + SG_TAB_Z0)
    s = s + blk
    cut = [box(SGX0 - 0.25, SGX1 + 0.25, -SG_W / 2 - 0.25, SG_W / 2 + 0.25, b1, b1 + SG_TAB_Z0 + 1),           # pocket
           box(TAB_X0 + 0.4, SGX0 + 0.1, -3.5, 3.5, b0 - 1, b1 + 7),                                       # cable channel
           box(SGX1 - 0.1, TAB_X1 - 1.5, -3.0, 3.0, b1 + SG_TAB_Z0 - 3, b1 + SG_TAB_Z0 + 1)]              # (lighter)
    for sd in (1, -1):     # slots for the torso's 2 snap legs
        y0, y1 = sorted((sd * (NECK_LEG_Y - 0.3), sd * (NECK_LEG_Y + 1.9)))
        cut.append(box(-4.4, 4.4, y0, y1, b0 - 1, b1 + 1))
    s = s - union_all(cut)
    add = []
    lip_z = b1 + SG_TAB_Z0 + SG_TAB_T + 0.15
    for sx, xe in ((-1, TAB_X0 - 0.2), (1, TAB_X1 + 0.2)):
        a0, a1 = sorted((xe + sx * 0.6, xe + sx * 2.2))
        add.append(box(a0, a1, -4.0, 4.0, b1 - 0.01, lip_z + 1.0))                                   # flexible arm
        add.append(prism('y', [(xe + sx * 0.61, lip_z), (xe - sx * 1.0, lip_z), (xe + sx * 0.61, lip_z + 2.0)], -4.0, 4.0))
    return s + union_all(add)

def head():
    s = rbox(HX0, HX1, -HW, HW, HZ0, HZ1, 4.0)
    cut = [box(HX0 - 1, HX1 - ht, -HW + ht, HW - ht, HZ0 + HFL, HZ1 - ht)]
    cut.append(box(HX0 + 0.8, HX0 + 2.7, -HW + ht - 1.0, HW - ht + 1.0, HZ0 + HFL - 1.0, HZ1 + 1))
    # window over the lit area (+1 mm each side), moved to where the display really is
    wy, wz = DISP_DY, SCR_Z0 + SCR_H / 2 + DISP_DZ
    cut.append(box(HX1 - ht - 1, HX1 + 1, wy - DISP_W / 2 - 1, wy + DISP_W / 2 + 1, wz - DISP_H / 2 - 1, wz + DISP_H / 2 + 1, r=1.0, axis='x'))
    cut.append(box(HX1 - 0.6, HX1 + 1, wy - DISP_W / 2 - 1.6, wy + DISP_W / 2 + 1.6, wz - DISP_H / 2 - 1.6, wz + DISP_H / 2 + 1.6, r=1.4, axis='x'))
    # horn socket under the floor (same triangle as the legs), optional centre screw hole
    hw0 = L.HORN_W0 / 2 + L.POCKET_CLR; hw1 = L.HORN_W1 / 2 + L.POCKET_CLR; Ln = L.HORN_LEN + L.POCKET_CLR + L.ARM_EXTRA
    cut.append(prism('z', circle(0, 0, L.HORN_HUB_D / 2 + L.POCKET_CLR, 28), HZ0 - 1, HZ0 + L.POCKET_DEPTH))
    cut.append(prism('z', [(-hw0, 0), (hw0, 0), (hw1, Ln), (-hw1, Ln)][::-1], HZ0 - 1, HZ0 + L.POCKET_DEPTH).transform(lambda v: (v[0], v[1], v[2])))
    cut.append(prism('z', circle(0, 0, 1.6, 16), HZ0 - 1, HZ0 + HFL + 1))
    # head wires down through the floor (2 big holes)
    for sd in (1, -1):
        x0, x1 = sorted((sd * 9.0, sd * (HX1 - ht + 0.5)))
        cut.append(rbox(x0, x1, -7.0, 7.0, HZ0 - 1, HZ0 + HFL + 1, 2))
    for k in range(3):     # side vents (lighter, lets the speaker-side air out)
        zc = HZ0 + 14 + k * 7
        cut.append(box(HX0 + 5, HX0 + 17, -HW - 1, HW + 1, zc - 1.5, zc + 1.5, r=1.4, axis='y'))
    # top: mic sound hole, hat snap slots, hat wire hole
    cut.append(prism('z', circle(MIC_X, MIC_Y, 1.0, 12), HZ1 - ht - 1, HZ1 + 1))
    for xs in (HAT_X0 - 0.2, HAT_X1 - 2.0 - 0.2):
        cut.append(box(xs, xs + 2.4, -4.4, 4.4, HZ1 - ht - 1, HZ1 + 1))
    cut.append(rbox(CB_X0 + 1.0, HAT_X1 - 4.5, -10, 10, HZ1 - ht - 1, HZ1 + 1, 2))
    s = s - union_all(cut)
    add = []
    # screen: 1 mm ledge underneath, lip on top, snap ridges just behind the board
    add.append(box(HX1 - ht - SCR_STACK - 2, HX1 - ht + 0.01, -SCR_W / 2 - 1.5, SCR_W / 2 + 1.5, HZ0 + HFL - 0.01, SCR_Z0))
    for sd in (1, -1):
        y0, y1 = sorted((sd * 6.5, sd * (SCR_W / 2 + 1.5)))
        add.append(box(HX1 - ht - SCR_STACK - 2, HX1 - ht + 0.01, y0, y1, SCR_Z0 + SCR_H + 0.4, HZ1 - ht + 0.01))
    bx = HX1 - ht - SCR_STACK - 0.2 - 0.7
    add.append(prism('y', [(bx - 0.7, SCR_Z0), (bx, SCR_Z0 + 0.6), (bx + 0.7, SCR_Z0)], -24, 24))
    for sd in (1, -1):
        y0, y1 = sorted((sd * 6.5, sd * 24))
        add.append(prism('y', [(bx - 0.7, SCR_Z0 + SCR_H + 0.41), (bx + 0.7, SCR_Z0 + SCR_H + 0.41), (bx, SCR_Z0 + SCR_H - 0.2)], y0, y1))
    # INMP441 rails under the ceiling (board slides in from the back, sound hole up)
    rz1 = HZ1 - ht; pcb_top = rz1 - 1.5; pcb_bot = pcb_top - 1.6
    for sd in (1, -1):
        y0, y1 = sorted((MIC_Y + sd * 7.4, MIC_Y + sd * 9.4))
        rail = box(HX0 + 3.2, MIC_X + 8.0, y0, y1, pcb_bot - 1.6, rz1 + 0.01)
        g0, g1 = sorted((MIC_Y + sd * 6.6, MIC_Y + sd * 8.4))
        rail = rail - box(HX0 + 3.0, MIC_X + 3.1, g0, g1, pcb_bot - 0.15, pcb_top + 0.15)
        add.append(rail)
    return s + union_all(add)

def camera_hat():
    """camera board stands up in here, camera centred; snaps onto the head top"""
    s = rbox(HAT_X0, HAT_X1, -HAT_HW, HAT_HW, HAT_Z0, HAT_Z1, 3.0)
    ix0, ix1, iy = HAT_X0 + 2.0, HAT_X1 - 2.0, HAT_HW - 2.0
    cut = [box(ix0, ix1, -iy, iy, HAT_Z0 - 1, HAT_Z1 - 2.0)]
    cut.append(box(CB_X0 - 0.15, CB_X1 + 0.15, -15.35 - 0.35, 15.35 + 0.35, HAT_Z0 - 1, HAT_Z0 + 50.2 + 0.5))   # PCB edge grooves
    # camera window (camera sits ~3 mm behind the front wall), flared
    cut.append(box(ix1 - 0.5, ix1 + 1.0, -5.0, 5.0, CAM_ZC - 4.2, CAM_ZC + 4.2, r=2.0, axis='x'))
    cut.append(box(ix1 + 0.99, HAT_X1 + 1, -6.2, 6.2, CAM_ZC - 5.2, CAM_ZC + 5.2, r=2.0, axis='x'))
    # slits that turn the bottom of the front + back walls into snap tongues
    for xw0, xw1 in ((HAT_X0 - 1, ix0 + 0.5), (ix1 - 0.5, HAT_X1 + 1)):
        for sd in (1, -1):
            y0, y1 = sorted((sd * 4.4, sd * 5.2))
            cut.append(box(xw0, xw1, y0, y1, HAT_Z0 - 1, HAT_Z0 + 14))
    for sd in (1, -1):
        y0, y1 = sorted((sd * (iy - 0.5), sd * (HAT_HW + 1)))
        cut.append(box(CB_X1 + 1.5, ix1 - 1.0, y0, y1, HAT_Z0 + 10, HAT_Z1 - 8, r=2.0, axis='y'))
    cut.append(box(HAT_X0 - 1, ix0 + 0.5, -10, 10, HAT_Z0 + 17, HAT_Z1 - 8, r=3.0, axis='x'))
    s = s - union_all(cut)
    add = []
    for sx, (x0, x1) in ((1, (HAT_X0 + 0.2, ix0)), (-1, (ix1, HAT_X1 - 0.2))):   # tongues go down through the head top
        add.append(box(x0, x1, -4.2, 4.2, HAT_Z0 - ht - 1.6, HAT_Z0 + 0.01))
        xi = x1 if sx > 0 else x0                                                 # barb points inward (tongue flexes outward)
        add.append(prism('y', [(xi, HAT_Z0 - ht - 0.15), (xi + sx * 0.7, HAT_Z0 - ht - 0.15), (xi, HAT_Z0 - ht - 1.6)] if sx > 0 else
                         [(xi + sx * 0.7, HAT_Z0 - ht - 0.15), (xi, HAT_Z0 - ht - 0.15), (xi, HAT_Z0 - ht - 1.6)], -4.2, 4.2))
    return s + union_all(add)

def snap_pin():
    """3 mm push pin: up through the pelvis into the torso boss, split end clicks over the boss top.
    Round 3.0 mm with flats top + bottom (prints lying down), fits the 3.3 mm holes."""
    Ls = (Z0 + 10) - (Z_PEL0 + 1.0 + 1.4) + 0.15        # shaft from under the head to the barb shoulder
    sec = [(y, z) for y, z in circle(0, 1.3, 1.5, 24)]
    sec = [(y, min(max(z, 0.0), 2.6)) for y, z in sec]
    s = prism('x', sec, 0, Ls + 1.8)
    s = s + box(-1.4, 0.01, -3.0, 3.0, 0, 2.6)                                          # head (sits in the pelvis recess)
    s = s - box(Ls - 6.0, Ls + 2.0, -0.4, 0.4, -1, 4)                                    # split: the 2 halves flex in
    for sd in (1, -1):
        yb = sd * 1.45
        s = s + prism('z', [(Ls - 0.05, yb), (Ls + 0.15, yb + sd * 0.5), (Ls + 1.8, yb)], 0, 2.6)
    return s

def arm(side):
    sv = arm_servo(side)
    # in the servo frame: z_s = out from the body, x_s = down
    pz0, pz1 = L.HRN0 - 0.3, L.HRN0 - 0.3 + 6.0
    prims = [(1, sv.cyl(0, 0, pz0, pz1, 10.0)),
             (1, sv.box(0, 62, -8.5, 8.5, pz0, pz1)),
             (1, sv.cyl(62, 0, pz0, pz1, 9.5)),                     # hand
             (-1, sv.cyl(0, 0, pz0 - 1, pz1 + 1, 3.0)),
             (-1, sv.box(23, 35, -4.0, 4.0, pz0 - 1, pz1 + 1)), (-1, sv.box(40, 52, -4.0, 4.0, pz0 - 1, pz1 + 1))]   # lighter; 6 mm hole: horn screw goes in from outside, head sits on the horn (like the legs)
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
    v = union_all([rbox(w / 2 + cx - 12, w / 2 + cx + 12, 6 + k * 11.5, 14 + k * 11.5, -1, 3, 2.5)
                   for k in range(5) for cx in (-28, 0, 28)])
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
        for name, fn in (('pelvis', pelvis), ('torso', torso), ('deck_tray', deck_tray), ('neck_tower', neck_tower),
                         ('head', head), ('camera_hat', camera_hat), ('snap_pin', snap_pin)):
            t0 = time.time(); out[name] = fn(); print(name, len(out[name].polys), '%.1fs' % (time.time() - t0)); sys.stdout.flush()
        out['arm_L'] = arm('L'); out['arm_R'] = arm('R')
        d, dw, dh = door(); b, bw, bh = back_cover(); hc, hw, hh = head_cover()
        out['door'] = d; out['back_cover'] = b; out['head_cover'] = hc
        pickle.dump({k: v.polys for k, v in out.items()}, open('body.pkl', 'wb'))
        csg.write_stl_clean(place(out['pelvis'], Rx(180)), 'stl/pelvis.stl', 'pelvis')
        csg.write_stl_clean(place(out['torso'], Ry(90)), 'stl/torso.stl', 'torso')          # lies on its chest
        csg.write_stl_clean(place(out['deck_tray']), 'stl/deck_tray.stl', 'deck_tray')      # flat
        csg.write_stl_clean(place(out['neck_tower']), 'stl/neck_tower.stl', 'neck_tower')   # upright (legs: brim/supports not needed, they hang 4 mm)
        csg.write_stl_clean(place(out['head'], Ry(90)), 'stl/head.stl', 'head')              # lies on its face
        csg.write_stl_clean(place(out['camera_hat'], Ry(-90)), 'stl/camera_hat.stl', 'hat')  # lies on its back
        csg.write_stl_clean(place(out['snap_pin']), 'stl/snap_pin.stl', 'pin')
        for sd in 'LR':
            sv = arm_servo(sd); Rp = sv.R.T
            a = out['arm_' + sd].transform(lambda v: tuple(Rp @ (np.array(v) - sv.P)))
            csg.write_stl_clean(place(a, Rx(180)), f'stl/arm_{sd}.stl', 'arm')
        for n, (p, w, h) in (('battery_door', (d, dw, dh)), ('back_cover', (b, bw, bh)), ('head_cover', (hc, hw, hh))):
            csg.write_stl_clean(place(p), f'stl/{n}.stl', n)
        print(dict(head=(HX1 - HX0, 2 * HW, HZ1 - HZ0), HEAD_Z0=HEAD_Z0, HZ1=HZ1, HAT_Z1=HAT_Z1, CAM_ZC=CAM_ZC))
    th = threading.Thread(target=run); th.start(); th.join()
