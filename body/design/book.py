"""LEGO-style assembly book for Cheo's upper body + wiring page."""
import sys, pickle, numpy as np
sys.path.insert(0, '/home/claude/leg'); sys.path.insert(0, '/home/claude/body')
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.patches import FancyBboxPatch
import csg
from csg import box
import legmodel as L
from build import part_solid
import body as B

P = pickle.load(open('/home/claude/body/body.pkl', 'rb'))
INK, INK2, SURF, GRAY = '#0b0b0b', '#52514e', '#ffffff', (0.80, 0.80, 0.82)
ORANGE = (0.98, 0.42, 0.10)                       # printed parts (the user's filament)
NEWC = {'screen': (0.20, 0.45, 0.85), 'board': (0.12, 0.12, 0.14), 'cam': (0.85, 0.25, 0.20),
        'plug': (0.95, 0.93, 0.85), 'mic': (0.15, 0.55, 0.35), 'sg90': (0.20, 0.45, 0.90), 'mg90': (0.15, 0.15, 0.17),
        'spk': (0.25, 0.25, 0.28), 'amp': (0.45, 0.25, 0.65), 's3': (0.10, 0.10, 0.12), 'bat': (0.12, 0.12, 0.12),
        'cell': (0.20, 0.55, 0.85), 'horn': (0.95, 0.95, 0.95)}

def polys(s): return [np.array(p.v) for p in s.polys]
def bx(*a): return polys(box(*a))

# ---------- electronics models (world coords) ----------
E = {}
E['screen'] = bx(B.HX1 - 2 - B.SCR_STACK, B.HX1 - 2, -B.SCR_W / 2, B.SCR_W / 2, B.SCR_Z0, B.SCR_Z0 + B.SCR_H)
E['cam_board'] = bx(B.CB_BACK, B.CB_FRONT, -15.35, 15.35, B.CB_Z0, B.CB_Z0 + 50.2)
E['camera'] = bx(B.CB_FRONT, B.CB_FRONT + 5.1, -3.95, 3.95, B.CAM_ZC - 3.95, B.CAM_ZC + 3.95)
E['plug'] = bx(B.CB_FRONT, B.CB_FRONT + 6.4, 2, 14, B.CB_Z0 + 1, B.CB_Z0 + 8)
E['mic'] = bx(B.MIC_X - 7, B.MIC_X + 7, -B.HW + 2, -B.HW + 3.6, B.MIC_Z - 7, B.MIC_Z + 7)
sgx0 = -(B.SG_L - B.SG_SHAFT)
E['sg90'] = (bx(sgx0, B.SG_SHAFT, -B.SG_W / 2, B.SG_W / 2, B.SG_CASE_Z0, B.SG_CASE_Z0 + 22.7)
             + bx((sgx0 + B.SG_SHAFT) / 2 - 16.1, (sgx0 + B.SG_SHAFT) / 2 + 16.1, -B.SG_W / 2, B.SG_W / 2, B.SG_CASE_Z0 + 15.9, B.SG_CASE_Z0 + 18.4)
             + polys(csg.prism('z', csg.circle(0, 0, 2.5, 16), B.SG_CASE_Z0 + 22.7, B.SG_CASE_Z0 + 27.5)))
E['horn'] = polys(csg.prism('z', csg.circle(0, 0, 3.5, 16), B.HEAD_Z0 - 1.6, B.HEAD_Z0)) + bx(-3, 3, 0, 16, B.HEAD_Z0 - 1.6, B.HEAD_Z0)
E['speaker'] = bx(B.IX1 - B.SPK_D, B.IX1, -B.SPK_W / 2, B.SPK_W / 2, B.Z_L3 + 3, B.Z_L3 + 3 + B.SPK_H)
E['amp'] = bx(-29.0, -5.4, -21.3, 21.3, B.Z_L3, B.Z_L3 + 6)
E['s3'] = bx(-16.7, 8.7, B.IY - 63.3, B.IY, B.Z_DECK1, B.Z_DECK1 + 1.6) + bx(-14, 4, B.IY - 63.3 + 3, B.IY - 63.3 + 22, B.Z_DECK1 + 1.6, B.Z_DECK1 + 4.8)
bat0 = B.SLOT_X1 + 0.5
E['battery'] = bx(bat0, bat0 + B.BAT_W, -B.BAT_L / 2, B.BAT_L / 2, B.Z0, B.Z0 + 4)
E['cells'] = (polys(csg.prism('y', [(bat0 + 11, B.Z0 + 11.5), (bat0 + 20, B.Z0 + 11.5), (bat0 + 20, B.Z0 + 20), (bat0 + 11, B.Z0 + 20)], -40, 40))
              + polys(csg.prism('y', [(bat0 + 24, B.Z0 + 11.5), (bat0 + 33, B.Z0 + 11.5), (bat0 + 33, B.Z0 + 20), (bat0 + 24, B.Z0 + 20)], -40, 40)))
E['bat_walls'] = bx(bat0, bat0 + B.BAT_W, -B.BAT_L / 2, -B.BAT_L / 2 + 2, B.Z0, B.Z0 + B.BAT_H) + bx(bat0, bat0 + B.BAT_W, B.BAT_L / 2 - 2, B.BAT_L / 2, B.Z0, B.Z0 + B.BAT_H)
E['arm_servos'] = polys(part_solid(L.servo_body(B.arm_servo('L')))) + polys(part_solid(L.servo_body(B.arm_servo('R'))))

# printed parts placed in the world (covers + door)
def place_flat(name, x0, ylo, z0, w, h):
    """covers are exported flat: (u along y, v up, thickness along x)"""
    out = []
    for p in polys(_flat[name]):
        q = p.copy(); out.append(np.c_[x0 + p[:, 2], ylo + p[:, 0], z0 + p[:, 1]])
    return out
d, dw, dh = B.door(); bc, bw, bh = B.back_cover(); hc, hw_, hh = B.head_cover()
_flat = {'door': d, 'back': bc, 'hcov': hc}
PR = {k: polys(csg.Solid(v)) if isinstance(v, list) else polys(v) for k, v in P.items()}
PR['door_w'] = place_flat('door', B.SLOT_X0, -dw / 2, B.Z_PEL1 - 1.2, dw, dh)
PR['back_w'] = place_flat('back', B.SLOT_X0, -bw / 2, B.Z_DECK1, bw, bh)
PR['hcov_w'] = place_flat('hcov', B.HX0 + 0.8, -hw_ / 2, B.HZ0 + B.HFL - 1.0, hw_, hh)

LIGHT = np.array([0.45, 0.55, 0.70]); LIGHT /= np.linalg.norm(LIGHT)
def shade(ps, col, alpha=1.0, shift=(0, 0, 0)):
    out = []
    for p in ps:
        q = p + np.array(shift)
        n = np.cross(q[1] - q[0], q[2] - q[0]); l = np.linalg.norm(n)
        s = 0.55 + 0.45 * abs(n @ LIGHT / l) if l > 0 else 0.7
        out.append((q, (col[0] * s, col[1] * s, col[2] * s, alpha)))
    return out

def page(n, title, parts_box, items, arrows, note, view, fname, zoom=1.0, center=None, labels=()):
    fig = plt.figure(figsize=(16, 11)); fig.patch.set_facecolor(SURF)
    ax = fig.add_axes([0.0, 0.0, 1.0, 0.88], projection='3d')
    allp, cols = [], []
    for ps, col, al, sh in items:
        for q, c in shade(ps, col, al, sh): allp.append(q); cols.append(c)
    ax.computed_zorder = False
    ax.add_collection3d(Poly3DCollection(allp, facecolors=cols, edgecolors=(0, 0, 0, 0.08), linewidths=0.2, zorder=1), autolim=False)
    V = np.concatenate(allp); mn, mx = V.min(0), V.max(0)
    c0 = (mn + mx) / 2 if center is None else np.array(center); r = (mx - mn).max() / 2 * zoom
    ax.set_xlim(c0[0] - r, c0[0] + r); ax.set_ylim(c0[1] - r, c0[1] + r); ax.set_zlim(c0[2] - r, c0[2] + r)
    ax.set_box_aspect((1, 1, 1)); ax.view_init(*view); ax.axis('off')
    for (p0, p1) in arrows:
        p0, p1 = np.array(p0, float), np.array(p1, float)
        ax.quiver(*p0, *(p1 - p0), color='#e8182b', linewidth=6, arrow_length_ratio=0.3, zorder=10)
    for (p, t) in labels:
        ax.text(*p, t, zorder=11, fontsize=13, color=INK, fontweight='bold', bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='#e8182b', lw=1.5))
    # header: big step number + title
    fig.text(0.03, 0.93, str(n), fontsize=64, fontweight='bold', color=INK, va='center')
    fig.text(0.11, 0.935, title, fontsize=26, fontweight='bold', color=INK, va='center')
    # parts box (LEGO style)
    if parts_box:
        h = 0.045 * len(parts_box) + 0.05
        fig.patches.append(FancyBboxPatch((0.03, 0.86 - h), 0.27, h, boxstyle='round,pad=0.01', transform=fig.transFigure,
                                          fc='#eef4fb', ec='#2a78d6', lw=2))
        for i, (cnt, txt) in enumerate(parts_box):
            fig.text(0.045, 0.835 - i * 0.045, f'{cnt}x', fontsize=17, fontweight='bold', color='#2a78d6')
            fig.text(0.085, 0.835 - i * 0.045, txt, fontsize=15, color=INK)
    if note:
        fig.patches.append(FancyBboxPatch((0.03, 0.025), 0.94, 0.075, boxstyle='round,pad=0.01', transform=fig.transFigure, fc='#fff7e0', ec='#eda100', lw=2))
        fig.text(0.045, 0.062, note, fontsize=15, color=INK, va='center', wrap=True)
    plt.savefig(fname, dpi=85, facecolor=SURF); plt.close()

O = ORANGE; G = GRAY
head = PR['head']; torso = PR['torso']
up = lambda k: (0, 0, k)
pages = []
# ---- 1: screen into the head (view from the back) ----
pages.append(dict(title='Screen into the head (wires plugged on first)',
    parts=[(1, '1.8" screen'), (1, 'head (cut off the 2 thin ribs on the floor)')],
    items=[(head, O, 1, (0, 0, 0)), (E['screen'], NEWC['screen'], 1, (-26, 0, 0))],
    arrows=[((B.HX0 - 14, 0, B.SCR_Z0 + 17), (B.HX0 + 10, 0, B.SCR_Z0 + 17))],
    note='Slide it in flat from the back, glass toward the front window. At the front, lift its bottom edge onto the 1 mm step, then push until it clicks.',
    view=(18, 205), zoom=0.75))
# ---- 2: camera board + mic ----
pages.append(dict(title='Camera board + microphone',
    parts=[(1, 'camera board (programmed already)'), (1, 'INMP441 mic'), (1, 'dab of hot glue')],
    items=[(head, O, 1, (0, 0, 0)), (E['screen'], G, 1, (0, 0, 0)), (E['cam_board'], NEWC['board'], 1, (-34, 0, 0)),
           (E['mic'], NEWC['mic'], 1, (-14, -6, 0))],
    arrows=[((B.CB_BACK - 62, 0, B.CB_Z0 + 30), (B.CB_BACK - 38, 0, B.CB_Z0 + 30)), ((B.MIC_X - 14, -B.HW + 12, B.MIC_Z), (B.MIC_X - 3, -B.HW + 6, B.MIC_Z))],
    labels=[((B.CB_BACK - 45, 30, B.HZ0 - 6), 'chip + camera side faces the screen (away from you)'), ((B.MIC_X - 18, -B.HW - 10, B.MIC_Z - 18), 'mic hole over the side hole')],
    note='Slide it in from the back, USB-C end down into the slot in the floor. Hot glue on the BACK where its bottom edge meets the floor. Head wires go down through the 2 floor slots.',
    view=(20, 215), zoom=0.85))
# ---- 3: SG90 up into the torso ----
pages.append(dict(title='Neck servo (set to 90 first!)',
    parts=[(1, 'SG90 at 90 degrees'), (2, 'SG90 tab screws')],
    items=[(torso, O, 1, (0, 0, 0)), (E['sg90'], NEWC['sg90'], 1, up(-30))],
    arrows=[((-5, 0, B.SG_CASE_Z0 - 40), (-5, 0, B.SG_CASE_Z0 - 10))],
    note='Reach in through the open back. Push it up through the top plate until the tabs touch, then screw the 2 tab screws up into the plastic.',
    view=(12, 150), zoom=0.7, center=(0, 0, 95)))
# ---- 4: head onto the neck ----
pages.append(dict(title='Head onto the neck',
    parts=[(1, 'SG90 horn + its tiny screws'), (1, 'SG90 center screw')],
    items=[(torso, G, 1, (0, 0, 0)), (E['sg90'], NEWC['sg90'], 1, (0, 0, 0)), (head, O, 1, up(35)), (E['screen'], G, 1, up(35)),
           (E['horn'], NEWC['horn'], 1, up(35))],
    arrows=[((0, 40, B.HZ1 + 20), (0, 40, B.HZ1 - 2)), ((0, 40, B.HZ0 + 28), (0, 40, B.HZ0 + 4))],
    labels=[((25, 40, B.HZ1 + 30), 'face points straight forward')],
    note='Screw the horn under the head floor first. Wires down through the collar (leave 5 cm slack). Press on, then the center screw through the floor hole from inside.',
    view=(14, -40), zoom=0.85, center=(0, 0, 140)))
# ---- 5: chest + deck electronics (from the back) ----
pages.append(dict(title='Speaker, amp and ESP32-S3',
    parts=[(1, 'speaker'), (1, 'amp'), (1, 'ESP32-S3'), (0, 'double-sided foam tape')],
    items=[(torso, O, 1, (0, 0, 0)), (PR['head'], G, 1, (0, 0, 0)), (E['speaker'], NEWC['spk'], 1, (-30, 0, 0)), (E['amp'], NEWC['amp'], 1, (-40, 0, 0)),
           (E['s3'], NEWC['s3'], 1, (-45, 0, 0))],
    arrows=[((B.X0 - 50, -10, B.Z_L3 + 20), (B.X0 - 20, -10, B.Z_L3 + 20)), ((B.X0 - 55, 20, B.Z_DECK1 + 6), (B.X0 - 25, 20, B.Z_DECK1 + 6))],
    labels=[((B.X0 - 40, 45, B.Z_DECK1 - 8), 'USB-C at the left side hole')],
    note='Speaker face-first against the grille. Amp flat under the neck servo. S3 flat on the middle shelf. Wire them to the pins on the last page.',
    view=(18, 150), zoom=0.75, center=(0, 0, 85)))
# ---- 6: arm servos ----
pages.append(dict(title='Arm servos (arms go on later)',
    parts=[(2, 'MG90S'), (4, 'long servo screws')],
    items=[(torso, O, 0.35, (0, 0, 0)), (E['arm_servos'], NEWC['mg90'], 1, (0, 0, 0))],
    arrows=[((0, 0, B.ARM_S), (0, 18, B.ARM_S)), ((0, 0, B.ARM_S), (0, -18, B.ARM_S))],
    note='From inside, push each one gear-first out through the shoulder hole until its tabs touch the pad. 2 long screws each.',
    view=(20, 145), zoom=0.65, center=(0, 0, 95)))
# ---- 7: pelvis + battery + door ----
pages.append(dict(title='Pelvis, battery and door',
    parts=[(1, 'pelvis'), (4, 'M2 x 8 screws (or superglue)'), (1, '2x18650 holder (switch OFF)'), (1, 'battery door')],
    items=[(torso, G, 1, (0, 0, 0)), (PR['pelvis'], O, 1, up(-35)), (E['battery'], NEWC['bat'], 1, (-45, 0, 0)), (E['bat_walls'], NEWC['bat'], 1, (-45, 0, 0)),
           (E['cells'], NEWC['cell'], 1, (-45, 0, 0)), (PR['door_w'], O, 1, (-80, 0, 0))],
    arrows=[((10, 0, -20), (10, 0, 12)), ((B.X0 - 70, 0, B.Z0 + 12), (B.X0 - 40, 0, B.Z0 + 12))],
    labels=[((B.X0 - 80, -40, B.Z0 + 34), 'switch faces the back')],
    note='Battery wire goes up through the slot in the back-right corner. Door: top edge up into the slot first, swing the bottom in, let it drop.',
    view=(16, 150), zoom=0.85, center=(-20, 0, 50)))
# ---- 8: covers, done ----
pages.append(dict(title='Covers on: done!',
    parts=[(1, 'back cover'), (1, 'head cover')],
    items=[(torso, O, 1, (0, 0, 0)), (PR['pelvis'], O, 1, (0, 0, 0)), (PR['head'], O, 1, (0, 0, 0)), (PR['door_w'], O, 1, (0, 0, 0)),
           (PR['back_w'], O, 1, up(40)), (PR['hcov_w'], O, 1, up(35))],
    arrows=[((B.X0 - 2, 0, B.Z_TOP1 + 45), (B.X0 - 2, 0, B.Z_TOP1 + 10)), ((B.HX0, 0, B.HZ1 + 40), (B.HX0, 0, B.HZ1 + 8))],
    note='Both slide down from the top. Power him from the USB-C side hole and test face, voice, mic and head turning.',
    view=(18, 150), zoom=0.62, center=(0, 0, 105)))

files = []
for i, pg in enumerate(pages, 1):
    fn = f'/home/claude/body/book/step{i}.png'
    page(i, pg['title'], pg['parts'], pg['items'], pg['arrows'], pg['note'], pg['view'], fn, pg.get('zoom', 1), pg.get('center'), pg.get('labels', ()))
    files.append(fn); print('page', i)

# ---------------- wiring page ----------------
def wiring(fname):
    fig = plt.figure(figsize=(16, 11)); fig.patch.set_facecolor(SURF)
    ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(0, 160); ax.set_ylim(0, 110); ax.axis('off')
    ax.text(3, 103, '9', fontsize=64, fontweight='bold', va='center')
    ax.text(14, 103.5, 'Wiring: pin numbers on the ESP32-S3', fontsize=26, fontweight='bold', va='center')
    # S3 board
    ax.add_patch(FancyBboxPatch((62, 18), 36, 72, boxstyle='round,pad=0.8', fc='#1c1c1f', ec='#1c1c1f'))
    ax.text(80, 86, 'ESP32-S3', color='white', fontsize=20, fontweight='bold', ha='center')
    ax.text(80, 82, '(on the middle shelf)', color='#cccccc', fontsize=12, ha='center')
    groups = [  # (module, colour, side, y, [(module pin, S3 pin, wire colour)])
        ('1.8" SCREEN', '#2a78d6', 'L', 92, [('SCL', '10', '#eda100'), ('SDA', '11', '#2fa84f'), ('RES', '12', '#7d5bd6'), ('DC', '13', '#888888'),
                                            ('CS', '14', '#e8182b'), ('VCC', '3V3', '#e8182b'), ('BL / LED', '3V3', '#e8182b'), ('GND', 'GND', '#222222')]),
        ('INMP441 MIC', '#1baf7a', 'L', 47, [('SCK', '1', '#eda100'), ('WS', '2', '#2fa84f'), ('SD', '42', '#7d5bd6'), ('L/R', 'GND', '#222222'),
                                           ('VDD', '3V3', '#e8182b'), ('GND', 'GND', '#222222')]),
        ('AMP (speaker)', '#7d5bd6', 'R', 92, [('BCLK', '4', '#eda100'), ('LRC', '5', '#2fa84f'), ('DIN', '6', '#7d5bd6'), ('VIN', '5V', '#e8182b'), ('GND', 'GND', '#222222')]),
        ('SG90 NECK', '#2a78d6', 'R', 60, [('orange (signal)', '7', '#eda100'), ('red', '5V', '#e8182b'), ('brown', 'GND', '#6b3e1e')]),
        ('CAMERA BOARD', '#333333', 'R', 38, [('5V', '5V', '#e8182b'), ('GND', 'GND', '#222222'), ('data wires', 'later', '#bbbbbb')]),
    ]
    for name, col, side, y, pins in groups:
        h = 4.2 * len(pins) + 7
        x = 6 if side == 'L' else 116
        ax.add_patch(FancyBboxPatch((x, y - h + 6), 38, h, boxstyle='round,pad=0.6', fc='white', ec=col, lw=2.5))
        ax.text(x + 19, y + 2.5, name, fontsize=15, fontweight='bold', ha='center', color=col)
        for i, (mp, sp, wc) in enumerate(pins):
            yy = y - 2.5 - i * 4.2
            if side == 'L':
                ax.text(x + 1.5, yy, mp, fontsize=13, va='center')
                ax.plot([x + 38, 62], [yy, 22 + (yy - 24) * 0.85], color=wc, lw=2.4, solid_capstyle='round')
                ax.text(x + 36.5, yy, sp, fontsize=13, va='center', ha='right', fontweight='bold',
                        bbox=dict(boxstyle='round,pad=0.25', fc='#eef4fb', ec='#2a78d6'))
            else:
                ax.text(x + 36.5, yy, mp, fontsize=13, va='center', ha='right')
                ax.plot([98, x], [22 + (yy - 24) * 0.85, yy], color=wc, lw=2.4, solid_capstyle='round')
                ax.text(x + 1.5, yy, sp, fontsize=13, va='center', fontweight='bold',
                        bbox=dict(boxstyle='round,pad=0.25', fc='#eef4fb', ec='#2a78d6'))
    ax.text(80, 50, 'BOOT button\n= pin 0\n(hold to talk)', color='white', fontsize=13, ha='center', va='center')
    ax.add_patch(FancyBboxPatch((4, 2.5), 152, 9, boxstyle='round,pad=0.6', fc='#fff7e0', ec='#eda100', lw=2))
    ax.text(6, 8.5, 'Blue boxes = the pin number on the S3.  3V3 = 3.3 V pin, 5V = 5 V pin, GND = any GND pin.', fontsize=14, va='center')
    ax.text(6, 4.8, 'Later, with the legs: PCA9685 SDA -> 17, SCL -> 18 (the IMU shares them). Battery -> 2 Mini560 -> 5 V. Not now.', fontsize=14, va='center', color=INK2)
    plt.savefig(fname, dpi=85, facecolor=SURF); plt.close()

wiring('/home/claude/body/book/step9_wiring.png'); files.append('/home/claude/body/book/step9_wiring.png')
from PIL import Image
ims = [Image.open(f).convert('RGB') for f in files]
ims[0].save('/home/claude/body/book/Cheo_assembly_book.pdf', save_all=True, append_images=ims[1:], resolution=85)
print('pdf done')
