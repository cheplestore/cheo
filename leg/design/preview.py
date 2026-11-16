import numpy as np, pickle
import legmodel as L, csg
from build import part_solid
from vox import rot
from raster import tris_of, render
from PIL import Image, ImageDraw
solids = pickle.load(open('solids.pkl', 'rb'))
for sv in L.SERVOS:
    solids[sv.name + ' servo'] = part_solid(L.servo_body(sv))
    solids[sv.name + ' shoe'] = part_solid(L.shoe(sv))
home = {n: n for n in L.CHAIN}
for sv in L.SERVOS: home[sv.name + ' servo'] = home[sv.name + ' shoe'] = L.SERVO_HOME[sv.name]
COL = {'pelvis_test_mount': (.55, .58, .65), 'hip_yoke': (.93, .94, .97), 'thigh': (.93, .94, .97), 'shin': (.93, .94, .97),
       'ankle_yoke': (.93, .94, .97), 'foot': (.93, .94, .97)}
def posed(q):   # q: angles (deg) for the 5 joints, applied top-down
    out = []
    for nm, s in solids.items():
        link = home[nm]; k = L.CHAIN.index(link)
        T = lambda v: np.array(v)
        V = [np.array(p.v) for p in s.polys]
        for ji in range(min(k, 5) - 1, -1, -1):          # joints above this link, innermost first
            name, sv, kk = L.JOINTS[ji]
            if kk <= k:
                o, a = sv.axis(); R = rot(a, np.radians(q[ji]))
                V = [(v - o) @ R.T + o for v in V]
        col = (.18, .38, .85) if 'servo' in nm else (.30, .30, .34) if 'shoe' in nm else COL[link]
        polys = [csg.Poly([tuple(x) for x in v]) for v in V]
        out.append(tris_of(polys, col))
    return out
poses = [("straight", [0, 0, 0, 0, 0], -35), ("walking step: hip 35 fwd, knee 60", [0, -35, 60, -25, 0], 0),
         ("ankle roll 30 (front view)", [0, 0, 0, 0, 30], -90), ("knee bent 115 (its max)", [0, -60, 115, -55, 0], 0)]
ims = []
for title, q, az in poses:
    render(posed(q), '/tmp/p.png', azim=az, elev=15, W=420, H=640); im = Image.open('/tmp/p.png')
    ImageDraw.Draw(im).text((12, 10), title, fill=(30, 30, 30)); ims.append(im)
W = sum(i.width for i in ims); out = Image.new('RGB', (W, 640), 'white'); x = 0
for i in ims: out.paste(i, (x, 0)); x += i.width
out.save('leg_preview.png'); print('ok')
