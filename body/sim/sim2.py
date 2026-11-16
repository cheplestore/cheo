"""Walking physics for the real Cheo design (legs from leg/, body from body/)."""
import sys, pickle, numpy as np
sys.path.insert(0, '/home/claude/leg'); sys.path.insert(0, '/home/claude/body')
import sim_core as sim
from sim_core import G, STALL, COMFY, hull, foot_poly, margin
import legmodel as LM

# ---- real leg geometry (m) ----
sim.L_hip = -LM.Z_P1 / 1000; sim.Lt = (LM.Z_P1 - LM.Z_K) / 1000; sim.Ls = (LM.Z_K - LM.Z_A1) / 1000
sim.hipY = LM.HIP_Y / 1000
CFG = dict(name='Cheo v1', ankle_roll=True, arm_m=0, gap=(LM.Z_A1 - LM.Z_R2) / 1000, h_ar=(LM.Z_R2 - LM.FOOT_BOT) / 1000)
SV = 0.0134; SHOE = 0.0016
LEGP = dict(hip_yoke=0.0142, thigh=0.0084, shin=0.0124, ankle_yoke=0.0142, foot=0.0179)

# ---- body: printed part centroids from the CAD ----
def centroid(polys):
    V = 0; C = np.zeros(3)
    for p in polys:
        a = np.array(p.v[0])
        for i in range(1, len(p.v) - 1):
            b, c = np.array(p.v[i]), np.array(p.v[i + 1]); v = np.dot(a, np.cross(b, c)) / 6
            V += v; C += v * (a + b + c) / 4
    return C / V / 1000, V / 1e9
bp = pickle.load(open('/home/claude/body/body.pkl', 'rb'))
def upper_list(print_factor):
    u = []
    for n in ('pelvis', 'torso', 'head'):
        c, v = centroid(bp[n]); u.append((n + ' (print)', v * 1270 * print_factor, tuple(c)))
    u += [('back cover + door', 0.016 * print_factor, (-0.031, 0, 0.075)),
          ('2x18650 + holder', 0.116, (-0.0068, 0, 0.037)),
          ('S3, PCA9685, Mini560s, wires', 0.052, (0.0, 0, 0.062)),
          ('speaker + amp', 0.030, (0.012, 0, 0.092)),
          ('SG90 neck', 0.009, (-0.005, 0, 0.110)),
          ('screen, camera board, mic', 0.036, (0.004, 0, 0.152)),
          ('arm servos', 2 * SV, (0.0, 0, 0.105))]
    return u
ARM_PRINT = 0.0096; SHOULDER_Y, SHOULDER_Z, ARM_R = 0.056, 0.1053, 0.033

def make_masses(print_factor, arms=True):
    up = upper_list(print_factor)
    if not arms: up = [x for x in up if x[0] != 'arm servos']
    def masses(P, legs, arm_swing, cfg):
        out = [(n, m, P + np.array(o), None) for n, m, o in up]
        for s, sg in (('L', 1), ('R', -1)):
            if arms:
                sh = P + np.array([0, sg * SHOULDER_Y, SHOULDER_Z]); ang = arm_swing * (-sg)
                out.append((f'arm{s}', ARM_PRINT * print_factor, sh + np.array([ARM_R * np.sin(ang), 0, -ARM_R * np.cos(ang)]), None))
            L = legs[s]; pf = print_factor
            parts = [(SV + SHOE, L['H'], 0),                                            # hip roll servo (in the pelvis)
                     (LEGP['hip_yoke'] * pf, (L['H'] + L['Hp']) / 2, 1),
                     (SV + SHOE, L['Hp'], 2), (LEGP['thigh'] * pf, (L['Hp'] + L['K']) / 2, 2), (SV + SHOE, L['K'], 2),
                     (LEGP['shin'] * pf, (L['K'] + L['Ap']) / 2, 4), (SV + SHOE, L['Ap'], 4),
                     (LEGP['ankle_yoke'] * pf, (L['Ap'] + L['A']) / 2, 6),
                     (SV + SHOE, L['A'], 7), (LEGP['foot'] * pf, L['sole'] + np.array([0.005, 0, 0.003]), 7)]
            for i, (m, p, ci) in enumerate(parts): out.append((f'leg{s}{i}', m, np.array(p, float), (s, ci)))
        return out
    return masses

def walk(print_factor=0.75, T=1.6, step=0.020, arms=True, lift=0.015):
    sim.masses = make_masses(print_factor, arms)
    return sim.run(CFG, step=step, lift=lift, T=T, footY=LM.LEG_Y / 1000)

def peak(r):
    jn = {}
    for x in r:
        for (s, j), v in x['tq'].items(): jn[j] = max(jn.get(j, 0), v)
    return jn

def stand(print_factor=0.75, arms=True):
    sim.masses = make_masses(print_factor, arms)
    zp = sim.L_hip + (sim.Lt + sim.Ls) * np.cos(np.radians(15)) + CFG['gap'] + CFG['h_ar']
    P = np.array([0.0, 0, zp]); legs = {}
    for s_, sg in (('L', 1), ('R', -1)):
        sole = np.array([0, sg * LM.LEG_Y / 1000, 0.0]); L = sim.leg_ik(P + np.array([0, sg * sim.hipY, 0]), sole, CFG); L['sole'] = sole; legs[s_] = L
    ms = sim.masses(P, legs, 0, CFG); m = np.array([x[1] for x in ms]); p = np.array([x[2] for x in ms])
    com = (m[:, None] * p).sum(0) / m.sum(); M = m.sum()
    poly = hull(np.vstack([foot_poly(legs['L']['sole']), foot_poly(legs['R']['sole'])]))
    d = {'forward': poly[:, 0].max() - com[0], 'backward': com[0] - poly[:, 0].min(), 'sideways': poly[:, 1].max() - com[1]}
    tilt = {k: np.degrees(np.arctan2(v, com[2])) for k, v in d.items()}
    return com, M, {k: M * G * v / com[2] for k, v in d.items()}, d, tilt, zp

if __name__ == '__main__':
    for pf, lab in ((0.75, 'realistic print (15% infill)'), (1.0, 'solid prints')):
        com, M, push, d, tilt, zp = stand(pf)
        print(f'== {lab}: weight {M*1000:.0f} g, balance point {com[2]*1000:.0f} mm high, {com[0]*1000:+.1f} mm fwd')
        print('   push to tip over (N):', {k: round(v, 2) for k, v in push.items()}, ' tip angle (deg):', {k: round(v, 1) for k, v in tilt.items()})
        for T in (2.0, 1.6, 1.2, 1.0, 0.8, 0.6):
            r, _ = walk(pf, T)
            pk = peak(r); mg = min(x['margin'] for x in r)
            print(f'   step {T:.1f}s: balance margin {mg*1000:+.1f} mm   peak torque (N.m): ' + ', '.join(f'{k} {v:.3f}' for k, v in pk.items()))
    com, M, *_ = stand(0.75, arms=False); r, _ = walk(0.75, 1.6, arms=False)
    print(f'== no arms: {M*1000:.0f} g, peaks', {k: round(v, 3) for k, v in peak(r).items()})
    print('upper body items:'); [print('  ', n, round(m * 1000, 1), 'g at', np.round(np.array(o) * 1000, 0)) for n, m, o in upper_list(0.75)]
