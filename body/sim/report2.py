import sys; sys.path.insert(0, '.')
import numpy as np, sim2
from sim2 import walk, stand, peak, STALL, COMFY, G
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
BLUE, INK, INK2, GRID, SURF = '#2a78d6', '#0b0b0b', '#52514e', '#e4e3df', '#fcfcfb'
plt.rcParams.update({'font.size': 11, 'text.color': INK, 'axes.labelcolor': INK2, 'xtick.color': INK2, 'ytick.color': INK2,
                     'axes.edgecolor': GRID, 'axes.facecolor': SURF, 'figure.facecolor': SURF, 'axes.spines.top': False, 'axes.spines.right': False})
com, M, push, d, tilt, zp = stand(0.75)
Ts = [2.0, 1.6, 1.4, 1.2, 1.0, 0.9, 0.8, 0.7, 0.6]
runs = {T: walk(0.75, T)[0] for T in Ts}
mg = [min(x['margin'] for x in runs[T]) * 1000 for T in Ts]
pk = peak(runs[1.6]); pk_noarm = peak(walk(0.75, 1.6, arms=False)[0]); M_noarm = stand(0.75, arms=False)[1]
head_top = zp + 0.184
v = np.sqrt(3 * G * com[2]) * head_top / com[2] * 0.75          # head speed at impact (tipping rod, rough)
F = 0.068 * v ** 2 / (2 * 0.004)                                  # head stops in ~4 mm

fig = plt.figure(figsize=(15, 10.5))
fig.text(0.04, 0.955, 'Cheo v1 physics check: real legs + new body', fontsize=22, fontweight='bold')
fig.text(0.04, 0.925, f'Weight {M*1000:.0f} g (15% infill)  ·  balance point {com[2]*1000:.0f} mm off the floor  ·  {head_top*1000:.0f} mm tall  ·  20 mm steps',
         fontsize=12, color=INK2)
# 1 torques
ax = fig.add_axes([0.05, 0.50, 0.42, 0.36])
names = ['hip roll', 'hip pitch', 'knee', 'ankle pitch', 'ankle roll']
vals = [pk[n] for n in names]
ax.set_xlim(-0.6, 4.6); ax.bar(names, vals, width=0.55, color=BLUE, zorder=3)
for i, val in enumerate(vals): ax.text(i, val + 0.004, f'{val/STALL*100:.0f}%', ha='center', color=INK, fontsize=11)
ax.axhline(COMFY, color=INK2, lw=1, ls='--', zorder=2); ax.text(1.5, COMFY + 0.003, 'comfortable (50%)', ha='center', color=INK2, fontsize=10)
ax.axhline(STALL, color=INK2, lw=1, zorder=2); ax.text(4.45, STALL + 0.003, 'MG90S stall', ha='right', color=INK2, fontsize=10)
ax.set_ylim(0, 0.21); ax.set_ylabel('peak torque (N·m)'); ax.grid(axis='y', color=GRID, zorder=0)
ax.set_title('Hardest moment for each joint, walking at 1.6 s per step\n(% of what an MG90S can hold)', loc='left', fontsize=13)
# 2 margin vs speed
ax = fig.add_axes([0.55, 0.50, 0.41, 0.36])
ax.plot(Ts, mg, color=BLUE, lw=2, marker='o', ms=7, zorder=3)
ax.axhline(0, color=INK2, lw=1); ax.text(1.9, -2.5, 'below 0 = he falls', ha='right', color=INK2, fontsize=10)
ax.axvspan(1.2, 1.6, color='#cde2fb', zorder=0); ax.text(1.4, -18, 'best range', ha='center', color=INK2, fontsize=10)
ax.set_xlabel('seconds per step (slower →)'); ax.set_ylabel('balance margin (mm)'); ax.grid(color=GRID, zorder=0)
ax.set_title('How far his balance point stays inside his feet', loc='left', fontsize=13)
# 3 top view
ax = fig.add_axes([0.05, 0.06, 0.30, 0.36])
r = runs[1.6]; seen = set()
for x in r[::10]:
    f = x['f']
    for s in 'LR':
        so = f['legs'][s]['sole']
        if so[2] < 0.0005:
            k = (round(so[0], 3), round(so[1], 3))
            if k in seen: continue
            seen.add(k); p = sim2.foot_poly(so) * 1000
            ax.fill(p[:, 0], p[:, 1], color='#e4e3df', ec='#a8a69f', lw=1, zorder=1)
z = np.array([x['zmp'] for x in r]) * 1000
ax.plot(z[:, 0], z[:, 1], color=BLUE, lw=2, zorder=3)
ax.set_aspect('equal'); ax.set_xlabel('forward (mm)'); ax.set_ylabel('left / right (mm)')
ax.set_title('Top view: footprints and his balance path', loc='left', fontsize=13)
# 4 summary
ax = fig.add_axes([0.40, 0.04, 0.56, 0.40]); ax.axis('off')
lines = [
 ('Verdict: he can walk, slowly.', 15, 'bold'),
 (f'Stays balanced at 1.0 to 2.0 s per step ({min(mg[0:5]):.0f}+ mm to spare). At 0.8 s it gets thin, at 0.6 s he falls.', 11, 'normal'),
 ('', 6, 'normal'),
 (f'Hip roll is the hardest-working joint: {pk["hip roll"]/STALL*100:.0f}% of the servo, right at the "comfortable" line.', 11, 'normal'),
 (f'Ankle roll {pk["ankle roll"]/STALL*100:.0f}%, ankle pitch {pk["ankle pitch"]/STALL*100:.0f}%. Nothing comes near stalling at 1.2-1.6 s per step.', 11, 'normal'),
 (f'Without arms: {M_noarm*1000:.0f} g, hip roll drops to {pk_noarm["hip roll"]/STALL*100:.0f}%. Small gain; keep the arms.', 11, 'normal'),
 ('', 6, 'normal'),
 (f'Standing still, a push of about {min(push.values()):.1f} N tips him over (like a {min(push.values())/G*1000:.0f} g weight pushing sideways).', 11, 'normal'),
 (f'Weakest direction: backward, he tips past {tilt["backward"]:.0f} deg. Sideways he can lean {tilt["sideways"]:.0f} deg.', 11, 'normal'),
 ('', 6, 'normal'),
 (f'Falls: his head lands at about {v:.1f} m/s, roughly {F:.0f} N on the SG90 neck (a {F/G:.1f} kg hit).', 11, 'normal'),
 ('That can strip the SG90\'s plastic gears. The code should go limp and', 11, 'normal'),
 ('turn the head to the side when the IMU feels a fall.', 11, 'normal'),
 ('', 6, 'normal'),
 ('Battery low in the hips is working: balance point is only 1 mm off center.', 11, 'normal'),
]
yy = 0.98
for t, s, w in lines:
    ax.text(0, yy, t, fontsize=s, fontweight=w, va='top', transform=ax.transAxes); yy -= 0.068 if s > 6 else 0.035
plt.savefig('/home/claude/biped/sim2/cheo_v1_physics.png', dpi=100)
print(dict(M=M, com=com, mg=dict(zip(Ts, np.round(mg, 1))), v=v, F=F, push=push, tilt=tilt))
