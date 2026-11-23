# Cheo build log: legs, walking, simulation, quadruped

Hand-off notes so a new chat can pick up with full context. Written after the "legs + walking + quadruped" chat.
Maker: cheeseking (Mac). Printer: Flashforge AD5X, PCTG filament. Board: ESP32-S3 16MB. Servos: MG90S.

## 1. Robot at a glance
- Biped Cheo: 10 leg servos (5 per leg), 2 arm servos, 1 neck servo, PCA9685 (HW-170) servo driver, 2x 18650 (holder has a built-in switch).
- Firmware: CDBos (`cdbos/cdbos.ino`), now **v17** (legs). Releases through GitHub Actions (builds only `cdbos/`).
- Status when this was written: legs built; PCA9685 and the second buck converter were NOT yet installed in the torso at the start of the chat (one buck was on the arms).

## 2. Power and wiring plan
- Battery + / - go to **both** bucks: IN+ to IN+, IN- to IN- (joined with a terminal block or soldered splice, never twisted).
- Buck 1 -> ESP32-S3 + arms. Buck 2 -> PCA9685 V+ screw terminal (legs). Set both to 5 V with a multimeter first. All grounds common.
- PCA9685 logic header: GND, VCC (3V3), SDA = GPIO17, SCL = GPIO18 (50 Hz PWM).
- Servo sockets: yellow = signal, red = V+, black = GND.
- Channel map: **0-4 left leg, 5-9 right leg, 10-11 arms**. Channel 0 = the socket next to the printed "16 x 12-bit PWM" text (confirmed by the maker).
- Per leg, in order: hip roll, hip pitch, knee, ankle pitch, ankle roll.
- Servo wires are short: use extension cables and leave slack loops so they don't trip the robot while walking (keep wires >= 2 mm clear of moving parts).

## 3. Firmware v17 (what changed)
Files: `cdbos/cdbos.ino`, `cdbos/legs_walk.h`, `cdbos/leg_limits.h` (copy of `leg/leg_limits.h`; **keep both identical**, the build only sees `cdbos/`).
- `.ino` edits: `FW_VERSION 17`; prototypes (`legsWalk, legsStand, legsSit, legsCrouch, legsSetOn, legsSetJoint, legsAvailable, legsCommand`); `localCommands` calls `legsCommand` and `legsWalk(0)` on stop; AI reply tags `[walk: forward|back|stop]`, `[sit]`, `[stand]`; `#include "legs_walk.h"`; `legsBegin()` after `armsBegin()`; `legsUpdate(); legsSerial();` after `armsLoop()`; web endpoint `/legs` and a Legs card on the main page.
- `legs_walk.h` reuses the firmware's own `pcaUs`/`pcaFound` (no second Wire driver). Startup sequences servos one at a time (120 ms apart) so they don't all pull current at once. Saved trims load from Preferences keys `lt0..lt9`. Serial commands: `stand|sit|crouch|walk|back|stop|limp|hold|j <0-9> <deg>`.
- Web `/legs`: `do=walk|back|stop|stand|sit|crouch|center|on|off|trimreset`, `j=&deg=` (one joint), `trim=&d=` (nudge a trim; saved automatically). Card has pose/walk buttons, Center (for fitting horns), Hold/Limp, joint slider, trim buttons (-15,-5,-1,+1,+5,+15) and Reset trims.
- Gait tunables (`legs_walk.h`): `legStepDeg=7, legShiftDeg=8, legLiftDeg=35, legCrouchDeg=30, legCycleMs=3200, legSlewDegS=150`. Gait = sine weight-shift walk: hip pitch swing, hip roll shift, knee lift/crouch, ankle pitch = knee - hip (foot stays flat), ankle roll = -hip roll, slew limited, ramped start/stop.
- Servo angle = `servoAtZero + dir * (clamp(joint) + trim)`.
- Rule: `SETUP_*` credential strings stay blank. Upload all changed files in ONE commit (the workflow has `cancel-in-progress`, one-file commits cancel each other); re-run "Build Cheo firmware" manually if it says "operation was canceled".

## 4. Calibration (measured by the maker, legs in the air)
`leg_limits.h` columns: name, channel, min, max, servoAtZero, dir, trim.

| Joint | ch | min | max | zero | dir | trim |
|---|---|---|---|---|---|---|
| L hip roll | 0 | -15 | 45 | 90 | +1 | 45 |
| L hip pitch | 1 | -80 | 80 | 90 | +1 | 22 |
| L knee | 2 | 0 | 115 | 45 | +1 | 46 |
| L ankle pitch | 3 | -80 | 80 | 90 | +1 | 34 |
| L ankle roll | 4 | -45 | 45 | 90 | +1 | 21 |
| R hip roll | 5 | -15 | 45 | 90 | -1 | -30 |
| R hip pitch | 6 | -80 | 80 | 90 | -1 | -15 |
| R knee | 7 | 0 | 115 | **135** | -1 | 0 |
| R ankle pitch | 8 | -80 | 80 | 90 | -1 | -15 |
| R ankle roll | 9 | -45 | 45 | 90 | -1 | -90 |

- Bug fixed: R knee `servoAtZero` was 45 (servo would hit 0 deg after ~45 deg of bend); mirror of the left knee is 135.
- The big trims (R ankle roll -90, R hip roll -30) mean the horns were mounted far from centre. R ankle roll trim of -81/-90 pushes the servo against its 180 deg stop: reseat that horn (or reprint with screw access).
- Maker's "air test" photo/values: L hip pitch 7, L knee 61, L ankle pitch 19, L ankle roll 16 were the first guesses before the final numbers above.

## 5. Open hardware issues
1. R knee buttons did not respond (check the servo wire/channel 7, then the old 45 vs 135 line).
2. R ankle roll at its servo stop (see above).
3. Servos twitched/went to random positions at power-up, later stopped on their own (likely loose connection or power glitch). Startup sequencing was added. Re-check the V+ and ground joints.
4. Wire length / slack for walking.
5. Top-heavy and the battery sits behind the centre: see simulation.

## 6. Simulation
- `docs/preview/cheo-walk-preview.html`: browser preview of the gait (side/front view, joint and servo range table, sliders for step, shift, lift, crouch, cycle and trims). Flags limit hits. Maker's values: lift 19, R ankle roll trim -81, everything fits. Published copy: https://claude.ai/artifact/Gn4j5yxiZS6nyZ9sCbeRvS
- MuJoCo physics (`cheo_mujoco.py`, kept by the maker in ~/Documents; the latest version is in the earlier chat): Mac setup `pip3 install mujoco`, then `python3 cheo_mujoco.py`, viewer with `mjpython cheo_mujoco.py --view`, `--log` prints per-0.2 s data, `key=value` overrides. Defaults `step=7 shift=8 lift=35 crouch=30 cycle=3.2 batx=-0.04 batz=0.05 kp=2.0 kv=0.03`; stall torque 0.183 N*m; masses: head 45 g, neck 70 g, body 110 g, battery 114 g, each arm 29 g.
- Isaac Sim is not suitable (no macOS, needs an NVIDIA RTX GPU).
- Results (all FELL): default 3.3 s (torso leans forward, 9 cm back); batx=0 3.5 s; batx=0.02 3.0 s; batx=0 batz=0 (battery low and centred) **5.3 s, the longest**; lift=0 4.0 s. Servo torque was never the limit. Conclusion: the biped is back-heavy and top-heavy, lower and more central battery helps most.
- Still to report: `kp=6`, `kp=12`, `kp=12 batx=0 batz=0`, `kp=12 batx=-0.01 batz=0 lift=25`.
- Idea discussed, not built: rear support arm with a caster ("shopping cart" wheel). The maker decided to add it only if the sim shows he falls (he does).

## 7. Quadruped (hexagon, new direction)
Maker's requirements: MG90S servos, hexagon shape, **fully enclosed body**, **screws only for servos**, size is good.
Files: `quad/quad_hex.py`, `quad/stl/quad_deck.stl`, `quad/stl/quad_dome.stl`, `quad/quad_hex.png`.
Run from `leg/design/` (it imports `csg`, `legmodel`, `build`): `python3 ../../quad/quad_hex.py r=72 x0=56 yq=24.4 clr=0.15` and move the STLs.
- **Deck** (print flat): hex plate, corner radius R+3, four MG90S rings underneath at X0=56 (front/back) and YQ=24.4 (side), cable holes, groove for the dome. About 130 x 150 x 37 mm, 68.3 cm3.
- **Dome**: walls + roof, snap-fit to the deck, 2.4 mm walls, 28 mm inner height, roof 3 mm, USB window and switch window on the rear wall, four battery-holder posts at (+-24.2, +-43.2). About 125 x 144 x 33 mm, 67.5 cm3.
- Parameters: `R=72, X0=56, YQ=24.4, CLR=0.15, WT=2.4, DECK_T=4, GROOVE_D=2, INNER_H=28, ROOF=3`. Both meshes closed (0 open edges).
- Legs: 3 servos per leg (hip roll, hip pitch, knee), X layout on the hexagon corners. Reuse the biped STLs from `leg/stl`: front-left and rear-right use `L_hip_yoke, L_thigh, L_shin`; front-right and rear-left use `R_hip_yoke, R_thigh, R_shin`; 12x `shoe_3.0mm`. No ankle yoke or foot. Rear legs are the front legs rotated 180 deg (knees point back).
- Older rectangular version (`quad_body.stl`, 134 x 105 x 36 mm, M3 holes on 60 x 56 grid, guessed) is superseded.
- **Untested**: nothing printed yet. Check dome fit and clearance, battery position, window positions. Still undecided: where the face/screen goes, electronics layout, crawl-gait code.
- Battery advice: put the cells low and central, under the deck between the legs (battery pod) for the lowest centre of gravity.

## 8. Next steps (for the next chat)
1. Report the four `kp` MuJoCo runs.
2. Fix R ankle roll horn, R knee, wire slack.
3. Print deck + dome + 12 legs' worth of parts and test fit; send measurements back so `quad_hex.py` can be tuned.
4. Decide face window and electronics layout on the dome; then write the quadruped crawl gait (reusing `legs_walk.h`, 12 servos over PCA9685 channels 0-11, arms/neck moved to other channels).
