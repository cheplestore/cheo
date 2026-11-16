# Cheo biped leg v1: how far each joint can move

Tested by turning each joint 1° at a time (from the straight-leg pose) until printed parts or servos touched.
Test cubes: 0.4 mm.

| Joint | Physical limit (parts touch) | What stops it | Safe software range | Servo at straight leg |
|---|---|---|---|---|
| Hip roll | out 78°, in 122° (test stand) | hip yoke hits the test stand | **−15° to +45°** | 90° |
| Hip roll, other leg standing | in **3°** | feet touch each other | (only when one leg moves in alone) | |
| Hip pitch | ±107° | thigh hits hip yoke | **−80° to +80°** | 90° |
| Knee | ±121° | shin hits thigh | **0° to 115°** (no bending backward) | 45° (knee bent 45° at servo 90°) |
| Ankle pitch | ±105° | ankle yoke hits shin | **−80° to +80°** | 90° |
| Ankle roll | ±91° | foot hits ankle yoke | **−45° to +45°** | 90° |

The servos themselves only turn 0–180° (±90° from center), so for most joints the servo's own end-stop comes before any parts touch.

Hip roll limits will be re-tested with the real pelvis (the test stand stands in for it).
