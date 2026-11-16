# Cheo biped leg v1 (5 servos)

![leg](leg_preview.png)

## Print (PCTG, 3 walls, 15-20% infill, tree supports on)
| File | Per leg | Notes |
|---|---|---|
| `pelvis_test_mount` | 1 (only for testing one leg) | Holds the hip roll servo; clamp it to a table |
| `hip_yoke` | 1 | Supports needed |
| `thigh` | 1 | Prints flat, no supports |
| `shin` | 1 | Supports needed |
| `ankle_yoke` | 1 | Supports needed |
| `foot` | 1 | Prints flat |
| `shoe_3.0mm` | 5 | One slides onto the bottom of each servo |

## Hardware
- 5x MG90S per leg, with their tab screws and single-arm horns
- 5x M2x5 screws (the pivot each joint turns on, into the shoe)

## Assembly
1. Center every servo first: set it to 90° (knee: see below) **before** attaching anything.
2. Slide each servo into its ring from the outside, horn side out, and screw the tabs.
3. Slide a shoe onto the servo's bottom from the long end.
4. Screw the single-arm horn into the bracket's horn pocket, pointing toward the bridge.
5. Fit the bracket over the servo, press the horn onto the spline, then put the M2x5 pivot screw through the counterbored hole into the shoe.
6. Knee only: with the knee servo at 90°, attach the shin bent 45° backward (so the knee can bend 0-115°).

If a bracket is too tight or loose between the horn and the shoe, use the 2.5 or 3.5 mm shoe.

## Sizes
Hip roll axis to floor: 177 mm. Hip roll axes 18 mm apart, feet centered 52 mm apart (26 mm from the center line).
