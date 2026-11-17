# Cheo biped: body, neck, head, arms (v2)

Fits the leg design in `leg/` (same coordinates; the pelvis replaces `L_pelvis_test_mount`).
**No glue and no screws for any electronics**: every board presses or slides into a printed holder, so parts can be swapped when the design changes.

## Parts to print (PCTG, 3 walls, 15% infill), all already turned the right way
| File | How many | Holds |
|---|---|---|
| `pelvis.stl` | 1 | the 2 hip-roll MG90S (legs hang from these) |
| `snap_pin.stl` | 6 (2 spare) | pins the pelvis to the torso, no screws |
| `torso.stl` | 1 | battery bay, deck, speaker, amp bay, arm servo pads |
| `deck_tray.stl` | 1 | ESP32-S3, PCA9685, 2 Mini560 (press-fit corners and pockets) |
| `battery_door.stl`, `back_cover.stl` | 1 each | unchanged from v1 |
| `neck_tower.stl` | 1 | SG90 stands in it, snap hooks hold its tabs |
| `head.stl`, `head_cover.stl` | 1 each | screen (window over the real lit area), INMP441 mic |
| `camera_hat.stl` | 1 (optional) | camera board standing up, camera centred |
| `arm_L.stl`, `arm_R.stl` | 1 each | unchanged from v1 |

## Assembly
1. **Neck servo:** set the SG90 to 90. Push it down into the neck tower (cable end toward the back) until both hooks click over its tabs. Feed its cable down the slot at the back.
2. **Neck tower onto the torso:** cable down through the hole in the torso top, then press the tower down onto the 2 legs until they click.
3. **Head:** plug the screen wires on, slide the screen in from the back, lift its bottom edge onto the 1 mm step, push until it clicks. Slide the INMP441 into the rails under the ceiling (sound hole up, under the small top hole). Wires down through the 2 big floor holes.
4. **Head onto the neck:** horn pressed into the triangle socket under the head (no screw needed), face forward, press the head onto the SG90 gear. Head wires down through the 2 big slots beside the tower (leave 5 cm slack). Head cover slides down from the top.
5. **Camera hat (later):** slide the camera board up into the hat's side grooves (chip + camera side facing forward), then press the hat onto the head top until both tongues click. Its wires go down through the hole under it.
6. **Chest:** speaker presses face-first between the ribs (little bumps hold it). Amp slides into the bay under the top plate from the back.
7. **Deck tray:** S3 upside down (pins up) presses into its 4 corners, USB-C to the left side hole. PCA9685 into its corners in front. Mini560s stand in their 2 pockets. Slide the tray in from the back onto the shelf.
8. **Arm servos:** in from inside, gear out through the shoulder hole, 2 long screws each.
9. **Pelvis:** push the 4 snap pins up through the pelvis into the torso bosses until they click. Battery in with the switch facing back, door on, back cover slides down.

Wiring (pins unchanged): see `book/step9_wiring.png`. The step pictures in `book/` show v1 and will be redrawn for v2.

## Numbers
- Robot about 43 cm tall with the hat (37 cm without), ~690 g.
- Physics re-checked with v2: walks at 1.0-1.6 s per step, hip roll 52% of an MG90S at 1.6 s.
- `check_views_v2.png`: cuts through the neck, head, hat, deck tray and chest showing every part has room (2 mm for wires).
