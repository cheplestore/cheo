# Cheo biped: body, neck, head, arms (v1)

Fits the leg design in `leg/` (same coordinates: the pelvis replaces `L_pelvis_test_mount`).

## Parts to print (PCTG, 3 walls, 15% infill)
| File | Print it | Holds |
|---|---|---|
| `pelvis.stl` | flat side down, as exported | the 2 hip-roll MG90S (legs hang from these) |
| `torso.stl` | lying on its chest, as exported | battery bay, deck, ESP32-S3, PCA9685, Mini560s, speaker + amp, 2 arm MG90S, neck SG90 |
| `battery_door.stl` | flat | covers the battery, window for the on/off switch |
| `back_cover.stl` | flat | slides down into the back of the chest |
| `head.stl` | lying on its face, as exported | 1.8" screen, camera board, camera, INMP441 mic |
| `head_cover.stl` | flat | slides down into the back of the head |
| `arm_L.stl`, `arm_R.stl` | flat | press onto the arm servo horns |

## Assembly
1. **Hips:** hip-roll servos go into the pelvis rings the same way as the leg rings (2 long screws each).
2. **Pelvis to torso:** 4 small self-tapping screws (M2 x 8) up through the pelvis into the torso's front bosses. Leg servo cables come up through the 2 slots in front of the battery, then through the slot in the deck.
3. **Battery:** holder lies in the bay with its switch + wire facing the back. Wire goes up through the slot in the back-right corner of the deck.
   **Door:** push its top edge up into the slot under the deck, swing the bottom in, let it drop into the groove in the pelvis. To open: lift it a little, swing the bottom out.
4. **Deck (middle shelf):** ESP32-S3 (USB-C toward the left side hole) and PCA9685 lie flat, Mini560s stand against the side walls. Double-sided foam tape.
5. **Chest:** speaker face against the grille, amp lying flat behind/under the neck servo (foam tape).
6. **Arms:** each MG90S goes in from the inside, gear through the shoulder hole, tabs screwed to the pad (2 long screws). Arm presses onto the horn (same triangle pocket as the legs), short screw through the arm.
7. **Neck:** SG90 goes up through the top plate from inside, its 2 tab screws up into the plate. Screw the SG90 horn to the head's floor with the horn's tiny screws, press the head onto the servo, then the center screw through the hole in the head floor. Set the SG90 to 90 (straight ahead) first.
8. **Head:** screen slides in from the back and clicks past the little ridges. The LAFVIN ESP32S3-Camera board stands up behind it, chip + camera side facing forward, USB-C end down in the floor groove; a dab of hot glue at the back of its bottom edge stops it tipping. Put the screen in first: slide it in flat along the floor, lift its bottom edge onto the 1 mm step at the front, push until it clicks. Its camera looks out the forehead window (see `head_section.png`). Program the camera board before putting it in (its USB-C faces the floor). INMP441 taped inside the right side, its sound hole over the small side hole. Cables go down through the 2 slots in the head floor and the neck.
9. Back cover and head cover slide down from the top.

## Numbers
- Torso 62 x 100 x 94 mm, head 41 x 62 x 53 mm, about 37 cm tall overall.
- Legs still clear the bigger pelvis: hip roll outward stops at 55 deg (code limit is 45).
- Estimated weight: printed parts ~340 g solid (less with 15% infill) + ~400 g servos, battery, boards = **650-750 g**, above the ~600 g the walking simulation liked. Weigh him once built; if the hip servos struggle, the arms are the first thing to leave off (~47 g).
