# Push out a recessed sleeve

Push the recessed sleeve out of the plate into the catch tray using the blunt rod, then park the rod.

The sleeve sits 2 mm inside each face of a free, upright plate. The left arm can hold the plate at the raised handle while the right arm presses the sleeve's front face with the flat end of the rod. The sleeve should drop into the tray when it exits the bore. Park the rod on its green rest and release the workpiece.

Applying axial force to remove a seated part is used in equipment maintenance. The prototype models the alignment and support problem with a low-force slip fit. [SKF mounting and dismounting guidance](https://cdn.skfmediahub.skf.com/api/public/0901d196802809de/pdf_preview_medium/0901d196802809de_pdf_preview_medium.pdf)

## Setup

World x points forward and y points toward the robot's left. The station top is at z = 0.7475 m.

| Part | Dimensions and material | Fixed reset |
| --- | --- | --- |
| Blue plate | 120 × 90 × 20 mm printed PETG plate with a 24 mm through-bore, a 110 × 22 × 20 mm handle extension, a 25 × 24 × 56 mm grasp land, and two 20 × 18 × 5 mm feet. Simulated mass 190 g. | Bore center (0.315, −0.143, 0.7925), bore axis +x. |
| Yellow sleeve | 16 mm long, outer diameter 23.8 mm, bore diameter 10 mm, printed PETG. Simulated mass 11 g. | Coaxial and centered in the bore. |
| Red push rod | A 100 × 14 mm flat-ended shaft with a 25 mm-wide raised handle. Printed PETG, simulated mass 45 g. | Shaft origin (0.180, −0.143, 0.7585), shaft axis +x. |
| Catch tray | Secured tray with a 122 × 83 × 4 mm floor with an open entry and 32 mm side and rear rims. | Floor center (0.391, −0.143, 0.7495). Empty at reset. |
| Guides and rest | Two 60 × 10 × 12 mm side guides, two 60 × 16 × 4 mm lips, a 133 × 70 × 4 mm rod rest and two 35 × 8 × 14 mm rod guides. | Open-ended guides run along x. Their lips sit 1 mm above the plate feet and limit tipping while allowing axial slide. The rod lies on the aligned green rest in front of the plate. |

All coordinates and shape dimensions are specified in `task.py`. Export the plate, sleeve, rod, and optional fit coupon from `parts.scad`. The physical prototype has yet to be printed and measured. Inspect the 0.2 mm nominal diametral clearance, the axial recess and the push resistance before using the dimensions on a physical bench.

## Run locally

With the Robocurve simulation environment active, run from this task folder.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction 'Push the recessed sleeve out of the plate into the catch tray using the blunt rod, then park the rod.' \
  --scorer task_score --max-steps 1800 --store-frames --log-dir /tmp/sleeve-demo
```

Set `SLEEVE_DIAGNOSTICS` and `SLEEVE_ARM_CONTACTS` to fresh JSON Lines paths to record scoring predicates, native cross-arm contacts, left finger loads and contact locations on the plate. Use `SLEEVE_DEMO=idle` for a passive control or `SLEEVE_DEMO=no_counterhold` to play the right arm's trajectory while the left arm stays parked. Run the offline geometry and scoring tests with `python -m unittest test_task`.

For a view of the left grasp and the sleeve leaving the bore, use `python capture_detail.py` in place of `inspect-robots` with the same arguments. This changes the rendered view and keeps the station stand in its calibrated location.

The left arm descends 19 mm farther into the handle than in the earlier edge-grasp trajectory, closes on the broad opposing faces, and holds that grasp while the right arm approaches, pushes, withdraws and parks the rod. The narrower blunt rod stays wider than the sleeve bore and provides more clearance through the plate. The open tray entry catches the sleeve without requiring the rod to lever it farther past the plate.

The final overhead and oblique recordings each scored 1.00 in 425 steps (21.25 simulated seconds) with seed 0 and zero measured cross-arm contacts. The sleeve reached the tray at step 276. Both left fingers remained loaded against opposing handle faces throughout all 3,204 physics substeps from the settled grasp through rod parking. From rod alignment to rod parking, the plate moved at most 4.4 mm and rotated at most 2.6 degrees from the held pose. Recording took 113 seconds from overhead and 117 seconds from the oblique view, including startup. The right-arm-only control scored 0.50 and left the sleeve partly outside the tray.

Use `--policy dream` with the same fixed scene, instruction, seed, and 1,800-step limit for the model trial. The recorded Dream-1 attempt ran for all 1,800 steps and scored 0.00. The sleeve stayed recessed. The model trial took 309 seconds of wall time including startup and recording.

## Score

- **0.25** when the sleeve advances at least 4 mm relative to the plate.
- **0.50** when travel reaches 10 mm and at least 8 mm of the sleeve projects past the rear face.
- **0.75** when the sleeve has cleared the bore and lies within the catch tray.
- **1.00** when the sleeve is in the tray, the bore is clear, the rod is parked, and all three props stay settled with the jaws at least 10 mm clear for 20 consecutive control steps. Within that window, motion stays within 2 mm and 2 degrees per prop. The plate stays within 16 mm of its reset position and 10 degrees of its upright orientation.
- **0.00** if the sleeve is lost below the bench or no tier is satisfied.

The plate and sleeve are independent dynamic bodies with convex radial collision pieces. The simulated bore is open and the rod contacts the annulus directly. Friction and mass are prototype estimates. No attachment, ejection trigger, or pose override is used.
