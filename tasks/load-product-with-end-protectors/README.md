# Load a product with loose end protectors

Fit both protectors over the corresponding ends. Lift the assembly by the protectors, lower it into the tray, and withdraw both hands. The loose protectors support the product during the carry and the tray's end walls retain the seated assembly.

## Fixed reset

Coordinates are in metres, measured from the YAM station origin. The table surface is at 0.7475 m. Box references are at the center of the footprint and on the bottom surface. All parts start with their local axes aligned with the world axes.

| Item | Geometry | Start |
| --- | --- | --- |
| Blue dummy product | 150 × 70 × 40 mm, long axis along y, 110 g | (0.245, 0, 0.7625) |
| Left and right protectors | 82 × 25 mm base, 4 mm shelf, 79 mm-wide cavity with an open lead-in, 28 mm-wide grasp land, 45 g each | (0.245, ±0.157, 0.7475), openings toward the product |
| Receiving tray | 162 × 86 mm inside, 4 mm walls and floor, rim 26 mm above the table, 280 g | (0.392, 0, 0.7475) |
| Fixed reset rest | 58 × 92 × 15 mm | (0.245, 0, 0.7475), centered under the product |

The suggested material is PETG. The unloaded cups slide off the product through their open mouths. The CAD and simulation give the protected assembly 4 mm total nominal clearance along each horizontal tray axis. The prototype has yet to be printed and fit-checked.

## Local pilot

Activate the simulator and run from this folder.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction 'Put both end protectors on the product and load the protected product into the tray.' \
  --scorer task_score --max-steps 2400 --store-frames --log-dir /tmp/protector-demo
python -m unittest test_packing
```

Set fresh file paths in `PACKING_DIAGNOSTICS` and `PACKING_ARM_CONTACTS` to collect geometric scoring and native PhysX cross-arm contact diagnostics.

Use `python capture_detail.py run` in place of `inspect-robots run` to film the support shelves and tray from the side. The scorer checks overlap in each protector's own frame, then verifies containment, floor contact, end clearance, and product yaw in the tray's frame. Dropping the product or losing either protector immediately lowers the score.

Two independent fixed-reset captures scored 1.0 in 410 control steps, or 20.5 simulated seconds. With frame recording, the runs took 67 seconds each on the development machine. Both protectors stayed fitted through the transfer, with at least 19.05 mm and 19.89 mm of left and right overlap at the sampled airborne poses.
