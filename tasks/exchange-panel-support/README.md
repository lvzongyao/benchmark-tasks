# Exchange a support without changing panel height

Take the weight of the hinged panel, withdraw its shipping block, place the permanent foot in the locating recess and release the panel onto the permanent foot. The panel's front edge stays at its marked height throughout.

## Prototype reset

Station coordinates use x away from the robots, y toward the left robot and a tabletop at z = 0.7475 m.

| Part | Material, size, mass | Reset |
| --- | --- | --- |
| Hinged panel | PETG with steel axle, 180 × 100 × 8 mm, 160 g with a 22 × 23 mm grasp tab. The rear base is a weighted 120 × 118 mm PETG fixture, 800 g. | Hinge (0.418, 0.046, 0.7865) m, flat on the temporary block |
| Shipping block | Orange PETG, 38 × 28 × 35 mm contact body and a 24 mm-wide integral handle. Mass 80 g. | Center (0.254, 0.028, 0.765) m, handle pointing toward right arm |
| Permanent foot | Green PETG, matching contact dimensions, 103 g, 24 mm handle. | Center (0.335, −0.141, 0.765) m |
| Recess and tray | Printed rigid PETG. Open recessed channel and a marked 84 × 137 mm discard tray. | Channel center x = 0.254 m; tray block reference (0.162, 0.084, 0.769) m |

The panel hinge is passive and the base's weight is sufficient to hold the pivot on the bench. The supports and panel are independent. The hinges and print clearances have not been tested on physical parts.

## Reference operation and prototype limits

Instron's crosshead method takes a load frame's weight, removes its shipping skid, installs the leveling feet and transfers the load to those feet before removing the lifting support. The transfer provides a properly supported frame for installation. Moving a frame can affect its alignment and measurement accuracy. [Instron 5960 system support, pages 3-7–3-8](https://www.instron.com/wp-content/uploads/2024/07/5960-dual-column-table-frames-system-support.pdf), [Instron relocation and reinstallation guidance](https://www.instron.com/en/service-and-support/services/system-move-and-relocation-services/)

This scaled bench model preserves the load path and the order of support removal and replacement. A hinged light panel reduces the operation to one free edge. An arm takes its weight as a second arm exchanges the loose, handled blocks. The new block bears directly on the panel and the bench. The simulated guides locate the new foot and the fixed witness scale supplies a height datum. The bench height limits, simple locating rails and fixed-height loose foot are prototype choices. The vendor procedure uses lifting equipment and threaded leveling feet.

## Real-trial checks

Place a millimeter scale behind the panel edge and mark its starting line in a fixed side-camera view. A drop greater than 3 mm or rise greater than 5 mm fails the height-preservation condition. After the grippers withdraw, use a close side view to check that the new foot touches the panel and both locating faces are seated. Confirm the old block lies completely in the tray and the edge remains within 2 mm of the starting line for one second. The fiducial, tray and physical foot check can be inspected from the side and above. For a comparison, remove the old block without counterholding the panel and confirm that the hinge lets the edge leave the band.

## Run and score locally

```bash
SUPPORT_DIAGNOSTICS=.local/demo-predicates.jsonl inspect-robots run \
  --embodiment yam_arena --policy yam_scripted -E task=task.py \
  -E control_hz=20 -E object_jitter=0 --scorer task_score \
  --instruction "Replace the temporary support with the permanent foot while keeping the panel at its current height." \
  --max-steps 2400 --store-frames --log-dir .local/demo
python -m unittest test_scoring
```

Use `python capture_detail.py` in place of `inspect-robots` to see the panel's front and the fixed height scale. The scorer permanently invalidates the episode if the front goes over 3 mm below or 5 mm above its reset height. Full credit requires actual foot contact, at most 3 mm of locating error, final height within 2 mm of the reference, the shipping block in its tray and 20 stable, hands-clear steps. For the unsupported control, set `SUPPORT_CONTROL=unsupported`.

The revised overhead and oblique runs independently scored 1.00 in 717 control steps (35.85 simulated seconds, 144.4–144.8 seconds elapsed with frame recording). Both recordings were inspected from reset through retreat, with the wrist views showing the individual grasps and the oblique view showing the height witness, seating and load transfer. The panel's front stayed between −0.0024 and +1.3597 mm of reset. The permanent foot settled 1.805 mm from the recess center and carried 0.927 N of panel load at release. The 20-step release window completed with zero PhysX contacts between the arms.

The matched unsupported run scored 0.0 in 400 steps. The block cleared and reached the tray, but the panel edge dropped 28.68 mm and crossed the visible lower boundary. This confirms that removal alone cannot pass the support-exchange test.

The seed-0 Dream-1 recording took 457.4 seconds wall time and scored 0.0 at 2,400 actions.
