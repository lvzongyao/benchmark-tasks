# Clear loose chips from a housing pocket

Remove all loose packing chips from the housing and collect them in the cup.

## Setup and reset

Open a 140 × 90 mm housing with a shallow internal rib and discharge edge at +y. Place ten independent 6 × 6 × 4 mm plastic chips in five pairs in its pocket. The 120 × 100 mm cup is centered at (0.264, 0.186, 0.7475) m. The housing footprint is centered at (0.264, 0.014, 0.7475) m. A blunt 25 mm sweeping tool begins on its right-hand pad at x=0.287 m and y=-0.135 m.

Empty the cup and return all ten chips to their five paired positions in the housing. Reset the housing, cup, and tool to task.py coordinates.

## Scoring and fidelity

0.25, 0.50, and 0.75 for at least three, five, and eight chips currently settled inside the cup. 1.0 when all ten are collected, the housing and cup rest at tabletop height, the tool is parked, and the jaws are clear for 20 stable samples. A chip that falls off the table invalidates the trial.

Each chip is a separate rigid body. The open discharge lip and receiving cup use collision geometry. A tilt-only technique is valid. The sweeping tool approximates a blunt brush head for coarse parts; fine dust, bristles, and adhered debris are outside this benchmark.

## Reference operation and prototype

MTS workmanship guidance treats chips and other loose foreign material remaining in an assembly as defects that can damage the product or cause failure. This task isolates the removal and collection of freely mobile debris from a small open pocket with one internal rib. [MTS Foreign Objects/Debris workmanship standard](https://www.mts.com/-/media/company/pdfs/work-en/mts-foreign-objects-debris-en.pdf?as=1&hash=B0933D623F68FCECB44D745FBA834362)

This benchmark represents coarse removable fragments of protective packing found during pre-closeout inspection. The custom housing models an accessible pocket with an open discharge edge and an integral pickup boss. The demonstrated execution lifts and tilts the housing. An observer counts every piece in the cup and checks the empty housing from the side after the arm releases it. This prototype has no physical trial or calibration claim.

## Physical prototype and inspection

Print the rigid parts from `parts.scad` in PETG and use the fixed reset coordinates above. The rectangular housing is 90 × 140 mm with a 5 mm floor, three 4 mm walls rising to 26 mm above the base, a 3 mm internal rib and an integral pickup boss rising to 72 mm. The flush +y edge is the outlet. The modeled housing, receiving cup, tool and individual chips weigh 160 g, 250 g, 34 g and 0.18 g, respectively. The chips are ten separate reusable 6 × 6 × 4 mm pieces. Their centers in housing coordinates use x = −19 or +13 mm and y = −42, −23, 0, +22 or +45 mm. The cup has an approximately 120 × 100 mm interior. The 25 mm-wide optional blunt tool starts on its pad.

Only pieces resting inside the cup count. Inspect through a side view after release to distinguish pieces inside the cup from pieces beside it or under its floor. Keep the start count at ten. A chip found on the bench does not count toward collection. All ten in the cup and clear hands for one second establish full success.

## Reproduction

Activate the local YAM simulator and run from this task folder.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction 'Remove all loose packing chips from the housing and collect them in the cup.' \
  --scorer task_score --max-steps 1800 --store-frames --log-dir /tmp/clear-loose-chips-from-housing
python -m unittest test_chips
```

Set fresh paths in `CHIPS_DIAGNOSTICS` and `CHIPS_ARM_CONTACTS` to collect score and native arm-contact traces. Use the `part` selector in `parts.scad` to view the rigid prototype pieces. The parts are in millimetres.

Use `python capture_detail.py run` for a side view of the discharge edge and the cup. The scripted sequence uses gravity-assisted collection and leaves the optional blunt sweeping tool parked.

## Local verification

Both fixed-reset recordings scored 1.0 in 409 steps. All ten chips were inside the cup by step 211 and remained there through release. The maximum housing tilt was 66.42°. The housing and cup finished on the bench with the tool parked and both arms clear for 20 steps. Each recording took approximately 78–79 seconds on the development machine.

Dream-1 completed 1,800 actions at seed 0, scored 0.0, and took 323.6 seconds wall time.
