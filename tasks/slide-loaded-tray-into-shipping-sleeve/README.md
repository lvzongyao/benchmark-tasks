# Slide a loaded tray into a shipping sleeve

Slide the loaded tray all the way into its outer sleeve without spilling the product. Set the closed package down between the arms, about 34 cm forward from the line through their bases, and let go.

## Real packing operation

Eastface's drawer cartons for earbuds and charging accessories use a fitted inner tray and an outer sleeve. This task models the packing step after loading, when the packer guides the loose tray through the sleeve mouth until the front is flush. A skewed tray can bind and leave the kit protruding when it is picked up for overpacking. In a commercial one-ended sleeve, the closed rear stops the tray at the flush position and prevents it from sliding out the far end when handled. [Eastface electronics drawer packaging](https://eastfacepackaging.com/folding-cartons/tray-and-sleeve/) · [Junye one-ended sleeve and flush closure](https://junyepackaging.com/one-end-sealed-sleeve-box/) · [Yuan Packaging fit and binding](https://yuanpackaging.com/products/rigid-boxes/drawer-boxes)

This is a reusable PETG test kit for that closure operation. Commercial drawer packs also use clear plastic sleeves and drawer pulls. The prototype's rigid tray tab stands in for the pull, and an added external sleeve tab lets a stock gripper counterhold without squeezing the side walls. Both tabs sit outside the sliding fit. The entrance chamfers, rigid transparent top panel and loose rectangular payload are explicit prototype choices. A retail kit typically holds the earbuds and cable in fitted recesses. The loose surrogate makes any loss of containment during the simulated packing step observable. [Fushen plastic drawer construction](https://fushenpack.com/products/plastic-drawer-boxes) · [Eastface fitted electronics inserts](https://eastfacepackaging.com/folding-cartons/tray-and-sleeve/)

The closest board entries found on November 23, 2026 cover restoring a crooked chest drawer already on its wooden runners, refitting a drawer behind cabinet slide hooks, and lowering a board-game lid onto a boxed base. Here both parts start separate and free, and axial insertion transfers the loaded tray's support onto the movable sleeve floor. [Crooked chest drawer](https://tasks.robocurve.org/board?task=TK-PZRSN3) · [Refitting a loaded drawer](https://tasks.robocurve.org/board?task=TK-YFUGFS) · [Packed board-game lid](https://tasks.robocurve.org/board?task=TK-TDE3AU)

## Fixed reset

Use a stock YAM station and its wood tabletop. At reset the PETG tray (210 × 120 × 45 mm, 100 g) sits at station x=210, y=181 mm with a 100 × 60 × 30 mm, 65 g earbud-kit surrogate centered on its floor. The separate PETG sleeve (220 × 130 × 55 mm, 150 g) sits at x=210, y=-135 mm, mouth facing +y. Both have an external grasp tab. The tray and sleeve axes are parallel and all three bodies are free. The target is the tabletop near x=340, y=0 mm. The stock tabletop surface is z=747.5 mm. Reset coordinates are in the YAM station frame.

Withdraw the tray, center the same block inside it, and restore both bodies to the reset marks.

## Fidelity

Rigid colliders form the open front cavity, transparent 4 mm viewing panel and back stop. Solid 10–14 mm entrance chamfers guide the tray. Cavity width is 124 mm and floor-to-roof height is 47 mm, leaving 2 mm on each side and 2 mm above the tray before tolerances. The collision panel is solid over the entire 86 × 155 mm viewing window and uses a thin-film mass approximation. The sleeve, tray and load are rigid. Physical print fit and sliding forces remain unmeasured.

## Scoring

Score 0.25 at 30 mm insertion, 0.50 at 105 mm insertion and 0.75 with a front gap at most 3 mm and absolute relative yaw at most 4°. Every tier requires the product to be within the tray walls and below its rim, within a 1 mm tolerance. For 1.00 place the package within 35 mm of the target and keep it stable for one second after both jaws clear by 10 mm.

For a bench trial, watch the load from reset through release and use the viewing panel to check that it remains within the tray. With a ruler, measure the front seam at its midpoint along the insertion axis. Use an overhead image or a square and protractor to check for at most 4° of yaw between the long tray and sleeve edges. Locate the sleeve center relative to the work mark at x=340 mm, y=0 mm and confirm that the package rests flat and unsupported. In a one-second clip after both jaws clear by at least 10 mm, each part must stay within 2 mm and 2° of its placement pose.

## Local pilot

Activate the local simulator and run from this task folder.

```bash
mkdir -p .local
SLEEVE_DIAGNOSTICS=.local/predicates.jsonl SLEEVE_ARM_CONTACTS=.local/arm-contacts.jsonl \
  flock -F /tmp/workcellbench-sim.lock inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction 'Slide the loaded tray all the way into its outer sleeve without spilling the product. Set the closed package down between the arms, about 34 cm forward from the line through their bases, and let go.' \
  --scorer task_score --max-steps 1800 --store-frames --log-dir .local/run
python -m unittest discover -p "test_*.py"
```

Use fresh diagnostic paths for each run. To record the overhead and fixed detail views in one trial, replace `inspect-robots run` with `python capture_detail.py run`. The fixed detail appears in the right camera stream. Use `inspect-robots video` with the run's saved evaluation log to encode each camera stream.

## Verified closed-roof run

The scripted demo scored 1.0 at step 492 of 1,800, or 24.6 simulated seconds. Seed 0 uses the fixed reset. The dual-view 480 × 270 rollout took 97.44 seconds on the development simulator. Its final front gap was 0.71 mm, relative yaw was 0.50° in magnitude and the sleeve center was 11.18 mm from the placement mark. The payload stayed within the tray in all 492 sampled steps. The sleeve moved 1.08 mm from staging to first flush seating, both intended finger pairs maintained contact for 1,368 physics substeps over that interval, and the arms recorded zero inter-arm contacts.

The overhead and oblique fixed detail views were reviewed from reset through retreat. The green payload is visibly carried into the sleeve and remains faintly visible through the translucent pane after closure. The view at the mouth shows the front seating flush, and the assembled pack stays in place during jaw withdrawal.

## Physical control

The earlier, higher approach wedged at the mouth of the same closed-roof kit. The tray entered by at most 4.87 mm and the final front gap was 206.81 mm. The product remained retained for all 550 sampled steps, and the score stayed at 0.0. Both angles were reviewed from reset through retreat. Set `SLEEVE_CONTROL_HIGH_ENTRY=1` to replay its waypoints. The offline cases also cover wrong-side overlap, loss of the payload, an empty tray and a block raised above the rim.

## Earlier open-window run

Before fitting the transparent pane, the fixed-reset run scored 1.0 at step 492, or 24.6 seconds of simulated control. Recording took 92.4 seconds on the development machine. The final front gap was 0.52 mm. The load stayed inside the tray through every sampled control step. The sleeve moved at most 0.87 mm from entry staging to first flush seating. Across those 1,344 physics substeps, both fingers of each intended gripper registered contact against its part. Neither arm contacted the other.

The overhead recording and the tray wrist view were checked from reset through retreat. That run records the geometry before the top panel was fitted.

## Dream-1 attempt

The seed-0 Dream-1 trial completed 1,800 controls with score 0.0 in 277.2 seconds wall time. The model brought the parts near one another without fully inserting and releasing the loaded tray. The submitted media includes the model run and the fixed detail view of the successful scripted run.
