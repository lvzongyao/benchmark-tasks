# Remove a transport spacer and lower a carrier

Remove the shipping block while supporting the carrier, then lower the carrier gently onto its stop.

## Real commissioning operation

The Schwinn 490 assembly manual calls for a slight lift at the inside pivot, shipping-block removal and gentle lowering. It calls for a second person when heavy or awkward handling requires one. The support transfer is the operation under test, since removing the spacer first leaves the moving mass free to drop. [Schwinn 490 assembly manual, step 5](https://download.bowflex.com/supportdocs/AM_OM/Schwinn/SCH.490.AM.OM.WARR.EN.pdf#page=16)

This benchtop fixture represents that load transfer with a short guided vertical stroke in place of the elliptical's pivoting frame. The fixture base is mounted to the bench. The small spring adds to the carrier’s gravity load. The raised carrier grip, broad spacer handle and recovery tray provide access and a repeatable reset. The spacer and final stop carry the load through direct contact. The guided motion, force scale and 0.10 m/s threshold define the bench test.

## Fixed reset

A 140 × 100 mm PETG base with a fixed lower contact stop sits at station x=277, y=0 mm. A 100 × 70 mm, 115 g carrier is vertically guided through 40 mm of nominal travel. A 35 mm-high, 73 g shipping spacer with a broad external handle supports the carrier at reset. The empty 104 × 175 mm recovery tray sits at x=397, y=-216 mm. The stock tabletop surface is z=747.5 mm. Reset coordinates are in the YAM station frame.

Raise the carrier and replace the same spacer under it. Confirm the carrier settles at the red upper reference, with the empty tray restored to its mark.

## Fidelity

A passive native prismatic joint and linear spring represent the captive guide. The spacer and lower stop support the carrier through contact. Nominal spring rate is 22 N/m, with 16 mm preload equivalent, for 0.35–1.23 N over the permitted travel. Physical spring rate and prototype speed threshold are unmeasured.

## Scoring

0.25 for withdrawing at least 10 mm while the carrier is within 3 mm of its upper reference. 0.50 for a fully clear spacer resting in its tray. 0.75 when the carrier also reaches within 3 mm of its lower reference. 1.00 for one second of stable lower-stop support with both jaws clear by 10 mm. A measured downward speed over 0.10 m/s revokes all credit. All credit is also revoked if the carrier falls below 32 mm above the lower stop while spacer travel is at least 10 mm and less than 70 mm.

### Physical trial scoring

Mark the carrier's upper and lower references and the 32 mm threshold on the fixed guide. Mark spacer travel at 10 mm and 70 mm and outline the carrier's clearance on the base. Use an overhead recording for spacer travel, full clearance, tray placement and hand retreat. In the side view, place a millimetre ruler in the plane of a high-contrast carrier marker. Mount the camera square to the guide so the scale and marker stay visible from the first grasp through release. Frame the 40 mm guide travel at 10 pixels/mm or better and record at 240 fps. Synchronize the two views with a visible signal before motion starts.

Measure the marker's downward movement between consecutive source frames, using the ruler to convert pixels to millimetres. At 240 fps, 0.10 m/s corresponds to 0.417 mm per frame. A larger measured step fails the speed check. Use the source frame timestamps if the capture rate varies. If marker or calibration uncertainty straddles the threshold, repeat with closer framing or a higher capture rate. The camera measures displacement between frames. An instantaneous peak within a frame interval may be missed.

Use the same views to find a frame where the spacer has travelled at least 10 mm and the carrier lies within 3 mm of the upper reference. In every frame with spacer travel from 10 mm inclusive to 70 mm exclusive, compare the carrier marker with the 32 mm mark and fail the trial if it goes below. Then check that the spacer is fully outside the carrier's footprint and inside the tray. At full credit, the carrier touches the lower stop with at least 10 mm of hand clearance and stays at rest for one second. The procedure uses the simulation's thresholds. The physical prototype's fit, spring load and speed threshold remain unmeasured.

## Local pilot

Activate the local simulator and run from this task folder.

```bash
mkdir -p .local
CARRIER_DIAGNOSTICS=.local/predicates.jsonl CARRIER_ARM_CONTACTS=.local/arm-contacts.jsonl \
  OMNI_CRASHREPORTER_ENABLED=0 OMNI_CRASHREPORTER_SKIPOLDDUMPUPLOAD=1 \
  flock -F /tmp/workcellbench-sim.lock python capture_detail.py run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction 'Remove the shipping block while supporting the carrier, then lower the carrier gently onto its stop.' \
  --scorer task_score --max-steps 2400 --store-frames --no-prompt --log-dir .local/run
python -m unittest discover -p "test_*.py"
```

Use fresh diagnostic paths for each run. The fixed detail appears in the right camera stream. Use `inspect-robots video` with the saved evaluation log to encode each camera stream. For the unsupported control, add `CARRIER_CONTROL=unsupported` to the same command. It repeats the normal spacer-arm actions from the same reset with the carrier arm parked.

The scripted demonstration scored 1.0 in 519 controls, or 25.95 simulated seconds and 102.30 seconds of wall time. The spacer supported 2.25 N at reset. The left gripper unloaded it before withdrawal and maintained contact at both fingertips throughout extraction. The spacer briefly touched the carrier again during the pull, reaching 1.74 N while the carrier stayed 36.81–38.86 mm above the stop. After the spacer reached the tray, the carrier descended at a peak of 22.86 mm/s. Following both hands' retreat, the stop carried 1.48 N and the spacer rested fully inside the tray. The native trace recorded zero arm-to-arm contacts.

The matched unsupported control scored 0.0 in 456 controls. With the left arm parked, the right arm completed the same spacer motion from the same reset. The carrier dropped during withdrawal. At the first control sample after premature descent was detected, the carrier stood 24.29 mm above the stop and spacer travel was 58.06 mm. Peak downward speed was 983.43 mm/s. The spacer still reached the tray and the carrier rested on the stop at the end.

## Dream-1 attempt

The seed-0 Dream-1 trial completed 2,400 controls with score 0.0 in 385.3 seconds wall time. The model repeatedly approached the fixture without completing the spacer removal and controlled descent. The submitted media includes the model run and the successful scripted run's full fixed detail view.
