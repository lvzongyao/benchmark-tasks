# Disconnect a plug with a secondary lock

Release both connector locks and separate the plug from the socket without breaking either part.

The red safety rail blocks the yellow spring latch until the rail retracts. Pull the rail, regrip across the yellow latch, and squeeze while withdrawing the plug. Place the two halves on the two green pads.

## Work reference and simulation fidelity

TE Connectivity’s AMPSEAL 16 application specification calls for pulling the connector position assurance slider rearward, holding down the rear latch pad, and pulling the housings straight apart to reach damaged or misplaced contacts. Replacing the peripheral seal uses the same disconnect procedure. The secondary lock prevents accidental release in vibration-prone equipment. [TE application specification 114-13065 §3.4 B, Figures 11–12](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocFormat=pdf&DocLang=English&DocNm=114-13065&DocType=Specification+Or+Standard&PartCntxt=776487-1#page=9), [TE AMPSEAL 16 service guidance](https://www.te.com/en/faqs/ampseal-16-connectors.html)

The reference pair is the four-position key-A AMPSEAL 16 plug 776487-1 and inline cap 776488-1. TE publishes engineering drawings for both housings. These part numbers identify the real service example and support measuring the latch and slider against the printed demonstrator. [Plug drawing and specifications](https://www.te.com/en/product-776487-1.html) · [Cap drawing and specifications](https://www.te.com/en/product-776488-1.html)

The red rail physically obstructs the yellow latch until it clears the latch foot. The tooth then has to move across the retaining shoulder before the plug can travel along the mating axis. The 12 mm rail stroke and 6.7 mm latch stroke are enlarged. The 22 × 23 mm rail tab extends behind the plug for jaw access. A side grip collar lets a parallel jaw compress the latch across the housing. The real connector uses a rear-facing finger pad. The socket grip center is 48 mm behind the seam. The demo tilts each wrist outward by 15° around its closing axis for clearance during the ordered release and withdrawal. The two staging pads add an explicit endpoint for the loose parts. The demonstrator has no measured force or dimensional calibration against the reference pair.

## Fixed reset and bill of materials

Station world x points forward, y toward the robot's left, and the tabletop is at z = 0.7475 m. Both coupled bodies start at the mating seam origin (0.310, 0, 0.7685), and the contact axes are parallel to y.

| Item | Material and dimensions |
| --- | --- |
| Teal socket | One PETG housing, 46 × 69 × 36 mm, 82 g simulated mass. Internal shoulder and a 26 mm-wide rear grip. |
| Orange plug | One PETG housing, 40 × 89 × 34 mm excluding guides and sliders, 81 g simulated mass. Nose projects 20 mm, with clearance for the latch. |
| Red safety slider | One PETG rail with a 22 × 23 mm tab and a 12 mm captive stroke. Simulated mass 8 g. Reset at its closed stop. |
| Yellow primary latch | One PETG tooth and grasp collar, simulated mass 8 g. A 6.7 mm lateral prismatic joint with a nominal 0.39 N/mm spring. Reset extended against its lower travel stop. |
| Green pads | Three 3 mm-thick, 78 mm-wide pads. The center support is 186 mm long at y = −0.018 m. The socket pad is 100 mm long at y = 0.133 m and the plug pad is 160 mm long at y = −0.195 m. All three have center x = 0.310 m and center z = 0.749 m. |

Export the body envelopes using `parts.scad`. The physical prototype needs captive guides and end stops for both sliders, plus a return spring on the latch. Use the white features as paint marks on the adjacent surfaces. The nominal stroke, spring rate and interlock clearances need measurement on that prototype.

## Run locally

Activate the installed Robocurve environment and run from this folder.

```bash
INTERLOCK_DIAGNOSTICS=.local/predicates.jsonl INTERLOCK_CONTACTS=.local/arm-contacts.jsonl \
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=480 -E cam_height=360 \
  --instruction 'Release both connector locks and separate the plug from the socket without breaking either part.' \
  --scorer task_score --seed 0 --max-steps 2400 --store-frames --log-dir .local/run
```

Create `.local` and choose fresh diagnostic paths. Run `python capture_detail.py` in place of `inspect-robots` for an oblique camera view. Set `INTERLOCK_DEMO=locked` to pull on the rear plug housing with the safety and latch engaged. Set `INTERLOCK_DEMO=locked_latch` to try the production latch grasp before applying the pull with the safety engaged. Both use the production demonstration's first 32 mm withdrawal demand. `INTERLOCK_DEMO=half` commands a 6 mm safety withdrawal before the same latch probe and pull. Set `INTERLOCK_DEMO=idle` for the untouched reset.

Run the offline checks with `python -m unittest test_checks`.

The installed environment uses Inspect Robots 1.0.0, YAM Arena 0.1.0, Isaac Lab Arena 0.3.0 and Isaac Sim 6.1.0.0. Both locks and the retaining shoulder are physical. The spring closes the primary latch by default.

## Local verification

With seed 0, the scripted demonstration scored 1.00 in 444 control steps, or 22.2 simulated seconds and 87.6 seconds wall time. The slider passed 10 mm at step 107, the primary latch first cleared at step 221, and axial withdrawal passed 15 mm at step 248. Before the staging lift, the socket moved at most 1.12 mm along the mating axis and 3.66 mm overall. Both halves settled on their separate pads for 20 steps with the jaws clear. The main and right-wrist recordings show the approach, lock sequence, withdrawal, placement, release and retreat. The native contact trace recorded zero contacts between arms.

The locked-latch probe scored 0.00. Through the matched 32 mm withdrawal command, maximum slider travel was 0.30 mm, latch compression 0.64 mm and relative axial withdrawal 2.15 mm. The right jaws made contact with the latch. One fingertip stayed caught in the collar during release, lifting the still-coupled pair.

The housing-grip control scored 0.00 in 296 steps with seed 0. Both right fingertips held the orange housing for all 38 steps of the matched 32 mm withdrawal demand. The housings moved only 1.94 mm relative to each other along the mating axis during that demand. The two parts remained coupled and came to rest on the center pad after both hands released and retreated. The native trace recorded zero contacts between arms.

## Scoring

- **0.25** when the safety rail retracts by at least 10 mm.
- **0.50** when the enabled latch is depressed by at least 4 mm, clearing its retaining shoulder.
- **0.75** when the plug travels at least 15 mm after legitimate latch clearance.
- **1.00** when the plug's nose is at least 30 mm clear of the socket, the separate parts rest on their marked staging pads, and both grippers remain at least 10 mm clear for 20 stable control steps.
- **0.00** if the plug is forced past the shoulder without a clearance event, a part is lost, or none of the tiers is satisfied.

Return the red safety rail only after resetting the plug and releasing the yellow latch. The scores depend on joint positions and relative phase of physical parts.

### Checks on a printed demonstrator

Mark the red rail against the plug body at its closed stop and 10 mm rearward. Mark the yellow latch against the same body at rest and 4 mm inward. Film both witness marks from above to verify that the latch stays depressed until its tooth passes the shoulder. Put a ruler parallel to the mating axis, zeroed at the coupled seam, to check 15 mm of axial withdrawal. After release, check at least 30 mm from the plug nose to the socket face. Keep both parts and the grippers in view for one second to confirm separate staging, at least 10 mm of jaw clearance, and motion no greater than 2 mm or 2°.

## Related tasks

TK-462BUT removes a pipe using a release collet, TK-8NDRZH disconnects a pneumatic fitting, and TK-4D4BN7 uses a safety loop and rocker latch. This task's captive secondary slider mechanically blocks access to the independent primary latch until it retracts. TK-XD266Y fits a terminal-level wedgelock after inserting a crimped contact. TK-VEXFRM describes mating sealed housings and closing their CPA. The disconnection operation here acts on an already mated housing pair, retracts the housing CPA, and uses a separate latch action under withdrawal load. [TK-XD266Y](https://tasks.robocurve.org/board?task=TK-XD266Y) · [TK-VEXFRM](https://tasks.robocurve.org/board?task=TK-VEXFRM)

## Dream-1 attempt

The seed-0 Dream-1 trial completed 2,400 controls with score 0.0 in 465.2 seconds wall time. The supplied media includes this attempt and the synchronized overhead and right-wrist views of the 444-action scripted success.
