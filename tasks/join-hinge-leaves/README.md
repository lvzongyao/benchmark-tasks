# Join two hinge leaves with a pivot pin

Align the separate flap eye between the two base ears, push the pin through all three, turn the flap while the pin remains engaged, then release the assembly with the flap on its stop.

## Prototype setup

The hinge axis runs along +y on the YAM station. The tabletop height is 0.7475 m. The base, flap and pin are independent dynamic bodies. The bores use convex radial sectors with chamfered mouths. The simulation uses contact between the pin and eyes to constrain the flap.

| Part | Prototype dimensions | Fixed start |
| --- | --- | --- |
| Base | PETG, 130 × 99 mm foot with two 10 mm-thick ears. Near bore 11.6 mm, relieved far bore 12.6 mm. Clear gap 35 mm. Simulated mass 460 g. | Hinge axis center (0.270, −0.058, 0.8115) m. |
| Flap | PETG, 80 × 60 × 10 mm leaf with 26 mm central eye, a raised grasp handle and the same 11.6 mm bore. Simulated mass 100 g. | Eye center (0.235, 0.145, 0.7605) m on the left stand. |
| Pin | 10 mm shaft, 65 mm usable length, 25 mm-wide handle. Simulated mass 32 g. | Pin reference (0.270, −0.200, 0.7555) m on the right rest, tip toward +y. |

The fixed supports can be printed in PETG. The flap rests on two blocks measuring 50 × 62 × 6 mm and 25 × 47 × 11 mm, centered at (0.283, 0.145, 0.7505) m and (0.301, 0.211, 0.7530) m. The pin rest is a 58 × 122 × 3 mm pad centered at (0.270, −0.200, 0.7490) m. Its two 6 × 38 × 12 mm guide blocks are centered at (0.261, −0.184, 0.7535) m and (0.279, −0.184, 0.7535) m.

The flap needs to turn by at least 15 degrees relative to the base with continuous capture. At full score the shaft protrudes at least 3 mm beyond the far ear, both jaws are clear, and the flap rests on the stop for 20 consecutive control steps. Pose drift during that window stays within 2 mm and 2 degrees. A pin that misses the flap earns zero.

## Local pilot

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction 'Join the flap to the base with the pivot pin and show that the flap can turn while remaining attached.' \
  --scorer task_score --max-steps 2400 --store-frames --log-dir /tmp/hinge-demo
```

Run from the task folder with the simulator activated. Use `python capture_detail.py run` in place of `inspect-robots run` to record the far ear and flap from the side. Run `python -m unittest test_hinge` for the offline tests. Use fresh paths in `HINGE_DIAGNOSTICS` and `HINGE_ARM_CONTACTS` to log geometric scoring and native cross-arm contact reports.

Two independent runs from the fixed reset scored 1.0 in 664 control steps (33.2 simulated seconds and 123–126 wall-clock seconds with frame recording on the development machine). Both recorded a 21.6° turn with the pin still through the three eyes. The final tip protrusion measured 6.22 mm. The physical prototype has yet to be printed and fit-checked.
