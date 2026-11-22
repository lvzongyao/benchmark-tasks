# Separate nested reusable trays

Separate the nested trays and place the orange tray upright on the left pad and the blue tray upright on the right pad. At the reset pose, two tabs on the top tray lie under two small ledges on the bottom tray. The ledges are 4 mm long along the slide axis. An 8 mm nominal shift gives clearance when the tray is square. The pair can be held in one place while the upper tray shifts along its long axis until the overlap clears, then separated vertically. The trays start loose and have no joints to each other or to the table.

Industrial tray denesting supplies individual containers to packaging stations. For a documented example of manual denesting and its automation, see [fruit packaging tray denesting](https://patents.google.com/patent/US5876180A/en). The reusable printed trays in this scene reproduce the hold-and-release problem with two visible lips. This particular 8 mm lip design is a prototype.

## Setup and reset

Use the calibrated bimanual YAM station. Positions are in metres from the midpoint between the arm bases, with x forward and y toward the robot's left. The table top is at z = 0.7475 m.

| Object | Material and dimensions | Fixed start |
| --- | --- | --- |
| Blue lower tray | PETG, 220 × 150 × 45 mm shell with 3 mm walls and floor. Two 4 × 9 × 5 mm inward yellow ledges. A 24 × 15 × 47 mm post on the right wall supports a 24 × 74 × 19 mm outward grip. Its centerline is 60 mm toward the near end of the tray. Simulated mass 240 g. | Upright at x = 0.265 m, y = 0, bottom on table. |
| Orange upper tray | PETG, 180 × 120 × 35 mm shell with 3 mm walls and floor. Two 4 × 11 × 4 mm tabs extend outside the long walls. Its 24 × 11 × 42 mm grip rises off the left wall and carries a 24 × 51 × 14 mm inward grip extension. Simulated mass 170 g. | Upright on the blue floor with the same x/y center and two tabs under the corresponding ledges. |
| Two green staging pads | 3 mm flat mats, secured. Left 254 × 203 mm, right 284 × 273 mm. | Left (0.265, 0.211), right (0.265, −0.246). |

Print the trays from `trays.scad` by choosing `part="outer"` or `part="inner"`. Print with supports under the overhanging ledges and tabs. Center marks on the long rims can be applied in yellow paint. For reset, place the lower tray on its center mark, lower the upper tray with its tabs alongside the ledges and shift it until the marks coincide. Keep both pads clear. See `reset.json` for all collider bounds and canonical poses. No consumables or object jitter.

## Run and score a trial

With the Robocurve Isaac Lab-Arena environment active, run the following in this directory. The example uses the full 1,200-step policy budget.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Separate the nested trays and place the orange tray upright on the left pad and the blue tray upright on the right pad." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir /tmp/tray-demo
```

The scripted demo parks the right arm at its ready pose for the first five seconds while the left arm grips the orange tray. The right arm then grips the outward handle to counterhold the blue tray while the left arm slides and lifts the orange tray. Set `TRAY_ARM_CONTACTS` to a fresh JSON Lines output path to record cross-arm contact pairs at each 240 Hz physics step, including the link names and contact separation. Set `TRAY_DIAGNOSTICS` to a fresh output path for the per-control-step poses, gaps, partial-credit predicates, dwell, score, and contact totals.

The local reference runs used Isaac Sim 6.1, Isaac Lab Arena release/0.3.1, Inspect Robots 1.0.0, and an RTX 5090. Both seed-zero resets reached 1.00 in 578 control steps (28.9 simulated seconds) with zero cross-arm contacts. Recording both took 200 seconds of wall time including simulator startup. Use `--policy dream` for the model trial with the same scene, instruction, and limit. To record a shortcut trial, run the scripted command with `TRAY_DEMO=straight`, `TRAY_DEMO=together`, or `TRAY_DEMO=one_pad`. Run `python -m unittest test_scoring.py` for the offline checks and `python validate.py results.json` for the same-process repeated-reset and shortcut controls.

The score is read at the end. A tray more than 50 mm below the bench or a tab penetrating a ledge more than 2.5 mm on all box separating axes invalidates the trial. Ordinary contact within the solver tolerance is allowed. Otherwise, use the highest qualifying score:

- **1.00**. Both trays are upright within 10 degrees, entirely over their assigned pads, and supported at pad height within 2 mm. The enclosing tray geometry is at least 30 mm apart. The jaws are at least 10 mm clear, with no motion beyond 2 mm or rotation beyond 2 degrees over 20 consecutive control steps.
- **0.75**. The trays are separate and both are upright within 10 degrees.
- **0.50**. The enclosing tray shapes have a gap of at least 2 mm, so neither contains, contacts or supports the other.
- **0.25**. Both upper tabs have cleared their ledges horizontally but the trays are still nested.
- **0.00**. All other outcomes. Re-nesting can remove previously earned credit.

The gap is measured between enclosing axis-aligned boxes. It is conservative and can be checked in the top and side views. With the trays resting at the pad height, a photo, ruler, and level provide a physical score witness. The action limit is **1,200 steps (60 simulated seconds at 20 Hz)**.

## Simulation fidelity

The walls, floors, grips, tabs and ledges are individual box colliders grouped into two free hard bodies. A straight lift meets native collision geometry at both lips, with no attachment logic. The station uses PhysX at 240 Hz and contacts with a 0.5 mm collision offset for these tray shapes. The trays use estimated rigid PETG behavior, without flex, wear, or fracture. The grip geometry and masses are prototype specifications. The design is provided here for a physical fit check.

The compound-body helper is adapted from the benchmark repository’s DIN rail task.
