# Fit a table lamp harp and shade

Fit the harp into the lamp, lower both retaining sleeves, and seat the shade on the harp.

## Reference operation

Paxton Hardware's 8-inch removable harp (SKU 1231A) is specified at 203.2 mm from the bottom of the saddle to the top below the 1/4–27 finial thread. Paxton's procedure raises both sleeves, squeezes the legs into the notched saddle channels and slides the sleeves down to secure the legs. The RH Lattice table lamp uses the same procedure and then sets the shade onto the threaded post and secures it with the top nut. This task ends at the documented intermediate step where the seated harp carries the shade and the finial can be installed. [Paxton product and installation instructions](https://paxtonhardware.com/products/antique-lamp-harps-8-inch) · [RH installation instructions, pp. 4 and 6](https://images.restorationhardware.com/content/catalog/product/us/AI_Lattice_TableLamp_en-US.pdf) · [Paxton shade and finial fit](https://paxtonhardware.com/pages/lamp-harps-finials)

The table lamp is unpowered and starts with its bulb socket empty. RH fits the harp and lowers both sleeves before its bulb installation step. [RH installation sequence, p. 4](https://images.restorationhardware.com/content/catalog/product/us/AI_Lattice_TableLamp_en-US.pdf)

The model uses a compatible compact host with a 142 mm diameter base and inferred shade dimensions. The shade has a three-spoke spider and a 7/16-inch center washer. The local scripted arms grasp one original spoke clear of the center washer. [Paxton spider shade description](https://paxtonhardware.com/blogs/news/choosing-harp-lamp-shades)

The two sleeves are free bodies. Each sleeve's upper opening bears against the wire and its skirt blocks the leg from moving inward far enough to clear the retaining shoulder. The documented lamp-harp design uses this wire and saddle contact. [Lamp harp engineering contact description, US2802098A](https://patents.google.com/patent/US2802098A/en)

The original harp post keeps its segmented collision surface at the published 0.941 mm pitch. [Paxton thread specification](https://paxtonhardware.com/products/antique-lamp-harps-8-inch)

## Fidelity and reset

The wire form uses two passive torsional springs with straight load-bearing legs. The thread seat stands 203.2 mm above the bottom of the winged saddle. The joint angles move under gripper and saddle contact. The 3.5 mm leg section, 84 mm leg span, 19 mm sleeve diameter, contact clearances and effective 6.5 N·m/rad hinge rates are estimates. The square leg sections, beveled cheek and cup mouths, and faceted thread flanks approximate the metal parts. The wire and sleeve friction settings are 0.66 and 0.82 respectively. The actual fit, contact friction and wire compliance await a measured physical sample. An independent bench rest keeps the uninstalled harp upright for gripper access during reset. The simulated arms raise each sleeve about 46 mm and hold its bore against the wire to carry the harp by contact. This robot grasp departs from the manual grip shown in the instructions.

The lamp starts with an empty saddle. The harp stands on the separate bench rest with both sleeves down, and the shade starts upright on the tabletop. The loose finial stays on the bench for the following assembly step. Full credit requires both harp feet to stay under their saddle shoulders, both sleeves to cover those shoulders, and the shade washer to stay seated for one simulated second with both grippers clear. The scene ends with shade placement and release, ready for the finial.

## Local verification

Run `python -m unittest test_geometry test_scoring test_expert` from this folder for the saddle clearances, sleeve bore and retention geometry, the original post geometry and scoring tests.

Activate the YAM simulator environment from the RoboCurve walkthrough, then run from the root of this repository. The optional `HARP_EXPERT=1` setting reads settled part poses before the shade approach and measures the in-hand shade offset before seating the washer. It retargets future waypoints at those phase boundaries and sends the standard stock joint actions throughout the run.

```sh
task_dir=tasks/fit-lamp-harp-and-shade
HARP_EXPERT=1 HARP_DETAIL=overview \
python "$task_dir/capture_detail.py" run \
  --embodiment yam_arena --policy yam_scripted \
  -E task="$task_dir/task.py" \
  -E object_jitter=0 -E control_hz=20 \
  -E cam_width=480 -E cam_height=360 --seed 0 \
  --instruction "Fit the harp into the lamp, lower both retaining sleeves, and seat the shade on the harp." \
  --scorer task_score --max-steps 950 --store-frames --no-prompt \
  --log-dir /tmp/a1-shade-demo
```

The task allows up to 2,400 actions. `HARP_PHASE=harp` stops after the two sleeves, and `HARP_PHASE=retention` adds the small inward and upward handling movement before release. The controls `HARP_CONTROL=one_foot` and `HARP_CONTROL=sleeves_up` attempt the same handling movement after an offset right-foot approach or with the sleeves raised. `capture_detail.py` runs the same CLI with a side camera. Select `HARP_DETAIL=overview`, `saddle`, or `thread` to frame different parts of the assembly.

### Score details

A foot must sit under its own saddle shoulder with at least 1.2 mm of overlap. The vertical gap from the saddle floor may range from -0.8 mm to +1.8 mm. Each sleeve covers at least 2 mm of the band where its wall blocks withdrawal. The shade's center washer must be within 2.1 mm laterally and 1.4 mm vertically of its seat, with less than 3.62° of relative tilt. For full credit the assembly must hold those conditions for one second with both gripper boxes at least 8 mm clear and with the harp and shade moving below 4 mm/s and 0.12 rad/s. The base may move by up to 6 mm and tilt less than 3.62°.

The revised shade-seating demo reached 1.0 in 841 actions at seed 0 and took 229.5 seconds of wall time. With both grippers clear, the harp supported the shade with 0.650–0.670 N of measured contact and both sleeves blocked roughly 4.0 mm of the retaining band. There was no recorded inter-arm contact or arm-to-base contact.
