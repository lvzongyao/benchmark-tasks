# Check a front brake pad’s remaining lining

Check the brake pad lining with both gauges, then return them to their rests.

## Reference operation

Toyota Tech Tip T-TT-0132-11 specifies checking the friction lining with a ruler or pad thickness gauge before judging replacement against the vehicle-specific service limit. Worn pads can damage the disc. The bench task reports a thickness interval for one removed specimen. [Toyota Tech Tip, p. 1](https://static.nhtsa.gov/odi/tsbs/2014/SB-10060762-2280.pdf)

Brembo lists the P83117N front pad at 158 × 57 mm and the 09.A417.11 front disc at 296 × 28 mm for the Camry ACV40. A lining-down bench reset uses the disc's working face as the reference plane. The steel backing overlaps that plane at the exposed abutment ear. The surfaces are separated by the remaining lining, so backing thickness is excluded. The approach uses the straight-on L-tip measurement documented for the Lisle 81850 set. Use its 8 mm tool 81920 and its 10 mm tool 81930. [Brembo fitment](https://www.bremboparts.com/america/en/catalogue/toyota-camry-saloon-v4-2-4-acv40/000020332-1) · [Lisle 81850 stock gauges](https://www.lislecorp.com/specialty-tools/brakes/combination-brake-lining-thickness-gauge-set) · [Lisle patent, figs. 5A and 8](https://patents.google.com/patent/US9500462B2/en)

## Fixed reset

The tabletop is at z = 0.7475 m. The rotor lies flat, centered at (0.188, 0.045) m. The pad rests lining-down on its annular working face, with its long axis along y. Its origin is (0.291, 0.046, 0.7965) m. The 8 mm and 10 mm gauge tip centers start at (0.328, −0.145, 0.7655) and (0.215, −0.145, 0.7655) m. Both handles sit edge-up in separate slotted rests, noses facing +y. Replace the same pad and tools to repeat the check.

The reconstructed service specimen has 8.4 mm of friction lining. The pad uses the published plan envelope with an estimated backing-ear shape and lining profile. Both green gauges use the published dimensions at their measuring tips. Their approximately 117 mm offset handles and 4 mm molded body thickness are reconstructed from stock photographs. The model includes the lanyard eye and non-colliding molded markings. The annular disc contact and lining-to-backing distance are physical surfaces. No working caliper, wheel, hydraulic parts, dust, heat or braking behavior is modeled. Actual pad wear, body dimensions and acetal flex have not been measured.

## Scoring and physical witness

The 8 mm gauge must insert beside the pad lining and demonstrate actual contact first with the disc and then with the underside of the backing at that same opening. The 10 mm gauge must rest against the disc, contact the front of the backing ear and stop at its edge. Both gauges are withdrawn and parked. A bench operator can reproduce the check with a low oblique view across the rotor face and a side view of the backing ear. The 8 mm tip disappears under the ear and has small vertical play. The 10 mm tip leaves its upper edge exposed against the front of the ear while its bottom contacts the disc. The comparison gives a lining thickness from 8 mm inclusive to below 10 mm. This interval is the fixture's measurement result.

The score is 0.25 after contact with the disc at the 8 mm insertion position, 0.50 for disc and underside contact with that tool, 0.75 after the larger tool produces a genuine stop, and 1.0 after both tools stay parked and the hands clear for 20 consecutive controls. At 8 mm, the measuring head's leading edge extends 7–21 mm past the front edge of the ear. At 10 mm, the leading edge stays within 2 mm of the front edge while both physical contacts hold. Each witness latches after four consecutive contact samples with the tool within 5° of the approach axis. The lining plane at the ear stays within 0.65 mm of the disc face and within 2° of parallel. The recorded measurement survives normal withdrawal. Ordinary pad repositioning is allowed when the lining rests on the disc again for the reading. Before a physical trial, measure the actual lining on a flat section with a ruler or depth gauge from the friction face to the exposed backing and record that reference independently.

## Local validation

Run from the task directory in an activated local YAM simulator environment. This workspace's activation script is shown below. The command generates its own capture from the source checkout.

```sh
source ../../../robocurve-sim/activate.sh
mkdir -p .local
OMNI_CRASHREPORTER_ENABLED=0 OMNI_CRASHREPORTER_SKIPOLDDUMPUPLOAD=1 \
GAUGE_DIAGNOSTICS=.local/lining-trace.jsonl \
  flock -F /tmp/workcellbench-sim.lock python capture_detail.py run \
  --embodiment yam_arena --policy yam_scripted -E task=task.py \
  -E control_hz=20 -E object_jitter=0 -E cam_width=320 -E cam_height=176 \
  --instruction 'Check the brake pad lining with both gauges, then return them to their rests.' \
  --scorer task_score --seed 0 --max-steps 1800 --store-frames --no-prompt --log-dir .local/lining
python -m unittest test_scoring
```

Use `GAUGE_CONTROL_GAP=narrow` for a 6.6 mm lining and `GAUGE_CONTROL_GAP=wide` for an 11.4 mm lining. The same two tools and scripted actions are used in both controls. These specimens are local scorer validation cases.

The verified nominal trial scored 1.0 at 767 controls with CLI seed 0 and episode seed 1415158438. It took 38.35 simulated seconds and 170.95 seconds of wall time. The 8 mm head made 0.246 N of rotor contact at 9.36 mm insertion, then 0.438 N of underside contact at 12.03 mm insertion. At the 10 mm stop, the head made 0.164 N of rotor contact and 0.500 N against the front of the backing. I checked pickup, both physical measurements, withdrawal, release and retreat in the encoded overhead and detail streams. Both tools finished supported in their rests.

In the matched 6.6 mm trial, the 8 mm head hit the backing front at up to 2.69 N and advanced at most 0.034 mm past the entrance while stopped. Both tools returned to their rests. The run scored 0.0.

The 11.4 mm control scored 0.25 at 848 controls. The 8 mm tool touched the disc without reaching the backing's underside. The 10 mm tool's leading edge traveled 1.74 mm past the front edge at the same attempted stop, with zero frontal contact. Both tools finished parked. I checked both control recordings from reset through retreat, including the side view at the backing edge.

The local project archive holds `review/setup.png`, `review/demo.mp4`, `review/detail.mp4`, `review/verification.json`, `review/narrow.mp4`, `review/wide.mp4` and `review/controls.json`.
