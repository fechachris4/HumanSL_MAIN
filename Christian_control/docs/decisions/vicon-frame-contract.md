# Vicon frame contract — Stage 0 lab record

Status (2026-08-13 ~16:40): **captures superseded — live stream only, by
Christian's decision** back from the lab. The two recordings below were
never made and will not be; stage 2's calibration will sample the live
stream or a stage 1.5 controller log instead. Live verification stands:
five valid segments confirmed streaming from this machine twice today
(see pre-check section). Still open from this record: axis convention
(read it off Nexus), segment template origin notes, the `Mount22` marker
name, and the Torso segment decision — these need a person at the Nexus
GUI, no recording required.

## Pre-check from the office, 2026-08-13 (evidence, not the lab session)

Run from this machine at ~15:50 BST with `connect_vicon` (read-only):

- `192.168.128.206:801` answered; exit 0, 10 frames received. `.210`
  pings but has port 801 closed — it is not the DataStream host.
- Server frame rate: 100 Hz.
- Subject seen streaming: **`Christian Test`** — note `record_vicon`'s
  default subject is `"Dr Octopus Christian"`, so pass the subject
  explicitly (it is provenance metadata in the file header; data rows
  are unaffected either way).
- All five segments present under that subject with finite quaternions:
  `Mount`, `LeftBase`, `RightBase`, `LeftEE`, `RightEE`.
- 22 labeled markers, 0 unlabeled: Mount1/2/3 + **Mount22** (odd name —
  check the template in Nexus: mislabel for Mount4?), LeftBase1–4,
  RightBase1–4, LeftEE1–5, RightEE1–5.
- Marker heights ~1.15–1.32 m for Mount/Base clusters are consistent
  with Z-up, but this is inference — the axis convention box below
  still needs Nexus's own setting read off the GUI.

The lab boxes below stay unchecked: they require a person at the rig
(nudging segments, wearing the backpack, Nexus GUI access).

## Slice-1 live validation, 2026-08-13 18:20 (authorized run, left arm, hold only)

Run: `controller --arm left`, 130 s, log
`runs/2026-08-13/loop_log_left_20260813_182005.csv` (log_format 10,
`vicon_source = sdk`). Arm held its startup pose; no motion commanded.

- **Columns live from row 1**: 65,068 rows, zero rows before the first
  Vicon sample (the source connects during startup, before takeover).
- **Rate/ZOH exactly as designed**: 13,015 distinct samples in 130.1 s
  (100.04 Hz); sequence strictly monotonic; zero-order-hold run length
  median 5, max 6 (100 Hz → 500 Hz).
- **Age**: median 5.5 ms, p95 9.6 ms, p99 9.7 ms, max 9.9 ms — uniform
  over the 10 ms Vicon period, never stale. Min −0.6 ms: age is stamped
  at cycle start while a frame can arrive mid-cycle; documented in
  Hardware.h, deliberately not clamped.
- **Cross-check against an independent client**: a concurrent
  `connect_vicon` read agreed with the logged Mount pose to
  sub-millimetre (−0.0274/−0.4785/1.2657 m both sides) and the
  quaternion element-for-element in the same x,y,z,w order.
- **Validity**: all five segments 100.00% valid, zero invalid spells.
- **Overruns**: 0 in 65,043 cycles — the slot read costs the loop
  nothing measurable.
- Mount position spread over the run: ≤3.7 mm (parked rig; noise floor).

Exit criteria for the hold slice: age histogram **met**; independent
cross-check **met**; occlusion behaviour **NOT yet observed** — every
segment stayed visible for the whole run, so the invalid-path has no
live evidence yet (deliberately occlude the Mount cluster for a few
seconds during the next run); axis convention / template origins still
need the Nexus GUI reads below.

## Streaming restored

- [ ] Nexus/Tracker is running and publishing (port 801 reachable from
      this machine — Windows firewall blocked it on 2026-08-10).
- [ ] `./build/connect_vicon <host:port>` exits 0 and reports 10 frames.
- Host used: `192.168.128.206:801` (settled by the 2026-08-13 pre-check
  above; `.210` was a 2026-08-10 misdiagnosis — spec open question 4 is
  answered).

## Segment verification

- [ ] All five expected segments (`Mount`, `LeftBase`, `RightBase`,
      `LeftEE`, `RightEE`) appear in `connect_vicon`'s segment listing,
      `occluded = false`, and move plausibly when nudged.
- Segments actually observed: _____________ (all five seen streaming in
  the 2026-08-13 pre-check, but "moves plausibly when nudged" needs
  hands at the rig)
- Any segment still occluded/absent: _____________ (if any, Stage 2 needs
  a fallback for that segment — note it here, do not proceed silently)
- Marker `Mount22`: renamed/explained? _____________

## Torso segment (added 2026-08-13, Christian's frame decision)

Christian defined (2026-08-13 ~16:25) the torso `T` in his control math
as a **separate tracked segment**, distinct from the mount plate: targets
are specified relative to the wearer's body (`p_d^T, R_d^T`), while the
control chain runs through the mount. That needs a sixth segment.

- [ ] If marker budget and time allow: add a `Torso` segment (3+ markers
      on the wearer's trunk, away from the backpack plate) to the Nexus
      subject, and capture it in BOTH recordings — the `wearer_moving`
      capture then measures mount-vs-torso flex directly, not just
      mount-plate rigidity.
- [ ] If not possible today: record here that Torso is absent from these
      recordings, so stage 2 knows, and a follow-up capture is needed
      before any torso-relative target work.
- Torso segment present in recordings: yes / no — _____________
- Torso marker placement notes: _____________

## Axis convention

- [ ] Nexus's configured axis mapping recorded here (Vicon defaults to
      Z-up; confirm rather than assume — neither `ViconInterface` project
      has ever called `SetAxisMapping`).
- Convention: _____________

## Segment template notes

For each of the five segments, as built in Nexus:
- `Mount`: origin/axis choice — _____________
- `LeftBase`: — _____________
- `RightBase`: — _____________
- `LeftEE`: — _____________
- `RightEE`: — _____________

## Recordings captured

Run from `Christian_control/tracking/build/` (subject name as seen streaming
on 2026-08-13 — adjust if Nexus renames it):

```
./record_vicon 192.168.128.206:801 static 15 "Christian Test"
./record_vicon 192.168.128.206:801 wearer_moving 20 "Christian Test"
```

- [ ] `static`: arms and wearer both stationary. Written to:
      `runs/<date>/vicon_static/{frames,entities}.csv`
- [ ] `wearer_moving`: arms stationary, wearer leans/twists/breathes/steps
      (the Mount-rigidity experiment — settles whether the `Mount`
      segment markers are on the rigid backpack plate or the wearer's
      body). Written to: `runs/<date>/vicon_wearer_moving/{frames,entities}.csv`

## Acceptance (from the approved design)

- [ ] Five valid segments observed live.
- [ ] Axis convention and segment template notes recorded above.
- [ ] Both recordings captured and confirmed non-empty (`record_vicon`
      reported a nonzero frame count for each).

## Open question this settles

`Christian_control/docs/superpowers/specs/2026-08-08-vicon-integration-design.md`
§4.4 and
`docs/superpowers/specs/2026-08-12-vicon-controller-planner-integration-design.md`
open question 1: are the `Mount` segment markers on the rigid backpack
plate? Stage 2's calibration tool will compute
`^ref T_leftbase` per frame from the `wearer_moving` recording and plot
its translation norm / rotation angle over time — constant to within
noise means rigid (proceed); systematic variation with posture means the
markers are on the body (the calibration approach in Stage 2 needs
revisiting before it is built). Record the outcome here once Stage 2 runs
this recording through that check.
