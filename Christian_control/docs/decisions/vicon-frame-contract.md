# Vicon frame contract — Stage 0 lab record

Status: template — fill in during the lab session, then commit.

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

Run from `Christian_control/vicon/build/` (subject name as seen streaming
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
