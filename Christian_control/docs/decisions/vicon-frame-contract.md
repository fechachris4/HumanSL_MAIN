# Vicon frame contract — Stage 0 lab record

Status: template — fill in during the lab session, then commit.

## Streaming restored

- [ ] Nexus/Tracker is running and publishing (port 801 reachable from
      this machine — Windows firewall blocked it on 2026-08-10).
- [ ] `./build/connect_vicon <host:port>` exits 0 and reports 10 frames.
- Host used: _____________ (`.206` is the code default; `.210` was
  observed live on 2026-08-10 — record whichever actually answers).

## Segment verification

- [ ] All five expected segments (`Mount`, `LeftBase`, `RightBase`,
      `LeftEE`, `RightEE`) appear in `connect_vicon`'s segment listing,
      `occluded = false`, and move plausibly when nudged.
- Segments actually observed: _____________
- Any segment still occluded/absent: _____________ (if any, Stage 2 needs
  a fallback for that segment — note it here, do not proceed silently)

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

Run from `Christian_control/vicon/build/`:

```
./record_vicon <host:port> static 15
./record_vicon <host:port> wearer_moving 20
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
