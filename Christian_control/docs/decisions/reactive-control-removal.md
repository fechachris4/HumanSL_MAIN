# Decision: reactive/Cartesian control removed — joint moves only

`src/Controller.h/cpp` implemented a task-space (Cartesian) velocity-resolved
servo: PD on pose+twist error → damped-least-squares inverse kinematics →
joint velocities, integrated client-side into position setpoints and
streamed at 1 kHz. `config::kStartupMode` had a third value, `kReactive`,
that ran it continuously toward a fixed target pose in `Config.h` until
Ctrl+C, and was the default.

Removed 2026-07-17 at Christian's request, after this path repeatedly
faulted the arm during testing — fault history and root-cause evidence at
the end of this document.

- `src/Controller.h` and `src/Controller.cpp` are deleted.
- `config::StartupMode` is now `{ kJoints, kRecord }` — `kReactive` no
  longer exists. `kJoints` is the default.
- Removed from `src/Config.h`: `kTargetPosition`, `kTargetOrientationRPYDeg`,
  `kKpPos`/`kKpRot`/`kKdPos`/`kKdRot`, `kDlsDamping`,
  `kUseNullspaceCentering`/`kNullGain`, `kReactiveSpeedLimitDegS`,
  `kCtrlLeadRad`, `kControlLogPrefix` — all were reactive-only.
- Removed from `src/Record.{h,cpp}`: `ControlTraceSample` and
  `write_control_trace_header`/`write_control_trace_row` (the reactive
  per-cycle trace CSV) — unused once `run_reactive_control` is gone.
- `main.cpp` no longer calls `report_position_error` (target-vs-FK report,
  lived in `Controller.cpp`) or branches on a reactive mode. It still runs
  `report_fk_vs_robot` (`Kinematics.cpp`) at startup — that check is
  general-purpose and unrelated to any control law.
- `CMakeLists.txt` no longer builds `src/Controller.cpp`.

`src/Kinematics.{h,cpp}` is unchanged: `forward_kinematics` and
`report_fk_vs_robot` are read-only diagnostics with no coupling to the
removed control law, so nothing there was reactive-only.

The only way to move the arm is now `move_joints_relative`
(`src/Motion.cpp`, `kJoints` mode): one relative move, all 7 joints, degrees,
per run — see `README.md`'s "Joint moves" section and `motion-txt-removal.md`
for how that path itself works and evolved. If task-space control is wanted
again later, this history and the fault analysis below are the starting
point — not a reason to avoid it forever, but the saturation and
stale-feedback issues found here need fixing first.

## Fault history (moved here from known-issues.md when it was retired, 2026-07-22)

- A large initial pose error (>1 m/149°, because the configured target was
  far from the arm's actual position) saturated 6-7 joints at the speed
  clamp on the very first cycle, in a direction that was no longer the
  least-squares solution (per-joint clamping does not preserve the DLS
  direction).
- One run drove joint 4 across what is very likely a *configured* position
  soft limit near −19.6° (raw 340.43°) — far inside the Gen3's ±147.8°
  factory range, so arm-specific, not a code defect. Still open: check the
  arm's web dashboard (`http://192.168.1.10`) for that actuator's active
  safety state and configured position limits — the arm's own firmware
  enforces the limit regardless of which controller drives it. (Joint 6
  later faulted the same way near +36° — see
  `resolved-rate-position-integration.md`.)
- Once parked past that point, every subsequent run faulted on the very
  first cyclic exchange, before any motion, with only the base's aggregate
  `JOINT_FAULT` bit visible.
- More generally, this control path used low-level velocity-mode streaming,
  which Kinova's own issue trackers document real problems with (kortex
  issues #42, #93, #156) — the evidence that later killed VELOCITY-mode
  actuation for good (`cartesian-velocity-controller.md`).
