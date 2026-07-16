# Decision: motion.txt removed — startup mode moved to Config.h

`motion.txt` was a runtime-discovered text file (searched in cwd, the
executable's dir, and its parent) whose presence and `mode:` line selected
`main()`'s behavior: absent -> recording (read-only); `mode: reactive` ->
task-space servo to the `Config.h` target pose; `mode: joints` (or no
`mode:` line) -> a one-shot relative joint move using the file's
`deltas_deg`/`speeds_deg_s` lines. It never held a trajectory — one delta
set per file, not a waypoint sequence.

Removed 2026-07-16 at Christian's request, to make reactive control the
default with no file required.

- The mode switch is now `config::kStartupMode` (`src/Config.h`), an enum:
  `kReactive` (default), `kJoints`, `kRecord`. Set it and rebuild.
- The joints-mode move's parameters moved to `config::kJointDeltasDeg` /
  `config::kJointSpeedsDegS` (`src/Config.h`).
- `load_motion_config`, `find_motion_config`, and `MotionConfig` (all in
  `src/Motion.{h,cpp}`) were deleted — they only existed to parse the file.
  `move_joints_relative`, `JointVector`, `kDefaultSpeedLimits`, and the rest
  of `Motion.{h,cpp}`'s low-level move machinery are unchanged.
- The old runtime speed check (joints-mode speed vs. `kDefaultSpeedLimits`)
  is now a `static_assert` in `Config.h`, since the input is a compile-time
  constant instead of an external file — an unsafe speed is now a build
  error, not a load-time exception.

Safety consequence (see `../../AGENTS.md`): because `kReactive` is the
default, a fresh checkout, build, and run moves the arm immediately — there
is no longer a "no file present" safe path. `./controller` can no longer be
run as an incidental "does it build/start" test.
