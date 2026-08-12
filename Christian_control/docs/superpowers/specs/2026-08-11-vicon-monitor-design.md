# Read-only Vicon monitor

## Goal

Add one graphical, read-only Vicon monitor to the existing HumanSL browser
panel. It must make the difference between labelled markers and usable segment
poses explicit, while remaining completely outside the Kinova command path.

The current live baseline is known: DataStream at `192.168.128.206:801`
delivers 100 Hz frames and 20 labelled markers for subject `Dr Octopus
Christian`, but all five expected segment poses are reported occluded. The
monitor must show that state as not ready rather than treating zero-valued
poses as valid.

## Approved architecture

The existing local browser panel remains the only graphical application. The
panel gains a `VICON` tab and a read-only `GET /api/vicon` endpoint.

A new sensing-only C++ executable, `vicon_monitor`, is built in the existing
standalone `vicon` CMake project. It reuses `ViconInterface` for all DataStream
access. It has no Kortex, Kinova, controller, actuator, or session-start
dependency.

The monitor process is a dedicated data boundary:

```text
Vicon DataStream
    |
    vicon_monitor acquisition boundary
    |  immutable validated snapshot, atomic publication
    vicon snapshot file
    |
    existing panel server GET /api/vicon
    |
    VICON tab (read-only rendering)
```

The acquisition boundary may block waiting for Vicon frames, but that work is
not performed by the UI and is not part of either Kinova control loop. The
panel reads only the most recent complete snapshot. Snapshot publication uses
a temporary file followed by an atomic rename so the UI never parses a
partially written sample.

The panel may start and stop this sensing-only helper with its own lifecycle.
This is not a second robot-control process: the helper cannot issue arm
commands because it does not link against or call any Kinova/control code.

## Configuration and data contract

`MonitorConfig` is an explicit value containing:

- endpoint, default `192.168.128.206:801`;
- subject, default `Dr Octopus Christian`;
- stale threshold, default `100 ms`;
- expected segments: `Mount`, `LeftBase`, `RightBase`, `LeftEE`, `RightEE`;
- expected markers: the 20 names supplied in the project requirements.

The endpoint, subject, and stale threshold are command-line/configuration
values and are included in every published snapshot. Expected names are
stored in the explicit configuration object and are included in diagnostics;
they are not inferred from whichever subjects happen to arrive.

The validated sample stores:

- connection state and diagnostic reason;
- Vicon frame number and server frame rate;
- local receive timestamp;
- received-frame rate and cumulative skipped-frame count;
- stale threshold;
- expected marker diagnostics, with positions in metres;
- expected segment diagnostics, with position in metres;
- raw Vicon quaternion in `(x, y, z, w)` order;
- derived roll, pitch, yaw in degrees;
- visibility, validity, freshness, and an explicit reason.

Vicon translations are converted from millimetres to metres exactly once while
building the validated sample. Occluded, missing, non-finite, stale, or
quaternion-invalid poses publish `null` numeric pose values and a reason; they
never publish zero as a usable pose.

A quaternion is accepted only when all four components are finite and its norm
is within the validation tolerance of one. It is not silently normalised.

Frame gaps are counted from advancing Vicon frame numbers: a transition from
`N` to `N+k` adds `k-1` skipped frames. Repeated or non-advancing frames do not
pretend that a new sample arrived.

## Readiness policy

The monitor derives exactly three top-level states:

- `NOT READY: no connection`: no current DataStream connection or no live
  frame source;
- `NOT READY: segments unavailable`: connected, but at least one required
  segment is missing, occluded, invalid, or older than 100 ms, or the frame is
  not advancing;
- `READY FOR POSE CALIBRATION`: all five required segments are valid and
  fresh, the frame is advancing, and the sample is connected.

Marker labels are diagnostic evidence only. They do not make the monitor ready
when segment poses are unavailable. Missing expected markers are shown in the
marker diagnostics and can explain an invalid segment, but the readiness gate
is explicitly based on the five world-frame segment poses.

## Graphical display

The `VICON` tab contains:

1. A persistent monitor header showing connection, endpoint, server rate,
   latest frame, received rate, latest-frame age, skipped frames, stale
   threshold, and the top-level readiness state.
2. A five-row segment table with tracking, world position in metres, RPY in
   degrees, sample age, colour/status, and reason. Green means valid and fresh;
   amber means connected but stale/incomplete; red means occluded, missing,
   invalid, or disconnected.
3. Expandable marker diagnostics grouped by Mount, LeftBase, RightBase,
   LeftEE, and RightEE, including labelled/unlabelled totals, each expected
   marker position or `missing`, and group counts.
4. Expandable frame/data-quality diagnostics containing the local receive
   timestamp, frame age/gaps, frame rate, valid/invalid segment counts, the
   Vicon-world-frame statement, and the definition of `T_world_cluster`.
5. A dependency-free canvas visualisation with Vicon world axes and labelled
   coordinate triads for valid segments. Invalid segments are omitted from the
   geometry and shown only as unknown/red status; none is drawn at the origin.

The page carries an explicit `READ-ONLY · NO ARM COMMANDS` indicator. It adds
no Vicon write controls and no controller integration.

## Replay and deterministic demonstration

`vicon_monitor` supports these replay modes:

- `disconnected`;
- `occluded`: connected, 20 labelled markers, all five segments occluded;
- `valid`: all five segments valid and moving with finite, normalised
  quaternions;
- `stale`: one required base segment becomes stale or occluded;
- `all`: cycles through the four states in a deterministic loop.

The panel launcher exposes the replay selection without involving Vicon or
Kinova. Replay snapshots use the same validation and rendering path as live
snapshots, so the tests and visual behaviour exercise the real diagnostic
contract.

## Error handling and lifecycle

Live acquisition retries connection after a failed attempt and publishes a
disconnected snapshot rather than terminating the UI or flooding the terminal.
The monitor uses quiet diagnostics for normal polling; detailed reasons are
carried in the snapshot and displayed in the page.

The panel treats a missing, malformed, or stale snapshot as disconnected/not
ready. It never retains a valid-looking pose past the freshness policy.

Shutdown stops the sensing-only helper. No Vicon configuration, marker label,
subject, segment, or Nexus setting is modified.

## Verification

Hardware-free C++ tests cover:

- occlusion rejection;
- stale-data rejection;
- millimetre-to-metre conversion;
- non-finite and non-normalised quaternion rejection;
- frame-gap counting;
- replay state construction.

Panel tests cover the new read-only route, static tab/view wiring, absence of
Vicon write actions, and rendering data contracts. The normal Vicon CMake
build and CTest suite are run. Replay mode is launched and checked for all
four states. A live sensing-only run may be attempted; it will report the
observed condition accurately and will not be treated as proof of robot
control readiness.

The final handoff will explicitly state that the call path ends at
`ViconInterface` read methods and never reaches Kinova/controller/session
command functions.
