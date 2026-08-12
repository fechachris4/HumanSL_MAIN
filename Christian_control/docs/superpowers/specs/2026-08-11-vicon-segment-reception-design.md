# Vicon segment reception

## Goal

Extend the standalone `vicon` sensing wrapper so callers can read every
Vicon subject segment in the most recently acquired frame. This does not
connect Vicon data to the robot controller.

## Data contract

`SegmentData` represents one segment in Vicon's global frame:

- `subject_name` and `segment_name` identify the Vicon object/segment.
- `x`, `y`, and `z` are the global translation in millimetres.
- `qx`, `qy`, `qz`, and `qw` are the global orientation in the Vicon SDK's
  `(x, y, z, w)` order, with `w` the real component.
- `occluded` means the pose is unavailable for that frame and its numeric
  values must not be used.

Each call to `getSegmentPoses()` reads the frame already obtained by
`getFrame()`; it does not acquire a frame or add timestamps.

## Implementation

After a successful DataStream connection, `ViconInterface::connect()` enables
segment data alongside the existing marker, unlabeled-marker, and device data.
`getSegmentPoses()` enumerates subject and segment names from the current
frame, obtains each segment's global translation and quaternion, and returns
one `SegmentData` entry for each successfully queried segment.

The `connect_vicon` sensing tool prints the segment poses for its ten sampled
frames. Existing marker behaviour is unchanged.

## Failure behaviour

If there is no connection or no current frame, `getSegmentPoses()` returns an
empty vector, matching the existing marker-list API. Segment entries whose
translation or orientation is occluded remain present with `occluded=true`;
callers must reject those poses.

## Verification

Add a Vicon-only hardware-free CTest target that validates the pose data
contract and its occlusion semantics. Build the standalone `connect_vicon`
target. A separate sensing-only smoke test against the live Vicon server will
be attempted only when the server is publishing frames; it is not evidence of
robot-control behaviour.

## Safety and scope

No Kinova, controller, timing, actuation, or control-loop code changes. The
Vicon interface retains Vicon-native millimetres and global-frame convention;
any conversion to metres or mapping into a robot frame remains a future,
explicit boundary operation.
