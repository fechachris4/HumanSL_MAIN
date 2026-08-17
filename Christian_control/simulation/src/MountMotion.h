//
// MountMotion — the analytic scripted Mount trajectory (Plan 03 Task 1):
// the one description of how the wearer's Mount moves during a simulated
// run, returning the pose the plant is given and the twist the ideal
// sensor reports from a SINGLE call, so the two can never disagree.
//
// Why one call returns both: the Mount is a world-welded mocap body, so
// MuJoCo carries no Mount velocity a sensor could read back (Plan 02's
// rework; see simulation/README.md "How the Mount is represented"). The
// twist therefore has to come from the description of the motion, and it
// must be the exact derivative of the pose being prescribed — otherwise a
// run could tell the controller the Mount is moving while the plant holds
// it still, and the world hold would feed forward a base velocity that
// does not exist. Plan 02's independent SimulationConfig::world_V_mount
// was deleted on 2026-08-17 for exactly that reason. Christian's
// reference simulation enforces the same coupling by requiring the torso
// pose and twist callables to be installed together and evaluating both
// at one data.time (msc_project/sim/world.py, configure_torso_driver /
// _refresh_torso).
//
// FRAMES, POINTS AND UNITS
//   W  Vicon world axes; M  the Mount frame.
//   world_T_mount = pose of M expressed in W (contract section 2).
//   Metres, radians, seconds throughout. Axes are given in W and are
//   normalized here; their length carries no meaning.
//   world_V_mount is the twist OF the Mount frame, taken AT the Mount
//   origin and expressed in W axes: linear_m_s = d/dt p_W_M, and
//   angular_rad_s is the spatial angular velocity omega with
//   d/dt R_W_M = [omega]x R_W_M.
//   The rotation is therefore about an axis through the Mount's OWN
//   origin — the Mount does not orbit the world origin, and the lever-arm
//   term omega x (p_W_E - p_W_M) that moves the end effector appears
//   downstream in world_frames::ArmControllerState (Frames.h), not here.
//
// EQUATIONS (the whole of the motion; s is the shared scalar sinusoid)
//   s(t)      = A sin(2 pi f t + phi)              [m or rad]
//   sdot(t)   = A (2 pi f) cos(2 pi f t + phi)     [m/s or rad/s]
//
//   translation, unit axis a:
//     p_W_M(t)  = p0 + a s_t(t)
//     v_W_M(t)  = a sdot_t(t)
//   rotation, unit world axis b (fixed in W):
//     R_W_M(t)  = exp(s_r(t) [b]x) R0              (world-axis PREmultiply)
//     omega(t)  = b sdot_r(t)
//
// The angular velocity is EXACT, not a first-order approximation, because
// the rotation axis is fixed in W: differentiating R(t) = exp(theta(t)[b]x) R0
// gives Rdot = thetadot [b]x R, i.e. omega = thetadot b. That is why the
// motion is written with Eigen::AngleAxisd about a fixed world axis and
// never as Euler angles integrated over time — there is no orientation
// state to drift, and every sample is computable from t alone.
//
// PHASE CONVENTION (the one genuine ambiguity, resolved deliberately)
//   world_T_mount_at_zero is the pose at ZERO DISPLACEMENT, i.e. the
//   anchor the sinusoid swings about. With the default phase 0 it is also
//   the pose at t = 0. With phi != 0 the Mount starts displaced by
//   A sin(phi) from it, so the run's first tick is NOT the anchor pose.
//   The alternative (subtracting A sin(phi) so t = 0 lands on the anchor)
//   was rejected: it changes the formula above and makes the amplitude no
//   longer the peak displacement. The plant is kept continuous instead by
//   seeding it from SampleMountMotion(config, 0, anchor) rather than from
//   the anchor itself (DualSimulationRunner::Start).
//
// DEFAULT AMPLITUDES AND FREQUENCY (feasibility, accepted analysis packet)
//   Worst-case joint-speed demand for a world hold under base motion is
//   bounded by ||q̇|| <= ||V_required|| / sigma_min(J6), with the
//   pessimistic home-configuration sigma_min = 0.152 (accepted packet),
//   against the tightest command clip kQdotLimitDegS = 66.5 deg/s
//   (Config.h). One frequency field is shared by both channels (a
//   deliberate simplification: combined motion is one rigid oscillation,
//   not two beating ones), so the default frequency has to be
//   conservative for the ROTATION channel, whose lever arm dominates:
//     translation 0.05 m @ 0.1 Hz: |v| = 0.031 m/s -> 11.8 deg/s   (18%)
//     rotation    0.1 rad @ 0.1 Hz: |omega| = 0.063 rad/s, worst
//         lever arm 0.739 m (right TCP at home) -> ||V|| = 0.078
//         -> 29.4 deg/s                                            (44%)
//     combined (both, worst case in phase): ||V|| = 0.100
//         -> 37.7 deg/s                                            (57%)
//   All three sit under 60% of the clip on the pessimistic bound, and
//   0.1 Hz is quasi-static against both the 500 Hz control tick and the
//   ~25 rad/s position-servo natural frequency, so the disturbance tests
//   the world hold rather than the plant's actuators.
//
// Evidence class: simulation. These are scripted disturbances, not
// recorded wearer motion.
//

#pragma once

#include <Eigen/Core>

#include "State.h" // CartesianPose, Twist

enum class MountMotionKind { kStatic, kTranslation, kRotation, kCombined };

// Conservative defaults, derived in the feasibility note above.
inline constexpr double kDefaultMountMotionFrequencyHz = 0.1;
inline constexpr double kDefaultMountTranslationAmplitudeM = 0.05;
inline constexpr double kDefaultMountRotationAmplitudeRad = 0.1;

// The complete scripted-motion description. Plain data; SampleMountMotion
// is its only interpreter.
//
// kind selects which channels are active: kStatic uses neither (the
// amplitudes below are then not consulted at all), kTranslation the
// translation channel, kRotation the rotation channel, kCombined both,
// sharing frequency_hz and phase_rad so combined motion is one rigid
// oscillation of the Mount.
//
// A negative amplitude is legal and means the same motion started half a
// period later (A sin(x) = -A sin(x + pi)); it is not rejected, because
// rejecting it would be a guard against a mistake nobody has made.
struct MountMotionConfig {
    MountMotionKind kind = MountMotionKind::kStatic;

    // World axis of the translation and its peak displacement, m.
    Eigen::Vector3d translation_axis_world = Eigen::Vector3d::UnitX();
    double translation_amplitude_m = kDefaultMountTranslationAmplitudeM;

    // World axis of the rotation (fixed in W, through the Mount origin)
    // and its peak angle, rad.
    Eigen::Vector3d rotation_axis_world = Eigen::Vector3d::UnitZ();
    double rotation_amplitude_rad = kDefaultMountRotationAmplitudeRad;

    // Shared by both channels. Hz and rad.
    double frequency_hz = kDefaultMountMotionFrequencyHz;
    double phase_rad = 0.0;
};

// The Mount at one instant: the pose the plant is prescribed and the
// exact time derivative of that pose. Simulator truth (contract section
// 4) — in ideal sensing mode it is copied verbatim into the WorldSample;
// realistic sensing (Plan 03 Task 2) degrades it at the sensor boundary.
struct MountTruth {
    CartesianPose world_T_mount;
    Twist world_V_mount; // d/dt of world_T_mount, at the Mount origin, in W
};

// Throws std::invalid_argument naming the offending field.
//
// Rejected: any non-finite field; a negative frequency; and — for an
// ACTIVE channel carrying a nonzero amplitude — a zero-length axis or a
// zero frequency, both of which describe a displacement the motion cannot
// produce. Fields the selected kind does not use are checked for
// finiteness only, because they describe nothing.
void ValidateMountMotionConfig(const MountMotionConfig& config);

// The scripted Mount truth at time_s, measured from the run's t = 0.
// Pure: same arguments in, bit-identical result out, no state, no clock.
//
// Validates its arguments on every call (ValidateMountMotionConfig plus
// finite time_s and anchor pose) so the function is total and carries no
// precondition a caller could violate silently. The cost is a handful of
// isfinite checks per 2 ms tick.
MountTruth SampleMountMotion(const MountMotionConfig& config, double time_s,
                             const CartesianPose& world_T_mount_at_zero);
