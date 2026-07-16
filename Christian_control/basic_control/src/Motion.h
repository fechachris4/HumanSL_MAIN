//
// Motion: low-level cyclic motion through BaseCyclic — all 7 joints at once.
// MOVES THE ARM — see Safety in README.md before use.
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_MOTION_H
#define HUMANSL_MASTERS_PROJECT_2025_MOTION_H

#include <array>
#include <atomic>
#include <chrono>
#include <iosfwd>
#include <string>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>

namespace k_api = Kinova::Api;

// One value per joint, in Kortex actuator order: index 0 = joint 1 ... index 6
// = joint 7 (same ordering as feedback.actuators(i) / command.actuators(i)).
// Fixed size, so "exactly 7 values" is enforced by the type at compile time.
using JointVector = std::array<double, 7>;

// RAII guard: LOW_LEVEL_SERVOING on construction, SINGLE_LEVEL_SERVOING on
// destruction — so the robot's supervisor is restored on every exit path
// (normal return, Ctrl+C freeze, or exception unwinding). Shared by every
// low-level loop (joint moves here and both controller folders).
class ServoingModeGuard
{
public:
    explicit ServoingModeGuard(k_api::Base::BaseClient* base);
    ~ServoingModeGuard();
private:
    k_api::Base::BaseClient* base_;
};

// One cyclic exchange: write `angles` (degrees, any winding — wrapped to
// [0, 360) on the way out) into the frame, stamp the ids the base uses to
// reject stale packets, send, and return the same cycle's feedback.
k_api::BaseCyclic::Feedback send_positions(
    k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
    k_api::BaseCyclic::Command& command,
    const JointVector& angles);

// Moves all 7 joints simultaneously by deltas_deg (relative, degrees;
// 0 = hold current setpoint), each joint capped at its own speed from
// speeds_deg_s (degrees/second; use broadcast_speed() for one shared value).
//
// Streams position command frames every `period` (default 1 ms = the Kortex
// low-level 1 kHz cycle) until every joint reaches its target, then holds
// for a 1 s settle window. Checks `stop` (Ctrl+C) every cycle and freezes
// the motion there if it fires. The arm is in LOW_LEVEL_SERVOING for the
// duration; SINGLE_LEVEL_SERVOING is restored via RAII on every exit path,
// including exceptions.
//
// Returns true only if, after the settle window, every MOVING joint's
// MEASURED angle is within kReachedToleranceDeg of its target — sending all
// the commands is not enough (the robot can stop following them silently;
// see docs/known-issues.md). Returns false, with a per-joint
// requested/measured/error report, when:
//   - the measured tracking error (commanded - measured) exceeds
//     kTrackingErrorLimitDeg for kTrackingErrorCycleLimit consecutive
//     cycles (the command is frozen at the measured position first),
//   - the robot reports a fault (base or actuator fault bank),
//   - a Kortex call fails (communication error mid-move),
//   - the final measured position misses the target,
//   - or the user stops the move with Ctrl+C.
//
// If `log` is non-null, one CSV row per cycle (time, dt, commanded/measured/
// error angles, velocities, torques, currents, fault and warning banks —
// see write_move_log_header in Record.h) is written to it.
//
// Throws std::invalid_argument if a joint has a nonzero delta but a
// non-positive speed (it could never arrive).
bool move_joints_relative(k_api::Base::BaseClient* base,
                          k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                          const JointVector& deltas_deg,
                          const JointVector& speeds_deg_s,
                          const std::atomic<bool>& stop,
                          std::ostream* log = nullptr,
                          std::chrono::microseconds period =
                              std::chrono::milliseconds(1));

// Convenience: the same speed for all 7 joints.
JointVector broadcast_speed(double speed_deg_s);

// Client-side completion and monitoring policy for move_joints_relative.
// These check OUR view of the move; they do not configure the robot.

// A moving joint counts as arrived when its measured angle ends within this
// many degrees of its target: well above the ~0.15 deg steady-state tracking
// lag seen at our speeds, far below any real miss.
constexpr double kReachedToleranceDeg = 0.5;

// Abort the move when |commanded - measured| exceeds this for the number of
// consecutive cycles below. The start-up transient reaches ~2 deg, and the
// arm's own safety kicks it out of low-level servoing near 5 deg, so 3 deg
// held for 50 ms sits between the two: no false trips, but we stop the
// stream (and say why) before the robot faults.
constexpr double kTrackingErrorLimitDeg = 3.0;
constexpr int kTrackingErrorCycleLimit = 50;   // cycles, i.e. ~50 ms at 1 kHz

// Per-joint speed limits used to validate motion configs, in deg/s.
//
// OUR arm's base is configured with 50 deg/s joint speed SOFT limits
// (read with ./query_limits; the hardware could do 80/70). Streaming
// position steps at >= the soft limit is not followed: the joint stands
// still, tracking error grows, and at ~5 deg the safety kicks the arm out
// of low-level servoing (seen as WRONG_SERVOING_MODE mid-move). So we
// validate against 45 = 10% under the enforced 50.
// Override per run with a "speed_limits_deg_s:" line in motion.txt —
// only useful if the robot's own soft limits are raised first.
constexpr JointVector kDefaultSpeedLimits =
    {45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0};

// A validated motion request loaded from a config file.
struct MotionConfig
{
    std::string mode = "joints";
    JointVector deltas;   // relative degrees; 0 = hold that joint
    JointVector speeds;   // deg/s, per joint (validated <= limits)
    JointVector limits;   // deg/s, per joint (defaults to kDefaultSpeedLimits)
};

// Loads and validates a motion config from a tiny text file:
//
//   # comment lines and blank lines are ignored
//   mode: joints                        (optional; "joints" default)
//   mode: simple_joint_position_hold    (measured startup pose hold)
//   mode: legacy_advanced               (archived task-space controller)
//   deltas_deg: 0 10 0 -15 0 5 0        (required in joints mode, exactly 7)
//   speeds_deg_s: 0 5 0 5 0 3 0         (optional, exactly 7; wins over default)
//   default_speed_deg_s: 5              (optional, one value for all joints)
//   speed_limits_deg_s: 79 79 79 79 69 69 69   (optional, exactly 7; replaces
//                                               kDefaultSpeedLimits)
//
// If neither speed line is present, 5 deg/s is used for all joints.
// Any speed above its joint's limit is rejected at load time.
// Throws std::invalid_argument with a message naming the problem line/field.
MotionConfig load_motion_config(const std::string& path);

// Looks for motion.txt in, in order: the current directory, the directory
// containing the executable, and that directory's parent (so a file kept in
// basic_control/ is found when running from build/). Returns the absolute
// path of the first hit, or "" if none exists.
std::string find_motion_config();

#endif //HUMANSL_MASTERS_PROJECT_2025_MOTION_H
