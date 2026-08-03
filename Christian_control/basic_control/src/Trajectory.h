//
// Trajectory — planned joint paths: the file contract (load, validate,
// sample), the law that follows one, and the reference source that replays
// it. Planner-independent, pure Eigen; never talks to the robot.
//
// THE CONTRACT (trajectory_format = 1). A trajectory file is a CSV:
//
//   # trajectory_format = 1
//   # <key> = <value>            (free metadata; recorded, not interpreted)
//   t_s,q1_deg,...,q7_deg,qd1_degs,...,qd7_degs
//   0,...
//
// What a producer must guarantee and a consumer may rely on:
//   - 7 joints, Kinova actuator order 1-7; one row per sample.
//   - q*_deg in DEGREES, continuous (unwrapped), on the branch nearest the
//     start configuration. Consumers compare against measured [0,360)
//     feedback with wrapped arithmetic, so the absolute branch is free.
//   - qd*_degs in DEGREES/SECOND, consistent with the position column
//     (finite differences must agree — validated).
//   - t_s starts at 0 and advances in EXACTLY uniform steps.
//   - first and last rows at rest (|qd| ~ 0): execution starts and ends in
//     a hold.
//
// Loading throws on any structural violation; ValidateTrajectory then
// checks motion limits and returns readable violations — an empty list is
// the ONLY go-ahead. Both must keep a bad file from reaching a takeover.
//

#pragma once

#include <cmath>
#include <istream>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "Config.h"
#include "State.h"

struct Trajectory {
    double dt_s = 0.0; // uniform sample spacing, from the t_s column
    std::vector<Eigen::Matrix<double, 7, 1>> pos_deg;
    std::vector<Eigen::Matrix<double, 7, 1>> vel_deg_s;
    std::map<std::string, std::string> metadata; // '#' preamble key = value
};

// Parse a trajectory_format = 1 stream/file. Throws std::runtime_error with
// a one-line reason on the first structural violation.
Trajectory LoadTrajectoryCsv(std::istream& in, const std::string& name);
Trajectory LoadTrajectoryCsv(const std::string& path);

// What ValidateTrajectory measured — the numbers an operator must see
// before authorizing motion.
struct TrajectorySummary {
    double duration_s = 0.0;
    std::size_t samples = 0;
    JointVector displacement_deg{};   // last row - first row, per joint
    JointVector peak_vel_deg_s{};     // max |qd| per joint
    JointVector peak_accel_deg_s2{};  // max |dqd/dt| per joint, finite diff
    double start_vel_deg_s = 0.0;     // max |qd| across joints, first row
    double end_vel_deg_s = 0.0;       // max |qd| across joints, last row
};

// Check a loaded trajectory against motion limits. Returns readable
// violations (empty = pass) and fills `summary` either way.
//   vel_limit_deg_s    per-joint |qd| ceiling — pass the command clip times
//                      a margin, so the Runner's clamp can never engage on
//                      a validated file
//   accel_limit_deg_s2 per-joint |dqd/dt| ceiling
//   pos_limit_deg      per-joint ceiling on WRAPPED |q|; 0 disables it for
//                      that joint (the continuous joints 1/3/5/7)
// Also enforced: velocity-vs-position consistency, rest at both ends (qd
// column AND the motion the positions imply), and per-sample steps small
// enough for the position servo. NOTE: this checks the URDF/Kinova model
// ranges only — the arm's CONFIGURED soft limits can sit far inside them;
// confirm them in the Kinova web dashboard before a session.
std::vector<std::string> ValidateTrajectory(const Trajectory& trajectory,
                                            const JointVector& vel_limit_deg_s,
                                            const JointVector& accel_limit_deg_s2,
                                            const JointVector& pos_limit_deg,
                                            TrajectorySummary& summary);

// Reference position at t_s, linearly interpolated and clamped to
// [0, duration]. Pure, allocation-free — safe in the loop.
Eigen::Matrix<double, 7, 1> SamplePositionDeg(const Trajectory& trajectory,
                                              double t_s);

double TrajectoryDurationS(const Trajectory& trajectory);

// ---------------------------------------------------------------
// following a joint reference
// ---------------------------------------------------------------

// The joint-space law: feed-forward from the reference pair plus a small
// proportional correction —
//
//   q̇_d = (q_ref(t+dt) − q_ref(t)) / dt  +  kp · wrap(q_ref(t) − q_meas)
//
// The feed-forward term is the exact reference difference over the measured
// cycle window, so the integrator telescopes to
// q_cmd = q_meas(0) + (q_ref(t) − q_ref(0)) with zero discretization drift;
// the kp term absorbs the (gated, small) start offset and clamp losses.
// Comparisons are wrapped, so a file's continuous angles and the arm's
// [0,360) feedback never produce a phantom 360-degree error.
inline Eigen::Matrix<double, 7, 1>
JointTrackVelocity(const JointReference& reference, const RobotState& state,
                   double dt_s, double kp_s_inv)
{
    constexpr double kDegToRad = M_PI / 180.0;
    Eigen::Matrix<double, 7, 1> qdot_rad_s;
    for (int j = 0; j < 7; ++j)
    {
        const double ff_rad_s =
            (reference.q_ref_next_deg[j] - reference.q_ref_deg[j]) * kDegToRad /
            dt_s;
        const double error_rad = std::remainder(
            reference.q_ref_deg[j] * kDegToRad - state.q_rad[j], 2.0 * M_PI);
        qdot_rad_s[j] = ff_rad_s + kp_s_inv * error_rad;
    }
    return qdot_rad_s;
}

// ---------------------------------------------------------------
// TrajectorySource
// ---------------------------------------------------------------

//
// Replays one pre-validated trajectory against the loop clock. Each Get
// returns the reference pair {q_ref(t), q_ref(t+dt)}. Playback time
// advances by the loop's MEASURED dt — the same dt the integrator uses — so
// command and reference stay consistent through jitter (a stalled cycle
// plays slightly slower; it never causes a command jump).
//
// Failure states:
//   - start mismatch at Reset (arm moved between the pre-takeover gate and
//     the takeover): REFUSES permanently, reports state 3 with one edge,
//     and returns an empty reference forever — the controller holds.
//   - after the last sample: the pair freezes at the final row (zero
//     feed-forward, correction holds it), state 2 with one edge. The
//     operator stops the run; there is no automatic exit.
//
struct PlaybackSettings {
    double kp_s_inv = 0.0;                 // joint-law correction gain, 1/s
    double start_mismatch_limit_deg = 0.0; // per-joint refusal threshold
};

class TrajectorySource : public ReferenceSource
{
public:
    TrajectorySource(Trajectory trajectory, const PlaybackSettings& settings);

    void Reset(const RobotState& state) override;
    Reference Get(const RobotState& state, double dt_s,
                  ControllerStatus& status) override;

    // For the post-run report.
    bool refused() const { return refused_; }
    bool completed() const { return done_; }

private:
    const Trajectory trajectory_;
    const PlaybackSettings settings_;
    const double duration_s_;

    double t_play_s_ = 0.0;
    bool refused_ = false;
    bool refusal_reported_ = false;
    bool done_ = false;
};
