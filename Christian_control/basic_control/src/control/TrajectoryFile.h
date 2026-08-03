//
// TrajectoryFile: the planner-independent joint-trajectory contract —
// loading, validation and time-indexed sampling. Pure Eigen + standard
// library; never talks to the robot, never allocates after loading.
//
// THE CONTRACT (trajectory_format = 1). A trajectory file is a CSV:
//
//   # trajectory_format = 1
//   # <key> = <value>            (free metadata; recorded, not interpreted)
//   t_s,q1_deg,...,q7_deg,qd1_degs,...,qd7_degs
//   0,...
//
// Semantics a producer must guarantee and a consumer may rely on:
//   - 7 joints, Kinova actuator order 1-7 (the right arm's Actuator 1..7);
//     one row per sample.
//   - q*_deg are joint positions in DEGREES, continuous (unwrapped) and on
//     the branch nearest the start configuration the file was planned
//     from. Consumers compare against measured [0,360) feedback with
//     wrapped (remainder) arithmetic, so the absolute branch is free.
//   - qd*_degs are joint velocities in DEGREES/SECOND, consistent with
//     the position column (finite differences must agree — validated).
//   - t_s starts at 0 and advances in EXACTLY uniform steps (the file's
//     dt). Time is implicit position-in-file; there are no per-row
//     timestamps beyond this grid.
//   - the first and last rows are at rest (|qd| ~ 0): execution starts
//     from a hold and ends in a hold.
//
// Loading throws std::runtime_error on any structural violation (missing
// format marker, malformed rows, non-finite values, non-uniform time).
// ValidateTrajectory then checks the loaded data against motion limits
// and returns human-readable violations — an empty list is the ONLY
// go-ahead. Load errors and validation failures must both keep the run
// from ever reaching a takeover.
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_TRAJECTORY_FILE_H
#define HUMANSL_MASTERS_PROJECT_2025_TRAJECTORY_FILE_H

#include <istream>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "JointVector.h"

struct Trajectory {
    double dt_s = 0.0; // uniform sample spacing, from the t_s column
    std::vector<Eigen::Matrix<double, 7, 1>> pos_deg;
    std::vector<Eigen::Matrix<double, 7, 1>> vel_deg_s;
    std::map<std::string, std::string> metadata; // '#' preamble key = value
};

// Parse a trajectory_format = 1 stream/file. Throws std::runtime_error
// with a one-line reason on the first structural violation.
Trajectory LoadTrajectoryCsv(std::istream& in, const std::string& name);
Trajectory LoadTrajectoryCsv(const std::string& path);

// Everything ValidateTrajectory measured, for the pre-run printout: the
// numbers an operator must see before authorizing motion.
struct TrajectorySummary {
    double duration_s = 0.0;
    std::size_t samples = 0;
    JointVector displacement_deg{};   // last row - first row, per joint
    JointVector peak_vel_deg_s{};     // max |qd| per joint, from the file
    JointVector peak_accel_deg_s2{};  // max |dqd/dt| per joint, finite diff
    double start_vel_deg_s = 0.0;     // max |qd| across joints, first row
    double end_vel_deg_s = 0.0;       // max |qd| across joints, last row
};

// Check a loaded trajectory against motion limits. Returns human-readable
// violations (empty = pass) and fills `summary` either way.
//   vel_limit_deg_s   per-joint ceiling on |qd| — pass the command clip
//                     times a margin, so the Runner's clamp can never
//                     engage on a validated file
//   accel_limit_deg_s2 per-joint ceiling on |dqd/dt|
//   pos_limit_deg     per-joint ceiling on the WRAPPED |q| (the Gen3's
//                     bounded joints have ranges symmetric about 0); 0
//                     disables the check for that joint (the continuous
//                     joints 1/3/5/7 have no position limit)
// Also enforced: velocity-vs-position consistency (the qd columns must
// describe the q columns), rest at both ends (both the qd column AND the
// motion the positions imply), and per-sample steps small enough for the
// position servo to follow. NOTE: this checks the URDF/Kinova model
// ranges only — the arm's CONFIGURED soft limits can sit far inside them
// (README "Safety"); confirm them in the Kinova web dashboard before a
// session.
std::vector<std::string> ValidateTrajectory(const Trajectory& trajectory,
                                            const JointVector& vel_limit_deg_s,
                                            const JointVector& accel_limit_deg_s2,
                                            const JointVector& pos_limit_deg,
                                            TrajectorySummary& summary);

// Reference position at time t_s, linearly interpolated between samples
// and clamped to [0, duration]. Pure, allocation-free — safe in the loop.
Eigen::Matrix<double, 7, 1> SamplePositionDeg(const Trajectory& trajectory,
                                              double t_s);

double TrajectoryDurationS(const Trajectory& trajectory);

#endif // HUMANSL_MASTERS_PROJECT_2025_TRAJECTORY_FILE_H
