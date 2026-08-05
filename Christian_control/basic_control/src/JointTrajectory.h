//
// JointTrajectory — the joint-trajectory wire grammar, the accumulator that
// rebuilds one trajectory from a line stream, and the validation gate.
// Degrees and seconds on the wire, radians in memory. No I/O, no robot, no
// control math.
//

#pragma once

#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

struct JointTrajectoryPoint {
    double t_s;
    Eigen::Matrix<double, 7, 1> q_rad;
    Eigen::Matrix<double, 7, 1> qdot_rad_s;
};

// Time base is relative to the trajectory's own start: strictly increasing
// t_s, first point at t_s == 0. ValidateJointTrajectory is what enforces it.
struct JointTrajectory {
    std::vector<JointTrajectoryPoint> points;
};

// Upper bound on one wire block, so a bogus <count> from the producer cannot
// pin unbounded memory in the controller process.
inline constexpr std::size_t kMaxJointTrajectoryPoints = 1000;

// Rebuilds a trajectory from the line grammar
//
//   TRAJ_BEGIN <count>
//   <t_s> <q1..q7 deg> <v1..v7 deg/s>     (count rows, 15 fields each)
//   TRAJ_END
//
// One line at a time, degrees converted to radians on parse. The block is
// complete once <count> rows have arrived, so Feed returns the trajectory on
// the last row; the TRAJ_END that follows is accepted as a terminator and
// carries nothing. A TRAJ_END that arrives early is an error, as is any other
// line that does not fit the grammar at the current state: such a line resets
// the accumulator and sets `error`, so a malformed block is never partially
// accepted and no line is ever silently skipped.
class JointTrajectoryAccumulator
{
public:
    std::optional<JointTrajectory> Feed(const std::string& line,
                                        std::string& error);

    // True between a valid TRAJ_BEGIN and the block's end. A caller that
    // multiplexes trajectory lines with other input needs this to route
    // rows here rather than to the other parser.
    bool Collecting() const noexcept { return collecting_; }

private:
    void Reset() noexcept;

    bool collecting_ = false;
    std::size_t expected_points_ = 0;
    JointTrajectory pending_;
};

// nullopt = valid; otherwise the reason, naming the offending point/joint.
// Checks: >= 2 points, strictly increasing t_s, first t_s == 0, all values
// finite, every q within limits_low/high_deg, every |qdot| <= vel_limit_deg_s,
// and the average velocity implied between adjacent points <= vel_limit_deg_s
// per joint (the stated velocities alone cannot bound the actual motion).
std::optional<std::string> ValidateJointTrajectory(
    const JointTrajectory& traj,
    const Eigen::Matrix<double, 7, 1>& limits_low_deg,
    const Eigen::Matrix<double, 7, 1>& limits_high_deg,
    const Eigen::Matrix<double, 7, 1>& vel_limit_deg_s);

struct JointTrajectorySample {
    Eigen::Matrix<double, 7, 1> q_rad;
    Eigen::Matrix<double, 7, 1> qdot_rad_s;
    bool complete;
};

// Cubic Hermite between the two bracketing points (positions + stated
// velocities); clamps: t <= 0 returns the first point, t >= last returns the
// last point with zero velocity. `complete` true once t >= last t.
//
// Called from the control loop, so it neither allocates nor does I/O; an
// empty trajectory yields a zero sample already marked complete.
JointTrajectorySample SampleJointTrajectory(const JointTrajectory& traj,
                                            double t_s);
