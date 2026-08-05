#pragma once
#include <cstddef>
#include <string>
#include <vector>
#include <gtsam/base/Vector.h>

// Mirrors the controller's kMaxJointTrajectoryPoints (JointTrajectory.h) —
// pinned by test_bridge_main rather than included, so bridge_core stays
// independent of basic_control. The optimizer densifies to ~1 kHz, which
// overruns this, so blocks longer than the cap are decimated below.
inline constexpr std::size_t kMaxTrajectoryBlockPoints = 1000;

// Formats a solved joint trajectory as the controller's wire block
//
//   TRAJ_BEGIN <count>
//   <t_s> <q1..q7 deg> <v1..v7 deg/s>     (count rows, 15 fields each)
//   TRAJ_END
//
// Radians in, degrees out. The solved states are evenly spaced in time:
// t_i = i * (total_time_sec / (n_states - 1)). When there are more than
// kMaxTrajectoryBlockPoints of them, an evenly spaced subset of that size
// is emitted — first and last state always kept, each row keeping the t_i
// of the state it came from. Returns the whole block as text and writes
// nothing itself — RunBridge owns the buffering that keeps a failed run
// from emitting anything.
//
// Requires pos_rad.size() == vel_rad_s.size() >= 2, every entry of length 7,
// and a finite positive total_time_sec; throws std::invalid_argument
// otherwise, since a malformed block would be motion the controller
// executes.
std::string FormatTrajectoryBlock(const std::vector<gtsam::Vector>& pos_rad,
                                  const std::vector<gtsam::Vector>& vel_rad_s,
                                  double total_time_sec);
