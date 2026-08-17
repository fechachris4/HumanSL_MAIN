#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "WorldCartesianTrajectory.h"
#include "WorldCartesianTrajectoryWire.h"

namespace {

int failures = 0;

void Check(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        ++failures;
    }
}

WorldCartesianTrajectory ValidTrajectory()
{
    WorldCartesianTrajectory trajectory;
    trajectory.trajectory_id = 9;
    trajectory.planner_vicon_sequence = 42;

    WorldCartesianTrajectoryPoint first;
    first.position_world_m = Eigen::Vector3d(1.0, 2.0, 3.0);
    first.orientation_world =
        Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()));
    first.linear_velocity_world_m_s = Eigen::Vector3d(0.1, -0.2, 0.3);
    first.angular_velocity_world_rad_s = Eigen::Vector3d(-0.4, 0.5, -0.6);

    WorldCartesianTrajectoryPoint last = first;
    last.t_from_start_s = 0.002;
    last.position_world_m.x() = 1.0002;
    last.linear_velocity_world_m_s.setZero();
    last.angular_velocity_world_rad_s.setZero();
    last.arrival_eligible = true;
    trajectory.points = {first, last};
    return trajectory;
}

std::vector<std::string> Lines(const std::string& block)
{
    std::istringstream input(block);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(input, line))
        lines.push_back(line);
    return lines;
}

void CheckInvalid(const WorldCartesianTrajectory& trajectory,
                  const std::string& message)
{
    Check(ValidateWorldCartesianTrajectory(trajectory).has_value(), message);
}

void CheckBadHeader(const std::string& line, const std::string& message)
{
    WorldCartesianTrajectoryAccumulator accumulator;
    std::string error;
    Check(!accumulator.Feed(line, error).has_value() && !error.empty() &&
              !accumulator.Collecting(),
          message);
}

} // namespace

int main()
{
    const WorldCartesianTrajectory valid = ValidTrajectory();
    Check(!ValidateWorldCartesianTrajectory(valid).has_value(),
          "valid world Cartesian trajectory passes");

    const std::string block = FormatWorldCartesianTrajectoryBlock(valid);
    Check(block.rfind("CART_TRAJ_BEGIN 1 9 42 WORLD 2\n", 0) == 0,
          "formatter emits the exact versioned WORLD header");
    Check(block.find("CART_TRAJ_END\n") != std::string::npos,
          "formatter emits the mandatory terminator");

    // Parsing is atomic: even after all declared rows, no trajectory is
    // returned until the terminator has arrived.
    WorldCartesianTrajectoryAccumulator accumulator;
    std::string error;
    std::optional<WorldCartesianTrajectory> parsed;
    const std::vector<std::string> lines = Lines(block);
    for (std::size_t i = 0; i < lines.size(); ++i) {
        const std::optional<WorldCartesianTrajectory> result =
            accumulator.Feed(lines[i], error);
        Check(error.empty(), "formatted line " + std::to_string(i) + " parses");
        if (i + 1 < lines.size())
            Check(!result.has_value(), "partial block is never published");
        else
            parsed = result;
    }
    Check(parsed.has_value() && !accumulator.Collecting(),
          "terminator atomically completes the block");
    if (parsed) {
        Check(parsed->trajectory_id == 9 &&
                  parsed->planner_vicon_sequence == 42 &&
                  parsed->points.size() == 2,
              "header provenance round-trips");
        Check((parsed->points.front().orientation_world.coeffs() -
               valid.points.front().orientation_world.coeffs()).norm() < 1e-15,
              "17-digit formatting preserves the unit quaternion");
        Check(parsed->points.back().arrival_eligible,
              "terminal arrival eligibility round-trips");
    }

    WorldCartesianTrajectory invalid = valid;
    invalid.points.resize(1);
    CheckInvalid(invalid, "fewer than two points is rejected");
    invalid = valid;
    invalid.points.front().t_from_start_s = 0.001;
    CheckInvalid(invalid, "nonzero first time is rejected");
    invalid = valid;
    invalid.points.back().t_from_start_s = 0.0;
    CheckInvalid(invalid, "non-increasing time is rejected");
    invalid = valid;
    invalid.points.front().position_world_m.x() =
        std::numeric_limits<double>::quiet_NaN();
    CheckInvalid(invalid, "non-finite point data is rejected");
    invalid = valid;
    invalid.points.front().orientation_world.coeffs() *= 1.01;
    CheckInvalid(invalid, "quaternion norm outside tolerance is rejected");
    invalid = valid;
    invalid.points.front().arrival_eligible = true;
    CheckInvalid(invalid, "arrival eligibility before the final point is rejected");
    invalid = valid;
    invalid.points.back().arrival_eligible = false;
    CheckInvalid(invalid, "the final point must be arrival eligible");
    invalid = valid;
    invalid.points.back().linear_velocity_world_m_s.x() = 0.01;
    CheckInvalid(invalid, "nonzero final linear twist is rejected");
    invalid = valid;
    invalid.points.back().angular_velocity_world_rad_s.z() = 0.01;
    CheckInvalid(invalid, "nonzero final angular twist is rejected");
    invalid = valid;
    invalid.points.resize(kMaxWorldCartesianTrajectoryPoints + 1,
                          valid.points.back());
    CheckInvalid(invalid, "trajectory over the point cap is rejected");

    CheckBadHeader("CART_TRAJ_BEGIN 2 9 42 WORLD 2",
                   "unknown version is rejected");
    CheckBadHeader("CART_TRAJ_BEGIN 1 9 42 MOUNT 2",
                   "non-WORLD frame is rejected");
    CheckBadHeader("CART_TRAJ_BEGIN 1 9 42 WORLD 20001",
                   "excessive declared count is rejected");
    CheckBadHeader("CART_TRAJ_BEGIN 1 9 42 WORLD 2 trailing",
                   "header trailing tokens are rejected");

    WorldCartesianTrajectoryAccumulator early;
    Check(!early.Feed(lines.front(), error).has_value() && error.empty(),
          "valid header starts collection");
    Check(!early.Feed("CART_TRAJ_END", error).has_value() && !error.empty() &&
              !early.Collecting(),
          "early terminator rejects and resets the block");

    WorldCartesianTrajectoryAccumulator missing;
    for (std::size_t i = 0; i + 1 < lines.size(); ++i) {
        Check(!missing.Feed(lines[i], error).has_value() && error.empty(),
              "unterminated block stays unpublished");
    }
    Check(missing.Collecting(), "missing terminator leaves block incomplete");

    WorldCartesianTrajectoryAccumulator trailing;
    Check(!trailing.Feed(lines[0], error).has_value() && error.empty(),
          "trailing-token fixture header accepted");
    Check(!trailing.Feed(lines[1] + " trailing", error).has_value() &&
              !error.empty() && !trailing.Collecting(),
          "point-row trailing tokens reject and reset the block");

    if (failures == 0) {
        std::cout << "all Cartesian-contract tests passed\n";
        return 0;
    }
    std::cerr << failures << " test(s) failed\n";
    return 1;
}
