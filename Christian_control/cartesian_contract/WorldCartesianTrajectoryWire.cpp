#include "WorldCartesianTrajectoryWire.h"

#include <charconv>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace {

std::string FirstToken(const std::string& line)
{
    std::istringstream input(line);
    std::string token;
    input >> token;
    return token;
}

template <typename Integer>
bool ParseInteger(const std::string& text, Integer& value)
{
    if (text.empty())
        return false;
    const char* const begin = text.data();
    const char* const end = begin + text.size();
    const auto result = std::from_chars(begin, end, value);
    return result.ec == std::errc() && result.ptr == end;
}

bool IsExactTerminator(const std::string& line)
{
    std::istringstream input(line);
    std::string token;
    std::string trailing;
    return (input >> token) && token == "CART_TRAJ_END" &&
           !(input >> trailing);
}

bool ParsePoint(const std::string& line,
                WorldCartesianTrajectoryPoint& point)
{
    std::istringstream input(line);
    double fields[14]{};
    for (double& field : fields) {
        if (!(input >> field) || !std::isfinite(field))
            return false;
    }
    std::string eligibility;
    std::string trailing;
    if (!(input >> eligibility) || (input >> trailing) ||
        (eligibility != "0" && eligibility != "1"))
        return false;

    point.t_from_start_s = fields[0];
    point.position_world_m = Eigen::Vector3d(fields[1], fields[2], fields[3]);
    point.orientation_world =
        Eigen::Quaterniond(fields[7], fields[4], fields[5], fields[6]);
    point.linear_velocity_world_m_s =
        Eigen::Vector3d(fields[8], fields[9], fields[10]);
    point.angular_velocity_world_rad_s =
        Eigen::Vector3d(fields[11], fields[12], fields[13]);
    point.arrival_eligible = eligibility == "1";
    return true;
}

}  // namespace

std::string FormatWorldCartesianTrajectoryBlock(
    const WorldCartesianTrajectory& trajectory)
{
    if (const std::optional<std::string> error =
            ValidateWorldCartesianTrajectory(trajectory))
        throw std::invalid_argument(*error);

    std::ostringstream block;
    block << std::setprecision(17);
    block << "CART_TRAJ_BEGIN 1 " << trajectory.trajectory_id << " "
          << trajectory.planner_vicon_sequence << " WORLD "
          << trajectory.points.size() << "\n";
    for (const WorldCartesianTrajectoryPoint& point : trajectory.points) {
        block << point.t_from_start_s;
        for (const double value : point.position_world_m)
            block << " " << value;
        block << " " << point.orientation_world.x()
              << " " << point.orientation_world.y()
              << " " << point.orientation_world.z()
              << " " << point.orientation_world.w();
        for (const double value : point.linear_velocity_world_m_s)
            block << " " << value;
        for (const double value : point.angular_velocity_world_rad_s)
            block << " " << value;
        block << " " << (point.arrival_eligible ? 1 : 0) << "\n";
    }
    block << "CART_TRAJ_END\n";
    return block.str();
}

void WorldCartesianTrajectoryAccumulator::Reset() noexcept
{
    collecting_ = false;
    expected_points_ = 0;
    pending_ = WorldCartesianTrajectory{};
}

std::optional<WorldCartesianTrajectory>
WorldCartesianTrajectoryAccumulator::Feed(const std::string& line,
                                          std::string& error)
{
    error.clear();
    const std::string keyword = FirstToken(line);

    if (!collecting_) {
        if (keyword != "CART_TRAJ_BEGIN") {
            error = "expected CART_TRAJ_BEGIN";
            Reset();
            return std::nullopt;
        }

        std::istringstream header(line);
        std::string begin;
        std::string version;
        std::string trajectory_id;
        std::string vicon_sequence;
        std::string frame;
        std::string count;
        std::string trailing;
        if (!(header >> begin >> version >> trajectory_id >> vicon_sequence >>
              frame >> count) ||
            (header >> trailing) || version != "1" || frame != "WORLD") {
            error = "expected 'CART_TRAJ_BEGIN 1 <trajectory_id> "
                    "<planner_vicon_sequence> WORLD <count>'";
            Reset();
            return std::nullopt;
        }

        std::uint64_t parsed_trajectory_id = 0;
        std::uint64_t parsed_vicon_sequence = 0;
        std::size_t parsed_count = 0;
        if (!ParseInteger(trajectory_id, parsed_trajectory_id) ||
            !ParseInteger(vicon_sequence, parsed_vicon_sequence) ||
            !ParseInteger(count, parsed_count) || parsed_count < 2 ||
            parsed_count > kMaxWorldCartesianTrajectoryPoints) {
            error = "CART_TRAJ_BEGIN IDs must be unsigned integers and count "
                    "must be in [2, " +
                    std::to_string(kMaxWorldCartesianTrajectoryPoints) + "]";
            Reset();
            return std::nullopt;
        }

        Reset();
        collecting_ = true;
        expected_points_ = parsed_count;
        pending_.trajectory_id = parsed_trajectory_id;
        pending_.planner_vicon_sequence = parsed_vicon_sequence;
        pending_.points.reserve(parsed_count);
        return std::nullopt;
    }

    if (keyword == "CART_TRAJ_END") {
        if (!IsExactTerminator(line)) {
            error = "CART_TRAJ_END has trailing tokens";
            Reset();
            return std::nullopt;
        }
        if (pending_.points.size() != expected_points_) {
            error = "CART_TRAJ_END after " +
                    std::to_string(pending_.points.size()) + " of " +
                    std::to_string(expected_points_) + " declared rows";
            Reset();
            return std::nullopt;
        }
        if (const std::optional<std::string> validation_error =
                ValidateWorldCartesianTrajectory(pending_)) {
            error = *validation_error;
            Reset();
            return std::nullopt;
        }
        WorldCartesianTrajectory complete = std::move(pending_);
        Reset();
        return complete;
    }

    if (pending_.points.size() >= expected_points_) {
        error = "expected CART_TRAJ_END after declared rows";
        Reset();
        return std::nullopt;
    }

    WorldCartesianTrajectoryPoint point;
    if (!ParsePoint(line, point)) {
        error = "expected 14 finite numbers followed by arrival_eligible 0 or 1";
        Reset();
        return std::nullopt;
    }
    pending_.points.push_back(point);
    return std::nullopt;
}
