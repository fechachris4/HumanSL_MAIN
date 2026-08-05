//
// JointTrajectory — implementations for JointTrajectory.h.
//

#include <sstream>

#include "JointTrajectory.h"

namespace
{
    constexpr double kDegToRad = M_PI / 180.0;
    constexpr std::size_t kRowFields = 15;
    constexpr double kStartTimeToleranceS = 1e-9;
    // Comparisons against a limit are inclusive; the slack keeps a value that
    // is exactly at the limit on the wire from being rejected by round-off.
    constexpr double kLimitSlackRad = 1e-9;

    std::string FirstToken(const std::string& line)
    {
        std::istringstream input(line);
        std::string token;
        input >> token;
        return token;
    }

    // Same discipline as ParsePoseTarget: exactly the expected count of
    // numbers, nothing trailing, every one of them finite.
    bool ParseFields(const std::string& line, std::size_t expected,
                     std::vector<double>& fields)
    {
        std::istringstream input(line);
        fields.clear();
        double value = 0.0;
        while (input >> value)
            fields.push_back(value);
        std::string trailing;
        input.clear();
        if (input >> trailing || fields.size() != expected)
            return false;
        for (const double field : fields)
            if (!std::isfinite(field))
                return false;
        return true;
    }

    std::string JointName(Eigen::Index joint)
    {
        return "joint " + std::to_string(joint + 1);
    }

    std::string PointName(std::size_t index)
    {
        return "point " + std::to_string(index);
    }
} // namespace

void JointTrajectoryAccumulator::Reset() noexcept
{
    collecting_ = false;
    expected_points_ = 0;
    pending_.points.clear();
}

std::optional<JointTrajectory> JointTrajectoryAccumulator::Feed(
    const std::string& line, std::string& error)
{
    error.clear();
    const std::string keyword = FirstToken(line);

    if (!collecting_) {
        // The trailing terminator of a block already completed by its last row.
        if (keyword == "TRAJ_END")
            return std::nullopt;
        if (keyword != "TRAJ_BEGIN") {
            error = "expected 'TRAJ_BEGIN <count>'";
            Reset();
            return std::nullopt;
        }
        std::istringstream header(line);
        std::string token;
        long long count = 0;
        header >> token >> count;
        std::string trailing;
        if (!header || header >> trailing || count < 2 ||
            static_cast<std::size_t>(count) > kMaxJointTrajectoryPoints) {
            error = "TRAJ_BEGIN count must be an integer in [2, " +
                    std::to_string(kMaxJointTrajectoryPoints) + "]";
            Reset();
            return std::nullopt;
        }
        Reset();
        collecting_ = true;
        expected_points_ = static_cast<std::size_t>(count);
        return std::nullopt;
    }

    if (keyword == "TRAJ_END") {
        error = "TRAJ_END after " + std::to_string(pending_.points.size()) +
                " of " + std::to_string(expected_points_) + " declared rows";
        Reset();
        return std::nullopt;
    }

    std::vector<double> fields;
    if (!ParseFields(line, kRowFields, fields)) {
        error = "expected 15 finite numbers '<t_s> <q1..q7 deg> <v1..v7 deg/s>'";
        Reset();
        return std::nullopt;
    }

    JointTrajectoryPoint point;
    point.t_s = fields[0];
    for (Eigen::Index joint = 0; joint < 7; ++joint) {
        point.q_rad(joint) = fields[1 + static_cast<std::size_t>(joint)] * kDegToRad;
        point.qdot_rad_s(joint) =
            fields[8 + static_cast<std::size_t>(joint)] * kDegToRad;
    }
    pending_.points.push_back(point);
    if (pending_.points.size() < expected_points_)
        return std::nullopt;

    JointTrajectory completed = std::move(pending_);
    Reset();
    return completed;
}

std::optional<std::string> ValidateJointTrajectory(
    const JointTrajectory& traj,
    const Eigen::Matrix<double, 7, 1>& limits_low_deg,
    const Eigen::Matrix<double, 7, 1>& limits_high_deg,
    const Eigen::Matrix<double, 7, 1>& vel_limit_deg_s)
{
    if (traj.points.size() < 2)
        return "trajectory needs at least 2 points";
    if (!limits_low_deg.allFinite() || !limits_high_deg.allFinite() ||
        !vel_limit_deg_s.allFinite())
        return "joint limits must be finite";
    for (Eigen::Index joint = 0; joint < 7; ++joint) {
        if (limits_high_deg(joint) < limits_low_deg(joint))
            return "upper limit below lower limit for " + JointName(joint);
        if (vel_limit_deg_s(joint) <= 0.0)
            return "velocity limit must be positive for " + JointName(joint);
    }

    const Eigen::Matrix<double, 7, 1> low_rad = limits_low_deg * kDegToRad;
    const Eigen::Matrix<double, 7, 1> high_rad = limits_high_deg * kDegToRad;
    const Eigen::Matrix<double, 7, 1> vel_limit_rad_s = vel_limit_deg_s * kDegToRad;

    if (std::abs(traj.points.front().t_s) > kStartTimeToleranceS)
        return "first point must be at t_s == 0";

    for (std::size_t index = 0; index < traj.points.size(); ++index) {
        const JointTrajectoryPoint& point = traj.points[index];
        if (!std::isfinite(point.t_s) || !point.q_rad.allFinite() ||
            !point.qdot_rad_s.allFinite())
            return PointName(index) + " has non-finite values";
        if (index > 0 && point.t_s <= traj.points[index - 1].t_s)
            return PointName(index) + " time is not strictly increasing";

        for (Eigen::Index joint = 0; joint < 7; ++joint) {
            if (point.q_rad(joint) < low_rad(joint) - kLimitSlackRad ||
                point.q_rad(joint) > high_rad(joint) + kLimitSlackRad)
                return PointName(index) + " " + JointName(joint) +
                       " position is outside its limits";
            if (std::abs(point.qdot_rad_s(joint)) >
                vel_limit_rad_s(joint) + kLimitSlackRad)
                return PointName(index) + " " + JointName(joint) +
                       " velocity exceeds its limit";
        }
    }

    // Stated velocities can look legal while the positions they connect
    // demand far more speed, so the implied average is checked too.
    for (std::size_t index = 1; index < traj.points.size(); ++index) {
        const double dt_s = traj.points[index].t_s - traj.points[index - 1].t_s;
        for (Eigen::Index joint = 0; joint < 7; ++joint) {
            const double dq_rad = traj.points[index].q_rad(joint) -
                                  traj.points[index - 1].q_rad(joint);
            if (std::abs(dq_rad) >
                (vel_limit_rad_s(joint) + kLimitSlackRad) * dt_s)
                return "segment to " + PointName(index) + " needs more than " +
                       JointName(joint) + "'s velocity limit";
        }
    }
    return std::nullopt;
}
