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

JointTrajectorySample SampleJointTrajectory(const JointTrajectory& traj,
                                            double t_s)
{
    JointTrajectorySample sample;
    sample.q_rad.setZero();
    sample.qdot_rad_s.setZero();
    sample.complete = true;
    if (traj.points.empty())
        return sample;

    const JointTrajectoryPoint& first = traj.points.front();
    const JointTrajectoryPoint& last = traj.points.back();
    if (!(t_s > first.t_s)) {
        sample.q_rad = first.q_rad;
        sample.qdot_rad_s = first.qdot_rad_s;
        // A NaN time lands here; holding the start with `complete` false is
        // the standing-still answer, and the loop keeps its own timeout.
        sample.complete = traj.points.size() < 2;
        return sample;
    }
    if (t_s >= last.t_s) {
        sample.q_rad = last.q_rad;
        return sample;
    }

    std::size_t upper = 1;
    while (traj.points[upper].t_s <= t_s)
        ++upper;
    const JointTrajectoryPoint& p0 = traj.points[upper - 1];
    const JointTrajectoryPoint& p1 = traj.points[upper];

    const double dt_s = p1.t_s - p0.t_s;
    // Degenerate-input fallback, unreachable for a validated trajectory: hold
    // the segment's start rather than divide by a width that is zero, negative
    // or NaN. Written as !(dt_s > 0) so a NaN width is caught too, which
    // dt_s <= 0 would let through. Not an assert: the production build may
    // define NDEBUG, and this must degrade safely exactly there.
    if (!(dt_s > 0.0)) {
        sample.q_rad = p0.q_rad;
        sample.complete = false;
        return sample;
    }

    const double s = (t_s - p0.t_s) / dt_s;
    const double s2 = s * s;
    const double s3 = s2 * s;

    const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
    const double h10 = s3 - 2.0 * s2 + s;
    const double h01 = -2.0 * s3 + 3.0 * s2;
    const double h11 = s3 - s2;

    const double d00 = 6.0 * s2 - 6.0 * s;
    const double d10 = 3.0 * s2 - 4.0 * s + 1.0;
    const double d01 = -6.0 * s2 + 6.0 * s;
    const double d11 = 3.0 * s2 - 2.0 * s;

    sample.q_rad = h00 * p0.q_rad + h10 * dt_s * p0.qdot_rad_s +
                   h01 * p1.q_rad + h11 * dt_s * p1.qdot_rad_s;
    sample.qdot_rad_s = (d00 * p0.q_rad + d01 * p1.q_rad) / dt_s +
                        d10 * p0.qdot_rad_s + d11 * p1.qdot_rad_s;
    sample.complete = false;
    return sample;
}
