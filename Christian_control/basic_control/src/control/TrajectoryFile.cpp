//
// TrajectoryFile: loading, validation and sampling (contract in the header).
//

#include "control/TrajectoryFile.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace
{
    constexpr int NUM_JOINTS = 7;

    // Uniform-grid tolerance: the t_s column carries ~6 significant digits
    // (the writer uses default stream precision), so at 100 s the absolute
    // quantization is ~1e-4 s. Anything beyond that is a real grid error.
    constexpr double kTimeGridToleranceS = 1e-4;

    // |finite difference of q - qd| tolerance, deg/s. The producer's qd is
    // an analytic derivative, the check is a one-sided difference: they
    // differ by ~0.5 * accel * dt (= 0.3 deg/s at the joint 5-7 limit) plus
    // CSV quantization. 1.0 deg/s catches a wrong or mislabeled column
    // without false-failing an honest file.
    constexpr double kVelConsistencyToleranceDegS = 1.0;

    // "At rest" for the first and last row, deg/s.
    constexpr double kRestVelDegS = 0.1;

    [[noreturn]] void Fail(const std::string& name, const std::string& why)
    {
        throw std::runtime_error(name + ": " + why);
    }

    std::vector<double> SplitRow(const std::string& line)
    {
        std::vector<double> values;
        std::istringstream fields(line);
        std::string field;
        while (std::getline(fields, field, ','))
        {
            try
            {
                std::size_t used = 0;
                const double v = std::stod(field, &used);
                if (used != field.size())
                    return {}; // trailing junk in a numeric field
                values.push_back(v);
            }
            catch (...)
            {
                return {}; // empty vector = parse failure, caller reports
            }
        }
        return values;
    }
} // namespace

Trajectory LoadTrajectoryCsv(std::istream& in, const std::string& name)
{
    Trajectory trajectory;

    // Preamble: '#' lines, "key = value" recorded as metadata. The format
    // marker is mandatory — it is the producer's declaration that the file
    // implements this contract rather than merely resembling a CSV.
    std::string line;
    bool header_seen = false;
    while (std::getline(in, line))
    {
        if (line.empty())
            continue;
        if (line[0] == '#')
        {
            const std::size_t eq = line.find('=');
            if (eq != std::string::npos)
            {
                auto trim = [](std::string s)
                {
                    const auto a = s.find_first_not_of(" \t#");
                    const auto b = s.find_last_not_of(" \t\r");
                    return a == std::string::npos ? std::string()
                                                  : s.substr(a, b - a + 1);
                };
                trajectory.metadata[trim(line.substr(0, eq))] =
                    trim(line.substr(eq + 1));
            }
            continue;
        }
        header_seen = true;
        break; // `line` now holds the column header
    }
    if (!header_seen)
        Fail(name, "no column header row");

    const auto format = trajectory.metadata.find("trajectory_format");
    if (format == trajectory.metadata.end())
        Fail(name, "missing '# trajectory_format = 1' marker — not a "
                   "trajectory file under the playback contract");
    if (format->second != "1")
        Fail(name, "unsupported trajectory_format '" + format->second + "'");

    // Header: exact names, exact order — the column meaning is positional
    // in every consumer, so a reordered file must be rejected, not adapted.
    {
        std::ostringstream expected;
        expected << "t_s";
        for (int j = 1; j <= NUM_JOINTS; ++j)
            expected << ",q" << j << "_deg";
        for (int j = 1; j <= NUM_JOINTS; ++j)
            expected << ",qd" << j << "_degs";
        std::string header = line;
        if (!header.empty() && header.back() == '\r')
            header.pop_back();
        if (header != expected.str())
            Fail(name, "header mismatch — expected '" + expected.str() +
                           "', got '" + header + "'");
    }

    std::vector<double> times;
    std::size_t row_number = 0;
    while (std::getline(in, line))
    {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        if (line.empty())
            continue;
        ++row_number;
        const std::vector<double> v = SplitRow(line);
        if (v.size() != 1 + 2 * NUM_JOINTS)
            Fail(name, "row " + std::to_string(row_number) + " has " +
                           std::to_string(v.size()) + " fields, expected " +
                           std::to_string(1 + 2 * NUM_JOINTS) +
                           (v.empty() ? " (unparseable field)" : ""));
        for (double x : v)
            if (!std::isfinite(x))
                Fail(name, "row " + std::to_string(row_number) +
                               " contains a non-finite value");
        times.push_back(v[0]);
        Eigen::Matrix<double, 7, 1> q, qd;
        for (int j = 0; j < NUM_JOINTS; ++j)
        {
            q[j] = v[1 + j];
            qd[j] = v[1 + NUM_JOINTS + j];
        }
        trajectory.pos_deg.push_back(q);
        trajectory.vel_deg_s.push_back(qd);
    }

    if (trajectory.pos_deg.size() < 2)
        Fail(name, "only " + std::to_string(trajectory.pos_deg.size()) +
                       " sample rows — a trajectory needs at least 2");
    if (std::abs(times.front()) > kTimeGridToleranceS)
        Fail(name, "t_s must start at 0 (got " +
                       std::to_string(times.front()) + ")");

    const double dt = times[1] - times[0];
    if (dt <= 0.0)
        Fail(name, "non-increasing t_s at row 2");
    // Knot-spacing bound: execution interpolates LINEARLY at the 1 kHz
    // loop rate, so the validator's acceleration gate only bounds what is
    // actually executed when the knots are at loop-rate scale. A coarse
    // grid (say 0.5 s) could pass every gate yet execute step-like
    // velocity changes at each knot (2026-07-31 review, finding 2). It
    // also keeps the grid tolerance meaningful (finding 6).
    if (dt < 0.0005 - kTimeGridToleranceS || dt > 0.002 + kTimeGridToleranceS)
        Fail(name, "knot spacing " + std::to_string(dt) +
                       " s outside the contract's [0.0005, 0.002] s — the "
                       "1 kHz executor requires loop-rate-scale samples");
    for (std::size_t i = 1; i < times.size(); ++i)
        if (std::abs(times[i] - times[i - 1] - dt) > kTimeGridToleranceS)
            Fail(name, "non-uniform t_s at row " + std::to_string(i + 1) +
                           " (step " + std::to_string(times[i] - times[i - 1]) +
                           ", grid " + std::to_string(dt) + ")");
    trajectory.dt_s = dt;
    return trajectory;
}

Trajectory LoadTrajectoryCsv(const std::string& path)
{
    std::ifstream in(path);
    if (!in)
        throw std::runtime_error(path + ": cannot open");
    return LoadTrajectoryCsv(in, path);
}

std::vector<std::string> ValidateTrajectory(const Trajectory& trajectory,
                                            const JointVector& vel_limit_deg_s,
                                            const JointVector& accel_limit_deg_s2,
                                            const JointVector& pos_limit_deg,
                                            TrajectorySummary& summary)
{
    std::vector<std::string> violations;
    const std::size_t n = trajectory.pos_deg.size();
    const double dt = trajectory.dt_s;

    summary = TrajectorySummary{};
    // The loader guarantees n >= 2 and dt > 0, but this is a public
    // function tools and tests call with hand-built data — the size_t
    // n - 1 below must never underflow.
    if (n < 2 || trajectory.vel_deg_s.size() != n || !(dt > 0.0))
    {
        violations.push_back("not a loadable trajectory (fewer than 2 "
                             "samples, mismatched columns, or dt <= 0)");
        return violations;
    }
    summary.samples = n;
    summary.duration_s = static_cast<double>(n - 1) * dt;
    for (int j = 0; j < NUM_JOINTS; ++j)
    {
        summary.displacement_deg[j] =
            trajectory.pos_deg.back()[j] - trajectory.pos_deg.front()[j];
        summary.start_vel_deg_s = std::max(
            summary.start_vel_deg_s, std::abs(trajectory.vel_deg_s.front()[j]));
        summary.end_vel_deg_s = std::max(
            summary.end_vel_deg_s, std::abs(trajectory.vel_deg_s.back()[j]));
    }

    auto violation = [&violations](const std::string& what)
    {
        // One line per rule keeps the report readable; the first offending
        // sample is evidence enough.
        violations.push_back(what);
    };

    for (int j = 0; j < NUM_JOINTS; ++j)
    {
        bool vel_reported = false, accel_reported = false, cons_reported = false,
             step_reported = false;
        bool pos_reported = false;
        for (std::size_t i = 0; i < n; ++i)
        {
            // Position range, wrapped: the file's continuous angles are
            // compared as the arm sees them. Only meaningful for the
            // bounded joints (limit 0 = continuous, skip).
            if (pos_limit_deg[j] > 0.0 && !pos_reported)
            {
                const double wrapped =
                    std::abs(std::remainder(trajectory.pos_deg[i][j], 360.0));
                if (wrapped > pos_limit_deg[j])
                {
                    pos_reported = true;
                    violation("joint " + std::to_string(j + 1) +
                              ": position " + std::to_string(wrapped) +
                              " deg exceeds the " +
                              std::to_string(pos_limit_deg[j]) +
                              " deg model range at t=" +
                              std::to_string(i * dt) + " s");
                }
            }
            const double vel = std::abs(trajectory.vel_deg_s[i][j]);
            summary.peak_vel_deg_s[j] = std::max(summary.peak_vel_deg_s[j], vel);
            if (vel > vel_limit_deg_s[j] && !vel_reported)
            {
                vel_reported = true;
                violation("joint " + std::to_string(j + 1) + ": |velocity| " +
                          std::to_string(vel) + " deg/s exceeds the " +
                          std::to_string(vel_limit_deg_s[j]) +
                          " deg/s gate at t=" + std::to_string(i * dt) + " s");
            }
            if (i > 0)
            {
                const double accel =
                    std::abs(trajectory.vel_deg_s[i][j] -
                             trajectory.vel_deg_s[i - 1][j]) /
                    dt;
                summary.peak_accel_deg_s2[j] =
                    std::max(summary.peak_accel_deg_s2[j], accel);
                if (accel > accel_limit_deg_s2[j] && !accel_reported)
                {
                    accel_reported = true;
                    violation("joint " + std::to_string(j + 1) +
                              ": |acceleration| " + std::to_string(accel) +
                              " deg/s^2 exceeds the " +
                              std::to_string(accel_limit_deg_s2[j]) +
                              " deg/s^2 gate at t=" + std::to_string(i * dt) +
                              " s");
                }
                const double step =
                    trajectory.pos_deg[i][j] - trajectory.pos_deg[i - 1][j];
                const double implied_vel = step / dt;
                if (std::abs(implied_vel - trajectory.vel_deg_s[i - 1][j]) >
                        kVelConsistencyToleranceDegS &&
                    !cons_reported)
                {
                    cons_reported = true;
                    violation("joint " + std::to_string(j + 1) +
                              ": velocity column disagrees with the position "
                              "column at t=" + std::to_string((i - 1) * dt) +
                              " s (position implies " +
                              std::to_string(implied_vel) + " deg/s, file says " +
                              std::to_string(trajectory.vel_deg_s[i - 1][j]) +
                              ") — columns mixed up or wrong dt?");
                }
                if (std::abs(step) > vel_limit_deg_s[j] * dt &&
                    !step_reported)
                {
                    step_reported = true;
                    violation("joint " + std::to_string(j + 1) +
                              ": per-sample position step " +
                              std::to_string(std::abs(step)) +
                              " deg exceeds vel-gate*dt at t=" +
                              std::to_string(i * dt) + " s");
                }
            }
        }
    }

    if (summary.start_vel_deg_s > kRestVelDegS)
        violation("first row is not at rest (max |qd| " +
                  std::to_string(summary.start_vel_deg_s) + " deg/s, limit " +
                  std::to_string(kRestVelDegS) + ")");
    if (summary.end_vel_deg_s > kRestVelDegS)
        violation("last row is not at rest (max |qd| " +
                  std::to_string(summary.end_vel_deg_s) + " deg/s, limit " +
                  std::to_string(kRestVelDegS) + ")");
    // Rest must also hold in the POSITIONS, not just the qd column — a
    // file whose qd lies at the endpoints would otherwise start playback
    // with an immediate feed-forward step (2026-07-31 review, finding 8a).
    for (int j = 0; j < NUM_JOINTS; ++j)
    {
        const double implied_start =
            std::abs(trajectory.pos_deg[1][j] - trajectory.pos_deg[0][j]) / dt;
        const double implied_end =
            std::abs(trajectory.pos_deg[n - 1][j] -
                     trajectory.pos_deg[n - 2][j]) /
            dt;
        if (implied_start > kRestVelDegS + kVelConsistencyToleranceDegS)
        {
            violation("joint " + std::to_string(j + 1) +
                      ": positions imply motion at the first row (" +
                      std::to_string(implied_start) + " deg/s)");
            break;
        }
        if (implied_end > kRestVelDegS + kVelConsistencyToleranceDegS)
        {
            violation("joint " + std::to_string(j + 1) +
                      ": positions imply motion at the last row (" +
                      std::to_string(implied_end) + " deg/s)");
            break;
        }
    }

    return violations;
}

Eigen::Matrix<double, 7, 1> SamplePositionDeg(const Trajectory& trajectory,
                                              double t_s)
{
    const std::size_t n = trajectory.pos_deg.size();
    const double duration = static_cast<double>(n - 1) * trajectory.dt_s;
    if (t_s <= 0.0)
        return trajectory.pos_deg.front();
    if (t_s >= duration)
        return trajectory.pos_deg.back();
    const double u = t_s / trajectory.dt_s;
    // Clamp the index explicitly: t_s < duration does NOT guarantee
    // (size_t)(t_s/dt) < n-1 — floating-point rounding can push the
    // quotient to exactly n-1 for a t one ulp below the duration, and
    // pos_deg[i+1] would then read past the end (2026-07-31 review,
    // finding 1).
    const std::size_t i =
        std::min(static_cast<std::size_t>(u), n - 2);
    const double frac = std::min(u - static_cast<double>(i), 1.0);
    return trajectory.pos_deg[i] +
           frac * (trajectory.pos_deg[i + 1] - trajectory.pos_deg[i]);
}

double TrajectoryDurationS(const Trajectory& trajectory)
{
    return static_cast<double>(trajectory.pos_deg.size() - 1) * trajectory.dt_s;
}
