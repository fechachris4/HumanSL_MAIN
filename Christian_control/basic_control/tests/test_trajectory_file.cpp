//
// Hardware-free tests for the trajectory-file contract
// (Trajectory.h): loading, structural rejection, motion-limit
// validation and time-indexed sampling. No robot, no Pinocchio.
// Returns nonzero on the first failure.
//

#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "Trajectory.h"

namespace
{
    int failures = 0;

    void Check(bool ok, const std::string& what)
    {
        if (!ok) {
            std::cout << "FAIL: " << what << "\n";
            ++failures;
        }
    }

    // A contract-conformant file: joint 2 moves `amplitude_deg` over
    // `duration_s` on a quintic (minimum-jerk) profile — rest at both ends,
    // analytic velocity consistent with the positions.
    std::string MakeCsv(double amplitude_deg = 10.0, double duration_s = 2.0,
                        double dt = 0.001, double base_deg = 20.0)
    {
        std::ostringstream out;
        out << std::setprecision(10);
        out << "# trajectory_format = 1\n";
        out << "# source = test\n";
        out << "t_s";
        for (int j = 1; j <= 7; ++j) out << ",q" << j << "_deg";
        for (int j = 1; j <= 7; ++j) out << ",qd" << j << "_degs";
        out << "\n";
        const int n = static_cast<int>(duration_s / dt) + 1;
        for (int i = 0; i < n; ++i)
        {
            const double t = i * dt;
            const double s = t / duration_s;
            const double shape = 10 * s * s * s - 15 * s * s * s * s +
                                 6 * s * s * s * s * s;
            const double dshape =
                (30 * s * s - 60 * s * s * s + 30 * s * s * s * s) / duration_s;
            out << t;
            for (int j = 1; j <= 7; ++j)
                out << "," << (j == 2 ? base_deg + amplitude_deg * shape
                                      : base_deg);
            for (int j = 1; j <= 7; ++j)
                out << "," << (j == 2 ? amplitude_deg * dshape : 0.0);
            out << "\n";
        }
        return out.str();
    }

    Trajectory LoadFrom(const std::string& text)
    {
        std::istringstream in(text);
        return LoadTrajectoryCsv(in, "test");
    }

    bool LoadRejects(const std::string& text, const std::string& what)
    {
        try
        {
            LoadFrom(text);
        }
        catch (const std::runtime_error&)
        {
            return true;
        }
        std::cout << "  (no exception for: " << what << ")\n";
        return false;
    }

    const JointVector kVelGate = {71.6, 71.6, 71.6, 71.6, 62.9, 62.9, 62.9};
    const JointVector kAccelGate = {57.3, 57.3, 57.3, 57.3, 573.0, 573.0, 573.0};
    const JointVector kPosGate = {0.0, 128.9, 0.0, 147.8, 0.0, 120.3, 0.0};

    void TestLoadValid()
    {
        const Trajectory t = LoadFrom(MakeCsv());
        Check(t.pos_deg.size() == 2001, "valid file: 2001 samples");
        Check(std::abs(t.dt_s - 0.001) < 1e-9, "valid file: dt = 1 ms");
        Check(t.metadata.at("trajectory_format") == "1", "metadata recorded");
        Check(t.metadata.at("source") == "test", "free metadata recorded");
        Check(std::abs(t.pos_deg.front()[1] - 20.0) < 1e-9, "start position");
        Check(std::abs(t.pos_deg.back()[1] - 30.0) < 1e-6, "end position");

        // CRLF line endings must load identically.
        std::string crlf = MakeCsv();
        std::string with_cr;
        for (char c : crlf)
        {
            if (c == '\n') with_cr += '\r';
            with_cr += c;
        }
        const Trajectory t2 = LoadFrom(with_cr);
        Check(t2.pos_deg.size() == t.pos_deg.size(), "CRLF file loads");
    }

    void TestLoadRejections()
    {
        std::string good = MakeCsv();

        Check(LoadRejects("t_s,q1_deg\n0,1\n", "no format marker"),
              "missing trajectory_format marker rejected");

        std::string wrong_version = good;
        wrong_version.replace(wrong_version.find("= 1"), 3, "= 9");
        Check(LoadRejects(wrong_version, "format 9"),
              "unknown trajectory_format rejected");

        std::string swapped = good;
        swapped.replace(swapped.find("q1_deg"), 6, "qd_wat");
        Check(LoadRejects(swapped, "renamed column"),
              "wrong header rejected (column names are the contract)");

        // A row with a missing field.
        std::string short_row = good;
        const auto last_comma = short_row.rfind(',');
        short_row.erase(last_comma); // cuts the final field of the last row
        Check(LoadRejects(short_row, "short row"), "short row rejected");

        // Non-numeric junk in a field.
        std::string junk = good;
        junk.replace(junk.rfind("0\n"), 1, "x");
        Check(LoadRejects(junk, "junk field"), "unparseable field rejected");

        // Non-finite value.
        std::string nan_row = good;
        nan_row.replace(nan_row.rfind(",0"), 2, ",nan");
        Check(LoadRejects(nan_row, "nan"), "non-finite value rejected");

        // Fewer than 2 rows.
        std::string one_row = MakeCsv(10.0, 2.0, 0.001);
        one_row.erase(one_row.find('\n', one_row.find("0,20")) + 1);
        Check(LoadRejects(one_row, "1 row"), "single-sample file rejected");

        // Non-uniform time grid.
        std::string skewed = MakeCsv();
        skewed.replace(skewed.find("\n0.001,"), 7, "\n0.0017,");
        Check(LoadRejects(skewed, "non-uniform t"),
              "non-uniform time grid rejected");

        // t_s not starting at zero.
        std::string late = MakeCsv();
        late.replace(late.find("\n0,"), 3, "\n0.5,");
        Check(LoadRejects(late, "t0 != 0"), "nonzero start time rejected");

        // Knot spacing outside the executable range: a 10 ms grid passes
        // every motion gate but would execute step-like velocity changes
        // at each knot — the loader must reject it (review finding 2).
        Check(LoadRejects(MakeCsv(10.0, 2.0, 0.01), "10 ms grid"),
              "coarse knot spacing rejected");
        // 2 ms is inside the contract.
        const Trajectory t2ms = LoadFrom(MakeCsv(10.0, 2.0, 0.002));
        Check(std::abs(t2ms.dt_s - 0.002) < 1e-9, "2 ms grid accepted");
    }

    void TestValidation()
    {
        TrajectorySummary summary;

        // The honest file passes and the summary describes it.
        {
            const Trajectory t = LoadFrom(MakeCsv());
            const auto violations =
                ValidateTrajectory(t, kVelGate, kAccelGate, kPosGate, summary);
            Check(violations.empty(), "honest file passes validation");
            Check(std::abs(summary.duration_s - 2.0) < 1e-6, "summary duration");
            Check(std::abs(summary.displacement_deg[1] - 10.0) < 1e-6,
                  "summary displacement, joint 2");
            // Quintic peak velocity = 1.875 * d / T = 9.375 deg/s.
            Check(std::abs(summary.peak_vel_deg_s[1] - 9.375) < 0.01,
                  "summary peak velocity");
            Check(summary.start_vel_deg_s < 0.01 && summary.end_vel_deg_s < 0.01,
                  "summary rest at both ends");
        }

        // Velocity above the gate: quintic peak 1.875*d/T; d=80 over 1 s
        // peaks at 150 deg/s > 71.6.
        {
            const Trajectory t = LoadFrom(MakeCsv(80.0, 1.0));
            const auto violations =
                ValidateTrajectory(t, kVelGate, kAccelGate, kPosGate, summary);
            Check(!violations.empty(), "over-speed file fails");
            bool mentions = false;
            for (const auto& v : violations)
                mentions = mentions || v.find("joint 2") != std::string::npos;
            Check(mentions, "over-speed violation names the joint");
        }

        // A velocity-column lie: positions say one thing, qd another.
        {
            Trajectory t = LoadFrom(MakeCsv());
            t.vel_deg_s[900][1] += 5.0;
            const auto violations =
                ValidateTrajectory(t, kVelGate, kAccelGate, kPosGate, summary);
            bool consistency = false;
            for (const auto& v : violations)
                consistency =
                    consistency || v.find("disagrees") != std::string::npos;
            Check(consistency, "velocity/position inconsistency detected");
        }

        // An acceleration spike (velocity step of 1 deg/s in 1 ms =
        // 1000 deg/s^2): must trip the accel gate on joint 2. Keep the
        // position column consistent so ONLY the accel rule fires.
        {
            Trajectory t = LoadFrom(MakeCsv());
            for (std::size_t i = 1000; i < t.vel_deg_s.size(); ++i)
            {
                t.vel_deg_s[i][1] += 1.0;
                t.pos_deg[i][1] += 1.0 * t.dt_s * (i - 999);
            }
            const auto violations =
                ValidateTrajectory(t, kVelGate, kAccelGate, kPosGate, summary);
            bool accel = false;
            for (const auto& v : violations)
                accel = accel || v.find("acceleration") != std::string::npos;
            Check(accel, "acceleration spike detected");
        }

        // Not at rest at the start.
        {
            Trajectory t = LoadFrom(MakeCsv());
            t.vel_deg_s.front()[3] = 0.5;
            const auto violations =
                ValidateTrajectory(t, kVelGate, kAccelGate, kPosGate, summary);
            bool rest = false;
            for (const auto& v : violations)
                rest = rest || v.find("first row is not at rest") !=
                                   std::string::npos;
            Check(rest, "moving first row detected");
        }

        // A raw position jump (teleport) must fail the step rule even if
        // someone also fakes plausible velocity columns around it.
        {
            Trajectory t = LoadFrom(MakeCsv());
            for (std::size_t i = 500; i < t.pos_deg.size(); ++i)
                t.pos_deg[i][4] += 2.0; // 2 deg in one 1 ms sample
            const auto violations =
                ValidateTrajectory(t, kVelGate, kAccelGate, kPosGate, summary);
            bool step = false;
            for (const auto& v : violations)
                step = step || v.find("per-sample position step") !=
                                   std::string::npos;
            Check(step, "position teleport detected");
        }

        // A bounded joint outside its model range: joint 4 held at -150
        // (range ±147.8) — the wrapped position gate must fire (review
        // finding 4).
        {
            Trajectory t = LoadFrom(MakeCsv());
            for (auto& q : t.pos_deg)
                q[3] = -150.0;
            const auto violations =
                ValidateTrajectory(t, kVelGate, kAccelGate, kPosGate, summary);
            bool pos = false;
            for (const auto& v : violations)
                pos = pos || (v.find("joint 4") != std::string::npos &&
                              v.find("position") != std::string::npos);
            Check(pos, "bounded-joint range violation detected");
            // The same value on a continuous joint (1) is fine.
            Trajectory t2 = LoadFrom(MakeCsv());
            for (auto& q : t2.pos_deg)
                q[0] = -150.0;
            Check(ValidateTrajectory(t2, kVelGate, kAccelGate, kPosGate, summary)
                      .empty(),
                  "continuous joint has no position gate");
        }

        // Positions that imply motion at the endpoints while the qd
        // column claims rest (review finding 8a).
        {
            Trajectory t = LoadFrom(MakeCsv());
            for (std::size_t i = 0; i < t.pos_deg.size(); ++i)
                t.pos_deg[i][2] += 2.0 * static_cast<double>(i) * t.dt_s;
            const auto violations =
                ValidateTrajectory(t, kVelGate, kAccelGate, kPosGate, summary);
            bool implied = false;
            for (const auto& v : violations)
                implied = implied || v.find("imply motion") != std::string::npos;
            Check(implied, "implied endpoint motion detected");
        }

        // Hand-built garbage must not underflow n - 1 (review finding 8c).
        {
            Trajectory empty;
            const auto violations =
                ValidateTrajectory(empty, kVelGate, kAccelGate, kPosGate, summary);
            Check(violations.size() == 1 &&
                      violations.front().find("not a loadable") !=
                          std::string::npos,
                  "empty trajectory reported, no underflow");
        }
    }

    void TestSampling()
    {
        const Trajectory t = LoadFrom(MakeCsv());
        Check(std::abs(TrajectoryDurationS(t) - 2.0) < 1e-9, "duration");
        Check((SamplePositionDeg(t, 0.0) - t.pos_deg.front()).norm() < 1e-12,
              "sample at t=0 is the first row");
        Check((SamplePositionDeg(t, 2.0) - t.pos_deg.back()).norm() < 1e-12,
              "sample at duration is the last row");
        Check((SamplePositionDeg(t, -1.0) - t.pos_deg.front()).norm() < 1e-12,
              "sample before 0 clamps to the first row");
        Check((SamplePositionDeg(t, 99.0) - t.pos_deg.back()).norm() < 1e-12,
              "sample past the end clamps to the last row");
        // Halfway between samples 1000 and 1001: exact linear midpoint.
        const Eigen::Matrix<double, 7, 1> mid = SamplePositionDeg(t, 1.0005);
        const Eigen::Matrix<double, 7, 1> expected =
            0.5 * (t.pos_deg[1000] + t.pos_deg[1001]);
        Check((mid - expected).norm() < 1e-9, "linear interpolation between samples");

        // One ulp below the duration: floating-point rounding can push
        // the index to n-1 — must clamp, not read past the end (review
        // finding 1). Sweep several durations/grids to hit the window.
        for (double dur : {2.0, 1.0, 0.9, 0.7, 0.3})
        {
            const Trajectory tt = LoadFrom(MakeCsv(5.0, dur, 0.001));
            const double just_below =
                std::nextafter(TrajectoryDurationS(tt), 0.0);
            const Eigen::Matrix<double, 7, 1> near_end =
                SamplePositionDeg(tt, just_below);
            Check((near_end - tt.pos_deg.back()).norm() < 1e-6,
                  "ulp-below-duration sample equals the last row (dur " +
                      std::to_string(dur) + ")");
        }
    }
} // namespace

int main()
{
    TestLoadValid();
    TestLoadRejections();
    TestValidation();
    TestSampling();

    if (failures == 0)
    {
        std::cout << "test_trajectory_file: all tests passed\n";
        return 0;
    }
    std::cout << "test_trajectory_file: " << failures << " FAILURES\n";
    return 1;
}
