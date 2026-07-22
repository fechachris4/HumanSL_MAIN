//
// replay_controller — the decision-14 replay harness. Feeds a recorded
// baseline CSV through the NEW pipeline (ResolvedRate -> per-joint clamp ->
// PositionIntegration) and compares its outputs against what the recorded
// loop actually commanded. Needs the bundled Pinocchio (Linux hardware
// machine only).
//
//   ./replay_controller <run_....csv>
//
// Reconstruction: row n's inputs are row n-1's raw feedback (measraw_j*),
// row n's target (pd_*) and row n's measured dt (dt_s, re-clamped exactly
// as the loop clamps it). Row 0 cannot be replayed (its input was the
// holding frame's reply, which is not logged) — the integrator is anchored
// at cmd_row0 instead and replay starts at row 1.
//
// Expected agreement is bounded by the CSV's ~6 significant digits, not
// machine epsilon: quantized inputs (measraw ~1e-3 deg) propagate through
// Kp·J⁺. The PASS thresholds below are set an order of magnitude above
// that floor; paste the printed stats into review either way.
//
// Gains/limits come from app/Config.h — they MUST match the values the
// baseline was recorded with (printed at startup for cross-checking).
//

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "actuation/PositionIntegration.h"
#include "app/Config.h"
#include "control/ResolvedRate.h"
#include "math/Dls.h"
#include "Dynamics.h"

namespace
{
    constexpr double kDegToRad = M_PI / 180.0;
    constexpr double kRadToDeg = 180.0 / M_PI;

    struct Row {
        double dt_s = 0.0;
        Eigen::Vector3d pd = Eigen::Vector3d::Zero();
        JointVector cmd{};
        JointVector cmdvel{};
        JointVector measraw{};
    };

    std::vector<Row> LoadCsv(const std::string& path)
    {
        std::ifstream in(path);
        if (!in)
        {
            std::cerr << "error: cannot open " << path << "\n";
            std::exit(2);
        }
        std::string line;
        // Skip any '#' config-preamble lines (added by a later migration
        // step); the first non-'#' line is the column header.
        do
        {
            if (!std::getline(in, line))
            {
                std::cerr << "error: " << path << " has no header row\n";
                std::exit(2);
            }
        } while (!line.empty() && line[0] == '#');

        std::map<std::string, int> col;
        {
            std::istringstream header(line);
            std::string name;
            int index = 0;
            while (std::getline(header, name, ','))
                col[name] = index++;
        }
        for (const char* required : {"dt_s", "pd_x", "cmd_j1", "cmdvel_j1", "measraw_j1"})
            if (col.find(required) == col.end())
            {
                std::cerr << "error: " << path << " is missing column " << required
                          << " — not a controller run log?\n";
                std::exit(2);
            }

        std::vector<Row> rows;
        while (std::getline(in, line))
        {
            if (line.empty())
                continue;
            std::vector<double> v;
            std::istringstream fields(line);
            std::string field;
            while (std::getline(fields, field, ','))
                v.push_back(std::stod(field));
            Row r;
            r.dt_s = v[col["dt_s"]];
            r.pd = {v[col["pd_x"]], v[col["pd_y"]], v[col["pd_z"]]};
            for (int j = 0; j < 7; ++j)
            {
                r.cmd[j] = v[col["cmd_j" + std::to_string(j + 1)]];
                r.cmdvel[j] = v[col["cmdvel_j" + std::to_string(j + 1)]];
                r.measraw[j] = v[col["measraw_j" + std::to_string(j + 1)]];
            }
            rows.push_back(r);
        }
        return rows;
    }
} // namespace

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "usage: replay_controller <run_....csv>\n";
        return 2;
    }
    const std::vector<Row> rows = LoadCsv(argv[1]);
    if (rows.size() < 3)
    {
        std::cerr << "error: only " << rows.size() << " data rows — nothing to replay\n";
        return 2;
    }

    std::cout << "replaying " << rows.size() << " rows from " << argv[1] << "\n"
              << "config at replay (MUST match the recording): Kp "
              << config::kKpCartesian << " 1/s, lambda " << config::kDlsLambda
              << ", dt_nominal " << config::kControlDtS << " s, clip "
              << config::kQdotLimitDegS[0] << "/" << config::kQdotLimitDegS[4]
              << " deg/s\n";

    Dynamics dynamics(GEN3_URDF_PATH);
    TargetStore targets;
    ResolvedRate controller(dynamics, targets, config::kKpCartesian,
                            config::kDlsLambda, config::kArrivalToleranceM,
                            config::kEndEffectorFrame);
    PositionIntegration actuation;

    // Anchor: the integrator state after cycle 0 is exactly cmd_row0.
    RobotState anchor;
    anchor.qdot_rad_s.setZero();
    for (int j = 0; j < 7; ++j)
        anchor.q_rad[j] = rows[0].cmd[j] * kDegToRad;
    actuation.Prepare(anchor);
    controller.Reset(anchor); // stored target is overwritten from pd below

    JointVector max_dvel{};
    JointVector sum_dvel{};
    JointVector max_dstep{};
    Eigen::Vector3d prev_pd = rows[0].pd - Eigen::Vector3d::Ones(); // force first Store
    JointVector setpoints{};
    JointVector velocity{};

    for (std::size_t n = 1; n < rows.size(); ++n)
    {
        const Row& input = rows[n - 1]; // cycle n consumes cycle n-1's reply
        const Row& expect = rows[n];

        RobotState state;
        state.qdot_rad_s.setZero();
        for (int j = 0; j < 7; ++j)
            state.q_rad[j] = input.measraw[j] * kDegToRad;

        if ((expect.pd - prev_pd).norm() > 0.0)
        {
            targets.Store(expect.pd);
            prev_pd = expect.pd;
        }

        const double dt_s = ClampedCycleDt(expect.dt_s, config::kControlDtS);

        ControllerStatus status;
        const Eigen::Matrix<double, 7, 1> qdot_raw =
            controller.DesiredVelocity(state, dt_s, status);
        Eigen::Matrix<double, 7, 1> qdot_clamped;
        for (int j = 0; j < 7; ++j)
            qdot_clamped[j] = std::clamp(qdot_raw[j] * kRadToDeg,
                                         -config::kQdotLimitDegS[j],
                                         config::kQdotLimitDegS[j]) *
                              kDegToRad;
        actuation.Apply(qdot_clamped, dt_s, setpoints, velocity);

        for (int j = 0; j < 7; ++j)
        {
            const double dvel = std::abs(velocity[j] - expect.cmdvel[j]);
            max_dvel[j] = std::max(max_dvel[j], dvel);
            sum_dvel[j] += dvel;
            const double predicted_step = velocity[j] * dt_s;
            const double actual_step = expect.cmd[j] - rows[n - 1].cmd[j];
            max_dstep[j] = std::max(max_dstep[j], std::abs(predicted_step - actual_step));
        }
    }

    const std::size_t cycles = rows.size() - 1;
    std::cout << "\nper-joint agreement over " << cycles << " replayed cycles:\n"
              << "joint   max|dvel| deg/s   mean|dvel| deg/s   max|dstep| deg   |drift| deg\n";
    double worst_dvel = 0.0;
    double worst_drift = 0.0;
    for (int j = 0; j < 7; ++j)
    {
        const double drift = std::abs(setpoints[j] - rows.back().cmd[j]);
        worst_dvel = std::max(worst_dvel, max_dvel[j]);
        worst_drift = std::max(worst_drift, drift);
        std::cout << "  " << (j + 1) << "     " << max_dvel[j] << "     "
                  << sum_dvel[j] / static_cast<double>(cycles) << "     "
                  << max_dstep[j] << "     " << drift << "\n";
    }

    // Thresholds: an order of magnitude above the CSV-quantization floor.
    const bool pass = worst_dvel < 0.05 && worst_drift < 0.5;
    std::cout << "\n" << (pass ? "PASS" : "FAIL")
              << " (thresholds: max|dvel| < 0.05 deg/s, |drift| < 0.5 deg; "
                 "agreement floor is CSV precision, ~6 significant digits)\n";
    return pass ? 0 : 1;
}
