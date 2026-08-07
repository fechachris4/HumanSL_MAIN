//
// expand_run_poses — turn a controller run log into a Cartesian pose trace.
//
// The run CSV records joint angles faithfully but its Cartesian columns
// (pd_*, p_*, quat_*) are NaN on every row: the joint-trajectory motion path
// computes no tool pose, because Controller.cpp takes the joint branch and
// returns before any FK. That is deliberate — the control loop carries no
// model — so the pose is recovered HERE instead, offline, by running the same
// Pinocchio kinematics the pipeline uses on the cmd_j*/meas_j* columns that
// ARE recorded.
//
// This tool owns geometry and row semantics and nothing else. It computes no
// errors and makes no judgements: those are metrics, and they live in Python
// (scripts/tracking_metrics.py) so there is one definition of each.
//
// Reads the URDF and one CSV. No Kortex, no connection, no motion — and it is
// deliberately not linked against Kortex, so it cannot command anything.
//
//   expand_run_poses --in <run.csv> [--arm right|left] [--out <trace.csv>]
//
// --arm defaults to the preamble's `# arm = <name> (<ip>)` line. An explicit
// --arm that disagrees with the preamble is an error, not an override: the
// wrong arm's kinematics would produce a plausible-looking wrong answer.
//

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "Config.h"
#include "Dynamics.h"
#include "Kinematics.h"

namespace {

constexpr double kDegToRad = M_PI / 180.0;
const double kNaN = std::numeric_limits<double>::quiet_NaN();

std::vector<std::string> SplitCsv(const std::string& line) {
    std::vector<std::string> fields;
    std::string field;
    std::istringstream stream(line);
    while (std::getline(stream, field, ',')) fields.push_back(field);
    return fields;
}

// Full-field parse: "1.2xyz" and "" are rejected rather than truncated.
// Blank and nan spellings become NaN, which is what a dead column holds.
double ParseCell(const std::string& text) {
    if (text.empty()) return kNaN;
    try {
        std::size_t consumed = 0;
        const double value = std::stod(text, &consumed);
        return consumed == text.size() ? value : kNaN;
    } catch (const std::exception&) {
        return kNaN;
    }
}

struct RunLog {
    std::vector<std::string> preamble;                        // '#' lines, verbatim
    std::unordered_map<std::string, std::size_t> column;      // name -> index
    std::vector<std::vector<double>> rows;
    std::string arm_name;                                     // from the preamble
    int log_format = 0;
};

RunLog ReadRunLog(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("cannot open " + path);

    RunLog log;
    std::string line;
    std::vector<std::string> header;
    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;
        if (line[0] == '#') {
            log.preamble.push_back(line);
            // "# arm = right (192.168.1.10)" and "# log_format = 9"
            const std::size_t eq = line.find('=');
            if (eq != std::string::npos) {
                std::string key = line.substr(1, eq - 1);
                std::string value = line.substr(eq + 1);
                const auto trim = [](std::string& s) {
                    const std::size_t a = s.find_first_not_of(" \t");
                    const std::size_t b = s.find_last_not_of(" \t");
                    s = (a == std::string::npos) ? std::string() : s.substr(a, b - a + 1);
                };
                trim(key);
                trim(value);
                // Preamble values carry a trailing annotation — "right
                // (192.168.1.10)", "9 (compiled)" — so take the first token.
                // ParseCell is deliberately strict about whole fields and
                // would reject these, then cast NaN to a garbage int.
                const std::string first = value.substr(0, value.find(' '));
                if (key == "arm") log.arm_name = first;
                if (key == "log_format") {
                    const double parsed = ParseCell(first);
                    if (std::isfinite(parsed)) log.log_format = static_cast<int>(parsed);
                }
            }
            continue;
        }
        if (header.empty()) {
            header = SplitCsv(line);
            for (std::size_t i = 0; i < header.size(); ++i) log.column[header[i]] = i;
            continue;
        }
        const std::vector<std::string> fields = SplitCsv(line);
        // A torn final row (the controller was still writing) has fewer
        // fields than the header. Drop it rather than shifting every column.
        if (fields.size() != header.size()) continue;
        std::vector<double> row(fields.size());
        for (std::size_t i = 0; i < fields.size(); ++i) row[i] = ParseCell(fields[i]);
        log.rows.push_back(std::move(row));
    }
    if (header.empty()) throw std::runtime_error("no CSV header in " + path);
    if (log.rows.empty()) throw std::runtime_error("no data rows in " + path);
    return log;
}

double Cell(const RunLog& log, const std::vector<double>& row, const std::string& name) {
    const auto it = log.column.find(name);
    return it == log.column.end() ? kNaN : row[it->second];
}

bool HasColumn(const RunLog& log, const std::string& name) {
    return log.column.find(name) != log.column.end();
}

// Joint vector in RADIANS from seven degree columns named <prefix>1..7.
// Returns false when any is missing or non-finite, so a dead column produces
// no pose rather than a pose built from NaN.
bool JointsRad(const RunLog& log, const std::vector<double>& row, const char* prefix,
               Eigen::Matrix<double, 7, 1>& q_rad) {
    for (int j = 0; j < 7; ++j) {
        const double deg = Cell(log, row, prefix + std::to_string(j + 1));
        if (!std::isfinite(deg)) return false;
        q_rad(j) = deg * kDegToRad;
    }
    return true;
}

// Hamilton, hemisphere-fixed to w >= 0 so successive rows do not flip sign
// on the same physical orientation (q and -q are the same rotation).
Eigen::Quaterniond QuaternionOf(const Eigen::Matrix3d& rotation) {
    Eigen::Quaterniond q(rotation);
    q.normalize();
    if (q.w() < 0.0) q.coeffs() *= -1.0;
    return q;
}

void WriteVec(std::ostream& out, const Eigen::Vector3d& v) {
    out << "," << v.x() << "," << v.y() << "," << v.z();
}
void WriteQuat(std::ostream& out, const Eigen::Quaterniond& q) {
    out << "," << q.x() << "," << q.y() << "," << q.z() << "," << q.w();
}
void WriteNaNs(std::ostream& out, int n) {
    for (int i = 0; i < n; ++i) out << ",nan";
}

}  // namespace

int main(int argc, char** argv) {
    std::string in_path, out_path, arm_flag;
    for (int i = 1; i < argc; ++i) {
        const std::string flag = argv[i];
        const auto next = [&]() -> std::string {
            if (i + 1 >= argc) throw std::invalid_argument("missing value after " + flag);
            return argv[++i];
        };
        try {
            if (flag == "--in") in_path = next();
            else if (flag == "--out") out_path = next();
            else if (flag == "--arm") arm_flag = next();
            else {
                std::cerr << "unrecognized flag: " << flag << "\n"
                          << "usage: expand_run_poses --in <run.csv> "
                             "[--arm right|left] [--out <trace.csv>]\n";
                return 2;
            }
        } catch (const std::exception& error) {
            std::cerr << "error: " << error.what() << "\n";
            return 2;
        }
    }
    if (in_path.empty()) {
        std::cerr << "usage: expand_run_poses --in <run.csv> "
                     "[--arm right|left] [--out <trace.csv>]\n";
        return 2;
    }

    RunLog log;
    try {
        log = ReadRunLog(in_path);
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << "\n";
        return 1;
    }

    // The preamble is the authority on which arm produced the log. A flag
    // that contradicts it is refused: running the left arm's kinematics on a
    // right-arm log yields a pose that looks entirely reasonable and is wrong
    // by the whole mounting transform.
    std::string arm_name = log.arm_name;
    if (!arm_flag.empty()) {
        if (arm_flag != "right" && arm_flag != "left") {
            std::cerr << "error: --arm must be 'right' or 'left'\n";
            return 2;
        }
        if (!arm_name.empty() && arm_name != arm_flag) {
            std::cerr << "error: --arm " << arm_flag << " contradicts the log's own "
                      << "preamble (# arm = " << arm_name << "). Refusing rather than "
                      << "silently applying the wrong arm's mounting transform.\n";
            return 1;
        }
        arm_name = arm_flag;
    }
    if (arm_name.empty()) {
        std::cerr << "error: no '# arm =' line in the preamble and no --arm given\n";
        return 1;
    }
    const bool left_arm = arm_name == "left";
    const Arm arm = left_arm ? Arm::kLeft : Arm::kRight;
    const config::ArmConfig& arm_config =
        left_arm ? config::kLeftArmConfig : config::kRightArmConfig;

    const bool has_reference = HasColumn(log, "ref_j1");
    if (!has_reference) {
        std::cerr << "note: this log has no ref_j* columns (log_format " << log.log_format
                  << "), so the reference the controller was tracking is not available.\n"
                  << "      It is not recoverable offline either: the tracking law is\n"
                  << "      qdot = qdot_ref + Kp*wrap(q_ref - q_meas) — one logged\n"
                  << "      equation, two unknowns per joint. Measured and commanded\n"
                  << "      poses below are complete; reference poses are omitted.\n";
    }

    if (out_path.empty()) {
        const std::size_t dot = in_path.find_last_of('.');
        out_path = (dot == std::string::npos ? in_path : in_path.substr(0, dot)) +
                   "_poses.csv";
    }

    Dynamics dynamics(GEN3_DUAL_URDF_PATH);
    DualArmKinematics model(dynamics, arm, arm_config.other_arm_nominal_rad,
                            config::kRightBaseFrame, config::kRightEndEffectorFrame);
    KinematicsWorkspace workspace(dynamics);

    std::ofstream out(out_path);
    if (!out) {
        std::cerr << "error: cannot write " << out_path << "\n";
        return 1;
    }
    out << std::setprecision(9);

    for (const std::string& line : log.preamble) out << "# src." << line.substr(1) << "\n";
    out << "# pose_trace_format = 1\n";
    out << "# source_csv = " << in_path << "\n";
    out << "# urdf = " << GEN3_DUAL_URDF_PATH << "\n";
    out << "# arm = " << arm_name << "\n";
    out << "# reference_available = " << (has_reference ? "true" : "false") << "\n";
    out << "# frames: *_x/y/z are " << arm_config.base_frame
        << "; *_mx/my/mz are mount\n";

    out << "cycle,row_kind,time_s,t_send_s,t_recv_s,t_state_s"
        << ",joint_follow_error_deg,traj_activated,traj_rejected,traj_complete,refresh_ok";
    for (const char* p : {"meas_j", "cmd_j"})
        for (int j = 1; j <= 7; ++j) out << "," << p << j;
    if (has_reference)
        for (int j = 1; j <= 7; ++j) out << ",ref_j" << j;
    out << ",meas_x,meas_y,meas_z,meas_qx,meas_qy,meas_qz,meas_qw"
        << ",cmd_x,cmd_y,cmd_z,cmd_qx,cmd_qy,cmd_qz,cmd_qw";
    if (has_reference)
        out << ",ref_x,ref_y,ref_z,ref_qx,ref_qy,ref_qz,ref_qw";
    out << ",meas_mx,meas_my,meas_mz,cmd_mx,cmd_my,cmd_mz";
    if (has_reference) out << ",ref_mx,ref_my,ref_mz";
    out << "\n";

    std::size_t dropped_duplicate = 0, hold_rows = 0, emitted = 0;
    for (std::size_t r = 0; r < log.rows.size(); ++r) {
        const std::vector<double>& row = log.rows[r];
        const double cycle = Cell(log, row, "cycle");
        const double refresh_ok = Cell(log, row, "refresh_ok");
        const double time_s = Cell(log, row, "time_s");

        // The exception paths in Runner.cpp push a COPY of the previous
        // sample with refresh_ok flipped to 0. Identify it by that copy —
        // same cycle AND same time_s as the row before — and drop it. A
        // refresh_ok == 0 row that is NOT a copy is a genuine failed
        // exchange: keep it, tagged, and say so.
        std::string row_kind = "control";
        if (cycle == 0.0) {
            row_kind = "hold";
            ++hold_rows;
        } else if (cycle == 1.0) {
            // Its input feedback came from the un-logged seed exchange, so
            // the N-1 pairing below has nothing to point at.
            row_kind = "control_unpaired";
        }
        if (r + 1 == log.rows.size() && refresh_ok == 0.0 && r > 0) {
            const std::vector<double>& prev = log.rows[r - 1];
            if (Cell(log, prev, "cycle") == cycle && Cell(log, prev, "time_s") == time_s) {
                ++dropped_duplicate;
                continue;
            }
            row_kind = "stale";
            std::cerr << "warning: final row has refresh_ok = 0 but is not a copy of "
                      << "the previous row — emitting it as row_kind=stale rather than "
                      << "assuming it is the known exception-path duplicate\n";
        }

        // The controller's inputs in row r derive from the feedback received
        // in row r-1 (Hardware.h documents the cross-exchange semantics), so
        // the state used at row r was sampled at the PREVIOUS t_recv_s.
        // Computed once here so no downstream consumer re-derives it.
        const double t_state_s =
            r == 0 ? time_s : Cell(log, log.rows[r - 1], "t_recv_s");

        out << cycle << "," << row_kind << "," << time_s << "," << Cell(log, row, "t_send_s")
            << "," << Cell(log, row, "t_recv_s") << "," << t_state_s
            << "," << Cell(log, row, "joint_follow_error_deg")
            << "," << Cell(log, row, "traj_activated")
            << "," << Cell(log, row, "traj_rejected")
            << "," << Cell(log, row, "traj_complete") << "," << refresh_ok;

        Eigen::Matrix<double, 7, 1> q_meas, q_cmd, q_ref;
        const bool have_meas = JointsRad(log, row, "meas_j", q_meas);
        const bool have_cmd = JointsRad(log, row, "cmd_j", q_cmd);
        const bool have_ref = has_reference && JointsRad(log, row, "ref_j", q_ref);

        for (int j = 0; j < 7; ++j) out << "," << Cell(log, row, "meas_j" + std::to_string(j + 1));
        for (int j = 0; j < 7; ++j) out << "," << Cell(log, row, "cmd_j" + std::to_string(j + 1));
        if (has_reference)
            for (int j = 0; j < 7; ++j) out << "," << Cell(log, row, "ref_j" + std::to_string(j + 1));

        // Base-frame pose + quaternion for each of measured / commanded /
        // reference, then the mount-frame positions.
        Eigen::Vector3d p_meas_base = Eigen::Vector3d::Constant(kNaN);
        Eigen::Vector3d p_cmd_base = Eigen::Vector3d::Constant(kNaN);
        Eigen::Vector3d p_ref_base = Eigen::Vector3d::Constant(kNaN);

        const auto emit_pose = [&](bool have, const Eigen::Matrix<double, 7, 1>& q,
                                   Eigen::Vector3d& position_out) {
            if (!have) {
                WriteNaNs(out, 7);
                return;
            }
            const PoseJacobian pose = model.ControlledPoseAndJacobian(q, workspace);
            position_out = pose.position;
            WriteVec(out, pose.position);
            WriteQuat(out, QuaternionOf(pose.rotation));
        };
        emit_pose(have_meas, q_meas, p_meas_base);
        emit_pose(have_cmd, q_cmd, p_cmd_base);
        if (has_reference) emit_pose(have_ref, q_ref, p_ref_base);

        const auto emit_mount = [&](bool have, const Eigen::Vector3d& p_base) {
            if (!have) {
                WriteNaNs(out, 3);
                return;
            }
            WriteVec(out, model.PointBaseToMount(arm, p_base));
        };
        emit_mount(have_meas, p_meas_base);
        emit_mount(have_cmd, p_cmd_base);
        if (has_reference) emit_mount(have_ref, p_ref_base);

        out << "\n";
        ++emitted;
    }

    std::printf("arm %s, %zu rows in -> %zu emitted (%zu takeover-hold, %zu duplicate "
                "final row dropped)\n",
                arm_name.c_str(), log.rows.size(), emitted, hold_rows, dropped_duplicate);
    std::printf("pose trace -> %s\n", out_path.c_str());
    return 0;
}
