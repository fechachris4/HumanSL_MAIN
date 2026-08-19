#pragma once
#include <optional>
#include <string>
#include <Eigen/Dense>

struct StartStateResult {
    Eigen::Matrix<double, 7, 1> q_rad;  // Kortex order
    std::string error;                  // set when read failed
};

// Reads the header to locate meas_j1..meas_j7 by NAME (column positions
// are not stable across log_format revisions), then returns the last
// complete data row converted to radians, RAW — still in Kortex's own
// convention. Canonicalisation for the planner happens once, in
// BridgeMain, whatever transport the start state arrived by. nullopt
// value + error on any missing column, short row, or non-finite value.
std::optional<Eigen::Matrix<double, 7, 1>> ReadLatestMeasuredQ(
    const std::string& csv_path, std::string& error);

// The planner's joint canonicalisation: wrap one angle to its principal
// value in (-pi, pi]. GPMP2 treats configuration space as flat signed
// radians with no revolution tracking, so every start state entering the
// planner passes through this exactly once, in BridgeMain, after the
// transport (CSV or --start-deg) has produced raw radians. Kortex's own
// [0, 360) convention is untouched: this converts at the planner boundary,
// it does not redefine the measurement.
double WrapToPrincipalRad(double angle_rad);

// Newest <filename_prefix>*.csv by modification time under
// <runs_root>/<subdir>/ (the controller's dated-directory layout).
// filename_prefix defaults to "loop_log" (every run, either arm); pass
// "loop_log_right" or "loop_log_left" (config::ArmConfig::log_prefix,
// Christian_control/control/Config.h) to find only that arm's
// own logs — a right-arm log is not evidence of the left arm's state, or
// vice versa. nullopt + error when the root is missing or holds no
// matching file; the error names the prefix searched for.
std::optional<std::string> FindLatestRunCsv(const std::string& runs_root,
                                            std::string& error,
                                            const std::string& filename_prefix = "loop_log");
