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
// complete data row converted to radians and wrapped to (-pi, pi]. The
// wrap matters most for continuous joints: Kortex reports them in
// [0, 360) deg, and a measurement near the seam (e.g. 359.93) would
// otherwise seed the planner a full turn away from the signed angle it
// physically means. nullopt value + error on any missing column, short
// row, or non-finite value.
std::optional<Eigen::Matrix<double, 7, 1>> ReadLatestMeasuredQ(
    const std::string& csv_path, std::string& error);

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
