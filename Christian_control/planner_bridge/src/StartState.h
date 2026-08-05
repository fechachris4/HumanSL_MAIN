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
// complete data row converted to radians. nullopt value + error on any
// missing column, short row, or non-finite value.
std::optional<Eigen::Matrix<double, 7, 1>> ReadLatestMeasuredQ(
    const std::string& csv_path, std::string& error);

// Newest loop_log*.csv by modification time under <runs_root>/<subdir>/
// (the controller's dated-directory layout). nullopt + error when the
// root is missing or holds no matching file.
std::optional<std::string> FindLatestRunCsv(const std::string& runs_root,
                                            std::string& error);
