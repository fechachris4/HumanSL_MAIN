#include "StartState.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>

namespace {
std::vector<std::string> SplitCsv(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) fields.push_back(field);
    return fields;
}
}  // namespace

std::optional<Eigen::Matrix<double, 7, 1>> ReadLatestMeasuredQ(
    const std::string& csv_path, std::string& error) {
    error.clear();
    std::ifstream csv(csv_path);
    if (!csv) { error = "cannot open " + csv_path; return std::nullopt; }

    std::string line;
    if (!std::getline(csv, line)) { error = "empty file"; return std::nullopt; }
    const auto header = SplitCsv(line);
    std::array<int, 7> column{};
    for (int j = 0; j < 7; ++j) {
        const std::string name = "meas_j" + std::to_string(j + 1);
        const auto it = std::find(header.begin(), header.end(), name);
        if (it == header.end()) { error = "missing column " + name; return std::nullopt; }
        column[j] = static_cast<int>(it - header.begin());
    }

    std::optional<Eigen::Matrix<double, 7, 1>> latest;
    while (std::getline(csv, line)) {
        const auto fields = SplitCsv(line);
        // Accept only full-width rows matching header (torn rows are skipped silently)
        if (static_cast<int>(fields.size()) != static_cast<int>(header.size())) continue;
        Eigen::Matrix<double, 7, 1> q_deg;
        bool valid = true;
        for (int j = 0; j < 7 && valid; ++j) {
            try {
                size_t pos = 0;
                const double val = std::stod(fields[column[j]], &pos);
                // Reject if entire field was not consumed (e.g., "12abc" parses 12 but pos != len)
                if (pos != fields[column[j]].size()) valid = false;
                else { q_deg[j] = val; }
            }
            catch (const std::exception&) { valid = false; }
            if (valid && !std::isfinite(q_deg[j])) valid = false;
        }
        if (valid) latest = q_deg * (M_PI / 180.0);
    }
    if (!latest) error = "no complete data row";
    return latest;
}
