#include "StartState.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
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

std::optional<std::string> FindLatestRunCsv(const std::string& runs_root,
                                            std::string& error) {
    error.clear();
    namespace fs = std::filesystem;
    std::error_code ec;
    if (!fs::is_directory(runs_root, ec)) {
        error = "runs root not found: " + runs_root;
        return std::nullopt;
    }
    std::optional<std::string> newest;
    fs::file_time_type newest_time{};
    // directory_iterator's operator++ (the range-for's implicit increment)
    // and the per-entry is_directory()/is_regular_file() checks below can
    // still throw std::filesystem::filesystem_error even though the
    // iterator was constructed with an error_code — e.g. an entry that
    // vanishes mid-scan (plausible here: a live controller is actively
    // writing this tree). Non-throwing overloads avoid the throw for the
    // status checks; the try/catch is a second layer so a throw from the
    // iterator increment itself degrades to "keep whatever was found so
    // far" instead of std::terminate.
    try {
        for (const auto& day : fs::directory_iterator(runs_root, ec)) {
            if (ec || !day.is_directory(ec) || ec) continue;
            for (const auto& entry : fs::directory_iterator(day.path(), ec)) {
                if (ec || !entry.is_regular_file(ec) || ec) continue;
                const std::string name = entry.path().filename().string();
                if (name.rfind("loop_log", 0) != 0 ||
                    entry.path().extension() != ".csv")
                    continue;
                const auto written = entry.last_write_time(ec);
                if (ec) continue;
                if (!newest || written > newest_time) {
                    newest = entry.path().string();
                    newest_time = written;
                }
            }
        }
    } catch (const fs::filesystem_error&) {
        // Entry vanished (or similar) mid-scan; fall through with whatever
        // was found before the throw.
    }
    if (!newest)
        error = "no loop_log*.csv under " + runs_root;
    return newest;
}
