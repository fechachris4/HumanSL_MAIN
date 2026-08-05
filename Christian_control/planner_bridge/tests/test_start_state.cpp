// Test assertions must never no-op under a Release (NDEBUG) configure.
#undef NDEBUG

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include "StartState.h"

int main() {
    const char* path = "test_start_state_tmp.csv";
    {
        std::ofstream csv(path);
        // Extended header with trailing columns after meas_j7 (simulating real log)
        csv << "time_s,dt_s,meas_j1,extra,meas_j2,meas_j3,meas_j4,"
               "meas_j5,meas_j6,meas_j7,vel_j1,torque_j1,fault_j1\n";
        // Valid full-width rows (13 columns each, matching header)
        csv << "0.001,0.002,10,99,20,30,40,50,60,70,100,200,300\n";
        csv << "0.003,0.002,11,99,21,31,41,51,61,71,101,201,301\n";
        // Partial-width row (10 fields): reaches meas_j7 column (index 9) but
        // narrower than full 13-column header; must be ignored (new width guard exercised)
        csv << "0.005,0.002,12,99,22,32,42,52,62,72";
        // Row with malformed field ("12abc"); full-width (13 columns) but invalid parse
        csv << "\n0.007,0.002,12abc,99,23,33,43,53,63,73,103,203,303\n";
    }
    std::string error;
    const auto q = ReadLatestMeasuredQ(path, error);
    // Should return the last valid row (11.0 degrees, 71.0 degrees)
    // Partial-width row (10 fields) is skipped by new strict width check.
    // Malformed field row is skipped by whole-field parse check.
    assert(q.has_value() && error.empty());
    assert(std::abs((*q)[0] - 11.0 * M_PI / 180.0) < 1e-12);
    assert(std::abs((*q)[6] - 71.0 * M_PI / 180.0) < 1e-12);

    const auto missing = ReadLatestMeasuredQ("does_not_exist.csv", error);
    assert(!missing.has_value() && !error.empty());
    std::remove(path);

    // FindLatestRunCsv: newest dated subdir wins by mtime.
    std::filesystem::create_directories("tsr_tmp/2026-08-04");
    std::filesystem::create_directories("tsr_tmp/2026-08-05");
    { std::ofstream("tsr_tmp/2026-08-04/loop_log_a.csv") << "x\n"; }
    { std::ofstream("tsr_tmp/2026-08-05/loop_log_b.csv") << "x\n"; }
    // Ensure strictly increasing mtimes regardless of filesystem resolution.
    std::filesystem::last_write_time("tsr_tmp/2026-08-04/loop_log_a.csv",
        std::filesystem::file_time_type::clock::now() - std::chrono::hours(1));
    std::string find_error;
    const auto latest = FindLatestRunCsv("tsr_tmp", find_error);
    assert(latest.has_value() && find_error.empty());
    assert(latest->find("loop_log_b.csv") != std::string::npos);
    { std::ofstream("tsr_tmp/2026-08-05/notes.txt") << "x\n"; }  // non-matching ignored
    assert(FindLatestRunCsv("tsr_tmp", find_error) == latest);
    assert(!FindLatestRunCsv("tsr_missing", find_error).has_value() && !find_error.empty());
    std::filesystem::remove_all("tsr_tmp");
    return 0;
}
