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

    // Real controller CSVs lead with a '#' config-echo preamble before the
    // header row; the reader must skip it rather than parsing it as header.
    const char* preamble_path = "test_start_state_preamble_tmp.csv";
    {
        std::ofstream csv(preamble_path);
        csv << "# controller run config — parsers skip '#' lines\n";
        csv << "# log_format = 8 (compiled)\n";
        csv << "time_s,dt_s,meas_j1,meas_j2,meas_j3,meas_j4,meas_j5,meas_j6,meas_j7\n";
        csv << "0.001,0.002,10,20,30,40,50,60,70\n";
        csv << "0.003,0.002,11,21,31,41,51,61,71\n";
    }
    std::string preamble_error;
    const auto q_preamble = ReadLatestMeasuredQ(preamble_path, preamble_error);
    assert(q_preamble.has_value() && preamble_error.empty());
    assert(std::abs((*q_preamble)[0] - 11.0 * M_PI / 180.0) < 1e-12);
    assert(std::abs((*q_preamble)[6] - 71.0 * M_PI / 180.0) < 1e-12);
    std::remove(preamble_path);

    // ReadLatestMeasuredQ is a pure reader: it returns Kortex's own raw
    // convention. Canonicalisation to GPMP2's signed principal values
    // happens once, in BridgeMain, whatever transport delivered the start
    // state (test_bridge_main.cpp pins that equivalence end to end); the
    // pieces WrapToPrincipalRad glues are pinned here.
    const char* wrap_path = "test_start_state_wrap_tmp.csv";
    {
        std::ofstream csv(wrap_path);
        csv << "time_s,dt_s,meas_j1,meas_j2,meas_j3,meas_j4,meas_j5,meas_j6,meas_j7\n";
        csv << "0.001,0.002,359.93,69.14,221.76,56.86,258.05,72.80,258.22\n";
    }
    std::string wrap_error;
    const auto q_wrap = ReadLatestMeasuredQ(wrap_path, wrap_error);
    assert(q_wrap.has_value() && wrap_error.empty());
    assert(std::abs((*q_wrap)[0] - 359.93 * M_PI / 180.0) < 1e-9 &&
           "the reader reports the raw measurement, unwrapped");
    // WrapToPrincipalRad: 359.93 deg means "0.07 deg short of zero", a
    // signed measurement is unchanged, and 221.76 wraps to -138.24.
    assert(std::abs(WrapToPrincipalRad((*q_wrap)[0]) -
                    (-0.07 * M_PI / 180.0)) < 1e-9);
    assert(std::abs(WrapToPrincipalRad(69.14 * M_PI / 180.0) -
                    69.14 * M_PI / 180.0) < 1e-9);
    assert(std::abs(WrapToPrincipalRad(221.76 * M_PI / 180.0) -
                    (-138.24 * M_PI / 180.0)) < 1e-9);
    std::remove(wrap_path);

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
