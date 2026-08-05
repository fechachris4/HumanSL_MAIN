#include <cassert>
#include <cmath>
#include <cstdio>
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
    return 0;
}
