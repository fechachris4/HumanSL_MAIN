#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include "StartState.h"

int main() {
    const char* path = "test_start_state_tmp.csv";
    {
        std::ofstream csv(path);
        csv << "time_s,dt_s,meas_j1,extra,meas_j2,meas_j3,meas_j4,"
               "meas_j5,meas_j6,meas_j7\n";
        // Valid full-width rows (10 columns each, matching header)
        csv << "0.001,0.002,10,99,20,30,40,50,60,70\n";
        csv << "0.003,0.002,11,99,21,31,41,51,61,71\n";
        // Torn row (only 4 fields) must be ignored
        csv << "0.005,0.002,12,99";
        // Row with malformed field ("12abc") must be ignored
        csv << "\n0.007,0.002,12abc,99,22,32,42,52,62,72\n";
    }
    std::string error;
    const auto q = ReadLatestMeasuredQ(path, error);
    // Should return the last valid row (11.0 degrees, 71.0 degrees)
    // because torn row and malformed field row are both skipped
    assert(q.has_value() && error.empty());
    assert(std::abs((*q)[0] - 11.0 * M_PI / 180.0) < 1e-12);
    assert(std::abs((*q)[6] - 71.0 * M_PI / 180.0) < 1e-12);

    const auto missing = ReadLatestMeasuredQ("does_not_exist.csv", error);
    assert(!missing.has_value() && !error.empty());
    std::remove(path);
    return 0;
}
