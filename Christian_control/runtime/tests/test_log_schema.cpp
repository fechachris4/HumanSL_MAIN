// The run-log CSV schema: format 13 records the sole world-Cartesian
// reference path and the coherent world measurement that generated qdot.

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Hardware.h"

namespace {
int failures = 0;

void Check(bool ok, const std::string& what) {
    if (!ok) {
        std::cout << "FAIL: " << what << "\n";
        ++failures;
    }
}

std::vector<std::string> Split(const std::string& line) {
    std::vector<std::string> fields;
    std::istringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) fields.push_back(field);
    return fields;
}

std::string OneLine(const std::string& text) {
    const std::size_t end = text.find('\n');
    return end == std::string::npos ? text : text.substr(0, end);
}
}  // namespace

int main() {
    std::ostringstream header_out;
    WriteCsvHeader(header_out);
    const std::vector<std::string> header = Split(OneLine(header_out.str()));

    LoopLogSample sample;
    sample.cart_traj_activated = true;
    sample.cart_traj_rejected = false;
    sample.cart_traj_complete = true;
    sample.cart_traj_cancelled = true;
    sample.cart_replan_requested = true;
    sample.cart_start_position_error_m = 0.0125;
    sample.cart_start_orientation_error_rad = 0.025;
    sample.cart_trajectory_id = 9;
    sample.cart_planner_vicon_sequence = 42;
    sample.cart_reference_time_s = 1.25;
    sample.cart_ref_position_world_m[0] = 1.1;
    sample.cart_ref_quat_world_xyzw[3] = 0.9;
    sample.cart_ref_linear_world_m_s[1] = 0.2;
    sample.cart_ref_angular_world_rad_s[2] = -0.3;
    sample.cart_measured_position_world_m[2] = 2.2;
    sample.cart_measured_quat_world_xyzw[0] = 0.1;
    sample.cart_measured_linear_world_m_s[0] = -0.4;
    sample.cart_measured_angular_world_rad_s[1] = 0.5;
    sample.world_fresh = true;
    sample.world_mount_twist_valid = true;

    std::ostringstream row_out;
    WriteCsvRow(row_out, sample);
    const std::vector<std::string> row = Split(OneLine(row_out.str()));

    Check(header.size() == row.size(),
          "header and data row have the same number of columns");
    Check(header.size() == 225,
          "log_format 13 has the documented 225 columns");

    const auto value_of = [&](const std::string& name) {
        for (std::size_t i = 0; i < header.size(); ++i)
            if (header[i] == name) return row[i];
        return std::string("<missing>");
    };
    const auto absent = [&](const std::string& name) {
        for (const std::string& field : header)
            if (field == name) return false;
        return true;
    };

    Check(absent("traj_activated") && absent("joint_follow_stop") &&
              absent("hold_state"),
          "retired joint-trajectory and world-hold columns are absent");
    Check(value_of("cart_traj_activated") == "1" &&
              value_of("cart_traj_rejected") == "0" &&
              value_of("cart_traj_complete") == "1" &&
              value_of("cart_traj_cancelled") == "1" &&
              value_of("cart_replan_requested") == "1",
          "Cartesian trajectory edges round-trip");
    Check(value_of("cart_start_position_error_m") == "0.0125" &&
              value_of("cart_start_orientation_error_rad") == "0.025",
          "Cartesian splice errors round-trip");
    Check(value_of("cart_trajectory_id") == "9" &&
              value_of("cart_planner_vicon_sequence") == "42" &&
              value_of("cart_reference_time_s") == "1.25",
          "trajectory identity, snapshot provenance and clock round-trip");
    Check(value_of("cart_ref_x_world_m") == "1.1" &&
              value_of("cart_ref_qw_world") == "0.9" &&
              value_of("cart_ref_vy_world_mps") == "0.2" &&
              value_of("cart_ref_wz_world_radps") == "-0.3",
          "world reference pose and twist round-trip");
    Check(value_of("cart_meas_z_world_m") == "2.2" &&
              value_of("cart_meas_qx_world") == "0.1" &&
              value_of("cart_meas_vx_world_mps") == "-0.4" &&
              value_of("cart_meas_wy_world_radps") == "0.5",
          "world measured pose and twist round-trip");
    Check(value_of("world_fresh") == "1" &&
              value_of("world_mount_twist_valid") == "1",
          "world trust state round-trips");

    Check(value_of("vicon_seq") == "0" &&
              value_of("vicon_age_s") == "nan" &&
              value_of("vicon_mount_x_m") == "nan" &&
              value_of("vicon_mount_valid") == "0",
          "no-Vicon defaults remain unmistakably absent");

    if (failures == 0) {
        std::cout << "all log-schema tests passed\n";
        return 0;
    }
    std::cout << failures << " test(s) failed\n";
    return 1;
}
