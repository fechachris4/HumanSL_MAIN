//
// The run-log CSV schema: WriteCsvHeader and WriteCsvRow are the authority
// for log_format 9, and every offline script matches them by NAME. A column
// appended to one and not the other silently shifts every later field, so
// this test pins the two together. Links Kortex only because the writer
// lives in Hardware.cpp; it never connects to anything.
//

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Hardware.h"

namespace
{
    int failures = 0;

    void Check(bool ok, const std::string& what)
    {
        if (!ok) {
            std::cout << "FAIL: " << what << "\n";
            ++failures;
        }
    }

    std::vector<std::string> Split(const std::string& line)
    {
        std::vector<std::string> fields;
        std::istringstream stream(line);
        std::string field;
        while (std::getline(stream, field, ','))
            fields.push_back(field);
        return fields;
    }

    std::string OneLine(const std::string& text)
    {
        const std::size_t end = text.find('\n');
        return end == std::string::npos ? text : text.substr(0, end);
    }
} // namespace

int main()
{
    std::ostringstream header_out;
    WriteCsvHeader(header_out);
    const std::string header_line = OneLine(header_out.str());
    const std::vector<std::string> header = Split(header_line);

    LoopLogSample sample;
    sample.joint_traj_activated = true;
    sample.joint_traj_rejected = false;
    sample.joint_traj_complete_edge = true;
    sample.joint_traj_start_error_deg = 1.25;
    sample.joint_following_error_stop = true;
    sample.joint_following_error_deg = 9.5;
    std::ostringstream row_out;
    WriteCsvRow(row_out, sample);
    const std::vector<std::string> row = Split(OneLine(row_out.str()));

    Check(header.size() == row.size(),
          "the header and a data row have the same number of columns");
    Check(header.size() == 190,
          "log_format 11 has the 190 columns the Hardware.h comment claims");

    // The format-10/11 tail: 4 sample-contract fields, 8 columns per
    // segment in the BasePose.h order, then the format-11 hold evidence.
    std::vector<std::string> tail = {"vicon_seq", "vicon_frame",
                                     "vicon_latency_s", "vicon_age_s"};
    for (const char* segment :
         {"mount", "leftbase", "rightbase", "leftee", "rightee"}) {
        const std::string s(segment);
        for (const char* field : {"_x_m", "_y_m", "_z_m", "_qx", "_qy",
                                  "_qz", "_qw", "_valid"})
            tail.push_back("vicon_" + s + field);
    }
    for (const char* name : {"hold_state", "world_err_m",
                             "world_err_rot_rad", "hold_ramp",
                             "hold_reanchor_count"})
        tail.push_back(name);
    for (std::size_t i = 0; i < tail.size(); ++i) {
        const std::size_t column = header.size() - tail.size() + i;
        Check(column < header.size() && header[column] == tail[i],
              "format-11 column " + tail[i] + " is in its documented place");
    }

    // The evidence a joint following-error stop leaves behind.
    const auto value_of = [&](const std::string& name) {
        for (std::size_t i = 0; i < header.size() && i < row.size(); ++i)
            if (header[i] == name)
                return row[i];
        return std::string("<missing>");
    };
    Check(value_of("traj_activated") == "1" && value_of("traj_rejected") == "0",
          "edge flags are written as 1/0");
    Check(value_of("joint_follow_stop") == "1",
          "a stopping row records that the joint gate fired");
    Check(value_of("joint_follow_error_deg") == "9.5",
          "a stopping row records the error that fired it");
    Check(value_of("traj_start_error_deg") == "1.25",
          "the splice distance is recorded next to the edges");

    // World-pose columns: the default sample means "no Vicon has ever
    // arrived" and must be unmistakable — NaN poses, 0 flags, never a
    // plausible zero position (telemetry.py turns nan into None).
    Check(value_of("vicon_seq") == "0",
          "no-Vicon rows carry sequence 0");
    Check(value_of("vicon_age_s") == "nan",
          "no-Vicon rows carry NaN age, never a plausible zero");
    Check(value_of("vicon_mount_x_m") == "nan",
          "no-Vicon rows carry NaN positions");
    Check(value_of("vicon_mount_valid") == "0",
          "no-Vicon rows mark segments invalid");

    // A populated sample round-trips through the row writer.
    LoopLogSample vicon_sample;
    vicon_sample.vicon_sequence = 42;
    vicon_sample.vicon_frame_number = 668410;
    vicon_sample.vicon_latency_s = 0.012;
    vicon_sample.vicon_age_s = 0.004;
    vicon_sample.vicon_seg_pos_m[0][0] = 1.5;
    vicon_sample.vicon_seg_quat_xyzw[0][3] = 1.0;
    vicon_sample.vicon_seg_valid[0] = true;
    std::ostringstream vicon_row_out;
    WriteCsvRow(vicon_row_out, vicon_sample);
    const std::vector<std::string> vicon_row =
        Split(OneLine(vicon_row_out.str()));
    const auto vicon_value_of = [&](const std::string& name) {
        for (std::size_t i = 0; i < header.size() && i < vicon_row.size(); ++i)
            if (header[i] == name)
                return vicon_row[i];
        return std::string("<missing>");
    };
    Check(vicon_value_of("vicon_seq") == "42", "sequence round-trips");
    Check(vicon_value_of("vicon_frame") == "668410",
          "frame number round-trips");
    Check(vicon_value_of("vicon_mount_x_m") == "1.5",
          "segment position round-trips");
    Check(vicon_value_of("vicon_mount_qw") == "1", "quaternion round-trips");
    Check(vicon_value_of("vicon_mount_valid") == "1",
          "validity round-trips");
    Check(vicon_value_of("vicon_leftee_x_m") == "nan",
          "an untouched segment in a populated sample stays NaN");

    // Hold evidence: absent-by-default, and a populated row round-trips.
    Check(value_of("hold_state") == "0" && value_of("world_err_m") == "nan" &&
              value_of("hold_ramp") == "nan",
          "no-hold rows record state 0 and NaN errors, never zeros");
    LoopLogSample hold_sample;
    hold_sample.hold_state = 2;
    hold_sample.world_err_m = 0.031;
    hold_sample.world_err_rot_rad = 0.02;
    hold_sample.hold_ramp = 1.0;
    hold_sample.hold_reanchor_count = 3;
    std::ostringstream hold_row_out;
    WriteCsvRow(hold_row_out, hold_sample);
    const std::vector<std::string> hold_row =
        Split(OneLine(hold_row_out.str()));
    const auto hold_value_of = [&](const std::string& name) {
        for (std::size_t i = 0; i < header.size() && i < hold_row.size(); ++i)
            if (header[i] == name)
                return hold_row[i];
        return std::string("<missing>");
    };
    Check(hold_value_of("hold_state") == "2" &&
              hold_value_of("world_err_m") == "0.031" &&
              hold_value_of("hold_reanchor_count") == "3",
          "hold evidence round-trips through the row writer");

    if (failures == 0) {
        std::cout << "all log-schema tests passed\n";
        return 0;
    }
    std::cout << failures << " test(s) failed\n";
    return 1;
}
