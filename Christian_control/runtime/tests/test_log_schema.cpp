#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "Hardware.h"

namespace {
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
    while (std::getline(stream, field, ',')) fields.push_back(field);
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
    const std::vector<std::string> header = Split(OneLine(header_out.str()));

    LoopLogSample sample;
    sample.measured_tcp_mount_m[0] = 1.1;
    sample.measured_tcp_mount_m[1] = 1.2;
    sample.measured_tcp_mount_m[2] = 1.3;
    sample.commanded_tcp_mount_m[0] = 2.1;
    sample.commanded_tcp_mount_m[1] = 2.2;
    sample.commanded_tcp_mount_m[2] = 2.3;
    sample.measured_tcp_quat_mount_xyzw[0] = 0.1;
    sample.measured_tcp_quat_mount_xyzw[3] = 0.9;
    sample.commanded_tcp_quat_mount_xyzw[1] = 0.2;
    sample.commanded_tcp_quat_mount_xyzw[3] = 0.8;
    sample.measured_arm_chain_mount_m[4][0] = 4.1;
    sample.measured_arm_chain_mount_m[4][1] = 4.2;
    sample.measured_arm_chain_mount_m[4][2] = 4.3;

    std::ostringstream row_out;
    WriteCsvRow(row_out, sample);
    const std::vector<std::string> row = Split(OneLine(row_out.str()));
    Check(header.size() == row.size(), "header and row widths match");
    Check(header.size() == 266, "format 15 has 266 columns");

    const auto value_of = [&](const std::string& name) {
        for (std::size_t i = 0; i < header.size(); ++i)
            if (header[i] == name) return row[i];
        return std::string("<missing>");
    };
    Check(value_of("measured_tcp_x_mount_m") == "1.1" &&
              value_of("measured_tcp_y_mount_m") == "1.2" &&
              value_of("measured_tcp_z_mount_m") == "1.3",
          "measured mount TCP round-trips");
    Check(value_of("commanded_tcp_x_mount_m") == "2.1" &&
              value_of("commanded_tcp_y_mount_m") == "2.2" &&
              value_of("commanded_tcp_z_mount_m") == "2.3",
          "commanded mount TCP round-trips");
    Check(value_of("measured_tcp_qx_mount") == "0.1" &&
              value_of("measured_tcp_qw_mount") == "0.9",
          "measured mount TCP orientation round-trips");
    Check(value_of("commanded_tcp_qy_mount") == "0.2" &&
              value_of("commanded_tcp_qw_mount") == "0.8",
          "commanded mount TCP orientation round-trips");
    Check(value_of("measured_chain_p4_x_mount_m") == "4.1" &&
              value_of("measured_chain_p4_y_mount_m") == "4.2" &&
              value_of("measured_chain_p4_z_mount_m") == "4.3",
          "measured arm chain round-trips");
    return failures == 0 ? 0 : 1;
}
