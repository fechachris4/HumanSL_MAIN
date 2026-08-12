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
    Check(header.size() == 141,
          "log_format 9 has the 141 columns the Hardware.h comment claims");

    // The joint-trajectory columns are the format-9 tail, in this order.
    const std::vector<std::string> tail = {
        "traj_activated", "traj_rejected", "traj_complete",
        "traj_start_error_deg", "joint_follow_stop", "joint_follow_error_deg"};
    for (std::size_t i = 0; i < tail.size(); ++i) {
        const std::size_t column = header.size() - tail.size() + i;
        Check(column < header.size() && header[column] == tail[i],
              "format-9 column " + tail[i] + " is in its documented place");
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

    if (failures == 0) {
        std::cout << "all log-schema tests passed\n";
        return 0;
    }
    std::cout << failures << " test(s) failed\n";
    return 1;
}
