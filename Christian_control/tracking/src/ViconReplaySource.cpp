#include "ViconReplaySource.h"

#include <sstream>
#include <stdexcept>

namespace {

std::vector<std::string> SplitCsvLine(const std::string& line) {
    std::vector<std::string> fields;
    std::string field;
    std::istringstream stream(line);
    while (std::getline(stream, field, ',')) {
        fields.push_back(field);
    }
    // Handle trailing comma: if line ends with comma, add empty final field.
    if (!line.empty() && line.back() == ',') {
        fields.push_back("");
    }
    return fields;
}

bool ParseUInt(const std::string& text, unsigned int& out) {
    if (text.empty()) return false;
    try {
        std::size_t consumed = 0;
        const unsigned long value = std::stoul(text, &consumed);
        if (consumed != text.size()) return false;
        out = static_cast<unsigned int>(value);
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool ParseDouble(const std::string& text, double& out) {
    if (text.empty()) return false;
    try {
        std::size_t consumed = 0;
        out = std::stod(text, &consumed);
        return consumed == text.size();
    } catch (const std::exception&) {
        return false;
    }
}

}  // namespace

ViconReplaySource::ViconReplaySource(std::istream& frames_csv,
                                      std::istream& entities_csv) {
    if (!ParseFrames(frames_csv)) return;
    if (!ParseEntities(entities_csv)) return;
    if (frames_.empty()) {
        last_error_ = "recording contains zero frames";
    }
}

bool ViconReplaySource::ParseFrames(std::istream& frames_csv) {
    std::string line;
    bool saw_format = false;
    bool have_header = false;
    while (std::getline(frames_csv, line)) {
        if (line.empty()) continue;
        if (line[0] == '#') {
            if (line.find("vicon_format = 1") != std::string::npos) {
                saw_format = true;
            }
            continue;
        }
        have_header = true;
        break;
    }
    if (!have_header) {
        last_error_ = "frames.csv has no header row";
        return false;
    }
    if (!saw_format) {
        last_error_ = "unknown or missing vicon_format in frames.csv";
        return false;
    }

    const auto header = SplitCsvLine(line);
    if (header.size() != 4 || header[0] != "frame_number") {
        last_error_ = "frames.csv header missing expected columns";
        return false;
    }

    while (std::getline(frames_csv, line)) {
        if (line.empty()) continue;
        const auto fields = SplitCsvLine(line);
        if (fields.size() != 4) {
            last_error_ = "frames.csv row has the wrong number of columns";
            return false;
        }
        FrameRow row;
        double host_time = 0, rate = 0, latency = 0;
        if (!ParseUInt(fields[0], row.frame_number) ||
            !ParseDouble(fields[1], host_time) ||
            !ParseDouble(fields[2], rate) || !ParseDouble(fields[3], latency)) {
            last_error_ = "frames.csv row could not be parsed";
            return false;
        }
        row.host_time_s = host_time;
        row.frame_rate_hz = rate;
        row.latency_total_s = latency;
        frames_.push_back(row);
    }
    return true;
}

bool ViconReplaySource::ParseEntities(std::istream& entities_csv) {
    std::string line;
    bool have_header = false;
    while (std::getline(entities_csv, line)) {
        if (line.empty()) continue;
        if (line[0] == '#') continue;
        have_header = true;
        break;
    }
    if (!have_header) {
        last_error_ = "entities.csv has no header row";
        return false;
    }

    const auto header = SplitCsvLine(line);
    if (header.size() != 13 || header[0] != "frame_number") {
        last_error_ = "entities.csv header missing expected columns";
        return false;
    }

    while (std::getline(entities_csv, line)) {
        if (line.empty()) continue;
        const auto fields = SplitCsvLine(line);
        if (fields.size() != 13) {
            last_error_ = "entities.csv row has the wrong number of columns";
            return false;
        }
        unsigned int frame_number = 0;
        if (!ParseUInt(fields[0], frame_number)) {
            last_error_ = "entities.csv row could not be parsed";
            return false;
        }
        const std::string& kind = fields[1];
        double x = 0, y = 0, z = 0;
        if (!ParseDouble(fields[4], x) || !ParseDouble(fields[5], y) ||
            !ParseDouble(fields[6], z)) {
            last_error_ = "entities.csv row could not be parsed";
            return false;
        }
        const bool valid = (fields[11] == "1");
        const std::string& invalid_reason = fields[12];

        if (kind == "marker") {
            MarkerSample sample;
            sample.name = fields[3];
            sample.position_m = Eigen::Vector3d(x, y, z);
            sample.valid = valid;
            sample.invalid_reason = invalid_reason;
            markers_by_frame_[frame_number].push_back(sample);
        } else if (kind == "segment") {
            double qx = 0, qy = 0, qz = 0, qw = 0;
            if (!ParseDouble(fields[7], qx) || !ParseDouble(fields[8], qy) ||
                !ParseDouble(fields[9], qz) || !ParseDouble(fields[10], qw)) {
                last_error_ = "entities.csv row could not be parsed";
                return false;
            }
            SegmentSample sample;
            sample.subject_name = fields[2];
            sample.segment_name = fields[3];
            sample.position_m = Eigen::Vector3d(x, y, z);
            sample.orientation = Eigen::Quaterniond(qw, qx, qy, qz);
            sample.valid = valid;
            sample.invalid_reason = invalid_reason;
            segments_by_frame_[frame_number].push_back(sample);
        } else {
            last_error_ = "entities.csv row has an unknown kind";
            return false;
        }
    }
    return true;
}

std::optional<ViconSnapshot> ViconReplaySource::Next() {
    if (next_index_ >= frames_.size()) return std::nullopt;
    if (!last_error_.empty()) return std::nullopt;

    const auto& row = frames_[next_index_];
    ViconSnapshot snapshot;
    snapshot.frame_number = row.frame_number;
    snapshot.host_time_s = row.host_time_s;
    snapshot.frame_rate_hz = row.frame_rate_hz;
    snapshot.latency_total_s = row.latency_total_s;

    const auto markers_it = markers_by_frame_.find(row.frame_number);
    if (markers_it != markers_by_frame_.end()) snapshot.markers = markers_it->second;
    const auto segments_it = segments_by_frame_.find(row.frame_number);
    if (segments_it != segments_by_frame_.end())
        snapshot.segments = segments_it->second;

    ++next_index_;
    return snapshot;
}
