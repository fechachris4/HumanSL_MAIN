#pragma once

#include "ViconSnapshot.h"

#include <istream>
#include <map>
#include <optional>
#include <string>
#include <vector>

// Reads back a recording written by ViconRecorder. Next() returns one
// ViconSnapshot per call, in the frame order stored in frames.csv, or
// std::nullopt when the stream is exhausted or the file could not be
// parsed -- LastError() distinguishes the two (empty string means an
// ordinary end of a valid recording).
class ViconReplaySource {
public:
    ViconReplaySource(std::istream& frames_csv, std::istream& entities_csv);

    std::optional<ViconSnapshot> Next();
    const std::string& LastError() const { return last_error_; }

private:
    struct FrameRow {
        unsigned int frame_number = 0;
        double host_time_s = 0.0;
        double frame_rate_hz = 0.0;
        double latency_total_s = 0.0;
    };

    bool ParseFrames(std::istream& frames_csv);
    bool ParseEntities(std::istream& entities_csv);

    std::vector<FrameRow> frames_;
    std::map<unsigned int, std::vector<MarkerSample>> markers_by_frame_;
    std::map<unsigned int, std::vector<SegmentSample>> segments_by_frame_;
    std::size_t next_index_ = 0;
    std::string last_error_;
};
