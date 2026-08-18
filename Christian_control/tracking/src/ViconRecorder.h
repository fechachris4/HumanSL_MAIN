#pragma once

#include "ViconSnapshot.h"

#include <ostream>
#include <string>

// Writes a stream of ViconSnapshots into the two-file house run-log
// layout: frames.csv holds one row per frame (frame-level scalars, with a
// '#'-prefixed preamble parsers skip -- see the controller's run logs for
// the same convention); entities.csv holds one row per marker/segment per
// frame in long format, joined to frames.csv by frame_number, so a
// variable entity count needs no schema change.
//
// Construct with the two already-open output streams (caller owns them --
// this class never opens or closes a file). Call WriteHeader() once
// before the first Write().
class ViconRecorder {
public:
    ViconRecorder(std::ostream& frames_csv, std::ostream& entities_csv,
                  std::string host, std::string subject);

    void WriteHeader();
    void Write(const ViconSnapshot& snapshot);

private:
    std::ostream& frames_csv_;
    std::ostream& entities_csv_;
    std::string host_;
    std::string subject_;
};
