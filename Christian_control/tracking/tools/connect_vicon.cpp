// Connection smoke test: connect to the Vicon DataStream server, report the
// frame rate, then print every labeled marker for a handful of frames and
// exit. Success (exit 0) means the link works end to end — TCP connect,
// streaming enabled, frames arriving, marker data readable.
//
// Usage: connect_vicon [host:port]   (default 192.168.128.206:801)

#include "ViconInterface.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

int main(int argc, char* argv[]) {
    const std::string host = (argc > 1) ? argv[1] : "192.168.128.206:801";

    ViconInterface vicon;
    if (!vicon.connect(host)) {
        return 1;
    }

    std::cout << "Server frame rate: " << vicon.getFrameRate() << " Hz\n";

    constexpr int kFramesToShow = 10;
    int shown = 0;
    int prev_frame_number = -1;

    while (shown < kFramesToShow) {
        const int frame_number = vicon.getFrameNumber();
        if (frame_number == prev_frame_number) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        if (!vicon.getFrame()) {
            std::cerr << "Frame " << frame_number << " could not be read — stopping.\n";
            return 1;
        }
        prev_frame_number = frame_number;
        ++shown;

        // Empty prefix matches every labeled marker of every subject.
        const auto labeled = vicon.getMarkerPositions(std::string(""));
        const auto unlabeled = vicon.getUnlabeledMarkers();
        const auto segments = vicon.getSegmentPoses();

        std::cout << "frame " << frame_number << ": " << labeled.size()
                  << " labeled, " << unlabeled.size() << " unlabeled, "
                  << segments.size() << " segments\n";
        for (const auto& s : segments) {
            std::cout << "  segment " << s.subject_name << "/" << s.segment_name
                      << std::fixed << std::setprecision(1)
                      << " pos=(" << s.x << ", " << s.y << ", " << s.z << ") mm"
                      << std::setprecision(4)
                      << " q=(" << s.qx << ", " << s.qy << ", " << s.qz << ", " << s.qw
                      << ")"
                      << (s.occluded ? "  (occluded)" : "") << "\n";
        }
        for (const auto& m : labeled) {
            std::cout << "  " << std::left << std::setw(16) << m.name
                      << std::right << std::fixed << std::setprecision(1)
                      << std::setw(10) << m.x << std::setw(10) << m.y
                      << std::setw(10) << m.z << " mm"
                      << (m.occluded ? "  (occluded)" : "") << "\n";
        }
    }

    vicon.disconnect();
    std::cout << "OK: " << kFramesToShow << " frames received from " << host << "\n";
    return 0;
}
