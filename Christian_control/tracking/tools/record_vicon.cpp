// Sensing-only Vicon recorder: connects, converts every frame through
// SnapshotBuilder, and writes it via ViconRecorder into
// runs/YYYY-MM-DD/vicon_<label>/{frames,entities}.csv. No Kortex, no
// robot -- this only reads Vicon and writes two CSVs.
//
// Usage: record_vicon <host:port> <label> [duration_s] [subject]
//   duration_s defaults to 10; subject defaults to "Dr Octopus Christian".
//
// Stage 0 uses this twice: once for a static recording (arms and wearer
// both still) and once with the arms still while the wearer moves, to
// settle whether the Mount segment markers are on the rigid backpack
// plate or on the wearer's body.

#include "SnapshotBuilder.h"
#include "ViconInterface.h"
#include "ViconRecorder.h"

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

namespace {

std::string TodayFolderName() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
    localtime_r(&now_c, &tm_buf);
    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y-%m-%d");
    return oss.str();
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: record_vicon <host:port> <label> [duration_s] "
                     "[subject]\n";
        return 1;
    }
    const std::string host = argv[1];
    const std::string label = argv[2];
    const double duration_s = (argc > 3) ? std::stod(argv[3]) : 10.0;
    const std::string subject = (argc > 4) ? argv[4] : "Dr Octopus Christian";

    ViconInterface vicon;
    if (!vicon.connect(host)) {
        return 1;
    }

    const std::filesystem::path dir =
        std::filesystem::path("runs") / TodayFolderName() / ("vicon_" + label);
    std::filesystem::create_directories(dir);

    std::ofstream frames_csv(dir / "frames.csv");
    std::ofstream entities_csv(dir / "entities.csv");
    if (!frames_csv || !entities_csv) {
        std::cerr << "Could not open output files under " << dir << "\n";
        return 1;
    }

    ViconRecorder recorder(frames_csv, entities_csv, host, subject);
    recorder.WriteHeader();

    const auto start = std::chrono::steady_clock::now();
    const auto deadline =
        start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(duration_s));

    unsigned int frames_written = 0;
    int prev_frame_number = -1;
    bool ended_early = false;
    while (std::chrono::steady_clock::now() < deadline) {
        if (!vicon.getFrame()) {
            std::cerr << "getFrame() failed after " << frames_written
                       << " frames -- capture ended early\n";
            ended_early = true;
            break;
        }
        const int frame_number = vicon.getFrameNumber();
        if (frame_number == prev_frame_number) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        prev_frame_number = frame_number;

        const double host_time_s =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
                .count();
        const auto markers = vicon.getMarkerPositions(std::string(""));
        const auto segments = vicon.getSegmentPoses();
        const auto snapshot = BuildSnapshot(
            static_cast<unsigned int>(frame_number), host_time_s,
            vicon.getFrameRate(), vicon.getLatencyTotal(), markers, segments);
        recorder.Write(snapshot);
        ++frames_written;
    }

    vicon.disconnect();

    if (frames_written == 0) {
        std::cerr << "record_vicon: wrote 0 frames to " << dir.string()
                   << " -- capture failed\n";
        return 1;
    }
    if (ended_early) {
        std::cerr << "record_vicon: capture ended early -- wrote " << frames_written
                   << " frames (partial) to " << dir.string() << "\n";
        return 1;
    }
    std::cout << "Wrote " << frames_written << " frames to " << dir.string()
              << "\n";
    return 0;
}
