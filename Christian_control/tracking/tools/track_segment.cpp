// Track one Vicon segment live and report how far it has moved from where
// it started. Written for the Torso segment (the wearer's trunk, tracked
// separately from the backpack Mount plate — see
// Christian_control/docs/decisions/vicon-frame-contract.md), but the
// segment name is an argument, so it works for any of them.
//
// Sensing only: connects to the Vicon DataStream server, reads frames, and
// prints. No Kortex, no robot, nothing commanded.
//
// Usage: track_segment [host:port] [segment] [duration_s] [csv_path]
//   host:port    defaults to 192.168.128.206:801
//   segment      defaults to "Torso"; may be "Segment" or "Subject/Segment"
//   duration_s   defaults to 0, meaning run until Ctrl-C
//   csv_path     if given, every frame is also written there
//
// The console line is throttled to ~10 Hz so it is readable; the CSV and
// the end-of-run summary use every frame received.

#include "SnapshotBuilder.h"
#include "ViconInterface.h"
#include "ViconSnapshot.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

volatile std::sig_atomic_t g_interrupted = 0;

void OnInterrupt(int) { g_interrupted = 1; }

std::string ToLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

// A segment the user asked for, as "Segment" or "Subject/Segment". Matching
// is case-insensitive because Nexus names are typed by hand.
struct SegmentQuery {
    std::string subject;  // empty means "any subject"
    std::string segment;
};

SegmentQuery ParseQuery(const std::string& text) {
    SegmentQuery query;
    const auto slash = text.find('/');
    if (slash == std::string::npos) {
        query.segment = ToLower(text);
    } else {
        query.subject = ToLower(text.substr(0, slash));
        query.segment = ToLower(text.substr(slash + 1));
    }
    return query;
}

bool Matches(const SegmentQuery& query, const SegmentSample& sample) {
    if (ToLower(sample.segment_name) != query.segment) return false;
    return query.subject.empty() || ToLower(sample.subject_name) == query.subject;
}

const SegmentSample* FindSegment(const ViconSnapshot& snapshot,
                                 const SegmentQuery& query) {
    for (const auto& sample : snapshot.segments) {
        if (Matches(query, sample)) return &sample;
    }
    return nullptr;
}

// Angle of the rotation that takes `reference` to `current`, in degrees.
// Both quaternions are already checked unit-norm by BuildSnapshot. The
// std::abs on w folds q and -q together: they are the same rotation, and
// without it a sign flip in the stream would read as a 360 deg jump.
double RotationFromReferenceDeg(const Eigen::Quaterniond& reference,
                                const Eigen::Quaterniond& current) {
    const Eigen::Quaterniond relative = reference.conjugate() * current;
    const double angle_rad =
        2.0 * std::atan2(relative.vec().norm(), std::abs(relative.w()));
    return angle_rad * 180.0 / M_PI;
}

// Everything the run accumulates, so the summary at the end does not
// depend on the CSV having been written.
struct RunStats {
    unsigned int frames = 0;
    // Vicon's own frame counter at the first and last frame we processed.
    // The span between them is how many frames the server produced; the
    // difference against `frames` is what this tool missed, and it must be
    // reported rather than left invisible — a 100 Hz stream read at 88 Hz
    // looks identical to a 88 Hz stream unless the counter is checked.
    unsigned int first_vicon_frame = 0;
    unsigned int last_vicon_frame = 0;
    unsigned int valid_frames = 0;
    unsigned int invalid_spells = 0;      // runs of consecutive invalid frames
    unsigned int longest_invalid_run = 0;
    unsigned int current_invalid_run = 0;
    double max_translation_mm = 0.0;
    double max_rotation_deg = 0.0;
    Eigen::Vector3d min_position_m = Eigen::Vector3d::Zero();
    Eigen::Vector3d max_position_m = Eigen::Vector3d::Zero();
    bool seen_any_valid = false;
};

void NoteInvalid(RunStats& stats) {
    if (stats.current_invalid_run == 0) ++stats.invalid_spells;
    ++stats.current_invalid_run;
    stats.longest_invalid_run =
        std::max(stats.longest_invalid_run, stats.current_invalid_run);
}

void NoteValid(RunStats& stats, const Eigen::Vector3d& position_m,
               double translation_mm, double rotation_deg) {
    stats.current_invalid_run = 0;
    ++stats.valid_frames;
    if (!stats.seen_any_valid) {
        stats.seen_any_valid = true;
        stats.min_position_m = position_m;
        stats.max_position_m = position_m;
    } else {
        stats.min_position_m = stats.min_position_m.cwiseMin(position_m);
        stats.max_position_m = stats.max_position_m.cwiseMax(position_m);
    }
    stats.max_translation_mm = std::max(stats.max_translation_mm, translation_mm);
    stats.max_rotation_deg = std::max(stats.max_rotation_deg, rotation_deg);
}

void PrintAvailableSegments(const ViconSnapshot& snapshot) {
    if (snapshot.segments.empty()) {
        std::cerr << "  (the stream carried no segments at all — is a subject "
                     "enabled in Nexus?)\n";
        return;
    }
    for (const auto& sample : snapshot.segments) {
        std::cerr << "  " << sample.subject_name << "/" << sample.segment_name
                  << (sample.valid ? "" : "  (invalid this frame: ")
                  << (sample.valid ? "" : sample.invalid_reason)
                  << (sample.valid ? "" : ")") << "\n";
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    const std::string host = (argc > 1) ? argv[1] : "192.168.128.206:801";
    const std::string segment_arg = (argc > 2) ? argv[2] : "Torso";
    const double duration_s = (argc > 3) ? std::stod(argv[3]) : 0.0;
    const std::string csv_path = (argc > 4) ? argv[4] : "";

    const SegmentQuery query = ParseQuery(segment_arg);
    if (query.segment.empty()) {
        std::cerr << "Segment name must not be empty.\n";
        return 1;
    }

    std::ofstream csv;
    if (!csv_path.empty()) {
        csv.open(csv_path);
        if (!csv) {
            std::cerr << "Could not open " << csv_path << " for writing.\n";
            return 1;
        }
        csv << "host_time_s,frame_number,valid,invalid_reason,x_m,y_m,z_m,"
               "qx,qy,qz,qw,translation_from_ref_mm,rotation_from_ref_deg\n";
    }

    ViconInterface vicon;
    if (!vicon.connect(host)) {
        return 1;
    }

    std::signal(SIGINT, OnInterrupt);

    const auto start = std::chrono::steady_clock::now();
    auto ReadSnapshot = [&vicon, &start]() {
        const double host_time_s =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
                .count();
        // Segments only. getMarkerPositions("") walks every marker of every
        // subject doing string comparisons, and this tool never reads a
        // marker — paying for it per frame made the loop miss roughly one
        // frame in ten at 100 Hz.
        return BuildSnapshot(static_cast<unsigned int>(vicon.getFrameNumber()),
                             host_time_s, vicon.getFrameRate(),
                             vicon.getLatencyTotal(), {},
                             vicon.getSegmentPoses());
    };

    // Resolve the segment before streaming, so a wrong or missing name fails
    // now with the list of what is actually there, rather than printing
    // nothing for a whole run. A few frames of grace: a segment can be
    // momentarily absent while the subject is being picked up.
    const SegmentSample* found = nullptr;
    ViconSnapshot probe;
    for (int attempt = 0; attempt < 50 && found == nullptr; ++attempt) {
        if (!vicon.getFrame()) {
            std::cerr << "getFrame() failed while looking for the segment.\n";
            return 1;
        }
        probe = ReadSnapshot();
        found = FindSegment(probe, query);
        if (found == nullptr) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (found == nullptr) {
        std::cerr << "No segment named \"" << segment_arg
                  << "\" in the stream. Segments present:\n";
        PrintAvailableSegments(probe);
        vicon.disconnect();
        return 1;
    }

    const std::string subject_name = found->subject_name;
    const std::string segment_name = found->segment_name;
    std::cout << "Tracking " << subject_name << "/" << segment_name << " at "
              << vicon.getFrameRate() << " Hz";
    if (duration_s > 0.0) {
        std::cout << " for " << duration_s << " s";
    } else {
        std::cout << " until Ctrl-C";
    }
    std::cout << ".\nPositions are metres in the Vicon world frame. The two "
                 "delta columns are\nmovement away from the first valid pose "
                 "of this run.\n\n"
              << std::left << std::setw(9) << "  t(s)" << std::setw(10) << "frame"
              << std::right << std::setw(10) << "x(m)" << std::setw(10) << "y(m)"
              << std::setw(10) << "z(m)" << std::setw(11) << "move(mm)"
              << std::setw(11) << "turn(deg)" << "  state\n";

    bool have_reference = false;
    Eigen::Vector3d reference_position_m = Eigen::Vector3d::Zero();
    Eigen::Quaterniond reference_orientation = Eigen::Quaterniond::Identity();

    RunStats stats;
    // When tracking actually begins. The summary's rate must be measured
    // over this window, not since process start: connecting and finding the
    // segment take a fraction of a second, and counting that time makes a
    // healthy 100 Hz stream report as ~84 Hz, which reads as dropped frames.
    const double track_start_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
            .count();
    int prev_frame_number = -1;
    double next_print_s = 0.0;
    bool read_failed = false;

    while (!g_interrupted) {
        const double elapsed_s =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
                .count();
        if (duration_s > 0.0 && elapsed_s >= duration_s) break;

        if (!vicon.getFrame()) {
            std::cerr << "\ngetFrame() failed after " << stats.frames
                      << " frames — stopping.\n";
            read_failed = true;
            break;
        }
        const int frame_number = vicon.getFrameNumber();
        if (frame_number == prev_frame_number) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        prev_frame_number = frame_number;

        const ViconSnapshot snapshot = ReadSnapshot();
        const SegmentSample* sample = FindSegment(snapshot, query);
        if (stats.frames == 0) stats.first_vicon_frame = snapshot.frame_number;
        stats.last_vicon_frame = snapshot.frame_number;
        ++stats.frames;

        std::string state;
        double translation_mm = 0.0;
        double rotation_deg = 0.0;

        if (sample == nullptr) {
            state = "absent";
            NoteInvalid(stats);
        } else if (!sample->valid) {
            state = sample->invalid_reason;
            NoteInvalid(stats);
        } else {
            if (!have_reference) {
                have_reference = true;
                reference_position_m = sample->position_m;
                reference_orientation = sample->orientation;
            }
            translation_mm =
                (sample->position_m - reference_position_m).norm() * 1000.0;
            rotation_deg =
                RotationFromReferenceDeg(reference_orientation, sample->orientation);
            state = "ok";
            NoteValid(stats, sample->position_m, translation_mm, rotation_deg);
        }

        const bool usable = (sample != nullptr && sample->valid);
        if (csv) {
            csv << std::fixed << std::setprecision(4) << snapshot.host_time_s << ","
                << snapshot.frame_number << "," << (usable ? 1 : 0) << ","
                << (usable ? "" : state) << ",";
            if (usable) {
                csv << std::setprecision(6) << sample->position_m.x() << ","
                    << sample->position_m.y() << "," << sample->position_m.z() << ","
                    << sample->orientation.x() << "," << sample->orientation.y() << ","
                    << sample->orientation.z() << "," << sample->orientation.w() << ","
                    << std::setprecision(3) << translation_mm << "," << rotation_deg;
            } else {
                csv << ",,,,,,,,";
            }
            csv << "\n";
        }

        if (snapshot.host_time_s >= next_print_s) {
            next_print_s = snapshot.host_time_s + 0.1;
            std::cout << std::left << std::fixed << std::setprecision(2)
                      << std::setw(9) << snapshot.host_time_s << std::setw(10)
                      << snapshot.frame_number << std::right;
            // Position to 0.1 mm and the deltas to 0.01 mm / 0.001 deg:
            // marker noise is a few hundredths of a millimetre, and at
            // coarser rounding the noise floor prints as a column of
            // zeros and the display looks dead when it is working.
            if (usable) {
                std::cout << std::setprecision(4) << std::setw(10)
                          << sample->position_m.x() << std::setw(10)
                          << sample->position_m.y() << std::setw(10)
                          << sample->position_m.z() << std::setprecision(2)
                          << std::setw(11) << translation_mm
                          << std::setprecision(3) << std::setw(11)
                          << rotation_deg;
            } else {
                std::cout << std::setw(10) << "-" << std::setw(10) << "-"
                          << std::setw(10) << "-" << std::setw(11) << "-"
                          << std::setw(11) << "-";
            }
            // Flushed every line: a live monitor read through a pipe or an
            // IDE console would otherwise sit in the stdout buffer for
            // several seconds and look dead.
            std::cout << "  " << state << std::endl;
        }
    }

    vicon.disconnect();

    const double elapsed_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() -
        track_start_s;
    const Eigen::Vector3d spread_mm =
        (stats.max_position_m - stats.min_position_m) * 1000.0;

    std::cout << "\n" << subject_name << "/" << segment_name << " — "
              << std::setprecision(1) << elapsed_s << " s, " << stats.frames
              << " frames";
    if (stats.frames > 0) {
        std::cout << " (" << std::setprecision(2)
                  << static_cast<double>(stats.frames) / std::max(elapsed_s, 1e-9)
                  << " Hz)";
    }
    std::cout << "\n";
    if (stats.frames == 0) {
        std::cout << "No frames received.\n";
        return read_failed ? 1 : 0;
    }
    const unsigned int stream_span =
        stats.last_vicon_frame - stats.first_vicon_frame + 1;
    if (stream_span > stats.frames) {
        std::cout << "  read " << stats.frames << " of the " << stream_span
                  << " frames the server produced ("
                  << std::setprecision(1)
                  << 100.0 * static_cast<double>(stats.frames) /
                         static_cast<double>(stream_span)
                  << "%) — the rest were skipped by this tool, not lost by "
                     "Vicon\n";
    }
    std::cout << "  valid: " << stats.valid_frames << "/" << stats.frames << " ("
              << std::setprecision(2)
              << 100.0 * static_cast<double>(stats.valid_frames) /
                     static_cast<double>(stats.frames)
              << "%)";
    if (stats.invalid_spells > 0) {
        std::cout << ", " << stats.invalid_spells << " invalid spell(s), longest "
                  << stats.longest_invalid_run << " frames";
    }
    std::cout << "\n";
    if (stats.seen_any_valid) {
        std::cout << std::setprecision(2)
                  << "  moved up to " << stats.max_translation_mm
                  << " mm and turned up to " << stats.max_rotation_deg
                  << " deg from the first valid pose\n"
                  << "  position spread: x " << spread_mm.x() << " mm, y "
                  << spread_mm.y() << " mm, z " << spread_mm.z() << " mm\n";
    } else {
        std::cout << "  never valid — no pose was usable during this run\n";
    }
    if (!csv_path.empty()) {
        std::cout << "  wrote " << csv_path << "\n";
    }
    return read_failed ? 1 : 0;
}
