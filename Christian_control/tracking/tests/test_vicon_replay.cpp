#include "SnapshotBuilder.h"
#include "ViconRecorder.h"
#include "ViconReplaySource.h"

#include <cassert>
#include <sstream>

namespace {

ViconSnapshot MakeSnapshot(unsigned int frame_number) {
    MarkerData marker;
    marker.name = "M1";
    marker.x = 1000.0;
    marker.y = 2000.0;
    marker.z = 3000.0;
    marker.occluded = false;

    SegmentData segment;
    segment.subject_name = "Dr Octopus Christian";
    segment.segment_name = "Mount";
    segment.x = 10.0;
    segment.y = 20.0;
    segment.z = 30.0;
    segment.qx = 0.0;
    segment.qy = 0.0;
    segment.qz = 0.0;
    segment.qw = 1.0;
    segment.occluded = false;

    return BuildSnapshot(frame_number, frame_number * 0.01, 100.0, 0.015,
                          {marker}, {segment});
}

}  // namespace

int main() {
    // Round trip: every field of two written snapshots survives read-back.
    {
        std::ostringstream frames_out, entities_out;
        ViconRecorder recorder(frames_out, entities_out, "192.168.128.206:801",
                                "Dr Octopus Christian");
        recorder.WriteHeader();
        const auto snap0 = MakeSnapshot(0);
        const auto snap1 = MakeSnapshot(1);
        recorder.Write(snap0);
        recorder.Write(snap1);

        std::istringstream frames_in(frames_out.str());
        std::istringstream entities_in(entities_out.str());
        ViconReplaySource replay(frames_in, entities_in);
        assert(replay.LastError().empty());

        const auto read0 = replay.Next();
        assert(read0.has_value());
        assert(read0->frame_number == snap0.frame_number);
        assert(read0->host_time_s == snap0.host_time_s);
        assert(read0->frame_rate_hz == snap0.frame_rate_hz);
        assert(read0->latency_total_s == snap0.latency_total_s);
        assert(read0->markers.size() == 1);
        assert(read0->markers[0].name == snap0.markers[0].name);
        assert(read0->markers[0].position_m == snap0.markers[0].position_m);
        assert(read0->markers[0].valid == snap0.markers[0].valid);
        assert(read0->segments.size() == 1);
        assert(read0->segments[0].subject_name == snap0.segments[0].subject_name);
        assert(read0->segments[0].segment_name == snap0.segments[0].segment_name);
        assert(read0->segments[0].position_m == snap0.segments[0].position_m);
        assert(read0->segments[0].orientation.coeffs() ==
               snap0.segments[0].orientation.coeffs());
        assert(read0->segments[0].valid == snap0.segments[0].valid);

        const auto read1 = replay.Next();
        assert(read1.has_value());
        assert(read1->frame_number == 1);

        const auto read2 = replay.Next();
        assert(!read2.has_value());
        assert(replay.LastError().empty());  // ordinary end, not an error
    }

    // Empty recording: zero frames after the header degrades with a reason.
    {
        std::ostringstream frames_out, entities_out;
        ViconRecorder recorder(frames_out, entities_out, "host", "subject");
        recorder.WriteHeader();

        std::istringstream frames_in(frames_out.str());
        std::istringstream entities_in(entities_out.str());
        ViconReplaySource replay(frames_in, entities_in);
        assert(!replay.LastError().empty());
        assert(!replay.Next().has_value());
    }

    // Unknown vicon_format: degrades with a reason, does not crash.
    {
        std::istringstream frames_in(
            "# vicon_format = 2\n"
            "frame_number,host_time_s,frame_rate_hz,latency_total_s\n"
            "0,0,0,0\n");
        std::istringstream entities_in(
            "# vicon_format = 2\n"
            "frame_number,kind,subject,name,x_m,y_m,z_m,qx,qy,qz,qw,valid,"
            "invalid_reason\n");
        ViconReplaySource replay(frames_in, entities_in);
        assert(!replay.LastError().empty());
        assert(!replay.Next().has_value());
    }

    // Truncated/malformed row: degrades with a reason, does not crash.
    {
        std::istringstream frames_in(
            "# vicon_format = 1\n"
            "frame_number,host_time_s,frame_rate_hz,latency_total_s\n"
            "0,0.0,100.0\n");  // one column short
        std::istringstream entities_in(
            "# vicon_format = 1\n"
            "frame_number,kind,subject,name,x_m,y_m,z_m,qx,qy,qz,qw,valid,"
            "invalid_reason\n");
        ViconReplaySource replay(frames_in, entities_in);
        assert(!replay.LastError().empty());
        assert(!replay.Next().has_value());
    }

    // Missing column in the header itself: degrades with a reason.
    {
        std::istringstream frames_in(
            "# vicon_format = 1\n"
            "frame_number,host_time_s,frame_rate_hz\n"  // latency_total_s missing
            "0,0.0,100.0\n");
        std::istringstream entities_in(
            "# vicon_format = 1\n"
            "frame_number,kind,subject,name,x_m,y_m,z_m,qx,qy,qz,qw,valid,"
            "invalid_reason\n");
        ViconReplaySource replay(frames_in, entities_in);
        assert(!replay.LastError().empty());
    }

    return 0;
}
