#include "SnapshotBuilder.h"
#include "ViconRecorder.h"

#include <cassert>
#include <sstream>

namespace {

MarkerData MakeMarker() {
    MarkerData m;
    m.name = "M1";
    m.x = 1000.0;
    m.y = 2000.0;
    m.z = 3000.0;
    m.occluded = false;
    return m;
}

SegmentData MakeSegment() {
    SegmentData s;
    s.subject_name = "Dr Octopus Christian";
    s.segment_name = "Mount";
    s.x = 10.0;
    s.y = 20.0;
    s.z = 30.0;
    s.qx = 0.0;
    s.qy = 0.0;
    s.qz = 0.0;
    s.qw = 1.0;
    s.occluded = false;
    return s;
}

}  // namespace

int main() {
    std::ostringstream frames_out, entities_out;
    ViconRecorder recorder(frames_out, entities_out, "192.168.128.206:801",
                            "Dr Octopus Christian");
    recorder.WriteHeader();

    const auto snapshot = BuildSnapshot(7, 0.07, 100.0, 0.015, {MakeMarker()},
                                         {MakeSegment()});
    recorder.Write(snapshot);

    const std::string frames_text = frames_out.str();
    assert(frames_text.find("# vicon_format = 1") != std::string::npos);
    assert(frames_text.find("frame_number,host_time_s,frame_rate_hz,latency_total_s") !=
           std::string::npos);
    assert(frames_text.find("7,0.07,100,0.015") != std::string::npos);

    const std::string entities_text = entities_out.str();
    assert(entities_text.find(
               "frame_number,kind,subject,name,x_m,y_m,z_m,qx,qy,qz,qw,valid,"
               "invalid_reason") != std::string::npos);
    assert(entities_text.find("7,marker,,M1,1,2,3,,,,,1,") != std::string::npos);
    assert(entities_text.find("7,segment,Dr Octopus Christian,Mount,0.01,0.02,0.03") !=
           std::string::npos);

    return 0;
}
