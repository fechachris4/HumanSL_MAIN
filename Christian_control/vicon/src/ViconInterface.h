#pragma once

// Ported 2026-08-10 from ViconDataStream/include/ViconInterface.h so the
// Vicon connection can build and run standalone. Same class, same calls;
// the only behaviour change is a bounded first-frame wait in connect()
// (the legacy version waited forever).

#include "DataStreamClient.h"

#include <memory>
#include <string>
#include <vector>

// One labeled marker in the Vicon world frame, millimetres. occluded means
// the cameras could not see it this frame (or the name was not found), and
// x/y/z are then meaningless.
struct MarkerData {
    std::string name;
    double x = 0.0, y = 0.0, z = 0.0;
    bool occluded = true;
};

// A marker the system sees but has not matched to a subject. Millimetres.
struct UnlabeledMarkerData {
    double x = 0.0, y = 0.0, z = 0.0;
    unsigned int id = 0;
};

// One subject segment pose in the Vicon world frame. Translation is
// millimetres; orientation uses the Vicon SDK quaternion order (x, y, z, w).
// occluded means the cameras could not determine the pose this frame, so its
// numeric values must not be used.
struct SegmentData {
    std::string subject_name;
    std::string segment_name;
    double x = 0.0, y = 0.0, z = 0.0;
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    bool occluded = true;
};

class ViconInterface {
public:
    ViconInterface();
    ~ViconInterface();

    // Connection management. hostname is "ip:port"; Vicon's DataStream
    // server listens on port 801.
    bool connect(const std::string& hostname = "localhost:801");
    void disconnect();
    bool isConnected() const { return connected_; }

    // Data acquisition. getFrame() pulls the next frame into the client;
    // every get* method below reads from that frame.
    bool getFrame();
    int getFrameNumber();

    MarkerData getMarkerPosition(const std::string& markerName);
    std::vector<MarkerData> getMarkerPositions(const std::vector<std::string>& markerNames);
    std::vector<MarkerData> getMarkerPositions(const std::string& prefix);
    std::vector<UnlabeledMarkerData> getUnlabeledMarkers();
    std::vector<SegmentData> getSegmentPoses();

    // Camera system frame rate in Hz, as reported by the server.
    double getFrameRate() const;

    void getForcePlateVector(std::vector<double>& fplate_left, std::vector<double>& fplate_right);

private:
    std::unique_ptr<ViconDataStreamSDK::CPP::Client> client_;
    bool connected_;
    std::string hostname_;
};
