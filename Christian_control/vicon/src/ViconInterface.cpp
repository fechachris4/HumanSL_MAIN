#include "ViconInterface.h"

#include <chrono>
#include <iostream>
#include <thread>

using namespace ViconDataStreamSDK::CPP;

ViconInterface::ViconInterface()
    : client_(std::make_unique<Client>()), connected_(false) {
}

ViconInterface::~ViconInterface() {
    if (connected_) {
        disconnect();
    }
}

bool ViconInterface::connect(const std::string& hostname) {
    hostname_ = hostname;

    std::cout << "Connecting to Vicon at " << hostname << "..." << std::endl;
    auto output = client_->Connect(hostname);

    if (output.Result != Result::Success) {
        std::cerr << "Failed to connect to Vicon system: " << output.Result << std::endl;
        return false;
    }

    connected_ = true;

    client_->EnableSegmentData();
    client_->EnableMarkerData();
    client_->EnableUnlabeledMarkerData();
    client_->EnableDeviceData();

    // Wait for the first frame, but not forever: a connection that opens
    // and then delivers no frames (server not streaming, firewall) must
    // fail loudly, not hang.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (client_->GetFrame().Result != Result::Success) {
        if (std::chrono::steady_clock::now() >= deadline) {
            std::cerr << "Connected to " << hostname
                      << " but no frame arrived within 5 s — is Vicon "
                         "streaming, and is port 801 open?" << std::endl;
            disconnect();
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "Connected to Vicon successfully!" << std::endl;
    return true;
}

void ViconInterface::disconnect() {
    if (connected_) {
        client_->Disconnect();
        connected_ = false;
        std::cout << "Disconnected from Vicon" << std::endl;
    }
}

bool ViconInterface::getFrame() {
    if (!connected_) return false;
    return client_->GetFrame().Result == Result::Success;
}

int ViconInterface::getFrameNumber() {
    if (!connected_) return 0;
    return client_->GetFrameNumber().FrameNumber;
}

double ViconInterface::getFrameRate() const {
    if (!connected_) return 0.0;

    Output_GetFrameRate frameRateOutput = client_->GetFrameRate();
    if (frameRateOutput.Result == Result::Success) {
        return frameRateOutput.FrameRateHz;
    }

    return 0.0;
}

MarkerData ViconInterface::getMarkerPosition(const std::string& markerName) {
    MarkerData marker;
    marker.name = markerName;
    marker.occluded = true; // Default to occluded if not found

    if (!connected_) return marker;

    unsigned int subjectCount = client_->GetSubjectCount().SubjectCount;

    // Search through all subjects for the marker
    for (unsigned int i = 0; i < subjectCount; ++i) {
        std::string subjectName = client_->GetSubjectName(i).SubjectName;
        unsigned int markerCount = client_->GetMarkerCount(subjectName).MarkerCount;

        for (unsigned int j = 0; j < markerCount; ++j) {
            std::string currentMarkerName = client_->GetMarkerName(subjectName, j).MarkerName;
            if (currentMarkerName == markerName) {
                Output_GetMarkerGlobalTranslation translation =
                    client_->GetMarkerGlobalTranslation(subjectName, markerName);
                if (translation.Result == Result::Success) {
                    marker.x = translation.Translation[0];
                    marker.y = translation.Translation[1];
                    marker.z = translation.Translation[2];
                    marker.occluded = translation.Occluded;
                    return marker;
                }
            }
        }
    }

    return marker; // Return with occluded=true if marker not found
}

std::vector<MarkerData> ViconInterface::getMarkerPositions(const std::vector<std::string>& markerNames) {
    std::vector<MarkerData> markers;
    markers.reserve(markerNames.size());

    for (const std::string& markerName : markerNames) {
        markers.push_back(getMarkerPosition(markerName));
    }

    return markers;
}

std::vector<MarkerData> ViconInterface::getMarkerPositions(const std::string& prefix) {
    std::vector<MarkerData> markers;

    if (!connected_) return markers;

    unsigned int subjectCount = client_->GetSubjectCount().SubjectCount;

    // Search through all subjects for markers starting with the prefix
    for (unsigned int i = 0; i < subjectCount; ++i) {
        std::string subjectName = client_->GetSubjectName(i).SubjectName;
        unsigned int markerCount = client_->GetMarkerCount(subjectName).MarkerCount;

        for (unsigned int j = 0; j < markerCount; ++j) {
            std::string currentMarkerName = client_->GetMarkerName(subjectName, j).MarkerName;

            if (currentMarkerName.substr(0, prefix.length()) == prefix) {
                Output_GetMarkerGlobalTranslation translation =
                    client_->GetMarkerGlobalTranslation(subjectName, currentMarkerName);

                if (translation.Result == Result::Success) {
                    MarkerData marker;
                    marker.name = currentMarkerName;
                    marker.x = translation.Translation[0];
                    marker.y = translation.Translation[1];
                    marker.z = translation.Translation[2];
                    marker.occluded = translation.Occluded;
                    markers.push_back(marker);
                }
            }
        }
    }

    return markers;
}

std::vector<UnlabeledMarkerData> ViconInterface::getUnlabeledMarkers() {
    std::vector<UnlabeledMarkerData> unlabeledMarkers;

    if (!connected_) return unlabeledMarkers;

    unsigned int unlabeledMarkerCount = client_->GetUnlabeledMarkerCount().MarkerCount;

    for (unsigned int i = 0; i < unlabeledMarkerCount; ++i) {
        Output_GetUnlabeledMarkerGlobalTranslation translation =
            client_->GetUnlabeledMarkerGlobalTranslation(i);

        if (translation.Result == Result::Success) {
            UnlabeledMarkerData marker;
            marker.x = translation.Translation[0];
            marker.y = translation.Translation[1];
            marker.z = translation.Translation[2];
            marker.id = i;

            unlabeledMarkers.push_back(marker);
        }
    }

    return unlabeledMarkers;
}

std::vector<SegmentData> ViconInterface::getSegmentPoses() {
    std::vector<SegmentData> segments;
    if (!connected_) return segments;

    const auto subject_count = client_->GetSubjectCount();
    if (subject_count.Result != Result::Success) return segments;

    for (unsigned int subject_index = 0; subject_index < subject_count.SubjectCount; ++subject_index) {
        const auto subject = client_->GetSubjectName(subject_index);
        if (subject.Result != Result::Success) continue;

        const auto segment_count = client_->GetSegmentCount(subject.SubjectName);
        if (segment_count.Result != Result::Success) continue;

        for (unsigned int segment_index = 0; segment_index < segment_count.SegmentCount; ++segment_index) {
            const auto segment_name = client_->GetSegmentName(subject.SubjectName, segment_index);
            if (segment_name.Result != Result::Success) continue;

            const auto translation =
                client_->GetSegmentGlobalTranslation(subject.SubjectName, segment_name.SegmentName);
            const auto rotation =
                client_->GetSegmentGlobalRotationQuaternion(subject.SubjectName, segment_name.SegmentName);
            if (translation.Result != Result::Success || rotation.Result != Result::Success) continue;

            SegmentData segment;
            segment.subject_name = subject.SubjectName;
            segment.segment_name = segment_name.SegmentName;
            segment.x = translation.Translation[0];
            segment.y = translation.Translation[1];
            segment.z = translation.Translation[2];
            segment.qx = rotation.Rotation[0];
            segment.qy = rotation.Rotation[1];
            segment.qz = rotation.Rotation[2];
            segment.qw = rotation.Rotation[3];
            segment.occluded = translation.Occluded || rotation.Occluded;
            segments.push_back(segment);
        }
    }

    return segments;
}

void ViconInterface::getForcePlateVector(std::vector<double>& fplate_left, std::vector<double>& fplate_right) {
    auto force1 = client_->GetGlobalForceVector(0);
    auto force2 = client_->GetGlobalForceVector(1);

    fplate_left[0] = force2.ForceVector[0];
    fplate_left[1] = force2.ForceVector[1];
    fplate_left[2] = force2.ForceVector[2];

    fplate_right[0] = force1.ForceVector[0];
    fplate_right[1] = force1.ForceVector[1];
    fplate_right[2] = force1.ForceVector[2];
}
