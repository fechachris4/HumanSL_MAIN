#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <deque>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <thread>
#include <yaml-cpp/yaml.h>

#include "DataStreamClient.h"
#include <string>
#include <vector>
#include <memory>
#include <chrono>


// USER GUIDE 
// 1. In vicon, create a subject
// 2. Create a segment called "right_base", then start by clicking 
// the middle marker of the 3 markers on the right robot base (This is important)
// then click the other 2 in any order 
// 3. Create another segment called "left_base", repeat the process.
// 4. Create a segment called "ref_base", click on the 4 markers on the 3d print in any order
// 5. link them with free_joint.
// 6. Run this file, note that the joint angles needs to be known and set in main().
// 7. The left/right end effector pose should be printed into the console.


struct MarkerData {
    std::string name;
    double x, y, z;
    bool occluded;
};

class ViconInterface {

    private:
        std::unique_ptr<ViconDataStreamSDK::CPP::Client> client_;
        bool connected_;
        std::string hostname_;
        
    public:
        ViconInterface();
        ~ViconInterface();
        
        // Connection management
        bool connect(const std::string& hostname = "localhost:801");
        void disconnect();
        bool isConnected() const { return connected_; }
        
        // Data acquisition
        bool getFrame();
        int getFrameNumber();

        MarkerData getMarkerPosition(const std::string& markerName);
};

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
    
    // Enable marker data and unlabeled marker data
    if (connected_) {
        client_->EnableMarkerData();
        client_->EnableUnlabeledMarkerData();
        client_->EnableDeviceData();
    }
    
    // Wait for first frame
    while (client_->GetFrame().Result != Result::Success) {
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
    if (!connected_) return false;
    return client_->GetFrameNumber().FrameNumber;
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
            // std::cout << currentMarkerName << "\n";
            if (currentMarkerName == markerName) {
                Output_GetMarkerGlobalTranslation translation = 
                    client_->GetMarkerGlobalTranslation(subjectName, markerName);
                if(translation.Result == Result::Success){
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


// Utility functions for pose operations using Eigen
namespace PoseUtils {

    // Create a 4x4 transformation matrix from rotation matrix and translation
    Eigen::Matrix4d createPose(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation) {
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose.block<3, 3>(0, 0) = rotation;
        pose.block<3, 1>(0, 3) = translation;
        return pose;
    }

    // Extract rotation matrix from 4x4 transformation
    Eigen::Matrix3d getRotation(const Eigen::Matrix4d& pose) {
        return pose.block<3, 3>(0, 0);
    }

    // Extract translation vector from 4x4 transformation
    Eigen::Vector3d getTranslation(const Eigen::Matrix4d& pose) {
        return pose.block<3, 1>(0, 3);
    }

    // Compute inverse of a transformation matrix
    Eigen::Matrix4d inversePose(const Eigen::Matrix4d& pose) {
        Eigen::Matrix4d inv = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d R = getRotation(pose);
        Eigen::Vector3d t = getTranslation(pose);
        inv.block<3, 3>(0, 0) = R.transpose();
        inv.block<3, 1>(0, 3) = -R.transpose() * t;
        return inv;
    }

    // Convert rotation matrix to axis-angle representation
    Eigen::Vector3d rotationToAxisAngle(const Eigen::Matrix3d& R) {
        Eigen::AngleAxisd angleAxis(R);
        return angleAxis.angle() * angleAxis.axis();
    }

    // Convert axis-angle to rotation matrix
    Eigen::Matrix3d axisAngleToRotation(const Eigen::Vector3d& axisAngle) {
        double angle = axisAngle.norm();
        if (angle < 1e-10) {
            return Eigen::Matrix3d::Identity();
        }
        Eigen::Vector3d axis = axisAngle / angle;
        Eigen::AngleAxisd angleAxis(angle, axis);
        return angleAxis.toRotationMatrix();
    }
}


// Struct to store DH param information
struct DHParameters {
    Eigen::VectorXd a;
    Eigen::VectorXd alpha;
    Eigen::VectorXd d;
    Eigen::VectorXd theta;
};

// Populate the DHParameters struct with yaml configuration file
DHParameters createDHParams(const std::string& yaml_path) {

    DHParameters dh_params;
    YAML::Node config = YAML::LoadFile(yaml_path);

    // Initialize vectors for 7 DOF
    dh_params.a = Eigen::VectorXd::Zero(7);
    dh_params.alpha = Eigen::VectorXd::Zero(7);
    dh_params.d = Eigen::VectorXd::Zero(7);
    dh_params.theta = Eigen::VectorXd::Zero(7);

    // Load DH parameters from YAML
    if (config["dh_parameters"]) {
        for (const auto& joint : config["dh_parameters"]) {
            int joint_id = joint["joint_id"].as<int>();
            // Convert to 0-based indexing
            int index = joint_id - 1;

            if (index >= 0 && index < 7) {
                dh_params.a(index) = joint["a"].as<double>();
                dh_params.alpha(index) = joint["alpha"].as<double>();
                dh_params.d(index) = joint["d"].as<double>();
                dh_params.theta(index) = joint["theta_offset"].as<double>();
            }
        }
    }

    return dh_params;
}

// Create DH transformation matrix
Eigen::Matrix4d createDHTransform(double a, double alpha, double d, double theta) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    double cos_alpha = cos(alpha);
    double sin_alpha = sin(alpha);

    // Standard DH transformation: Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
    T(0, 0) = cos_theta;
    T(0, 1) = -sin_theta * cos_alpha;
    T(0, 2) = sin_theta * sin_alpha;
    T(0, 3) = a * cos_theta;

    T(1, 0) = sin_theta;
    T(1, 1) = cos_theta * cos_alpha;
    T(1, 2) = -cos_theta * sin_alpha;
    T(1, 3) = a * sin_theta;

    T(2, 0) = 0;
    T(2, 1) = sin_alpha;
    T(2, 2) = cos_alpha;
    T(2, 3) = d;

    T(3, 0) = 0;
    T(3, 1) = 0;
    T(3, 2) = 0;
    T(3, 3) = 1;

    return T;
}
// Compute forward kinematics from base to end effector
Eigen::Matrix4d computeBaseToEE(const DHParameters& dh, const Eigen::VectorXd& joint_angles) {
    // Start with identity transformation
    Eigen::Matrix4d T_base_to_ee = Eigen::Matrix4d::Identity();

    // Chain all DH transformations from base to end effector
    for (int i = 0; i < 7; i++) {
        // Actual theta = theta_offset + joint_angle
        double theta_i = dh.theta(i) + joint_angles(i);

        // Create DH transformation for joint i
        Eigen::Matrix4d T_i = createDHTransform(dh.a(i), dh.alpha(i), dh.d(i), theta_i);

        // Compose with previous transformations
        T_base_to_ee = T_base_to_ee * T_i;
    }

    return T_base_to_ee;
}

// Forward kinematics: base_pose_in_world * T_base_to_ee -> ee_pose_in_world
Eigen::Matrix4d fk(const DHParameters& dh,
                                  const Eigen::VectorXd& joint_angles,
                                  const Eigen::Matrix4d& base_pose_in_world) {
    // Compute transformation from base to end effector
    Eigen::Matrix4d T_base_to_ee = computeBaseToEE(dh, joint_angles);

    // Transform to world frame: T_world_to_ee = T_world_to_base * T_base_to_ee
    return base_pose_in_world * T_base_to_ee;
}

// Calculates world_frame pose given 4 marker in world space
// p1,p2,p3 defines a circle in the XY-plane of the local frame, 
// the center of the circle is the origin in world frame.
// p1 needs to be in positive X-axis of the local frame
// p4 determines which direction z-axis points in, set it accordingly as local frame definition.
// The z-offset translates the local frame around its z-axis, meant for aligning in world frame.
Eigen::Matrix4d calculateFramePose(const Eigen::Vector3d& world_p1,
                                  const Eigen::Vector3d& world_p2,
                                  const Eigen::Vector3d& world_p3,
                                  const Eigen::Vector3d& world_p4,
                                  double z_offset, bool p1_in_positive_x, bool p4_in_positive_z) {

    // Find circle center (circumcenter of triangle P1,P2,P3)
    Eigen::Vector3d v1 = world_p2 - world_p1;
    Eigen::Vector3d v2 = world_p3 - world_p1;
    Eigen::Vector3d normal = v1.cross(v2);

    // Create orthonormal basis for the plane
    Eigen::Vector3d u1 = v1.normalized();
    Eigen::Vector3d u2 = (v2 - v2.dot(u1) * u1).normalized();

    // Project points onto 2D plane coordinates
    Eigen::Vector2d a(0, 0);  // p1 at origin
    Eigen::Vector2d b(v1.norm(), 0);  // p2 on u1 axis
    Eigen::Vector2d c(v2.dot(u1), v2.dot(u2));  // p3 in plane

    // Calculate 2D circumcenter
    double d = 2 * (a.x() * (b.y() - c.y()) + b.x() * (c.y() - a.y()) + c.x() * (a.y() - b.y()));

    Eigen::Vector3d world_center;
  
    double ux = ((a.x()*a.x() + a.y()*a.y()) * (b.y() - c.y()) +
                    (b.x()*b.x() + b.y()*b.y()) * (c.y() - a.y()) +
                    (c.x()*c.x() + c.y()*c.y()) * (a.y() - b.y())) / d;

    double uy = ((a.x()*a.x() + a.y()*a.y()) * (c.x() - b.x()) +
                    (b.x()*b.x() + b.y()*b.y()) * (a.x() - c.x()) +
                    (c.x()*c.x() + c.y()*c.y()) * (b.x() - a.x())) / d;

    // Convert back to 3D
    world_center = world_p1 + ux * u1 + uy * u2;
    

    // Calculate plane normal (two possible directions)
    Eigen::Vector3d plane_normal = v1.cross(v2).normalized();

    // Use P4 to determine correct z-axis direction
    Eigen::Vector3d center_to_p4 = world_p4 - world_center;
    double dot_product = center_to_p4.dot(plane_normal);

    // If P4 is on the positive side of the plane, flip the normal
    Eigen::Vector3d world_z_axis;

    if(p4_in_positive_z) {
        world_z_axis = (dot_product > 0) ? plane_normal : -plane_normal;
    }
    else {
        world_z_axis = (dot_product > 0) ? -plane_normal : plane_normal;
    }

    // Calculate world x-axis (from center to P1)
    Eigen::Vector3d world_x_axis;
    if (p1_in_positive_x) {
        world_x_axis = (world_p1 - world_center).normalized();
    } else {
        // If P1 is not in positive X, flip the x-axis direction
        world_x_axis = -(world_p1 - world_center).normalized();
    }

    // Calculate world y-axis (z cross x for right-handed orthogonal frame)
    Eigen::Vector3d world_y_axis = world_z_axis.cross(world_x_axis).normalized();

    // Build rotation matrix
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix.col(0) = world_x_axis;
    rotation_matrix.col(1) = world_y_axis;
    rotation_matrix.col(2) = world_z_axis;

    // Create transformation matrix
    Eigen::Vector3d translation_with_z_offset = world_center + (z_offset * world_z_axis);

    return PoseUtils::createPose(rotation_matrix, translation_with_z_offset);
}


// Finds the frame given base frame markers
Eigen::Matrix4d updatePoseInfo2(std::vector<MarkerData>& vicon_data, MarkerData other_arm_base_1) {
    std::vector<Eigen::Vector3d> base_positions;

    for (const auto& marker : vicon_data) {
        base_positions.push_back(Eigen::Vector3d(
            marker.x/1000, marker.y/1000, marker.z/1000));
    }
    Eigen::Vector3d other_arm_point(other_arm_base_1.x/1000, other_arm_base_1.y/1000, other_arm_base_1.z/1000);

    Eigen::Matrix4d base_guess = calculateFramePose(base_positions[0],
        base_positions[1], base_positions[2],
        other_arm_point, 0.133, false, true);

    return base_guess;
}


// Finds the the fixed transformation from reference frame to left/right robot base frame
// 1. Takes 100 readings of the fixed transformation, 
// 2. Applies median aboslute deviation filter to filter outliers, 
// 3. Averages the rest to get the final transformation
std::pair<Eigen::Matrix4d, Eigen::Matrix4d> calibrate_fixed_transformation(ViconInterface& vicon){
    
    std::vector<MarkerData> right_base_data;
    std::vector<MarkerData> left_base_data;
    std::vector<MarkerData> ref_base_data;

    bool right_base_occluded = false;
    bool left_base_occluded = false;
    bool ref_base_occluded = false;
    bool set_ref_to_base = false;

    // Static variables for averaging
    std::deque<Eigen::Matrix4d> ref_to_left_array;
    std::deque<Eigen::Matrix4d> ref_to_right_array;
    std::deque<Eigen::Matrix4d> ref_base_array;

    Eigen::Matrix4d avg_ref_base = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d ref_to_left = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d ref_to_right = Eigen::Matrix4d::Identity();

    Eigen::Matrix4d left_base = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d right_base = Eigen::Matrix4d::Identity();

    int prev_frame_number = -1;

    while (true) {
        
        int cur_frame_number = vicon.getFrameNumber();
        // std::cout << "Vicon frame number: " << cur_frame_number <<"\n";

        if (cur_frame_number == prev_frame_number){
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        else{
            prev_frame_number = cur_frame_number;
        }

        if (!vicon.getFrame()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        ref_base_data.clear();
        right_base_data.clear();
        left_base_data.clear();

        ref_base_data.push_back(vicon.getMarkerPosition("ref_base1")); 
        ref_base_data.push_back(vicon.getMarkerPosition("ref_base2"));
        ref_base_data.push_back(vicon.getMarkerPosition("ref_base3"));
        ref_base_data.push_back(vicon.getMarkerPosition("ref_base4"));

        right_base_data.push_back(vicon.getMarkerPosition("right_base1"));
        right_base_data.push_back(vicon.getMarkerPosition("right_base2"));
        right_base_data.push_back(vicon.getMarkerPosition("right_base3"));

        left_base_data.push_back(vicon.getMarkerPosition("left_base1"));
        left_base_data.push_back(vicon.getMarkerPosition("left_base2"));
        left_base_data.push_back(vicon.getMarkerPosition("left_base3"));


        for (const auto& marker : right_base_data) {
            if (marker.occluded) {
                right_base_occluded = true;
                break;
            }
        }

        for (const auto& marker : left_base_data) {
            if (marker.occluded) {
                left_base_occluded = true;
                break;
            }
        }

        for (const auto& marker : ref_base_data) {
            if (marker.occluded) {
                ref_base_occluded = true;
                break;
            }
        }

        // Estimate base poses if not occluded
        Eigen::Matrix4d left_base_current, right_base_current;
        Eigen::Matrix4d ref_base_current;
        Eigen::Matrix4d ref_to_left_current;
        Eigen::Matrix4d ref_to_right_current;

        if (!left_base_occluded && !right_base_occluded && !ref_base_occluded) {
            left_base_current = updatePoseInfo2(left_base_data, right_base_data.front());
            right_base_current = updatePoseInfo2(right_base_data, left_base_data.front());
            ref_base_current = updatePoseInfo2(ref_base_data, ref_base_data.back());

            ref_to_right_current = PoseUtils::inversePose(ref_base_current) * right_base_current;
            ref_to_left_current =  PoseUtils::inversePose(ref_base_current) * left_base_current;

            ref_to_left_array.push_back(ref_to_left_current);
            ref_to_right_array.push_back(ref_to_right_current);
        }

        // Calculate moving average poses
        if (ref_to_left_array.size() >= 100 && !set_ref_to_base) {

            // Apply Median Absolute Deviation filtering on translational components
            std::vector<double> left_x_vals, left_y_vals, left_z_vals;
            for (const auto& pose : ref_to_left_array) {
                Eigen::Vector3d translation = PoseUtils::getTranslation(pose);
                left_x_vals.push_back(translation.x());
                left_y_vals.push_back(translation.y());
                left_z_vals.push_back(translation.z());
            }

            // Sort to find median
            auto getMedian = [](std::vector<double> vals) -> double {
                std::sort(vals.begin(), vals.end());
                size_t n = vals.size();
                if (n % 2 == 0) {
                    return (vals[n/2 - 1] + vals[n/2]) / 2.0;
                } else {
                    return vals[n/2];
                }
            };

            double left_median_x = getMedian(left_x_vals);
            double left_median_y = getMedian(left_y_vals);
            double left_median_z = getMedian(left_z_vals);

            // Calculate MAD for each dimension
            std::vector<double> left_abs_dev_x, left_abs_dev_y, left_abs_dev_z;
            for (const auto& pose : ref_to_left_array) {
                Eigen::Vector3d translation = PoseUtils::getTranslation(pose);
                left_abs_dev_x.push_back(std::abs(translation.x() - left_median_x));
                left_abs_dev_y.push_back(std::abs(translation.y() - left_median_y));
                left_abs_dev_z.push_back(std::abs(translation.z() - left_median_z));
            }

            double left_mad_x = getMedian(left_abs_dev_x) * 1.4826;
            double left_mad_y = getMedian(left_abs_dev_y) * 1.4826;
            double left_mad_z = getMedian(left_abs_dev_z) * 1.4826;

            // Filter ref_to_left_array based on MAD threshold
            double mad_threshold = 2.0;
            std::deque<Eigen::Matrix4d> filtered_ref_to_left_array;
            for (const auto& pose : ref_to_left_array) {
                Eigen::Vector3d translation = PoseUtils::getTranslation(pose);
                bool is_outlier = (std::abs(translation.x() - left_median_x) > mad_threshold * left_mad_x) ||
                                (std::abs(translation.y() - left_median_y) > mad_threshold * left_mad_y) ||
                                (std::abs(translation.z() - left_median_z) > mad_threshold * left_mad_z);
                if (!is_outlier) {
                    filtered_ref_to_left_array.push_back(pose);
                }
            }

            // Apply MAD filtering for ref_to_right_array
            std::vector<double> right_x_vals, right_y_vals, right_z_vals;
            for (const auto& pose : ref_to_right_array) {
                Eigen::Vector3d translation = PoseUtils::getTranslation(pose);
                right_x_vals.push_back(translation.x());
                right_y_vals.push_back(translation.y());
                right_z_vals.push_back(translation.z());
            }

            double right_median_x = getMedian(right_x_vals);
            double right_median_y = getMedian(right_y_vals);
            double right_median_z = getMedian(right_z_vals);

            // Calculate MAD for each dimension
            std::vector<double> right_abs_dev_x, right_abs_dev_y, right_abs_dev_z;
            for (const auto& pose : ref_to_right_array) {
                Eigen::Vector3d translation = PoseUtils::getTranslation(pose);
                right_abs_dev_x.push_back(std::abs(translation.x() - right_median_x));
                right_abs_dev_y.push_back(std::abs(translation.y() - right_median_y));
                right_abs_dev_z.push_back(std::abs(translation.z() - right_median_z));
            }

            double right_mad_x = getMedian(right_abs_dev_x) * 1.4826;
            double right_mad_y = getMedian(right_abs_dev_y) * 1.4826;
            double right_mad_z = getMedian(right_abs_dev_z) * 1.4826;

            // Filter ref_to_right_array based on MAD threshold
            std::deque<Eigen::Matrix4d> filtered_ref_to_right_array;
            for (const auto& pose : ref_to_right_array) {
                Eigen::Vector3d translation = PoseUtils::getTranslation(pose);
                bool is_outlier = (std::abs(translation.x() - right_median_x) > mad_threshold * right_mad_x) ||
                                (std::abs(translation.y() - right_median_y) > mad_threshold * right_mad_y) ||
                                (std::abs(translation.z() - right_median_z) > mad_threshold * right_mad_z);
                if (!is_outlier) {
                    filtered_ref_to_right_array.push_back(pose);
                }
            }

            // Replace original arrays with filtered versions
            ref_to_left_array = std::move(filtered_ref_to_left_array);
            ref_to_right_array = std::move(filtered_ref_to_right_array);

            std::cout << "ref to right size: " << ref_to_right_array.size() << "\n";
            std::cout << "ref to left size: " << ref_to_left_array.size() << "\n";

            // Calculate average left transform
            Eigen::Vector3d avg_left_translation = Eigen::Vector3d::Zero();
            Eigen::Vector3d avg_left_rotation_vector = Eigen::Vector3d::Zero();

            for (const auto& pose : ref_to_left_array) {
                avg_left_translation += PoseUtils::getTranslation(pose);
                Eigen::Vector3d rotation_vector = PoseUtils::rotationToAxisAngle(PoseUtils::getRotation(pose));
                avg_left_rotation_vector += rotation_vector;
            }

            avg_left_translation /= ref_to_left_array.size();
            avg_left_rotation_vector /= ref_to_left_array.size();
            Eigen::Matrix3d avg_left_rotation = PoseUtils::axisAngleToRotation(avg_left_rotation_vector);

            ref_to_left = PoseUtils::createPose(avg_left_rotation, avg_left_translation);

            // Calculate average right transform
            Eigen::Vector3d avg_right_translation = Eigen::Vector3d::Zero();
            Eigen::Vector3d avg_right_rotation_vector = Eigen::Vector3d::Zero();

            for (const auto& pose : ref_to_right_array) {
                avg_right_translation += PoseUtils::getTranslation(pose);
                Eigen::Vector3d rotation_vector = PoseUtils::rotationToAxisAngle(PoseUtils::getRotation(pose));
                avg_right_rotation_vector += rotation_vector;
            }

            avg_right_translation /= ref_to_right_array.size();
            avg_right_rotation_vector /= ref_to_right_array.size();
            Eigen::Matrix3d avg_right_rotation = PoseUtils::axisAngleToRotation(avg_right_rotation_vector);

            ref_to_right = PoseUtils::createPose(avg_right_rotation, avg_right_translation);

            std::cout << "\n Ref to right: \n";
            std::cout << ref_to_right << "\n";

            std::cout << "\n Ref to left: \n";
            std::cout << ref_to_left << "\n";

            std::cout << "\n Ref avg: \n";
            std::cout << avg_ref_base << "\n";

            std::cout << "\n left cur: \n";
            std::cout << left_base_current << "\n";

            std::cout << "\n right cur: \n";
            std::cout << right_base_current << "\n";

            std::cout << "\n Ref cur: \n";
            std::cout << ref_base_current << "\n";

            // Return the calibrated transformations
            return std::make_pair(ref_to_left, ref_to_right);
        }
    }
}


// Updates the left/right robot base frame given the fixed transformation and reference frame readings
void get_robot_base_frames(ViconInterface& vicon, Eigen::Matrix4d& ref_base, Eigen::Matrix4d& left_base, Eigen::Matrix4d& right_base, Eigen::Matrix4d& ref_to_left, Eigen::Matrix4d& ref_to_right){
    
    std::vector<MarkerData> ref_base_data;

    ref_base_data.push_back(vicon.getMarkerPosition("ref_base1")); 
    ref_base_data.push_back(vicon.getMarkerPosition("ref_base2"));
    ref_base_data.push_back(vicon.getMarkerPosition("ref_base3"));
    ref_base_data.push_back(vicon.getMarkerPosition("ref_base4"));

    bool ref_base_occluded = false;

    for (const auto& marker : ref_base_data) {
        if (marker.occluded) {
            ref_base_occluded = true;
            break;
        }
    }

    if (!ref_base_occluded) {
        ref_base =  updatePoseInfo2(ref_base_data, ref_base_data.back());
        left_base = ref_base * ref_to_left;
        right_base = ref_base * ref_to_right;
    }
}


// Example implementation
// The joint angles are hard set, incoporate kinova joint feedback for  
// real time end-effector pose update 
int main(){

    ViconInterface vicon;

    if (!vicon.connect("192.168.128.206")) {
        std::cerr << "Failed to connect to Vicon system. Exiting." << std::endl;
        return -1;
    }

    Eigen::Matrix4d ref_base = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d left_base = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d right_base = Eigen::Matrix4d::Identity();

    Eigen::Matrix4d ref_to_left, ref_to_right;
    auto transformation_pair = calibrate_fixed_transformation(vicon);
    ref_to_left = transformation_pair.first;
    ref_to_right = transformation_pair.second;

    int prev_frame_number = -1;

    DHParameters dh = createDHParams("../dh_params.yaml");
    
    Eigen::VectorXd q_init_left(7); 
    q_init_left << -90,90,-15,45,5,5,-175;  // in deg
    q_init_left = q_init_left * (M_PI/180);
    
    Eigen::VectorXd q_init_right(7);
    q_init_right << 90,90,15,45,5,5,5; // in deg
    q_init_right = q_init_right * (M_PI/180);

    while (true) {
        
        int cur_frame_number = vicon.getFrameNumber();
        // std::cout << "Vicon frame number: " << cur_frame_number <<"\n";

        if (cur_frame_number == prev_frame_number){
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        else{
            prev_frame_number = cur_frame_number;
        }

        if (!vicon.getFrame()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }


        get_robot_base_frames(vicon, ref_base, left_base, right_base, ref_to_left, ref_to_right);

        Eigen::Matrix4d left_ee = fk(dh,q_init_left,left_base);
        Eigen::Matrix4d right_ee = fk(dh, q_init_right, right_base);

        std::cout << "\n Left EE: \n";
        std::cout << left_ee << "\n";

        std::cout << "\n Right EE: \n";
        std::cout << right_ee << "\n";

    }

    return 0;
}

