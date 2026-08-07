#ifndef TRAJECTORY_INITIATION_H
#define TRAJECTORY_INITIATION_H
#include "spline.h"

#include <vector>
#include <cmath>
#include <limits>
#include <iostream>
#include <stdexcept>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <Eigen/Dense>
#include <memory>
#include "utils.h"


// What an IK attempt produced, including when it did not succeed. `solved`
// is the old boolean; the rest describe the BEST CONFIGURATION THE SOLVER
// REACHED even when that configuration was rejected. A rejected result is
// still useful: a pose 6 cm out is a far better starting sketch for the
// optimiser than standing still, and the two errors say WHICH half of the
// pose was the obstacle. Read `config` and the errors only when `attempted`
// is true.
struct IKAttempt {
    bool solved = false;        // a solution passed every acceptance test
    bool attempted = false;     // the solver ran and produced a candidate
    gtsam::Vector config;       // best configuration reached (7, radians)
    double position_error_m = std::numeric_limits<double>::infinity();
    double orientation_error_rad = std::numeric_limits<double>::infinity();
};

// How the waypoint sketch handed to the optimiser was built. The optimiser
// works from a starting guess and has its own goal term, so a poor guess
// costs quality, not correctness — but the operator must be told which one
// they got, because a plan built from kHeldStart deserves more scrutiny
// than one built from a solved pose.
enum class InitSource {
    kSolvedIk,    // a fully accepted IK solution
    kNearMiss,    // rejected by the acceptance test, but the closest reached
    kHeldStart    // no usable candidate; every waypoint at the start config
};

// One initialisation, with the provenance the operator needs to judge it.
struct TrajectoryInit {
    gtsam::Values values;
    InitSource source = InitSource::kSolvedIk;
    double position_error_m = 0.0;      // of the end configuration used
    double orientation_error_rad = 0.0; // (both zero for kHeldStart)
};

class InitializeTrajectory {
private:

    DHParameters dh_params_;
    // Which arm's chain solveIK's Pinocchio-backed FK targets — must match
    // the arm dh_params_ was generated from (see utils.h forwardKinematics).
    // Defaults (constructor) = right arm, mounted tool: every
    // pre-left-arm caller unchanged.
    std::string end_effector_frame_;
    bool left_arm_;

    // Returns the best configuration the solver reached, whether or not it
    // passed — see IKAttempt. Callers that only care about success read
    // .solved; the fallback path reads the rest.
    IKAttempt solveIK(const gtsam::Pose3& target_pose,
                const gtsam::Pose3& base_pose,
                const gtsam::Vector& seed_config,
                int max_attempts = 10,
                double guess_range = 0.5);
    
    bool solveQuik(const gtsam::Pose3& target_pose, 
                    const gtsam::Pose3& base_pose,
                    const gtsam::Vector& seed_config,
                    gtsam::Vector& result_config,
                    int max_attempts = 10,
                    double guess_range = 0.5);
                                
    void wrapAngles(gtsam::Vector& angles, const gtsam::Vector& reference); 

    void wrapAngles(Eigen::Vector<double,7>& angles, const Eigen::Vector<double,7>& reference);

public:
    InitializeTrajectory(DHParameters dh_params,
                         std::string end_effector_frame = "ConfiguredTool_Link",
                         bool left_arm = false);

    // Never throws on IK failure. IK here only produces the STARTING SKETCH
    // the optimiser refines; the optimiser carries its own goal term, so a
    // failed IK is a worse guess, not an impossible plan. When no pose is
    // accepted this falls back to the closest near miss, or to holding the
    // start configuration, and reports which via TrajectoryInit::source so
    // the caller can describe the plan's quality honestly rather than
    // silently treating all plans alike.
    TrajectoryInit initJointTrajectoryFromTarget(
                                        const gtsam::Vector& start_conf,
                                    const gtsam::Pose3& end_pose,
                                const gtsam::Pose3& base_pose,
                            const size_t total_time_step);
                            
    gtsam::Values initJointTrajectoryFromVicon(
                                        const gtsam::Vector& start_conf,
                                        const TubeInfo& tube_info,
                                        double offset_from_base_y,
                                        double offset_from_tube_z,
                                        const gtsam::Pose3& base_pose,
                                        const size_t total_time_step,
                                        gtsam::Pose3& best_end_pose,
                                        double angle_deg = 45,
                                        bool tune_pose = true);

    
    std::tuple<std::deque<Eigen::VectorXd>, std::deque<Eigen::VectorXd>, std::deque<Eigen::VectorXd>> 
        initTaskSpaceTrajectory(const gtsam::Pose3& start_pose,
                        const gtsam::Pose3& end_pose,
                    const double& duration_sec,
                const double& percentage = 0.25,
            const double& height = 0.23,
        double dt = 0.002);

    gtsam::Values initTaskSpaceTrajectory(const gtsam::Pose3& start_pose,
                                        const gtsam::Pose3& end_pose,
                                    const gtsam::Pose3& base_pose,
                                const gtsam::Vector& start_conf,
                            std::deque<gtsam::Pose3>& pose_trajectory,
                        double percentage,
                      double z_offset,
                      double y_offset,
                    int num_points);
};

#endif // TRAJECTORY_INITIATION_H