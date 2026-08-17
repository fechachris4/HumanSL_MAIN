//
// probe_direction — OFFLINE (no robot): at a given measured joint pose,
// evaluate the controller's own position-DLS solution for candidate
// Cartesian directions and report each joint's initial velocity sign.
//
// Purpose (2026-08-03): joints 4 and 6 carry degenerate configured
// JOINT_LIMIT bands (error thresholds 0/0), so the firmware faults them on
// any OUTWARD motion — j4 must not move down (raw decreasing), j6 must not
// move up (raw increasing). This probe finds a test direction whose
// resolved joint motion is INWARD on both, using the same URDF, adapter,
// and DLS the controller runs.
//
//   ./probe_direction q1 q2 q3 q4 q5 q6 q7   (deg, raw Kortex order)
//

#include "Config.h"
#include "Kinematics.h"
#include "ReactiveLaw.h"

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>

int main(int argc, char** argv)
{
    if (argc != 8)
    {
        std::cerr << "usage: probe_direction q1..q7 (deg raw)\n";
        return 2;
    }
    Eigen::Matrix<double, 7, 1> q_rad;
    for (int i = 0; i < 7; ++i)
        q_rad[i] = std::atof(argv[i + 1]) * M_PI / 180.0;

    RobotModel robot_model(GEN3_DUAL_URDF_PATH);
    DualArmKinematics model(robot_model, Arm::kRight, config::kLeftNominalRad,
                            config::kRightBaseFrame,
                            config::kRightEndEffectorFrame);
    KinematicsWorkspace workspace(robot_model);

    const PositionJacobian pj = model.ControlledPositionAndJacobian(q_rad, workspace);
    const PoseJacobian pose = model.ControlledPoseAndJacobian(q_rad, workspace);
    std::cout << "EE position: " << pj.position.transpose() << " m in "
              << config::kRightBaseFrame << "\n\n";

    const struct { const char* name; Eigen::Vector3d v; } candidates[] = {
        {"+x", {1, 0, 0}}, {"-x", {-1, 0, 0}},
        {"+y", {0, 1, 0}}, {"-y", {0, -1, 0}},
        {"+z", {0, 0, 1}}, {"-z", {0, 0, -1}},
        {"+x+z", {M_SQRT1_2, 0, M_SQRT1_2}},
        {"-x+z", {-M_SQRT1_2, 0, M_SQRT1_2}},
        {"+y+z", {0, M_SQRT1_2, M_SQRT1_2}},
        {"-y+z", {0, -M_SQRT1_2, M_SQRT1_2}},
        {"+x-z", {M_SQRT1_2, 0, -M_SQRT1_2}},
        {"-x-z", {-M_SQRT1_2, 0, -M_SQRT1_2}},
        {"+y-z", {0, M_SQRT1_2, -M_SQRT1_2}},
        {"-y-z", {0, -M_SQRT1_2, -M_SQRT1_2}},
    };

    // The controller (TrackingController's pose law) solves a 6-DOF task:
    // linear velocity
    // plus an orientation-HOLD (angular objective ~0 at the start of a
    // move). The 6-row DLS distributes joint motion differently from the
    // position-only solve — the 2026-08-03 14:05 run proved the difference
    // matters (position-only predicted j6 down for +z; the real controller
    // drove j6 up). So evaluate BOTH; trust the 6-DOF row.
    std::cout << "qdot (deg/s) for 0.1 m/s along each direction; need "
        "qdot4 > 0 AND qdot6 < 0 (inward for both):\n";
    std::cout << std::fixed << std::setprecision(2);
    for (const auto& c : candidates)
    {
        const Eigen::Matrix<double, 7, 1> qdot3 =
            DampedLeastSquares(pj.jacobian_p, 0.1 * c.v, config::kDlsLambda) *
            180.0 / M_PI;

        Eigen::Matrix<double, 6, 1> twist;
        twist << 0.1 * c.v, 0.0, 0.0, 0.0; // hold orientation
        Eigen::Matrix<double, 6, 6> jjt =
            pose.jacobian * pose.jacobian.transpose();
        jjt.diagonal().array() += config::kDlsLambda * config::kDlsLambda;
        const Eigen::Matrix<double, 7, 1> qdot6 =
            pose.jacobian.transpose() * jjt.ldlt().solve(twist) * 180.0 / M_PI;

        const bool ok3 = qdot3[3] > 0.05 && qdot3[5] < -0.05;
        const bool ok6 = qdot6[3] > 0.05 && qdot6[5] < -0.05;
        std::cout << (ok3 ? " ok3 " : "     ") << std::setw(5) << c.name
            << " pos-only:";
        for (int i = 0; i < 7; ++i)
            std::cout << std::setw(8) << qdot3[i];
        std::cout << "\n" << (ok6 ? " OK6 " : "     ") << std::setw(5) << c.name
            << " pose6dof:";
        for (int i = 0; i < 7; ++i)
            std::cout << std::setw(8) << qdot6[i];
        std::cout << "\n";
    }
    return 0;
}
