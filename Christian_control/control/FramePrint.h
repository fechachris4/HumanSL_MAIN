//
// FramePrint — the dual-arm FK verification table: each arm's tool frame in
// the mount frame and in its own base_link, side by side, for one
// configuration. Shared by the controller's startup print (Main.cpp) and the
// hardware-free tools/print_dual_arm_fk.cpp so both report identically.
//
// Printing only: no control math, no hardware.
//

#pragma once

#include <iomanip>
#include <iostream>
#include <string>

#include <Eigen/Dense>

#include "Config.h"
#include "Kinematics.h"

// One arm's row pair. `mount` and `base` describe the same tool point in two
// frames, so a disagreement between them is a mounting-transform error and
// nothing else.
inline void PrintArmFrameRows(const char* arm_label, const char* tool_frame,
                              const char* base_frame, const Pose& mount,
                              const Pose& base)
{
    const auto xyz = [](const Eigen::Vector3d& p) {
        std::ostringstream row;
        row << std::fixed << std::setprecision(6) << std::setw(11) << p.x() << " "
            << std::setw(11) << p.y() << " " << std::setw(11) << p.z();
        return row.str();
    };
    // R = Rz*Ry*Rx, printed as roll pitch yaw to match RotationFromRpy and the
    // controller's existing orientation line.
    const auto rpy = [](const Eigen::Matrix3d& R) {
        const Eigen::Vector3d zyx = R.eulerAngles(2, 1, 0);
        std::ostringstream row;
        row << std::fixed << std::setprecision(6) << std::setw(11) << zyx.z() << " "
            << std::setw(11) << zyx.y() << " " << std::setw(11) << zyx.x();
        return row.str();
    };

    std::cout << "  " << arm_label << "  tool frame " << tool_frame << "\n"
              << "    mount      p " << xyz(mount.position) << "   rpy "
              << rpy(mount.rotation) << "\n"
              << "    " << std::left << std::setw(10) << base_frame << std::right
              << " p " << xyz(base.position) << "   rpy " << rpy(base.rotation)
              << "\n";
}

// The full table for one configuration of both arms.
//
// The two arms' tool frames are NOT the same point on the arm: the right ends
// at right_tool_link (the mounted tool) and the left at its bare flange,
// because the tool is physically on the right arm only. Each row names its own
// frame for that reason — do not read the two position rows as a symmetry
// check.
inline void PrintDualArmFkTable(DualArmKinematics& model,
                                const Eigen::Matrix<double, 7, 1>& right_q_rad,
                                const Eigen::Matrix<double, 7, 1>& left_q_rad,
                                const std::string& caption)
{
    std::cout << caption << "\n";

    const auto arm_rows = [&](Arm arm, const char* label, const char* tool_frame,
                              const char* base_frame) {
        const Pose mount = model.ToolPoseInMount(arm, right_q_rad, left_q_rad);
        // p_base = T_mount_base^-1 * p_mount — the legacy conversion, run
        // backwards. Nothing here recomputes FK.
        const pinocchio::SE3& mount_from_base = model.MountFromBase(arm);
        const Pose base{mount_from_base.actInv(mount.position),
                        mount_from_base.rotation().transpose() * mount.rotation};
        PrintArmFrameRows(label, tool_frame, base_frame, mount, base);
    };

    arm_rows(Arm::kRight, "right", config::kRightEndEffectorFrame,
             config::kRightBaseFrame);
    arm_rows(Arm::kLeft, "left ", config::kLeftEndEffectorFrame,
             config::kLeftBaseFrame);
}
