#include "RobotModel.h"

#include <iostream>

#include <pinocchio/parsers/urdf.hpp>

RobotModel::RobotModel(const std::string& urdf_path) {
    // Load the URDF model
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);

    // Gravity in the world frame: z-up, -9.81 m/s² (Pinocchio's default,
    // set explicitly so the convention is visible here).
    model_.gravity.linear() = Eigen::Vector3d(0.0, 0.0, -9.81);

    // Keep model diagnostics on stderr so planner/runtime diagnostics remain
    // separate from the controller's operator-facing stdout.
    std::cerr << "Model loaded successfully!" << std::endl;
    std::cerr << "Number of joints: " << model_.njoints << std::endl;
    std::cerr << "Number of DOFs: " << model_.nv << std::endl;
}
