#include "GenerateArmModel.h"

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace {

constexpr std::size_t kNumJoints = 7;

constexpr std::size_t kProximalLink = 0;
constexpr std::size_t kUpperArmLink = 2;
constexpr std::size_t kForearmLink = 4;
constexpr std::size_t kWristLink = 6;

struct SphereSpec {
    std::size_t link;
    double radius_scale;
    gtsam::Point3 offset;
    CollisionSphereGroup group;
};

const SphereSpec kArmSphereSpecs[] = {
    // Proximal arm
    {kProximalLink, 1.0, {0.0, 0.00, 0.0},
     CollisionSphereGroup::kMountInterface},

    {kProximalLink, 0.7, {0.0, 0.07, 0.0},
     CollisionSphereGroup::kProximalArm},

    {kProximalLink, 0.6, {0.0, 0.12, 0.0},
     CollisionSphereGroup::kProximalArm},

    {kProximalLink, 0.5, {0.0, 0.17, 0.0},
     CollisionSphereGroup::kProximalArm},

    // Upper arm
    {kUpperArmLink, 1.3, {0.0, 0.00, 0.0},
     CollisionSphereGroup::kUpperArm},

    {kUpperArmLink, 1.0, {0.0, 0.05, 0.0},
     CollisionSphereGroup::kUpperArm},

    {kUpperArmLink, 1.0, {0.0, 0.10, 0.0},
     CollisionSphereGroup::kUpperArm},

    {kUpperArmLink, 1.0, {0.0, 0.18, 0.0},
     CollisionSphereGroup::kUpperArm},

    {kUpperArmLink, 1.0, {0.0, 0.26, 0.0},
     CollisionSphereGroup::kUpperArm},

    {kUpperArmLink, 1.0, {0.0, 0.34, 0.0},
     CollisionSphereGroup::kUpperArm},

    {kUpperArmLink, 1.0, {0.0, 0.42, 0.0},
     CollisionSphereGroup::kUpperArm},

    // Forearm
    {kForearmLink, 1.3, {0.0, 0.00, 0.0},
     CollisionSphereGroup::kForearm},

    {kForearmLink, 1.0, {0.0, 0.08, 0.0},
     CollisionSphereGroup::kForearm},

    {kForearmLink, 1.0, {0.0, 0.16, 0.0},
     CollisionSphereGroup::kForearm},

    {kForearmLink, 1.0, {0.0, 0.24, 0.0},
     CollisionSphereGroup::kForearm},

    {kForearmLink, 1.0, {0.0, 0.31, 0.0},
     CollisionSphereGroup::kForearm},

    // Forearm girth
    {kForearmLink, 1.0, {0.00, 0.0,  0.08},
     CollisionSphereGroup::kForearm},

    {kForearmLink, 1.0, {0.00, 0.0, -0.08},
     CollisionSphereGroup::kForearm},

    {kForearmLink, 1.0, { 0.08, 0.0, 0.0},
     CollisionSphereGroup::kForearm},

    {kForearmLink, 1.0, {-0.08, 0.0, 0.0},
     CollisionSphereGroup::kForearm},

    // Wrist / fixed distal section
    {kWristLink, 1.0, {0.0, 0.0, -0.14},
     CollisionSphereGroup::kTool},

    {kWristLink, 1.0, {0.0, 0.0, -0.20},
     CollisionSphereGroup::kTool},

    {kWristLink, 1.0, {0.0, 0.0, -0.25},
     CollisionSphereGroup::kTool},
};

const SphereSpec kToolSphereSpecs[] = {
    {kWristLink, 0.6, { 0.05, 0.00,  0.00},
     CollisionSphereGroup::kTool},

    {kWristLink, 0.6, {-0.05, 0.00,  0.00},
     CollisionSphereGroup::kTool},

    {kWristLink, 0.6, { 0.05, 0.00, -0.04},
     CollisionSphereGroup::kTool},

    {kWristLink, 0.6, {-0.05, 0.00, -0.04},
     CollisionSphereGroup::kTool},

    {kWristLink, 0.6, { 0.07, 0.00, -0.08},
     CollisionSphereGroup::kTool},

    {kWristLink, 0.6, {-0.07, 0.00, -0.08},
     CollisionSphereGroup::kTool},

    {kWristLink, 0.6, { 0.07, 0.00, -0.12},
     CollisionSphereGroup::kTool},

    {kWristLink, 0.6, {-0.07, 0.00, -0.12},
     CollisionSphereGroup::kTool},

    {kWristLink, 0.6, { 0.05, 0.00, -0.14},
     CollisionSphereGroup::kTool},

    {kWristLink, 0.6, {-0.05, 0.00, -0.14},
     CollisionSphereGroup::kTool},

    {kWristLink, 0.8, {0.00, 0.05, -0.15},
     CollisionSphereGroup::kTool},
};

void validateSphereRadius(double radius) {
    if (!std::isfinite(radius) || radius <= 0.0) {
        throw std::invalid_argument(
            "generateArmSpheres: sphere_radius must be finite and positive");
    }
}

void appendSphereSpecs(
    const SphereSpec* specs,
    std::size_t count,
    std::size_t arm_id_offset,
    double base_radius,
    gpmp2::BodySphereVector& spheres,
    std::vector<CollisionSphereGroup>* groups) {

    for (std::size_t i = 0; i < count; ++i) {
        const auto& spec = specs[i];

        spheres.emplace_back(
            arm_id_offset + spec.link,
            base_radius * spec.radius_scale,
            spec.offset);

        if (groups) {
            groups->push_back(spec.group);
        }
    }
}

} // namespace


ArmModel::ArmModel() = default;


std::unique_ptr<gpmp2::ArmModel> ArmModel::createArmModel(
    const gtsam::Pose3& base_pose,
    const DHParameters& dh_params,
    bool has_tool,
    std::vector<CollisionSphereGroup>* sphere_groups,
    gpmp2::BodySphereVector* authored_spheres) {

    if (dh_params.a.size() != kNumJoints ||
        dh_params.alpha.size() != kNumJoints ||
        dh_params.d.size() != kNumJoints ||
        dh_params.theta.size() != kNumJoints) {

        throw std::invalid_argument(
            "createArmModel: expected 7-DOF DH parameters");
    }

    gpmp2::Arm arm(
        kNumJoints,
        dh_params.a,
        dh_params.alpha,
        dh_params.d,
        base_pose,
        dh_params.theta);

    auto arm_spheres =
        generateArmSpheres(0, 0.05, dh_params.d, has_tool);

    if (authored_spheres) {
        *authored_spheres = arm_spheres;
    }

    if (sphere_groups) {
        sphere_groups->clear();
        sphere_groups->reserve(
            std::size(kArmSphereSpecs) +
            (has_tool ? std::size(kToolSphereSpecs) : 0));

        // The groups are authored with the same sphere definitions.
        for (const auto& spec : kArmSphereSpecs) {
            sphere_groups->push_back(spec.group);
        }

        if (has_tool) {
            for (const auto& spec : kToolSphereSpecs) {
                sphere_groups->push_back(spec.group);
            }
        }
    }

    return std::make_unique<gpmp2::ArmModel>(
        arm,
        arm_spheres);
}


gpmp2::BodySphereVector ArmModel::generateArmSpheres(
    std::size_t arm_id_offset,
    double sphere_radius,
    const gtsam::Vector& d,
    bool has_tool) {

    validateSphereRadius(sphere_radius);

    if (d.size() != kNumJoints) {
        throw std::invalid_argument(
            "generateArmSpheres: expected 7 DH d parameters");
    }

    gpmp2::BodySphereVector spheres;

    const std::size_t sphere_count =
        std::size(kArmSphereSpecs) +
        (has_tool ? std::size(kToolSphereSpecs) : 0);

    spheres.reserve(sphere_count);

    appendSphereSpecs(
        kArmSphereSpecs,
        std::size(kArmSphereSpecs),
        arm_id_offset,
        sphere_radius,
        spheres,
        nullptr);

    if (has_tool) {
        appendSphereSpecs(
            kToolSphereSpecs,
            std::size(kToolSphereSpecs),
            arm_id_offset,
            sphere_radius,
            spheres,
            nullptr);
    }

    return spheres;
}