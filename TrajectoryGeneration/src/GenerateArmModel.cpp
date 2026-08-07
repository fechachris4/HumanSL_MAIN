#include "GenerateArmModel.h"

#include <cmath>
#include <stdexcept>


ArmModel::ArmModel(){};


std::unique_ptr<gpmp2::ArmModel> ArmModel::createArmModel(const gtsam::Pose3& base_pose, const DHParameters& dh_params, bool has_tool) {

    // Create 7-DOF arms
    auto arm = std::make_unique<gpmp2::Arm>(7, dh_params.a, dh_params.alpha, dh_params.d,
                                                base_pose, dh_params.theta);

    auto arm_spheres = generateArmSpheres(0, 0.05, dh_params.d, has_tool);

    // Create ArmModel instances
    auto arm_model = std::make_unique<gpmp2::ArmModel>(*arm, arm_spheres);


    return arm_model;
}


gpmp2::BodySphereVector ArmModel::generateArmSpheres(
    size_t arm_id_offset,
    double sphere_radius,
    const gtsam::Vector& d,
    bool has_tool) {

    // The station offsets below were authored against these d values (the
    // 2026-08-05 hand YAML). Each along-link station is scaled by the signed
    // ratio s = d_current / d_authored, so today's generated (URDF-exact)
    // values reproduce the authored geometry to <0.1 mm and a future URDF
    // length change (e.g. a new tool offset in d7) moves the stations
    // proportionally. Lateral offsets model arm girth and stay absolute.
    //
    // gpmp2 sphere link index L is the DH frame at the FAR end of
    // translation d(L): index 0 <-> d(0)=d1, 2 <-> d(2)=d3, 4 <-> d(4)=d5,
    // 6 <-> d(6)=d7. Stations run back along the link toward the parent
    // frame: +y on links 0/2/4, -z on link 6 (alpha7 = pi flips z).
    //
    // No stdout here: in the production bridge this runs before the stdout
    // FIFO redirect guard, so diagnostics are exceptions only (BridgeMain
    // converts them to exit 3).
    const double authored_d[7] = {-0.2848, -0.0118, -0.4208, -0.0128,
                                  -0.3143, 0.0, -0.2874};
    double scale[7] = {0, 0, 0, 0, 0, 0, 0};
    for (int link : {0, 2, 4, 6}) {
        if (std::abs(d(link)) < 0.05)
            throw std::runtime_error(
                "generateArmSpheres: link d" + std::to_string(link + 1) +
                " collapsed to " + std::to_string(d(link)) +
                " m — sphere layout must be re-derived by hand");
        const double s = d(link) / authored_d[link];
        if (s <= 0.0)
            throw std::runtime_error(
                "generateArmSpheres: link d" + std::to_string(link + 1) +
                " changed sign (" + std::to_string(d(link)) +
                " vs authored " + std::to_string(authored_d[link]) +
                ") — frame geometry changed, sphere layout must be re-derived");
        scale[link] = s;
    }

    gpmp2::BodySphereVector spheres;

    // Joint 0: Base to Joint 1 (|d1| ~ 28.5cm link)
    spheres.emplace_back(arm_id_offset + 0, sphere_radius, gtsam::Point3(0.0, 0.0, 0.0));
    spheres.emplace_back(arm_id_offset + 0, sphere_radius * 0.7, gtsam::Point3(0.0, 0.07 * scale[0], 0.0));
    spheres.emplace_back(arm_id_offset + 0, sphere_radius * 0.6, gtsam::Point3(0.0, 0.12 * scale[0], 0.0));
    spheres.emplace_back(arm_id_offset + 0, sphere_radius * 0.5, gtsam::Point3(0.0, 0.17 * scale[0], 0.0));


    // Joint 1: Short connector (|d2| ~ 1.2cm) - SKIP

    // Joint 2: Upper arm long link (|d3| ~ 42.1cm link)
    spheres.emplace_back(arm_id_offset + 2, sphere_radius * 1.3, gtsam::Point3(0.0, 0.0, 0.0));
    spheres.emplace_back(arm_id_offset + 2, sphere_radius, gtsam::Point3(0.0, 0.05 * scale[2], 0.0));
    spheres.emplace_back(arm_id_offset + 2, sphere_radius, gtsam::Point3(0.0, 0.10 * scale[2], 0.0));
    spheres.emplace_back(arm_id_offset + 2, sphere_radius, gtsam::Point3(0.0, 0.18 * scale[2], 0.0));
    spheres.emplace_back(arm_id_offset + 2, sphere_radius, gtsam::Point3(0.0, 0.26 * scale[2], 0.0));
    spheres.emplace_back(arm_id_offset + 2, sphere_radius, gtsam::Point3(0.0, 0.34 * scale[2], 0.0));
    spheres.emplace_back(arm_id_offset + 2, sphere_radius, gtsam::Point3(0.0, 0.42 * scale[2], 0.0));

    // Joint 3: Short connector (|d4| ~ 1.3cm) - SKIP

    // Joint 4: Forearm long link (|d5| ~ 31.4cm link)
    spheres.emplace_back(arm_id_offset + 4, sphere_radius * 1.3, gtsam::Point3(0.0, 0.0, 0.0));
    spheres.emplace_back(arm_id_offset + 4, sphere_radius, gtsam::Point3(0.0, 0.08 * scale[4], 0.0));
    spheres.emplace_back(arm_id_offset + 4, sphere_radius, gtsam::Point3(0.0, 0.16 * scale[4], 0.0));
    spheres.emplace_back(arm_id_offset + 4, sphere_radius, gtsam::Point3(0.0, 0.24 * scale[4], 0.0));
    spheres.emplace_back(arm_id_offset + 4, sphere_radius, gtsam::Point3(0.0, 0.31 * scale[4], 0.0));

    // Lateral cross on the forearm — girth, absolute.
    spheres.emplace_back(arm_id_offset + 4, sphere_radius * 1, gtsam::Point3(0.0, 0.0, 0.08));
    spheres.emplace_back(arm_id_offset + 4, sphere_radius * 1, gtsam::Point3(0.0, 0.0, -0.08));
    spheres.emplace_back(arm_id_offset + 4, sphere_radius * 1, gtsam::Point3(0.08, 0.0, 0.0));
    spheres.emplace_back(arm_id_offset + 4, sphere_radius * 1, gtsam::Point3(-0.08, 0.0, 0.0));

    // Joint 5: No link (d6 ~ 0) - SKIP

    // Joint 6: Wrist to configured tool (|d7| ~ 28.7cm with the mounted
    // tool). Stations are -z fractions of the link; the tool offset lives
    // in d7, so a tool change rescales these automatically.
    spheres.emplace_back(arm_id_offset + 6, sphere_radius, gtsam::Point3(0.0, 0.0, -0.14 * scale[6]));
    spheres.emplace_back(arm_id_offset + 6, sphere_radius, gtsam::Point3(0.0, 0.0, -0.20 * scale[6]));
    spheres.emplace_back(arm_id_offset + 6, sphere_radius, gtsam::Point3(0.0, 0.0, -0.25 * scale[6]));


    // Gripper/End-effector spheres — finger spread (x) and the y bump are
    // girth, absolute; the -z stations ride the link scale. Right arm only:
    // these model the mounted tool's physical footprint, which the left
    // arm's bare flange does not have (see the .h comment on has_tool).
    if (has_tool) {
        spheres.emplace_back(arm_id_offset + 6, sphere_radius*0.6, gtsam::Point3(0.05, 0.0, 0.0));
        spheres.emplace_back(arm_id_offset + 6, sphere_radius*0.6, gtsam::Point3(-0.05, 0.0, 0.0));
        spheres.emplace_back(arm_id_offset + 6, sphere_radius*0.6, gtsam::Point3(0.05, 0.0, -0.04 * scale[6]));
        spheres.emplace_back(arm_id_offset + 6, sphere_radius*0.6, gtsam::Point3(-0.05, 0.0, -0.04 * scale[6]));
        spheres.emplace_back(arm_id_offset + 6, sphere_radius*0.6, gtsam::Point3(0.07, 0.0, -0.08 * scale[6]));
        spheres.emplace_back(arm_id_offset + 6, sphere_radius*0.6, gtsam::Point3(-0.07, 0.0, -0.08 * scale[6]));
        spheres.emplace_back(arm_id_offset + 6, sphere_radius*0.6, gtsam::Point3(0.07, 0.0, -0.12 * scale[6]));
        spheres.emplace_back(arm_id_offset + 6, sphere_radius*0.6, gtsam::Point3(-0.07, 0.0, -0.12 * scale[6]));
        spheres.emplace_back(arm_id_offset + 6, sphere_radius*0.6, gtsam::Point3(0.05, 0.0, -0.14 * scale[6]));
        spheres.emplace_back(arm_id_offset + 6, sphere_radius*0.6, gtsam::Point3(-0.05, 0.0, -0.14 * scale[6]));
        spheres.emplace_back(arm_id_offset + 6, sphere_radius*0.8, gtsam::Point3(0.0, 0.05, -0.15 * scale[6]));
    }

    return spheres;
}
