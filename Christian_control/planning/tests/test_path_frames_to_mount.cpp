// ToMount is the planner's ONE input conversion (PathFrames.h). This test
// proves the architectural claims it rests on: each declared frame lands in
// mount through the URDF-derived chain, a world-declared input REQUIRES a
// valid world_T_mount snapshot, and the point/rotation helpers agree with
// the pose core so a pose can never be half-converted.

#include <cassert>
#include <cstdio>
#include <cmath>

#include "PathFrames.h"
#include "PinocchioKinematicsAdapter.h"

namespace {

Eigen::Isometry3d SamplePose() {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() =
        Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized())
            .toRotationMatrix();
    pose.translation() = Eigen::Vector3d(0.4, -0.2, 0.9);
    return pose;
}

bool Near(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
    return (a.matrix() - b.matrix()).cwiseAbs().maxCoeff() < 1e-12;
}

}  // namespace

int main() {
    const Eigen::Isometry3d pose = SamplePose();

    // mount -> mount is the identity.
    assert(Near(ToMount(pose, config::ReferenceFrame::kMount, std::nullopt),
                pose));

    // Base frames compose through the URDF's mount_T_base — both halves of
    // the pose, in one multiply.
    for (const bool left : {false, true}) {
        const auto frame = left ? config::ReferenceFrame::kLeftBase
                                : config::ReferenceFrame::kRightBase;
        const Eigen::Isometry3d expected =
            pinocchio_kinematics_adapter::MountFromBase(left) * pose;
        assert(Near(ToMount(pose, frame, std::nullopt), expected));
    }

    // World: with a valid snapshot, ToMount inverts it exactly — a pose
    // written in world as T_W_M * X must come back as X.
    Eigen::Isometry3d world_T_mount = Eigen::Isometry3d::Identity();
    world_T_mount.linear() =
        Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    world_T_mount.translation() = Eigen::Vector3d(0.3, -0.2, 1.1);
    assert(Near(ToMount(world_T_mount * pose, config::ReferenceFrame::kWorld,
                        world_T_mount),
                pose));

    // World without a snapshot, or with a non-finite one, is REJECTED —
    // per-request, as ToMountError, never a silent identity.
    bool threw = false;
    try {
        (void)ToMount(pose, config::ReferenceFrame::kWorld, std::nullopt);
    } catch (const ToMountError&) {
        threw = true;
    }
    assert(threw);
    threw = false;
    Eigen::Isometry3d bad = world_T_mount;
    bad.translation().x() = std::nan("");
    try {
        (void)ToMount(pose, config::ReferenceFrame::kWorld, bad);
    } catch (const ToMountError&) {
        threw = true;
    }
    assert(threw);

    // The point and rotation helpers are the pose core, restricted — never
    // an independent chain that could drift from it.
    for (const auto frame :
         {config::ReferenceFrame::kMount, config::ReferenceFrame::kRightBase,
          config::ReferenceFrame::kLeftBase, config::ReferenceFrame::kWorld}) {
        const Eigen::Isometry3d whole = ToMount(pose, frame, world_T_mount);
        const Eigen::Vector3d point =
            PointToMount(pose.translation(), frame, world_T_mount);
        const Eigen::Matrix3d rotation =
            RotationToMount(pose.linear(), frame, world_T_mount);
        assert((point - whole.translation()).cwiseAbs().maxCoeff() < 1e-12);
        assert((rotation - whole.linear()).cwiseAbs().maxCoeff() < 1e-12);
    }

    // PathToMount stamps the converted path kMount so it cannot be
    // mistaken for an unconverted one downstream.
    CartesianPath path;
    path.frame = config::ReferenceFrame::kWorld;
    PathSample sample;
    sample.t_s = 0.5;
    sample.pose = world_T_mount * pose;
    path.samples.push_back(sample);
    const CartesianPath converted = PathToMount(path, world_T_mount);
    assert(converted.frame == config::ReferenceFrame::kMount);
    assert(converted.samples.size() == 1);
    assert(converted.samples[0].t_s == 0.5);
    assert(Near(converted.samples[0].pose, pose));

    std::puts("test_path_frames_to_mount: all assertions passed");
    return 0;
}
