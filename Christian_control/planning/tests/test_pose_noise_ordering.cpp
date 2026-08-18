// Holds PoseNoiseModel to gpmp2's actual error ordering.
//
// gpmp2::GaussianPriorWorkspacePose's error is `des_pose.logmap(actual)`,
// and gtsam's Pose3 logmap returns [rotation; translation] — the reverse of
// the [translation; rotation] most people assume. Swapping them is SILENT:
// the noise model is still six plausible numbers, the solve still
// converges, and the wrong three axes are weighted. A comment cannot
// prevent that; this test can.
//
// Method: build a noise model that is stiff in rotation and slack in
// translation, then whiten a pure-rotation error and a pure-translation
// error of the same magnitude. If the ordering is right the rotation error
// is amplified and the translation error attenuated. If the two blocks were
// swapped, the results swap too — and the assertions fail.
//
// The pure-translation and pure-rotation error vectors are constructed
// through gtsam's own logmap rather than by hand, so the test is checking
// against gtsam's convention rather than against a second copy of my
// assumption about it.
#undef NDEBUG

#include <cassert>
#include <cmath>
#include <cstdio>

#include "TrajectoryOptimization.h"

int main() {
    // Stiff in rotation (small sigma), slack in translation (large sigma).
    // Two orders of magnitude apart so the comparison cannot be noise.
    const Eigen::Vector3d rotation_sigma(0.001, 0.001, 0.001);
    const Eigen::Vector3d translation_sigma(0.1, 0.1, 0.1);
    const auto model = PoseNoiseModel(rotation_sigma, translation_sigma);

    // A pure ROTATION discrepancy, expressed the way gpmp2 will express it.
    const double angle_rad = 0.01;
    const gtsam::Pose3 desired = gtsam::Pose3::Identity();
    const gtsam::Pose3 rotated(gtsam::Rot3::Rz(angle_rad), gtsam::Point3(0, 0, 0));
    const gtsam::Vector6 rotation_error = desired.logmap(rotated);

    // A pure TRANSLATION discrepancy of the same numeric magnitude, so the
    // only thing distinguishing them is which block they land in.
    const gtsam::Pose3 translated(gtsam::Rot3::Identity(),
                                  gtsam::Point3(0, 0, angle_rad));
    const gtsam::Vector6 translation_error = desired.logmap(translated);

    // Sanity: the two errors really are in different halves of the vector.
    assert(rotation_error.head<3>().norm() > 1e-6 &&
           "a pure rotation must appear in the FIRST three components");
    assert(rotation_error.tail<3>().norm() < 1e-9 &&
           "a pure rotation must not appear in the last three components");
    assert(translation_error.tail<3>().norm() > 1e-6 &&
           "a pure translation must appear in the LAST three components");
    assert(translation_error.head<3>().norm() < 1e-9 &&
           "a pure translation must not appear in the first three components");

    // Whitening divides by sigma, so the stiff block is amplified.
    const double whitened_rotation = model->whiten(rotation_error).norm();
    const double whitened_translation = model->whiten(translation_error).norm();

    std::printf("same-magnitude error, whitened: rotation %.3f, translation %.3f\n",
                whitened_rotation, whitened_translation);

    // rotation: 0.01 / 0.001 = 10; translation: 0.01 / 0.1 = 0.1.
    assert(std::abs(whitened_rotation - 10.0) < 1e-6 &&
           "the ROTATION sigma must weight the rotation error");
    assert(std::abs(whitened_translation - 0.1) < 1e-6 &&
           "the TRANSLATION sigma must weight the translation error");
    assert(whitened_rotation > whitened_translation &&
           "stiff rotation must dominate slack translation; if this fails the "
           "two sigma blocks are swapped");

    // Independent axes: a sigma on one axis must not affect another.
    const auto anisotropic = PoseNoiseModel(Eigen::Vector3d(0.001, 1.0, 1.0),
                                            Eigen::Vector3d(1.0, 1.0, 0.001));
    const gtsam::Pose3 roll_only(gtsam::Rot3::Rx(angle_rad), gtsam::Point3(0, 0, 0));
    const gtsam::Pose3 z_only(gtsam::Rot3::Identity(), gtsam::Point3(0, 0, angle_rad));
    const double whitened_roll = anisotropic->whiten(desired.logmap(roll_only)).norm();
    const double whitened_z = anisotropic->whiten(desired.logmap(z_only)).norm();
    assert(std::abs(whitened_roll - 10.0) < 1e-6 &&
           "rotation_sigma[0] must weight roll (rotation about x)");
    assert(std::abs(whitened_z - 10.0) < 1e-6 &&
           "translation_sigma[2] must weight z translation");

    std::printf("all pose-noise ordering tests passed\n");
    return 0;
}
