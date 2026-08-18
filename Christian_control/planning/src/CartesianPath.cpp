#include "CartesianPath.h"

#include <cmath>
#include <stdexcept>

namespace {

// Rotation from roll/pitch/yaw, R = Rz*Ry*Rx. Duplicated from BridgeMain's
// helper of the same name rather than shared, because that one lives in an
// anonymous namespace in a translation unit that pulls in yaml-cpp and the
// whole bridge; this file must stay includable from the Pinocchio-side
// probe. The convention is pinned by test_cartesian_path.
Eigen::Matrix3d RotationFromRpy(const Eigen::Vector3d& rpy_rad) {
    return (Eigen::AngleAxisd(rpy_rad.z(), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy_rad.y(), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy_rad.x(), Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

}  // namespace

int CircleSamplesForChordError(double radius_m, double max_chord_error_m) {
    if (!(radius_m > 0.0) || !(max_chord_error_m > 0.0))
        throw std::invalid_argument(
            "chord-error sampling needs a positive radius and tolerance");
    // e >= r means even a triangle is inside tolerance; the formula's acos
    // argument would leave [-1, 1], so clamp to the floor instead.
    const double ratio = 1.0 - max_chord_error_m / radius_m;
    if (ratio <= -1.0) return 8;
    const double n = M_PI / std::acos(std::min(1.0, ratio));
    const int samples = static_cast<int>(std::ceil(n));
    return std::max(8, std::min(512, samples));
}

void PlaneBasis(const Eigen::Vector3d& normal, Eigen::Vector3d& u, Eigen::Vector3d& v) {
    const Eigen::Vector3d n = normal.normalized();
    // Cross with whichever axis is least aligned with n, so the cross
    // product is never near-degenerate and the basis is numerically sound
    // for every normal direction — picking a fixed axis fails when the
    // normal happens to be parallel to it.
    Eigen::Vector3d seed = Eigen::Vector3d::UnitX();
    if (std::abs(n.x()) > std::abs(n.y()))
        seed = std::abs(n.y()) <= std::abs(n.z()) ? Eigen::Vector3d::UnitY()
                                                  : Eigen::Vector3d::UnitZ();
    else
        seed = std::abs(n.x()) <= std::abs(n.z()) ? Eigen::Vector3d::UnitX()
                                                  : Eigen::Vector3d::UnitZ();
    u = n.cross(seed).normalized();
    v = n.cross(u);  // already unit: n and u are orthonormal
}

CartesianPath GenerateCircle(const CircleSpec& spec) {
    if (!spec.centre_m.allFinite() || !spec.normal.allFinite() ||
        !std::isfinite(spec.radius_m) || !std::isfinite(spec.duration_s) ||
        !std::isfinite(spec.start_angle_rad) || !spec.fixed_rpy_rad.allFinite())
        throw std::invalid_argument("circle spec contains a non-finite value");
    if (spec.radius_m <= 0.0)
        throw std::invalid_argument("circle radius must be positive, got " +
                                    std::to_string(spec.radius_m));
    if (spec.duration_s <= 0.0)
        throw std::invalid_argument("circle duration must be positive, got " +
                                    std::to_string(spec.duration_s));
    if (spec.samples < 3)
        throw std::invalid_argument(
            "a circle needs at least 3 samples, got " + std::to_string(spec.samples) +
            " — fewer cannot describe a closed curve");
    // A normal this short cannot be normalised meaningfully; the plane it
    // claims to define is noise.
    if (spec.normal.norm() < 1e-9)
        throw std::invalid_argument("circle normal is degenerate (near-zero length)");

    Eigen::Vector3d u, v;
    PlaneBasis(spec.normal, u, v);
    const Eigen::Vector3d n = spec.normal.normalized();
    const Eigen::Matrix3d fixed_rotation = RotationFromRpy(spec.fixed_rpy_rad);

    CartesianPath path;
    path.frame = spec.frame;
    // samples + 1: the extra sample closes the loop back onto the start, so
    // `samples` points bound `samples` arcs rather than samples-1.
    path.samples.reserve(static_cast<std::size_t>(spec.samples) + 1);
    for (int i = 0; i <= spec.samples; ++i) {
        const double fraction = static_cast<double>(i) / static_cast<double>(spec.samples);
        const double angle = spec.start_angle_rad + fraction * 2.0 * M_PI;
        const Eigen::Vector3d radial = std::cos(angle) * u + std::sin(angle) * v;

        PathSample sample;
        sample.t_s = fraction * spec.duration_s;
        sample.pose.translation() = spec.centre_m + spec.radius_m * radial;

        if (spec.orientation == OrientationPolicy::kFixed) {
            sample.pose.linear() = fixed_rotation;
        } else {
            // Approach axis (+z) points from the rim at the centre, i.e.
            // along -radial. x is taken tangent to the circle so the frame
            // turns smoothly with the motion instead of flipping; y closes
            // the right-handed set. radial ⟂ n by construction, so this is
            // never degenerate.
            const Eigen::Vector3d z_axis = -radial;
            const Eigen::Vector3d x_axis = n.cross(z_axis);  // unit: both are
            const Eigen::Vector3d y_axis = z_axis.cross(x_axis);
            Eigen::Matrix3d rotation;
            rotation.col(0) = x_axis;
            rotation.col(1) = y_axis;
            rotation.col(2) = z_axis;
            sample.pose.linear() = rotation;
        }
        path.samples.push_back(sample);
    }
    return path;
}
