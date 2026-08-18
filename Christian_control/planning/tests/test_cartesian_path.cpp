// Hardware-free tests for the Cartesian path types and the circle
// generator. No robot, no Kortex, no gtsam — pure geometry.
//
// The checks deliberately RECOVER the circle's properties from the emitted
// samples (every point equidistant from the centre, every point in the
// declared plane, arc lengths equal) rather than re-deriving them with the
// generator's own arithmetic, which would only prove the code agrees with
// itself.
#undef NDEBUG

#include <cassert>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "CartesianPath.h"

namespace {

int failures = 0;

void Check(bool ok, const char* what) {
    if (!ok) {
        std::printf("FAIL: %s\n", what);
        ++failures;
    }
}

bool Throws(const CircleSpec& spec) {
    try {
        GenerateCircle(spec);
        return false;
    } catch (const std::invalid_argument&) {
        return true;
    }
}

CircleSpec BaseSpec() {
    CircleSpec spec;
    spec.centre_m = Eigen::Vector3d(0.4, -0.1, 0.3);
    spec.radius_m = 0.08;
    spec.normal = Eigen::Vector3d(0.0, 0.0, 1.0);
    spec.samples = 36;
    spec.duration_s = 12.0;
    spec.frame = config::ReferenceFrame::kMount;
    return spec;
}

}  // namespace

int main() {
    // --- the plane basis is orthonormal and spans the plane, for an
    // awkward normal as well as an axis-aligned one. A fixed seed axis
    // would go degenerate when the normal happened to be parallel to it.
    for (const Eigen::Vector3d& normal :
         {Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 1, 0),
          Eigen::Vector3d(0.3, -0.5, 0.81), Eigen::Vector3d(-1, 0, 0)}) {
        Eigen::Vector3d u, v;
        PlaneBasis(normal, u, v);
        const Eigen::Vector3d n = normal.normalized();
        Check(std::abs(u.norm() - 1.0) < 1e-12, "plane basis u is unit length");
        Check(std::abs(v.norm() - 1.0) < 1e-12, "plane basis v is unit length");
        Check(std::abs(u.dot(v)) < 1e-12, "plane basis is orthogonal");
        Check(std::abs(u.dot(n)) < 1e-12, "u lies in the plane");
        Check(std::abs(v.dot(n)) < 1e-12, "v lies in the plane");
    }

    // --- geometry recovered from the samples themselves
    {
        const CircleSpec spec = BaseSpec();
        const CartesianPath path = GenerateCircle(spec);
        const Eigen::Vector3d n = spec.normal.normalized();

        Check(path.frame == spec.frame, "path carries the declared frame");
        Check(path.samples.size() == static_cast<std::size_t>(spec.samples) + 1,
              "a closed circle emits samples+1 points");
        Check(std::abs(path.duration_s() - spec.duration_s) < 1e-12,
              "path duration matches the requested lap time");

        double worst_radius = 0.0, worst_plane = 0.0;
        for (const PathSample& sample : path.samples) {
            const Eigen::Vector3d offset = sample.pose.translation() - spec.centre_m;
            worst_radius = std::max(worst_radius, std::abs(offset.norm() - spec.radius_m));
            worst_plane = std::max(worst_plane, std::abs(offset.dot(n)));
        }
        Check(worst_radius < 1e-12, "every sample is exactly the radius from the centre");
        Check(worst_plane < 1e-12, "every sample lies in the declared plane");

        // Closed: last position returns to the first.
        Check((path.samples.front().pose.translation() -
               path.samples.back().pose.translation()).norm() < 1e-12,
              "the last sample closes the loop onto the first");

        // Constant angular rate => equal chords and equal time steps, which
        // is what makes the trace constant-speed.
        double min_chord = 1e30, max_chord = 0.0, min_dt = 1e30, max_dt = 0.0;
        for (std::size_t i = 1; i < path.samples.size(); ++i) {
            const double chord = (path.samples[i].pose.translation() -
                                  path.samples[i - 1].pose.translation()).norm();
            const double dt = path.samples[i].t_s - path.samples[i - 1].t_s;
            min_chord = std::min(min_chord, chord);
            max_chord = std::max(max_chord, chord);
            min_dt = std::min(min_dt, dt);
            max_dt = std::max(max_dt, dt);
        }
        Check(max_chord - min_chord < 1e-12, "samples are equally spaced around the rim");
        Check(max_dt - min_dt < 1e-12, "time steps are uniform");
        Check(min_dt > 0.0, "time is strictly increasing");
    }

    // --- start_angle_rad moves where the trace begins, without changing
    //     the circle it traces.
    {
        CircleSpec spec = BaseSpec();
        spec.start_angle_rad = M_PI / 2.0;
        const CartesianPath shifted = GenerateCircle(spec);
        const Eigen::Vector3d offset =
            shifted.samples.front().pose.translation() - spec.centre_m;
        Check(std::abs(offset.norm() - spec.radius_m) < 1e-12,
              "a shifted start still begins on the rim");

        spec.start_angle_rad = 0.0;
        const CartesianPath plain = GenerateCircle(spec);
        Check((plain.samples.front().pose.translation() -
               shifted.samples.front().pose.translation()).norm() > 1e-6,
              "a shifted start actually begins somewhere else");
    }

    // --- orientation policies
    {
        CircleSpec spec = BaseSpec();
        spec.orientation = OrientationPolicy::kFixed;
        spec.fixed_rpy_rad = Eigen::Vector3d(0.1, -0.2, 0.3);
        const CartesianPath fixed = GenerateCircle(spec);
        const Eigen::Matrix3d first = fixed.samples.front().pose.linear();
        double worst = 0.0;
        for (const PathSample& sample : fixed.samples)
            worst = std::max(worst, (sample.pose.linear() - first).norm());
        Check(worst < 1e-12, "kFixed holds one orientation for the whole path");

        spec.orientation = OrientationPolicy::kRadialInward;
        const CartesianPath radial = GenerateCircle(spec);
        double worst_aim = 0.0, worst_orthonormal = 0.0;
        for (const PathSample& sample : radial.samples) {
            const Eigen::Matrix3d R = sample.pose.linear();
            // +z must point from the rim at the centre.
            const Eigen::Vector3d to_centre =
                (spec.centre_m - sample.pose.translation()).normalized();
            worst_aim = std::max(worst_aim, (R.col(2) - to_centre).norm());
            worst_orthonormal = std::max(
                worst_orthonormal,
                (R.transpose() * R - Eigen::Matrix3d::Identity()).norm());
            Check(R.determinant() > 0.0, "kRadialInward stays right-handed");
        }
        Check(worst_aim < 1e-12, "kRadialInward points +z at the centre at every sample");
        Check(worst_orthonormal < 1e-12, "kRadialInward emits orthonormal rotations");
    }

    // --- a spec that cannot describe a circle is refused, not silently
    //     turned into something the operator never asked for.
    {
        CircleSpec spec = BaseSpec();
        spec.radius_m = 0.0;
        Check(Throws(spec), "zero radius is refused");
        spec = BaseSpec(); spec.radius_m = -0.05;
        Check(Throws(spec), "negative radius is refused");
        spec = BaseSpec(); spec.samples = 2;
        Check(Throws(spec), "fewer than three samples is refused");
        spec = BaseSpec(); spec.duration_s = 0.0;
        Check(Throws(spec), "zero duration is refused");
        spec = BaseSpec(); spec.normal = Eigen::Vector3d::Zero();
        Check(Throws(spec), "a degenerate normal is refused");
        spec = BaseSpec(); spec.radius_m = std::nan("");
        Check(Throws(spec), "a non-finite radius is refused");
    }

    if (failures == 0) {
        std::printf("all CartesianPath tests passed\n");
        return 0;
    }
    std::printf("%d test(s) failed\n", failures);
    return 1;
}
