#include "InterArmDistance.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <vector>

namespace {

// Sphere centres for one configuration, in `mount`. Both arms' models are
// built at DhRootInMount(left_arm), so these are directly comparable across
// arms with no further transform.
std::vector<Eigen::Vector3d> SphereCentres(const PlannerModel& model,
                                           const Eigen::Matrix<double, 7, 1>& q) {
    const gtsam::Matrix centres = model.arm_model->sphereCentersMat(gtsam::Vector(q));
    std::vector<Eigen::Vector3d> out;
    out.reserve(static_cast<std::size_t>(centres.cols()));
    for (int i = 0; i < centres.cols(); ++i)
        out.emplace_back(centres(0, i), centres(1, i), centres(2, i));
    return out;
}

}  // namespace

std::string InterArmClearanceReport::Summary() const {
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(2);
    out << "== inter-arm clearance ==\n";
    if (!evaluated) {
        out << "  NOT EVALUATED: " << error << "\n";
        return out.str();
    }
    out << "  minimum clearance " << minimum_clearance_m * 1000.0 << " mm (required "
        << required_clearance_m * 1000.0 << " mm) — "
        << (clearance_satisfied ? "satisfied" : "VIOLATED") << "\n"
        << "  at left t = " << time_of_minimum_s << " s vs right t = "
        << right_time_of_minimum_s << " s\n"
        << "  responsible pair: left sphere " << closest_pair.left_sphere << " (r "
        << closest_pair.left_radius_m * 1000.0 << " mm) vs right sphere "
        << closest_pair.right_sphere << " (r "
        << closest_pair.right_radius_m * 1000.0 << " mm)\n"
        << "  checked " << samples_checked << " grid times over a start-skew window of +-"
        << skew_window_s * 1000.0 << " ms\n"
        << "  (compared at CORRESPONDING TIMES over that window, not against a "
           "static swept volume; tracking error is not modelled)\n";
    return out.str();
}

InterArmClearanceReport CheckInterArmClearance(
    const PlannerModel& left_model, const std::string& left_block,
    const PlannerModel& right_model, const std::string& right_block,
    double required_clearance_m, double skew_window_s, double grid_dt_s) {

    InterArmClearanceReport report;
    report.required_clearance_m = required_clearance_m;
    report.skew_window_s = skew_window_s;

    const double dt = grid_dt_s > 0.0 ? grid_dt_s : 0.002;
    const ReconstructedTrajectory left = ReconstructEmittedBlock(left_block, dt);
    const ReconstructedTrajectory right = ReconstructEmittedBlock(right_block, dt);
    if (!left.parsed) {
        report.error = "left block did not parse: " + left.error;
        return report;
    }
    if (!right.parsed) {
        report.error = "right block did not parse: " + right.error;
        return report;
    }
    if (left.samples.empty() || right.samples.empty()) {
        report.error = "a reconstructed trajectory was empty";
        return report;
    }

    // Precompute the right arm's sphere centres once per grid time. Without
    // this the skew window would re-evaluate the same forward kinematics
    // once per offset, which is the dominant cost.
    std::vector<std::vector<Eigen::Vector3d>> right_centres;
    right_centres.reserve(right.samples.size());
    for (const ReconstructedSample& sample : right.samples)
        right_centres.push_back(SphereCentres(right_model, sample.q_rad));

    const std::size_t left_spheres = left_model.arm_model->nr_body_spheres();
    const std::size_t right_spheres = right_model.arm_model->nr_body_spheres();
    const int window_steps =
        static_cast<int>(std::ceil(std::max(0.0, skew_window_s) / dt));

    double best = std::numeric_limits<double>::infinity();
    for (std::size_t li = 0; li < left.samples.size(); ++li) {
        const std::vector<Eigen::Vector3d> left_centres =
            SphereCentres(left_model, left.samples[li].q_rad);

        // The skew window. Activation is not atomic, so the two arms may be
        // offset from each other by up to the configured maximum; checking
        // only li against li would verify a synchronisation the system does
        // not provide.
        const std::size_t lo =
            static_cast<std::size_t>(std::max<long long>(
                0, static_cast<long long>(li) - window_steps));
        const std::size_t hi = std::min(right.samples.size() - 1,
                                        li + static_cast<std::size_t>(window_steps));
        for (std::size_t ri = lo; ri <= hi; ++ri) {
            for (std::size_t a = 0; a < left_spheres && a < left_centres.size(); ++a) {
                const double ra = left_model.arm_model->sphere_radius(a);
                for (std::size_t b = 0; b < right_spheres && b < right_centres[ri].size();
                     ++b) {
                    const double rb = right_model.arm_model->sphere_radius(b);
                    // Surface to surface, so zero means touching and
                    // negative means the models overlap.
                    const double clearance =
                        (left_centres[a] - right_centres[ri][b]).norm() - ra - rb;
                    if (clearance < best) {
                        best = clearance;
                        report.time_of_minimum_s = left.samples[li].t_s;
                        report.right_time_of_minimum_s = right.samples[ri].t_s;
                        report.closest_pair.left_sphere = a;
                        report.closest_pair.right_sphere = b;
                        report.closest_pair.left_radius_m = ra;
                        report.closest_pair.right_radius_m = rb;
                        report.closest_pair.left_centre_m = left_centres[a];
                        report.closest_pair.right_centre_m = right_centres[ri][b];
                    }
                }
            }
        }
        ++report.samples_checked;
    }

    report.evaluated = true;
    report.minimum_clearance_m = best;
    report.clearance_satisfied = best >= required_clearance_m;
    return report;
}
