//
// Feasibility — graded supervision of the pose-primary controller (slice 4
// of the world-frame architecture). Everything here is ADVISORY: these
// functions measure how much margin the controller has left and raise a
// debounced "replan advised" level when it is eroding. They never stop,
// clamp, or veto anything — motion authority stays with the existing
// safety pipeline, and the layer with context (operator, session script,
// UI) decides what to do with the advice. Pure Eigen, no I/O, no model.
//

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Dense>

// The worst bounded joint's distance to its software limit, radians.
// `limit_rad` is the same symmetric-magnitude vector the reactive law uses
// (0 = unbounded joint); positions wrap to (-pi, pi] because Kortex reports
// [0, 360). +infinity when no joint is bounded; negative once a joint is
// past its limit (the margin keeps grading, it does not saturate at zero).
inline double JointLimitMarginRad(const Eigen::Matrix<double, 7, 1>& q_rad,
                                  const Eigen::Matrix<double, 7, 1>& limit_rad)
{
    double margin = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 7; ++i) {
        if (limit_rad[i] <= 0.0)
            continue;
        const double signed_rad = std::remainder(q_rad[i], 2.0 * M_PI);
        margin = std::min(margin, limit_rad[i] - std::abs(signed_rad));
    }
    return margin;
}

// The advisory thresholds, radians/meters. Values come from Config.h.
struct FeasibilityThresholds {
    double sigma_min = 0.0;         // advise BELOW this
    double joint_margin_rad = 0.0;  // advise BELOW this
    double posture_error_rad = 0.0; // advise ABOVE this
    double position_error_m = 0.0;  // advise ABOVE this
};

// One cycle's degradation vote. A NaN measure abstains — a cycle that did
// not compute a quantity must not count as evidence either way (the same
// NaN convention ControllerStatus uses). Any finite measure past its
// threshold votes degraded.
inline bool FeasibilityDegraded(double sigma_min, double joint_margin_rad,
                                double posture_error_rad,
                                double position_error_m,
                                const FeasibilityThresholds& t)
{
    const bool sigma_bad =
        std::isfinite(sigma_min) && sigma_min < t.sigma_min;
    const bool margin_bad =
        std::isfinite(joint_margin_rad) && joint_margin_rad < t.joint_margin_rad;
    const bool posture_bad = std::isfinite(posture_error_rad) &&
                             posture_error_rad > t.posture_error_rad;
    const bool tracking_bad = std::isfinite(position_error_m) &&
                              position_error_m > t.position_error_m;
    return sigma_bad || margin_bad || posture_bad || tracking_bad;
}

// Debounce: the advisory is a LEVEL that comes up only after `cycles`
// consecutive degraded cycles and drops the first healthy one — a single
// noisy sample never advises, a sustained erosion always does.
class ReplanAdvisor
{
public:
    explicit ReplanAdvisor(int cycles) : cycles_(cycles) {}

    bool Update(bool degraded)
    {
        count_ = degraded ? count_ + 1 : 0;
        return cycles_ > 0 && count_ >= cycles_;
    }

private:
    int cycles_;
    int count_ = 0;
};
