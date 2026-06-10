#pragma once
// Trivial Milestone-1 controllers that prove the seam end-to-end.
// The QP MPC (Milestone 2) is just another JointController.
#include "joint_controller.h"

namespace jmpc {

// Hold the current configuration. Safety baseline / no-op goal.
class HoldController : public JointController {
public:
    JointSetpoint compute(const JointState& s, const Eigen::VectorXd&) override {
        return JointSetpoint{ s.q, Eigen::VectorXd::Zero(s.q.size()) };
    }
};

// Reference generator: ramps an internal setpoint toward the goal at a capped
// per-joint speed (v_max, rad/s) per tick (dt, s). It integrates its OWN reference
// rather than re-anchoring to the (possibly lagging) measurement, so it produces a
// clean ramp regardless of plant lag — the same thing plan.cpp's trajectories do.
// Stateful: the reference is lazily initialized to the first measured q.
class RateLimitedController : public JointController {
public:
    RateLimitedController(double v_max, double dt) : v_max_(v_max), dt_(dt) {}

    JointSetpoint compute(const JointState& s, const Eigen::VectorXd& goal) override {
        if (!initialized_) { ref_ = s.q; initialized_ = true; }
        const Eigen::VectorXd prev_ref = ref_;
        const double maxstep = v_max_ * dt_;
        for (int i = 0; i < goal.size(); ++i) {
            double d = goal(i) - ref_(i);
            if (d >  maxstep) d =  maxstep;
            if (d < -maxstep) d = -maxstep;
            ref_(i) += d;
        }
        return JointSetpoint{ ref_, (ref_ - prev_ref) / dt_ };
    }

    // Position-bumpless: continue the ramp from the current measured configuration.
    // (This controller has no velocity state; it always ramps at +/-v_max, so a switch
    // into it is not velocity-continuous — acceptable for a bang-bang reference.)
    void reset(const JointState& s) override { ref_ = s.q; initialized_ = true; }

private:
    double v_max_;
    double dt_;
    bool initialized_ = false;
    Eigen::VectorXd ref_;
};

}  // namespace jmpc
