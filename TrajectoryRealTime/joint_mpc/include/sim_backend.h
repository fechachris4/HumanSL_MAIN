#pragma once
// Laptop stand-in for the real arm: a first-order-lag kinematic "robot".
// Drop-in role of joint_position_control_execution() minus all Kortex I/O.
// ALL safety clamping (velocity, position) lives here, so the seam is
// protected regardless of controller bugs.
#include <algorithm>
#include "joint_types.h"

namespace jmpc {

class SimBackend {
public:
    // q0: initial joint angles. lim: limits. tau: lag time constant (s).
    SimBackend(Eigen::VectorXd q0, JointLimits lim, double tau)
        : q_(std::move(q0)), lim_(std::move(lim)), tau_(tau) {
        dq_ = Eigen::VectorXd::Zero(q_.size());
    }

    // Integrate one control tick toward the commanded setpoint.
    void apply(const JointSetpoint& cmd, double dt) {
        const Eigen::VectorXd q_prev = q_;
        const Eigen::VectorXd target = clampPos(cmd.q_des);

        // First-order lag toward the (position-clamped) target.
        Eigen::VectorXd q_new = q_ + (target - q_) * (dt / tau_);

        // Velocity clamp: bound per-tick motion to dq_max * dt.
        for (int i = 0; i < q_new.size(); ++i) {
            const double maxstep = lim_.dq_max(i) * dt;
            const double d = q_new(i) - q_prev(i);
            if (d >  maxstep) q_new(i) = q_prev(i) + maxstep;
            if (d < -maxstep) q_new(i) = q_prev(i) - maxstep;
        }

        q_new = clampPos(q_new);
        dq_ = (q_new - q_prev) / dt;
        q_  = q_new;
        t_ += dt;
    }

    JointState state() const { return JointState{ q_, dq_, t_ }; }

private:
    Eigen::VectorXd clampPos(Eigen::VectorXd v) const {
        for (int i = 0; i < v.size(); ++i)
            v(i) = std::min(lim_.q_upper(i), std::max(lim_.q_lower(i), v(i)));
        return v;
    }

    Eigen::VectorXd q_, dq_;
    JointLimits lim_;
    double tau_;
    double t_ = 0.0;
};

}  // namespace jmpc
