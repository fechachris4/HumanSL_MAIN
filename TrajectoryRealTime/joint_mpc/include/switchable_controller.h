#pragma once
// Runtime toggle between a "simple" controller (e.g. RateLimited) and the MPC.
// Delegates compute() to whichever is active. Flipping the toggle performs a
// BUMPLESS transfer: the newly-activated controller is reset() to the current
// measured state, so its internal reference continues from where the arm actually
// is instead of jumping. The driver is unchanged — this is just a JointController.
//
// In the app, set_use_mpc() is what a "MPC on/off" voice command or CLI flag drives.
#include <memory>
#include "joint_controller.h"

namespace jmpc {

class SwitchableController : public JointController {
public:
    SwitchableController(std::shared_ptr<JointController> simple,
                         std::shared_ptr<JointController> mpc,
                         bool use_mpc = false)
        : simple_(std::move(simple)), mpc_(std::move(mpc)), use_mpc_(use_mpc) {}

    // Toggle. `s` is the current measured state, used for the bumpless reset.
    void set_use_mpc(bool on, const JointState& s) {
        if (on == use_mpc_) return;
        use_mpc_ = on;
        active()->reset(s);   // re-seed the now-active controller to reality
    }

    bool use_mpc() const { return use_mpc_; }

    JointSetpoint compute(const JointState& s, const Eigen::VectorXd& goal) override {
        return active()->compute(s, goal);
    }

    int horizon_steps() const override { return active()->horizon_steps(); }

    void reset(const JointState& s) override { active()->reset(s); }

private:
    JointController* active() { return use_mpc_ ? mpc_.get() : simple_.get(); }
    const JointController* active() const { return use_mpc_ ? mpc_.get() : simple_.get(); }

    std::shared_ptr<JointController> simple_;
    std::shared_ptr<JointController> mpc_;
    bool use_mpc_;
};

}  // namespace jmpc
