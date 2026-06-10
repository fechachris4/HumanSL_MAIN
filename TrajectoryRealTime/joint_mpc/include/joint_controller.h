#pragma once
// The MPC seam: anything that turns (state, goal) into a setpoint each tick.
// An MPC controller (Milestone 2) implements this exact interface.
#include "joint_types.h"

namespace jmpc {

class JointController {
public:
    virtual ~JointController() = default;

    // Produce the next setpoint given current state and the joint-space goal.
    virtual JointSetpoint compute(const JointState& s,
                                  const Eigen::VectorXd& q_goal) = 0;

    // Number of setpoints a single compute() conceptually plans ahead.
    // 1 for the trivial controllers; N for a receding-horizon MPC.
    virtual int horizon_steps() const { return 1; }

    // Re-seed any internal reference/warm-start to the given state. Called on a
    // bumpless mode switch so the newly-activated controller continues smoothly
    // from where the arm actually is. Stateless controllers can ignore it.
    virtual void reset(const JointState& /*s*/) {}
};

}  // namespace jmpc
