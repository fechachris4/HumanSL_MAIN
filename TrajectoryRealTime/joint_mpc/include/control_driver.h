#pragma once
// Ties controller + setpoint queue + backend into a deterministic, single-threaded
// receding-horizon loop. step() runs one tick; run() loops for a duration and logs
// t, q, q_des, goal to CSV. The threaded real-time wrapper and the JointTrajectory
// robot adapter (Milestone 2) reuse this exact control flow.
#include <memory>
#include <fstream>
#include <string>
#include <deque>
#include "controllers.h"
#include "sim_backend.h"
#include "setpoint_queue.h"

namespace jmpc {

class ControlDriver {
public:
    ControlDriver(std::shared_ptr<JointController> ctrl, SimBackend backend, double dt)
        : ctrl_(std::move(ctrl)), backend_(std::move(backend)), dt_(dt) {
        last_good_ = JointSetpoint{ backend_.state().q,
                                    Eigen::VectorXd::Zero(backend_.state().q.size()) };
    }

    // One control tick: compute -> replace horizon -> apply next setpoint.
    // Returns the setpoint actually applied (after the NaN-hold guard).
    JointSetpoint step(const Eigen::VectorXd& goal) {
        const JointState s = backend_.state();
        JointSetpoint sp = ctrl_->compute(s, goal);
        if (!sp.q_des.allFinite()) sp = last_good_;   // hold last on NaN
        else last_good_ = sp;
        queue_.replace(std::deque<JointSetpoint>{ sp });
        if (auto next = queue_.next()) backend_.apply(*next, dt_);
        return sp;
    }

    // Run for `seconds`, logging each tick to csv_path.
    void run(double seconds, const Eigen::VectorXd& goal, const std::string& csv_path) {
        std::ofstream log(csv_path);
        const int n = static_cast<int>(goal.size());
        log << "t";
        for (int i = 0; i < n; ++i) log << ",q" << i;
        for (int i = 0; i < n; ++i) log << ",qdes" << i;
        for (int i = 0; i < n; ++i) log << ",goal" << i;
        log << "\n";

        const int ticks = static_cast<int>(seconds / dt_);
        for (int k = 0; k < ticks; ++k) {
            const JointSetpoint sp = step(goal);   // single source of control logic
            const JointState a = backend_.state();
            log << a.t;
            for (int i = 0; i < n; ++i) log << "," << a.q(i);
            for (int i = 0; i < n; ++i) log << "," << sp.q_des(i);
            for (int i = 0; i < n; ++i) log << "," << goal(i);
            log << "\n";
        }
    }

    const SimBackend& backend() const { return backend_; }

private:
    std::shared_ptr<JointController> ctrl_;
    SimBackend backend_;
    SetpointQueue queue_;
    double dt_;
    JointSetpoint last_good_;
};

}  // namespace jmpc
