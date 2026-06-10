// Standalone test harness for the joint-control MPC seam (Milestone 1).
// No gtest — matches the repo's standalone test_*.cpp precedent.
// Builds against bundled Eigen only; exits non-zero on any failure.
#include <iostream>
#include <fstream>
#include <deque>
#include <memory>

#include "gen3_limits.h"
#include "controllers.h"
#include "sim_backend.h"
#include "setpoint_queue.h"
#include "control_driver.h"

static int g_fail = 0;
static void check(bool cond, const char* msg) {
    std::cout << (cond ? "  PASS  " : "  FAIL  ") << msg << "\n";
    if (!cond) ++g_fail;
}

static void test_sim_converges() {
    jmpc::SimBackend sim(Eigen::VectorXd::Zero(7), jmpc::gen3_limits(), /*tau=*/0.1);
    Eigen::VectorXd goal = Eigen::VectorXd::Constant(7, 0.5);
    jmpc::JointSetpoint cmd{ goal, Eigen::VectorXd::Zero(7) };
    for (int i = 0; i < 2000; ++i) sim.apply(cmd, 0.002);  // 4 s @ 500 Hz
    check((sim.state().q - goal).norm() < 1e-3, "sim converges to a constant goal");
}

static void test_velocity_clamp() {
    jmpc::SimBackend sim(Eigen::VectorXd::Zero(7), jmpc::gen3_limits(), /*tau=*/0.001);
    jmpc::JointSetpoint cmd{ Eigen::VectorXd::Constant(7, 1.0), Eigen::VectorXd::Zero(7) };
    const double dt = 0.002;
    const Eigen::VectorXd q0 = sim.state().q;
    sim.apply(cmd, dt);
    const Eigen::VectorXd step = (sim.state().q - q0).cwiseAbs();
    check((step.array() <= 0.8727 * dt + 1e-9).all(), "velocity clamp caps per-tick motion");
}

static void test_position_clamp() {
    jmpc::SimBackend sim(Eigen::VectorXd::Zero(7), jmpc::gen3_limits(), /*tau=*/0.001);
    jmpc::JointSetpoint cmd{ Eigen::VectorXd::Constant(7, 1000.0), Eigen::VectorXd::Zero(7) };
    for (int i = 0; i < 5000; ++i) sim.apply(cmd, 0.002);
    auto L = jmpc::gen3_limits();
    const bool ok = (sim.state().q.array() <= L.q_upper.array() + 1e-9).all() &&
                    (sim.state().q.array() >= L.q_lower.array() - 1e-9).all();
    check(ok, "position clamp keeps q within limits");
}

static void test_rate_limited_controller() {
    const double dt = 0.002, vmax = 0.5;
    jmpc::RateLimitedController ctrl(vmax, dt);
    jmpc::JointState s;
    s.q = Eigen::VectorXd::Zero(7);
    s.dq = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd goal = Eigen::VectorXd::Constant(7, 0.2);

    auto sp = ctrl.compute(s, goal);
    check(((sp.q_des - s.q).cwiseAbs().array() <= vmax * dt + 1e-12).all(),
          "rate-limited controller caps each step");

    for (int i = 0; i < 10000 && (s.q - goal).norm() > 1e-6; ++i) {
        auto c = ctrl.compute(s, goal);
        s.q = c.q_des;
    }
    check((s.q - goal).norm() < 1e-6, "rate-limited controller reaches goal");
}

static void test_setpoint_queue() {
    jmpc::SetpointQueue q;
    std::deque<jmpc::JointSetpoint> h;
    for (int k = 0; k < 3; ++k)
        h.push_back({ Eigen::VectorXd::Constant(7, k), Eigen::VectorXd::Zero(7) });
    q.replace(h);
    check(q.next()->q_des(0) == 0, "queue pops first");
    check(q.next()->q_des(0) == 1, "queue pops second");
    check(q.next()->q_des(0) == 2, "queue pops third");
    check(q.next()->q_des(0) == 2, "queue holds last when one remains");

    std::deque<jmpc::JointSetpoint> h2{ { Eigen::VectorXd::Constant(7, 9), Eigen::VectorXd::Zero(7) } };
    q.replace(h2);
    check(q.next()->q_des(0) == 9, "queue replace swaps the horizon");
}

static void test_driver_end_to_end() {
    auto L = jmpc::gen3_limits();
    const double dt = 0.002;
    auto ctrl = std::make_shared<jmpc::RateLimitedController>(0.5, dt);
    jmpc::SimBackend sim(Eigen::VectorXd::Zero(7), L, /*tau=*/0.05);
    jmpc::ControlDriver drv(ctrl, sim, dt);
    Eigen::VectorXd goal = Eigen::VectorXd::Constant(7, 0.3);
    const std::string csv = "run_log.csv";
    drv.run(/*seconds=*/5.0, goal, csv);
    check((drv.backend().state().q - goal).norm() < 1e-2, "driver converges end-to-end");

    std::ifstream f(csv);
    check(f.good() && f.peek() != std::ifstream::traits_type::eof(), "driver wrote a non-empty CSV");
}

int main() {
    std::cout << "=== joint_mpc Milestone 1 tests ===\n";
    test_sim_converges();
    test_velocity_clamp();
    test_position_clamp();
    test_rate_limited_controller();
    test_setpoint_queue();
    test_driver_end_to_end();
    std::cout << (g_fail ? "RESULT: FAILURES\n" : "RESULT: ALL PASS\n");
    return g_fail ? 1 : 0;
}
