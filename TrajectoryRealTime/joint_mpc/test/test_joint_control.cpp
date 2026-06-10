// Standalone test harness for the joint-control MPC seam (Milestones 1 & 2).
// No gtest — matches the repo's standalone test_*.cpp precedent.
// Builds against bundled Eigen only; exits non-zero on any failure.
#include <iostream>
#include <fstream>
#include <deque>
#include <memory>
#include <algorithm>
#include <cmath>

#include "gen3_limits.h"
#include "controllers.h"
#include "sim_backend.h"
#include "setpoint_queue.h"
#include "control_driver.h"
#include "mpc_controller.h"
#include "switchable_controller.h"

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

// ----------------------------- Milestone 2 -----------------------------------

static void test_hold_controller() {
    jmpc::HoldController hold;
    jmpc::JointState s;
    s.q = Eigen::VectorXd::Constant(7, 0.4);
    s.dq = Eigen::VectorXd::Zero(7);
    auto sp = hold.compute(s, Eigen::VectorXd::Constant(7, 9.9));  // goal ignored
    check((sp.q_des - s.q).norm() < 1e-12, "hold controller holds current q");
    check(sp.dq_des.norm() < 1e-12, "hold controller commands zero velocity");
}

// Drive the MPC controller open-loop as a reference generator (feed its own output
// back as the measured state) and report ticks-to-goal + velocity profile stats.
static int mpc_ticks_to_goal(double w_u, double* peak_v = nullptr, double* first_v = nullptr,
                             double goal_val = 0.3) {
    auto L = jmpc::gen3_limits();
    jmpc::MpcParams p; p.dt = 0.002; p.w_u = w_u;
    jmpc::MpcJointController mpc(p, L);
    jmpc::JointState s; s.q = Eigen::VectorXd::Zero(7); s.dq = Eigen::VectorXd::Zero(7);
    mpc.reset(s);
    Eigen::VectorXd goal = Eigen::VectorXd::Constant(7, goal_val);
    double peak = 0.0;
    for (int k = 0; k < 8000; ++k) {
        auto sp = mpc.compute(s, goal);
        const double v = sp.dq_des.cwiseAbs().maxCoeff();
        if (k == 0 && first_v) *first_v = v;
        peak = std::max(peak, v);
        s.q = sp.q_des; s.dq = sp.dq_des;
        if ((s.q - goal).norm() < 1e-3) { if (peak_v) *peak_v = peak; return k; }
    }
    if (peak_v) *peak_v = peak;
    return 8000;
}

static void test_mpc_converges() {
    auto L = jmpc::gen3_limits();
    const double dt = 0.002;
    jmpc::MpcParams p; p.dt = dt;
    auto mpc = std::make_shared<jmpc::MpcJointController>(p, L);
    jmpc::SimBackend sim(Eigen::VectorXd::Zero(7), L, /*tau=*/0.05);
    jmpc::ControlDriver drv(mpc, sim, dt);
    Eigen::VectorXd goal = Eigen::VectorXd::Constant(7, 0.3);
    drv.run(/*seconds=*/6.0, goal, "mpc_run.csv");
    const double err = (drv.backend().state().q - goal).norm();
    check(err < 1e-2, "MPC converges end-to-end");
}

static void test_mpc_respects_velocity_limit() {
    // Large goal + aggressive effort weight so the unclamped MPC WANTS to exceed the
    // 0.8727 rad/s limit — this actually exercises the clamp (a 0.3 rad goal never does).
    double peak = 0.0;
    mpc_ticks_to_goal(/*w_u=*/0.001, &peak, nullptr, /*goal_val=*/2.5);
    check(peak > 0.85, "velocity clamp is actually exercised (MPC saturates the limit)");
    check(peak <= 0.8727 + 1e-9, "MPC reference never exceeds the joint velocity limit");
}

static void test_mpc_effort_weight_changes_behavior() {
    const int fast = mpc_ticks_to_goal(/*w_u=*/0.005);
    const int slow = mpc_ticks_to_goal(/*w_u=*/2.0);
    check(slow > fast, "higher effort weight slows the MPC (cost weights change behavior)");
}

static void test_mpc_smooth_start() {
    double peak = 0.0, first = 0.0;
    mpc_ticks_to_goal(/*w_u=*/0.05, &peak, &first);
    check(first < 0.9 * peak, "MPC accelerates gradually (peak velocity is not at t=0)");
}

static void test_toggle_switches_output() {
    auto L = jmpc::gen3_limits();
    const double dt = 0.002;
    jmpc::MpcParams p; p.dt = dt;
    auto simple = std::make_shared<jmpc::RateLimitedController>(0.5, dt);
    auto mpc    = std::make_shared<jmpc::MpcJointController>(p, L);
    jmpc::SwitchableController sw(simple, mpc, /*use_mpc=*/false);
    Eigen::VectorXd goal = Eigen::VectorXd::Constant(7, 0.3);
    check(!sw.use_mpc(), "toggle starts on the simple controller");

    // OFF: the switchable must delegate EXACTLY to a standalone simple controller
    // (over many steps), not merely "differ from MPC".
    jmpc::RateLimitedController simple_ref(0.5, dt);
    jmpc::JointState s; s.q = Eigen::VectorXd::Zero(7); s.dq = Eigen::VectorXd::Zero(7);
    sw.reset(s); simple_ref.reset(s);
    double off_diff = 0.0;
    for (int k = 0; k < 50; ++k) {
        auto a = sw.compute(s, goal);
        auto b = simple_ref.compute(s, goal);
        off_diff = std::max(off_diff, (a.q_des - b.q_des).cwiseAbs().maxCoeff());
        s.q = a.q_des; s.dq = a.dq_des;
    }
    check(off_diff < 1e-12, "toggle OFF delegates exactly to the simple controller");

    // ON: the switchable must delegate EXACTLY to a standalone MPC, and the MPC and
    // simple branches must genuinely differ on the same state trajectory.
    jmpc::JointState s2; s2.q = Eigen::VectorXd::Zero(7); s2.dq = Eigen::VectorXd::Zero(7);
    sw.set_use_mpc(true, s2);
    jmpc::MpcJointController mpc_ref(p, L);            mpc_ref.reset(s2);
    jmpc::RateLimitedController simple_only(0.5, dt);  simple_only.reset(s2);
    check(sw.use_mpc(), "toggle reports MPC active after switch-on");
    double on_diff = 0.0, branch_diff = 0.0;
    for (int k = 0; k < 50; ++k) {
        auto a = sw.compute(s2, goal);
        auto b = mpc_ref.compute(s2, goal);
        auto c = simple_only.compute(s2, goal);
        on_diff     = std::max(on_diff,     (a.q_des - b.q_des).cwiseAbs().maxCoeff());
        branch_diff = std::max(branch_diff, (b.q_des - c.q_des).cwiseAbs().maxCoeff());
        s2.q = a.q_des; s2.dq = a.dq_des;
    }
    check(on_diff < 1e-12, "toggle ON delegates exactly to the MPC");
    check(branch_diff > 1e-4, "MPC and simple controllers genuinely differ");
}

static void test_toggle_bumpless() {
    auto L = jmpc::gen3_limits();
    const double dt = 0.002;
    auto simple = std::make_shared<jmpc::RateLimitedController>(0.5, dt);
    jmpc::MpcParams p; p.dt = dt;
    auto mpc = std::make_shared<jmpc::MpcJointController>(p, L);
    auto sw = std::make_shared<jmpc::SwitchableController>(simple, mpc, /*use_mpc=*/false);
    jmpc::SimBackend sim(Eigen::VectorXd::Zero(7), L, /*tau=*/0.05);
    jmpc::ControlDriver drv(sw, sim, dt);
    Eigen::VectorXd goal = Eigen::VectorXd::Constant(7, 0.3);

    for (int k = 0; k < 200; ++k) drv.step(goal);   // move under the simple controller
    const jmpc::JointState before = drv.backend().state();
    check(before.dq.cwiseAbs().maxCoeff() > 0.3, "arm is genuinely moving at switch time");

    sw->set_use_mpc(true, before);                   // bumpless switch to MPC
    const jmpc::JointSetpoint sp = drv.step(goal);   // first MPC tick

    const double pos_jump = (sp.q_des - before.q).cwiseAbs().maxCoeff();
    const double vel_jump = (sp.dq_des - before.dq).cwiseAbs().maxCoeff();
    check(pos_jump <= 0.8727 * dt + 1e-6, "switch is position-bumpless (no position jump)");
    // The real test: seeding v_ref=measured dq means the commanded velocity does NOT
    // snap to zero. (With the old reset-to-rest, vel_jump would be ~the full 0.5 rad/s.)
    check(vel_jump < 0.05, "switch is velocity-bumpless (no velocity discontinuity)");
}

// A controller that emits NaN, to exercise the driver's hold-last safety path.
struct NaNController : public jmpc::JointController {
    jmpc::JointSetpoint compute(const jmpc::JointState& s, const Eigen::VectorXd&) override {
        return { Eigen::VectorXd::Constant(s.q.size(), std::nan("")),
                 Eigen::VectorXd::Zero(s.q.size()) };
    }
};

static void test_driver_nan_hold() {
    auto L = jmpc::gen3_limits();
    const double dt = 0.002;
    auto nanc = std::make_shared<NaNController>();
    jmpc::SimBackend sim(Eigen::VectorXd::Zero(7), L, /*tau=*/0.05);
    jmpc::ControlDriver drv(nanc, sim, dt);
    Eigen::VectorXd goal = Eigen::VectorXd::Constant(7, 0.3);
    for (int k = 0; k < 100; ++k) drv.step(goal);
    const Eigen::VectorXd q = drv.backend().state().q;
    check(q.allFinite() && q.norm() < 1e-9,
          "driver holds last setpoint on NaN (no NaN propagation, no motion)");
}

static void test_mpc_distinct_goals() {
    auto L = jmpc::gen3_limits();
    jmpc::MpcParams p; p.dt = 0.002;
    jmpc::MpcJointController mpc(p, L);
    jmpc::JointState s; s.q = Eigen::VectorXd::Zero(7); s.dq = Eigen::VectorXd::Zero(7);
    mpc.reset(s);
    Eigen::VectorXd goal(7); goal << 0.1, -0.2, 0.3, -0.1, 0.2, -0.3, 0.15;
    for (int k = 0; k < 6000; ++k) { auto sp = mpc.compute(s, goal); s.q = sp.q_des; s.dq = sp.dq_des; }
    check((s.q - goal).norm() < 1e-2, "MPC reaches distinct per-joint goals");
}

static void test_mpc_determinism() {
    auto run_once = []() {
        auto L = jmpc::gen3_limits();
        jmpc::MpcParams p; p.dt = 0.002;
        jmpc::MpcJointController mpc(p, L);
        jmpc::JointState s; s.q = Eigen::VectorXd::Zero(7); s.dq = Eigen::VectorXd::Zero(7);
        mpc.reset(s);
        Eigen::VectorXd goal = Eigen::VectorXd::Constant(7, 0.25);
        for (int k = 0; k < 1000; ++k) { auto sp = mpc.compute(s, goal); s.q = sp.q_des; s.dq = sp.dq_des; }
        return s.q;
    };
    check((run_once() - run_once()).norm() == 0.0,
          "MPC is deterministic (identical runs match exactly)");
}

int main() {
    std::cout << "=== joint_mpc Milestone 1 tests ===\n";
    test_sim_converges();
    test_velocity_clamp();
    test_position_clamp();
    test_rate_limited_controller();
    test_setpoint_queue();
    test_driver_end_to_end();
    std::cout << "=== joint_mpc Milestone 2 tests (MPC + toggle) ===\n";
    test_hold_controller();
    test_mpc_converges();
    test_mpc_respects_velocity_limit();
    test_mpc_effort_weight_changes_behavior();
    test_mpc_smooth_start();
    test_toggle_switches_output();
    test_toggle_bumpless();
    test_driver_nan_hold();
    test_mpc_distinct_goals();
    test_mpc_determinism();
    std::cout << (g_fail ? "RESULT: FAILURES\n" : "RESULT: ALL PASS\n");
    return g_fail ? 1 : 0;
}
