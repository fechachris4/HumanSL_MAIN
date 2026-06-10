# Joint-Control MPC Seam — Implementation Plan (Milestone 1)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement task-by-task. Steps use checkbox (`- [ ]`) syntax.

**Goal:** A laptop-testable, Eigen-only joint-control seam (controller interface +
first-order-lag sim backend + setpoint queue + driver) that a real MPC and a robot
adapter plug into later.

**Architecture:** Header-only, single-threaded, deterministic units under
`TrajectoryRealTime/joint_mpc/`. A standalone `CMakeLists` builds a test harness against
bundled Eigen + pthread only. No GTSAM/Kortex/yaml-cpp in the core.

**Tech Stack:** C++17, bundled Eigen3 (`third_party/include/eigen3`), CMake, pthread.

---

## File Structure

```
TrajectoryRealTime/joint_mpc/
  CMakeLists.txt                 standalone, Eigen + Threads only
  README.md                      build/run instructions
  include/
    joint_types.h                JointState, JointSetpoint, JointLimits
    gen3_limits.h                Gen3 limits (mirror config/joint_limits.yaml)
    joint_controller.h           JointController abstract interface
    controllers.h                HoldController, RateLimitedController
    sim_backend.h                SimBackend (first-order lag + clamps)
    setpoint_queue.h             SetpointQueue (thread-safe deque, replace/next)
    control_driver.h             ControlDriver (step / run + CSV)
  test/
    test_joint_control.cpp       standalone harness, check() asserts, returns nonzero on fail
```

`config/joint_limits.yaml` is the source of truth the hardcoded `gen3_limits.h` mirrors.

---

## Task 1: Project skeleton + bundled-Eigen build sanity

**Files:** Create `TrajectoryRealTime/joint_mpc/CMakeLists.txt`,
`TrajectoryRealTime/joint_mpc/test/test_joint_control.cpp` (stub).

- [ ] **Step 1: Stub harness that includes Eigen**

```cpp
// test/test_joint_control.cpp
#include <Eigen/Dense>
#include <iostream>
int main() { Eigen::VectorXd v = Eigen::VectorXd::Zero(7); std::cout << "eigen ok " << v.size() << "\n"; return 0; }
```

- [ ] **Step 2: CMakeLists pointing at bundled Eigen**

```cmake
cmake_minimum_required(VERSION 3.12)
project(joint_mpc CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# Bundled Eigen lives at repo_root/third_party/include/eigen3
set(EIGEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/include/eigen3")
include_directories(${EIGEN_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)
find_package(Threads REQUIRED)
add_executable(test_joint_control test/test_joint_control.cpp)
target_link_libraries(test_joint_control Threads::Threads)
```

- [ ] **Step 3: Build & run**

Run: `cd TrajectoryRealTime/joint_mpc && cmake -S . -B build && cmake --build build && ./build/test_joint_control`
Expected: prints `eigen ok 7`, exits 0.

- [ ] **Step 4: Commit** — `feat(joint_mpc): skeleton + bundled-Eigen build`

---

## Task 2: Shared types + Gen3 limits

**Files:** Create `include/joint_types.h`, `include/gen3_limits.h`.

- [ ] **Step 1: `joint_types.h`**

```cpp
#pragma once
#include <Eigen/Dense>
namespace jmpc {
struct JointState    { Eigen::VectorXd q, dq; double t = 0.0; };
struct JointSetpoint { Eigen::VectorXd q_des, dq_des; };
struct JointLimits   { Eigen::VectorXd q_lower, q_upper, dq_max; };
}
```

- [ ] **Step 2: `gen3_limits.h`** (mirrors `config/joint_limits.yaml`, radians)

```cpp
#pragma once
#include "joint_types.h"
namespace jmpc {
constexpr int kDof = 7;
inline JointLimits gen3_limits() {
  JointLimits L;
  L.q_lower = Eigen::VectorXd(7); L.q_lower << -1e20,-2.2515,-1e20,-2.5807,-1e20,-2.0996,-1e20;
  L.q_upper = Eigen::VectorXd(7); L.q_upper <<  1e20, 2.2515, 1e20, 2.5807, 1e20, 2.0996, 1e20;
  L.dq_max  = Eigen::VectorXd::Constant(7, 0.8727); // rad/s, Table 41
  return L;
}
}
```

- [ ] **Step 3: Compile-only test** — add to harness:

```cpp
// in test_joint_control.cpp, replace body incrementally as tasks land
#include "gen3_limits.h"
// assert sizes
```

- [ ] **Step 4: Build, run, commit** — `feat(joint_mpc): shared types + Gen3 limits`

---

## Task 3: SimBackend (TDD — convergence + clamps)

**Files:** Create `include/sim_backend.h`; tests in `test/test_joint_control.cpp`.

- [ ] **Step 1: Write failing tests**

```cpp
#include "sim_backend.h"
// (1) converges to constant goal
{
  jmpc::SimBackend sim(Eigen::VectorXd::Zero(7), jmpc::gen3_limits(), /*tau=*/0.1);
  Eigen::VectorXd goal = Eigen::VectorXd::Constant(7, 0.5);
  jmpc::JointSetpoint cmd{goal, Eigen::VectorXd::Zero(7)};
  for (int i=0;i<2000;++i) sim.apply(cmd, 0.002);   // 4 s at 500 Hz
  check((sim.state().q - goal).norm() < 1e-3, "sim converges to goal");
}
// (2) velocity clamp
{
  jmpc::SimBackend sim(Eigen::VectorXd::Zero(7), jmpc::gen3_limits(), 0.001); // tiny tau -> wants to jump
  jmpc::JointSetpoint cmd{Eigen::VectorXd::Constant(7,1.0), Eigen::VectorXd::Zero(7)};
  double dt = 0.002;
  Eigen::VectorXd q0 = sim.state().q;
  sim.apply(cmd, dt);
  Eigen::VectorXd step = (sim.state().q - q0).cwiseAbs();
  check((step.array() <= 0.8727*dt + 1e-9).all(), "velocity clamp caps per-tick motion");
}
// (3) position clamp
{
  jmpc::SimBackend sim(Eigen::VectorXd::Zero(7), jmpc::gen3_limits(), 0.001);
  jmpc::JointSetpoint cmd{Eigen::VectorXd::Constant(7,1000.0), Eigen::VectorXd::Zero(7)};
  for (int i=0;i<5000;++i) sim.apply(cmd, 0.002);
  auto L = jmpc::gen3_limits();
  check((sim.state().q.array() <= L.q_upper.array()+1e-9).all() &&
        (sim.state().q.array() >= L.q_lower.array()-1e-9).all(), "position clamp holds");
}
```

- [ ] **Step 2: Run, verify FAIL** (no `sim_backend.h`). Expected: compile error.

- [ ] **Step 3: Implement `sim_backend.h`**

```cpp
#pragma once
#include "joint_types.h"
namespace jmpc {
class SimBackend {
public:
  SimBackend(Eigen::VectorXd q0, JointLimits lim, double tau)
    : q_(std::move(q0)), lim_(std::move(lim)), tau_(tau) { dq_ = Eigen::VectorXd::Zero(q_.size()); }

  void apply(const JointSetpoint& cmd, double dt) {
    const Eigen::VectorXd q_prev = q_;
    Eigen::VectorXd target = clampPos(cmd.q_des);
    Eigen::VectorXd q_new = q_ + (target - q_) * (dt / tau_);   // first-order lag
    // velocity clamp
    for (int i = 0; i < q_new.size(); ++i) {
      const double vmax = lim_.dq_max(i), maxstep = vmax * dt;
      const double d = q_new(i) - q_prev(i);
      if (d >  maxstep) q_new(i) = q_prev(i) + maxstep;
      if (d < -maxstep) q_new(i) = q_prev(i) - maxstep;
    }
    q_new = clampPos(q_new);
    dq_ = (q_new - q_prev) / dt;
    q_  = q_new;
    t_ += dt;
  }
  JointState state() const { return JointState{q_, dq_, t_}; }

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
}
```

- [ ] **Step 4: Run, verify PASS. Commit** — `feat(joint_mpc): first-order-lag sim backend`

---

## Task 4: Controllers (TDD — rate-limited reaches goal)

**Files:** Create `include/joint_controller.h`, `include/controllers.h`; extend harness.

- [ ] **Step 1: Failing test**

```cpp
#include "controllers.h"
{
  double dt = 0.002, vmax = 0.5;
  jmpc::RateLimitedController ctrl(vmax, dt);
  jmpc::JointState s; s.q = Eigen::VectorXd::Zero(7); s.dq = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd goal = Eigen::VectorXd::Constant(7, 0.2);
  // one step cannot exceed vmax*dt
  auto sp = ctrl.compute(s, goal);
  check(((sp.q_des - s.q).cwiseAbs().array() <= vmax*dt + 1e-12).all(), "rate cap per step");
  // iterate to convergence
  for (int i=0;i<10000 && (s.q-goal).norm()>1e-6; ++i) { auto c=ctrl.compute(s,goal); s.q=c.q_des; }
  check((s.q-goal).norm() < 1e-6, "rate-limited reaches goal");
}
```

- [ ] **Step 2: Run, verify FAIL.**

- [ ] **Step 3: Implement**

```cpp
// joint_controller.h
#pragma once
#include "joint_types.h"
namespace jmpc {
class JointController {
public:
  virtual ~JointController() = default;
  virtual JointSetpoint compute(const JointState& s, const Eigen::VectorXd& q_goal) = 0;
  virtual int horizon_steps() const { return 1; }
};
}
```

```cpp
// controllers.h
#pragma once
#include "joint_controller.h"
namespace jmpc {
class HoldController : public JointController {
public:
  JointSetpoint compute(const JointState& s, const Eigen::VectorXd&) override {
    return JointSetpoint{ s.q, Eigen::VectorXd::Zero(s.q.size()) };
  }
};
class RateLimitedController : public JointController {
public:
  RateLimitedController(double v_max, double dt) : v_max_(v_max), dt_(dt) {}
  JointSetpoint compute(const JointState& s, const Eigen::VectorXd& goal) override {
    const double maxstep = v_max_ * dt_;
    Eigen::VectorXd q_des = s.q;
    for (int i = 0; i < goal.size(); ++i) {
      double d = goal(i) - s.q(i);
      if (d >  maxstep) d =  maxstep;
      if (d < -maxstep) d = -maxstep;
      q_des(i) = s.q(i) + d;
    }
    return JointSetpoint{ q_des, (q_des - s.q) / dt_ };
  }
private:
  double v_max_, dt_;
};
}
```

- [ ] **Step 4: Run, verify PASS. Commit** — `feat(joint_mpc): controller interface + Hold/RateLimited`

---

## Task 5: SetpointQueue (TDD — replace + hold-last)

**Files:** Create `include/setpoint_queue.h`; extend harness.

- [ ] **Step 1: Failing test**

```cpp
#include "setpoint_queue.h"
{
  jmpc::SetpointQueue q;
  std::deque<jmpc::JointSetpoint> h;
  for (int k=0;k<3;++k) h.push_back({Eigen::VectorXd::Constant(7,k), Eigen::VectorXd::Zero(7)});
  q.replace(h);
  check(q.next()->q_des(0)==0, "pop first");
  check(q.next()->q_des(0)==1, "pop second");
  check(q.next()->q_des(0)==2, "pop third");
  check(q.next()->q_des(0)==2, "hold last when one remains");   // not removed
  // replace swaps
  std::deque<jmpc::JointSetpoint> h2{{Eigen::VectorXd::Constant(7,9), Eigen::VectorXd::Zero(7)}};
  q.replace(h2);
  check(q.next()->q_des(0)==9, "replace swaps horizon");
}
```

- [ ] **Step 2: Run, verify FAIL.**

- [ ] **Step 3: Implement**

```cpp
#pragma once
#include <deque>
#include <mutex>
#include <optional>
#include "joint_types.h"
namespace jmpc {
class SetpointQueue {
public:
  void replace(const std::deque<JointSetpoint>& horizon) {
    std::lock_guard<std::mutex> lk(m_); q_ = horizon;
  }
  std::optional<JointSetpoint> next() {
    std::lock_guard<std::mutex> lk(m_);
    if (q_.empty()) return std::nullopt;
    JointSetpoint front = q_.front();
    if (q_.size() > 1) q_.pop_front();   // hold last when one remains
    return front;
  }
  size_t size() const { std::lock_guard<std::mutex> lk(m_); return q_.size(); }
private:
  mutable std::mutex m_;
  std::deque<JointSetpoint> q_;
};
}
```

- [ ] **Step 4: Run, verify PASS. Commit** — `feat(joint_mpc): thread-safe setpoint queue`

---

## Task 6: ControlDriver + CSV (TDD — end-to-end convergence)

**Files:** Create `include/control_driver.h`; extend harness.

- [ ] **Step 1: Failing test**

```cpp
#include "control_driver.h"
{
  auto L = jmpc::gen3_limits();
  double dt = 0.002;
  auto ctrl = std::make_shared<jmpc::RateLimitedController>(0.5, dt);
  jmpc::SimBackend sim(Eigen::VectorXd::Zero(7), L, 0.05);
  jmpc::ControlDriver drv(ctrl, sim, dt);
  Eigen::VectorXd goal = Eigen::VectorXd::Constant(7, 0.3);
  std::string csv = "run_log.csv";
  drv.run(/*seconds=*/5.0, goal, csv);
  check((drv.backend().state().q - goal).norm() < 1e-2, "driver converges end-to-end");
  std::ifstream f(csv); check(f.good() && f.peek()!=std::ifstream::traits_type::eof(), "csv non-empty");
}
```

- [ ] **Step 2: Run, verify FAIL.**

- [ ] **Step 3: Implement**

```cpp
#pragma once
#include <memory>
#include <fstream>
#include <cmath>
#include <iostream>
#include "controllers.h"
#include "sim_backend.h"
#include "setpoint_queue.h"
namespace jmpc {
class ControlDriver {
public:
  ControlDriver(std::shared_ptr<JointController> ctrl, SimBackend backend, double dt)
    : ctrl_(std::move(ctrl)), backend_(std::move(backend)), dt_(dt) {}

  // One tick: compute -> push horizon -> backend applies next setpoint.
  void step(const Eigen::VectorXd& goal) {
    JointState s = backend_.state();
    JointSetpoint sp = ctrl_->compute(s, goal);
    if (!sp.q_des.allFinite()) { sp = last_good_; }   // hold last on NaN
    else { last_good_ = sp; }
    std::deque<JointSetpoint> h{sp};
    queue_.replace(h);
    auto next = queue_.next();
    if (next) backend_.apply(*next, dt_);
  }

  void run(double seconds, const Eigen::VectorXd& goal, const std::string& csv_path) {
    std::ofstream log(csv_path);
    log << "t";
    for (int i=0;i<goal.size();++i) log << ",q"<<i;
    for (int i=0;i<goal.size();++i) log << ",qdes"<<i;
    for (int i=0;i<goal.size();++i) log << ",goal"<<i;
    log << "\n";
    const int N = static_cast<int>(seconds / dt_);
    last_good_ = JointSetpoint{ backend_.state().q, Eigen::VectorXd::Zero(goal.size()) };
    for (int k=0;k<N;++k) {
      JointState s = backend_.state();
      JointSetpoint sp = ctrl_->compute(s, goal);
      if (!sp.q_des.allFinite()) sp = last_good_; else last_good_ = sp;
      std::deque<JointSetpoint> h{sp}; queue_.replace(h);
      auto next = queue_.next(); if (next) backend_.apply(*next, dt_);
      JointState a = backend_.state();
      log << a.t;
      for (int i=0;i<a.q.size();++i)  log << "," << a.q(i);
      for (int i=0;i<sp.q_des.size();++i) log << "," << sp.q_des(i);
      for (int i=0;i<goal.size();++i) log << "," << goal(i);
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
}
```

- [ ] **Step 4: Run, verify PASS. Commit** — `feat(joint_mpc): control driver + CSV logging`

---

## Task 7: Finalize harness + README + full-suite green

**Files:** finalize `test/test_joint_control.cpp` (`check()` helper + summary),
create `TrajectoryRealTime/joint_mpc/README.md`.

- [ ] **Step 1: `check()` helper + main**

```cpp
static int g_fail = 0;
static void check(bool cond, const char* msg) {
  std::cout << (cond ? "  PASS  " : "  FAIL  ") << msg << "\n";
  if (!cond) ++g_fail;
}
// ... all test blocks above ...
int main() { /* run blocks */ std::cout << (g_fail? "FAILURES\n":"ALL PASS\n"); return g_fail?1:0; }
```

- [ ] **Step 2: Build & run full suite**

Run: `cmake --build build && ./build/test_joint_control`
Expected: every line `PASS`, final `ALL PASS`, exit 0.

- [ ] **Step 3: README** — build/run instructions, mirror `basic_control/README.md` tone.

- [ ] **Step 4: Commit** — `feat(joint_mpc): harness summary + README; M1 green`

---

## Self-Review

- **Spec coverage:** types (T2), Gen3 limits (T2), controller interface + Hold/RateLimited
  (T4), sim backend + clamps (T3), setpoint queue (T5), driver + CSV (T6), harness (T1,T7).
  All six spec tests mapped. ✓
- **Placeholders:** none — every code step is complete. ✓
- **Type consistency:** `JointState{q,dq,t}`, `JointSetpoint{q_des,dq_des}`,
  `JointLimits{q_lower,q_upper,dq_max}`, `gen3_limits()`, `SimBackend::apply/state`,
  `RateLimitedController(v_max,dt)`, `SetpointQueue::replace/next`,
  `ControlDriver(ctrl,backend,dt)/run/backend()` — consistent across tasks. ✓

## Out of Scope (M2+)

QP MPC, `JointTrajectory` adapter, threaded real-time loop, dynamics/torque, voice.
