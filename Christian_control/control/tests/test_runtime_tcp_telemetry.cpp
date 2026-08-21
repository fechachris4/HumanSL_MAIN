#undef NDEBUG

#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <new>
#include <vector>

#include "ExecutionCore.h"
#include "ExecutionConfig.h"
#include "Kinematics.h"
#include "RobotModel.h"

namespace {
std::atomic<bool> g_count_allocations{false};
std::atomic<long> g_allocations{0};
}

void* operator new(std::size_t size)
{
    if (g_count_allocations.load(std::memory_order_relaxed))
        g_allocations.fetch_add(1, std::memory_order_relaxed);
    if (void* pointer = std::malloc(size != 0 ? size : 1)) return pointer;
    throw std::bad_alloc();
}

void* operator new[](std::size_t size) { return ::operator new(size); }
void operator delete(void* pointer) noexcept { std::free(pointer); }
void operator delete[](void* pointer) noexcept { std::free(pointer); }
void operator delete(void* pointer, std::size_t) noexcept { std::free(pointer); }
void operator delete[](void* pointer, std::size_t) noexcept { std::free(pointer); }

namespace {

constexpr double kDegToRad = M_PI / 180.0;

void CheckNear(double actual, double expected, double tolerance)
{
    assert(std::abs(actual - expected) <= tolerance);
}

ArmExecutionInput InputAt(const JointVector& measured_deg, double t_s)
{
    ArmExecutionInput input;
    input.dt_s = config::kControlDtS;
    input.t_s = t_s;
    input.measured_position_deg = measured_deg;
    input.world.mount_valid = true;
    input.world.mount_position_m.setZero();
    input.world.mount_quat_xyzw[0] = 0.0;
    input.world.mount_quat_xyzw[1] = 0.0;
    input.world.mount_quat_xyzw[2] = 0.0;
    input.world.mount_quat_xyzw[3] = 1.0;
    input.world.sequence = 1;
    input.world.age_s = 0.0;
    return input;
}

} // namespace

int main()
{
    RobotModel robot_model(GEN3_DUAL_URDF_PATH);
    DualArmKinematics model(robot_model, Arm::kRight,
                            config::kLeftNominalRad,
                            config::kRightBaseFrame,
                            config::kRightEndEffectorFrame);
    CartesianTrajectoryMailbox mailbox;
    ArmExecutionCore core(model, ProductionExecutionConfig(), mailbox,
                          config::kControlDtS);

    JointVector command_seed_deg{10, 20, 30, 40, 50, 60, 70};
    JointVector zero_deg{};
    core.Seed(command_seed_deg, zero_deg);

    ArmExecutionResult first = core.Step(InputAt(zero_deg, 0.0));
    core.ResolveStop(first.commanded_deg, AdapterHealth{});

    // Measured pose reuses the controller's existing FK result at q = 0.
    CheckNear(first.measured.ee_pose_mount.position_m.x(), 0.0, 2e-6);
    CheckNear(first.measured.ee_pose_mount.position_m.y(), -1.268828, 2e-6);
    CheckNear(first.measured.ee_pose_mount.position_m.z(), 0.440120, 2e-6);
    Eigen::Quaterniond measured_q(first.measured.ee_pose_mount.rotation);
    if (measured_q.w() < 0.0) measured_q.coeffs() = -measured_q.coeffs();
    CheckNear(measured_q.x(), 0.568148, 3e-6);
    CheckNear(measured_q.y(), 0.0, 3e-6);
    CheckNear(measured_q.z(), 0.0, 3e-6);
    CheckNear(measured_q.w(), 0.822926, 3e-6);

    // The transmitted command remains the seed on the first zero-error cycle,
    // so the post-send telemetry query must match the frozen pre-change FK
    // fixture for [10,20,30,40,50,60,70] degrees.
    const CartesianPose commanded = core.CommandedTcpMount();
    CheckNear(commanded.position_m.x(), 0.396290, 2e-6);
    CheckNear(commanded.position_m.y(), -0.976259, 2e-6);
    CheckNear(commanded.position_m.z(), -0.149543, 2e-6);
    Eigen::Quaterniond q(commanded.rotation);
    if (q.w() < 0.0) q.coeffs() = -q.coeffs();
    CheckNear(q.x(), -0.258031, 3e-6);
    CheckNear(q.y(), 0.953876, 3e-6);
    CheckNear(q.z(), -0.150327, 3e-6);
    CheckNear(q.w(), 0.030710, 3e-6);

    // The measured path must reproduce the same frozen nonzero fixture, not
    // merely the zero-joint special case.
    CartesianTrajectoryMailbox equal_mailbox;
    ArmExecutionCore equal_core(model, ProductionExecutionConfig(),
                                equal_mailbox, config::kControlDtS);
    equal_core.Seed(command_seed_deg, zero_deg);
    ArmExecutionResult equal =
        equal_core.Step(InputAt(command_seed_deg, 0.0));
    equal_core.ResolveStop(equal.commanded_deg, AdapterHealth{});
    CheckNear(equal.measured.ee_pose_mount.position_m.x(), 0.396290, 2e-6);
    CheckNear(equal.measured.ee_pose_mount.position_m.y(), -0.976259, 2e-6);
    CheckNear(equal.measured.ee_pose_mount.position_m.z(), -0.149543, 2e-6);
    Eigen::Quaterniond equal_measured_q(equal.measured.ee_pose_mount.rotation);
    if (equal_measured_q.w() < 0.0)
        equal_measured_q.coeffs() = -equal_measured_q.coeffs();
    CheckNear(equal_measured_q.x(), -0.258031, 3e-6);
    CheckNear(equal_measured_q.y(), 0.953876, 3e-6);
    CheckNear(equal_measured_q.z(), -0.150327, 3e-6);
    CheckNear(equal_measured_q.w(), 0.030710, 3e-6);
    const CartesianPose equal_command = equal_core.CommandedTcpMount();
    CheckNear((equal.measured.ee_pose_mount.position_m -
               equal_command.position_m).norm(), 0.0, 1e-12);

    // Mirrored left-arm mounting must remain distinct from the right branch.
    RobotModel left_robot_model(GEN3_DUAL_URDF_PATH);
    DualArmKinematics left_model(
        left_robot_model, Arm::kLeft, config::kRightNominalRad,
        config::kRightBaseFrame, config::kRightEndEffectorFrame);
    CartesianTrajectoryMailbox left_mailbox;
    ArmExecutionCore left_core(left_model, ProductionExecutionConfig(),
                               left_mailbox, config::kControlDtS);
    left_core.Seed(zero_deg, zero_deg);
    ArmExecutionResult left = left_core.Step(InputAt(zero_deg, 0.0));
    left_core.ResolveStop(left.commanded_deg, AdapterHealth{});
    CheckNear(left.measured.ee_pose_mount.position_m.x(), 0.0, 2e-6);
    CheckNear(left.measured.ee_pose_mount.position_m.y(), 1.138995, 2e-6);
    CheckNear(left.measured.ee_pose_mount.position_m.z(), 0.444082, 2e-6);
    Eigen::Quaterniond left_q(left.measured.ee_pose_mount.rotation);
    if (left_q.w() < 0.0) left_q.coeffs() = -left_q.coeffs();
    CheckNear(left_q.x(), -0.568142, 3e-6);
    CheckNear(left_q.w(), 0.822930, 3e-6);

    // Same production tail as Runner's no-stop path. The pre-change p99 was
    // 2.315 us on this workstation; accepted gate is total < 1 ms and delta
    // < 100 us. Warmup absorbs model/cache startup.
    constexpr int warmup = 1000;
    constexpr int count = 10000;
    std::vector<double> elapsed_us;
    elapsed_us.reserve(count);
    g_allocations.store(0, std::memory_order_relaxed);
    g_count_allocations.store(true, std::memory_order_relaxed);
    for (int i = 0; i < warmup + count; ++i) {
        const auto start = std::chrono::steady_clock::now();
        ArmExecutionResult result =
            core.Step(InputAt(zero_deg, (i + 1) * config::kControlDtS));
        const ExecutionStopDecision stop =
            core.ResolveStop(result.commanded_deg, AdapterHealth{});
        assert(stop.priority.reason == StopPriorityReason::kNone);
        assert(!stop.nonfinite_stop && !stop.overrun_stop);
        (void)core.CommandedTcpMount();
        const auto finish = std::chrono::steady_clock::now();
        if (i >= warmup)
            elapsed_us.push_back(
                std::chrono::duration<double, std::micro>(finish - start).count());
    }
    g_count_allocations.store(false, std::memory_order_relaxed);
    std::sort(elapsed_us.begin(), elapsed_us.end());
    const double p99 = elapsed_us[static_cast<std::size_t>(count * 99 / 100)];
    std::printf("runtime TCP telemetry p99 %.3f us\n", p99);
    assert(p99 < 1000.0);
    assert(p99 - 2.315 < 100.0);
    assert(g_allocations.load(std::memory_order_relaxed) == 0);
    return 0;
}
