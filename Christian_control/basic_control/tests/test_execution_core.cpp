//
// test_execution_core — the explicit arm execution contract (Plan 01,
// Task 3).
//
// The primary gate is INDEPENDENT evidence: every case of the frozen
// Task 1 fixture (tests/fixtures/execution_preextract_v1.csv, produced
// before ArmExecutionCore existed) is replayed through the core and the
// regenerated data section must match the stored one byte for byte
// (prediction P2; P8 rides along because the stored rows include the
// commanded frame of every stop cycle). Task 1 already demonstrated the
// fixture fails under order mutations, so byte equality here pins the
// per-cycle order, the relocated world-freshness/stale-decay block, the
// counters and the stop precedence — not merely "similar numbers".
//
// On top of the replay, targeted checks pin the call contract:
//   - Seed is required exactly once (Step before Seed throws; a second
//     Seed throws);
//   - ResolveStop needs a completed Step and is one-shot per Step
//     (send-then-resolve: the adapter transmits the returned command
//     frame, then resolves the stop with the reply's facts);
//   - the request_replan edge is produced BEFORE the non-finite hold
//     (risk R3: both fire on the same cycle when the first fresh world
//     sample overflows the law);
//   - the previous integrated command persists (command evolves from
//     command, never re-seeded from measurement);
//   - injected ExecutionConfig values govern behaviour (a non-production
//     nonfinite_stop_cycles and disable_following_error_stop change the
//     verdicts exactly as predicted — a hidden config:: read would not);
//   - adapter facts keep the documented stop precedence through the
//     core's fact assembly.
//
// Allocation freedom (prediction P5): this file defines a counting
// operator new/delete hook — NEW infrastructure; no allocation-counter
// pattern existed before this task — and requires ZERO allocations and
// deallocations inside every Step and ResolveStop after Seed, across all
// fixture cases including trajectory-activation cycles (activation moves
// an already-allocated trajectory; Retire hands it back to the mailbox).
//
// Labelling per contract section 13: everything here is a UNIT TEST plus
// RECORDED-DATA REPLAY of software behaviour. It proves nothing about
// physical hardware.
//

#undef NDEBUG

#include <atomic>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <new>
#include <sstream>
#include <string>
#include <vector>

#include "ExecutionCore.h"
#include "execution_characterization.h"

// ---------------------------------------------------------------------
// Counting operator new/delete hook. Counts only while enabled, so test
// scaffolding (file I/O, vectors of results) does not pollute the tally.
// Every form forwards to malloc/free so ASan still tracks each block.
// ---------------------------------------------------------------------

namespace {

std::atomic<long> g_hook_allocations{0};
std::atomic<long> g_hook_deallocations{0};
std::atomic<bool> g_hook_counting{false};

void* HookedAllocate(std::size_t size) noexcept
{
    if (g_hook_counting.load(std::memory_order_relaxed))
        g_hook_allocations.fetch_add(1, std::memory_order_relaxed);
    return std::malloc(size != 0 ? size : 1);
}

void* HookedAllocateAligned(std::size_t size, std::size_t alignment) noexcept
{
    if (g_hook_counting.load(std::memory_order_relaxed))
        g_hook_allocations.fetch_add(1, std::memory_order_relaxed);
    if (alignment == 0)
        return nullptr;
    // aligned_alloc requires the size to be a multiple of the alignment.
    const std::size_t rounded =
        (size + alignment - 1) / alignment * alignment;
    return std::aligned_alloc(alignment,
                              rounded != 0 ? rounded : alignment);
}

void HookedFree(void* pointer) noexcept
{
    if (pointer != nullptr &&
        g_hook_counting.load(std::memory_order_relaxed))
        g_hook_deallocations.fetch_add(1, std::memory_order_relaxed);
    std::free(pointer);
}

} // namespace

void* operator new(std::size_t size)
{
    if (void* pointer = HookedAllocate(size))
        return pointer;
    throw std::bad_alloc();
}
void* operator new[](std::size_t size)
{
    if (void* pointer = HookedAllocate(size))
        return pointer;
    throw std::bad_alloc();
}
void* operator new(std::size_t size, const std::nothrow_t&) noexcept
{
    return HookedAllocate(size);
}
void* operator new[](std::size_t size, const std::nothrow_t&) noexcept
{
    return HookedAllocate(size);
}
void* operator new(std::size_t size, std::align_val_t alignment)
{
    if (void* pointer =
            HookedAllocateAligned(size, static_cast<std::size_t>(alignment)))
        return pointer;
    throw std::bad_alloc();
}
void* operator new[](std::size_t size, std::align_val_t alignment)
{
    if (void* pointer =
            HookedAllocateAligned(size, static_cast<std::size_t>(alignment)))
        return pointer;
    throw std::bad_alloc();
}
void* operator new(std::size_t size, std::align_val_t alignment,
                   const std::nothrow_t&) noexcept
{
    return HookedAllocateAligned(size, static_cast<std::size_t>(alignment));
}
void* operator new[](std::size_t size, std::align_val_t alignment,
                     const std::nothrow_t&) noexcept
{
    return HookedAllocateAligned(size, static_cast<std::size_t>(alignment));
}
void operator delete(void* pointer) noexcept { HookedFree(pointer); }
void operator delete[](void* pointer) noexcept { HookedFree(pointer); }
void operator delete(void* pointer, std::size_t) noexcept
{
    HookedFree(pointer);
}
void operator delete[](void* pointer, std::size_t) noexcept
{
    HookedFree(pointer);
}
void operator delete(void* pointer, std::align_val_t) noexcept
{
    HookedFree(pointer);
}
void operator delete[](void* pointer, std::align_val_t) noexcept
{
    HookedFree(pointer);
}
void operator delete(void* pointer, std::size_t, std::align_val_t) noexcept
{
    HookedFree(pointer);
}
void operator delete[](void* pointer, std::size_t, std::align_val_t) noexcept
{
    HookedFree(pointer);
}
void operator delete(void* pointer, const std::nothrow_t&) noexcept
{
    HookedFree(pointer);
}
void operator delete[](void* pointer, const std::nothrow_t&) noexcept
{
    HookedFree(pointer);
}

namespace {

using namespace characterization;

// RAII window during which the hook counts. Deltas are read before the
// destructor disables counting.
class AllocationCountScope {
public:
    AllocationCountScope()
        : allocations_before_(
              g_hook_allocations.load(std::memory_order_relaxed)),
          deallocations_before_(
              g_hook_deallocations.load(std::memory_order_relaxed))
    {
        g_hook_counting.store(true, std::memory_order_relaxed);
    }
    ~AllocationCountScope()
    {
        g_hook_counting.store(false, std::memory_order_relaxed);
    }
    long allocations() const
    {
        return g_hook_allocations.load(std::memory_order_relaxed) -
               allocations_before_;
    }
    long deallocations() const
    {
        return g_hook_deallocations.load(std::memory_order_relaxed) -
               deallocations_before_;
    }

private:
    long allocations_before_;
    long deallocations_before_;
};

// Sanity check of the hook itself: an obviously allocating operation must
// be counted, or a passing "zero allocations" claim would be vacuous.
void CheckAllocationHookCounts()
{
    long allocations = 0;
    long deallocations = 0;
    {
        AllocationCountScope scope;
        int* probe = new int(7);
        delete probe;
        allocations = scope.allocations();
        deallocations = scope.deallocations();
    }
    assert(allocations >= 1 && "the hook must count operator new");
    assert(deallocations >= 1 && "the hook must count operator delete");
}

// ---------------------------------------------------------------------
// Shared scaffolding
// ---------------------------------------------------------------------

// One production-configured core with everything it composes. A fresh
// fixture per case mirrors one run of the controller (the Task 1 harness
// did the same).
struct CoreFixture {
    Dynamics dynamics;
    DualArmKinematics model;
    CartesianTrajectoryMailbox mailbox;
    ArmExecutionCore core;

    explicit CoreFixture(const ExecutionConfig& execution_config)
        : dynamics(GEN3_DUAL_URDF_PATH),
          model(dynamics, Arm::kRight, config::kLeftNominalRad,
                config::kRightBaseFrame, config::kRightEndEffectorFrame),
          core(model, execution_config, mailbox, config::kControlDtS)
    {
    }
};

WorldSample ToWorldSample(const WorldSampleInput& in)
{
    WorldSample world;
    world.mount_valid = in.mount_valid;
    world.mount_position_m = in.mount_position_m;
    for (int i = 0; i < 4; ++i)
        world.mount_quat_xyzw[i] = in.mount_quat_xyzw[i];
    world.sequence = in.sequence;
    world.age_s = in.age_s;
    world.mount_twist_valid = in.mount_twist_valid;
    world.mount_linear_world_m_s = in.mount_linear_world_m_s;
    world.mount_angular_world_rad_s = in.mount_angular_world_rad_s;
    return world;
}

WorldSample FreshWorldSample(std::uint64_t sequence,
                             const Eigen::Vector3d& position_m,
                             double age_s = 0.008)
{
    WorldSample world;
    world.mount_valid = true;
    world.mount_position_m = position_m;
    const Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
    world.mount_quat_xyzw[0] = quat.x();
    world.mount_quat_xyzw[1] = quat.y();
    world.mount_quat_xyzw[2] = quat.z();
    world.mount_quat_xyzw[3] = quat.w();
    world.sequence = sequence;
    world.age_s = age_s;
    return world;
}

ArmExecutionInput MakeInput(double t_s, const JointVector& measured_deg,
                            const WorldSample& world,
                            double dt_s = config::kControlDtS)
{
    ArmExecutionInput input;
    input.dt_s = dt_s;
    input.t_s = t_s;
    input.measured_position_deg = measured_deg;
    input.world = world;
    return input;
}

// The adapter-side (non-real-time on hardware) trajectory publication the
// replay performs BETWEEN core calls, exactly as the FIFO input thread
// would.
void PublishTrajectory(CartesianTrajectoryMailbox& mailbox,
                       const TrajectoryInput& trajectory_input)
{
    auto trajectory = std::make_unique<WorldCartesianTrajectory>();
    trajectory->trajectory_id = trajectory_input.trajectory_id;
    trajectory->planner_vicon_sequence =
        trajectory_input.planner_vicon_sequence;
    trajectory->points = {trajectory_input.points[0],
                          trajectory_input.points[1]};
    mailbox.Publish(std::move(trajectory));
}

// Rebuild a CharacterizationExpected record from the core's outputs so it
// serializes through the SAME RowIO schema the Task 1 fixture used —
// byte-for-byte comparability with the frozen evidence.
CharacterizationExpected ToExpected(const ArmExecutionResult& result,
                                    const ExecutionStopDecision& stop)
{
    CharacterizationExpected expected;
    expected.world_fresh = result.state.world_fresh;
    expected.world_mount_twist_valid = result.state.world_mount_twist_valid;
    expected.world_p = result.state.world_p_mountseg;
    expected.world_R = result.state.world_R_mountseg;
    expected.world_v = result.state.world_v_mountseg_m_s;
    expected.world_w = result.state.world_w_mountseg_rad_s;
    expected.meas_p = result.measured.ee_pose_world.position_m;
    expected.meas_R = result.measured.ee_pose_world.rotation;
    expected.meas_v = result.measured.ee_twist_world.linear_m_s;
    expected.meas_w = result.measured.ee_twist_world.angular_rad_s;
    expected.sigma_min = result.controller_status.sigma_min;
    expected.rot_error_rad = result.controller_status.rot_error_rad;
    expected.null_leak_m_s = result.controller_status.null_leak_m_s;
    expected.qdot_task_rad_s = result.controller_status.qdot_task_rad_s;
    expected.qdot_null_rad_s = result.controller_status.qdot_null_rad_s;
    expected.ref_p = result.reference.ee_pose_world.position_m;
    expected.ref_R = result.reference.ee_pose_world.rotation;
    expected.ref_v = result.reference.ee_twist_world.linear_m_s;
    expected.ref_w = result.reference.ee_twist_world.angular_rad_s;
    expected.ref_trajectory_id = result.reference.trajectory_id;
    expected.ref_planner_vicon_sequence =
        result.reference.planner_vicon_sequence;
    expected.ref_time_s = result.reference.t_from_start_s;
    expected.ref_arrival_eligible = result.reference.arrival_eligible;
    expected.arrived_edge = result.controller_status.arrived_edge;
    expected.not_reached_edge = result.controller_status.not_reached_edge;
    expected.arrival_error_m = result.controller_status.arrival_error_m;
    expected.traj_activated =
        result.controller_status.cartesian_traj_activated;
    expected.traj_rejected =
        result.controller_status.cartesian_traj_rejected;
    expected.traj_complete_edge =
        result.controller_status.cartesian_traj_complete_edge;
    expected.traj_cancelled_edge =
        result.controller_status.cartesian_traj_cancelled_edge;
    expected.request_replan_edge =
        result.controller_status.request_replan_edge;
    expected.start_position_error_m =
        result.controller_status.cartesian_traj_start_position_error_m;
    expected.start_orientation_error_rad =
        result.controller_status.cartesian_traj_start_orientation_error_rad;
    for (int i = 0; i < kNumJoints; ++i)
        expected.qdot_raw_rad_s[i] = result.qdot_raw_rad_s[i];
    expected.nonfinite = result.nonfinite;
    expected.nonfinite_count = result.nonfinite_count;
    expected.overrun = result.overrun;
    expected.overrun_count = result.overrun_count;
    expected.qdot_limited_deg_s = result.qdot_limited_deg_s;
    expected.saturated = result.saturated;
    expected.requested_position_deg = result.actuation_status.requested_deg;
    expected.lead_limited = result.actuation_status.lead_limited;
    expected.joint_limit_warning_joint =
        result.actuation_status.joint_limit_warning_joint.value_or(-1);
    expected.commanded_deg = result.commanded_deg;
    expected.commanded_velocity_deg_s = result.commanded_velocity_deg_s;
    if (stop.priority.reason != StopPriorityReason::kNone)
        expected.stop = static_cast<StopKind>(
            static_cast<int>(stop.priority.reason));
    else if (stop.nonfinite_stop)
        expected.stop = StopKind::kNonFiniteCommand;
    else if (stop.overrun_stop)
        expected.stop = StopKind::kOverrun;
    else
        expected.stop = StopKind::kNone;
    expected.live_fault_observed = stop.priority.live_fault_observed;
    return expected;
}

// Prints the first differing line so a replay failure names the cycle.
// (Local copy of the Task 1 helper; Task 1 files stay untouched because
// the frozen fixture's provenance hashes describe them.)
void RequireEqualText(const std::string& actual, const std::string& stored,
                      const char* what)
{
    if (actual == stored)
        return;
    std::istringstream a(actual), s(stored);
    std::string line_a, line_s;
    int line_number = 0;
    while (true) {
        const bool got_a = static_cast<bool>(std::getline(a, line_a));
        const bool got_s = static_cast<bool>(std::getline(s, line_s));
        ++line_number;
        if (!got_a && !got_s)
            break;
        if (line_a != line_s || got_a != got_s) {
            std::cerr << what << ": first difference at line " << line_number
                      << "\n  stored:   " << (got_s ? line_s : "<missing>")
                      << "\n  computed: " << (got_a ? line_a : "<missing>")
                      << "\n";
            break;
        }
    }
    assert(false && "execution core text mismatch — see stderr");
}

// ---------------------------------------------------------------------
// The replay gate (predictions P2, P5, P8)
// ---------------------------------------------------------------------

// Drives one frozen case through the core exactly as the hardware runner
// would: the adapter publishes plans and hands over the raw world sample;
// the core Steps; the adapter "transmits" the returned command frame and
// only then resolves the stop with the reply's facts (send-then-resolve).
// The reply of this cycle's absent exchange is the fixture input, and
// becomes the NEXT cycle's measurement (the pipeline's one-cycle delay).
std::vector<CharacterizationExpected> ReplayCase(
    const CharacterizationCase& one_case)
{
    CoreFixture fixture(ProductionExecutionConfig());
    fixture.core.Seed(one_case.seed_deg, one_case.seed_velocity_deg_s);

    JointVector previous_feedback_deg = one_case.seed_deg;
    JointVector previous_feedback_velocity_deg_s =
        one_case.seed_velocity_deg_s;

    std::vector<CharacterizationExpected> replayed;
    replayed.reserve(one_case.rows.size());
    for (std::size_t r = 0; r < one_case.rows.size(); ++r) {
        const CharacterizationCycle& row = one_case.rows[r];
        if (row.trajectory.publish)
            PublishTrajectory(fixture.mailbox, row.trajectory);

        ArmExecutionInput input;
        input.dt_s = row.dt_s;
        input.t_s = row.t_s;
        input.measured_position_deg = previous_feedback_deg;
        input.measured_velocity_deg_s = previous_feedback_velocity_deg_s;
        input.world = ToWorldSample(row.world);

        AdapterHealth health;
        health.live_fault = row.adapter.live_fault;
        health.low_level_state_lost = row.adapter.left_low_level;
        health.stale_feedback = row.adapter.stale_feedback;

        ArmExecutionResult result;
        ExecutionStopDecision stop;
        long allocations = 0;
        long deallocations = 0;
        {
            // Prediction P5: neither call allocates or frees after Seed.
            AllocationCountScope scope;
            result = fixture.core.Step(input);
            stop = fixture.core.ResolveStop(row.feedback_deg, health);
            allocations = scope.allocations();
            deallocations = scope.deallocations();
        }
        if (allocations != 0 || deallocations != 0) {
            std::fprintf(stderr,
                         "allocation in case %s row %zu: %ld allocations, "
                         "%ld deallocations\n",
                         one_case.name.c_str(), r, allocations,
                         deallocations);
            assert(false && "Step/ResolveStop must not allocate (P5)");
        }

        replayed.push_back(ToExpected(result, stop));

        previous_feedback_deg = row.feedback_deg;
        previous_feedback_velocity_deg_s = row.feedback_velocity_deg_s;
        // The non-real-time reclamation the input thread performs on
        // hardware — never inside the cycle.
        fixture.mailbox.DeleteRetired();
    }
    return replayed;
}

// ---------------------------------------------------------------------
// Call-contract checks
// ---------------------------------------------------------------------

// A non-singular seed pose, raw Kortex degrees [0, 360) (same values the
// fixture cases use, kept local so this file does not depend on Task 1
// case builders).
JointVector TestSeedDeg()
{
    const double rad[7] = {0.3, -0.5, 0.4, 1.0, -0.2, 0.8, 0.1};
    JointVector deg{};
    for (int i = 0; i < 7; ++i) {
        deg[i] = rad[i] * kRadToDeg;
        if (deg[i] < 0.0)
            deg[i] += 360.0;
    }
    return deg;
}

const Eigen::Vector3d kTestMountP(0.5, -0.2, 0.8);

void CheckSeedAndCallOrderContract()
{
    CoreFixture fixture(ProductionExecutionConfig());
    const JointVector seed = TestSeedDeg();

    bool threw = false;
    try {
        fixture.core.Step(MakeInput(0.0, seed, FreshWorldSample(1,
                                                               kTestMountP)));
    } catch (const std::logic_error&) {
        threw = true;
    }
    assert(threw && "Step before Seed must throw");

    threw = false;
    try {
        fixture.core.ResolveStop(seed, AdapterHealth{});
    } catch (const std::logic_error&) {
        threw = true;
    }
    assert(threw && "ResolveStop before any Step must throw");

    fixture.core.Seed(seed, JointVector{});
    threw = false;
    try {
        fixture.core.Seed(seed, JointVector{});
    } catch (const std::logic_error&) {
        threw = true;
    }
    assert(threw && "a second Seed must throw (seeded exactly once)");

    const ArmExecutionResult result = fixture.core.Step(
        MakeInput(0.0, seed, FreshWorldSample(1, kTestMountP)));
    (void)result;
    fixture.core.ResolveStop(seed, AdapterHealth{});
    threw = false;
    try {
        fixture.core.ResolveStop(seed, AdapterHealth{});
    } catch (const std::logic_error&) {
        threw = true;
    }
    assert(threw &&
           "ResolveStop is one-shot per completed Step (send-then-resolve)");
}

// Risk R3: the replan EDGE is produced by the reference state machine
// BEFORE the non-finite hold — both must appear on the same cycle. A
// position overflow cannot coexist with the edge (every edge cycle
// re-anchors the hold to that cycle's measured pose, so e_pos is exactly
// zero — CartesianReference.cpp:190-201); the coexistence path is the Kd
// term: a finite but astronomically large MEASURED Mount twist on the
// first fresh sample overflows the law while the pose error is zero.
// The magnitude was found empirically and frozen, mirroring the Task 1
// fixture's overflow construction; the assertions below fail loudly if
// it ever stops overflowing.
void CheckReplanEdgeSurvivesNonfiniteHold()
{
    CoreFixture fixture(ProductionExecutionConfig());
    const JointVector seed = TestSeedDeg();
    fixture.core.Seed(seed, JointVector{});

    WorldSample world = FreshWorldSample(1, kTestMountP);
    world.mount_twist_valid = true;
    world.mount_linear_world_m_s =
        Eigen::Vector3d(1.7e308, 1.7e308, 1.7e308);
    world.mount_angular_world_rad_s =
        Eigen::Vector3d(1.7e308, 1.7e308, 1.7e308);
    const ArmExecutionResult result =
        fixture.core.Step(MakeInput(0.0, seed, world));
    assert(result.controller_status.request_replan_edge &&
           "the first fresh sample must fire the replan edge");
    assert(result.nonfinite &&
           "the overflowed law output must be recorded as non-finite");
    for (int i = 0; i < kNumJoints; ++i) {
        assert(result.commanded_velocity_deg_s[i] == 0.0 &&
               "a non-finite cycle holds: zero setpoint velocity");
    }
    fixture.core.ResolveStop(seed, AdapterHealth{});
}

// Injected configuration governs behaviour: with nonfinite_stop_cycles=1
// ONE non-finite cycle already stops (production needs 3), and with
// disable_following_error_stop=true a grossly wrong reply no longer
// stops. A hidden config:: read inside the core would fail both.
void CheckCustomConfigInjection()
{
    ExecutionConfig custom = ProductionExecutionConfig();
    assert(custom.nonfinite_stop_cycles == 3 &&
           "production baseline expected (fixture nonfinite case relies "
           "on 3)");
    assert(!custom.disable_following_error_stop);
    custom.nonfinite_stop_cycles = 1;
    custom.disable_following_error_stop = true;
    ValidateExecutionConfig(custom);

    CoreFixture fixture(custom);
    const JointVector seed = TestSeedDeg();
    fixture.core.Seed(seed, JointVector{});

    // Anchor the hold at a normal pose first; the jump to 1e308 then
    // overflows Kp * e_pos exactly as the fixture's nonfinite cases do
    // (with these same production gains).
    fixture.core.Step(MakeInput(0.0, seed, FreshWorldSample(1, kTestMountP)));
    fixture.core.ResolveStop(seed, AdapterHealth{});
    const Eigen::Vector3d overflow_position(1.0e308, 0.0, 0.0);
    const ArmExecutionResult result = fixture.core.Step(
        MakeInput(0.002, seed, FreshWorldSample(2, overflow_position)));
    assert(result.nonfinite && result.nonfinite_count == 1);

    // The reply is 10 deg off on joint 1 — far beyond the production
    // 3 deg following-error limit — but the injected config disables that
    // stop, so only the injected 1-cycle non-finite stop may fire.
    JointVector reply = result.commanded_deg;
    reply[0] += 10.0;
    const ExecutionStopDecision stop =
        fixture.core.ResolveStop(reply, AdapterHealth{});
    assert(!stop.following_error &&
           "disable_following_error_stop must suppress the fact");
    assert(stop.priority.reason == StopPriorityReason::kNone);
    assert(stop.nonfinite_stop &&
           "injected nonfinite_stop_cycles=1 must stop on the first "
           "non-finite cycle");
}

// The persistent command evolves from the PREVIOUS command, never from
// the measurement: with the measurement pinned at the seed pose and a
// displaced world hold pulling hard, cycle 2's unconstrained proposal
// continues from cycle 1's integrated command.
void CheckIntegratedCommandPersists()
{
    CoreFixture fixture(ProductionExecutionConfig());
    const JointVector seed = TestSeedDeg();
    fixture.core.Seed(seed, JointVector{});

    // Anchor the hold at the first fresh pose, then displace the Mount:
    // the world hold stays fixed, the measured pose moves, the error is
    // large and the command must move cycle after cycle.
    const ArmExecutionResult anchor = fixture.core.Step(
        MakeInput(0.0, seed, FreshWorldSample(1, kTestMountP)));
    (void)anchor;
    fixture.core.ResolveStop(seed, AdapterHealth{});

    const Eigen::Vector3d moved = kTestMountP + Eigen::Vector3d(0.3, 0, 0);
    const ArmExecutionResult first = fixture.core.Step(
        MakeInput(0.002, seed, FreshWorldSample(2, moved)));
    fixture.core.ResolveStop(seed, AdapterHealth{});
    const ArmExecutionResult second = fixture.core.Step(
        MakeInput(0.004, seed, FreshWorldSample(3, moved)));
    fixture.core.ResolveStop(seed, AdapterHealth{});

    double moved_deg = 0.0;
    for (int i = 0; i < kNumJoints; ++i) {
        moved_deg = std::max(
            moved_deg, std::abs(first.commanded_deg[i] - seed[i]));
        // requested = previous command + qdot_limited * dt, where
        // "previous" is cycle 1's INTEGRATED command. A re-seed from the
        // (pinned) measurement would instead continue from the seed pose,
        // one full first-cycle step behind.
        const double continued = first.commanded_deg[i] +
                                 second.qdot_limited_deg_s[i] *
                                     config::kControlDtS;
        assert(std::abs(second.actuation_status.requested_deg[i] -
                        continued) < 1e-9 &&
               "the integrated command must persist between cycles");
    }
    assert(moved_deg > 1e-3 &&
           "the displaced hold must actually move the command, or the "
           "persistence check above cannot discriminate");
}

// Adapter facts keep the documented precedence through the core's fact
// assembly (StopPriority.h ladder; the facts come from the adapter, the
// following-error fact from the reply, the joint-boundary fact from the
// core's own actuation).
void CheckAdapterFactPrecedence()
{
    CoreFixture fixture(ProductionExecutionConfig());
    const JointVector seed = TestSeedDeg();
    fixture.core.Seed(seed, JointVector{});

    const auto step = [&](std::uint64_t sequence) {
        return fixture.core.Step(
            MakeInput(0.002 * static_cast<double>(sequence), seed,
                      FreshWorldSample(sequence, kTestMountP)));
    };

    step(1);
    AdapterHealth both;
    both.live_fault = true;
    both.low_level_state_lost = true;
    const ExecutionStopDecision left =
        fixture.core.ResolveStop(seed, both);
    assert(left.priority.reason == StopPriorityReason::kLeftLowLevel &&
           "leaving low-level servoing outranks the fault stop");
    assert(left.priority.live_fault_observed);

    const ArmExecutionResult second = step(2);
    JointVector bad_reply = second.commanded_deg;
    bad_reply[0] += 5.0; // > 3 deg production following-error limit
    AdapterHealth fault_only;
    fault_only.live_fault = true;
    const ExecutionStopDecision following =
        fixture.core.ResolveStop(bad_reply, fault_only);
    assert(following.following_error);
    assert(following.priority.reason ==
               StopPriorityReason::kFollowingError &&
           "following error outranks a live fault");
    assert(following.priority.live_fault_observed);

    step(3);
    AdapterHealth stale_only;
    stale_only.stale_feedback = true;
    const ExecutionStopDecision stale =
        fixture.core.ResolveStop(seed, stale_only);
    assert(stale.priority.reason == StopPriorityReason::kStaleFeedback);
    assert(!stale.priority.live_fault_observed);
}

} // namespace

int main()
{
    CheckAllocationHookCounts();

    // The frozen Task 1 fixture is REQUIRED evidence: it predates the
    // core, so matching it is validation the extraction did not create.
    std::ifstream fixture_stream(EXECUTION_FIXTURE_PATH);
    assert(fixture_stream &&
           "missing frozen fixture tests/fixtures/execution_preextract_v1"
           ".csv — run execution_characterization first");
    ParsedFixture fixture = ParseFixture(fixture_stream);
    assert(fixture.cases.size() == 16 &&
           "the frozen fixture must contain all 16 Task 1 cases");

    std::vector<std::vector<CharacterizationExpected>> replayed;
    replayed.reserve(fixture.cases.size());
    std::size_t total_rows = 0;
    for (const CharacterizationCase& one_case : fixture.cases) {
        replayed.push_back(ReplayCase(one_case));
        total_rows += one_case.rows.size();
    }
    const std::string replay_text =
        SerializeDataSection(fixture.cases, replayed);
    RequireEqualText(replay_text, fixture.data_section,
                     "core fixture replay (P2/P8)");
    std::cout << "core replayed " << fixture.cases.size() << " cases, "
              << total_rows
              << " rows byte-identically with zero allocations\n";

    CheckSeedAndCallOrderContract();
    CheckReplanEdgeSurvivesNonfiniteHold();
    CheckCustomConfigInjection();
    CheckIntegratedCommandPersists();
    CheckAdapterFactPrecedence();

    std::cout << "all execution core checks passed\n";
    return 0;
}
