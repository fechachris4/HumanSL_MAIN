//
// test_execution_characterization — freezes the pre-extraction behaviour
// of the per-cycle command pipeline (Plan 01, Task 1).
//
// First run (no fixture): runs every scenario case through the REAL
// production units in the exact Runner.cpp order, checks the behavioural
// sanity claims below, proves the run is deterministic (prediction P1),
// and writes tests/fixtures/execution_preextract_v1.csv with a
// provenance header (git revision + SHA-256 of every source input, so an
// uncommitted working tree is identified exactly).
//
// Later runs (fixture exists): re-runs the pipeline on the fixture's OWN
// recorded inputs and requires the regenerated data section to match the
// stored one byte for byte. After Tasks 2-4 this is the replay gate that
// proves the extraction changed nothing (prediction P2).
//
// Scenario coverage (plan Task 1 Step 4 + limiting cases 1-9 + risk R1):
// fresh startup hold, active Cartesian reference, nonzero measured Mount
// twist, velocity saturation, bounded-joint outward proposal, non-finite
// controller output (stop and recovery), final hold with arrival, brief
// stale pause with twist decay (fresh->stale->fresh->stale), prolonged
// stale cancellation, recovery replan edge with provenance rejection,
// fresh-invalid-twist memory clearing, adapter stop precedence, and the
// overrun counter stop.
//
// Labelling per contract section 13: everything here is a UNIT TEST of
// software behaviour. It proves nothing about physical hardware.
//

#undef NDEBUG

#include <cassert>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "execution_characterization.h"

using namespace characterization;

namespace {

// ---------------------------------------------------------------------
// Shared physical inputs
// ---------------------------------------------------------------------

// A non-singular arm pose, in raw Kortex degrees [0, 360).
JointVector SeedDeg()
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

const Eigen::Vector3d kMountP(0.5, -0.2, 0.8);
const Eigen::Quaterniond kMountQ(
    Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));

WorldSampleInput FreshWorld(std::uint64_t sequence,
                            const Eigen::Vector3d& position = kMountP,
                            const Eigen::Quaterniond& quat = kMountQ,
                            double age_s = 0.008)
{
    WorldSampleInput world;
    world.mount_valid = true;
    world.mount_position_m = position;
    world.mount_quat_xyzw[0] = quat.x();
    world.mount_quat_xyzw[1] = quat.y();
    world.mount_quat_xyzw[2] = quat.z();
    world.mount_quat_xyzw[3] = quat.w();
    world.sequence = sequence;
    world.age_s = age_s;
    return world;
}

WorldSampleInput WithTwist(WorldSampleInput world,
                           const Eigen::Vector3d& v,
                           const Eigen::Vector3d& w)
{
    world.mount_twist_valid = true;
    world.mount_linear_world_m_s = v;
    world.mount_angular_world_rad_s = w;
    return world;
}

// Occluded Mount: Vicon still delivers frames (fresh age, advancing
// sequence) but the Mount segment is invalid, so its pose fields are NaN
// — occlusion is not a pose (BasePose.h).
WorldSampleInput OccludedWorld(std::uint64_t sequence, double age_s = 0.008)
{
    WorldSampleInput world;
    world.sequence = sequence;
    world.age_s = age_s;
    return world;
}

CharacterizationCycle Row(double t_s, const WorldSampleInput& world,
                          const JointVector& feedback_deg,
                          double dt_s = config::kControlDtS)
{
    CharacterizationCycle row;
    row.dt_s = dt_s;
    row.t_s = t_s;
    row.feedback_deg = feedback_deg;
    row.world = world;
    return row;
}

TrajectoryInput TwoPointTrajectory(std::uint64_t id, std::uint64_t seq,
                                   const CartesianPose& start,
                                   const Eigen::Vector3d& end_position,
                                   double duration_s)
{
    TrajectoryInput traj;
    traj.publish = true;
    traj.trajectory_id = id;
    traj.planner_vicon_sequence = seq;
    traj.points[0].t_from_start_s = 0.0;
    traj.points[0].position_world_m = start.position_m;
    traj.points[0].orientation_world = Eigen::Quaterniond(start.rotation);
    traj.points[0].linear_velocity_world_m_s =
        (end_position - start.position_m) / duration_s;
    traj.points[0].angular_velocity_world_rad_s.setZero();
    traj.points[0].arrival_eligible = false;
    traj.points[1].t_from_start_s = duration_s;
    traj.points[1].position_world_m = end_position;
    traj.points[1].orientation_world = traj.points[0].orientation_world;
    traj.points[1].linear_velocity_world_m_s.setZero();
    traj.points[1].angular_velocity_world_rad_s.setZero();
    traj.points[1].arrival_eligible = true;
    return traj;
}

// Measured world tool pose the pipeline will compute at `row` of a case —
// needed to build trajectories that pass the activation continuity gate.
// Publishing at `row` cannot change that row's measurement (Measure runs
// before the reference Get), so this two-pass construction is exact.
CartesianPose MeasuredPoseAt(const CharacterizationCase& one_case,
                             std::size_t row)
{
    CharacterizationHarness harness;
    harness.Seed(one_case.seed_deg, one_case.seed_velocity_deg_s);
    CharacterizationExpected expected;
    for (std::size_t i = 0; i <= row; ++i)
        expected = harness.Step(one_case.rows[i]);
    CartesianPose pose;
    pose.position_m = expected.meas_p;
    pose.rotation = expected.meas_R;
    return pose;
}

// ---------------------------------------------------------------------
// Scenario cases
// ---------------------------------------------------------------------

// Limiting case 8 (AwaitingWorld tracks the measured pose with exactly
// zero error), the first-fresh replan edge, and limiting case 2 (a
// repeated Vicon sample is byte-identical ZOH, never differentiated).
CharacterizationCase FreshStartupHold()
{
    CharacterizationCase c;
    c.name = "fresh_startup_hold";
    c.seed_deg = SeedDeg();
    WorldSampleInput never;
    never.sequence = 0; // no sample has ever arrived: NaN pose, NaN age
    for (int r = 0; r < 4; ++r)
        c.rows.push_back(Row(0.002 * r, never, c.seed_deg));
    const Eigen::Vector3d v(0.05, -0.02, 0.01);
    const Eigen::Vector3d w(0.01, 0.02, -0.01);
    c.rows.push_back(Row(0.008, WithTwist(FreshWorld(1), v, w), c.seed_deg));
    // Rows 5 and 6 re-read the SAME sample (sequence 2): zero-order hold,
    // only the age advances and stays within the fresh bound.
    c.rows.push_back(
        Row(0.010, WithTwist(FreshWorld(2, kMountP, kMountQ, 0.002), v, w),
            c.seed_deg));
    c.rows.push_back(
        Row(0.012, WithTwist(FreshWorld(2, kMountP, kMountQ, 0.004), v, w),
            c.seed_deg));
    for (int r = 7; r < 10; ++r)
        c.rows.push_back(Row(0.002 * r,
                             WithTwist(FreshWorld(r - 4), v, w), c.seed_deg));
    return c;
}

// A valid plan activates once, interpolates, and completes exactly once.
CharacterizationCase ActiveCartesianReference()
{
    CharacterizationCase c;
    c.name = "active_cartesian_reference";
    c.seed_deg = SeedDeg();
    for (int r = 0; r < 33; ++r)
        c.rows.push_back(Row(0.002 * r, FreshWorld(r + 1), c.seed_deg));
    const CartesianPose start = MeasuredPoseAt(c, 3);
    c.rows[3].trajectory = TwoPointTrajectory(
        101, 2, start, start.position_m + Eigen::Vector3d(0.02, 0.0, 0.0),
        0.05);
    return c;
}

// The Kd path: a moving Mount makes the measured world twist nonzero for
// a stationary arm, and the law opposes that drift immediately.
CharacterizationCase NonzeroMountTwist()
{
    CharacterizationCase c;
    c.name = "nonzero_mount_twist";
    c.seed_deg = SeedDeg();
    const Eigen::Vector3d v(0.3, -0.1, 0.05);
    const Eigen::Vector3d w(0.05, 0.02, -0.03);
    for (int r = 0; r < 6; ++r)
        c.rows.push_back(Row(2.0 + 0.002 * r,
                             WithTwist(FreshWorld(r + 1), v, w), c.seed_deg));
    return c;
}

// Limiting case 6: a far reference pins the per-joint clamp indefinitely
// and there is deliberately NO saturation stop.
CharacterizationCase VelocitySaturation()
{
    CharacterizationCase c;
    c.name = "velocity_saturation";
    c.seed_deg = SeedDeg();
    c.rows.push_back(Row(2.0, FreshWorld(1), c.seed_deg));
    // The wearer lunges half a metre: the world hold stays fixed, the
    // measured pose moves with the Mount, the error is enormous.
    const Eigen::Vector3d moved = kMountP + Eigen::Vector3d(0.5, 0.0, 0.0);
    for (int r = 1; r < 6; ++r)
        c.rows.push_back(Row(2.0 + 0.002 * r,
                             FreshWorld(r + 1, moved), c.seed_deg));
    // One stalled cycle while moving at the clip: the raw dt (10 ms)
    // counts as an overrun, but integration must use the CLAMPED dt
    // (2 x nominal = 4 ms) so the stall cannot become one large jump.
    c.rows.push_back(Row(2.0 + 0.002 * 6, FreshWorld(8, moved), c.seed_deg,
                         0.010));
    return c;
}

// Limiting case 5: a bounded joint (j4, software limit 145 deg) sitting a
// hair inside its boundary receives an outward task pull; the integrator
// must hold the ENTIRE 7-joint frame, name the joint, and the stop is
// kJointLimitWarning. t_s ~ 0 keeps the null-space limit-avoidance ramp
// at zero (exactly the post-takeover condition), so the task term alone
// determines the outward step. The Mount displacement direction was
// found empirically and then frozen; the sanity check below fails if it
// ever stops producing the outward crossing.
CharacterizationCase BoundedJointOutwardHold()
{
    CharacterizationCase c;
    c.name = "bounded_joint_outward_hold";
    c.seed_deg = SeedDeg();
    c.seed_deg[3] = 144.9995; // signed deg, 0.0005 inside the 145 limit
    c.rows.push_back(Row(0.0, FreshWorld(1), c.seed_deg));
    // +x displacement measured to pull j4 outward at this pose
    // (~ +51 deg/s task velocity on joint 4).
    const Eigen::Vector3d moved = kMountP + Eigen::Vector3d(0.05, 0.0, 0.0);
    for (int r = 1; r < 6; ++r)
        c.rows.push_back(Row(0.002 * r, FreshWorld(r + 1, moved),
                             c.seed_deg));
    return c;
}

// Limiting case 4: non-finite controller OUTPUT is held (never
// integrated), counted, and stops after kNonFiniteStopCycles = 3. A
// non-finite joint MEASUREMENT cannot reach this guard: the kinematics
// layer throws on it and the Runner exits as kInternalError. The guard
// protects against non-finite arithmetic inside the law, so the case
// drives it honestly: a finite but astronomically large Mount position
// (contract-valid input) overflows Kp * poseError to infinity.
const Eigen::Vector3d kOverflowMountP(1.0e308, 0.0, 0.0);

CharacterizationCase NonfiniteStop()
{
    CharacterizationCase c;
    c.name = "nonfinite_stop";
    c.seed_deg = SeedDeg();
    for (int r = 0; r < 2; ++r)
        c.rows.push_back(Row(0.002 * r, FreshWorld(r + 1), c.seed_deg));
    for (int r = 2; r < 5; ++r)
        c.rows.push_back(Row(0.002 * r,
                             FreshWorld(r + 1, kOverflowMountP),
                             c.seed_deg));
    return c;
}

// One healthy cycle resets the non-finite counter.
CharacterizationCase NonfiniteRecovery()
{
    CharacterizationCase c;
    c.name = "nonfinite_recovery";
    c.seed_deg = SeedDeg();
    for (int r = 0; r < 2; ++r)
        c.rows.push_back(Row(0.002 * r, FreshWorld(r + 1), c.seed_deg));
    for (int r = 2; r < 4; ++r)
        c.rows.push_back(Row(0.002 * r,
                             FreshWorld(r + 1, kOverflowMountP),
                             c.seed_deg));
    for (int r = 4; r < 6; ++r)
        c.rows.push_back(Row(0.002 * r, FreshWorld(r + 1), c.seed_deg));
    return c;
}

// Trajectory completion into the final hold, then the debounced arrival
// edge after kArrivalDwellS of settled zero error. dt = 0.0029 s (inside
// the overrun threshold) compresses the dwell into fewer cycles.
CharacterizationCase FinalHoldArrival()
{
    CharacterizationCase c;
    c.name = "final_hold_arrival";
    c.seed_deg = SeedDeg();
    const double dt = 0.0029;
    for (int r = 0; r < 62; ++r)
        c.rows.push_back(Row(2.0 + dt * r, FreshWorld(r + 1), c.seed_deg,
                             dt));
    const CartesianPose start = MeasuredPoseAt(c, 2);
    c.rows[2].trajectory =
        TwoPointTrajectory(150, 2, start, start.position_m, 0.01);
    return c;
}

// Limiting case 3 (first half) and risk R1: brief staleness freezes the
// pose and DECAYS the held twist; every fresh cycle resets the stale
// clock, and a second stale interval decays from the NEW twist.
CharacterizationCase BriefStaleDecay()
{
    CharacterizationCase c;
    c.name = "brief_stale_decay";
    c.seed_deg = SeedDeg();
    const Eigen::Vector3d v(0.2, -0.1, 0.15);
    const Eigen::Vector3d w(0.02, -0.01, 0.03);
    for (int r = 0; r < 5; ++r)
        c.rows.push_back(Row(2.0 + 0.002 * r,
                             WithTwist(FreshWorld(r + 1), v, w), c.seed_deg));
    // Occluded Mount, frames still arriving: control-clock staleness.
    for (int r = 5; r < 10; ++r)
        c.rows.push_back(Row(2.0 + 0.002 * r, OccludedWorld(r + 1),
                             c.seed_deg));
    // Stream loss: the slot re-reads the held sample, its age grows.
    for (int r = 10; r < 14; ++r)
        c.rows.push_back(Row(2.0 + 0.002 * r,
                             OccludedWorld(10, 0.052 + 0.002 * (r - 10)),
                             c.seed_deg));
    // One delayed sample far beyond the fresh bound: the SOURCE age term
    // of EffectiveStaleDurationS must dominate the control clock.
    c.rows.push_back(Row(2.0 + 0.002 * 14, OccludedWorld(10, 0.25),
                         c.seed_deg));
    // Recovery, then a second stale interval: decay restarts from the
    // NEW fresh twist because the stale clock reset on EVERY fresh cycle.
    const Eigen::Vector3d v2(0.1, 0.05, -0.05);
    const Eigen::Vector3d w2(-0.01, 0.02, 0.01);
    for (int r = 15; r < 18; ++r)
        c.rows.push_back(Row(2.0 + 0.002 * r,
                             WithTwist(FreshWorld(r), v2, w2), c.seed_deg));
    for (int r = 18; r < 21; ++r)
        c.rows.push_back(Row(2.0 + 0.002 * r, OccludedWorld(r), c.seed_deg));
    return c;
}

// Limiting case 3 (second half) + recovery replan + provenance floor:
// prolonged staleness cancels the active trajectory exactly once and
// pauses at the reference; recovery re-anchors to the CURRENT measured
// pose, fires one replan edge, and advances the minimum planner
// sequence, after which a stale-provenance plan is rejected.
CharacterizationCase ProlongedStaleCancelAndRecovery()
{
    CharacterizationCase c;
    c.name = "prolonged_stale_cancel";
    c.seed_deg = SeedDeg();
    const double dt = 0.0029;
    for (int r = 0; r < 2; ++r)
        c.rows.push_back(Row(2.0 + dt * r, FreshWorld(r + 1), c.seed_deg,
                             dt));
    for (int r = 2; r < 7; ++r)
        c.rows.push_back(Row(2.0 + dt * r, FreshWorld(r + 1), c.seed_deg,
                             dt));
    // Stale long enough for kWorldProlongedStaleS = 0.2 s of accumulated
    // control-cycle dt: 69 cycles at 0.0029 s.
    for (int r = 7; r < 81; ++r)
        c.rows.push_back(Row(2.0 + dt * r, OccludedWorld(r + 1), c.seed_deg,
                             dt));
    for (int r = 81; r < 85; ++r)
        c.rows.push_back(Row(2.0 + dt * r, FreshWorld(93 + r), c.seed_deg,
                             dt));
    const CartesianPose start = MeasuredPoseAt(c, 2);
    c.rows[2].trajectory = TwoPointTrajectory(
        201, 2, start, start.position_m + Eigen::Vector3d(0.02, 0.0, 0.0),
        1.0);
    // Stale provenance: planner sequence 6 predates the recovery minimum.
    const CartesianPose at_recovery = MeasuredPoseAt(c, 84);
    c.rows[84].trajectory = TwoPointTrajectory(
        202, 6, at_recovery,
        at_recovery.position_m + Eigen::Vector3d(0.01, 0.0, 0.0), 0.5);
    return c;
}

// Limiting case 7: a fresh estimator-reset frame (no derivative yet)
// zeroes the twist AND clears the last-fresh memory — a pre-gap velocity
// must never be carried into a new pose, so later staleness has nothing
// to decay.
CharacterizationCase FreshInvalidTwistClearsMemory()
{
    CharacterizationCase c;
    c.name = "fresh_invalid_twist_clears_memory";
    c.seed_deg = SeedDeg();
    const Eigen::Vector3d v(0.2, 0.0, 0.0);
    const Eigen::Vector3d w(0.0, 0.0, 0.1);
    for (int r = 0; r < 3; ++r)
        c.rows.push_back(Row(2.0 + 0.002 * r,
                             WithTwist(FreshWorld(r + 1), v, w), c.seed_deg));
    for (int r = 3; r < 5; ++r)
        c.rows.push_back(Row(2.0 + 0.002 * r, OccludedWorld(r + 1),
                             c.seed_deg));
    // Fresh again but with NO valid twist (estimator reset after a gap).
    c.rows.push_back(Row(2.0 + 0.002 * 5, FreshWorld(6), c.seed_deg));
    for (int r = 6; r < 8; ++r)
        c.rows.push_back(Row(2.0 + 0.002 * r, OccludedWorld(r + 1),
                             c.seed_deg));
    return c;
}

// Stop precedence through the priority ladder (StopPriority.h): each
// case plants adapter facts on row 2 and expects the documented winner.
CharacterizationCase StopCase(const std::string& name, bool live_fault,
                              bool left_low_level, bool stale_feedback,
                              bool following_error)
{
    CharacterizationCase c;
    c.name = name;
    c.seed_deg = SeedDeg();
    for (int r = 0; r < 2; ++r)
        c.rows.push_back(Row(0.002 * r, FreshWorld(r + 1), c.seed_deg));
    JointVector feedback = c.seed_deg;
    if (following_error)
        feedback[0] += 5.0; // 5 deg > kFollowingErrorLimitDeg = 3 deg
    CharacterizationCycle row = Row(0.004, FreshWorld(3), feedback);
    row.adapter.live_fault = live_fault;
    row.adapter.left_low_level = left_low_level;
    row.adapter.stale_feedback = stale_feedback;
    c.rows.push_back(row);
    return c;
}

// Consecutive overrun cycles stop after kOverrunStopCycles = 10. The raw
// measured dt drives the counter; integration uses the CLAMPED dt.
CharacterizationCase OverrunStop()
{
    CharacterizationCase c;
    c.name = "overrun_stop";
    c.seed_deg = SeedDeg();
    for (int r = 0; r < 11; ++r)
        c.rows.push_back(Row(0.005 * r, FreshWorld(r + 1), c.seed_deg,
                             0.005));
    return c;
}

std::vector<CharacterizationCase> BuildCases()
{
    std::vector<CharacterizationCase> cases;
    cases.push_back(FreshStartupHold());
    cases.push_back(ActiveCartesianReference());
    cases.push_back(NonzeroMountTwist());
    cases.push_back(VelocitySaturation());
    cases.push_back(BoundedJointOutwardHold());
    cases.push_back(NonfiniteStop());
    cases.push_back(NonfiniteRecovery());
    cases.push_back(FinalHoldArrival());
    cases.push_back(BriefStaleDecay());
    cases.push_back(ProlongedStaleCancelAndRecovery());
    cases.push_back(FreshInvalidTwistClearsMemory());
    cases.push_back(StopCase("stop_following_error", true, false, false,
                             true));
    cases.push_back(StopCase("stop_live_fault", true, false, false, false));
    cases.push_back(StopCase("stop_left_low_level", true, true, false,
                             false));
    cases.push_back(StopCase("stop_stale_feedback", false, false, true,
                             false));
    cases.push_back(OverrunStop());
    return cases;
}

// ---------------------------------------------------------------------
// Behavioural sanity checks. These are the case-level claims the fixture
// exists to pin; they run on generation AND on every verification, so a
// fixture that no longer demonstrates its behaviour fails loudly.
// ---------------------------------------------------------------------

int CountEdges(const std::vector<CharacterizationExpected>& exp,
               bool CharacterizationExpected::*flag)
{
    int count = 0;
    for (const auto& e : exp)
        if (e.*flag)
            ++count;
    return count;
}

void CheckCase(const CharacterizationCase& c,
               const std::vector<CharacterizationExpected>& exp)
{
    const std::string& n = c.name;
    if (n == "fresh_startup_hold") {
        for (int r = 0; r < 4; ++r) {
            assert(!exp[r].world_fresh);
            assert((exp[r].ref_p - exp[r].meas_p).norm() == 0.0 &&
                   "AwaitingWorld must track the measured pose exactly");
            // Zero error and zero twist command zero velocity, up to the
            // rotation-log of a rotation matrix times its own transpose
            // (identity only to orthonormality rounding, ~1e-16).
            assert(exp[r].qdot_raw_rad_s.norm() < 1e-12 &&
                   "zero error and zero twist must command zero velocity");
            assert(!exp[r].request_replan_edge);
            assert(exp[r].stop == StopKind::kNone);
        }
        assert(exp[4].world_fresh && exp[4].request_replan_edge &&
               "the first fresh world sample requests exactly one plan");
        assert(CountEdges(exp, &CharacterizationExpected::
                                   request_replan_edge) == 1);
        // Limiting case 2: a repeated sample is pure ZOH — every world
        // field of rows 5 and 6 is identical.
        assert(exp[5].world_p == exp[6].world_p);
        assert(exp[5].world_R == exp[6].world_R);
        assert(exp[5].world_v == exp[6].world_v &&
               exp[5].world_w == exp[6].world_w &&
               "a re-read sample must reuse the held twist undecayed");
        assert(exp[5].world_fresh && exp[6].world_fresh);
    } else if (n == "active_cartesian_reference") {
        assert(exp[3].traj_activated &&
               CountEdges(exp, &CharacterizationExpected::traj_activated) ==
                   1);
        assert(CountEdges(exp, &CharacterizationExpected::
                                   traj_complete_edge) == 1);
        assert(exp[4].ref_trajectory_id == 101);
        assert(!exp[4].ref_arrival_eligible &&
               "an interior sample must not be arrival eligible");
        assert(exp[4].ref_time_s > 0.0 && exp[4].ref_time_s < 0.05);
    } else if (n == "nonzero_mount_twist") {
        for (const auto& e : exp) {
            assert(e.meas_v.norm() > 0.0 &&
                   "Mount motion must appear in the measured world twist");
            assert(e.qdot_raw_rad_s.norm() > 0.0 &&
                   "the Kd term must oppose base-injected drift");
            assert(e.stop == StopKind::kNone);
        }
    } else if (n == "velocity_saturation") {
        bool any_saturated = false;
        for (std::size_t r = 1; r < exp.size(); ++r) {
            for (int i = 0; i < 7; ++i) {
                any_saturated = any_saturated || exp[r].saturated[i];
                assert(std::abs(exp[r].qdot_limited_deg_s[i]) <=
                       config::kQdotLimitDegS[i] + 1e-9);
            }
            assert(exp[r].stop == StopKind::kNone &&
                   "clamped transit is normal: NO saturation stop exists");
        }
        assert(any_saturated);
        // The stalled final cycle: raw dt counted as one overrun, no
        // stop; the integrated step (pinned byte-for-byte in the
        // fixture) used the clamped 2 x nominal dt.
        assert(exp.back().overrun && exp.back().overrun_count == 1);
    } else if (n == "bounded_joint_outward_hold") {
        int first_warning = -1;
        for (std::size_t r = 0; r < exp.size(); ++r)
            if (exp[r].joint_limit_warning_joint >= 0) {
                first_warning = static_cast<int>(r);
                break;
            }
        assert(first_warning > 0 &&
               "the outward pull must produce a bounded-joint proposal");
        for (std::size_t r = first_warning; r < exp.size(); ++r) {
            assert(exp[r].joint_limit_warning_joint == 3 &&
                   "joint 4 (index 3) owns the crossing");
            assert(exp[r].stop == StopKind::kJointLimitWarning);
            for (int i = 0; i < 7; ++i) {
                assert(exp[r].commanded_deg[i] ==
                           exp[first_warning - 1].commanded_deg[i] &&
                       "the ENTIRE command frame holds, not one joint");
                assert(exp[r].commanded_velocity_deg_s[i] == 0.0);
            }
        }
    } else if (n == "nonfinite_stop") {
        assert(!exp[1].nonfinite);
        for (int r = 2; r < 5; ++r) {
            assert(exp[r].nonfinite);
            assert(exp[r].nonfinite_count == r - 1);
            for (int i = 0; i < 7; ++i)
                assert(exp[r].commanded_deg[i] == exp[1].commanded_deg[i] &&
                       "a non-finite cycle must hold, never integrate");
        }
        assert(exp[2].stop == StopKind::kNone);
        assert(exp[3].stop == StopKind::kNone);
        assert(exp[4].stop == StopKind::kNonFiniteCommand &&
               "3 consecutive non-finite cycles stop the loop");
    } else if (n == "nonfinite_recovery") {
        assert(exp[3].nonfinite_count == 2);
        assert(!exp[4].nonfinite && exp[4].nonfinite_count == 0 &&
               "one finite cycle resets the non-finite counter");
        for (const auto& e : exp)
            assert(e.stop == StopKind::kNone);
    } else if (n == "final_hold_arrival") {
        assert(exp[2].traj_activated);
        assert(CountEdges(exp, &CharacterizationExpected::
                                   traj_complete_edge) == 1);
        assert(CountEdges(exp, &CharacterizationExpected::arrived_edge) ==
                   1 &&
               "arrival fires exactly once, after the settling dwell");
        assert(CountEdges(exp, &CharacterizationExpected::
                                   not_reached_edge) == 0);
    } else if (n == "brief_stale_decay") {
        for (int r = 5; r < 15; ++r) {
            assert(!exp[r].world_fresh);
            assert(exp[r].world_p == exp[4].world_p &&
                   "staleness freezes the ZOH pose");
            assert(exp[r].world_mount_twist_valid);
            assert(exp[r].world_v.norm() < exp[r - 1].world_v.norm() &&
                   "the held twist must decay continuously, never step");
            assert(!exp[r].traj_cancelled_edge);
        }
        // The delayed row 14 sample (age 0.25 s) makes the SOURCE-age
        // clock dominate: a much stronger decay than the control clock
        // alone would give.
        assert(exp[14].world_v.norm() <
               0.05 * exp[4].world_v.norm());
        for (int r = 15; r < 18; ++r)
            assert(exp[r].world_fresh);
        // Second stale interval decays from the NEW fresh twist (R1).
        assert(exp[18].world_v.norm() < exp[17].world_v.norm());
        assert(exp[18].world_v.norm() > 0.5 * exp[17].world_v.norm() &&
               "the stale clock reset on recovery, so decay restarts");
        assert(CountEdges(exp, &CharacterizationExpected::
                                   request_replan_edge) == 1);
    } else if (n == "prolonged_stale_cancel") {
        assert(exp[2].traj_activated);
        assert(CountEdges(exp, &CharacterizationExpected::
                                   traj_cancelled_edge) == 1 &&
               "prolonged staleness cancels the trajectory exactly once");
        int cancel_row = -1;
        for (std::size_t r = 0; r < exp.size(); ++r)
            if (exp[r].traj_cancelled_edge)
                cancel_row = static_cast<int>(r);
        assert(cancel_row > 7 && cancel_row < 81);
        assert(!exp[cancel_row].ref_arrival_eligible &&
               "the paused hold is never arrival eligible");
        assert(exp[cancel_row].ref_p == exp[cancel_row - 1].ref_p &&
               "cancellation holds the PAUSED reference pose");
        // Recovery: one replan edge, hold re-anchored to the CURRENT
        // measured pose.
        assert(exp[81].world_fresh && exp[81].request_replan_edge);
        assert(CountEdges(exp, &CharacterizationExpected::
                                   request_replan_edge) == 2 &&
               "startup and recovery each request exactly one plan");
        assert((exp[81].ref_p - exp[81].meas_p).norm() == 0.0 &&
               "recovery re-anchors the hold to the measured pose");
        // Provenance floor: the recovery advanced the minimum planner
        // sequence, so a pre-blackout plan is rejected.
        assert(exp[84].traj_rejected && !exp[84].traj_activated);
    } else if (n == "fresh_invalid_twist_clears_memory") {
        assert(exp[3].world_mount_twist_valid &&
               exp[3].world_v.norm() > 0.0);
        assert(!exp[5].world_mount_twist_valid &&
               exp[5].world_v.norm() == 0.0 &&
               "an estimator-reset frame has no derivative yet");
        for (int r = 6; r < 8; ++r) {
            assert(!exp[r].world_mount_twist_valid &&
                   exp[r].world_v.norm() == 0.0 &&
                   "cleared memory leaves nothing to decay: a pre-gap "
                   "velocity never enters the new pose");
        }
    } else if (n == "stop_following_error") {
        assert(exp[2].stop == StopKind::kFollowingError &&
               "following error outranks a live fault");
        assert(exp[2].live_fault_observed);
    } else if (n == "stop_live_fault") {
        assert(exp[2].stop == StopKind::kRobotFault);
        assert(exp[2].live_fault_observed);
    } else if (n == "stop_left_low_level") {
        assert(exp[2].stop == StopKind::kLeftLowLevel &&
               "leaving low-level servoing outranks the fault stop");
        assert(exp[2].live_fault_observed);
    } else if (n == "stop_stale_feedback") {
        assert(exp[2].stop == StopKind::kStaleFeedback);
    } else if (n == "overrun_stop") {
        assert(exp[0].overrun_count == 0 &&
               "the first cycle uses the nominal dt and cannot overrun");
        for (int r = 1; r <= 10; ++r)
            assert(exp[r].overrun && exp[r].overrun_count == r);
        for (int r = 1; r < 10; ++r)
            assert(exp[r].stop == StopKind::kNone);
        assert(exp[10].stop == StopKind::kOverrun &&
               "10 consecutive overruns stop the loop");
    } else {
        assert(false && "unknown characterization case name");
    }
}

// ---------------------------------------------------------------------
// Provenance: identify EXACTLY which sources produced the fixture, even
// on an uncommitted working tree.
// ---------------------------------------------------------------------

std::string RunCommand(const std::string& command)
{
    std::string output;
    FILE* pipe = popen(command.c_str(), "r");
    if (pipe == nullptr)
        return output;
    char buffer[4096];
    while (fgets(buffer, sizeof buffer, pipe) != nullptr)
        output += buffer;
    pclose(pipe);
    while (!output.empty() &&
           (output.back() == '\n' || output.back() == '\r'))
        output.pop_back();
    return output;
}

// Relative to CONTROL_SOURCE_DIR. Runner.cpp is included deliberately: the
// fixture froze the per-cycle ORDER the Runner established, so a change there
// is a change to what this evidence describes, even though the file now lives
// in ../runtime.
const char* const kProvenanceInputs[] = {
    "Actuation.cpp", "Actuation.h", "Arrival.h",
    "CartesianReference.cpp", "CartesianReference.h",
    "CartesianTrajectoryMailbox.cpp", "CartesianTrajectoryMailbox.h",
    "Config.h", "Controller.cpp", "Controller.h",
    "RobotModel.cpp", "RobotModel.h", "Frames.h",
    "Kinematics.cpp", "Kinematics.h", "ReactiveLaw.h",
    "../runtime/Runner.cpp", "State.h", "StopPriority.h",
    "../contracts/WorldCartesianTrajectory.cpp",
    "../contracts/WorldCartesianTrajectory.h",
    "../model/GEN3_dual_mounted.urdf",
    "tests/execution_characterization.h",
    "tests/test_execution_characterization.cpp",
};

std::string ProvenanceHeader()
{
    const std::string source_dir = CONTROL_SOURCE_DIR;
    std::string header = "# execution_characterization_fixture = 1\n";
    std::string revision =
        RunCommand("git -C '" + source_dir + "' rev-parse HEAD 2>/dev/null");
    if (revision.empty())
        revision = "unknown";
    const std::string dirty = RunCommand(
        "git -C '" + source_dir + "' status --porcelain 2>/dev/null | head -1");
    if (!dirty.empty())
        revision += "+dirty";
    header += "# git_revision = " + revision + "\n";
    header += "# nominal_dt_s = " + FormatDouble(config::kControlDtS) + "\n";
    for (const char* relative : kProvenanceInputs) {
        const std::string hash_line =
            RunCommand("sha256sum '" + source_dir + "/" + relative +
                       "' 2>/dev/null");
        const std::string hash =
            hash_line.empty() ? "unavailable"
                              : hash_line.substr(0, hash_line.find(' '));
        header += std::string("# sha256 ") + relative + " = " + hash + "\n";
    }
    return header;
}

// Prints the first differing line so a replay failure names the cycle.
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
    assert(false && "characterization text mismatch — see stderr");
}

} // namespace

int main()
{
    const std::vector<CharacterizationCase> cases = BuildCases();

    // Prediction P1: the characterization is deterministic. Two
    // independent harness runs must serialize byte-identically.
    const auto results = RunCases(cases);
    const auto results_again = RunCases(cases);
    const std::string data = SerializeDataSection(cases, results);
    const std::string data_again =
        SerializeDataSection(cases, results_again);
    RequireEqualText(data_again, data, "determinism (P1)");

    std::size_t total_rows = 0;
    for (std::size_t c = 0; c < cases.size(); ++c) {
        CheckCase(cases[c], results[c]);
        total_rows += cases[c].rows.size();
    }

    const std::string fixture_path = EXECUTION_FIXTURE_PATH;
    std::ifstream existing(fixture_path);
    if (!existing) {
        // First run: freeze the evidence.
        std::filesystem::create_directories(
            std::filesystem::path(fixture_path).parent_path());
        std::ofstream out(fixture_path);
        assert(out && "cannot write the fixture file");
        out << ProvenanceHeader() << data;
        out.close();
        std::cout << "generated " << fixture_path << " (" << cases.size()
                  << " cases, " << total_rows << " rows)\n";
    } else {
        // Replay gate: the STORED fixture is the authority. Re-run the
        // pipeline on the fixture's own recorded inputs and require the
        // regenerated data section byte-for-byte.
        ParsedFixture fixture = ParseFixture(existing);
        const auto replayed = RunCases(fixture.cases);
        const std::string replay_text =
            SerializeDataSection(fixture.cases, replayed);
        RequireEqualText(replay_text, fixture.data_section,
                         "fixture replay (P2)");
        for (std::size_t c = 0; c < fixture.cases.size(); ++c)
            CheckCase(fixture.cases[c], replayed[c]);
        // The in-code case builder must still describe the same fixture;
        // if it changed, regenerate deliberately (delete the fixture)
        // rather than letting two sources of truth drift apart.
        RequireEqualText(data, fixture.data_section,
                         "case builder vs stored fixture");
        std::cout << "verified " << fixture_path << " ("
                  << fixture.cases.size() << " cases, " << total_rows
                  << " rows) against the current implementation\n";
    }
    std::cout << "all execution characterization checks passed\n";
    return 0;
}
