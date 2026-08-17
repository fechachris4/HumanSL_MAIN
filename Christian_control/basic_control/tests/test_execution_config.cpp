//
// Hardware-free tests for the immutable execution-configuration snapshot
// (Plan 01 Task 2). Three claims are checked:
//
//   1. ProductionExecutionConfig() is the IDENTITY mapping onto today's
//      compiled config:: constants — every member bitwise-equal, per-joint
//      vectors element by element (prediction P4).
//   2. ValidateExecutionConfig rejects physically meaningless values:
//      non-finite gains, non-positive limits, inverted stale ordering,
//      negative tolerances, and bounded-joint magnitudes the wrapped
//      signed-degree representation cannot express.
//   3. The snapshot is what the per-cycle code actually consumes: a custom
//      (non-production) config visibly changes the bounded-joint hold and
//      the reference source's activation/prolonged-stale behaviour. A
//      production-config unit under the same inputs behaves as today, so an
//      implementation that secretly kept reading config:: would fail here.
//
// No Kortex, no Pinocchio, no robot. Returns nonzero on the first failure.
//

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "Actuation.h"
#include "CartesianReference.h"
#include "Config.h"
#include "ExecutionConfig.h"

namespace
{

    int failures = 0;

    void Check(bool ok, const std::string& what)
    {
        if (!ok) {
            std::cout << "FAIL: " << what << "\n";
            ++failures;
        }
    }

    constexpr double kDegToRad = M_PI / 180.0;

    // ------------------------------------------------------------------
    // 1. Identity mapping (P4): every member equals its config:: source.
    // ------------------------------------------------------------------

    void TestProductionMappingIdentity()
    {
        const ExecutionConfig cfg = ProductionExecutionConfig();

        // Reactive-law gains, exactly ConfiguredGains() as compiled today.
        Check(cfg.gains.kp_position_s_inv == config::kKpCartesian,
              "gains.kp_position_s_inv == kKpCartesian");
        Check(cfg.gains.kp_rotation_s_inv == config::kKpRotation,
              "gains.kp_rotation_s_inv == kKpRotation");
        Check(cfg.gains.kd_position == config::kKdPosition,
              "gains.kd_position == kKdPosition");
        Check(cfg.gains.kd_rotation == config::kKdRotation,
              "gains.kd_rotation == kKdRotation");
        Check(cfg.gains.limit_avoid_gain_s_inv == config::kLimitAvoidGain,
              "gains.limit_avoid_gain_s_inv == kLimitAvoidGain");
        Check(cfg.gains.dls_lambda == config::kDlsLambda,
              "gains.dls_lambda == kDlsLambda");
        // The position term has no config switch: production always enables
        // it (Controller.cpp's gain mapping has always left the default).
        Check(cfg.gains.position_enabled,
              "gains.position_enabled is unconditionally true");
        Check(cfg.gains.orientation_enabled == config::kOrientationEnabled,
              "gains.orientation_enabled == kOrientationEnabled");
        Check(cfg.gains.velocity_enabled == config::kVelocityTermEnabled,
              "gains.velocity_enabled == kVelocityTermEnabled");
        Check(cfg.gains.null_space_enabled == config::kNullSpaceEnabled,
              "gains.null_space_enabled == kNullSpaceEnabled");

        for (int i = 0; i < 7; ++i) {
            Check(cfg.velocity_limit_deg_s[i] == config::kQdotLimitDegS[i],
                  "velocity_limit_deg_s[" + std::to_string(i) +
                      "] == kQdotLimitDegS");
            Check(cfg.software_limit_deg[i] ==
                      config::kJointSoftwareLimitDeg[i],
                  "software_limit_deg[" + std::to_string(i) +
                      "] == kJointSoftwareLimitDeg");
        }

        // The zone is stored in radians using the SAME conversion expression
        // Controller.cpp used, so the stored double is bitwise identical.
        Check(cfg.limit_avoid_zone_rad ==
                  config::kLimitAvoidZoneDeg * kDegToRad,
              "limit_avoid_zone_rad == kLimitAvoidZoneDeg * pi/180");

        Check(cfg.command_lead_limit_deg == config::kCommandLeadLimitDeg,
              "command_lead_limit_deg == kCommandLeadLimitDeg");
        Check(cfg.following_error_limit_deg ==
                  config::kFollowingErrorLimitDeg,
              "following_error_limit_deg == kFollowingErrorLimitDeg");
        Check(cfg.world_fresh_max_age_s == config::kWorldFreshMaxAgeS,
              "world_fresh_max_age_s == kWorldFreshMaxAgeS");
        Check(cfg.world_prolonged_stale_s == config::kWorldProlongedStaleS,
              "world_prolonged_stale_s == kWorldProlongedStaleS");
        // R4 resolution: the stale-twist decay time constant travels in the
        // snapshot so Task 3 can relocate the decay without a config:: read.
        Check(cfg.mount_twist_filter_tau_s ==
                  config::kViconMountTwistFilterTauS,
              "mount_twist_filter_tau_s == kViconMountTwistFilterTauS");
        Check(cfg.arrival_position_tolerance_m == config::kArrivalToleranceM,
              "arrival_position_tolerance_m == kArrivalToleranceM");
        Check(cfg.arrival_orientation_tolerance_rad ==
                  config::kArrivalOrientationToleranceRad,
              "arrival_orientation_tolerance_rad == "
              "kArrivalOrientationToleranceRad");
        Check(cfg.arrival_dwell_s == config::kArrivalDwellS,
              "arrival_dwell_s == kArrivalDwellS");
        Check(cfg.target_hold_s == config::kTargetHoldS,
              "target_hold_s == kTargetHoldS");
        Check(cfg.null_ramp_duration_s == config::kNullRampDurationS,
              "null_ramp_duration_s == kNullRampDurationS");
        Check(cfg.stop_on_fault == config::kStopOnFault,
              "stop_on_fault == kStopOnFault");
        Check(cfg.disable_following_error_stop ==
                  config::kDisableFollowingErrorStop,
              "disable_following_error_stop == kDisableFollowingErrorStop");
        Check(cfg.nonfinite_stop_cycles == config::kNonFiniteStopCycles,
              "nonfinite_stop_cycles == kNonFiniteStopCycles");
        Check(cfg.overrun_stop_cycles == config::kOverrunStopCycles,
              "overrun_stop_cycles == kOverrunStopCycles");
    }

    // ------------------------------------------------------------------
    // 2. Validation: production passes; meaningless values are rejected.
    // ------------------------------------------------------------------

    bool Rejected(const ExecutionConfig& config)
    {
        try {
            ValidateExecutionConfig(config);
        } catch (const std::invalid_argument&) {
            return true;
        }
        return false;
    }

    template <typename Mutate>
    void ExpectRejected(const std::string& what, Mutate mutate)
    {
        ExecutionConfig config = ProductionExecutionConfig();
        mutate(config);
        Check(Rejected(config), "rejects " + what);
    }

    template <typename Mutate>
    void ExpectAccepted(const std::string& what, Mutate mutate)
    {
        ExecutionConfig config = ProductionExecutionConfig();
        mutate(config);
        Check(!Rejected(config), "accepts " + what);
    }

    void TestValidation()
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double inf = std::numeric_limits<double>::infinity();

        Check(!Rejected(ProductionExecutionConfig()),
              "the production snapshot validates");

        // Non-finite or sign-inverted gains: the law's output would be
        // non-finite or destabilizing by construction.
        ExpectRejected("NaN kp_position",
                       [&](ExecutionConfig& c) {
                           c.gains.kp_position_s_inv = nan;
                       });
        ExpectRejected("negative kp_position",
                       [](ExecutionConfig& c) {
                           c.gains.kp_position_s_inv = -1.0;
                       });
        ExpectRejected("infinite kd_rotation",
                       [&](ExecutionConfig& c) { c.gains.kd_rotation = inf; });
        ExpectRejected("negative limit_avoid_gain",
                       [](ExecutionConfig& c) {
                           c.gains.limit_avoid_gain_s_inv = -0.5;
                       });
        // lambda = 0 removes the damped inverse's well-posedness guarantee
        // (JJ^T + lambda^2 I is only guaranteed invertible for lambda > 0).
        ExpectRejected("zero dls_lambda",
                       [](ExecutionConfig& c) { c.gains.dls_lambda = 0.0; });
        ExpectRejected("NaN dls_lambda",
                       [&](ExecutionConfig& c) { c.gains.dls_lambda = nan; });

        // Non-positive velocity limits: std::clamp(x, -lim, lim) with a
        // negative lim has lo > hi (undefined); zero silently freezes a
        // joint the law believes it is moving.
        ExpectRejected("zero velocity limit",
                       [](ExecutionConfig& c) {
                           c.velocity_limit_deg_s[0] = 0.0;
                       });
        ExpectRejected("negative velocity limit",
                       [](ExecutionConfig& c) {
                           c.velocity_limit_deg_s[6] = -5.0;
                       });
        ExpectRejected("NaN velocity limit",
                       [&](ExecutionConfig& c) {
                           c.velocity_limit_deg_s[3] = nan;
                       });

        // Bounded-joint magnitudes: 0 is the continuous-joint sentinel;
        // negative boundaries are meaningless; magnitudes >= 180 deg cannot
        // be expressed by the signed-remainder wrap the boundary check and
        // the null-space deadband both use.
        ExpectRejected("negative software limit",
                       [](ExecutionConfig& c) {
                           c.software_limit_deg[1] = -1.0;
                       });
        ExpectRejected("software limit >= 180 deg",
                       [](ExecutionConfig& c) {
                           c.software_limit_deg[3] = 200.0;
                       });
        ExpectRejected("NaN software limit",
                       [&](ExecutionConfig& c) {
                           c.software_limit_deg[5] = nan;
                       });

        ExpectRejected("negative limit-avoid zone",
                       [](ExecutionConfig& c) {
                           c.limit_avoid_zone_rad = -0.1;
                       });

        // Lead limit: zero would pin the command to feedback (no motion
        // possible), negative is meaningless; +infinity is the documented
        // "bound disabled" value and must pass.
        ExpectRejected("zero command lead limit",
                       [](ExecutionConfig& c) {
                           c.command_lead_limit_deg = 0.0;
                       });
        ExpectRejected("negative command lead limit",
                       [](ExecutionConfig& c) {
                           c.command_lead_limit_deg = -1.0;
                       });
        ExpectRejected("NaN command lead limit",
                       [&](ExecutionConfig& c) {
                           c.command_lead_limit_deg = nan;
                       });
        ExpectAccepted("infinite command lead limit (bound disabled)",
                       [&](ExecutionConfig& c) {
                           c.command_lead_limit_deg = inf;
                       });

        ExpectRejected("zero following-error limit",
                       [](ExecutionConfig& c) {
                           c.following_error_limit_deg = 0.0;
                       });

        // Staleness ladder ordering: a sample may be up to
        // world_fresh_max_age_s old while still trusted, so the prolonged
        // cancellation threshold below that inverts the ladder.
        ExpectRejected("zero world fresh max age",
                       [](ExecutionConfig& c) {
                           c.world_fresh_max_age_s = 0.0;
                       });
        ExpectRejected("prolonged-stale below the fresh age bound",
                       [](ExecutionConfig& c) {
                           c.world_prolonged_stale_s =
                               c.world_fresh_max_age_s * 0.5;
                       });
        // The decay multiplier is exp(-stale/tau): tau <= 0 is undefined.
        ExpectRejected("non-positive mount twist filter tau",
                       [](ExecutionConfig& c) {
                           c.mount_twist_filter_tau_s = 0.0;
                       });

        ExpectRejected("negative arrival position tolerance",
                       [](ExecutionConfig& c) {
                           c.arrival_position_tolerance_m = -1e-9;
                       });
        ExpectRejected("negative arrival orientation tolerance",
                       [](ExecutionConfig& c) {
                           c.arrival_orientation_tolerance_rad = -0.001;
                       });
        ExpectRejected("NaN arrival dwell",
                       [&](ExecutionConfig& c) { c.arrival_dwell_s = nan; });
        ExpectAccepted("non-positive arrival dwell (debounce disabled)",
                       [](ExecutionConfig& c) { c.arrival_dwell_s = -1.0; });

        // Mirrors Config.h's static_assert: an enabled non-arrival timeout
        // must outlast the settling window.
        ExpectRejected("target hold not outlasting the arrival dwell",
                       [](ExecutionConfig& c) {
                           c.target_hold_s = c.arrival_dwell_s * 0.5;
                       });
        ExpectAccepted("zero target hold (timeout disabled)",
                       [](ExecutionConfig& c) { c.target_hold_s = 0.0; });

        ExpectRejected("infinite null ramp duration",
                       [&](ExecutionConfig& c) {
                           c.null_ramp_duration_s = inf;
                       });
        ExpectAccepted("zero null ramp duration (full gain immediately)",
                       [](ExecutionConfig& c) {
                           c.null_ramp_duration_s = 0.0;
                       });
    }

    // ------------------------------------------------------------------
    // 3. The injected snapshot is what the code consumes.
    // ------------------------------------------------------------------

    // A custom bounded-joint boundary must move the whole-frame hold: if
    // Apply still read config::kJointSoftwareLimitDeg per cycle, the custom
    // unit would not warn and the production unit comparison would hide it.
    void TestPositionIntegrationConsumesSnapshot()
    {
        ExecutionConfig custom = ProductionExecutionConfig();
        custom.software_limit_deg[1] = 10.0; // production: 126.9 deg
        ValidateExecutionConfig(custom);

        RobotState state{};
        state.q_rad[1] = 9.99 * kDegToRad;
        Eigen::Matrix<double, 7, 1> qdot =
            Eigen::Matrix<double, 7, 1>::Zero();
        qdot[1] = 50.0 * kDegToRad; // 0.1 deg outward step at dt = 2 ms
        JointVector setpoints{};
        JointVector velocity{};

        PositionIntegration custom_unit(custom);
        custom_unit.Prepare(state);
        const PositionIntegration::ApplyStatus custom_status =
            custom_unit.Apply(qdot, state, 0.002, setpoints, velocity);
        Check(custom_status.joint_limit_warning_joint.has_value() &&
                  custom_status.joint_limit_warning_joint.value() == 1,
              "custom 10 deg boundary holds joint 2's outward proposal");
        Check(setpoints[1] == 9.99 && velocity[1] == 0.0,
              "custom boundary holds the whole frame at the prior command");

        PositionIntegration production_unit(ProductionExecutionConfig());
        production_unit.Prepare(state);
        const PositionIntegration::ApplyStatus production_status =
            production_unit.Apply(qdot, state, 0.002, setpoints, velocity);
        Check(!production_status.joint_limit_warning_joint.has_value(),
              "production boundary (126.9 deg) does not hold at 10 deg");
        Check(std::abs(setpoints[1] - 10.09) < 1e-9,
              "production unit integrates the outward step normally");
    }

    std::unique_ptr<WorldCartesianTrajectory> OffsetTrajectory(
        std::uint64_t id, std::uint64_t planner_sequence, double start_x_m)
    {
        auto trajectory = std::make_unique<WorldCartesianTrajectory>();
        trajectory->trajectory_id = id;
        trajectory->planner_vicon_sequence = planner_sequence;
        WorldCartesianTrajectoryPoint first;
        first.position_world_m = Eigen::Vector3d(start_x_m, 0.0, 0.0);
        first.orientation_world = Eigen::Quaterniond::Identity();
        WorldCartesianTrajectoryPoint last;
        last.t_from_start_s = 1.0;
        last.position_world_m = Eigen::Vector3d(start_x_m + 0.1, 0.0, 0.0);
        last.orientation_world = Eigen::Quaterniond::Identity();
        last.arrival_eligible = true;
        trajectory->points = {first, last};
        return trajectory;
    }

    // A trajectory starting 0.3 m from the measured pose activates only
    // under a 0.5 m continuity tolerance — proof the stored tolerance, not
    // config::kArrivalToleranceM, gates activation.
    void TestReferenceSourceConsumesArrivalTolerances()
    {
        RobotState state{};
        state.world_fresh = true;
        state.world_sequence = 10;
        MeasuredCartesianState measured{}; // origin, identity

        ExecutionConfig loose = ProductionExecutionConfig();
        loose.arrival_position_tolerance_m = 0.5;
        loose.arrival_orientation_tolerance_rad = 0.5;
        ValidateExecutionConfig(loose);

        CartesianTrajectoryMailbox loose_mailbox;
        CartesianReferenceSource loose_source(loose_mailbox, loose);
        ControllerStatus status{};
        loose_source.Get(state, measured, 0.002, 0.0, status); // awaiting -> hold
        loose_mailbox.Publish(OffsetTrajectory(1, 10, 0.3));
        status = ControllerStatus{};
        loose_source.Get(state, measured, 0.002, 0.0, status);
        Check(status.cartesian_traj_activated,
              "0.3 m start offset activates under the injected 0.5 m "
              "tolerance");

        CartesianTrajectoryMailbox strict_mailbox;
        CartesianReferenceSource strict_source(strict_mailbox,
                                               ProductionExecutionConfig());
        status = ControllerStatus{};
        strict_source.Get(state, measured, 0.002, 0.0, status);
        strict_mailbox.Publish(OffsetTrajectory(1, 10, 0.3));
        status = ControllerStatus{};
        strict_source.Get(state, measured, 0.002, 0.0, status);
        Check(status.cartesian_traj_rejected && !status.cartesian_traj_activated,
              "production 1 mm continuity tolerance rejects the same start");
        strict_mailbox.DeleteRetired();
    }

    // 0.06 s of accumulated stale time crosses a 0.05 s prolonged-stale
    // threshold but not the production 0.2 s one: only the injected source
    // may request a replan on recovery.
    void TestReferenceSourceConsumesProlongedStale()
    {
        MeasuredCartesianState measured{};
        RobotState fresh{};
        fresh.world_fresh = true;
        fresh.world_sequence = 5;
        RobotState stale{};
        stale.world_fresh = false;
        stale.world_sequence = 5;

        ExecutionConfig quick = ProductionExecutionConfig();
        quick.world_prolonged_stale_s = 0.05; // == fresh age bound: minimum
        ValidateExecutionConfig(quick);

        CartesianTrajectoryMailbox quick_mailbox;
        CartesianReferenceSource quick_source(quick_mailbox, quick);
        CartesianTrajectoryMailbox slow_mailbox;
        CartesianReferenceSource slow_source(slow_mailbox,
                                             ProductionExecutionConfig());

        ControllerStatus status{};
        quick_source.Get(fresh, measured, 0.002, 0.0, status);
        status = ControllerStatus{};
        slow_source.Get(fresh, measured, 0.002, 0.0, status);

        // The stale clock is owned by ArmExecutionCore in production
        // (ExecutionCore.h); here the test advances it exactly as the core
        // does: += dt per stale cycle, passed post-update.
        for (int i = 0; i < 3; ++i) { // 0.06 s accumulated stale time
            const double stale_elapsed_s = 0.02 * (i + 1);
            status = ControllerStatus{};
            quick_source.Get(stale, measured, 0.02, stale_elapsed_s, status);
            status = ControllerStatus{};
            slow_source.Get(stale, measured, 0.02, stale_elapsed_s, status);
        }

        fresh.world_sequence = 6;
        ControllerStatus quick_status{};
        quick_source.Get(fresh, measured, 0.002, 0.0, quick_status);
        Check(quick_status.request_replan_edge,
              "0.06 s stale crosses the injected 0.05 s threshold: replan");
        ControllerStatus slow_status{};
        slow_source.Get(fresh, measured, 0.002, 0.0, slow_status);
        Check(!slow_status.request_replan_edge,
              "0.06 s stale stays below the production 0.2 s threshold");
    }

} // namespace

int main()
{
    TestProductionMappingIdentity();
    TestValidation();
    TestPositionIntegrationConsumesSnapshot();
    TestReferenceSourceConsumesArrivalTolerances();
    TestReferenceSourceConsumesProlongedStale();

    if (failures != 0) {
        std::cout << failures << " execution-config check(s) failed\n";
        return 1;
    }
    std::cout << "all execution-config checks passed\n";
    return 0;
}
