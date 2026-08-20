//
// DecidePlanVerdict — the three-valued gate over the validation report.
// Pure logic, no planner or robot model involved: build a report that a
// good plan would produce, perturb one measurement at a time, and check
// which side of ACCEPT / WARNING / REJECT it lands on.
//

#include <cstdio>
#include <cstdlib>
#include <string>

#include "PathValidationReport.h"

namespace {

int failures = 0;

void Check(bool condition, const std::string& what) {
    if (!condition) {
        std::printf("FAIL: %s\n", what.c_str());
        ++failures;
    }
}

VerdictThresholds Thresholds() {
    VerdictThresholds t;
    t.maximum_planning_error_m = 0.005;
    t.maximum_orientation_error_rad = 0.1;
    return t;
}

// A report every check passes cleanly.
PathValidationReport CleanReport() {
    PathValidationReport report;
    report.command.max_position_m = 0.002;
    report.command.max_orientation_rad = 0.05;
    report.modelled_collision_valid = true;
    report.minimum_clearance_m = 0.05;
    report.joint_limits_valid = true;
    report.minimum_joint_limit_margin_rad = 0.3;
    report.max_velocity_limit_ratio = 0.8;
    report.max_acceleration_limit_ratio = 0.5;
    report.start_state_valid = true;
    report.all_finite = true;
    report.optimiser_converged = true;
    return report;
}

}  // namespace

int main() {
    const VerdictThresholds t = Thresholds();

    {
        PathValidationReport r = CleanReport();
        Check(DecidePlanVerdict(r, t) == PlanVerdict::kAccept, "clean plan accepts");
        Check(r.warnings.empty(), "clean plan carries no warnings");
    }

    // Fidelity: within tolerance accepts, within 2x warns, beyond 2x rejects.
    {
        PathValidationReport r = CleanReport();
        r.command.max_position_m = 0.007;  // 5 < 7 <= 10 mm
        Check(DecidePlanVerdict(r, t) == PlanVerdict::kWarning,
              "7 mm position error warns");
        Check(!r.warnings.empty(), "fidelity warning is recorded");
    }
    {
        PathValidationReport r = CleanReport();
        r.command.max_position_m = 0.012;  // > 10 mm
        Check(DecidePlanVerdict(r, t) == PlanVerdict::kReject,
              "12 mm position error rejects");
    }
    {
        PathValidationReport r = CleanReport();
        r.command.max_orientation_rad = 0.15;  // 0.1 < 0.15 <= 0.2
        Check(DecidePlanVerdict(r, t) == PlanVerdict::kWarning,
              "0.15 rad orientation error warns");
    }
    {
        PathValidationReport r = CleanReport();
        r.command.max_orientation_rad = 0.25;  // > 0.2
        Check(DecidePlanVerdict(r, t) == PlanVerdict::kReject,
              "0.25 rad orientation error rejects");
    }

    // Collision: penetration rejects, low positive clearance warns.
    {
        PathValidationReport r = CleanReport();
        r.modelled_collision_valid = false;
        r.minimum_clearance_m = -0.001;
        Check(DecidePlanVerdict(r, t) == PlanVerdict::kReject, "penetration rejects");
    }
    {
        PathValidationReport r = CleanReport();
        r.minimum_clearance_m = 0.006;  // positive but under 10 mm
        Check(DecidePlanVerdict(r, t) == PlanVerdict::kWarning,
              "6 mm clearance warns");
    }

    // Velocity: <=5% over the per-joint limit warns, more rejects.
    {
        PathValidationReport r = CleanReport();
        r.max_velocity_limit_ratio = 1.03;
        Check(DecidePlanVerdict(r, t) == PlanVerdict::kWarning,
              "3% velocity overage warns");
    }
    {
        PathValidationReport r = CleanReport();
        r.max_velocity_limit_ratio = 1.10;
        Check(DecidePlanVerdict(r, t) == PlanVerdict::kReject,
              "10% velocity overage rejects");
    }

    // Acceleration limits are a heuristic (velocity limit / 0.5 s), so an
    // overage is a warning, never a rejection on its own.
    {
        PathValidationReport r = CleanReport();
        r.max_acceleration_limit_ratio = 1.4;
        Check(DecidePlanVerdict(r, t) == PlanVerdict::kWarning,
              "acceleration overage alone warns");
    }

    // Hard gates unchanged: non-finite, joint limit, splice, convergence.
    {
        PathValidationReport r = CleanReport();
        r.all_finite = false;
        Check(DecidePlanVerdict(r, t) == PlanVerdict::kReject, "non-finite rejects");
    }
    {
        PathValidationReport r = CleanReport();
        r.joint_limits_valid = false;
        Check(DecidePlanVerdict(r, t) == PlanVerdict::kReject,
              "joint-limit violation rejects");
    }
    {
        PathValidationReport r = CleanReport();
        r.start_state_valid = false;
        Check(DecidePlanVerdict(r, t) == PlanVerdict::kReject,
              "splice-guard failure rejects");
    }
    {
        PathValidationReport r = CleanReport();
        r.optimiser_converged = false;
        Check(DecidePlanVerdict(r, t) == PlanVerdict::kReject,
              "non-converged optimiser rejects");
    }

    if (failures == 0) std::printf("test_plan_verdict: all checks passed\n");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
