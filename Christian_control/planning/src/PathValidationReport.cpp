#include "PathValidationReport.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace {

const char* YesNo(bool value) { return value ? "yes" : "NO"; }

const char* VerdictName(PlanVerdict verdict) {
    switch (verdict) {
    case PlanVerdict::kAccept: return "ACCEPT";
    case PlanVerdict::kWarning: return "WARNING";
    case PlanVerdict::kReject: return "REJECT";
    }
    return "REJECT";
}

// Band edges, relative to the requested tolerances. Deliberately compiled
// in rather than configured: the yaml keys stay the single statement of
// what the task requires, and these only say how far past "required" is
// still usable (2026-08-20 decision, Christian).
constexpr double kClearanceWarningM = 0.010;        // positive but tight
constexpr double kVelocityRejectRatio = 1.05;       // >5% over a limit

}  // namespace

PlanVerdict DecidePlanVerdict(PathValidationReport& report,
                              const VerdictThresholds& thresholds) {
    report.warnings.clear();
    std::ostringstream text;
    text.setf(std::ios::fixed);
    text.precision(3);
    const auto warn = [&](const std::string& message) {
        report.warnings.push_back(message);
    };

    // ---- hard rejections: physically invalid trajectories ONLY ----------
    // Each line here has a concrete physical failure behind it; everything
    // that is merely IMPERFECT (tracking error, clearance margin, optimiser
    // convergence quality) is quality information, reported as a warning
    // and left to the layer with context. (Policy change 2026-08-21: task
    // error beyond twice tolerance and non-convergence used to reject.)
    bool reject = false;
    // Non-finite states cannot be commanded at all.
    if (!report.all_finite) reject = true;
    // A state beyond a position limit faults the actuator's firmware bank.
    if (!report.joint_limits_valid) reject = true;
    // A trajectory not starting at the measured arm makes the controller
    // command a physical step (the splice guard's reason for existing).
    if (!report.start_state_valid) reject = true;
    // Velocity far over the firmware MAXIMUM_VELOCITY bank faults the arm
    // mid-motion (docs/decisions/qdot-limit-raise.md); small overage warns.
    if (report.max_velocity_limit_ratio > kVelocityRejectRatio) reject = true;
    if (reject) {
        report.verdict = PlanVerdict::kReject;
        return report.verdict;
    }

    // ---- warnings: usable, imperfect, said out loud ---------------------
    if (!report.optimiser_converged)
        warn("optimiser did not report convergence; the fidelity numbers "
             "below say what the trajectory actually achieves");
    if (report.command.max_position_m > thresholds.maximum_planning_error_m) {
        text.str("");
        text << "position error " << report.command.max_position_m * 1000.0
             << " mm exceeds the requested "
             << thresholds.maximum_planning_error_m * 1000.0 << " mm";
        warn(text.str());
    }
    if (report.command.max_orientation_rad >
        thresholds.maximum_orientation_error_rad) {
        text.str("");
        text << "orientation error "
             << report.command.max_orientation_rad * 180.0 / M_PI
             << " deg exceeds the requested "
             << thresholds.maximum_orientation_error_rad * 180.0 / M_PI << " deg";
        warn(text.str());
    }
    if (!report.modelled_collision_valid)
        warn("modelled obstacle clearance is negative or unavailable; "
             "the obstacle result is diagnostic and does not veto emission");
    if (report.minimum_clearance_m < kClearanceWarningM) {
        text.str("");
        text << "modelled clearance " << report.minimum_clearance_m * 1000.0
             << " mm is under " << kClearanceWarningM * 1000.0 << " mm";
        warn(text.str());
    }
    if (report.max_velocity_limit_ratio > 1.0) {
        text.str("");
        text << "peak joint velocity is "
             << (report.max_velocity_limit_ratio - 1.0) * 100.0
             << "% over its limit after time scaling; the controller's "
                "saturation will absorb it";
        warn(text.str());
    }
    // The acceleration limit is a heuristic (velocity limit reached in
    // ~0.5 s, PlanSolver), not a hardware table, so an overage warns but
    // never rejects on its own.
    if (report.max_acceleration_limit_ratio > 1.0) {
        text.str("");
        text << "peak joint acceleration is "
             << (report.max_acceleration_limit_ratio - 1.0) * 100.0
             << "% over the heuristic bound";
        warn(text.str());
    }

    report.verdict = report.warnings.empty() ? PlanVerdict::kAccept
                                             : PlanVerdict::kWarning;
    return report.verdict;
}

std::string PathValidationReport::Summary() const {
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(3);

    out << "== path validation ==\n";

    // The gate first, then the two errors that explain it. e_command is
    // what the arm does; the other two say why.
    out << "planning fidelity (traced phase only)\n"
        << "  e_command       (desired vs final dense timed view) max "
        << command.max_position_m * 1000.0 << " mm, rms "
        << command.rms_position_m * 1000.0 << " mm, p95 "
        << command.p95_position_m * 1000.0 << " mm, rot "
        << command.max_orientation_rad * 180.0 / M_PI << " deg   <- GATED\n"
        << "  e_planner       (desired vs GP-dense)               max "
        << planner.max_position_m * 1000.0 << " mm\n"
        << "  e_reconstruction(GP-dense vs dense timed view)       max "
        << reconstruction.max_position_m * 1000.0
        << " mm  (subsample + Hermite transport loss)\n"
        << "  worst point at t = " << command.worst_time_s << " s, path parameter "
        << command.worst_path_parameter << "\n";

    if (command_circle.applicable)
        out << "  circle decomposition: out-of-plane "
            << command_circle.max_plane_error_m * 1000.0 << " mm, radial "
            << command_circle.max_radial_error_m * 1000.0 << " mm\n";

    out << "collision (MODELLED geometry only)\n"
        << "  modelled_collision_valid: " << YesNo(modelled_collision_valid)
        << ", minimum clearance " << minimum_clearance_m * 1000.0 << " mm at t = "
        << minimum_clearance_time_s << " s\n"
        << "  SDF contained: " << sdf_contents << "\n";

    out << "dynamics\n"
        << "  max |qdot| " << max_joint_velocity_rad_s * 180.0 / M_PI
        << " deg/s (worst joint at " << max_velocity_limit_ratio
        << " of its limit), max |qddot| "
        << max_joint_acceleration_rad_s2 * 180.0 / M_PI
        << " deg/s^2 (at " << max_acceleration_limit_ratio
        << " of the heuristic bound)\n"
        << "  joint-limit margin " << minimum_joint_limit_margin_rad * 180.0 / M_PI
        << " deg, ok: " << YesNo(joint_limits_valid) << "\n";

    out << "start state\n"
        << "  first command vs measured " << start_configuration_error_rad * 180.0 / M_PI
        << " deg (splice guard), initial |qdot| "
        << start_velocity_rad_s * 180.0 / M_PI << " deg/s, finite: "
        << YesNo(all_finite) << ", ok: " << YesNo(start_state_valid) << "\n";

    out << "verdict: " << VerdictName(verdict) << "\n";
    for (const std::string& warning : warnings)
        out << "    warning: " << warning << "\n";
    if (verdict != PlanVerdict::kReject)
        out << "  (every MODELLED check was evaluated. The SDF does not "
               "contain the wearer, the torso or the other arm, so this is "
               "not a statement that the motion is safe near a person.)\n";
    return out.str();
}
