#include "PathValidationReport.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace {

const char* YesNo(bool value) { return value ? "yes" : "NO"; }

}  // namespace

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
        << " deg/s, max |qddot| " << max_joint_acceleration_rad_s2 * 180.0 / M_PI
        << " deg/s^2, limits ok: " << YesNo(dynamic_limits_valid) << "\n"
        << "  joint-limit margin " << minimum_joint_limit_margin_rad * 180.0 / M_PI
        << " deg, ok: " << YesNo(joint_limits_valid) << "\n";

    out << "start state\n"
        << "  first command vs measured " << start_configuration_error_rad * 180.0 / M_PI
        << " deg (splice guard), initial |qdot| "
        << start_velocity_rad_s * 180.0 / M_PI << " deg/s, finite: "
        << YesNo(all_finite) << ", ok: " << YesNo(start_state_valid) << "\n";

    out << "verdict\n"
        << "  optimiser_converged      " << YesNo(optimiser_converged) << "\n"
        << "  task_fidelity_valid      " << YesNo(task_fidelity_valid) << "\n"
        << "  modelled_collision_valid " << YesNo(modelled_collision_valid) << "\n"
        << "  joint_limits_valid       " << YesNo(joint_limits_valid) << "\n"
        << "  dynamic_limits_valid     " << YesNo(dynamic_limits_valid) << "\n"
        << "  start_state_valid        " << YesNo(start_state_valid) << "\n"
        << "  hardware_execution_allowed " << YesNo(hardware_execution_allowed)
        << "\n";
    if (hardware_execution_allowed)
        out << "  (every MODELLED check passed. The SDF does not contain the "
               "wearer, the torso or the other arm, so this is not a statement "
               "that the motion is safe near a person.)\n";
    return out.str();
}
