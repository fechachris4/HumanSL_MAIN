//
// Controller: task-space control of the end-effector.
// Control law ported from the MuJoCo reactive controller: PD on pose +
// twist error -> damped-least-squares inverse kinematics -> null-space
// joint centering -> integrated position setpoints.
//

#include "Controller.h"

#include "Config.h"
#include "Measure.h"
#include "Kinematics.h"
#include "Motion.h"    // JointVector, ServoingModeGuard, send_positions
#include "Record.h"    // ControlTraceSample

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/explog.hpp>   // log3

#include <KDetailedException.h>

using Vector6 = Eigen::Matrix<double, 6, 1>;
using Vector7 = Eigen::Matrix<double, 7, 1>;
using Matrix67 = Eigen::Matrix<double, 6, 7>;

ControllerState controller_error(Dynamics& dynamics,
                                 k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{
    JointReading joints = measure_configuration(base_cyclic, dynamics);
    Pose ee = forward_kinematics(dynamics, joints.q_pin);

    ControllerState state;
    state.current = ee.position;
    state.target = Eigen::Vector3d(config::kTargetPosition[0],
                                   config::kTargetPosition[1],
                                   config::kTargetPosition[2]);
    state.error = state.target - state.current;
    return state;
}

void report_position_error(Dynamics& dynamics,
                           k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                           std::ostream& out)
{
    // One read, one FK: position and orientation from the same instant.
    JointReading joints = measure_configuration(base_cyclic, dynamics);
    Pose ee = forward_kinematics(dynamics, joints.q_pin);

    // Rotation matrix -> fixed-axis roll-pitch-yaw (the Config.h convention,
    // R = Rz*Ry*Rx), in degrees.
    const Eigen::Matrix3d& R = ee.rotation;
    constexpr double kRadToDeg = 180.0 / M_PI;
    double roll  = std::atan2(R(2, 1), R(2, 2)) * kRadToDeg;
    double pitch = std::asin(std::clamp(-R(2, 0), -1.0, 1.0)) * kRadToDeg;
    double yaw   = std::atan2(R(1, 0), R(0, 0)) * kRadToDeg;

    ControllerState s;
    s.current = ee.position;
    s.target = Eigen::Vector3d(config::kTargetPosition[0],
                               config::kTargetPosition[1],
                               config::kTargetPosition[2]);
    s.error = s.target - s.current;

    // .norm() = Euclidean length of the vector: straight-line distance to go.
    out << "Controller target (base):  x=" << s.target.x()
        << "  y=" << s.target.y() << "  z=" << s.target.z() << "  [m]\n"
        << "EE position (FK):          x=" << s.current.x()
        << "  y=" << s.current.y() << "  z=" << s.current.z() << "  [m]\n"
        << "Position error:            x=" << s.error.x()
        << "  y=" << s.error.y() << "  z=" << s.error.z()
        << "  |e|=" << s.error.norm() << "  [m]\n"
        << "EE orientation (rpy deg):  roll=" << roll << "  pitch=" << pitch
        << "  yaw=" << yaw
        << "   (copy into kTargetOrientationRPYDeg to hold it)\n";
}

// --- reactive servo -----------------------------------------------------

namespace {

constexpr int NUM_JOINTS = 7;
constexpr double kDegToRad = M_PI / 180.0;

// Feedback -> joint state in radians. Positions arrive wrapped to [0, 360)
// deg, so each is shifted by whole turns next to its (continuous) command.
void read_measured(const k_api::BaseCyclic::Feedback& fb,
                   const JointVector& cmd_deg, Vector7& q, Vector7& qdot)
{
    for (int i = 0; i < NUM_JOINTS; ++i) {
        double meas_deg = cmd_deg[i] +
            std::remainder(fb.actuators(i).position() - cmd_deg[i], 360.0);
        q(i) = meas_deg * kDegToRad;
        qdot(i) = fb.actuators(i).velocity() * kDegToRad;
    }
}

// The control law: pose + twist errors -> joint rates (rad/s). Fills the
// trace's error/twist/qdot_raw fields as it goes.
Vector7 compute_qdot(Dynamics& dynamics, pinocchio::FrameIndex frame_id,
                     const Eigen::Vector3d& p_ref, const Eigen::Matrix3d& R_ref,
                     const Vector7& q, const Vector7& qdot,
                     ControlTraceSample& s)
{
    Eigen::VectorXd q_pin = dynamics.convertJointAnglesToConfig(q);
    Pose ee = forward_kinematics(dynamics, q_pin);

    // World-frame Jacobian at the EE (rows: linear xyz, then angular xyz).
    Matrix67 J = Matrix67::Zero();
    pinocchio::computeFrameJacobian(dynamics.model_, dynamics.data_, q_pin,
                                    frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);

    // Errors: e = reference - actual. e_rot is the axis-angle of the world
    // rotation taking the actual orientation to the reference. The target is
    // static and the base fixed, so the twist error is just minus the EE twist.
    Eigen::Vector3d e_pos = p_ref - ee.position;
    Eigen::Vector3d e_rot = pinocchio::log3(R_ref * ee.rotation.transpose());
    Vector6 twist = J * qdot;
    Eigen::Vector3d e_v = -twist.head<3>();
    Eigen::Vector3d e_w = -twist.tail<3>();

    // PD law: errors -> commanded task twist.
    Vector6 v;
    v.head<3>() = config::kKpPos * e_pos + config::kKdPos * e_v;
    v.tail<3>() = config::kKpRot * e_rot + config::kKdRot * e_w;

    // Damped least squares: bounded joint rates through singularities.
    Eigen::Matrix<double, 6, 6> JJt =
        J * J.transpose() + config::kDlsDamping * config::kDlsDamping *
                                Eigen::Matrix<double, 6, 6>::Identity();
    Vector7 qdot_task = J.transpose() * JJt.ldlt().solve(v);

    // Optional null-space centering on the limited joints 2/4/6 (symmetric
    // ranges, mid = 0; the continuous joints have no range to center in).
    // Uses the exact pseudoinverse projector — the damped one leaks into the
    // task. Gated by Config.h so the plain task-space law can be compared
    // against task + centering; off also skips the pseudoinverse entirely.
    Vector7 qdot_cmd = qdot_task;
    if (config::kUseNullspaceCentering) {
        Vector7 qdot_null = Vector7::Zero();
        for (int i : {1, 3, 5})
            qdot_null(i) = -config::kNullGain * std::remainder(q(i), 2.0 * M_PI);
        Eigen::Matrix<double, 7, 6> J_pinv =
            J.completeOrthogonalDecomposition().pseudoInverse();
        qdot_cmd += (Eigen::Matrix<double, 7, 7>::Identity() - J_pinv * J)
                    * qdot_null;
    }

    for (int k = 0; k < 3; ++k) {
        s.e_pos[k] = e_pos(k);
        s.e_rot[k] = e_rot(k);
        s.e_v[k] = e_v(k);
        s.e_w[k] = e_w(k);
    }
    for (int k = 0; k < 6; ++k) s.v_cmd[k] = v(k);
    for (int i = 0; i < NUM_JOINTS; ++i) s.qdot_raw[i] = qdot_cmd(i);
    return qdot_cmd;
}

bool abort_control(const char* kind, const char* what,
                   std::ostream* log, ControlTraceSample& s)
{
    s.refresh_ok = false;
    if (log)
        write_control_trace_row(*log, s);
    std::cout << "reactive control failed: " << kind << ": " << what << "\n";
    return false;
}

} // namespace

bool run_reactive_control(k_api::Base::BaseClient* base,
                          k_api::BaseCyclic::BaseCyclicClient* base_cyclic,
                          Dynamics& dynamics,
                          const std::atomic<bool>& stop,
                          std::ostream* log)
{
    constexpr auto period = std::chrono::milliseconds(1);
    const double dt = std::chrono::duration<double>(period).count();
    const double qdot_limit = config::kReactiveSpeedLimitDegS * kDegToRad;

    const Eigen::Vector3d p_ref(config::kTargetPosition[0],
                                config::kTargetPosition[1],
                                config::kTargetPosition[2]);
    // Config rpy (fixed axes, deg) -> quaternion -> rotation matrix.
    // R = Rz(yaw) * Ry(pitch) * Rx(roll); the quaternion is the internal
    // orientation representation, the matrix is what the control math uses.
    const Eigen::Quaterniond q_ref =
        Eigen::AngleAxisd(config::kTargetOrientationRPYDeg[2] * kDegToRad,
                          Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(config::kTargetOrientationRPYDeg[1] * kDegToRad,
                          Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(config::kTargetOrientationRPYDeg[0] * kDegToRad,
                          Eigen::Vector3d::UnitX());
    const Eigen::Matrix3d R_ref = q_ref.normalized().toRotationMatrix();
    const auto frame_id = dynamics.model_.getFrameId("EndEffector_Link");

    // Say which variant this run is, so its trace CSV can be attributed.
    std::cout << "null-space joint centering: "
              << (config::kUseNullspaceCentering ? "ON" : "off") << "\n";

    if (log)
        write_control_trace_header(*log);
    ControlTraceSample sample;   // last cycle's row, reused for the error row

    try {
        // From here until the guard is destroyed, WE are the controller.
        // Read the joints AFTER the mode switch (see Motion.cpp), and start
        // the commands at the measurement so taking over causes no jump.
        ServoingModeGuard servoing(base);
        k_api::BaseCyclic::Feedback feedback = base_cyclic->RefreshFeedback();

        JointVector cmd_deg;
        for (int i = 0; i < NUM_JOINTS; ++i)
            cmd_deg[i] = feedback.actuators(i).position();
        // So the freeze-at-measured frame is safe even if Ctrl+C fires
        // before the first cycle completes.
        sample.measured_deg = cmd_deg;

        k_api::BaseCyclic::Command command;
        for (int i = 0; i < NUM_JOINTS; ++i)
            command.add_actuators();
        feedback = send_positions(base_cyclic, command, cmd_deg);  // hold

        using clock = std::chrono::steady_clock;
        const auto t_start = clock::now();
        auto t_prev = t_start;
        auto next_cycle = t_start;
        auto next_heartbeat = t_start;
        Vector7 q, qdot;

        while (!stop) {
            next_cycle += period;

            // Sense (from last cycle's feedback) and compute joint rates.
            read_measured(feedback, cmd_deg, q, qdot);
            Vector7 qdot_cmd =
                compute_qdot(dynamics, frame_id, p_ref, R_ref, q, qdot, sample);

            // Clamp, integrate the setpoints, clamp the lead over the
            // measurement (anti-windup), send.
            for (int i = 0; i < NUM_JOINTS; ++i) {
                double clipped = std::clamp(qdot_cmd(i), -qdot_limit, qdot_limit);
                sample.speed_clipped[i] = clipped != qdot_cmd(i);
                sample.qdot_cmd[i] = clipped;

                // std::clamp returns the value itself when in range, so the
                // comparison is exact — no false "clamped" flags.
                double cmd_rad = cmd_deg[i] * kDegToRad + clipped * dt;
                double clamped_cmd = std::clamp(cmd_rad,
                                                q(i) - config::kCtrlLeadRad,
                                                q(i) + config::kCtrlLeadRad);
                sample.lead_clamped[i] = clamped_cmd != cmd_rad;
                cmd_deg[i] = clamped_cmd / kDegToRad;
            }
            feedback = send_positions(base_cyclic, command, cmd_deg);

            auto t_now = clock::now();
            sample.t_s = std::chrono::duration<double>(t_now - t_start).count();
            sample.dt_s = std::chrono::duration<double>(t_now - t_prev).count();
            t_prev = t_now;
            for (int i = 0; i < NUM_JOINTS; ++i) {
                const auto& a = feedback.actuators(i);
                sample.commanded_deg[i] = cmd_deg[i];
                sample.measured_deg[i] = cmd_deg[i] +
                    std::remainder(a.position() - cmd_deg[i], 360.0);
                sample.velocity_deg_s[i] = a.velocity();
                sample.fault_bank[i] = a.fault_bank_a();
            }
            sample.arm_state = feedback.base().active_state();
            sample.base_fault_bank = feedback.base().fault_bank_a();
            sample.refresh_ok = true;
            if (log)
                write_control_trace_row(*log, sample);

            bool fault = sample.base_fault_bank != 0;
            for (int i = 0; i < NUM_JOINTS; ++i)
                fault = fault || sample.fault_bank[i] != 0;
            if (fault) {
                // Freeze where the arm actually is before giving up.
                send_positions(base_cyclic, command, sample.measured_deg);
                if (sample.base_fault_bank != 0)
                    std::cout << "reactive control failed: base fault bank "
                              << sample.base_fault_bank << " at t="
                              << sample.t_s << " s\n";
                for (int i = 0; i < NUM_JOINTS; ++i)
                    if (sample.fault_bank[i] != 0)
                        std::cout << "reactive control failed: actuator "
                                  << (i + 1) << " fault bank "
                                  << sample.fault_bank[i] << " at t="
                                  << sample.t_s << " s\n";
                return false;
            }

            if (t_now >= next_heartbeat) {
                // From now, not += 1 s: no catch-up burst after a stall.
                next_heartbeat = t_now + std::chrono::seconds(1);
                Eigen::Vector3d e_pos(sample.e_pos.data());
                Eigen::Vector3d e_rot(sample.e_rot.data());
                std::cout << "t=" << sample.t_s << " s  |e_pos|="
                          << e_pos.norm() * 1000.0 << " mm  |e_rot|="
                          << e_rot.norm() / kDegToRad << " deg\n";
            }

            auto now = clock::now();
            if (next_cycle > now)
                std::this_thread::sleep_until(next_cycle);
            else
                next_cycle = now;   // fell behind: don't burst to catch up
        }

        // Ctrl+C: freeze at the measured position, not at a setpoint the
        // arm is still straining toward.
        send_positions(base_cyclic, command, sample.measured_deg);
        std::cout << "reactive control stopped by user (Ctrl+C)\n";
        return true;
    }
    catch (k_api::KDetailedException& ex) {
        return abort_control("Kortex API error", ex.what(), log, sample);
    }
    catch (std::runtime_error& ex) {
        return abort_control("communication error", ex.what(), log, sample);
    }
}
