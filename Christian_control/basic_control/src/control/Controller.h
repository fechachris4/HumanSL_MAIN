//
// Controller-facing robot state. Eigen only — no Kortex, no vendor types,
// ever.
//
// HARD RULE (Christian, 2026-07-22): a field belongs in RobotState only if
// the Runner can fill it validly EVERY cycle from arm feedback alone.
// External sensing (Vicon, anything future) never goes here — it reaches a
// controller by store injection: a mutex-protected latest-value store with
// a timestamp, written by its own thread, passed by const reference to
// only the controllers that need it (TargetStore is the pattern to copy).
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_CONTROLLER_H
#define HUMANSL_MASTERS_PROJECT_2025_CONTROLLER_H

#include <Eigen/Dense>

struct RobotState {
    Eigen::Matrix<double, 7, 1> q_rad;      // measured joint positions
    Eigen::Matrix<double, 7, 1> qdot_rad_s; // measured joint velocities
    double t_s = 0.0;                       // time since takeover
};

#endif // HUMANSL_MASTERS_PROJECT_2025_CONTROLLER_H
