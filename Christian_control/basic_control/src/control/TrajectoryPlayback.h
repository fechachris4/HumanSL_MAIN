//
// TrajectoryPlayback: executes one pre-validated joint trajectory
// (control/TrajectoryFile.h contract) through the standard velocity
// interface — feed-forward from the reference positions plus a small
// proportional correction toward the reference:
//
//   q̇_d = (q_ref(t+dt) - q_ref(t)) / dt  +  kp * wrap(q_ref(t) - q_meas)
//
// The feed-forward term is the exact difference of reference positions
// over the measured cycle window, so the Runner's integrator telescopes
// to q_cmd = q_meas(0) + (q_ref(t) - q_ref(0)) with zero discretization
// drift; the kp term absorbs the (gated, small) start offset and any
// clamp-induced loss. Playback time advances by the loop's MEASURED dt —
// the same dt the integrator uses — so command and reference stay
// consistent through timing jitter (a stalled cycle plays the trajectory
// slightly slower; it never causes a command jump).
//
// All reference-vs-measured comparisons are wrapped (std::remainder), so
// the file's continuous angles and the arm's [0,360) feedback never
// produce a phantom 360-degree error.
//
// Failure states:
//   - start mismatch at Reset (arm moved between the pre-takeover gate
//     and the takeover): the controller REFUSES — it latches a permanent
//     zero-velocity hold, reports playback_state = 3 with one edge, and
//     never plays. The integrator was seeded from measurement, so the
//     hold is exactly "stay where you are".
//   - after the last sample: feed-forward is zero, the kp term holds the
//     final reference, playback_state = 2 with one edge. The operator
//     stops the run (Ctrl+C); there is no automatic exit.
//
// Pure computation per the Controller contract: no I/O, no allocation
// (the trajectory is owned by value and sampled by index arithmetic).
//

#ifndef HUMANSL_MASTERS_PROJECT_2025_TRAJECTORY_PLAYBACK_H
#define HUMANSL_MASTERS_PROJECT_2025_TRAJECTORY_PLAYBACK_H

#include "control/Controller.h"
#include "control/TrajectoryFile.h"

struct PlaybackSettings {
    double kp_s_inv = 0.0;                // P gain on the wrapped reference error
    double start_mismatch_limit_deg = 0.0; // per-joint refusal threshold at Reset
};

class TrajectoryPlayback : public Controller
{
public:
    TrajectoryPlayback(Trajectory trajectory, const PlaybackSettings& settings);

    void Reset(const RobotState& state) override;

    Eigen::Matrix<double, 7, 1> DesiredVelocity(const RobotState& state,
                                                double dt_s,
                                                ControllerStatus& status) override;

    // For the post-run report: true when Reset refused to play.
    bool refused() const { return refused_; }
    bool completed() const { return done_; }

private:
    const Trajectory trajectory_;
    const PlaybackSettings settings_;
    const double duration_s_;

    double t_play_s_ = 0.0;
    bool refused_ = false;
    bool refusal_reported_ = false;
    bool done_ = false;
};

#endif // HUMANSL_MASTERS_PROJECT_2025_TRAJECTORY_PLAYBACK_H
