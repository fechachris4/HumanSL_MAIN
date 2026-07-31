//
// TrajectoryPlayback: the playback law (design rationale in the header).
//

#include "control/TrajectoryPlayback.h"

#include <cmath>

namespace
{
    constexpr int NUM_JOINTS = 7;
    constexpr double kDegToRad = M_PI / 180.0;
    constexpr double kTwoPi = 2.0 * M_PI;
} // namespace

TrajectoryPlayback::TrajectoryPlayback(Trajectory trajectory,
                                       const PlaybackSettings& settings)
    : trajectory_(std::move(trajectory)), settings_(settings),
      duration_s_(TrajectoryDurationS(trajectory_))
{
}

void TrajectoryPlayback::Reset(const RobotState& state)
{
    t_play_s_ = 0.0;
    done_ = false;
    refusal_reported_ = false;

    // Defense in depth: main already gated the start state on the
    // pre-takeover read, but the arm can be moved between that read and
    // the takeover. Re-check against the seed state; refuse permanently
    // on mismatch — a hold is always safe, a jump toward a stale
    // trajectory start never is.
    refused_ = false;
    for (int j = 0; j < NUM_JOINTS; ++j)
    {
        const double error_rad = std::remainder(
            trajectory_.pos_deg.front()[j] * kDegToRad - state.q_rad[j], kTwoPi);
        if (std::abs(error_rad) >
            settings_.start_mismatch_limit_deg * kDegToRad)
            refused_ = true;
    }
}

Eigen::Matrix<double, 7, 1>
TrajectoryPlayback::DesiredVelocity(const RobotState& state, double dt_s,
                                    ControllerStatus& status)
{
    status.playback_t_s = t_play_s_;

    if (refused_)
    {
        status.playback_state = 3;
        if (!refusal_reported_)
        {
            refusal_reported_ = true;
            status.playback_refused_edge = true;
        }
        status.q_ref_deg = trajectory_.pos_deg.front();
        return Eigen::Matrix<double, 7, 1>::Zero();
    }

    const Eigen::Matrix<double, 7, 1> q_ref_now =
        SamplePositionDeg(trajectory_, t_play_s_);
    const double t_next = std::min(t_play_s_ + dt_s, duration_s_);
    const Eigen::Matrix<double, 7, 1> q_ref_next =
        SamplePositionDeg(trajectory_, t_next);

    Eigen::Matrix<double, 7, 1> qdot_rad_s;
    for (int j = 0; j < NUM_JOINTS; ++j)
    {
        const double ff_rad_s =
            (q_ref_next[j] - q_ref_now[j]) * kDegToRad / dt_s;
        const double error_rad =
            std::remainder(q_ref_now[j] * kDegToRad - state.q_rad[j], kTwoPi);
        qdot_rad_s[j] = ff_rad_s + settings_.kp_s_inv * error_rad;
    }

    t_play_s_ = t_next;
    if (t_play_s_ >= duration_s_ && !done_)
    {
        done_ = true; // latched, so the edge below fires exactly once
        status.playback_done_edge = true;
    }
    status.playback_state = done_ ? 2 : 1;
    // The reference this cycle's integrated command should land on.
    status.q_ref_deg = q_ref_next;
    status.playback_t_s = t_play_s_;
    return qdot_rad_s;
}
