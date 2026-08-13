//
// WorldHold — the Vicon trust boundary of the world-frame hold (slice 2
// spec §2). Everything here is the hardware logic the simulation never
// needed, because the sim's world pose is perfect every step: engage
// gating, staleness freeze, re-anchor on recovery, ramp-in, and the
// one-way divergence latch. The frame MATHS lives in Frames.h (ported
// from the sim); this file owns only time and trust.
//
// State machine (spec §2):
//
//   kInactive ──fresh+valid──▶ kEngaged (anchor X_des^W := current EE
//        ▲                        │  ▲     world pose; ramp restarts)
//        │                 stale/invalid │ recovered ⇒ RE-ANCHOR + ramp
//        │                        ▼  │     (hold_reanchor_count++)
//        │                     kFrozen
//        │                        │
//   trajectory takes over    ‖e_p^W‖ > max_error (Engaged or Frozen)
//   (Reset(); re-engage          ▼
//    later re-anchors)       kLatchedOff — one-way for the rest of the
//                            run: prevents an engage/diverge/engage
//                            oscillation when a frame sign is wrong.
//                            The arm falls back to the joint hold —
//                            degradation to today's behaviour, no stop.
//
// While kFrozen nothing snaps: the slot's zero-order hold freezes
// world_T_B by itself, so the commanded reference simply stops moving
// with the world — the arm behaves as today's base-frame hold until the
// stream recovers. Recovery re-anchors at the CURRENT pose: zero error at
// the instant of re-engage, so there is no step toward wherever the old
// target ended up (spec: never command a jump toward a point the wearer
// may now occupy). The absolute point is lost across a blackout and the
// re-anchor counter says so honestly.
//
// Pure logic: no clocks (the caller passes age and time), no SDK, no
// Kortex, fixed-size state. Unit-tested in tests/test_world_hold.cpp.
//

#pragma once

#include <cmath>

#include "Frames.h"      // world_frames::FramePose
#include "ReactiveLaw.h" // UnitRamp — the ramp the law already uses

enum class WorldHoldState {
    kInactive = 0,  // no anchor; waiting for a fresh+valid sample
    kEngaged = 1,   // holding X_des^W in the world
    kFrozen = 2,    // sample stale/invalid: reference frozen, no world update
    kLatchedOff = 3 // diverged once: off until the process restarts
};

// One update's inputs, all computed by the caller (Runner/Targets): the
// hold never reads sensors. `sample_fresh` means age <= the configured
// threshold AND the needed segment valid AND a sample has ever arrived.
struct WorldHoldInput {
    bool sample_fresh = false;
    world_frames::FramePose ee_pose_world; // current EE pose, world frame
    double t_s = 0.0;                      // loop time (RobotState::t_s)
};

// One update's outputs. `provides_target` is true only in kEngaged and
// kFrozen — the world hold is then the pose reference source. The target
// is ramped: at ramp < 1 it is interpolated from the (re-)anchor pose
// toward X_des^W, so a wrong sign appears as slow drift at low authority,
// not a step at full gain (spec §1 ramp).
struct WorldHoldOutput {
    bool provides_target = false;
    double error_rot_rad = std::numeric_limits<double>::quiet_NaN();
    world_frames::FramePose target_world; // ramped target, world frame
    world_frames::FramePose anchor_world; // X_des^W unramped (telemetry)
    WorldHoldState state = WorldHoldState::kInactive;
    double ramp = 0.0;              // 0..1, UnitRamp since last (re-)engage
    double error_norm_m = std::numeric_limits<double>::quiet_NaN();
    int reanchor_count = 0;
};

class WorldHold
{
public:
    // Thresholds come from Config.h at the call site; passed here so the
    // logic is testable with plain numbers.
    WorldHold(double ramp_s, double max_error_m, double max_rot_error_rad,
              double reanchor_after_s)
        : ramp_s_(ramp_s), max_error_m_(max_error_m),
          max_rot_error_rad_(max_rot_error_rad),
          reanchor_after_s_(reanchor_after_s)
    {
    }

    // A trajectory (or any higher-priority reference) has taken over:
    // drop the anchor. The next fresh cycle re-engages with a fresh
    // anchor and a fresh ramp. Deliberately NOT clearing the latch —
    // divergence is evidence about the frame chain, not about the
    // trajectory that interrupted us.
    void Reset()
    {
        if (state_ == WorldHoldState::kEngaged ||
            state_ == WorldHoldState::kFrozen)
            state_ = WorldHoldState::kInactive;
    }

    WorldHoldOutput Update(const WorldHoldInput& in)
    {
        switch (state_) {
        case WorldHoldState::kLatchedOff:
            break; // one-way; nothing to do

        case WorldHoldState::kInactive:
            if (in.sample_fresh)
                Engage(in, /*count_reanchor=*/false);
            break;

        case WorldHoldState::kEngaged:
            if (!in.sample_fresh) {
                state_ = WorldHoldState::kFrozen;
                freeze_t_s_ = in.t_s;
            }
            break;

        case WorldHoldState::kFrozen:
            if (in.sample_fresh) {
                // Re-anchor churn guard (review finding, 2026-08-13): a
                // marginal stream flickering around the freshness
                // threshold would otherwise re-anchor on every blip, each
                // time restarting the 2 s ramp — the hold would sit at
                // low authority indefinitely and quietly leak its
                // absolute anchor. A BRIEF blackout keeps the anchor (the
                // base cannot have moved far; the divergence latch still
                // bounds the worst case); only an extended one re-anchors.
                if (in.t_s - freeze_t_s_ <= reanchor_after_s_)
                    state_ = WorldHoldState::kEngaged; // resume, same anchor
                else
                    Engage(in, /*count_reanchor=*/true); // long gap: re-anchor
            }
            break;
        }

        WorldHoldOutput out;
        out.state = state_;
        out.reanchor_count = reanchor_count_;
        const bool holding = state_ == WorldHoldState::kEngaged ||
                             state_ == WorldHoldState::kFrozen;
        if (!holding)
            return out;

        // Divergence latch, tested on the UNRAMPED error: the distance
        // between the anchored world target and where the end-effector
        // actually is. Non-finite inputs (a NaN pose sneaking through)
        // count as divergence: a hold that cannot evaluate its own error
        // has no business commanding an arm.
        out.anchor_world = anchor_;
        out.error_norm_m =
            (anchor_.position_m - in.ee_pose_world.position_m).norm();
        out.error_rot_rad =
            RotationLog(anchor_.rotation *
                        in.ee_pose_world.rotation.transpose())
                .norm();
        // Both error channels latch: a wrong ROTATION sign can reorient
        // the arm while the position error stays small (an orbit about
        // the EE point), so rotation divergence must latch on its own.
        //
        // Authority scaling (review finding, 2026-08-13): during ramp-in
        // the law only fights r·e, so the latch tests r·e too. Otherwise
        // an engage during fast base motion — correct signs, low
        // authority, error transiently large — would latch the hold off
        // for the whole run. At r·e the latch stays proportional to what
        // the arm can actually be commanded to do wrong; a wrong sign
        // still latches as the ramp brings authority (and drift) up.
        const double ramp_now = UnitRamp(in.t_s - engage_t_s_, ramp_s_);
        if (!std::isfinite(out.error_norm_m) ||
            !std::isfinite(out.error_rot_rad) ||
            ramp_now * out.error_norm_m > max_error_m_ ||
            ramp_now * out.error_rot_rad > max_rot_error_rad_) {
            state_ = WorldHoldState::kLatchedOff;
            out.state = state_;
            return out; // provides_target stays false from this cycle on
        }

        // Ramped target — the AUTHORITY ramp: interpolate from the CURRENT
        // pose toward the anchor, so the effective error the law sees is
        // r·e. (Interpolating from the engage pose would be vacuous: both
        // endpoints are constants that coincide at engage. The spec's §1
        // formula meant this; corrected here and noted there.) A wrong
        // frame sign therefore produces slow drift at authority r, and the
        // latch above fires long before full gain.
        out.ramp = ramp_now;
        const double r = out.ramp;
        out.target_world.position_m =
            (1.0 - r) * in.ee_pose_world.position_m + r * anchor_.position_m;
        const Eigen::Quaterniond q_current(in.ee_pose_world.rotation);
        const Eigen::Quaterniond q_anchor(anchor_.rotation);
        out.target_world.rotation =
            q_current.slerp(r, q_anchor).toRotationMatrix();
        out.provides_target = true;
        return out;
    }

private:
    void Engage(const WorldHoldInput& in, bool count_reanchor)
    {
        anchor_ = in.ee_pose_world; // X_des^W: hold HERE, in the room
        engage_t_s_ = in.t_s;
        state_ = WorldHoldState::kEngaged;
        if (count_reanchor)
            ++reanchor_count_;
    }

    double ramp_s_;
    double max_error_m_;
    double max_rot_error_rad_;
    double reanchor_after_s_;
    WorldHoldState state_ = WorldHoldState::kInactive;
    world_frames::FramePose anchor_; // X_des^W
    double engage_t_s_ = 0.0;
    double freeze_t_s_ = 0.0;
    int reanchor_count_ = 0;
};
