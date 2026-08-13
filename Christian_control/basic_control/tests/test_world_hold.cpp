//
// WorldHold state-machine tests — the Vicon trust boundary of the world
// hold (spec §2). Pins: engage gating, the authority ramp (effective
// error = r·e), staleness freeze, recovery re-anchor with zero
// instantaneous error, the one-way divergence latch (including on
// non-finite input), trajectory Reset semantics, and that the latch
// survives a Reset. Hardware-free, pure logic, synthetic inputs.
//

#include <cmath>
#include <iostream>
#include <string>

#include "WorldHold.h"

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

    WorldHoldInput At(double x, double t_s, bool fresh = true)
    {
        WorldHoldInput in;
        in.sample_fresh = fresh;
        in.ee_pose_world.position_m = Eigen::Vector3d(x, 0.0, 0.0);
        in.t_s = t_s;
        return in;
    }
} // namespace

int main()
{
    constexpr double kRampS = 2.0;
    constexpr double kMaxErrM = 0.08;
    constexpr double kMaxRotRad = 0.5;
    constexpr double kReanchorAfterS = 0.2;

    // --- engage gating and anchor
    {
        WorldHold hold(kRampS, kMaxErrM, kMaxRotRad, kReanchorAfterS);
        WorldHoldOutput out = hold.Update(At(1.0, 0.0, /*fresh=*/false));
        Check(!out.provides_target && out.state == WorldHoldState::kInactive,
              "no engage while the sample is not fresh");

        out = hold.Update(At(1.0, 0.1));
        Check(out.provides_target && out.state == WorldHoldState::kEngaged,
              "fresh sample engages");
        Check(out.error_norm_m == 0.0, "anchor is the current pose: zero error");
        Check(out.target_world.position_m.x() == 1.0,
              "target equals current pose at engage");
    }

    // --- authority ramp: effective error is r * e
    {
        WorldHold hold(kRampS, kMaxErrM, kMaxRotRad, kReanchorAfterS);
        hold.Update(At(1.0, 0.0)); // engage, anchor at x=1
        // Half the ramp later, the EE has drifted 40 mm from the anchor.
        const WorldHoldOutput out = hold.Update(At(1.04, 1.0));
        Check(std::abs(out.ramp - 0.5) < 1e-12, "ramp is t/ramp_s");
        // target = (1-r)*current + r*anchor = 1.04 - 0.5*0.04 = 1.02
        Check(std::abs(out.target_world.position_m.x() - 1.02) < 1e-12,
              "ramped target scales the effective error to r*e");
        const WorldHoldOutput full = hold.Update(At(1.04, 2.5));
        Check(full.ramp == 1.0 &&
                  std::abs(full.target_world.position_m.x() - 1.0) < 1e-12,
              "after the ramp the target is the anchor itself");
    }

    // --- freeze on staleness, re-anchor on recovery
    {
        WorldHold hold(kRampS, kMaxErrM, kMaxRotRad, kReanchorAfterS);
        hold.Update(At(1.0, 0.0)); // engage at x=1
        WorldHoldOutput out = hold.Update(At(1.01, 0.5, /*fresh=*/false));
        Check(out.provides_target && out.state == WorldHoldState::kFrozen,
              "stale sample freezes but keeps providing the reference");
        // Base moved during the blackout; recovery must NOT chase x=1.
        out = hold.Update(At(1.05, 1.0));
        Check(out.state == WorldHoldState::kEngaged, "recovery re-engages");
        Check(out.error_norm_m == 0.0,
              "recovery re-anchors at the current pose: zero error, no step");
        Check(out.reanchor_count == 1, "the re-anchor is counted");
        Check(out.target_world.position_m.x() == 1.05,
              "the new anchor is where the arm actually is");
    }

    // --- divergence latch: one-way, and Reset does not clear it
    {
        WorldHold hold(kRampS, kMaxErrM, kMaxRotRad, kReanchorAfterS);
        hold.Update(At(1.0, 0.0));
        hold.Update(At(1.0, 3.0)); // ramp complete: authority 1
        WorldHoldOutput out = hold.Update(At(1.05, 3.1));
        Check(out.provides_target, "49 mm error < 80 mm threshold: still held");
        out = hold.Update(At(1.09, 3.2));
        Check(!out.provides_target &&
                  out.state == WorldHoldState::kLatchedOff,
              "90 mm error latches the hold off");
        out = hold.Update(At(1.0, 3.3)); // error would be zero again
        Check(!out.provides_target && out.state == WorldHoldState::kLatchedOff,
              "the latch is one-way even when the error vanishes");
        hold.Reset();
        out = hold.Update(At(1.0, 3.4));
        Check(out.state == WorldHoldState::kLatchedOff,
              "Reset does not clear the latch (divergence is frame evidence)");
    }

    // --- brief blip: anchor kept, no re-anchor, no ramp restart
    {
        WorldHold hold(kRampS, kMaxErrM, kMaxRotRad, kReanchorAfterS);
        hold.Update(At(1.0, 0.0)); // engage, anchor x=1
        hold.Update(At(1.01, 3.0)); // ramp complete
        WorldHoldOutput out = hold.Update(At(1.01, 3.05, /*fresh=*/false));
        Check(out.state == WorldHoldState::kFrozen, "blip freezes");
        out = hold.Update(At(1.02, 3.15)); // fresh again after 100 ms
        Check(out.state == WorldHoldState::kEngaged &&
                  out.reanchor_count == 0,
              "a brief blip resumes with the SAME anchor, no re-anchor");
        Check(std::abs(out.target_world.position_m.x() - 1.0) < 1e-12,
              "after the blip the target is still the original anchor");
        Check(out.ramp == 1.0, "the ramp does not restart on a brief blip");
    }

    // --- authority-scaled latch: large error at low ramp does NOT latch
    {
        WorldHold hold(kRampS, kMaxErrM, kMaxRotRad, kReanchorAfterS);
        hold.Update(At(1.0, 0.0)); // engage
        // 0.1 s in: ramp = 0.05, error 100 mm -> r*e = 5 mm < 80 mm
        WorldHoldOutput out = hold.Update(At(1.10, 0.1));
        Check(out.provides_target &&
                  out.state == WorldHoldState::kEngaged,
              "100 mm at 5% authority does not latch (r*e rule)");
        // Ramp complete, same error -> r*e = 100 mm > 80 mm: latches.
        out = hold.Update(At(1.10, 2.5));
        Check(out.state == WorldHoldState::kLatchedOff,
              "the same error at full authority latches");
    }

    // --- rotation divergence latches even with zero position error
    {
        WorldHold hold(kRampS, kMaxErrM, kMaxRotRad, kReanchorAfterS);
        hold.Update(At(1.0, 0.0)); // anchor with identity rotation
        hold.Update(At(1.0, 3.0)); // ramp complete
        WorldHoldInput in = At(1.0, 3.1); // same position...
        in.ee_pose_world.rotation =        // ...rotated 0.6 rad about z
            Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ())
                .toRotationMatrix();
        const WorldHoldOutput out = hold.Update(in);
        Check(out.state == WorldHoldState::kLatchedOff,
              "0.6 rad rotation error latches off at the 0.5 rad bound");
    }

    // --- non-finite pose counts as divergence
    {
        WorldHold hold(kRampS, kMaxErrM, kMaxRotRad, kReanchorAfterS);
        hold.Update(At(1.0, 0.0));
        WorldHoldInput in = At(1.0, 0.1);
        in.ee_pose_world.position_m.x() =
            std::numeric_limits<double>::quiet_NaN();
        const WorldHoldOutput out = hold.Update(in);
        Check(out.state == WorldHoldState::kLatchedOff,
              "a NaN pose latches off: an unevaluable hold commands nothing");
    }

    // --- trajectory Reset: disengage cleanly, re-engage with fresh anchor
    {
        WorldHold hold(kRampS, kMaxErrM, kMaxRotRad, kReanchorAfterS);
        hold.Update(At(1.0, 0.0));
        hold.Reset(); // a trajectory took over
        WorldHoldOutput out = hold.Update(At(2.0, 5.0));
        Check(out.provides_target && out.error_norm_m == 0.0 &&
                  out.target_world.position_m.x() == 2.0,
              "after a trajectory the hold re-engages anchored at the new pose");
        Check(out.reanchor_count == 0,
              "a post-trajectory engage is a fresh engage, not a re-anchor");
    }

    if (failures == 0) {
        std::cout << "test_world_hold: all checks passed\n";
        return 0;
    }
    std::cout << "test_world_hold: " << failures << " check(s) failed\n";
    return 1;
}
