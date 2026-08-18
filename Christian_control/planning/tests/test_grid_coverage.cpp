// Test assertions must never no-op under a Release (NDEBUG) configure.
#undef NDEBUG

// The SDF grid must contain everywhere EITHER arm can put a collision sphere.
//
// gpmp2 returns zero obstacle cost — "all clear" — for any query outside the
// grid volume, silently (ObstacleCost.h catches SDFQueryOutOfRange and returns
// zero). So a grid smaller than the reachable collision envelope is not a
// smaller map, it is a blind spot in obstacle avoidance.
//
// Since the planner moved into the shared `mount` frame there is ONE grid for
// both arms, so this unions the right (tool) and left (flange) envelopes. The
// two models are genuinely different — the flange chain omits the gripper
// spheres — so neither can be mirrored from the other.
//
// This test is also the measuring instrument: when it fails it prints a
// paste-ready constant block for WorldSdf.h, so the extents are derived rather
// than guessed. If it fails, the arms' reach changed (URDF, mounting, sphere
// layout) or the grid shrank. Re-derive the extents. Do NOT loosen the margin.

#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "PlannerModel.h"
#include "WorldSdf.h"
#include "utils.h"

namespace {

struct Envelope {
    Eigen::Vector3d min = Eigen::Vector3d::Constant(1e9);
    Eigen::Vector3d max = Eigen::Vector3d::Constant(-1e9);

    void Add(const Eigen::Vector3d& lo, const Eigen::Vector3d& hi) {
        min = min.cwiseMin(lo);
        max = max.cwiseMax(hi);
    }
    void Merge(const Envelope& other) { Add(other.min, other.max); }
};

// Every collision sphere at `q`, inflated by its own radius, folded into `out`.
void Accumulate(const PlannerModel& model, const gtsam::Vector& q, Envelope& out) {
    const gtsam::Matrix centers = model.arm_model->sphereCentersMat(q);
    for (size_t s = 0; s < model.arm_model->nr_body_spheres(); ++s) {
        const double radius = model.arm_model->sphere_radius(s);
        const Eigen::Vector3d center = centers.col(s);
        out.Add((center.array() - radius).matrix(), (center.array() + radius).matrix());
    }
}

Envelope MeasureEnvelope(const PlannerModel& model, const JointLimits& pos_limits) {
    // Continuous joints get one full turn; bounded joints their real range.
    const auto lower = [&](int j) {
        return pos_limits.lower(j) < -1e10 ? -M_PI : pos_limits.lower(j);
    };
    const auto upper = [&](int j) {
        return pos_limits.upper(j) > 1e10 ? M_PI : pos_limits.upper(j);
    };

    Envelope envelope;

    // Deterministic extremal sweep FIRST. Uniform random sampling in seven
    // dimensions systematically under-covers the fully extended poses that
    // actually set the envelope, because reaching one needs several joints
    // near an end of range simultaneously. Bounded joints at {lower, 0, upper}
    // crossed with continuous joints at {-pi, -pi/2, 0, +pi/2} pins them, and
    // 27 * 256 = 6912 configurations costs nothing.
    const int bounded[3] = {1, 3, 5};        // joints 2, 4, 6 (0-based)
    const int continuous[4] = {0, 2, 4, 6};  // joints 1, 3, 5, 7
    const double turns[4] = {-M_PI, -M_PI / 2.0, 0.0, M_PI / 2.0};
    for (int combo = 0; combo < 27 * 256; ++combo) {
        gtsam::Vector q = gtsam::Vector::Zero(7);
        int rest = combo;
        for (const int j : bounded) {
            const int pick = rest % 3;
            rest /= 3;
            q[j] = pick == 0 ? lower(j) : (pick == 1 ? 0.0 : upper(j));
        }
        for (const int j : continuous) {
            q[j] = turns[rest % 4];
            rest /= 4;
        }
        Accumulate(model, q, envelope);
    }

    // Fixed seed: the sampled envelope, and so the pass/fail, is identical on
    // every run. The margin below absorbs what sampling still understates.
    std::mt19937 generator(1u);
    constexpr long kSamples = 200000;
    for (long sample = 0; sample < kSamples; ++sample) {
        gtsam::Vector q(7);
        for (int j = 0; j < 7; ++j) {
            std::uniform_real_distribution<double> pick(lower(j), upper(j));
            q[j] = pick(generator);
        }
        Accumulate(model, q, envelope);
    }
    return envelope;
}

void PrintRow(const char* label, const Eigen::Vector3d& lo, const Eigen::Vector3d& hi) {
    std::printf("%-8s x [%6.3f, %6.3f]  y [%6.3f, %6.3f]  z [%6.3f, %6.3f]\n", label,
                lo.x(), hi.x(), lo.y(), hi.y(), lo.z(), hi.z());
}

}  // namespace

int main(int argc, char** argv) {
    assert(argc == 4 &&
           "usage: test_grid_coverage <dh_tool.yaml> <dh_flange.yaml> <joint_limits.yaml>");
    const Eigen::Isometry3d world_T_mount =
        Eigen::Translation3d(1.0, 2.0, 3.0) *
        Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ());
    const PlannerModel right =
        LoadPlannerModel(argv[1], /*has_tool=*/true, world_T_mount);
    const PlannerModel left =
        LoadPlannerModel(argv[2], /*has_tool=*/false, world_T_mount);
    const auto [pos_limits, vel_limits] = createJointLimits(argv[3]);
    (void)vel_limits;
    const GridBounds bounds = WorldGridBounds(WorldGridGeometry(world_T_mount));

    const Envelope right_envelope = MeasureEnvelope(right, pos_limits);
    const Envelope left_envelope = MeasureEnvelope(left, pos_limits);
    Envelope both = right_envelope;
    both.Merge(left_envelope);

    PrintRow("grid", bounds.min_m, bounds.max_m);
    PrintRow("right", right_envelope.min, right_envelope.max);
    PrintRow("left", left_envelope.min, left_envelope.max);
    PrintRow("union", both.min, both.max);

    const Eigen::Vector3d margin_low = both.min - bounds.min_m;
    const Eigen::Vector3d margin_high = bounds.max_m - both.max;
    PrintRow("margin", margin_low, margin_high);

    // 0.05 m: covers what sampling still understates the true envelope by,
    // while failing loudly long before a real hole opens.
    constexpr double kMarginM = 0.05;

    // The measuring instrument. On failure, print the constants that WOULD
    // satisfy the margin, so WorldSdf.h is updated from a measurement rather
    // than from arithmetic done by hand. n uses (max - origin)/cell + 1
    // because the last queryable sample is index n-1 (see WorldGridBounds).
    if ((margin_low.array() < kMarginM).any() || (margin_high.array() < kMarginM).any()) {
        std::printf("\n--- paste into WorldSdf.h ---\n");
        const char* axis[3] = {"X", "Y", "Z"};
        int n[3];
        for (int a = 0; a < 3; ++a) {
            const double origin =
                std::floor((both.min[a] - kMarginM) / kGridCellM) * kGridCellM;
            n[a] = static_cast<int>(
                       std::ceil((both.max[a] + kMarginM - origin) / kGridCellM)) + 1;
            std::printf("inline constexpr double kGridOrigin%sM = %.2f;\n", axis[a], origin);
        }
        for (int a = 0; a < 3; ++a)
            std::printf("inline constexpr int kGridN%c = %d;\n", "xyz"[a], n[a]);
        std::printf("--- end ---\n\n");
    }

    // The two arms are mirrored about the mount XZ plane, so their envelopes
    // must sit on opposite sides in y. Without this, loading the same model
    // twice would produce a half-sized "union" that still passed.
    assert(std::abs(right_envelope.min.y() - left_envelope.min.y()) > 0.05 &&
           "right and left envelopes are suspiciously alike — same model loaded twice?");

    assert((margin_low.array() >= kMarginM).all() &&
           "arm envelope reaches the grid's lower faces — grow the grid");
    assert((margin_high.array() >= kMarginM).all() &&
           "arm envelope reaches the grid's upper faces — grow the grid");
    return 0;
}
