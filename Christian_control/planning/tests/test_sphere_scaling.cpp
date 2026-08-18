// Guard on the collision-sphere <-> DH coupling in generateArmSpheres
// (optimisation/GenerateArmModel.cpp): along-link sphere
// stations must scale with the generated d values (signed ratio against the
// authored lengths), lateral girth offsets must not, and the sign-flip /
// collapsed-link guards must throw instead of silently rescaling. Without
// this test no ctest asserts a single sphere coordinate — the FK-agreement
// tests are blind to the collision model, so an inverted ratio or a broken
// guard would reach the planner unnoticed.
#undef NDEBUG

#include <cassert>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "PlannerModel.h"

namespace {

// True when the guard path throws (any std::exception).
template <typename Callable>
bool Throws(Callable&& callable) {
    try {
        callable();
    } catch (const std::exception&) {
        return true;
    }
    return false;
}

} // namespace

int main(int argc, char** argv) {
    assert(argc == 2 && "usage: test_sphere_scaling <generated dh_params_tool.yaml>");
    const PlannerModel baseline = LoadPlannerModel(
        argv[1], /*has_tool=*/true, Eigen::Isometry3d::Identity());
    const size_t sphere_count = baseline.arm_model->nr_body_spheres();
    assert(sphere_count > 0);

    // --- Scaling: stretch the tool link (d7) by 1.5x. Link-6 spheres' z
    // stations must scale by exactly 1.5; their lateral x/y and every
    // sphere on links 0/2/4 must be untouched.
    constexpr double kStretch = 1.5;
    DHParameters stretched_dh = baseline.dh;
    stretched_dh.d(6) *= kStretch;
    ArmModel factory;
    const auto stretched = factory.createArmModel(baseline.base_pose, stretched_dh);
    assert(stretched->nr_body_spheres() == sphere_count);

    int link6_spheres = 0;
    for (size_t i = 0; i < sphere_count; ++i) {
        const size_t link = baseline.arm_model->sphere_link_id(i);
        const gtsam::Point3& before = baseline.arm_model->sphere_center_wrt_link(i);
        const gtsam::Point3& after = stretched->sphere_center_wrt_link(i);
        if (link == 6) {
            ++link6_spheres;
            assert(std::abs(after.z() - before.z() * kStretch) < 1e-12 &&
                   "link-6 z stations must scale with d7");
            assert(std::abs(after.x() - before.x()) < 1e-12 &&
                   std::abs(after.y() - before.y()) < 1e-12 &&
                   "lateral offsets must not scale");
        } else {
            assert((after - before).norm() < 1e-12 &&
                   "spheres on other links must be unaffected by d7");
        }
    }
    assert(link6_spheres > 0 && "expected tool-link spheres to exist");

    // --- Guards: a sign flip or a collapsed link must throw, not rescale.
    DHParameters flipped_dh = baseline.dh;
    flipped_dh.d(6) = -flipped_dh.d(6);
    assert(Throws([&] { factory.createArmModel(baseline.base_pose, flipped_dh); }) &&
           "sign flip must be refused");

    DHParameters collapsed_dh = baseline.dh;
    collapsed_dh.d(0) = -0.01;
    assert(Throws([&] { factory.createArmModel(baseline.base_pose, collapsed_dh); }) &&
           "collapsed link must be refused");

    std::printf("sphere scaling: %zu spheres, %d on the tool link — "
                "scaling, lateral, and guard checks passed\n",
                sphere_count, link6_spheres);
    return 0;
}
