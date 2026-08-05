// Test assertions must never no-op under a Release (NDEBUG) configure.
#undef NDEBUG

#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>
#include "PlannerModel.h"
#include "AnalyticalKinematics.h"  // basic_control/tools — the controller's URDF chain

int main(int argc, char** argv) {
    assert(argc == 2 && "usage: test_planner_model <dh_params_tool.yaml>");
    const PlannerModel model = LoadPlannerModel(argv[1]);
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-2.0, 2.0);
    double worst_mm = 0.0;
    for (int trial = 0; trial < 200; ++trial) {
        Eigen::Matrix<double, 7, 1> q = Eigen::Matrix<double, 7, 1>::Zero();
        if (trial > 0)
            for (int j = 0; j < 7; ++j) q[j] = dist(rng);
        const Eigen::Vector3d p_dh = ToolPositionInBaseLink(model, q);
        const Eigen::Vector3d p_urdf = AnalyticalForwardKinematics(q).translation();
        worst_mm = std::max(worst_mm, (p_dh - p_urdf).norm() * 1000.0);
    }
    std::printf("worst tool-position disagreement: %.3f mm\n", worst_mm);
    assert(worst_mm < 1.0 && "DH and URDF chains must agree under 1 mm");
    return 0;
}
