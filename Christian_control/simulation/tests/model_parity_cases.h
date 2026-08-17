//
// Case data and tolerances for the MuJoCo-vs-Pinocchio parity gate
// (Plan 02 Task 3). Data only — every comparison, finite-difference and
// failure-reporting decision lives in test_model_parity.cpp.
//
// Units: radians (joint angles, orientation errors), metres (positions),
// dimensionless mixed m/rad per rad for Jacobian columns. Quaternions are
// stored in MuJoCo wire order (w, x, y, z) and are unit up to double
// rounding; the test renormalizes once and feeds the SAME normalized
// rotation to both MuJoCo and Pinocchio.
//

#pragma once

namespace parity_cases
{
    // Plan Task 3 Step 5 tolerances. Loosening any of these requires a
    // recorded MuJoCo precision experiment BEFORE results are viewed —
    // a parity failure at 1e-5..1e-3 m is a modelling error, never a
    // tolerance problem.
    constexpr double kPositionToleranceM = 1e-8;
    // Norm of log(R_mujoco * R_pinocchio^T) — the geodesic angle between
    // the two rotations, computed via the quaternion of the relative
    // rotation (robust for small angles, unlike matrix log near identity).
    constexpr double kOrientationToleranceRad = 1e-8;
    constexpr double kJacobianTolerance = 1e-6; // per component

    // Forward-difference step (plan Step 2: 1e-7 rad). Error budget from
    // the accepted analysis packet: truncation ~(h/2)|d2p/dq2| ~ 5e-8,
    // round-off ~eps*|p|/h ~ 2e-9 — both well under the 1e-6 gate.
    // Central differences are the first fallback if a component ever
    // sits near the line; the tolerance is never widened instead.
    constexpr double kFiniteDifferenceStepRad = 1e-7;

    struct Case {
        const char* name;
        double right_q_rad[7]; // joints 1..7, chain order (Kortex order)
        double left_q_rad[7];
        double mount_position_m[3]; // world_T_mount translation
        double mount_quat_wxyz[4];  // world_T_mount rotation (w, x, y, z)
    };

    // Fixed cases. Bounded joints 2/4/6 stay inside the URDF ranges
    // (+/-2.24, +/-2.57, +/-2.09 rad); continuous joints 1/3/5/7
    // deliberately cross +/-pi and +/-2*pi in the wrap case, where
    // MuJoCo's unwrapped hinge angle meets Pinocchio's cos/sin
    // representation.
    inline constexpr Case kFixedCases[] = {
        {
            "home_identity_mount",
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0},
            {1.0, 0.0, 0.0, 0.0},
        },
        {
            // Retracted-style posture; mount translated and yawed 90 deg
            // (quat = [cos(pi/4), 0, 0, sin(pi/4)]) so world axes differ
            // from mount axes. Joints 2 and 4 differ strongly on purpose:
            // the swapped-joint mutation reuses this case.
            "retracted_translated_yawed_mount",
            {0.0, -0.35, 3.1, -2.0, 0.6, -1.0, 1.57},
            {0.4, 0.30, -2.8, 1.9, -0.5, 1.1, -1.57},
            {0.30, -0.20, 0.90},
            {0.7071067811865476, 0.0, 0.0, 0.7071067811865476},
        },
        {
            // Bounded joints near (not at) their software edges,
            // asymmetric between arms; mount rotated 120 deg about
            // (1,1,1)/sqrt(3) (quat = [1/2, 1/2, 1/2, 1/2]).
            "asymmetric_bounded_edges",
            {0.3, 2.2, -1.1, -2.5, 0.7, 2.0, -0.4},
            {-0.6, -2.2, 2.4, 2.5, -1.9, -2.0, 3.0},
            {-0.15, 0.25, 1.10},
            {0.5, 0.5, 0.5, 0.5},
        },
        {
            // Continuous joints beyond +/-pi and +/-2*pi; mount rolled
            // 1 rad about x (quat = [cos(0.5), sin(0.5), 0, 0]).
            "continuous_wrap",
            {6.0, 0.5, -5.5, -1.0, 9.42, 1.2, -7.0},
            {-6.5, -0.4, 5.8, 1.4, -9.0, -0.9, 12.0},
            {0.05, 0.40, 0.75},
            {0.8775825618903728, 0.479425538604203, 0.0, 0.0},
        },
    };

    // Seeded deterministic random cases, generated in the test with
    // std::mt19937(kRandomSeed): continuous joints uniform in
    // [-2*pi, 2*pi]; bounded joints uniform inside the URDF range minus
    // kBoundedMarginRad; mount position uniform in [-0.8, 1.2] m per
    // axis; mount rotation from a normalized 4-vector of uniform
    // [-1, 1] samples.
    constexpr unsigned int kRandomSeed = 20260817u;
    constexpr int kRandomCaseCount = 5;
    constexpr double kBoundedMarginRad = 0.05;
    // URDF bounded ranges (Actuator2/4/6 limits), indexed by joint 0..6;
    // 0 marks a continuous joint. Same numbers ModelContract pins.
    constexpr double kBoundedRangeRad[7] = {0.0, 2.24, 0.0, 2.57,
                                            0.0, 2.09, 0.0};
} // namespace parity_cases
