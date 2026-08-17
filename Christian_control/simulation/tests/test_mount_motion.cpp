//
// MountMotion tests (Plan 03 Task 1): the analytic scripted Mount
// trajectory that gives the plant its pose and the ideal sensor its
// twist from one call.
//
// The physical question: does SampleMountMotion describe a Mount that
// translates and rotates about its OWN origin in Vicon-world axes, with a
// twist that is the exact derivative of the pose it hands out?
//
// INDEPENDENT ORACLES (contract section 12 — a test must not compute its
// expectation through the helper under test):
//   - rotations are rebuilt here with an explicit Rodrigues formula
//     R = I + sin(th) K + (1 - cos(th)) K^2, never Eigen::AngleAxisd;
//   - the angular velocity is checked against a central finite difference
//     of the rotation log, vec(log(R(t+h) R(t-h)^T)) / 2h, with the log
//     written out here from the trace and the antisymmetric part;
//   - the linear velocity is checked against a central finite difference
//     of the position, and separately against the closed-form derivative
//     A (2 pi f) cos(.) evaluated in the test;
//   - closed-form values at t = 0, T/4 and T/2 are written as plain
//     numbers (0, A, 0 displacement; peak, 0, -peak rate).
//
// PRE-REGISTERED MUTATION CHECKS (could wrong physics still pass?):
//   - spatial vs body angular velocity: the anchor rotation R0 is
//     deliberately NOT the identity and its axis is NOT the motion axis,
//     so omega = b sdot and R^T b sdot differ by ~0.06 rad/s — six orders
//     of magnitude above the 1e-8 finite-difference tolerance;
//   - transform order: pre-multiplying exp(th [b]x) R0 (rotation about a
//     world axis) and post-multiplying R0 exp(th [b]x) (about a body
//     axis) are separated by an explicit check, which an identity anchor
//     would have hidden completely;
//   - reference point: with a pure rotation and an anchor position far
//     from the world origin, the Mount origin must not move at all — a
//     rotation applied about the world origin would sweep it by |p0| th;
//   - missing normalization: a 3x-long axis must produce exactly the same
//     motion, not three times the displacement;
//   - metres vs millimetres: the quarter-period displacement must equal
//     the configured amplitude exactly.
//
// Evidence class: unit test (pure arithmetic; no simulator, no robot).
//

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "MountMotion.h"

namespace
{
    int failures = 0;

    void Check(bool ok, const std::string& what)
    {
        if (!ok) {
            std::printf("FAIL: %s\n", what.c_str());
            ++failures;
        }
    }

    constexpr double kTwoPi = 6.283185307179586;

    // Finite-difference step, s. Truncation error is (h^2/6)|third
    // derivative| ~ 4e-11 at the default amplitudes, round-off ~5e-13, so
    // 1e-8 leaves a wide margin while staying far below every mutation
    // above (all ~1e-2 or larger).
    constexpr double kFdStep = 1e-4;
    constexpr double kFdTol = 1e-8;

    // --- independent oracles -------------------------------------------

    Eigen::Matrix3d Skew(const Eigen::Vector3d& v)
    {
        Eigen::Matrix3d k;
        k << 0.0, -v.z(), v.y(),
             v.z(), 0.0, -v.x(),
            -v.y(), v.x(), 0.0;
        return k;
    }

    // Rodrigues, written out rather than delegated to Eigen::AngleAxisd
    // (the formula the implementation uses through Eigen).
    Eigen::Matrix3d RodriguesRotation(const Eigen::Vector3d& unit_axis,
                                      double angle_rad)
    {
        const Eigen::Matrix3d k = Skew(unit_axis);
        return Eigen::Matrix3d::Identity() + std::sin(angle_rad) * k +
               (1.0 - std::cos(angle_rad)) * (k * k);
    }

    // vec(log(R)) for |theta| < pi: w = 2 sin(theta) * axis from the
    // antisymmetric part, theta from atan2(sin, cos).
    Eigen::Vector3d RotationLog(const Eigen::Matrix3d& r)
    {
        const Eigen::Vector3d w(r(2, 1) - r(1, 2), r(0, 2) - r(2, 0),
                                r(1, 0) - r(0, 1));
        const double sin_theta = 0.5 * w.norm();
        const double cos_theta = 0.5 * (r.trace() - 1.0);
        const double theta = std::atan2(sin_theta, cos_theta);
        if (sin_theta < 1e-12)
            return 0.5 * w; // theta -> 0: log(R) -> 0.5 w
        return (theta / (2.0 * sin_theta)) * w;
    }

    // --- fixtures -------------------------------------------------------

    // A deliberately awkward anchor: far from the world origin (so a
    // rotation applied about the wrong point is visible) and already
    // rotated about an axis unrelated to the motion axes (so pre- and
    // post-multiplication, and spatial vs body angular velocity, differ).
    CartesianPose Anchor()
    {
        CartesianPose pose;
        pose.position_m = Eigen::Vector3d(0.31, -0.22, 1.07);
        pose.rotation =
            Eigen::AngleAxisd(0.7, Eigen::Vector3d(1.0, 2.0, -3.0).normalized())
                .toRotationMatrix();
        return pose;
    }

    // Non-axis-aligned, non-unit axes: normalization and cross-axis
    // coupling are exercised on every case that uses these.
    Eigen::Vector3d TranslationAxis() { return Eigen::Vector3d(1.0, -2.0, 0.5); }
    Eigen::Vector3d RotationAxis() { return Eigen::Vector3d(-0.3, 0.4, 1.2); }

    MountMotionConfig CombinedConfig()
    {
        MountMotionConfig config;
        config.kind = MountMotionKind::kCombined;
        config.translation_axis_world = TranslationAxis();
        config.translation_amplitude_m = 0.05;
        config.rotation_axis_world = RotationAxis();
        config.rotation_amplitude_rad = 0.1;
        config.frequency_hz = 0.1;
        config.phase_rad = 0.0;
        return config;
    }

    bool Throws(const MountMotionConfig& config)
    {
        try {
            SampleMountMotion(config, 0.37, Anchor());
        } catch (const std::invalid_argument&) {
            return true;
        }
        return false;
    }

    // --- checks ---------------------------------------------------------

    // The documented defaults are the feasibility argument in
    // MountMotion.h; if they drift, that arithmetic silently stops being
    // true, so they are pinned here.
    void TestDocumentedDefaults()
    {
        const MountMotionConfig config;
        Check(config.kind == MountMotionKind::kStatic,
              "defaults: a config with nothing set describes no motion");
        Check(config.frequency_hz == 0.1,
              "defaults: 0.1 Hz (the conservative shared frequency)");
        Check(config.translation_amplitude_m == 0.05,
              "defaults: 0.05 m translation amplitude");
        Check(config.rotation_amplitude_rad == 0.1,
              "defaults: 0.1 rad rotation amplitude");

        // Peak speeds the feasibility note quotes: A*2*pi*f.
        const double peak_v = 0.05 * kTwoPi * 0.1;
        const double peak_w = 0.1 * kTwoPi * 0.1;
        Check(std::abs(peak_v - 0.0314159265) < 1e-9,
              "defaults: peak Mount speed 0.0314 m/s as documented");
        Check(std::abs(peak_w - 0.0628318531) < 1e-9,
              "defaults: peak Mount angular speed 0.0628 rad/s as documented");
    }

    void TestStaticIsExactlyTheAnchor()
    {
        MountMotionConfig config = CombinedConfig();
        config.kind = MountMotionKind::kStatic; // amplitudes must be ignored
        const CartesianPose anchor = Anchor();

        for (const double t : {0.0, 0.002, 1.37, 250.0}) {
            const MountTruth truth = SampleMountMotion(config, t, anchor);
            // Bit-identical, not within a tolerance: a stationary Mount
            // must not jitter the plant's mocap row between ticks.
            Check(std::memcmp(truth.world_T_mount.position_m.data(),
                              anchor.position_m.data(), 3 * sizeof(double)) == 0,
                  "static: position is the anchor, bit for bit");
            Check(std::memcmp(truth.world_T_mount.rotation.data(),
                              anchor.rotation.data(), 9 * sizeof(double)) == 0,
                  "static: rotation is the anchor, bit for bit");
            Check(truth.world_V_mount.linear_m_s == Eigen::Vector3d::Zero() &&
                      truth.world_V_mount.angular_rad_s ==
                          Eigen::Vector3d::Zero(),
                  "static: the derivative of a constant pose is exactly zero");
        }
    }

    // t = 0, T/4, T/2 with phase 0, against numbers written out by hand.
    void TestClosedFormLandmarks()
    {
        const CartesianPose anchor = Anchor();
        const MountMotionConfig config = CombinedConfig();
        const double period_s = 1.0 / config.frequency_hz;
        const Eigen::Vector3d a = TranslationAxis().normalized();
        const Eigen::Vector3d b = RotationAxis().normalized();
        const double peak_v = config.translation_amplitude_m * kTwoPi *
                              config.frequency_hz;
        const double peak_w = config.rotation_amplitude_rad * kTwoPi *
                              config.frequency_hz;

        const MountTruth at_zero = SampleMountMotion(config, 0.0, anchor);
        Check(std::memcmp(at_zero.world_T_mount.position_m.data(),
                          anchor.position_m.data(), 3 * sizeof(double)) == 0,
              "t=0, phase 0: the pose is the anchor, bit for bit");
        Check((at_zero.world_T_mount.rotation - anchor.rotation).norm() < 1e-15,
              "t=0, phase 0: the rotation is the anchor");
        Check((at_zero.world_V_mount.linear_m_s - a * peak_v).norm() < 1e-15,
              "t=0, phase 0: linear velocity is at its positive peak");
        Check((at_zero.world_V_mount.angular_rad_s - b * peak_w).norm() < 1e-15,
              "t=0, phase 0: angular velocity is at its positive peak");

        const MountTruth quarter =
            SampleMountMotion(config, 0.25 * period_s, anchor);
        const Eigen::Vector3d displacement =
            quarter.world_T_mount.position_m - anchor.position_m;
        // Metres, not millimetres: the peak displacement IS the amplitude.
        Check(std::abs(displacement.norm() - config.translation_amplitude_m) <
                  1e-15,
              "T/4: displaced by exactly the configured amplitude, in metres");
        Check((displacement - a * config.translation_amplitude_m).norm() < 1e-15,
              "T/4: displaced along the (normalized) translation axis");
        const Eigen::Matrix3d expected_quarter =
            RodriguesRotation(b, config.rotation_amplitude_rad) *
            anchor.rotation;
        Check((quarter.world_T_mount.rotation - expected_quarter).norm() < 1e-14,
              "T/4: rotated by exactly the configured angle about the world "
              "axis (independent Rodrigues oracle)");
        Check(quarter.world_V_mount.linear_m_s.norm() < 1e-16 &&
                  quarter.world_V_mount.angular_rad_s.norm() < 1e-16,
              "T/4: at peak displacement the twist is zero");

        const MountTruth half =
            SampleMountMotion(config, 0.5 * period_s, anchor);
        Check((half.world_T_mount.position_m - anchor.position_m).norm() < 1e-16,
              "T/2: back at the anchor position");
        Check((half.world_T_mount.rotation - anchor.rotation).norm() < 1e-15,
              "T/2: back at the anchor rotation");
        Check((half.world_V_mount.linear_m_s + a * peak_v).norm() < 1e-15,
              "T/2: linear velocity is at its negative peak");
        Check((half.world_V_mount.angular_rad_s + b * peak_w).norm() < 1e-15,
              "T/2: angular velocity is at its negative peak");
    }

    // With a nonzero phase the run starts displaced: documented
    // convention, checked against A sin(phi) computed here.
    void TestPhaseOffsetStartsDisplaced()
    {
        const CartesianPose anchor = Anchor();
        MountMotionConfig config = CombinedConfig();
        config.phase_rad = 0.7;

        const MountTruth truth = SampleMountMotion(config, 0.0, anchor);
        const Eigen::Vector3d a = TranslationAxis().normalized();
        const Eigen::Vector3d b = RotationAxis().normalized();
        const double s_t = config.translation_amplitude_m * std::sin(0.7);
        const double s_r = config.rotation_amplitude_rad * std::sin(0.7);

        Check((truth.world_T_mount.position_m - (anchor.position_m + a * s_t))
                      .norm() < 1e-15,
              "phase: t=0 starts displaced by A sin(phi), not at the anchor");
        Check((truth.world_T_mount.rotation -
               RodriguesRotation(b, s_r) * anchor.rotation)
                      .norm() < 1e-14,
              "phase: t=0 starts rotated by A_r sin(phi)");
        Check((truth.world_T_mount.position_m - anchor.position_m).norm() >
                  1e-3,
              "phase: the displacement is large enough to be a real check");
    }

    void TestAxisNormalizationAndZeroAmplitude()
    {
        const CartesianPose anchor = Anchor();
        MountMotionConfig unit = CombinedConfig();
        unit.translation_axis_world = TranslationAxis().normalized();
        unit.rotation_axis_world = RotationAxis().normalized();
        MountMotionConfig scaled = CombinedConfig();
        scaled.translation_axis_world = 3.0 * TranslationAxis();
        scaled.rotation_axis_world = 7.5 * RotationAxis();

        const double t = 0.63;
        const MountTruth from_unit = SampleMountMotion(unit, t, anchor);
        const MountTruth from_scaled = SampleMountMotion(scaled, t, anchor);
        Check((from_unit.world_T_mount.position_m -
               from_scaled.world_T_mount.position_m)
                      .norm() < 1e-15 &&
                  (from_unit.world_T_mount.rotation -
                   from_scaled.world_T_mount.rotation)
                          .norm() < 1e-15 &&
                  (from_unit.world_V_mount.linear_m_s -
                   from_scaled.world_V_mount.linear_m_s)
                          .norm() < 1e-15 &&
                  (from_unit.world_V_mount.angular_rad_s -
                   from_scaled.world_V_mount.angular_rad_s)
                          .norm() < 1e-15,
              "axis length carries no meaning: a 3x axis gives the same "
              "motion (an unnormalized axis would give 3x the displacement)");

        // Zero amplitude with a zero axis is meaningless but harmless: no
        // throw, and above all no 0/0 NaN reaching the plant.
        MountMotionConfig none = CombinedConfig();
        none.translation_amplitude_m = 0.0;
        none.rotation_amplitude_rad = 0.0;
        none.translation_axis_world = Eigen::Vector3d::Zero();
        none.rotation_axis_world = Eigen::Vector3d::Zero();
        const MountTruth still = SampleMountMotion(none, t, anchor);
        Check(std::memcmp(still.world_T_mount.position_m.data(),
                          anchor.position_m.data(), 3 * sizeof(double)) == 0 &&
                  std::memcmp(still.world_T_mount.rotation.data(),
                              anchor.rotation.data(), 9 * sizeof(double)) == 0,
              "zero amplitude with a zero axis holds the anchor exactly");
        Check(still.world_V_mount.linear_m_s.allFinite() &&
                  still.world_V_mount.angular_rad_s.allFinite() &&
                  still.world_V_mount.linear_m_s.norm() == 0.0 &&
                  still.world_V_mount.angular_rad_s.norm() == 0.0,
              "zero amplitude with a zero axis gives an exact zero twist, "
              "never a 0/0 NaN");
    }

    void TestRejections()
    {
        const double nan_value = std::nan("");
        const double inf_value = std::numeric_limits<double>::infinity();

        MountMotionConfig bad = CombinedConfig();
        bad.translation_amplitude_m = nan_value;
        Check(Throws(bad), "rejects a non-finite translation amplitude");

        bad = CombinedConfig();
        bad.rotation_amplitude_rad = inf_value;
        Check(Throws(bad), "rejects a non-finite rotation amplitude");

        bad = CombinedConfig();
        bad.frequency_hz = nan_value;
        Check(Throws(bad), "rejects a non-finite frequency");

        bad = CombinedConfig();
        bad.phase_rad = inf_value;
        Check(Throws(bad), "rejects a non-finite phase");

        bad = CombinedConfig();
        bad.translation_axis_world.y() = nan_value;
        Check(Throws(bad), "rejects a non-finite translation axis");

        bad = CombinedConfig();
        bad.rotation_axis_world.z() = inf_value;
        Check(Throws(bad), "rejects a non-finite rotation axis");

        bad = CombinedConfig();
        bad.frequency_hz = -0.1;
        Check(Throws(bad), "rejects a negative frequency");

        bad = CombinedConfig();
        bad.frequency_hz = 0.0;
        Check(Throws(bad),
              "rejects zero frequency with a nonzero amplitude (a "
              "displacement the motion can never reach)");

        bad = CombinedConfig();
        bad.translation_axis_world = Eigen::Vector3d::Zero();
        Check(Throws(bad),
              "rejects a zero translation axis with a nonzero amplitude");

        bad = CombinedConfig();
        bad.rotation_axis_world = Eigen::Vector3d::Zero();
        Check(Throws(bad),
              "rejects a zero rotation axis with a nonzero amplitude");

        // Inactive channels describe nothing, so their axes are not
        // consulted; the same config becomes invalid once the kind
        // activates the channel (checked immediately above).
        MountMotionConfig translation_only = CombinedConfig();
        translation_only.kind = MountMotionKind::kTranslation;
        translation_only.rotation_axis_world = Eigen::Vector3d::Zero();
        Check(!Throws(translation_only),
              "a zero rotation axis is ignored when the kind is translation");

        MountMotionConfig rotation_only = CombinedConfig();
        rotation_only.kind = MountMotionKind::kRotation;
        rotation_only.translation_axis_world = Eigen::Vector3d::Zero();
        Check(!Throws(rotation_only),
              "a zero translation axis is ignored when the kind is rotation");

        MountMotionConfig still = CombinedConfig();
        still.kind = MountMotionKind::kStatic;
        still.translation_axis_world = Eigen::Vector3d::Zero();
        still.rotation_axis_world = Eigen::Vector3d::Zero();
        still.frequency_hz = 0.0;
        Check(!Throws(still),
              "a static Mount consults neither axis nor frequency");

        // A non-finite time or anchor would silently poison the plant's
        // mocap row instead of failing where the mistake was made.
        bool threw_on_time = false;
        try {
            SampleMountMotion(CombinedConfig(), nan_value, Anchor());
        } catch (const std::invalid_argument&) {
            threw_on_time = true;
        }
        Check(threw_on_time, "rejects a non-finite time");

        bool threw_on_anchor = false;
        try {
            CartesianPose anchor = Anchor();
            anchor.position_m.x() = inf_value;
            SampleMountMotion(CombinedConfig(), 0.5, anchor);
        } catch (const std::invalid_argument&) {
            threw_on_anchor = true;
        }
        Check(threw_on_anchor, "rejects a non-finite anchor pose");
    }

    // Rotation alone must leave the Mount ORIGIN exactly where it was,
    // and translation alone must leave the orientation untouched.
    void TestChannelsAreIndependentAndCombinedIsTheirComposition()
    {
        const CartesianPose anchor = Anchor();
        const double t = 3.1;

        MountMotionConfig translation = CombinedConfig();
        translation.kind = MountMotionKind::kTranslation;
        MountMotionConfig rotation = CombinedConfig();
        rotation.kind = MountMotionKind::kRotation;
        const MountMotionConfig combined = CombinedConfig();

        const MountTruth only_t = SampleMountMotion(translation, t, anchor);
        const MountTruth only_r = SampleMountMotion(rotation, t, anchor);
        const MountTruth both = SampleMountMotion(combined, t, anchor);

        Check(std::memcmp(only_t.world_T_mount.rotation.data(),
                          anchor.rotation.data(), 9 * sizeof(double)) == 0 &&
                  only_t.world_V_mount.angular_rad_s.norm() == 0.0,
              "translation alone leaves the orientation and angular velocity "
              "exactly untouched");
        // The anchor sits 1.14 m from the world origin, so a rotation
        // applied about the world origin instead of the Mount origin
        // would move this point by ~0.1 m.
        Check(std::memcmp(only_r.world_T_mount.position_m.data(),
                          anchor.position_m.data(), 3 * sizeof(double)) == 0 &&
                  only_r.world_V_mount.linear_m_s.norm() == 0.0,
              "rotation alone leaves the Mount origin exactly fixed: the "
              "rotation is about the Mount's own origin, not the world's");

        Check((both.world_T_mount.position_m - only_t.world_T_mount.position_m)
                      .norm() == 0.0 &&
                  (both.world_T_mount.rotation - only_r.world_T_mount.rotation)
                          .norm() == 0.0,
              "combined is exactly the two single-channel motions, with no "
              "cross-coupling");
        Check((both.world_V_mount.linear_m_s - only_t.world_V_mount.linear_m_s)
                      .norm() == 0.0 &&
                  (both.world_V_mount.angular_rad_s -
                   only_r.world_V_mount.angular_rad_s)
                          .norm() == 0.0,
              "combined twist is exactly the two single-channel twists");
    }

    // The main event: is the reported twist the derivative of the
    // reported pose? Central differences of the pose itself, never of an
    // internal quantity.
    void TestTwistIsTheDerivativeOfThePose()
    {
        const CartesianPose anchor = Anchor();
        double worst_linear = 0.0;
        double worst_angular = 0.0;

        for (const MountMotionKind kind :
             {MountMotionKind::kTranslation, MountMotionKind::kRotation,
              MountMotionKind::kCombined}) {
            MountMotionConfig config = CombinedConfig();
            config.kind = kind;
            config.phase_rad = 0.9; // arbitrary, non-landmark phase

            for (const double t : {0.0, 0.37, 1.25, 2.5, 4.13}) {
                const MountTruth here = SampleMountMotion(config, t, anchor);
                const MountTruth ahead =
                    SampleMountMotion(config, t + kFdStep, anchor);
                const MountTruth behind =
                    SampleMountMotion(config, t - kFdStep, anchor);

                const Eigen::Vector3d fd_linear =
                    (ahead.world_T_mount.position_m -
                     behind.world_T_mount.position_m) /
                    (2.0 * kFdStep);
                const Eigen::Vector3d fd_angular =
                    RotationLog(ahead.world_T_mount.rotation *
                                behind.world_T_mount.rotation.transpose()) /
                    (2.0 * kFdStep);

                worst_linear = std::max(
                    worst_linear,
                    (fd_linear - here.world_V_mount.linear_m_s).norm());
                worst_angular = std::max(
                    worst_angular,
                    (fd_angular - here.world_V_mount.angular_rad_s).norm());
            }
        }

        Check(worst_linear < kFdTol,
              "linear velocity equals the central difference of the reported "
              "position");
        Check(worst_angular < kFdTol,
              "angular velocity equals the central difference of the rotation "
              "log (spatial convention)");
        std::printf("finite-difference residuals: linear %.3e m/s, angular "
                    "%.3e rad/s (tolerance %.0e)\n",
                    worst_linear, worst_angular, kFdTol);
    }

    // Would the check above still pass if the conventions were wrong?
    // Each mutation is quantified here, so the tolerance is known to be
    // far tighter than the error it must catch.
    void TestMutationsAreDistinguishable()
    {
        const CartesianPose anchor = Anchor();
        MountMotionConfig config = CombinedConfig();
        config.phase_rad = 0.9;
        const double t = 1.25;

        const MountTruth truth = SampleMountMotion(config, t, anchor);
        const Eigen::Vector3d b = RotationAxis().normalized();
        const double angle =
            config.rotation_amplitude_rad *
            std::sin(kTwoPi * config.frequency_hz * t + config.phase_rad);
        const double rate = config.rotation_amplitude_rad * kTwoPi *
                            config.frequency_hz *
                            std::cos(kTwoPi * config.frequency_hz * t +
                                     config.phase_rad);

        // Body-frame angular velocity instead of spatial.
        const Eigen::Vector3d body_omega =
            truth.world_T_mount.rotation.transpose() * (b * rate);
        const double spatial_vs_body =
            (truth.world_V_mount.angular_rad_s - body_omega).norm();
        Check(spatial_vs_body > 1e4 * kFdTol,
              "the finite-difference check can tell a spatial angular "
              "velocity from a body-frame one");

        // Post-multiplied rotation (about a body axis) instead of pre.
        const Eigen::Matrix3d post =
            anchor.rotation * RodriguesRotation(b, angle);
        const double pre_vs_post = (truth.world_T_mount.rotation - post).norm();
        Check(pre_vs_post > 1e-3,
              "the anchor rotation is nontrivial enough to separate a "
              "world-axis rotation from a body-axis one");

        // Rotation applied about the world origin instead of the Mount
        // origin (the classic lever-arm mistake).
        const Eigen::Vector3d orbited =
            RodriguesRotation(b, angle) * anchor.position_m;
        Check((truth.world_T_mount.position_m - orbited).norm() > 1e-3,
              "the anchor is far enough from the world origin to separate a "
              "Mount-origin rotation from an orbit about the world origin");

        std::printf("mutation separations: spatial-vs-body %.3e rad/s, "
                    "pre-vs-post %.3e, origin-vs-orbit %.3e m\n",
                    spatial_vs_body, pre_vs_post,
                    (truth.world_T_mount.position_m - orbited).norm());
    }

    void TestDeterminism()
    {
        const CartesianPose anchor = Anchor();
        const MountMotionConfig config = CombinedConfig();
        const MountTruth first = SampleMountMotion(config, 7.77, anchor);
        const MountTruth second = SampleMountMotion(config, 7.77, anchor);
        // Field by field rather than over the whole struct: padding bytes
        // are not part of the value.
        const bool identical =
            std::memcmp(first.world_T_mount.position_m.data(),
                        second.world_T_mount.position_m.data(),
                        3 * sizeof(double)) == 0 &&
            std::memcmp(first.world_T_mount.rotation.data(),
                        second.world_T_mount.rotation.data(),
                        9 * sizeof(double)) == 0 &&
            std::memcmp(first.world_V_mount.linear_m_s.data(),
                        second.world_V_mount.linear_m_s.data(),
                        3 * sizeof(double)) == 0 &&
            std::memcmp(first.world_V_mount.angular_rad_s.data(),
                        second.world_V_mount.angular_rad_s.data(),
                        3 * sizeof(double)) == 0;
        Check(identical,
              "the same arguments give a bit-identical sample (no state, no "
              "clock, no accumulated orientation)");
    }
} // namespace

int main()
{
    try {
        TestDocumentedDefaults();
        TestStaticIsExactlyTheAnchor();
        TestClosedFormLandmarks();
        TestPhaseOffsetStartsDisplaced();
        TestAxisNormalizationAndZeroAmplitude();
        TestRejections();
        TestChannelsAreIndependentAndCombinedIsTheirComposition();
        TestTwistIsTheDerivativeOfThePose();
        TestMutationsAreDistinguishable();
        TestDeterminism();
    } catch (const std::exception& failure) {
        std::printf("FAIL: unexpected exception: %s\n", failure.what());
        ++failures;
    }
    if (failures == 0) {
        std::printf("test_mount_motion: all checks passed\n");
        return 0;
    }
    std::printf("test_mount_motion: %d check(s) failed\n", failures);
    return 1;
}
