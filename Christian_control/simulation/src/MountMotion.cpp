#include "MountMotion.h"

#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>

namespace
{
    constexpr double kTwoPi = 6.283185307179586;

    bool UsesTranslation(MountMotionKind kind)
    {
        return kind == MountMotionKind::kTranslation ||
               kind == MountMotionKind::kCombined;
    }

    bool UsesRotation(MountMotionKind kind)
    {
        return kind == MountMotionKind::kRotation ||
               kind == MountMotionKind::kCombined;
    }

    // The shared scalar sinusoid and its exact derivative:
    //   s(t)    = A sin(2 pi f t + phi)
    //   sdot(t) = A (2 pi f) cos(2 pi f t + phi)
    // Units follow the amplitude: metres for the translation channel,
    // radians for the rotation channel.
    struct Sinusoid {
        double value;
        double rate;
    };

    Sinusoid EvaluateSinusoid(double amplitude, double frequency_hz,
                              double phase_rad, double time_s)
    {
        const double angular_frequency_rad_s = kTwoPi * frequency_hz;
        const double angle_rad = angular_frequency_rad_s * time_s + phase_rad;
        return {amplitude * std::sin(angle_rad),
                amplitude * angular_frequency_rad_s * std::cos(angle_rad)};
    }

    void RequireFinite(bool finite, const char* field)
    {
        if (!finite)
            throw std::invalid_argument(std::string("MountMotionConfig: "
                                                    "non-finite ") +
                                        field);
    }

    // An active channel that asks for a nonzero displacement must be able
    // to produce one: a zero-length axis has no direction to move along,
    // and a zero frequency never leaves sin(phase).
    void RequireReachable(double amplitude, double axis_norm,
                          double frequency_hz, const char* channel)
    {
        if (amplitude == 0.0)
            return;
        if (axis_norm == 0.0)
            throw std::invalid_argument(
                std::string("MountMotionConfig: nonzero ") + channel +
                " amplitude with a zero-length axis");
        if (frequency_hz == 0.0)
            throw std::invalid_argument(
                std::string("MountMotionConfig: nonzero ") + channel +
                " amplitude with a zero frequency");
    }
} // namespace

void ValidateMountMotionConfig(const MountMotionConfig& config)
{
    RequireFinite(std::isfinite(config.translation_amplitude_m),
                  "translation_amplitude_m");
    RequireFinite(std::isfinite(config.rotation_amplitude_rad),
                  "rotation_amplitude_rad");
    RequireFinite(std::isfinite(config.frequency_hz), "frequency_hz");
    RequireFinite(std::isfinite(config.phase_rad), "phase_rad");
    RequireFinite(config.translation_axis_world.allFinite(),
                  "translation_axis_world");
    RequireFinite(config.rotation_axis_world.allFinite(),
                  "rotation_axis_world");

    if (config.frequency_hz < 0.0)
        throw std::invalid_argument(
            "MountMotionConfig: frequency_hz must not be negative");

    if (UsesTranslation(config.kind))
        RequireReachable(config.translation_amplitude_m,
                         config.translation_axis_world.norm(),
                         config.frequency_hz, "translation");
    if (UsesRotation(config.kind))
        RequireReachable(config.rotation_amplitude_rad,
                         config.rotation_axis_world.norm(), config.frequency_hz,
                         "rotation");
}

MountTruth SampleMountMotion(const MountMotionConfig& config, double time_s,
                             const CartesianPose& world_T_mount_at_zero)
{
    ValidateMountMotionConfig(config);
    if (!std::isfinite(time_s))
        throw std::invalid_argument("SampleMountMotion: non-finite time_s");
    if (!world_T_mount_at_zero.position_m.allFinite() ||
        !world_T_mount_at_zero.rotation.allFinite())
        throw std::invalid_argument(
            "SampleMountMotion: non-finite world_T_mount_at_zero");

    // Start from the anchor: the zero-displacement pose, with a zero
    // twist that is its exact derivative. kStatic, and any channel the
    // kind leaves inactive, is finished right here — the anchor is
    // returned bit for bit, so a stationary Mount cannot jitter the
    // plant's mocap row between ticks.
    MountTruth truth;
    truth.world_T_mount = world_T_mount_at_zero;

    // The amplitude guards are not defensive padding: they are what makes
    // a zero amplitude with a zero axis (a config that describes no
    // motion at all, and is legal) return the anchor instead of a 0/0
    // NaN out of normalized().
    if (UsesTranslation(config.kind) && config.translation_amplitude_m != 0.0) {
        // p_W_M(t) = p0 + a s_t(t);  v_W_M(t) = a sdot_t(t).
        const Eigen::Vector3d axis_world =
            config.translation_axis_world.normalized();
        const Sinusoid s =
            EvaluateSinusoid(config.translation_amplitude_m, config.frequency_hz,
                             config.phase_rad, time_s);
        truth.world_T_mount.position_m =
            world_T_mount_at_zero.position_m + axis_world * s.value;
        truth.world_V_mount.linear_m_s = axis_world * s.rate;
    }

    if (UsesRotation(config.kind) && config.rotation_amplitude_rad != 0.0) {
        // R_W_M(t) = exp(s_r(t) [b]x) R0, a rotation about the world axis
        // b through the Mount's own origin, so it PREmultiplies the
        // anchor. Differentiating gives Rdot = sdot_r [b]x R, i.e. the
        // spatial angular velocity omega = b sdot_r exactly — no
        // small-angle approximation, because b is fixed in W.
        const Eigen::Vector3d axis_world =
            config.rotation_axis_world.normalized();
        const Sinusoid s = EvaluateSinusoid(config.rotation_amplitude_rad,
                                            config.frequency_hz,
                                            config.phase_rad, time_s);
        truth.world_T_mount.rotation =
            Eigen::AngleAxisd(s.value, axis_world).toRotationMatrix() *
            world_T_mount_at_zero.rotation;
        truth.world_V_mount.angular_rad_s = axis_world * s.rate;
    }

    return truth;
}
