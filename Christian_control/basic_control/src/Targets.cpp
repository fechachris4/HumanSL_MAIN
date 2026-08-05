//
// Targets — implementations for Targets.h.
//

#include <array>
#include <cerrno>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <poll.h>
#include <stdexcept>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include "Config.h"
#include "JointTrajectory.h"
#include "Targets.h"

namespace
{
    constexpr int kInputPollTimeoutMs = 50;
    constexpr std::size_t kInputReadBytes = 256;
    constexpr std::size_t kMaxInputLineBytes = 512;
    constexpr double kDegreesToRadians = M_PI / 180.0;

    // Continuous joints (1/3/5/7) carry a zero sentinel in the compiled
    // software limits. Validation demands finite bounds, so they get one full
    // turn either way: wide enough for any legal angle, tight enough that a
    // nonsense number still fails at ingest.
    constexpr double kContinuousJointBoundDeg = 360.0;

    // The gate is the software limit the actuation guard already stops on,
    // not the wider model limit — a block that would trip that guard mid-run
    // is refused at ingest instead.
    Eigen::Matrix<double, 7, 1> PositionLimitMagnitudesDeg()
    {
        Eigen::Matrix<double, 7, 1> limits;
        for (int i = 0; i < 7; ++i) {
            const double software_limit_deg = config::kJointSoftwareLimitDeg[i];
            limits(i) = software_limit_deg > 0.0 ? software_limit_deg
                                                 : kContinuousJointBoundDeg;
        }
        return limits;
    }

    Eigen::Matrix<double, 7, 1> VelocityLimitsDegS()
    {
        Eigen::Matrix<double, 7, 1> limits;
        for (int i = 0; i < 7; ++i)
            limits(i) = config::kQdotLimitDegS[i];
        return limits;
    }

    void ProcessPoseTargetLine(PoseTargetMailbox& mailbox, const std::string& line)
    {
        if (line.empty())
            return;

        std::string error;
        const std::optional<PoseTarget> target = ParsePoseTarget(line, error);
        if (!target) {
            std::cout << "target rejected: " << error << "\n";
        } else if (!mailbox.Enqueue(*target)) {
            std::cout << "target rejected: queue is full (8 pending targets)\n";
        } else {
            std::cout << "target queued: " << target->p_desired.x() << " "
                      << target->p_desired.y() << " " << target->p_desired.z()
                      << " m (base_link, "
                      << (target->rotation.has_value() ? "+orientation" : "position-only")
                      << ")\n";
        }
    }

    struct TargetInputRouter {
        PoseTargetMailbox& pose_mailbox;
        JointTrajectoryMailbox& traj_mailbox;
        JointTrajectoryAccumulator accumulator;
    };

    bool IsTrajectoryKeywordLine(const std::string& line)
    {
        return line.rfind("TRAJ_BEGIN", 0) == 0 || line.rfind("TRAJ_END", 0) == 0;
    }

    void ProcessInputLine(TargetInputRouter& router, const std::string& line)
    {
        if (!router.accumulator.Collecting() && !IsTrajectoryKeywordLine(line)) {
            ProcessPoseTargetLine(router.pose_mailbox, line);
            return;
        }

        std::string error;
        std::optional<JointTrajectory> block = router.accumulator.Feed(line, error);
        if (!error.empty()) {
            std::cerr << "trajectory rejected: " << error << " [" << line << "]\n";
            return;
        }
        if (!block)
            return;

        // The sole gate: nothing is published without passing validation.
        const std::optional<std::string> invalid = ValidateJointTrajectory(
            *block, -PositionLimitMagnitudesDeg(), PositionLimitMagnitudesDeg(),
            VelocityLimitsDegS());
        if (invalid) {
            std::cerr << "trajectory rejected: " << *invalid << " [" << line << "]\n";
            return;
        }
        router.traj_mailbox.Publish(
            std::make_unique<JointTrajectory>(std::move(*block)));
    }

    void ConsumeInputBytes(TargetInputRouter& router, const char* bytes,
                           std::size_t count, std::string& pending_line,
                           bool& discarding_overlong_line)
    {
        for (std::size_t i = 0; i < count; ++i) {
            const char character = bytes[i];
            if (character == '\n') {
                if (discarding_overlong_line) {
                    std::cout << "target rejected: input line exceeds 512 bytes\n";
                    discarding_overlong_line = false;
                } else {
                    if (!pending_line.empty() && pending_line.back() == '\r')
                        pending_line.pop_back();
                    ProcessInputLine(router, pending_line);
                }
                pending_line.clear();
            } else if (!discarding_overlong_line) {
                if (pending_line.size() == kMaxInputLineBytes) {
                    pending_line.clear();
                    discarding_overlong_line = true;
                } else {
                    pending_line.push_back(character);
                }
            }
        }
    }
} // namespace

Eigen::Matrix3d RotationFromRpy(double roll, double pitch, double yaw)
{
    return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

std::optional<PoseTarget> ParsePoseTarget(const std::string& line, std::string& error)
{
    std::istringstream input(line);
    std::vector<double> fields;
    double value = 0.0;
    while (input >> value)
        fields.push_back(value);
    std::string trailing;
    input.clear();
    if (input >> trailing || (fields.size() != 3 && fields.size() != 7)) {
        error = "expected 'x y z' or 'x y z qx qy qz qw'";
        return std::nullopt;
    }
    for (const double field : fields)
        if (!std::isfinite(field)) {
            error = "all target values must be finite";
            return std::nullopt;
        }

    PoseTarget target;
    target.p_desired = Eigen::Vector3d(fields[0], fields[1], fields[2]);
    if (fields.size() == 7) {
        if (!config::kAcceptOrientationTargets) {
            error = "orientation targets disabled "
                    "(config::kAcceptOrientationTargets)";
            return std::nullopt;
        }
        const Eigen::Quaterniond q(fields[6], fields[3], fields[4], fields[5]);
        if (std::abs(q.norm() - 1.0) > 1e-3) {
            error = "quaternion must be unit norm (xyzw)";
            return std::nullopt;
        }
        target.rotation = q.normalized().toRotationMatrix();
    }
    return target;
}

bool PoseTargetMailbox::Enqueue(const PoseTarget& target)
{
    const std::uint64_t write = write_index_.load(std::memory_order_relaxed);
    const std::uint64_t read = read_index_.load(std::memory_order_acquire);
    if (write - read >= kCapacity)
        return false;

    entries_[write % kCapacity] = target;
    write_index_.store(write + 1, std::memory_order_release);
    return true;
}

std::optional<PoseTarget> PoseTargetMailbox::TryDequeue()
{
    const std::uint64_t read = read_index_.load(std::memory_order_relaxed);
    const std::uint64_t write = write_index_.load(std::memory_order_acquire);
    if (read == write)
        return std::nullopt;

    PoseTarget target = entries_[read % kCapacity];
    read_index_.store(read + 1, std::memory_order_release);
    return target;
}

JointTrajectoryMailbox::~JointTrajectoryMailbox()
{
    delete slot_.exchange(nullptr);
}

void JointTrajectoryMailbox::Publish(std::unique_ptr<JointTrajectory> traj)
{
    delete slot_.exchange(traj.release());
}

std::unique_ptr<JointTrajectory> JointTrajectoryMailbox::Take()
{
    return std::unique_ptr<JointTrajectory>(slot_.exchange(nullptr));
}

void RunPoseTargetInputFromPipe(PoseTargetMailbox& mailbox,
                                const std::atomic<bool>& stop,
                                const std::string& pipe_path)
{
    JointTrajectoryMailbox discarded_trajectories;
    RunTargetInputFromPipe(mailbox, discarded_trajectories, stop, pipe_path);
}

void RunPoseTargetInputFromFd(PoseTargetMailbox& mailbox,
                              const std::atomic<bool>& stop, int input_fd)
{
    JointTrajectoryMailbox discarded_trajectories;
    RunTargetInput(mailbox, discarded_trajectories, stop, input_fd);
}

void RunTargetInputFromPipe(PoseTargetMailbox& pose_mailbox,
                            JointTrajectoryMailbox& traj_mailbox,
                            const std::atomic<bool>& stop,
                            const std::string& pipe_path)
{
    while (!stop.load(std::memory_order_relaxed)) {
        // Non-blocking open: a blocking O_RDONLY open would wedge teardown
        // until a writer appears. With no writer yet, poll below sees
        // nothing and the FromFd loop idles at its own poll cadence.
        const int fd = open(pipe_path.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(kInputPollTimeoutMs));
            continue;
        }
        RunTargetInput(pose_mailbox, traj_mailbox, stop, fd);
        close(fd);
        // FromFd returned: stop (outer loop exits) or EOF (writer left) —
        // brief pause so a vanished writer cannot spin this loop hot.
        std::this_thread::sleep_for(
            std::chrono::milliseconds(kInputPollTimeoutMs));
    }
}

void RunTargetInput(PoseTargetMailbox& pose_mailbox,
                    JointTrajectoryMailbox& traj_mailbox,
                    const std::atomic<bool>& stop, int input_fd)
{
    TargetInputRouter router{pose_mailbox, traj_mailbox, {}};
    std::array<char, kInputReadBytes> bytes{};
    std::string pending_line;
    bool discarding_overlong_line = false;

    while (!stop.load(std::memory_order_relaxed)) {
        pollfd input{input_fd, POLLIN, 0};
        const int ready = poll(&input, 1, kInputPollTimeoutMs);
        if (ready == 0)
            continue;
        if (ready < 0) {
            if (errno == EINTR)
                continue;
            break;
        }
        if (stop.load(std::memory_order_relaxed))
            break;
        if (input.revents & POLLNVAL)
            break;
        if (!(input.revents & (POLLIN | POLLHUP))) {
            if (input.revents & POLLERR)
                break;
            continue;
        }

        const ssize_t read_count = read(input_fd, bytes.data(), bytes.size());
        if (read_count > 0) {
            ConsumeInputBytes(router, bytes.data(),
                              static_cast<std::size_t>(read_count), pending_line,
                              discarding_overlong_line);
            continue;
        }
        if (read_count < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
                continue;
            break;
        }
        // EOF terminates a final line even without '\n'; a stop deliberately
        // drops an incomplete line instead of enqueueing during teardown.
        if (!stop.load(std::memory_order_relaxed)) {
            if (discarding_overlong_line)
                std::cout << "target rejected: input line exceeds 512 bytes\n";
            else if (!pending_line.empty())
                ProcessInputLine(router, pending_line);
        }
        break;
    }
}

PoseTargetSource::PoseTargetSource(Eigen::Vector3d startup_position_m,
                                   PoseTarget terminal_target,
                                   PoseTargetMailbox& mailbox,
                                   CartesianMotionLimits profile_limits,
                                   double target_hold_s)
    : mailbox_(mailbox), active_target_(std::move(terminal_target)),
      profile_limits_(profile_limits), target_hold_s_(target_hold_s)
{
    if (!startup_position_m.allFinite())
        throw std::invalid_argument("startup position must be finite");
    if (!active_target_.p_desired.allFinite())
        throw std::invalid_argument("terminal target position must be finite");
    if (!std::isfinite(profile_limits_.max_speed_m_s) ||
        !std::isfinite(profile_limits_.max_acceleration_m_s2) ||
        !std::isfinite(profile_limits_.max_jerk_m_s3) ||
        profile_limits_.max_speed_m_s <= 0.0 ||
        profile_limits_.max_acceleration_m_s2 <= 0.0 ||
        profile_limits_.max_jerk_m_s3 <= 0.0)
        throw std::invalid_argument("Cartesian profile limits must be finite and positive");
    if (!std::isfinite(target_hold_s_) || target_hold_s_ < 0.0)
        throw std::invalid_argument("target hold duration must be finite and non-negative");
    active_target_.rotation.reset();
    const std::optional<CartesianSegmentProfile> profile =
        CartesianSegmentProfile::Create(startup_position_m,
                                        active_target_.p_desired,
                                        profile_limits_);
    if (!profile)
        throw std::invalid_argument("startup-to-terminal profile is invalid");
    active_profile_ = *profile;
    phase_ = Phase::kFollowingProfile;
}

bool PoseTargetSource::ValidDt(double dt_s) const
{
    return std::isfinite(dt_s) && dt_s > 0.0;
}

void PoseTargetSource::ActivateOneQueuedTarget()
{
    const std::optional<PoseTarget> next = mailbox_.TryDequeue();
    if (!next)
        return;
    const std::optional<CartesianSegmentProfile> profile =
        CartesianSegmentProfile::Create(active_target_.p_desired,
                                        next->p_desired, profile_limits_);
    if (!profile)
        throw std::invalid_argument("queued target cannot form a finite Cartesian profile");

    active_target_ = *next;
    active_target_.rotation.reset();
    active_profile_ = *profile;
    profile_elapsed_s_ = 0.0;
    ++sequence_;
    phase_ = Phase::kFollowingProfile;
}

Reference PoseTargetSource::StationaryReference() const
{
    Reference reference;
    PoseReference pose;
    pose.p_desired = active_target_.p_desired;
    pose.rotation.reset();
    pose.twist = Twist{};
    pose.sequence = sequence_;
    pose.arrival_eligible = true;
    reference.pose = pose;
    return reference;
}

Reference PoseTargetSource::Get(const RobotState& /*state*/, double dt_s,
                                ControllerStatus& /*status*/)
{
    const bool valid_dt = ValidDt(dt_s);
    if (phase_ == Phase::kDwell && valid_dt) {
        dwell_elapsed_s_ += dt_s;
        if (dwell_elapsed_s_ >= target_hold_s_)
            phase_ = Phase::kReadyForNext;
    }
    if (phase_ == Phase::kReadyForNext)
        ActivateOneQueuedTarget();

    if (phase_ != Phase::kFollowingProfile)
        return StationaryReference();

    const std::optional<CartesianProfileSample> profile_sample =
        active_profile_->Sample(profile_elapsed_s_);
    if (!profile_sample)
        throw std::logic_error("active Cartesian profile has invalid elapsed time");

    Reference reference;
    PoseReference pose;
    pose.p_desired = profile_sample->position_m;
    pose.rotation.reset();
    pose.twist.linear_m_s = profile_sample->velocity_m_s;
    pose.twist.angular_rad_s.setZero();
    pose.sequence = sequence_;
    pose.arrival_eligible = profile_sample->complete;
    reference.pose = pose;

    if (profile_sample->complete) {
        active_profile_.reset();
        phase_ = Phase::kAwaitTerminalArrival;
    } else if (valid_dt) {
        profile_elapsed_s_ += dt_s;
    }
    return reference;
}

void PoseTargetSource::OnArrived()
{
    if (phase_ != Phase::kAwaitTerminalArrival)
        return;
    dwell_elapsed_s_ = 0.0;
    phase_ = Phase::kDwell;
}

JointTrajectorySource::JointTrajectorySource(
    Eigen::Matrix<double, 7, 1> hold_q_rad, JointTrajectoryMailbox& mailbox)
    : mailbox_(mailbox), hold_q_rad_(std::move(hold_q_rad))
{
    if (!hold_q_rad_.allFinite())
        throw std::invalid_argument("takeover hold position must be finite");
}

JointTrajectorySource::~JointTrajectorySource() = default;

Reference JointTrajectorySource::Get(const RobotState& state, double dt_s,
                                     ControllerStatus& status)
{
    // The activation splice guard. A trajectory that does not begin where the
    // arm actually is would step the command by that whole distance on its
    // first cycle, so it is dropped whole rather than followed in part. The
    // distance is the WRAPPED one (State.h): measured positions arrive on
    // [0, 360) and trajectory points are signed, so a joint at -20 deg reads
    // 340 deg and an unwrapped guard would reject everything.
    if (std::unique_ptr<JointTrajectory> incoming = mailbox_.Take()) {
        const double tolerance_rad =
            config::kTrajStartToleranceDeg * kDegreesToRadians;
        const double start_error_rad =
            (incoming->points.empty() || !state.q_rad.allFinite())
                ? std::numeric_limits<double>::infinity()
                : WrappedJointError(incoming->points.front().q_rad, state.q_rad)
                      .cwiseAbs()
                      .maxCoeff();
        if (!(start_error_rad <= tolerance_rad)) {
            status.joint_traj_rejected = true;
            status.joint_traj_start_error_deg =
                start_error_rad / kDegreesToRadians;
            incoming.reset();
        } else {
            active_ = std::move(incoming);
            elapsed_s_ = 0.0;
            complete_reported_ = false;
            status.joint_traj_activated = true;
        }
    }

    Reference reference;
    JointReference joint;
    joint.qdot_rad_s.setZero();
    if (!active_) {
        joint.q_rad = hold_q_rad_;
        reference.joint = joint;
        return reference;
    }

    const JointTrajectorySample sample =
        SampleJointTrajectory(*active_, elapsed_s_);
    if (std::isfinite(dt_s) && dt_s > 0.0)
        elapsed_s_ += dt_s;
    if (sample.complete) {
        hold_q_rad_ = sample.q_rad;
        if (!complete_reported_) {
            complete_reported_ = true;
            status.joint_traj_complete_edge = true;
        }
    }
    joint.q_rad = sample.q_rad;
    joint.qdot_rad_s = sample.qdot_rad_s;
    reference.joint = joint;
    return reference;
}

void NotifyPoseTargetSourceOnArrivalEdge(PoseTargetSource& source,
                                         const ControllerStatus& status)
{
    if (status.arrived_edge)
        source.OnArrived();
}
