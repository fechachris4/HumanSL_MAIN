//
// Targets — implementations for Targets.h.
//

#include <array>
#include <cctype>
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

    struct TargetInputRouter {
        JointTrajectoryMailbox& traj_mailbox;
        JointTrajectoryAccumulator accumulator;
    };

    bool IsTrajectoryKeywordLine(const std::string& line)
    {
        return line.rfind("TRAJ_BEGIN", 0) == 0 || line.rfind("TRAJ_END", 0) == 0;
    }

    void ProcessInputLine(TargetInputRouter& router, const std::string& line)
    {
        // Trajectory blocks are the only input this controller accepts. A
        // stray line while idle is reported and dropped rather than ignored:
        // it is usually a target line from the retired Cartesian grammar, and
        // silently swallowing it would look like the arm ignoring a command.
        if (!router.accumulator.Collecting() && !IsTrajectoryKeywordLine(line)) {
            if (!line.empty())
                std::cerr << "input rejected: expected TRAJ_BEGIN — this "
                             "controller follows joint trajectories only ["
                          << line << "]\n";
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


void RunTargetInputFromPipe(JointTrajectoryMailbox& traj_mailbox,
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
        RunTargetInput(traj_mailbox, stop, fd);
        close(fd);
        // FromFd returned: stop (outer loop exits) or EOF (writer left) —
        // brief pause so a vanished writer cannot spin this loop hot.
        std::this_thread::sleep_for(
            std::chrono::milliseconds(kInputPollTimeoutMs));
    }
}

void RunTargetInput(JointTrajectoryMailbox& traj_mailbox,
                    const std::atomic<bool>& stop, int input_fd)
{
    TargetInputRouter router{traj_mailbox, {}};
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
        // Both sides are checked for finiteness before the distance is taken:
        // Eigen's maxCoeff skips a NaN, so an unchecked non-finite value would
        // look like a zero distance and splice. Ingest validation already
        // refuses non-finite trajectories; this is the guard not depending on
        // that.
        const bool measurable =
            !incoming->points.empty() && state.q_rad.allFinite() &&
            incoming->points.front().q_rad.allFinite() &&
            incoming->points.front().qdot_rad_s.allFinite();
        const double start_error_rad =
            !measurable
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
            status.joint_traj_points = static_cast<int>(active_->points.size());
            status.joint_traj_duration_s = active_->points.back().t_s;
        }
    }

    Reference reference;
    JointReference joint;
    joint.qdot_rad_s.setZero();
    if (!active_) {
        joint.q_rad = hold_q_rad_;
        reference.joint = joint;
        reference.joint_is_idle_hold = true; // offerable to the world hold
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
    // A completed trajectory holding its endpoint is an idle hold too —
    // the world hold may take over there (spec §2: re-engage after a
    // trajectory), anchored wherever the trajectory ended.
    reference.joint_is_idle_hold = sample.complete;
    return reference;
}

