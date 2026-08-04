//
// Hardware — implementations for Hardware.h.
//

#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

#include "Hardware.h"

// ---------------------------------------------------------------
// Connect — the Kortex sessions
// ---------------------------------------------------------------

//
// Connect: the two Kortex sessions to the arm (see Connect.h).
//





namespace
{

    // The Gen3 base always listens on these two ports (fixed by the robot's
    // firmware, not a choice of ours — so not in Config.h).
    constexpr unsigned kTcpPort = 10000; // configuration + high-level commands
    constexpr unsigned kUdpPort = 10001; // 1 kHz low-level cyclic streaming

} // namespace

Connect::Channel::Channel(std::unique_ptr<k_api::ITransportClient> transport_in,
                          const std::string& ip_address, unsigned port, const char* label)
    : transport(std::move(transport_in))
{
    // connect() reports failure by return value, not by throwing — check it,
    // or an unreachable arm only fails later, inside CreateSession, with a
    // much less helpful error.
    if (!transport->connect(ip_address, port))
        throw std::runtime_error(std::string("could not reach the arm at ") + ip_address + ":" +
                                 std::to_string(port) + " (" + label + " channel)");

    router = std::make_unique<k_api::RouterClient>(transport.get(), [](k_api::KError err) {
        std::cout << "Kortex API error: " << err.toString() << std::endl;
    });

    auto session_info = k_api::Session::CreateSessionInfo();
    session_info.set_username(config::kSessionUsername);
    session_info.set_password(config::kSessionPassword);
    session_info.set_session_inactivity_timeout(config::kSessionInactivityTimeoutMs);
    session_info.set_connection_inactivity_timeout(config::kConnectionInactivityTimeoutMs);

    session = std::make_unique<k_api::SessionManager>(router.get());
    session->CreateSession(session_info);
}

Connect::Channel::~Channel()
{
    // CloseSession talks to the robot, so it can throw if the link is
    // already dead — and a throwing destructor would abort the program.
    try {
        session->CloseSession();
    } catch (...) {
        std::cerr << "warning: failed to close a Kortex session cleanly\n";
    }
    router->SetActivationStatus(false);
    transport->disconnect();
}

Connect::Connect(const std::string& ip_address)
    : tcp_(std::make_unique<k_api::TransportClientTcp>(), ip_address, kTcpPort, "TCP"),
      udp_(std::make_unique<k_api::TransportClientUdp>(), ip_address, kUdpPort, "UDP"),
      base_(std::make_unique<k_api::Base::BaseClient>(tcp_.router.get())),
      device_config_(std::make_unique<k_api::DeviceConfig::DeviceConfigClient>(
          tcp_.router.get())),
      base_cyclic_(std::make_unique<k_api::BaseCyclic::BaseCyclicClient>(udp_.router.get()))
{
    std::cout << "Connected to arm at " << ip_address << " (TCP + real-time UDP).\n";
}

bool Connect::EnsureJointLimits(std::ostream& out)
{
    // Actuator device identifiers are 1..7 in joint order.
    constexpr unsigned kHigh =
        k_api::ActuatorConfig::SafetyIdentifierBankA::JOINT_LIMIT_HIGH;
    constexpr unsigned kLow =
        k_api::ActuatorConfig::SafetyIdentifierBankA::JOINT_LIMIT_LOW;

    bool changed = false;
    for (std::uint32_t id = 1; id <= 7; ++id)
    {
        const double warn = config::kJointLimitWarnDeg[id - 1];
        const double error = config::kJointLimitErrorDeg[id - 1];
        if (warn == 0.0 && error == 0.0)
            continue; // continuous joint, or one we deliberately leave alone

        // Per joint, so one actuator whose configuration service has stopped
        // answering cannot prevent every OTHER joint's limits from being
        // restored — the failure mode found on 2026-08-04, where joint 6
        // aborted the gate and joint 4 was left on a degenerate 0/0 band.
        try
        {
        for (const unsigned identifier : {kHigh, kLow})
        {
            const double sign = (identifier == kHigh) ? 1.0 : -1.0;
            const float want_warn = static_cast<float>(sign * warn);
            const float want_error = static_cast<float>(sign * error);

            k_api::Common::SafetyHandle handle;
            handle.set_identifier(identifier);

            k_api::DeviceConfig::SafetyConfiguration cfg =
                device_config_->GetSafetyConfiguration(handle, id);
            if (cfg.warning_threshold() != want_warn ||
                cfg.error_threshold() != want_error)
            {
                out << "joint " << id << " "
                    << (identifier == kHigh ? "JOINT_LIMIT_HIGH" : "JOINT_LIMIT_LOW")
                    << " was warn=" << cfg.warning_threshold()
                    << " error=" << cfg.error_threshold() << "; setting warn="
                    << want_warn << " error=" << want_error << "\n";
                cfg.mutable_handle()->set_identifier(identifier);
                cfg.set_warning_threshold(want_warn);
                cfg.set_error_threshold(want_error);
                device_config_->SetSafetyConfiguration(cfg, id);
                changed = true;
            }

            const auto verified =
                device_config_->GetSafetyConfiguration(handle, id);
            if (verified.error_threshold() != want_error)
            {
                out << "robot NOT ready: joint " << id
                    << " did not accept its JOINT_LIMIT threshold (read back "
                    << verified.error_threshold() << ", wanted " << want_error
                    << ")\n";
                return false;
            }
        }
        } // try
        catch (std::exception& ex)
        {
            out << "joint " << id
                << " did not answer its safety-configuration service ("
                << ex.what() << ")\n";
            if (!config::kAllowUnverifiedActuators)
            {
                out << "  robot NOT ready: its JOINT_LIMIT band cannot be"
                       " restored, and a degenerate 0/0 band faults outward"
                       " motion. Power-cycle the arm, or set"
                       " config::kAllowUnverifiedActuators.\n";
                return false;
            }
            out << "  WARNING: continuing with joint " << id
                << "'s JOINT_LIMIT band UNKNOWN"
                   " (config::kAllowUnverifiedActuators). If it is 0/0, the"
                   " firmware will fault this joint on any motion away from"
                   " zero.\n";
        }
    }
    out << "joint-limit gate: PASS (configured thresholds verified"
        << (changed ? ", corrections applied" : "") << ")\n";
    return true;
}

// The per-joint control-mode verification gate was removed 2026-08-04:
// actuators boot in POSITION mode, nothing in this program ever commands
// another mode, the gate never once found a deviation — and probing cost a
// full RPC timeout per unreachable actuator (joint 6) on every start.

// ---------------------------------------------------------------
// CyclicSession — the per-cycle command frame
// ---------------------------------------------------------------

//
// Cyclic: the UDP cyclic exchange (see Cyclic.h).
//
// Pattern follows TrajectoryExecution/src/KinovaTrajectory.cpp
// (joint_position_control): LOW_LEVEL_SERVOING + one command frame per cycle.
//




namespace
{
    // The number of physical joints is defined by the JointVector type.
    constexpr int NUM_JOINTS = std::tuple_size_v<JointVector>;

    // The robot expects command angles in [0, 360).
    double wrap_0_360(double angle_deg)
    {
        double wrapped = std::fmod(angle_deg, 360.0);
        return wrapped < 0.0 ? wrapped + 360.0 : wrapped;
    }
} // namespace

CyclicSession::CyclicSession(k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
    : base_cyclic_(base_cyclic)
{
}

k_api::BaseCyclic::Feedback CyclicSession::Seed()
{
    // Read the robot's current state before sending the first low-level command.
    k_api::BaseCyclic::Feedback feedback = read_feedback(base_cyclic_);

    // Make one persistent command slot for each physical actuator.
    for (int i = 0; i < NUM_JOINTS; ++i)
        command_.add_actuators();
    return feedback;
}

k_api::BaseCyclic::Feedback CyclicSession::Send(const JointVector& setpoints_deg)
{
    // Put the next desired absolute angle into each joint's existing command slot.
    for (int i = 0; i < NUM_JOINTS; ++i)
        command_.mutable_actuators(i)->set_position(wrap_0_360(setpoints_deg[i]));

    // Advance the packet sequence number so this cyclic exchange is identifiable.
    command_.set_frame_id((command_.frame_id() + 1) % 65536);

    // Give every actuator command the same packet ID for this control cycle.
    for (int i = 0; i < NUM_JOINTS; ++i)
        command_.mutable_actuators(i)->set_command_id(command_.frame_id());

    // Send the complete seven-joint packet and return the robot's feedback reply.
    return base_cyclic_->Refresh(command_, 0);
}

// ---------------------------------------------------------------
// Measurement — the standalone feedback read
// ---------------------------------------------------------------

//
// Measure: read sensor data from the arm (no motion commands).
//


k_api::BaseCyclic::Feedback read_feedback(k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{
    // The ONLY RefreshFeedback call in the program (see Measure.h).
    return base_cyclic->RefreshFeedback();
}

// ---------------------------------------------------------------
// Recording — the run log and its writer
// ---------------------------------------------------------------

//
// Record: preallocated sample queue for the cyclic loop + streaming CSV
// output. See Record.h for the SPSC contract and why the log is written
// during the run rather than after it.
//



LoopLog::LoopLog(std::size_t capacity)
{
    samples_.resize(capacity); // all allocation happens here, before the loop
}

void LoopLog::push(const LoopLogSample& sample)
{
    // head_ is ours; tail_ tells us how far the writer has caught up. The
    // acquire pairs with the writer's release in Drain, so a slot it has
    // released is safe to reuse.
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t tail = tail_.load(std::memory_order_acquire);
    if (head - tail >= samples_.size()) {
        ++dropped_; // writer a whole buffer behind — never overwrite it
        return;
    }
    samples_[head % samples_.size()] = sample;
    // Release: the sample is fully written before the writer can see the
    // index that publishes it.
    head_.store(head + 1, std::memory_order_release);
}

std::size_t LoopLog::Drain(std::vector<LoopLogSample>& out)
{
    const std::size_t tail = tail_.load(std::memory_order_relaxed); // ours
    const std::size_t head = head_.load(std::memory_order_acquire);
    const std::size_t count = head - tail;
    out.resize(count); // no allocation once reserved to capacity()
    for (std::size_t n = 0; n < count; ++n)
        out[n] = samples_[(tail + n) % samples_.size()];
    // Release only after the copy: until this store the producer treats
    // these slots as still in use.
    tail_.store(head, std::memory_order_release);
    return count;
}

std::size_t LoopLog::capacity() const
{
    return samples_.size();
}

std::size_t LoopLog::total_pushed() const
{
    return head_.load(std::memory_order_relaxed) + dropped_;
}

std::size_t LoopLog::dropped() const
{
    return dropped_;
}

void WriteCsvHeader(std::ostream& csv)
{
    csv << "time_s,dt_s,pd_x,pd_y,pd_z,p_x,p_y,p_z";
    for (int i = 1; i <= 7; ++i)
        csv << ",cmd_j" << i;
    for (int i = 1; i <= 7; ++i)
        csv << ",cmdvel_j" << i;
    for (int i = 1; i <= 7; ++i)
        csv << ",meas_j" << i;
    for (int i = 1; i <= 7; ++i)
        csv << ",measraw_j" << i;
    for (int i = 1; i <= 7; ++i)
        csv << ",vel_j" << i;
    for (int i = 1; i <= 7; ++i)
        csv << ",torque_j" << i;
    for (int i = 1; i <= 7; ++i)
        csv << ",fault_j" << i;
    csv << ",arm_state,base_fault,refresh_ok,sigma_min,rot_error_rad"
        << ",t_send_s,t_recv_s,quat_x,quat_y,quat_z,quat_w,pd_beyond_reach";
    for (int i = 1; i <= 7; ++i)
        csv << ",ref_j" << i;
    csv << ",playback_t_s,playback_state,command_frame_id,feedback_frame_id";
    for (int i = 1; i <= 7; ++i)
        csv << ",command_ack_j" << i;
    for (int i = 1; i <= 7; ++i)
        csv << ",status_flags_j" << i;
    for (int i = 1; i <= 7; ++i)
        csv << ",jitter_us_j" << i;
    csv << ",cycle";
    for (int i = 1; i <= 7; ++i)
        csv << ",req_j" << i;
    for (int i = 1; i <= 7; ++i)
        csv << ",reqvel_j" << i;
    for (int i = 1; i <= 7; ++i)
        csv << ",lead_limited_j" << i;
    for (int i = 1; i <= 7; ++i)
        csv << ",ack_unchanged_j" << i;
    csv << "\n";
}

void WriteCsvRow(std::ostream& csv, const LoopLogSample& s)
{
    csv << s.t_s << "," << s.dt_s;
    for (double v : s.p_desired_m)
        csv << "," << v;
    for (double v : s.p_current_m)
        csv << "," << v;
    for (double v : s.commanded_deg)
        csv << "," << v;
    for (double v : s.commanded_velocity_deg_s)
        csv << "," << v;
    for (double v : s.measured_deg)
        csv << "," << v;
    for (double v : s.measured_raw_deg)
        csv << "," << v;
    for (double v : s.velocity_deg_s)
        csv << "," << v;
    for (double v : s.torque_nm)
        csv << "," << v;
    for (std::uint32_t v : s.fault_bank)
        csv << "," << v;
    csv << "," << s.arm_state << "," << s.base_fault_bank << ","
        << (s.refresh_ok ? 1 : 0) << "," << s.sigma_min << ","
        << s.rot_error_rad << "," << s.t_send_s << "," << s.t_recv_s;
    for (double v : s.tool_quat_xyzw)
        csv << "," << v;
    csv << "," << (s.pd_beyond_reach ? 1 : 0);
    for (double v : s.ref_deg)
        csv << "," << v;
    csv << "," << s.playback_t_s << "," << s.playback_state
        << "," << s.command_frame_id << "," << s.feedback_frame_id;
    for (std::uint32_t v : s.actuator_command_ack)
        csv << "," << v;
    for (std::uint32_t v : s.actuator_status_flags)
        csv << "," << v;
    for (std::uint32_t v : s.actuator_jitter_us)
        csv << "," << v;
    csv << "," << s.cycle;
    for (double v : s.requested_deg)
        csv << "," << v;
    for (double v : s.requested_velocity_deg_s)
        csv << "," << v;
    for (bool v : s.lead_limited)
        csv << "," << (v ? 1 : 0);
    for (int v : s.ack_unchanged_cycles)
        csv << "," << v;
    csv << "\n";
}

LoopLogWriter::LoopLogWriter(LoopLog& log, std::ostream& csv,
                             std::chrono::milliseconds interval)
    : log_(log), csv_(csv), interval_(interval)
{
    WriteCsvHeader(csv_);
    // Preamble + header on disk before the takeover: a run killed while
    // connecting still leaves a file that says what it was going to do.
    csv_.flush();
    staging_.reserve(log_.capacity()); // the drain loop allocates nothing
    thread_ = std::thread(&LoopLogWriter::Run, this);
}

LoopLogWriter::~LoopLogWriter()
{
    try {
        Stop();
    } catch (...) {
        // A destructor on an unwinding path must not throw. A failed final
        // drain has already cost us at most the last interval of samples.
    }
}

void LoopLogWriter::Run()
{
    while (!stop_.load(std::memory_order_acquire)) {
        std::this_thread::sleep_for(interval_);
        DrainOnce();
    }
}

void LoopLogWriter::DrainOnce()
{
    const std::size_t count = log_.Drain(staging_);
    if (count == 0)
        return;
    // Format the whole batch first, at the stream's default precision, then
    // hand it over in one write: the file can only ever be cut between
    // rows, never inside one.
    std::ostringstream batch;
    for (const LoopLogSample& s : staging_)
        WriteCsvRow(batch, s);
    const std::string text = batch.str();
    csv_.write(text.data(), static_cast<std::streamsize>(text.size()));
    csv_.flush(); // out of our buffer and into the kernel's
    rows_written_ += count;
}

void LoopLogWriter::Stop()
{
    if (!thread_.joinable())
        return;
    stop_.store(true, std::memory_order_release);
    thread_.join();
    // The producer has stopped and the thread is gone, so this drain is the
    // last one and it sees everything the loop left behind.
    DrainOnce();
}

std::size_t LoopLogWriter::rows_written() const
{
    return rows_written_;
}

std::string timestamped_csv_name(const std::string& prefix)
{
    std::time_t now = std::time(nullptr);
    std::tm local{};
    localtime_r(&now, &local);
    std::ostringstream name;
    name << prefix << std::put_time(&local, "_%Y%m%d_%H%M%S") << ".csv";
    return name.str();
}

std::string dated_run_dir(const std::string& runs_root)
{
    std::time_t now = std::time(nullptr);
    std::tm local{};
    localtime_r(&now, &local);
    std::ostringstream dir;
    dir << runs_root << std::put_time(&local, "/%Y-%m-%d");
    return dir.str();
}
