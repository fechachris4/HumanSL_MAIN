//
// Record: preallocated sample queue for the cyclic loop + streaming CSV
// output. See Record.h for the SPSC contract and why the log is written
// during the run rather than after it.
//

#include "hardware/Record.h"

#include <ctime>
#include <iomanip>
#include <sstream>

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
