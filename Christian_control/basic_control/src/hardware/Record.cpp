//
// Record: preallocated in-memory log for the cyclic loop + CSV output.
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
    samples_[next_] = sample;
    next_ = (next_ + 1) % samples_.size();
    ++total_pushed_;
}

std::size_t LoopLog::size() const
{
    return total_pushed_ < samples_.size() ? total_pushed_ : samples_.size();
}

std::size_t LoopLog::total_pushed() const
{
    return total_pushed_;
}

void LoopLog::WriteCsv(std::ostream& csv) const
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
    csv << ",arm_state,base_fault,refresh_ok,sigma_min\n";

    // Oldest-first: when the ring has wrapped, the oldest sample sits at
    // next_ (the slot about to be overwritten).
    const std::size_t count = size();
    const std::size_t start = total_pushed_ > count ? next_ : 0;
    for (std::size_t n = 0; n < count; ++n) {
        const LoopLogSample& s = samples_[(start + n) % samples_.size()];
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
            << (s.refresh_ok ? 1 : 0) << "," << s.sigma_min << "\n";
    }
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
