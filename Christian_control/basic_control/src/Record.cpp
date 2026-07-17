//
// Record: fixed-rate logging loop — read joints, timestamp, append to CSV.
//

#include "Record.h"

#include "Measure.h"

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

long record_joint_angles(k_api::BaseCyclic::BaseCyclicClient* base_cyclic, std::ostream& csv,
                         double rate_hz, const std::atomic<bool>& stop)
{
    using clock = std::chrono::steady_clock;
    const auto period =
        std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(1.0 / rate_hz));

    csv << "time_s,dt_s,j1,j2,j3,j4,j5,j6,j7\n";

    const auto t_start = clock::now();
    auto next_tick = t_start; // when the next sample is due
    auto t_prev = t_start;    // for dt
    auto next_heartbeat = t_start + std::chrono::seconds(1);
    long samples = 0;

    while (!stop) {
        // 1. read
        std::vector<double> angles_deg = measure_joint_angles(base_cyclic);

        // 2. timestamp with the steady clock, and dt since last sample
        auto t_now = clock::now();
        double t = std::chrono::duration<double>(t_now - t_start).count();
        double dt = std::chrono::duration<double>(t_now - t_prev).count();
        t_prev = t_now;

        // 3. log one CSV row (the ofstream buffers in memory for us)
        csv << t << "," << dt;
        for (double a : angles_deg)
            csv << "," << a;
        csv << "\n";
        ++samples;

        // heartbeat: one line per second, not per sample
        if (t_now >= next_heartbeat) {
            std::cout << "recording... " << samples << " samples, t=" << static_cast<long>(t)
                      << " s\n";
            next_heartbeat += std::chrono::seconds(1);
        }

        // 4. sleep until the next slot on the fixed grid
        //    (sleep_until, not sleep_for, so timing errors don't add up)
        next_tick += period;
        if (next_tick > t_now)
            std::this_thread::sleep_until(next_tick);
        else
            next_tick = t_now; // fell behind: skip, don't burst to catch up
    }

    csv.flush();
    return samples;
}

void write_move_log_header(std::ostream& csv)
{
    csv << "time_s,dt_s";
    for (int i = 1; i <= 7; ++i)
        csv << ",cmd_j" << i;
    for (int i = 1; i <= 7; ++i)
        csv << ",meas_j" << i;
    for (int i = 1; i <= 7; ++i)
        csv << ",err_j" << i;
    for (int i = 1; i <= 7; ++i)
        csv << ",vel_j" << i;
    for (int i = 1; i <= 7; ++i)
        csv << ",torque_j" << i;
    for (int i = 1; i <= 7; ++i)
        csv << ",current_j" << i;
    for (int i = 1; i <= 7; ++i)
        csv << ",fault_j" << i;
    for (int i = 1; i <= 7; ++i)
        csv << ",warn_j" << i;
    csv << ",arm_state,base_fault,base_warn,refresh_ok\n";
}

void write_move_log_row(std::ostream& csv, const MoveLogSample& s)
{
    csv << s.t_s << "," << s.dt_s;
    for (double v : s.commanded_deg)
        csv << "," << v;
    for (double v : s.measured_deg)
        csv << "," << v;
    for (double v : s.error_deg)
        csv << "," << v;
    for (double v : s.velocity_deg_s)
        csv << "," << v;
    for (double v : s.torque_nm)
        csv << "," << v;
    for (double v : s.current_a)
        csv << "," << v;
    for (std::uint32_t v : s.fault_bank)
        csv << "," << v;
    for (std::uint32_t v : s.warning_bank)
        csv << "," << v;
    csv << "," << s.arm_state << "," << s.base_fault_bank << "," << s.base_warning_bank << ","
        << (s.refresh_ok ? 1 : 0) << "\n";
}

std::string timestamped_csv_name(const std::string& prefix)
{
    std::time_t now = std::time(nullptr);
    std::tm local{};
    localtime_r(&now, &local);
    std::ostringstream name;
    name << prefix << std::put_time(&local, "_%Y-%m-%d_%H-%M-%S") << ".csv";
    return name.str();
}

void plot_move_log(const std::string& csv_path)
{
    namespace fs = std::filesystem;

    // The script lives in basic_control/scripts/; the executable runs from a
    // build directory one level below basic_control (build/ or CLion's
    // cmake-build-debug/), so look one level up from the executable.
    std::error_code ec;
    fs::path exe = fs::read_symlink("/proc/self/exe", ec);
    if (ec) {
        std::cout << "plots skipped: cannot locate the executable's directory\n";
        return;
    }
    fs::path script = exe.parent_path().parent_path() / "scripts" / "plot_move.py";
    if (!fs::exists(script)) {
        std::cout << "plots skipped: " << script << " not found\n";
        return;
    }

    std::cout << "\nAnalyzing " << csv_path << " ...\n";
    std::string cmd = "python3 \"" + script.string() + "\" \"" + csv_path + "\"";
    if (std::system(cmd.c_str()) != 0)
        std::cout << "plot script failed — is python3 with numpy/matplotlib "
                     "installed? Run it by hand: "
                  << cmd << "\n";
}
