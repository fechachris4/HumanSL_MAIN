#include "TrajectoryEmit.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace {

constexpr double kRadToDeg = 180.0 / M_PI;

}  // namespace

std::string FormatTrajectoryBlock(const std::vector<gtsam::Vector>& pos_rad,
                                  const std::vector<gtsam::Vector>& vel_rad_s,
                                  double total_time_sec) {
    if (pos_rad.size() < 2)
        throw std::invalid_argument("trajectory needs at least two support states");
    if (pos_rad.size() != vel_rad_s.size())
        throw std::invalid_argument("position/velocity state counts differ");
    if (!std::isfinite(total_time_sec) || total_time_sec <= 0.0)
        throw std::invalid_argument("trajectory duration must be finite and positive");
    for (std::size_t i = 0; i < pos_rad.size(); ++i) {
        if (pos_rad[i].size() != 7 || vel_rad_s[i].size() != 7)
            throw std::invalid_argument("support state " + std::to_string(i) +
                                        " is not seven-dimensional");
    }

    const std::size_t n_states = pos_rad.size();
    const double dt_s = total_time_sec / static_cast<double>(n_states - 1);
    const std::size_t emitted = std::min(n_states, kMaxTrajectoryBlockPoints);

    std::ostringstream block;
    block << std::fixed << std::setprecision(6);
    block << "TRAJ_BEGIN " << emitted << "\n";
    for (std::size_t row = 0; row < emitted; ++row) {
        // Rounded even spread over [0, n_states-1]; strictly increasing
        // because emitted <= n_states, and exact at both endpoints.
        const std::size_t i = static_cast<std::size_t>(
            std::llround(static_cast<double>(row) *
                         static_cast<double>(n_states - 1) /
                         static_cast<double>(emitted - 1)));
        block << static_cast<double>(i) * dt_s;
        for (int joint = 0; joint < 7; ++joint)
            block << ' ' << pos_rad[i](joint) * kRadToDeg;
        for (int joint = 0; joint < 7; ++joint)
            block << ' ' << vel_rad_s[i](joint) * kRadToDeg;
        block << "\n";
    }
    block << "TRAJ_END\n";
    return block.str();
}
