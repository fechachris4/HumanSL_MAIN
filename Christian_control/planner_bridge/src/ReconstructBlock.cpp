// The ONLY translation unit that includes basic_control's JointTrajectory.h
// on the planner side. See the header: utils.h defines a different struct of
// the same name, and the two cannot meet.

#include "ReconstructBlock.h"

#include <memory>
#include <sstream>

#include "JointTrajectory.h"  // basic_control — the controller's own types

namespace {

// Parses the block with the controller's accumulator. Returns false and
// fills `error` rather than throwing, so a malformed block is a reported
// validation failure rather than an exception crossing the boundary.
bool ParseBlock(const std::string& block, ::JointTrajectory& out,
                std::string& error) {
    JointTrajectoryAccumulator accumulator;
    std::istringstream lines(block);
    std::string line;
    bool complete = false;
    while (std::getline(lines, line)) {
        const std::optional<::JointTrajectory> finished =
            accumulator.Feed(line, error);
        if (!error.empty()) return false;
        if (finished) {
            out = *finished;
            complete = true;
        }
    }
    if (!complete) {
        error = "no complete TRAJ_BEGIN/TRAJ_END block";
        return false;
    }
    return true;
}

}  // namespace

ReconstructedTrajectory ReconstructEmittedBlock(const std::string& block,
                                                double dt_s) {
    ReconstructedTrajectory out;
    ::JointTrajectory parsed;
    if (!ParseBlock(block, parsed, out.error)) return out;
    if (parsed.points.empty()) {
        out.error = "block parsed but contained no points";
        return out;
    }
    out.parsed = true;
    out.duration_s = parsed.points.back().t_s;

    const double dt = dt_s > 0.0 ? dt_s : 0.002;
    for (double t = 0.0; t <= out.duration_s + 1e-12; t += dt) {
        const JointTrajectorySample sample = SampleJointTrajectory(parsed, t);
        ReconstructedSample entry;
        entry.t_s = t;
        entry.q_rad = sample.q_rad;
        entry.qdot_rad_s = sample.qdot_rad_s;
        out.samples.push_back(entry);
    }
    return out;
}

std::function<ReconstructedSample(double)> MakeReconstructionSampler(
    const std::string& block) {
    // shared_ptr so the parsed trajectory lives exactly as long as the
    // returned std::function (which must be copyable).
    auto parsed = std::make_shared<::JointTrajectory>();
    std::string error;
    const bool ok = ParseBlock(block, *parsed, error) && !parsed->points.empty();
    return [parsed, ok](double t_s) {
        ReconstructedSample entry;
        entry.t_s = t_s;
        if (!ok) return entry;
        const JointTrajectorySample sample = SampleJointTrajectory(*parsed, t_s);
        entry.q_rad = sample.q_rad;
        entry.qdot_rad_s = sample.qdot_rad_s;
        return entry;
    };
}
