//
// Targets — desired positions, their SPSC mailbox, and the reference source.
// Never talks to the robot; does no control math.
//

#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "Config.h"
#include "State.h"

// Declared, not included: planner_bridge/trajectory_generation/include/
// utils.h defines a
// different global struct of this name, and planner_bridge translation units
// include both that and this header. Callers that touch the trajectory's
// contents include JointTrajectory.h themselves.
struct JointTrajectory;

// Single-slot, latest-wins handoff for whole joint trajectories. The input
// thread is the only producer and validates every block BEFORE publishing,
// so the RT consumer only ever sees a valid trajectory. Publish deletes the
// block it displaces; Take is one atomic exchange, and the caller owns what
// it receives — the one bounded free per plan activation is the accepted RT
// exception recorded in the plan header.
class JointTrajectoryMailbox
{
public:
    JointTrajectoryMailbox() = default;
    ~JointTrajectoryMailbox();
    JointTrajectoryMailbox(const JointTrajectoryMailbox&) = delete;
    JointTrajectoryMailbox& operator=(const JointTrajectoryMailbox&) = delete;

    void Publish(std::unique_ptr<JointTrajectory> traj); // input thread
    std::unique_ptr<JointTrajectory> Take();             // RT thread; may be null

private:
    std::atomic<JointTrajectory*> slot_{nullptr};
};

// Reads trajectory blocks from a named pipe, surviving writer disconnects:
// on EOF the pipe is reopened, so each bridge invocation may open, write,
// and close independently. `stop` is the only exit. The fd-level loop is
// the tested RunTargetInput, unchanged.
void RunTargetInputFromPipe(JointTrajectoryMailbox& traj_mailbox,
                            const std::atomic<bool>& stop,
                            const std::string& pipe_path);

// Same input loop over a borrowed POSIX file descriptor. Kept separate so the
// pipe entry point stays simple and the partial-pipe teardown contract is
// testable without robot dependencies.
//
// Line routing: while the accumulator is collecting, every line is a
// trajectory line; while it is idle, a TRAJ_* keyword opens a block and
// anything else is rejected — joint trajectories are the only input this
// controller accepts. A completed block is validated against the compiled
// joint limits and only then published. Any grammar or validation error goes
// to stderr with the offending line and resets the accumulator — nothing is
// silently skipped, and the thread survives.
void RunTargetInput(JointTrajectoryMailbox& traj_mailbox,
                    const std::atomic<bool>& stop, int input_fd);

// Follows whole joint trajectories published by the input thread.
//
// Startup holds the measured takeover q at zero velocity. Each RT cycle it
// Takes any newly published trajectory: if the first point is within
// config::kTrajStartToleranceDeg of the measured q on EVERY joint it is
// activated with its clock at zero, otherwise it is deleted, the rejection
// is reported in ControllerStatus, and the current reference keeps holding —
// a trajectory that fails the splice guard is never partially followed.
// While active, Get returns the Hermite sample; past the end it holds the
// final point and reports arrival once.
//
// Never blocks, never does I/O and never allocates: Take is one atomic
// exchange, and the bounded delete of a rejected or displaced trajectory is
// the same accepted RT exception as the mailbox itself.
//
// CALLER CONTRACT: `dt_s` must be the Runner's already-clamped cycle time
// (ClampedCycleDt). Non-positive and non-finite values are ignored here, but
// a large positive dt is trusted and would jump the sample clock — the loop's
// single dt clamp is what bounds it, and duplicating that bound here would
// mean two places deciding what a cycle may last.
class JointTrajectorySource : public ReferenceSource
{
public:
    JointTrajectorySource(Eigen::Matrix<double, 7, 1> hold_q_rad,
                          JointTrajectoryMailbox& mailbox);
    ~JointTrajectorySource() override;
    JointTrajectorySource(const JointTrajectorySource&) = delete;
    JointTrajectorySource& operator=(const JointTrajectorySource&) = delete;

    // Sequencing is the planner's job, so there is no arrival edge to honour
    // here: the inherited no-op OnArrivalEdge is deliberate.
    Reference Get(const RobotState& state, double dt_s,
                  ControllerStatus& status) override;

private:
    JointTrajectoryMailbox& mailbox_;
    Eigen::Matrix<double, 7, 1> hold_q_rad_;
    // Incomplete here on purpose (see the forward declaration above), so the
    // destructor is defined in the .cpp.
    std::unique_ptr<JointTrajectory> active_;
    double elapsed_s_ = 0.0;
    bool complete_reported_ = false;
};
