//
// ReconstructBlock — turns an emitted wire block back into the dense joint
// trajectory the CONTROLLER will actually execute, using the controller's
// own parser and interpolator.
//
// Deliberately isolated in its own translation unit. `utils.h` (gtsam side)
// and `JointTrajectory.h` (controller side) each define a DIFFERENT
// `struct JointTrajectory`, and including both is a hard redefinition
// error. Targets.h documents the same collision. So the controller's types
// are used HERE, behind a plain-Eigen result that the gtsam-side validator
// can consume without ever seeing them.
//
// Nothing in here re-implements the controller's behaviour: it calls
// JointTrajectoryAccumulator::Feed and SampleJointTrajectory directly. A
// validator that reconstructed what it merely believed the controller does
// could certify a trajectory the controller then executes differently.
//

#pragma once

#include <functional>
#include <string>
#include <vector>

#include <Eigen/Dense>

// One instant of the reconstructed trajectory.
struct ReconstructedSample {
    double t_s = 0.0;
    Eigen::Matrix<double, 7, 1> q_rad = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> qdot_rad_s = Eigen::Matrix<double, 7, 1>::Zero();
};

struct ReconstructedTrajectory {
    std::vector<ReconstructedSample> samples;  // uniform dt, from t = 0
    double duration_s = 0.0;
    bool parsed = false;
    std::string error;  // set when parsed is false
};

// Parses `block` with the controller's accumulator and evaluates it at
// `dt_s` intervals with the controller's cubic-Hermite sampler.
ReconstructedTrajectory ReconstructEmittedBlock(const std::string& block, double dt_s);

// Samples at arbitrary instants, for comparing the reconstruction against
// the GP-dense trajectory at the GP's own timestamps. The block is parsed
// ONCE, by the controller's own parser, when the sampler is made; each call
// then only interpolates. (Sampling per call used to re-parse the full
// block — quadratic in trajectory length, ~20 s for a 16 s circle plan.)
// An unparseable block yields a sampler that returns zeroed samples, same
// as the per-call form did.
std::function<ReconstructedSample(double)> MakeReconstructionSampler(
    const std::string& block);
