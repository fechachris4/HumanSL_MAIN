//
// ValidatePath — measures a planned trajectory against what was requested,
// on the reconstruction the controller will actually execute.
//
// gtsam-side: it forward-kinematics through PlannerModel and queries the
// SDF. It therefore CANNOT include basic_control's JointTrajectory.h
// (utils.h defines a rival struct of the same name), which is why the
// reconstruction arrives as plain Eigen samples from ReconstructBlock.h.
//

#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "PathValidationReport.h"
#include "PlannerModel.h"
#include "ReconstructBlock.h"
#include "WorldSdf.h"

// `reconstructed` is the controller's own cubic-Hermite reconstruction of
// the FINAL emitted block, sampled at the controller's rate — the joint
// path the arm actually follows.
//
// `gp_dense` is the optimiser's output before subsampling, used only to
// separate planner error from wire-transport loss. `reconstruction_at`
// samples the same block at an arbitrary instant so the two can be compared
// at the GP's own timestamps.
PathValidationReport ValidatePlannedPath(
    const PlannerModel& model,
    const ReconstructedTrajectory& reconstructed,
    const std::vector<gtsam::Vector>& gp_dense,
    double gp_dense_duration_s,
    const std::function<ReconstructedSample(double)>& reconstruction_at,
    const gpmp2::SignedDistanceField& sdf,
    const std::string& sdf_contents,
    const ValidationInputs& inputs,
    bool optimiser_converged);
