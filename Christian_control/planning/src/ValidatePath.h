//
// ValidatePath — measures GPMP2's final dense joint trajectory against what
// was requested. Joint samples remain an internal planner artefact; the
// controller receives only the separately projected world-Cartesian path.
//

#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "PathValidationReport.h"
#include "PlannerModel.h"
#include "TimedJointTrajectory.h"
#include "MountSdf.h"

// `trajectory` is the exact final dense GPMP2 result on its uniform grid.
// `sample_at` evaluates the same result at arbitrary instants.
PathValidationReport ValidatePlannedPath(
    const PlannerModel& model,
    const TimedJointTrajectory& trajectory,
    const std::vector<gtsam::Vector>& gp_dense,
    double gp_dense_duration_s,
    const TimedJointSampler& sample_at,
    const gpmp2::SignedDistanceField& sdf,
    const std::string& sdf_contents,
    const ValidationInputs& inputs,
    bool optimiser_converged);
