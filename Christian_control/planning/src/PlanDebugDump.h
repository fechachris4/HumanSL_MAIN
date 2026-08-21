//
// PlanDebugDump — offline CSVs for looking at what a plan actually did,
// including a plan that failed.
//
// The planner's stdout contract carries only the world-frame Cartesian
// block: joint angles never cross the controller boundary, and that stays
// true. But a plan that fails produces no block at all, and "unresolved run
// of 4 sample(s)" is the whole of what the operator is currently told. The
// evidence needed to understand that message — which samples failed, where
// they sit on the path, and how close the joints were to their limits when
// they did — exists inside the solve and is thrown away.
//
// So this is a diagnostic SIDE-CHANNEL, written only when --debug-dir is
// given, never to stdout, and never read by the controller. Plain CSV with
// a header row, degrees for joints (the same unit the controller's own
// telemetry uses, so the two can be read side by side).
//
// Each function returns an error string on failure and nullopt on success.
//

#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "CartesianPath.h"
#include "PathIk.h"
#include "PlanSolver.h"
#include "utils.h"  // optimisation — TrajectoryResult

// One row per dense trajectory sample: time, the seven joint angles and the
// seven joint velocities. Written for point and path plans alike.
std::optional<std::string> WriteJointTrajectoryCsv(
    const std::string& directory, const TrajectoryResult& trajectory);

// The band each joint was solved against, margin already applied. Separate
// file because it is seven rows, not one per sample.
std::optional<std::string> WriteJointLimitsCsv(const std::string& directory,
                                               const PlanJointLimits& limits);

// One row per path sample: where on the path it is (index, time and percent
// of the way round), the tool position asked for, the sample's status
// (solved / interpolated seed / no IK convergence / converged only outside
// the joint limits), the residuals, the configuration, and how far that
// configuration sits from the nearest bounded joint limit. This is the file
// that explains a failed continuation walk, so it is written even when the
// walk did not succeed.
std::optional<std::string> WritePathIkCsv(const std::string& directory,
                                          const CartesianPath& path_mount,
                                          const PathIkResult& walk,
                                          const PlanJointLimits& limits);

// The failed samples as compact index ranges, e.g. "23-26" or
// "4, 11-12" — the same text the terminal summary prints, so the two can
// never disagree. Empty when every sample solved.
std::string DescribeFailedRanges(const PathIkResult& walk);

// Smallest distance from `q` to a bounded joint's nearer limit, radians.
// Continuous joints have no position limit and are skipped; +infinity when
// every joint is continuous. Plain arithmetic over values the planner
// already produced — no planning decision is recomputed here.
double JointLimitMarginRad(const Eigen::Matrix<double, 7, 1>& q_rad,
                           const PlanJointLimits& limits);

// What this run was, in key,value rows: arm, plan kind, status, and the
// headline numbers. Lets a plot be labelled without re-parsing diagnostics.
struct PlanDebugMeta {
    std::string arm;
    std::string plan_kind;  // "point" or "path"
    std::string status;     // "ok", or the error text when the plan failed
    double final_goal_error_m = 0.0;
    double total_time_s = 0.0;
    // Site-specific summary numbers, written as additional key,value rows in
    // the order given. The two solve sites have genuinely different evidence
    // (a point plan has no validation report; a failed path plan has no
    // trajectory), so a fixed field list would be mostly empty at one site
    // or the other.
    std::vector<std::pair<std::string, std::string>> extra;
};

std::optional<std::string> WritePlanMetaCsv(const std::string& directory,
                                            const PlanDebugMeta& meta);
