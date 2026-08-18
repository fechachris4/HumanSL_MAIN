//
// CartesianPath — a task described as a PATH the tool should trace, rather
// than a single pose it should end at.
//
// The existing goal path answers "where should the hand finish"; everything
// between the endpoints is left to the optimiser, which is correct for
// reaching and useless for drawing. A circle is the opposite: the shape in
// between IS the deliverable.
//
// A path carries its own frame. That is the whole point — it is what stops
// frame conversions leaking into every call site as new frames arrive. The
// same shape can be written in mount, in either arm's base, and later in a
// Vicon room frame, and only the ONE conversion at the planner's edge has
// to know the difference. `msc_project`'s controller/trajectory.py proves
// the pattern: every trajectory type there samples to a target that carries
// its frame with it.
//
// Eigen-only, deliberately, so this header is includable from the gtsam
// side (PlanSolver) and the Pinocchio side (the reachability probe) alike —
// the two cannot share a translation unit, see PinocchioKinematicsAdapter.h.
//

#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "Config.h"  // control — config::ReferenceFrame

// One point on the path: where the tool should be, and when.
struct PathSample {
    double t_s = 0.0;                                       // from path start
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();  // in the path's frame
};

// An ordered, timed sequence of poses in ONE declared frame. Times are
// strictly increasing and start at zero. A point-to-point move is the
// degenerate two-sample case; a receding-horizon controller would later
// sample one of these over its horizon.
struct CartesianPath {
    std::vector<PathSample> samples;
    config::ReferenceFrame frame = config::kReferenceFrame;

    double duration_s() const {
        return samples.empty() ? 0.0 : samples.back().t_s;
    }
};

// How the tool is oriented as it travels.
enum class OrientationPolicy {
    // One orientation held for the whole path. What "draw on a table"
    // wants: the tool keeps pointing into the surface while it traces.
    kFixed,
    // The tool's approach axis (+z) points at the circle centre, so the
    // hand turns as it goes round. What "point at the thing in the middle"
    // wants. Well-defined for a circle because the radial direction is
    // always perpendicular to the plane normal.
    kRadialInward
};

// A circle to trace. `normal` need not be unit length; it is normalised.
// `start_angle_rad` picks where on the rim the path begins, which matters
// because the arm has to reach that point first.
struct CircleSpec {
    Eigen::Vector3d centre_m = Eigen::Vector3d::Zero();
    double radius_m = 0.0;
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    int samples = 0;         // points around the rim; >= 3
    double duration_s = 0.0; // one full lap
    double start_angle_rad = 0.0;
    config::ReferenceFrame frame = config::kReferenceFrame;
    OrientationPolicy orientation = OrientationPolicy::kFixed;
    // Used only when orientation == kFixed. Roll/pitch/yaw, R = Rz*Ry*Rx —
    // the convention BridgeMain's RotationFromRpy and FramePrint.h use.
    Eigen::Vector3d fixed_rpy_rad = Eigen::Vector3d::Zero();
};

// Samples the rim at a constant angular rate, so a circle is traced at
// constant SPEED and the emitted timing needs no reshaping.
//
// The returned path is CLOSED: the last sample repeats the first position
// at t = duration_s, so the trace ends where it began rather than one step
// short. That means `samples` points describe `samples` arcs.
//
// Throws std::invalid_argument on a spec that cannot describe a circle —
// non-positive radius or duration, fewer than three samples, a degenerate
// (near-zero) normal, or non-finite input. A silently-degenerate circle
// would plan and execute as something the operator never asked for.
CartesianPath GenerateCircle(const CircleSpec& spec);

// Sample count for a circle of `radius_m` whose straight-chord deviation
// from the true arc must not exceed `max_chord_error_m`.
//
// A sampled circle is a polygon: between two samples the trajectory cuts
// the chord, and the gap at the midpoint is
//
//     e = r (1 - cos(pi/N))
//
// so N = pi / acos(1 - e/r). Choosing N from the geometry rather than by
// hand is what keeps a stated tolerance meaningful — a 5 mm requirement is
// nonsense if the samples are 30 mm apart, because the interpolation alone
// misses by more than that. Clamped to at least 8 samples so a very coarse
// tolerance still yields a recognisable circle, and to 512 so a very tight
// one cannot silently produce an enormous factor graph.
int CircleSamplesForChordError(double radius_m, double max_chord_error_m);

// Two orthonormal vectors spanning the plane whose normal is `normal`.
// Exposed for the tests, which recover the circle's geometry from its
// samples rather than trusting the generator's own arithmetic.
void PlaneBasis(const Eigen::Vector3d& normal, Eigen::Vector3d& u, Eigen::Vector3d& v);
