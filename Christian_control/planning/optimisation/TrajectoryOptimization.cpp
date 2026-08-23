#include "TrajectoryOptimization.h"

#include <set>
#include <stdexcept>

#include <gtsam/nonlinear/NonlinearEquality.h>

gtsam::SharedNoiseModel PoseNoiseModel(const Eigen::Vector3d& rotation_sigma_rad,
                                       const Eigen::Vector3d& translation_sigma_m) {
    // ROTATION FIRST. gpmp2::GaussianPriorWorkspacePose's error is
    // des_pose.logmap(actual), and gtsam's Pose3 logmap returns
    // [rotation; translation]. Swapping these is silent — six plausible
    // numbers, a converging solve, and the wrong axes weighted — which is
    // why this ordering is written once and pinned by
    // test_pose_noise_ordering rather than trusted to a comment.
    gtsam::Vector6 sigmas;
    sigmas << rotation_sigma_rad, translation_sigma_m;
    return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

gtsam::Matrix SelfCollisionData(double collision_sigma) {
    gtsam::Matrix data(kSelfCollisionPairs.size(), 4);
    for (std::size_t row = 0; row < kSelfCollisionPairs.size(); ++row) {
        const SelfCollisionPair& pair = kSelfCollisionPairs[row];
        data(row, 0) = static_cast<double>(pair.first_sphere_index);
        data(row, 1) = static_cast<double>(pair.second_sphere_index);
        data(row, 2) = pair.minimum_surface_clearance_m;
        data(row, 3) = collision_sigma;
    }
    return data;
}

OptimizeTrajectory::OptimizeTrajectory() {}

TrajectoryResult OptimizeTrajectory::optimizeJointTrajectory(
    const gpmp2::ArmModel& arm_model,
    const std::vector<NamedObstacleField>& obstacle_fields,
    const gtsam::Values& init_values,
    const gtsam::Vector& start_config,
    const std::optional<gtsam::Vector>& start_vel,
    const std::optional<gtsam::Vector>& terminal_config,
    const JointLimits& pos_limits,
    const JointLimits& vel_limits,
    const size_t total_time_step,
    const double total_time_sec,
    const OptimizerTuning& tuning,
    const double scene_collision_sigma,
    const double self_collision_sigma,
    const double target_dt) {

    std::cout << "Creating arm trajectory..." << std::endl;

    std::vector<std::string> factor_keys;
    std::unordered_map<std::string, double> init_factor_costs;
    std::unordered_map<std::string, double> final_factor_costs;

    // Trajectory parameters

    double delta_t = total_time_sec / total_time_step;

    // GP and optimization parameters
    gtsam::Matrix Qc = gtsam::Matrix::Identity(7, 7) * tuning.qc_scale;
    auto Qc_model = gtsam::noiseModel::Gaussian::Covariance(Qc);
    auto vel_fix_model = gtsam::noiseModel::Isotropic::Sigma(7, 0.001);
    
    gtsam::Vector end_vel = gtsam::Vector::Zero(7);

    const gtsam::Matrix self_collision_data = SelfCollisionData(self_collision_sigma);
    
    
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial_values = init_values;
    initial_values.update(gtsam::Symbol('x', 0), start_config);
    if (terminal_config)
        initial_values.update(gtsam::Symbol('x', total_time_step),
                             *terminal_config);
    if (start_vel)
        initial_values.update(gtsam::Symbol('v', 0), *start_vel);
    
    for (size_t i = 0; i <= total_time_step; ++i) {
        gtsam::Symbol key_pos('x', i);
        gtsam::Symbol key_vel('v', i);
        
        // Start/end priors
        if (i == 0) {
            graph.add(gtsam::NonlinearEquality<gtsam::Vector>(key_pos, start_config));
            factor_keys.push_back("StartPosEquality");
            if (start_vel) {
                graph.add(gtsam::NonlinearEquality<gtsam::Vector>(key_vel, *start_vel));
                factor_keys.push_back("StartVelEquality");
            }
        
        } else if (i == total_time_step) {
            graph.add(gtsam::PriorFactor<gtsam::Vector>(key_vel, end_vel, vel_fix_model));
                    factor_keys.push_back("EndVelPrior");
        }           
        
        // Joint limits
        auto pos_limit_model = gtsam::noiseModel::Isotropic::Sigma(7, 0.001);
        gtsam::Vector limit_thresh = gtsam::Vector::Constant(7, 0.2);
        graph.add(gpmp2::JointLimitFactorVector(
            key_pos, pos_limit_model, pos_limits.lower, pos_limits.upper, limit_thresh));
        factor_keys.push_back("JointPosLimits");

        // Velocity limits
        auto vel_limit_model = gtsam::noiseModel::Isotropic::Sigma(7, 0.001);
        gtsam::Vector vel_limit_thresh = gtsam::Vector::Constant(7, 0.05);
        graph.add(gpmp2::VelocityLimitFactorVector(
            key_vel, vel_limit_model, vel_limits.upper, vel_limit_thresh));
        factor_keys.push_back("JointVelLimits");

        // Obstacle collision avoidance
        for (const auto& field : obstacle_fields) {
            graph.add(gpmp2::ObstacleSDFFactorArm(
                key_pos, *field.participating_arm, field.sdf,
                scene_collision_sigma, tuning.preferred_clearance_m));
            factor_keys.push_back("ObstacleFactor:" + field.id);
        }

        if (i > 0) {
            gtsam::Symbol key_pos_prev('x', i - 1);
            gtsam::Symbol key_vel_prev('v', i - 1);
            
            double delta_t_segment = delta_t / 5.0;  // divide segment into 5 parts
            for (int interp = 1; interp < 5; ++interp) {  // skip endpoints (checked at waypoints)
                double tau_check = interp * delta_t_segment;
                for (const auto& field : obstacle_fields) {
                    graph.add(gpmp2::ObstacleSDFFactorGPArm(
                        key_pos_prev, key_vel_prev, key_pos, key_vel,
                        *field.participating_arm, field.sdf, scene_collision_sigma,
                        tuning.preferred_clearance_m, Qc_model, delta_t,
                        tau_check));
                    factor_keys.push_back("ObstacleFactorGP:" + field.id);
                }
            }
        }

        graph.add(gpmp2::SelfCollisionArm(
            key_pos, arm_model, self_collision_data));
        factor_keys.push_back("SelfCollisionFactor");

        // GP priors
        if (i > 0) {
            gtsam::Symbol key_pos1('x', i - 1);
            gtsam::Symbol key_pos2('x', i);
            gtsam::Symbol key_vel1('v', i - 1);
            gtsam::Symbol key_vel2('v', i);
            
            graph.add(gpmp2::GaussianProcessPriorLinear(
                key_pos1, key_vel1, key_pos2, key_vel2, delta_t, Qc_model));
            factor_keys.push_back("GP_Prior");
        }
    }

    gtsam::Symbol final_key('x', total_time_step);
    if (terminal_config) {
        graph.add(gtsam::NonlinearEquality<gtsam::Vector>(
            final_key, *terminal_config));
        factor_keys.push_back("TerminalPosEquality");
    }

    for (size_t i = 0; i < graph.size(); ++i) {

        double constraint_error = graph.at(i)->error(initial_values);

        init_factor_costs[factor_keys[i]] += constraint_error;
    }

    // Setup optimizer
    gtsam::LevenbergMarquardtParams parameters;
    parameters.setVerbosity("none");
    parameters.setRelativeErrorTol(1e-6);
    parameters.setAbsoluteErrorTol(1e-6);
    parameters.setMaxIterations(tuning.max_iterations);
    parameters.setlambdaInitial(1e-5);
    parameters.setlambdaFactor(10.0);
    parameters.setlambdaUpperBound(1e6);
    
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, parameters);
    
    // Optimize
    gtsam::Values result = optimizer.optimize();
    
    for (size_t i = 0; i < graph.size(); ++i) {
  
        double constraint_error = graph.at(i)->error(result);
   
        final_factor_costs[factor_keys[i]] += constraint_error;
    }

    auto [densified_pos, densified_vel] = densifyTrajectory(result, Qc_model, delta_t, total_time_sec, target_dt);    
    
    // Formulate result
    TrajectoryResult trajectory_result;

    trajectory_result.dt = target_dt;
    trajectory_result.start_error = graph.error(initial_values);
    trajectory_result.final_error = graph.error(result);
    trajectory_result.trajectory_pos = densified_pos;
    trajectory_result.trajectory_vel = densified_vel;
    trajectory_result.start_costs = init_factor_costs;
    trajectory_result.final_costs = final_factor_costs;

    return trajectory_result;
}

TrajectoryResult OptimizeTrajectory::optimizeTaskTrajectory(
    const gpmp2::ArmModel& arm_model,
    const std::vector<NamedObstacleField>& obstacle_fields,
    const gtsam::Values& init_values,
    const std::vector<OptimisationWaypoint>& waypoints,
    const gtsam::Vector& start_config,
    const std::optional<gtsam::Vector>& start_vel,
    const std::optional<gtsam::Vector>& terminal_config,
    const std::vector<size_t>& zero_velocity_indices,
    const JointLimits& pos_limits,
    const JointLimits& vel_limits,
    const double total_time_sec,
    const OptimizerTuning& tuning,
    const double scene_collision_sigma,
    const double self_collision_sigma,
    const double target_dt) {

    if (waypoints.size() < 2)
        throw std::invalid_argument(
            "optimizeTaskTrajectory needs at least two waypoints, got " +
            std::to_string(waypoints.size()));
    size_t bad_index = 0;
    if (!WaypointTimesAreMonotonic(waypoints, &bad_index))
        throw std::invalid_argument(
            "optimizeTaskTrajectory waypoint times must start at zero and strictly "
            "increase; first offending index " + std::to_string(bad_index));

    // The last support state's index. Named for what it is rather than
    // taken as a parameter, so it cannot disagree with `waypoints`.
    const size_t total_time_step = waypoints.size() - 1;

    std::cout << "Creating arm trajectory..." << std::endl;

    std::vector<std::string> factor_keys;
    std::unordered_map<std::string, double> init_factor_costs;
    std::unordered_map<std::string, double> final_factor_costs;

    // Trajectory parameters

    double delta_t = total_time_sec / total_time_step;

    // GP and optimization parameters — from `tuning` (config/planner.yaml)
    // rather than hardcoded, so the same file governs path plans and
    // point-to-point plans. The defaults in OptimizerTuning are exactly the
    // literals this function used before, so the plumbing change alone
    // reproduces the previous behaviour.
    gtsam::Matrix Qc = gtsam::Matrix::Identity(7, 7) * tuning.qc_scale;
    auto Qc_model = gtsam::noiseModel::Gaussian::Covariance(Qc);
    auto vel_fix_model = gtsam::noiseModel::Isotropic::Sigma(7, 0.001);

    const gtsam::Vector rest_vel = gtsam::Vector::Zero(7);

    // Which support states are pinned to rest. The caller supplies the
    // task-path start and endpoint; a set makes a repeated index harmless.
    std::set<size_t> rest_indices(zero_velocity_indices.begin(),
                                  zero_velocity_indices.end());
    rest_indices.insert(total_time_step);
    gtsam::Values initial_values = init_values;
    initial_values.update(gtsam::Symbol('x', 0), start_config);
    if (terminal_config)
        initial_values.update(gtsam::Symbol('x', total_time_step),
                             *terminal_config);
    if (start_vel)
        initial_values.update(gtsam::Symbol('v', 0), *start_vel);

    const gtsam::Matrix self_collision_data = SelfCollisionData(self_collision_sigma);


    gtsam::NonlinearFactorGraph graph;

    for (size_t i = 0; i <= total_time_step; ++i) {
        gtsam::Symbol key_pos('x', i);
        gtsam::Symbol key_vel('v', i);

        if (i == 0) {
            graph.add(gtsam::NonlinearEquality<gtsam::Vector>(key_pos, start_config));
            factor_keys.push_back("StartPosEquality");
            if (start_vel) {
                graph.add(gtsam::NonlinearEquality<gtsam::Vector>(key_vel, *start_vel));
                factor_keys.push_back("StartVelEquality");
            }
        }
        // Rest states. Hermite through a zero-velocity support state
        // genuinely comes to a stop there, so no repeated waypoints (and no
        // zero-duration wire segments) are needed to dwell.
        if (rest_indices.count(i)) {
            graph.add(gtsam::PriorFactor<gtsam::Vector>(key_vel, rest_vel, vel_fix_model));
                    factor_keys.push_back("RestVelPrior");
        }

        // Joint limits
        auto pos_limit_model = gtsam::noiseModel::Isotropic::Sigma(7, 0.001);
        gtsam::Vector limit_thresh = gtsam::Vector::Constant(7, 0.2);
        graph.add(gpmp2::JointLimitFactorVector(
            key_pos, pos_limit_model, pos_limits.lower, pos_limits.upper, limit_thresh));
        factor_keys.push_back("JointPosLimits");

        // Velocity limits
        auto vel_limit_model = gtsam::noiseModel::Isotropic::Sigma(7, 0.001);
        gtsam::Vector vel_limit_thresh = gtsam::Vector::Constant(7, 0.1);
        graph.add(gpmp2::VelocityLimitFactorVector(
            key_vel, vel_limit_model, vel_limits.upper, vel_limit_thresh));
        factor_keys.push_back("JointVelLimits");

        // Obstacle collision avoidance
        for (const auto& field : obstacle_fields) {
            graph.add(gpmp2::ObstacleSDFFactorArm(
                key_pos, *field.participating_arm, field.sdf,
                scene_collision_sigma, tuning.preferred_clearance_m));
            factor_keys.push_back("ObstacleFactor:" + field.id);
        }

        if (i > 0) {
            gtsam::Symbol key_pos_prev('x', i - 1);
            gtsam::Symbol key_vel_prev('v', i - 1);
            
            double delta_t_segment = delta_t / 5.0;  // divide segment into 5 parts
            for (int interp = 1; interp < 5; ++interp) {  // skip endpoints (checked at waypoints)
                double tau_check = interp * delta_t_segment;
                for (const auto& field : obstacle_fields) {
                    graph.add(gpmp2::ObstacleSDFFactorGPArm(
                        key_pos_prev, key_vel_prev, key_pos, key_vel,
                        *field.participating_arm, field.sdf, scene_collision_sigma,
                        tuning.preferred_clearance_m, Qc_model, delta_t,
                        tau_check));
                    factor_keys.push_back("ObstacleFactorGP:" + field.id);
                }
            }
        }


        graph.add(gpmp2::SelfCollisionArm(
            key_pos, arm_model, self_collision_data));
        factor_keys.push_back("SelfCollisionFactor");

        // GP priors
        if (i > 0) {
            gtsam::Symbol key_pos1('x', i - 1);
            gtsam::Symbol key_pos2('x', i);
            gtsam::Symbol key_vel1('v', i - 1);
            gtsam::Symbol key_vel2('v', i);
            
            graph.add(gpmp2::GaussianProcessPriorLinear(
                key_pos1, key_vel1, key_pos2, key_vel2, delta_t, Qc_model));
            factor_keys.push_back("GP_Prior");
        }
    }
    
    // Pose priors, one per waypoint that asked for one. A waypoint with no
    // prior is FREE: only smoothness, joint limits and obstacle avoidance
    // shape it. That is what lets one graph hold an unconstrained approach
    // and a constrained task path, which the previous all-or-nothing
    // `target_pose_only` flag could not express.
    //
    // Every prior is built through PoseNoiseModel, so gpmp2's
    // [rotation; translation] ordering lives in exactly one place.
    size_t constrained_count = 0;
    for (size_t i = 0; i <= total_time_step; i++) {
        const auto& prior = waypoints[i].pose_prior;
        if (!prior || (terminal_config && i == total_time_step)) continue;
        graph.add(gpmp2::GaussianPriorWorkspacePoseArm(
            gtsam::Symbol('x', i), arm_model, 6,
            gtsam::Pose3(gtsam::Rot3(prior->target.linear()),
                         gtsam::Point3(prior->target.translation())),
            PoseNoiseModel(prior->rotation_sigma_rad, prior->translation_sigma_m)));
        factor_keys.push_back("PoseFactor");
        ++constrained_count;
    }
    if (terminal_config) {
        graph.add(gtsam::NonlinearEquality<gtsam::Vector>(
            gtsam::Symbol('x', total_time_step), *terminal_config));
        factor_keys.push_back("TerminalPosEquality");
    }
    if (constrained_count == 0)
        throw std::invalid_argument(
            "optimizeTaskTrajectory was given no pose priors at all — nothing "
            "would pull the trajectory toward the requested path");


    for (size_t i = 0; i < graph.size(); ++i) {
      
        double constraint_error = graph.at(i)->error(initial_values);

        init_factor_costs[factor_keys[i]] += constraint_error;    
    }
        
    // Setup optimizer
    gtsam::LevenbergMarquardtParams parameters;
    parameters.setVerbosity("none");
    parameters.setRelativeErrorTol(1e-6);
    parameters.setAbsoluteErrorTol(1e-6);
    parameters.setMaxIterations(1000);
    parameters.setlambdaInitial(1e-5);
    parameters.setlambdaFactor(10.0);
    parameters.setlambdaUpperBound(1e6);
    
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, parameters);
    
    // Optimize
    gtsam::Values result = optimizer.optimize();
    
    for (size_t i = 0; i < graph.size(); ++i) {
  
        double constraint_error = graph.at(i)->error(result);
   
        final_factor_costs[factor_keys[i]] += constraint_error;
    }

    auto [densified_pos, densified_vel] = densifyTrajectory(result, Qc_model, delta_t, total_time_sec, target_dt);    
    
    // Formulate result
    TrajectoryResult trajectory_result;

    trajectory_result.dt = target_dt;
    trajectory_result.start_error = graph.error(initial_values);
    trajectory_result.final_error = graph.error(result);
    trajectory_result.trajectory_pos = densified_pos;
    trajectory_result.trajectory_vel = densified_vel;
    trajectory_result.start_costs = init_factor_costs;
    trajectory_result.final_costs = final_factor_costs;

    return trajectory_result;
}


std::pair<std::vector<gtsam::Vector>, std::vector<gtsam::Vector>> OptimizeTrajectory::densifyTrajectory(
    const gtsam::Values& optimized_values,
    const gtsam::SharedNoiseModel& Qc_model,
    double delta_t,
    double total_time_sec,
    double target_dt) {
    
    std::vector<gtsam::Vector> dense_trajectory;
    std::vector<gtsam::Vector> dense_velocities;
    
    // Calculate total points needed for 1000Hz
    size_t total_dense_points = static_cast<size_t>(total_time_sec / target_dt) + 1;
    
    // Calculate number of interpolation points between each pair of waypoints
    size_t inter_step = static_cast<size_t>(delta_t / target_dt) - 1;
    
    // Find the maximum waypoint index in the optimized values
    size_t max_waypoint_idx = 0;
    gtsam::KeyVector key_vec = optimized_values.keys();
    for (const auto& key : key_vec) {
        if (gtsam::Symbol(key).chr() == 'x') {
            max_waypoint_idx = std::max(max_waypoint_idx, gtsam::Symbol(key).index());
        }
    }
    
    // Use GPMP2's interpolateArmTraj function to get densified values
    gtsam::Values dense_values = gpmp2::interpolateArmTraj(
        optimized_values,
        Qc_model,
        delta_t,
        inter_step,
        0,                    // start_index
        max_waypoint_idx      // end_index
    );
    
    // Convert gtsam::Values to std::vector<gtsam::Vector> for both positions and velocities
    // The interpolated values will have sequential indices starting from 0
    size_t dense_idx = 0;
    while (dense_values.exists(gtsam::Symbol('x', dense_idx))) {
        // Extract position
        gtsam::Vector config = dense_values.at<gtsam::Vector>(gtsam::Symbol('x', dense_idx));
        dense_trajectory.push_back(config);
        
        // Extract velocity
        if (dense_values.exists(gtsam::Symbol('v', dense_idx))) {
            gtsam::Vector velocity = dense_values.at<gtsam::Vector>(gtsam::Symbol('v', dense_idx));
            dense_velocities.push_back(velocity);
        }
        
        dense_idx++;
    }

    if (!dense_velocities.empty() &&
        optimized_values.exists(gtsam::Symbol('v', 0)))
        dense_velocities.front() =
            optimized_values.at<gtsam::Vector>(gtsam::Symbol('v', 0));

    std::cout << "Generated " << dense_trajectory.size() << " dense position waypoints" << std::endl;
    std::cout << "Generated " << dense_velocities.size() << " dense velocity waypoints" << std::endl;
    
    // Verify timing
    double actual_dt = total_time_sec / (dense_trajectory.size() - 1);
    double actual_freq = 1.0 / actual_dt;
    std::cout << "Actual frequency: " << actual_freq << " Hz" << std::endl;
    
    return std::make_pair(dense_trajectory, dense_velocities);
}
