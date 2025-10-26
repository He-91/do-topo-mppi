#include "path_searching/mppi_planner.h"
#include <cmath>
#include <algorithm>
#include <numeric>

using namespace std;
using namespace Eigen;

namespace ego_planner {

MPPIPlanner::MPPIPlanner() 
    : num_samples_(1000), horizon_steps_(20), dt_(0.1), lambda_(1.0),
      sigma_pos_(0.2), sigma_vel_(0.5), sigma_acc_(1.0),
      w_obstacle_(100.0), w_smoothness_(10.0), w_goal_(50.0), w_velocity_(20.0),
      max_velocity_(3.0), max_acceleration_(3.0),
      generator_(std::random_device{}()), normal_dist_(0.0, 1.0) {
}

MPPIPlanner::~MPPIPlanner() {
}

void MPPIPlanner::init(ros::NodeHandle& nh, GridMap::Ptr grid_map) {
    grid_map_ = grid_map;
    
    // Initialize visualization publishers
    mppi_trajectories_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/mppi_trajectories", 10);
    optimal_trajectory_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/mppi_optimal_trajectory", 10);
    
    // Get frame_id from node parameter, default to "world" if not set
    nh.param("grid_map/frame_id", frame_id_, std::string("world"));
    
    ROS_INFO("[MPPI] Initialized with %d samples, horizon %d steps, dt %.3f", 
             num_samples_, horizon_steps_, dt_);
    ROS_INFO("[MPPI] Initialized publishers on topics '/mppi_trajectories' and '/mppi_optimal_trajectory'");
    ROS_INFO("[MPPI] Using frame_id: %s", frame_id_.c_str());
}

bool MPPIPlanner::planTrajectory(const Vector3d& start_pos,
                                const Vector3d& start_vel,
                                const Vector3d& goal_pos,
                                const Vector3d& goal_vel,
                                MPPITrajectory& optimal_trajectory) {
    
    vector<MPPITrajectory> trajectories(num_samples_);
    double min_cost = std::numeric_limits<double>::max();
    
    // Generate rollout trajectories
    for (int i = 0; i < num_samples_; ++i) {
        trajectories[i].resize(horizon_steps_);
        rolloutTrajectory(start_pos, start_vel, goal_pos, goal_vel, trajectories[i]);
        
        double cost = calculateTrajectoryCost(trajectories[i], goal_pos, goal_vel);
        trajectories[i].cost = cost;
        
        if (cost < min_cost) {
            min_cost = cost;
        }
    }
    
    if (min_cost >= std::numeric_limits<double>::max()) {
        ROS_WARN("[MPPI] All trajectories have infinite cost");
        return false;
    }
    
    // Calculate importance weights using exponential of negative cost
    double weight_sum = 0.0;
    for (auto& traj : trajectories) {
        traj.weight = exp(-(traj.cost - min_cost) / lambda_);
        weight_sum += traj.weight;
    }
    
    // Normalize weights
    if (weight_sum > 1e-8) {
        for (auto& traj : trajectories) {
            traj.weight /= weight_sum;
        }
    } else {
        ROS_WARN("[MPPI] Weight sum too small, using uniform weights");
        for (auto& traj : trajectories) {
            traj.weight = 1.0 / num_samples_;
        }
    }
    
    // Compute weighted average trajectory
    optimal_trajectory = weightedAverage(trajectories);
    
    // Visualize trajectories
    visualizeTrajectories(trajectories);
    visualizeOptimalTrajectory(optimal_trajectory);
    
    ROS_DEBUG("[MPPI] Generated optimal trajectory with cost: %f", optimal_trajectory.cost);
    return true;
}

// ✅ NEW: planTrajectory with initial path guidance
bool MPPIPlanner::planTrajectory(const Vector3d& start_pos,
                                const Vector3d& start_vel,
                                const Vector3d& goal_pos,
                                const Vector3d& goal_vel,
                                const vector<Vector3d>& initial_path,
                                MPPITrajectory& optimal_trajectory) {
    
    vector<MPPITrajectory> trajectories(num_samples_);
    double min_cost = std::numeric_limits<double>::max();
    
    // Generate rollout trajectories guided by initial path
    for (int i = 0; i < num_samples_; ++i) {
        trajectories[i].resize(horizon_steps_);
        rolloutTrajectory(start_pos, start_vel, goal_pos, goal_vel, initial_path, trajectories[i]);
        
        double cost = calculateTrajectoryCost(trajectories[i], goal_pos, goal_vel);
        trajectories[i].cost = cost;
        
        if (cost < min_cost) {
            min_cost = cost;
        }
    }
    
    if (min_cost >= std::numeric_limits<double>::max()) {
        ROS_WARN("[MPPI] All guided trajectories have infinite cost");
        return false;
    }
    
    // Calculate importance weights
    double weight_sum = 0.0;
    for (auto& traj : trajectories) {
        traj.weight = exp(-(traj.cost - min_cost) / lambda_);
        weight_sum += traj.weight;
    }
    
    // Normalize weights
    if (weight_sum > 1e-8) {
        for (auto& traj : trajectories) {
            traj.weight /= weight_sum;
        }
    } else {
        ROS_WARN("[MPPI] Weight sum too small, using uniform weights");
        for (auto& traj : trajectories) {
            traj.weight = 1.0 / num_samples_;
        }
    }
    
    // Compute weighted average trajectory
    optimal_trajectory = weightedAverage(trajectories);
    
    // Visualize
    visualizeTrajectories(trajectories);
    visualizeOptimalTrajectory(optimal_trajectory);
    
    ROS_INFO("[MPPI] Guided trajectory with cost: %.3f (using %zu waypoints)", 
             optimal_trajectory.cost, initial_path.size());
    return true;
}

void MPPIPlanner::rolloutTrajectory(const Vector3d& start_pos,
                                   const Vector3d& start_vel,
                                   const Vector3d& goal_pos,
                                   const Vector3d& goal_vel,
                                   MPPITrajectory& trajectory) {
    // Initialize trajectory
    trajectory.positions[0] = start_pos;
    trajectory.velocities[0] = start_vel;
    trajectory.accelerations[0] = Vector3d::Zero();
    
    // Generate noisy control inputs and rollout dynamics
    for (int t = 1; t < horizon_steps_; ++t) {
        // Calculate nominal control towards goal
        Vector3d pos_error = goal_pos - trajectory.positions[t-1];
        Vector3d vel_error = goal_vel - trajectory.velocities[t-1];
        
        // Simple PD control for nominal trajectory
        Vector3d nominal_acc = 2.0 * pos_error + 1.0 * vel_error;
        
        // Add noise to control
        Vector3d noise_acc(
            sigma_acc_ * normal_dist_(generator_),
            sigma_acc_ * normal_dist_(generator_),
            sigma_acc_ * normal_dist_(generator_)
        );
        
        trajectory.accelerations[t] = nominal_acc + noise_acc;
        
        // Apply dynamic constraints
        constrainDynamics(trajectory.velocities[t-1], trajectory.accelerations[t]);
        
        // Forward integrate dynamics
        trajectory.velocities[t] = trajectory.velocities[t-1] + trajectory.accelerations[t] * dt_;
        trajectory.positions[t] = trajectory.positions[t-1] + trajectory.velocities[t-1] * dt_ + 
                                 0.5 * trajectory.accelerations[t] * dt_ * dt_;
        
        // Add position noise
        Vector3d pos_noise(
            sigma_pos_ * normal_dist_(generator_),
            sigma_pos_ * normal_dist_(generator_),
            sigma_pos_ * normal_dist_(generator_)
        );
        trajectory.positions[t] += pos_noise;
        
        // Add velocity noise
        Vector3d vel_noise(
            sigma_vel_ * normal_dist_(generator_),
            sigma_vel_ * normal_dist_(generator_),
            sigma_vel_ * normal_dist_(generator_)
        );
        trajectory.velocities[t] += vel_noise;
        
        // Constrain velocities
        if (trajectory.velocities[t].norm() > max_velocity_) {
            trajectory.velocities[t] = trajectory.velocities[t].normalized() * max_velocity_;
        }
    }
}

// ✅ NEW: rolloutTrajectory with path guidance
void MPPIPlanner::rolloutTrajectory(const Vector3d& start_pos,
                                   const Vector3d& start_vel,
                                   const Vector3d& goal_pos,
                                   const Vector3d& goal_vel,
                                   const vector<Vector3d>& guide_path,
                                   MPPITrajectory& trajectory) {
    // Initialize trajectory
    trajectory.positions[0] = start_pos;
    trajectory.velocities[0] = start_vel;
    trajectory.accelerations[0] = Vector3d::Zero();
    
    // Precompute path segment lengths for interpolation
    vector<double> segment_lengths;
    double total_length = 0.0;
    if (guide_path.size() >= 2) {
        for (size_t i = 1; i < guide_path.size(); ++i) {
            double len = (guide_path[i] - guide_path[i-1]).norm();
            segment_lengths.push_back(len);
            total_length += len;
        }
    }
    
    // Generate noisy control inputs guided by path
    for (int t = 1; t < horizon_steps_; ++t) {
        // Find target point on guide path based on time
        Vector3d target_point = goal_pos;
        
        if (!guide_path.empty() && total_length > 1e-3) {
            // Interpolate along guide path based on progress ratio
            double progress_ratio = (double)t / horizon_steps_;
            double target_dist = progress_ratio * total_length;
            
            double accumulated_dist = 0.0;
            for (size_t i = 0; i < segment_lengths.size(); ++i) {
                if (accumulated_dist + segment_lengths[i] >= target_dist) {
                    // Interpolate within this segment
                    double local_ratio = (target_dist - accumulated_dist) / segment_lengths[i];
                    target_point = guide_path[i] + local_ratio * (guide_path[i+1] - guide_path[i]);
                    break;
                }
                accumulated_dist += segment_lengths[i];
            }
        }
        
        // Calculate control towards target point (path-guided PD)
        Vector3d pos_error = target_point - trajectory.positions[t-1];
        Vector3d vel_error = goal_vel - trajectory.velocities[t-1];
        
        Vector3d nominal_acc = 2.0 * pos_error + 1.0 * vel_error;
        
        // Add noise to control
        Vector3d noise_acc(
            sigma_acc_ * normal_dist_(generator_),
            sigma_acc_ * normal_dist_(generator_),
            sigma_acc_ * normal_dist_(generator_)
        );
        
        trajectory.accelerations[t] = nominal_acc + noise_acc;
        
        // Apply dynamic constraints
        constrainDynamics(trajectory.velocities[t-1], trajectory.accelerations[t]);
        
        // Forward integrate dynamics
        trajectory.velocities[t] = trajectory.velocities[t-1] + trajectory.accelerations[t] * dt_;
        trajectory.positions[t] = trajectory.positions[t-1] + trajectory.velocities[t-1] * dt_ + 
                                 0.5 * trajectory.accelerations[t] * dt_ * dt_;
        
        // Add position noise
        Vector3d pos_noise(
            sigma_pos_ * normal_dist_(generator_),
            sigma_pos_ * normal_dist_(generator_),
            sigma_pos_ * normal_dist_(generator_)
        );
        trajectory.positions[t] += pos_noise;
        
        // Add velocity noise
        Vector3d vel_noise(
            sigma_vel_ * normal_dist_(generator_),
            sigma_vel_ * normal_dist_(generator_),
            sigma_vel_ * normal_dist_(generator_)
        );
        trajectory.velocities[t] += vel_noise;
        
        // Constrain velocities
        if (trajectory.velocities[t].norm() > max_velocity_) {
            trajectory.velocities[t] = trajectory.velocities[t].normalized() * max_velocity_;
        }
    }
}

double MPPIPlanner::calculateTrajectoryCost(const MPPITrajectory& trajectory,
                                          const Vector3d& goal_pos,
                                          const Vector3d& goal_vel) {
    double total_cost = 0.0;
    
    for (int t = 0; t < trajectory.size(); ++t) {
        // ✅ Phase 3: Use ESDF for O(1) obstacle distance query
        double dist = grid_map_->getDistance(trajectory.positions[t]);
        
        // Obstacle cost - exponentially increase as we get closer
        total_cost += w_obstacle_ * obstacleCost(trajectory.positions[t], dist);
        
        // Check for collision - infinite cost if inside obstacle (negative distance)
        if (dist < 0.0) {
            return std::numeric_limits<double>::max();
        }
    }
    
    // Smoothness cost
    total_cost += w_smoothness_ * smoothnessCost(trajectory);
    
    // Goal reaching cost
    total_cost += w_goal_ * goalCost(trajectory, goal_pos, goal_vel);
    
    // Velocity cost
    total_cost += w_velocity_ * velocityCost(trajectory, goal_vel);
    
    return total_cost;
}

double MPPIPlanner::obstacleCost(const Vector3d& position) {
    // ✅ Phase 3: Use ESDF for O(1) distance query instead of O(n³) sampling
    double dist = grid_map_->getDistance(position);
    return obstacleCost(position, dist);
}

double MPPIPlanner::obstacleCost(const Vector3d& position, double dist) {
    // ✅ Phase 3: ESDF-based obstacle cost - O(1) instead of O(n³)
    // 
    // Previously: Sampled 11×11×11 = 1331 points around position (O(n³))
    // Now: Single ESDF lookup (O(1)) - ~1000x faster!
    //
    // Cost function: Exponentially increases as distance decreases
    // - dist > safety_distance: no cost (0.0)
    // - dist < safety_distance: exponential cost increase
    // - dist < 0: inside obstacle (handled in calculateTrajectoryCost)
    
    const double safety_distance = 1.0;  // Safe distance from obstacles (meters)
    const double cost_scale = 1.0;       // Cost scaling factor
    
    if (dist >= safety_distance) {
        return 0.0;  // Safe distance, no cost
    }
    
    if (dist < 0.0) {
        // Inside obstacle - return very high cost
        // (infinite cost is handled in calculateTrajectoryCost)
        return 1000.0;
    }
    
    // Exponential cost: cost = scale * exp(-dist / sigma)
    // As dist → 0, cost → infinity
    // As dist → safety_distance, cost → 0
    double normalized_dist = dist / safety_distance;
    double cost = cost_scale * std::exp(-normalized_dist * 5.0) / (dist + 0.01);
    
    return cost;
}

double MPPIPlanner::smoothnessCost(const MPPITrajectory& trajectory) {
    double cost = 0.0;
    
    // Acceleration smoothness
    for (int t = 1; t < trajectory.size(); ++t) {
        Vector3d acc_diff = trajectory.accelerations[t] - trajectory.accelerations[t-1];
        cost += acc_diff.squaredNorm();
    }
    
    // Velocity smoothness  
    for (int t = 1; t < trajectory.size(); ++t) {
        Vector3d vel_diff = trajectory.velocities[t] - trajectory.velocities[t-1];
        cost += 0.5 * vel_diff.squaredNorm();
    }
    
    return cost;
}

double MPPIPlanner::goalCost(const MPPITrajectory& trajectory,
                           const Vector3d& goal_pos,
                           const Vector3d& goal_vel) {
    // Terminal state cost
    Vector3d final_pos = trajectory.positions.back();
    Vector3d final_vel = trajectory.velocities.back();
    
    double pos_error = (final_pos - goal_pos).squaredNorm();
    double vel_error = (final_vel - goal_vel).squaredNorm();
    
    return pos_error + 0.5 * vel_error;
}

double MPPIPlanner::velocityCost(const MPPITrajectory& trajectory,
                               const Vector3d& desired_vel) {
    double cost = 0.0;
    
    for (int t = 0; t < trajectory.size(); ++t) {
        Vector3d vel_error = trajectory.velocities[t] - desired_vel;
        cost += vel_error.squaredNorm();
    }
    
    return cost / trajectory.size();
}

void MPPIPlanner::constrainDynamics(Vector3d& velocity, Vector3d& acceleration) {
    // Limit acceleration magnitude
    if (acceleration.norm() > max_acceleration_) {
        acceleration = acceleration.normalized() * max_acceleration_;
    }
    
    // Predict next velocity and limit if necessary
    Vector3d next_vel = velocity + acceleration * dt_;
    if (next_vel.norm() > max_velocity_) {
        next_vel = next_vel.normalized() * max_velocity_;
        acceleration = (next_vel - velocity) / dt_;
    }
}

MPPITrajectory MPPIPlanner::weightedAverage(const vector<MPPITrajectory>& trajectories) {
    MPPITrajectory avg_trajectory;
    avg_trajectory.resize(horizon_steps_);
    
    // Initialize with zeros
    for (int t = 0; t < horizon_steps_; ++t) {
        avg_trajectory.positions[t] = Vector3d::Zero();
        avg_trajectory.velocities[t] = Vector3d::Zero();
        avg_trajectory.accelerations[t] = Vector3d::Zero();
    }
    
    // Weighted average
    double total_cost = 0.0;
    for (const auto& traj : trajectories) {
        for (int t = 0; t < horizon_steps_; ++t) {
            avg_trajectory.positions[t] += traj.weight * traj.positions[t];
            avg_trajectory.velocities[t] += traj.weight * traj.velocities[t];
            avg_trajectory.accelerations[t] += traj.weight * traj.accelerations[t];
        }
        total_cost += traj.weight * traj.cost;
    }
    
    avg_trajectory.cost = total_cost;
    
    return avg_trajectory;
}

bool MPPIPlanner::planLocalPath(const Vector3d& start_pos,
                               const Vector3d& goal_pos,
                               vector<Vector3d>& path_points) {
    path_points.clear();
    
    // Use a simplified MPPI for local path planning
    Vector3d start_vel = Vector3d::Zero();
    Vector3d goal_vel = Vector3d::Zero();
    
    // Reduce samples and horizon for faster local planning
    int original_samples = num_samples_;
    int original_horizon = horizon_steps_;
    num_samples_ = 200;  // Fewer samples for speed
    horizon_steps_ = 10; // Shorter horizon for local planning
    
    MPPITrajectory local_trajectory;
    bool success = planTrajectory(start_pos, start_vel, goal_pos, goal_vel, local_trajectory);
    
    // Restore original parameters
    num_samples_ = original_samples;
    horizon_steps_ = original_horizon;
    
    if (!success || local_trajectory.positions.empty()) {
        ROS_WARN("[MPPI] Local path planning failed");
        return false;
    }
    
    // Extract path points from trajectory (subsample for efficiency)
    int step = std::max(1, (int)(local_trajectory.positions.size() / 10)); // Max 10 points
    for (size_t i = 0; i < local_trajectory.positions.size(); i += step) {
        path_points.push_back(local_trajectory.positions[i]);
    }
    
    // Always include the goal point
    if (path_points.empty() || (path_points.back() - goal_pos).norm() > 0.1) {
        path_points.push_back(goal_pos);
    }
    
    ROS_DEBUG("[MPPI] Generated local path with %zu points", path_points.size());
    return true;
}

void MPPIPlanner::visualizeTrajectories(const vector<MPPITrajectory>& trajectories) {
    if (trajectories.empty()) return;
    
    ROS_DEBUG("[MPPI] Visualizing %zu sample trajectories with frame_id: %s", trajectories.size(), frame_id_.c_str());
    
    visualization_msgs::MarkerArray marker_array;
    
    // Clear previous markers
    visualization_msgs::Marker clear_marker;
    clear_marker.header.frame_id = frame_id_;
    clear_marker.header.stamp = ros::Time::now();
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    
    // Visualize a subset of sample trajectories (to avoid overwhelming RViz)
    int visualization_step = std::max(1, (int)(trajectories.size() / 50));  // Show at most 50 trajectories
    
    for (size_t i = 0; i < trajectories.size(); i += visualization_step) {
        if (trajectories[i].positions.empty()) continue;
        
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = frame_id_;
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "mppi_sample_trajectories";
        line_marker.id = i;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.pose.orientation.w = 1.0;
        
        // Color based on trajectory cost (red = high cost, green = low cost)
        double normalized_cost = trajectories[i].weight; // Use weight for coloring
        line_marker.color.r = 1.0 - normalized_cost;
        line_marker.color.g = normalized_cost;
        line_marker.color.b = 0.2;
        line_marker.color.a = 0.3;  // Make them semi-transparent
        line_marker.scale.x = 0.05;  // Thin lines for sample trajectories
        
        for (const auto& pos : trajectories[i].positions) {
            geometry_msgs::Point p;
            p.x = pos.x();
            p.y = pos.y();
            p.z = pos.z();
            line_marker.points.push_back(p);
        }
        
        marker_array.markers.push_back(line_marker);
    }
    
    mppi_trajectories_pub_.publish(marker_array);
    ROS_DEBUG("[MPPI] Published %zu sample trajectory markers", marker_array.markers.size() - 1);
}

void MPPIPlanner::visualizeOptimalTrajectory(const MPPITrajectory& trajectory) {
    if (trajectory.positions.empty()) return;
    
    ROS_DEBUG("[MPPI] Visualizing optimal trajectory with frame_id: %s", frame_id_.c_str());
    
    visualization_msgs::MarkerArray marker_array;
    
    // Clear previous markers
    visualization_msgs::Marker clear_marker;
    clear_marker.header.frame_id = frame_id_;
    clear_marker.header.stamp = ros::Time::now();
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    
    // Optimal trajectory line
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = frame_id_;
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = "mppi_optimal_trajectory";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.pose.orientation.w = 1.0;
    
    // Bright orange for optimal trajectory
    line_marker.color.r = 1.0;
    line_marker.color.g = 0.5;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;
    line_marker.scale.x = 0.15;  // Thicker line for optimal trajectory
    
    for (const auto& pos : trajectory.positions) {
        geometry_msgs::Point p;
        p.x = pos.x();
        p.y = pos.y();
        p.z = pos.z();
        line_marker.points.push_back(p);
    }
    
    marker_array.markers.push_back(line_marker);
    
    // Add velocity vectors as arrows (optional, show every few steps)
    int arrow_step = std::max(1, horizon_steps_ / 5);  // Show 5 arrows max
    for (int i = 0; i < trajectory.size(); i += arrow_step) {
        if (trajectory.velocities[i].norm() < 0.1) continue;  // Skip very small velocities
        
        visualization_msgs::Marker arrow_marker;
        arrow_marker.header.frame_id = frame_id_;
        arrow_marker.header.stamp = ros::Time::now();
        arrow_marker.ns = "mppi_velocity_arrows";
        arrow_marker.id = i;
        arrow_marker.type = visualization_msgs::Marker::ARROW;
        arrow_marker.action = visualization_msgs::Marker::ADD;
        
        // Arrow position
        arrow_marker.pose.position.x = trajectory.positions[i].x();
        arrow_marker.pose.position.y = trajectory.positions[i].y();
        arrow_marker.pose.position.z = trajectory.positions[i].z();
        
        // Arrow orientation (pointing in velocity direction)
        Vector3d vel_normalized = trajectory.velocities[i].normalized();
        double yaw = atan2(vel_normalized.y(), vel_normalized.x());
        double pitch = asin(vel_normalized.z());
        
        arrow_marker.pose.orientation.x = 0;
        arrow_marker.pose.orientation.y = sin(pitch/2);
        arrow_marker.pose.orientation.z = sin(yaw/2) * cos(pitch/2);
        arrow_marker.pose.orientation.w = cos(yaw/2) * cos(pitch/2);
        
        // Arrow scale based on velocity magnitude
        double vel_mag = trajectory.velocities[i].norm();
        arrow_marker.scale.x = vel_mag * 0.5;  // Arrow length
        arrow_marker.scale.y = 0.05;  // Arrow width
        arrow_marker.scale.z = 0.05;  // Arrow height
        
        // Blue color for velocity arrows
        arrow_marker.color.r = 0.0;
        arrow_marker.color.g = 0.3;
        arrow_marker.color.b = 1.0;
        arrow_marker.color.a = 0.7;
        
        marker_array.markers.push_back(arrow_marker);
    }
    
    optimal_trajectory_pub_.publish(marker_array);
    ROS_DEBUG("[MPPI] Published optimal trajectory with %zu markers", marker_array.markers.size() - 1);
}

} // namespace ego_planner