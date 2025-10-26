#ifndef _MPPI_PLANNER_H_
#define _MPPI_PLANNER_H_

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <vector>
#include <random>
#include <memory>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace ego_planner {

struct MPPITrajectory {
    std::vector<Eigen::Vector3d> positions;
    std::vector<Eigen::Vector3d> velocities;
    std::vector<Eigen::Vector3d> accelerations;
    double cost;
    double weight;
    
    MPPITrajectory() : cost(0.0), weight(0.0) {}
    
    void resize(int size) {
        positions.resize(size);
        velocities.resize(size);
        accelerations.resize(size);
    }
    
    int size() const { return positions.size(); }
};

class MPPIPlanner {
private:
    GridMap::Ptr grid_map_;
    
    // Visualization
    ros::Publisher mppi_trajectories_pub_;
    ros::Publisher optimal_trajectory_pub_;
    std::string frame_id_;
    
    // MPPI parameters
    int num_samples_;          // Number of rollout samples
    int horizon_steps_;        // Planning horizon steps
    double dt_;                // Time step
    double lambda_;            // Temperature parameter for importance sampling
    double sigma_pos_;         // Position noise standard deviation
    double sigma_vel_;         // Velocity noise standard deviation
    double sigma_acc_;         // Acceleration noise standard deviation
    
    // Cost weights
    double w_obstacle_;        // Obstacle avoidance weight
    double w_smoothness_;      // Smoothness weight
    double w_goal_;           // Goal reaching weight
    double w_velocity_;       // Velocity tracking weight
    
    // Vehicle dynamics parameters
    double max_velocity_;     // Maximum velocity
    double max_acceleration_; // Maximum acceleration
    
    // Random number generator
    std::mt19937 generator_;
    std::normal_distribution<double> normal_dist_;
    
    // Helper functions
    void rolloutTrajectory(const Eigen::Vector3d& start_pos,
                          const Eigen::Vector3d& start_vel,
                          const Eigen::Vector3d& goal_pos,
                          const Eigen::Vector3d& goal_vel,
                          MPPITrajectory& trajectory);
    
    void rolloutTrajectory(const Eigen::Vector3d& start_pos,
                          const Eigen::Vector3d& start_vel,
                          const Eigen::Vector3d& goal_pos,
                          const Eigen::Vector3d& goal_vel,
                          const std::vector<Eigen::Vector3d>& guide_path,
                          MPPITrajectory& trajectory);
    
    double calculateTrajectoryCost(const MPPITrajectory& trajectory,
                                  const Eigen::Vector3d& goal_pos,
                                  const Eigen::Vector3d& goal_vel);
    
    double obstacleCost(const Eigen::Vector3d& position);
    double obstacleCost(const Eigen::Vector3d& position, double dist);  // ✅ Phase 3: ESDF-based cost
    double smoothnessCost(const MPPITrajectory& trajectory);
    double goalCost(const MPPITrajectory& trajectory,
                   const Eigen::Vector3d& goal_pos,
                   const Eigen::Vector3d& goal_vel);
    double velocityCost(const MPPITrajectory& trajectory,
                       const Eigen::Vector3d& desired_vel);
    
    void constrainDynamics(Eigen::Vector3d& velocity, Eigen::Vector3d& acceleration);
    
    MPPITrajectory weightedAverage(const std::vector<MPPITrajectory>& trajectories);
    
    // Visualization functions
    void visualizeTrajectories(const std::vector<MPPITrajectory>& trajectories);
    void visualizeOptimalTrajectory(const MPPITrajectory& trajectory);

public:
    typedef std::shared_ptr<MPPIPlanner> Ptr;
    
    MPPIPlanner();
    ~MPPIPlanner();
    
    void init(ros::NodeHandle& nh, GridMap::Ptr grid_map);
    
    // Main planning interface
    bool planTrajectory(const Eigen::Vector3d& start_pos,
                       const Eigen::Vector3d& start_vel,
                       const Eigen::Vector3d& goal_pos,
                       const Eigen::Vector3d& goal_vel,
                       MPPITrajectory& optimal_trajectory);
    
    // Overload: with initial path guidance (for topological paths)
    bool planTrajectory(const Eigen::Vector3d& start_pos,
                       const Eigen::Vector3d& start_vel,
                       const Eigen::Vector3d& goal_pos,
                       const Eigen::Vector3d& goal_vel,
                       const std::vector<Eigen::Vector3d>& initial_path,
                       MPPITrajectory& optimal_trajectory);
    
    // Local path planning interface (for replacing A* in B-spline optimizer)
    bool planLocalPath(const Eigen::Vector3d& start_pos,
                      const Eigen::Vector3d& goal_pos,
                      std::vector<Eigen::Vector3d>& path_points);
    
    // Parameter setters
    void setNumSamples(int num_samples) { num_samples_ = num_samples; }
    void setHorizonSteps(int steps) { horizon_steps_ = steps; }
    void setTimeStep(double dt) { dt_ = dt; }
    void setTemperature(double lambda) { lambda_ = lambda; }
    void setNoiseParameters(double sigma_pos, double sigma_vel, double sigma_acc) {
        sigma_pos_ = sigma_pos;
        sigma_vel_ = sigma_vel; 
        sigma_acc_ = sigma_acc;
    }
    void setCostWeights(double w_obs, double w_smooth, double w_goal, double w_vel) {
        w_obstacle_ = w_obs;
        w_smoothness_ = w_smooth;
        w_goal_ = w_goal;
        w_velocity_ = w_vel;
    }
    void setVehicleLimits(double max_vel, double max_acc) {
        max_velocity_ = max_vel;
        max_acceleration_ = max_acc;
    }
    
    // Getters
    int getHorizonSteps() const { return horizon_steps_; }
    double getTimeStep() const { return dt_; }
};

} // namespace ego_planner

#endif