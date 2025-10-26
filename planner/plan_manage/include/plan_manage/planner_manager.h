#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <ego_planner/DataDisp.h>
#include <plan_env/grid_map.h>
#include <plan_manage/plan_container.hpp>
#include <ros/ros.h>
#include <traj_utils/planning_visualization.h>
#include <path_searching/topo_prm.h>
#include <path_searching/mppi_planner.h>
#include <visualization_msgs/MarkerArray.h>

namespace ego_planner
{

  // 🎨 RAINBOW COLOR SCHEME: 统一的彩虹色谱 for TOPO paths和MPPI trajectories
  // 与topo_prm.cpp中的可视化颜色完全一致，确保MPPI优化后的路径与原Topo路径颜色对应
  struct PathColor {
    double r, g, b;
  };

  const PathColor TOPO_COLORS[10] = {
    {1.0, 0.0, 0.0},      // #0: 红色 Red
    {1.0, 0.65, 0.0},     // #1: 橙色 Orange
    {1.0, 1.0, 0.0},      // #2: 黄色 Yellow
    {0.0, 1.0, 0.0},      // #3: 绿色 Green
    {0.0, 1.0, 1.0},      // #4: 青色 Cyan
    {0.0, 0.0, 1.0},      // #5: 蓝色 Blue
    {0.5, 0.0, 1.0},      // #6: 紫色 Purple
    {1.0, 0.0, 1.0},      // #7: 品红 Magenta
    {1.0, 0.5, 0.5},      // #8: 粉色 Pink
    {0.5, 1.0, 0.5},      // #9: 浅绿 Light Green
  };

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class EGOPlannerManager
  {
    // SECTION stable
  public:
    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    /* Topological planning interface */
    bool planWithTopo(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &goal_pos,
                     std::vector<TopoPath> &topo_paths);
    
    /* MPPI planning interface */
    bool planWithMPPI(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
                     const Eigen::Vector3d &goal_pos, const Eigen::Vector3d &goal_vel,
                     MPPITrajectory &optimal_traj);

    void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);

    PlanParameters pp_;
    LocalTrajData local_data_;
    GlobalTrajData global_data_;
    GridMap::Ptr grid_map_;

  private:
    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;

    BsplineOptimizer::Ptr bspline_optimizer_rebound_;
    
    /* New topological and MPPI planning modules */
    TopoPRM::Ptr topo_planner_;
    MPPIPlanner::Ptr mppi_planner_;

    /* Visualization for TOPO+MPPI combined paths */
    ros::Publisher topo_mppi_vis_pub_;
    void visualizeTopoMPPIPaths(int path_id,
                               const TopoPath& topo_path,
                               const MPPITrajectory& mppi_result,
                               bool is_best);

    int continous_failures_count_{0};

    void updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now);

    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    // !SECTION stable

    // SECTION developing

  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };
} // namespace ego_planner

#endif