// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <path_searching/topo_prm.h>
#include <path_searching/mppi_planner.h>
#include <thread>
#include <omp.h>

namespace ego_planner
{

  // SECTION interfaces for setup and query

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() { std::cout << "des manager" << std::endl; }

  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */

    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
    
    // 🚀 NEW: Parallel MPPI optimization parameter
    nh.param("manager/use_parallel_mppi_optimization", pp_.use_parallel_mppi_optimization, false);
    ROS_INFO("[PlannerManager] Parallel MPPI optimization: %s", pp_.use_parallel_mppi_optimization ? "ENABLED" : "DISABLED");

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    bspline_optimizer_rebound_.reset(new BsplineOptimizer);
    bspline_optimizer_rebound_->setParam(nh);
    bspline_optimizer_rebound_->setEnvironment(grid_map_);
    bspline_optimizer_rebound_->mppi_planner_.reset(new MPPIPlanner);
    bspline_optimizer_rebound_->mppi_planner_->init(nh, grid_map_);

    /* Initialize new planning modules */
    topo_planner_.reset(new TopoPRM);
    topo_planner_->init(nh, grid_map_);
    topo_planner_->setStepSize(0.2);
    topo_planner_->setSearchRadius(3.0);
    topo_planner_->setMaxSampleNum(1000);

    mppi_planner_.reset(new MPPIPlanner);
    mppi_planner_->init(nh, grid_map_);
    mppi_planner_->setNumSamples(500);
    mppi_planner_->setHorizonSteps(20);
    mppi_planner_->setTimeStep(0.1);
    mppi_planner_->setTemperature(1.0);
    mppi_planner_->setNoiseParameters(0.2, 0.5, 1.0);
    mppi_planner_->setCostWeights(100.0, 10.0, 50.0, 20.0);
    mppi_planner_->setVehicleLimits(pp_.max_vel_, pp_.max_acc_);

    ROS_INFO("[PlannerManager] Initialized topological and MPPI planners");

    visualization_ = vis;
  }

  // !SECTION

  // SECTION rebond replanning

  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {

    static int count = 0;
    std::cout << endl
              << "[rebo replan]: -------------------------------------" << count++ << std::endl;
    cout.precision(3);
    cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
         << endl;

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      continous_failures_count_++;
      return false;
    }

    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt, t_refine;

    /*** STEP 1: INIT ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        flag_first_call = false;
        flag_force_polynomial = false;

        PolynomialTraj gl_traj;

        double dist = (start_pt - local_target_pt).norm();
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        if (!flag_randomPolyTraj)
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        double t;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Initial path generated from previous trajectory.
      {

        double t;
        double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              ROS_ERROR("pseudo_arc_length is empty, return!");
              continous_failures_count_++;
              return false;
            }
          }
        }

        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) // The initial path is unnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    // 🔧 Initialize B-spline optimizer internal structures
    // Note: After architecture refactor (Topo→MPPI→BSpline), initControlPoints() now 
    // initializes with topological/MPPI-optimized points, then BsplineOptimizer refines them
    vector<vector<Eigen::Vector3d>> a_star_pathes;
    a_star_pathes = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

    t_init = ros::Time::now() - t_start;

    // 🎨 Visualization: Clear old A* paths (replaced by TOPO+MPPI visualization)
    // static int vis_id = 0;
    // visualization_->displayInitPathList(point_set, 0.2, 0);
    // Note: MPPI paths are now visualized in STEP 1.5 with matching colors

    /*** STEP 1.5: TOPOLOGICAL PLANNING ***/
    // 🎯 Architecture: Topo → MPPI → B-spline
    // Topological planning provides global collision-free paths
    std::vector<TopoPath> topo_paths;
    ROS_INFO("[PlannerManager] Attempting topological planning from [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]", 
             start_pt.x(), start_pt.y(), start_pt.z(), 
             local_target_pt.x(), local_target_pt.y(), local_target_pt.z());
    
    bool use_mppi_topo_path = false;
    bool use_parallel_mppi = pp_.use_parallel_mppi_optimization;  // New parameter for multi-path MPPI
    
    if (topo_planner_ != nullptr && planWithTopo(start_pt, local_target_pt, topo_paths)) {
        ROS_WARN("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        ROS_INFO("[PlannerManager] STEP 1: Topological Planning");
        ROS_INFO("[PlannerManager]   Found %zu topological paths", topo_paths.size());
        
        // Ensure we have valid paths before processing
        if (!topo_paths.empty()) {
            TopoPath best_path;
            
            // 🚀 CRITICAL OPTIMIZATION: 降低MPPI触发阈值从 >1 到 >=1
            // 即使只有1条路径也运行MPPI优化，确保架构升级生效
            if (use_parallel_mppi && mppi_planner_ != nullptr && topo_paths.size() >= 1) {
                ROS_INFO("[PlannerManager] STEP 2: MPPI Optimization (🚀 Triggered for %zu path(s))", topo_paths.size());
                if (topo_paths.size() > 1) {
                    ROS_INFO("[PlannerManager]   🎯 Multi-path parallel optimization mode");
                } else {
                    ROS_INFO("[PlannerManager]   🎯 Single-path optimization mode (still beneficial!)");
                }
                ROS_INFO("[PlannerManager]   🚀 Optimizing all %zu topological paths...", topo_paths.size());
                
                struct MPPICandidate {
                    TopoPath topo_path;
                    MPPITrajectory mppi_result;
                    double normalized_cost;
                    bool success;
                };
                
                std::vector<MPPICandidate> mppi_candidates(topo_paths.size());
                Eigen::Vector3d current_vel = start_vel;
                Eigen::Vector3d target_vel = local_target_vel;
                
                // 🚀 Optimize each topological path with MPPI in PARALLEL
                #pragma omp parallel for schedule(dynamic)
                for (size_t i = 0; i < topo_paths.size(); ++i) {
                    mppi_candidates[i].topo_path = topo_paths[i];
                    mppi_candidates[i].success = false;
                    mppi_candidates[i].normalized_cost = std::numeric_limits<double>::max();
                    
                    // Densify path if needed (for MPPI initial trajectory)
                    std::vector<Eigen::Vector3d> dense_path = topo_paths[i].path;
                    if (dense_path.size() < 7) {
                        std::vector<Eigen::Vector3d> tmp_dense;
                        for (size_t j = 0; j < dense_path.size() - 1; ++j) {
                            tmp_dense.push_back(dense_path[j]);
                            Eigen::Vector3d seg_vec = dense_path[j+1] - dense_path[j];
                            double seg_len = seg_vec.norm();
                            int num_inter = std::max(1, (int)(seg_len / (pp_.ctrl_pt_dist * 0.5)));
                            for (int k = 1; k < num_inter; ++k) {
                                double t = (double)k / num_inter;
                                tmp_dense.push_back(dense_path[j] + t * seg_vec);
                            }
                        }
                        tmp_dense.push_back(dense_path.back());
                        dense_path = tmp_dense;
                    }
                    
                    // Run MPPI on this path with topo guidance
                    ROS_INFO("[PlannerManager]   Path %zu/%zu (topo_cost=%.3f, waypoints=%zu): Running MPPI...", 
                             i+1, topo_paths.size(), topo_paths[i].cost, dense_path.size());
                    
                    // ✅ FIX: Pass dense_path to guide MPPI optimization
                    bool mppi_success = mppi_planner_->planTrajectory(start_pt, current_vel, 
                                                                     local_target_pt, target_vel, 
                                                                     dense_path, mppi_candidates[i].mppi_result);
                    
                    if (mppi_success && mppi_candidates[i].mppi_result.positions.size() >= 7) {
                        // Calculate normalized cost (cost per unit length)
                        double path_length = 0.0;
                        for (size_t j = 1; j < mppi_candidates[i].mppi_result.positions.size(); ++j) {
                            path_length += (mppi_candidates[i].mppi_result.positions[j] - mppi_candidates[i].mppi_result.positions[j-1]).norm();
                        }
                        mppi_candidates[i].normalized_cost = (path_length > 0.1) ? (mppi_candidates[i].mppi_result.cost / path_length) : mppi_candidates[i].mppi_result.cost;
                        mppi_candidates[i].success = true;
                        
                        #pragma omp critical
                        ROS_INFO("[PlannerManager]   Path %zu: MPPI ✅ cost=%.3f, norm_cost=%.3f, length=%.2fm", 
                                 i+1, mppi_candidates[i].mppi_result.cost, mppi_candidates[i].normalized_cost, path_length);
                    } else {
                        #pragma omp critical
                        ROS_WARN("[PlannerManager]   Path %zu: MPPI ❌ failed", i+1);
                        // Use topo path as fallback for visualization
                        mppi_candidates[i].mppi_result.positions = dense_path;
                        mppi_candidates[i].mppi_result.cost = topo_paths[i].cost;
                    }
                }
                
                
                // Select best MPPI result based on normalized cost
                double best_norm_cost = std::numeric_limits<double>::max();
                int best_idx = -1;
                for (size_t i = 0; i < mppi_candidates.size(); ++i) {
                    if (mppi_candidates[i].success && mppi_candidates[i].normalized_cost < best_norm_cost) {
                        best_norm_cost = mppi_candidates[i].normalized_cost;
                        best_idx = i;
                    }
                }
                
                // 🎨 NEW: Store all MPPI-optimized paths for visualization
                all_mppi_paths_.clear();
                for (size_t i = 0; i < mppi_candidates.size(); ++i) {
                    MPPIPathCandidate path_vis;
                    path_vis.positions = mppi_candidates[i].mppi_result.positions;
                    path_vis.cost = mppi_candidates[i].mppi_result.cost;
                    path_vis.normalized_cost = mppi_candidates[i].normalized_cost;
                    path_vis.is_best = (i == best_idx);
                    path_vis.success = mppi_candidates[i].success;
                    all_mppi_paths_.push_back(path_vis);
                }
                ROS_INFO("[PlannerManager]   💾 Stored %zu MPPI-optimized paths for visualization", all_mppi_paths_.size());
                
                if (best_idx >= 0) {
                    ROS_INFO("[PlannerManager]   ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
                    ROS_INFO("[PlannerManager]   🏆 Best MPPI: Path #%d with normalized_cost=%.3f", 
                             best_idx+1, best_norm_cost);
                    ROS_INFO("[PlannerManager]   ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
                    ROS_WARN("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
                    
                    // 🚀 IMPROVED: Backup MPPI result for potential B-spline fallback
                    mppi_result_backup_ = mppi_candidates[best_idx].mppi_result;
                    
                    // Use MPPI-optimized trajectory directly
                    point_set = mppi_result_backup_.positions;
                    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
                    use_mppi_topo_path = true;
                    
                    ROS_INFO("[PlannerManager]   Using MPPI result with %zu points", point_set.size());
                } else {
                    ROS_WARN("[PlannerManager]   ❌ All MPPI failed, fallback to best topo path");
                    best_path = topo_planner_->selectBestPath(topo_paths);
                }
            } else {
                // Original single-path selection
                best_path = topo_planner_->selectBestPath(topo_paths);
                ROS_INFO("[PlannerManager] Using topological path with cost %.3f", best_path.cost);
            }
            
            // If not using parallel MPPI or it failed, use traditional approach
            if (!use_mppi_topo_path && best_path.path.size() >= 2) {
                // Replace control points with topological path
                point_set = best_path.path;
                
                // Ensure sufficient points for B-spline generation
                if (point_set.size() < 7) {
                    // 🚀 OPTIMIZED: 稀疏插值，保持Topo路径的引导特性
                    // 旧策略: num = segment_len / (0.4 * 0.5) → 5米段插25个点 → waypoints=55-133
                    // 新策略: num = segment_len / (0.4 * 4.0) → 5米段插3个点 → waypoints=10以内
                    // 目标: Topo路径保持稀疏(3-10个转折点)，MPPI优化局部细节
                    std::vector<Eigen::Vector3d> dense_path;
                    for (size_t i = 0; i < point_set.size() - 1; ++i) {
                        dense_path.push_back(point_set[i]);
                        Eigen::Vector3d segment_vec = point_set[i+1] - point_set[i];
                        double segment_len = segment_vec.norm();
                        
                        // 优化: 4倍稀疏 (0.5 → 4.0)
                        int num_intermediate = std::max(0, (int)(segment_len / (pp_.ctrl_pt_dist * 4.0)));
                        
                        for (int j = 1; j < num_intermediate; ++j) {
                            double t = (double)j / num_intermediate;
                            dense_path.push_back(point_set[i] + t * segment_vec);
                        }
                    }
                    dense_path.push_back(point_set.back());
                    point_set = dense_path;
                    
                    ROS_INFO("[PlannerManager] 🎯 Sparse interpolation: %zu → %zu waypoints", 
                             best_path.path.size(), point_set.size());
                }
                
                // Re-parameterize control points using topological path
                UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
                use_mppi_topo_path = true;
                
                ROS_INFO("[PlannerManager] Successfully integrated topological path with %zu waypoints", point_set.size());
            } else if (!use_mppi_topo_path) {
                ROS_WARN("[PlannerManager] Selected topological path has insufficient waypoints (%zu), using original approach", best_path.path.size());
            }
        } else {
            ROS_WARN("[PlannerManager] No valid topological paths found, using original approach");
        }
    } else {
        if (topo_planner_ == nullptr) {
            ROS_WARN("[PlannerManager] Topological planner not initialized, using original approach");
        } else {
            ROS_WARN("[PlannerManager] Topological planning failed, continuing with original approach");
        }
    }

    t_start = ros::Time::now();

    /*** STEP 2: MPPI DYNAMIC OPTIMIZATION ***/
    // 🚀 Apply MPPI optimization to topological path (if available)
    // MPPI considers dynamics, ESDF, and control smoothness
    
    // Note: If parallel MPPI was used in STEP 1.5, this step is skipped
    // as MPPI optimization was already applied to all topological paths
    if (use_mppi_topo_path && !use_parallel_mppi && mppi_planner_ != nullptr) {
        ROS_INFO("[PlannerManager] Applying MPPI dynamic optimization with ESDF...");
        
        Eigen::Vector3d current_vel = start_vel;
        Eigen::Vector3d target_vel = local_target_vel;
        
        MPPITrajectory mppi_result;
        bool mppi_success = planWithMPPI(start_pt, current_vel, local_target_pt, target_vel, mppi_result);
        
        if (mppi_success && mppi_result.positions.size() >= 7) {
            ROS_INFO("[PlannerManager] MPPI optimization succeeded with %zu points, using as control points", 
                     mppi_result.positions.size());
            
            // Replace control points with MPPI-optimized trajectory
            point_set = mppi_result.positions;
            
            // Re-parameterize to B-spline with MPPI result
            UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
            
            ROS_INFO("[PlannerManager] MPPI control points integrated successfully");
        } else {
            ROS_WARN("[PlannerManager] MPPI optimization failed or insufficient points (%zu), using topo path", 
                     mppi_result.positions.size());
            // Fallback: keep original topological path control points
        }
        
        ros::Duration mppi_time = ros::Time::now() - t_start;
        ROS_INFO("[PlannerManager] MPPI optimization took %.3f ms", mppi_time.toSec() * 1000.0);
    } else if (use_parallel_mppi) {
        ROS_INFO("[PlannerManager] Skipping STEP 2: Parallel MPPI already applied in STEP 1.5");
    }

    t_start = ros::Time::now();

    /*** STEP 3: B-SPLINE SMOOTHING ***/
    ROS_WARN("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    ROS_INFO("[PlannerManager] STEP 3: B-spline smoothing (ONLY smoothing, NOT planning!)");
    ROS_INFO("[PlannerManager]   Input: %d control points from TOPO/MPPI", (int)ctrl_pts.rows());
    
    // 🎨 Final smoothing and collision avoidance refinement
    bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    
    ROS_INFO("[PlannerManager]   B-spline result: %s", flag_step_1_success ? "✅ SUCCESS" : "❌ FAILED");
    
    if (!flag_step_1_success)
    {
      ROS_WARN("[PlannerManager]   ⚠️ B-spline failed - using MPPI trajectory as fallback");
      
      // 🚀 IMPROVED: Use MPPI trajectory directly instead of failing
      if (mppi_result_backup_.positions.size() >= 3) {
        ROS_INFO("[PlannerManager]   📦 Fallback: Using MPPI trajectory (%zu points)", mppi_result_backup_.positions.size());
        
        // Use MPPI trajectory as-is
        point_set = mppi_result_backup_.positions;
        UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
        
        // Mark as success with fallback
        flag_step_1_success = true;
        ROS_INFO("[PlannerManager]   ✅ Fallback succeeded, continuing with MPPI path");
      } else {
        ROS_WARN("[PlannerManager]   ❌ No MPPI backup available, planning failed");
        continous_failures_count_++;
        return false;
      }
    }
    ROS_WARN("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    //visualization_->displayOptimalList( ctrl_pts, vis_id );

    t_opt = ros::Time::now() - t_start;

    t_start = ros::Time::now();

    /*** STEP 4: TIME REALLOCATION FOR FEASIBILITY ***/
    // ⏱️ Adjust time allocation to satisfy velocity/acceleration constraints
    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    double ratio;
    bool flag_step_2_success = true;
    if (!pos.checkFeasibility(ratio, false))
    {
      cout << "Need to reallocate time." << endl;

      Eigen::MatrixXd optimal_control_points;
      flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
      if (flag_step_2_success)
        pos = UniformBspline(optimal_control_points, 3, ts);
    }

    if (!flag_step_2_success)
    {
      printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
      continous_failures_count_++;
      return false;
    }

    t_refine = ros::Time::now() - t_start;

    // save planned results
    updateTrajInfo(pos, ros::Time::now());

    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << endl;

    // success. YoY
    continous_failures_count_ = 0;
    return true;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), ros::Time::now());

    return true;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    total_len += (start_pos - waypoints[0]).norm();
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
      total_len += (waypoints[i + 1] - waypoints[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // for ( int i=0; i<inter_points.size(); i++ )
    // {
    //   cout << inter_points[i].transpose() << endl;
    // }

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_rebound_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

  // SECTION new topological and MPPI planning methods

  bool EGOPlannerManager::planWithTopo(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &goal_pos,
                                      std::vector<TopoPath> &topo_paths) {
    if (topo_planner_ == nullptr) {
      ROS_ERROR("[PlannerManager] Topological planner not initialized");
      return false;
    }

    bool success = topo_planner_->searchTopoPaths(start_pos, goal_pos, topo_paths);
    
    if (success) {
      ROS_INFO("[PlannerManager] Topological planning succeeded, found %zu paths", topo_paths.size());
    } else {
      ROS_WARN("[PlannerManager] Topological planning failed");
    }

    return success;
  }

  bool EGOPlannerManager::planWithMPPI(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
                                      const Eigen::Vector3d &goal_pos, const Eigen::Vector3d &goal_vel,
                                      MPPITrajectory &optimal_traj) {
    if (mppi_planner_ == nullptr) {
      ROS_ERROR("[PlannerManager] MPPI planner not initialized");
      return false;
    }

    bool success = mppi_planner_->planTrajectory(start_pos, start_vel, goal_pos, goal_vel, optimal_traj);
    
    if (success) {
      ROS_INFO("[PlannerManager] MPPI planning succeeded, trajectory cost: %f", optimal_traj.cost);
    } else {
      ROS_WARN("[PlannerManager] MPPI planning failed");
    }

    return success;
  }

  // !SECTION

} // namespace ego_planner
