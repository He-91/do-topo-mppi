#include "path_searching/topo_prm.h"
#include <cmath>
#include <algorithm>
#include <fstream>
#include <chrono>    // 🚀 P0: 添加时间测量支持
#include <sstream>   // 🎨 可视化: 字符串格式化
#include <iomanip>   // 🎨 可视化: 数字格式化

using namespace std;
using namespace Eigen;

namespace ego_planner {

TopoPRM::TopoPRM() 
    : step_size_(0.2), search_radius_(5.0), max_sample_num_(1000), 
      collision_check_resolution_(0.2),  // 进一步放宽碰撞检测
      max_raw_paths_(30),               // ✅ GLOBAL FIX: 降到30 (配合早停12+智能早停8)
      reserve_num_(8),                  // 🚀 NEW: 保留8条最短路径
      clearance_(0.6),                  // 🚀 PROVEN: 节点最小安全距离保持0.6m (0.4m导致B-spline失败率增加)
      sample_inflate_(4.0),             // 🚀 OPTIMIZED: 椭球采样膨胀 3.0->4.0m (更大采样范围)
      ratio_to_short_(2.5),             // 🚀 NEW: 最短路径2.5倍以内保留
      discretize_points_num_(20) {      // 🚀 OPTIMIZED: 拓扑去重离散化 30->20点 (放宽去重)
}

TopoPRM::~TopoPRM() {
    clearGraph();
}

void TopoPRM::clearGraph() {
    for (auto node : graph_nodes_) {
        delete node;
    }
    graph_nodes_.clear();
    raw_paths_.clear();
}

void TopoPRM::init(ros::NodeHandle& nh, GridMap::Ptr grid_map) {
    grid_map_ = grid_map;
    topo_paths_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/topo_paths", 10);
    
    // Get frame_id from node parameter, default to "world" if not set
    nh.param("grid_map/frame_id", frame_id_, std::string("world"));
    
    ROS_INFO("[TopoPRM] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    ROS_INFO("[TopoPRM] 🚀 TOPO-PRM BALANCED CONFIG v4.1:");
    ROS_INFO("[TopoPRM]   📊 采样: 椭球100+边界35 (稳定配置)");
    ROS_INFO("[TopoPRM]   🕸️  图构建: KNN K=22 (恢复连通性,修复孤立节点问题)");
    ROS_INFO("[TopoPRM]   🔍 DFS策略: 早停12条/智能8条@100ms, 深度≤20");
    ROS_INFO("[TopoPRM]   ⏱️  超时控制: %.0fms + 连通性预检", MAX_DFS_TIME_MS);
    ROS_INFO("[TopoPRM]   🎯 去重阈值: 3.5%% (适度放宽保留拓扑)");
    ROS_INFO("[TopoPRM]   ✅ 安全参数: clearance=%.2fm", clearance_);
    ROS_INFO("[TopoPRM]   🎯 目标: 超时率<10%%, 单路径率<25%%, 平均路径>3.5");
    ROS_INFO("[TopoPRM] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
}

bool TopoPRM::searchTopoPaths(const Vector3d& start, const Vector3d& goal,
                             vector<TopoPath>& topo_paths) {
    topo_paths.clear();
    
    ROS_INFO("[TopoPRM] ═══════════════════════════════════════════════════════");
    ROS_INFO("[TopoPRM] 🚀 Fast-Planner PRM: [%.2f,%.2f,%.2f] → [%.2f,%.2f,%.2f]", 
             start.x(), start.y(), start.z(), goal.x(), goal.y(), goal.z());
    
    // ✅ GLOBAL FIX: 采样优化 - 减少节点数降低图复杂度
    ROS_INFO("[TopoPRM] STEP 1: 多层自由空间采样...");
    // Layer 1: 椭球核心采样 (100点,从120降低16.7%)
    vector<Vector3d> sample_points = sampleFreeSpaceInEllipsoid(start, goal, 100);
    int ellipsoid_count = sample_points.size();
    
    // Layer 2: 边界层采样 (35点,从40降低12.5%)
    vector<Vector3d> boundary_points = sampleBoundaryLayer(start, goal, 35);
    sample_points.insert(sample_points.end(), boundary_points.begin(), boundary_points.end());
    
    ROS_INFO("[TopoPRM]   采样到 %zu 个有效点 (椭球:%d + 边界:%zu)", 
             sample_points.size(), ellipsoid_count, boundary_points.size());
    
    if (sample_points.size() < 10) {
        ROS_WARN("[TopoPRM] 采样点太少，回退到Legacy方法");
        vector<TopoPath> legacy_paths = findTopoPathsLegacy(start, goal);
        topo_paths = legacy_paths;
        visualizeTopoPaths(topo_paths);
        return !topo_paths.empty();
    }
    
    // Step 2: 构建可见性图 (Week 2)
    ROS_INFO("[TopoPRM] STEP 2: 构建可见性图...");
    buildVisibilityGraph(start, goal, sample_points);
    ROS_INFO("[TopoPRM]   图节点数: %zu", graph_nodes_.size());
    
    if (graph_nodes_.size() < 3) {
        ROS_WARN("[TopoPRM] 图节点太少，回退到Legacy方法");
        clearGraph();
        vector<TopoPath> legacy_paths = findTopoPathsLegacy(start, goal);
        topo_paths = legacy_paths;
        visualizeTopoPaths(topo_paths);
        return !topo_paths.empty();
    }
    
    // Step 3: DFS多路径搜索 (Week 3)
    ROS_INFO("[TopoPRM] STEP 3: DFS多路径搜索...");
    GraphNode* start_node = graph_nodes_[0];
    GraphNode* goal_node = graph_nodes_[1];
    vector<vector<Vector3d>> raw_paths = searchMultiplePaths(start_node, goal_node);
    ROS_INFO("[TopoPRM]   原始路径数: %zu", raw_paths.size());
    
    if (raw_paths.empty()) {
        ROS_WARN("[TopoPRM] 未找到路径，回退到Legacy方法");
        clearGraph();
        vector<TopoPath> legacy_paths = findTopoPathsLegacy(start, goal);
        topo_paths = legacy_paths;
        visualizeTopoPaths(topo_paths);
        return !topo_paths.empty();
    }
    
    // Step 4: 拓扑去重 (Week 4)
    ROS_INFO("[TopoPRM] STEP 4: 拓扑等价性去重...");
    vector<vector<Vector3d>> unique_paths = pruneEquivalentPaths(raw_paths);
    ROS_INFO("[TopoPRM]   去重后路径数: %zu", unique_paths.size());
    
    // Step 5: 选择最短路径
    ROS_INFO("[TopoPRM] STEP 5: 选择最优路径...");
    vector<vector<Vector3d>> selected_paths = selectShortPaths(unique_paths);
    ROS_INFO("[TopoPRM]   最终选择: %zu 条路径", selected_paths.size());
    
    // 转换为TopoPath格式
    for (size_t i = 0; i < selected_paths.size(); ++i) {
        double cost = calculatePathCost(selected_paths[i]);
        topo_paths.emplace_back(selected_paths[i], cost, i);
    }
    
    ROS_INFO("[TopoPRM] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    ROS_INFO("[TopoPRM] 📊 PRM搜索总结:");
    ROS_INFO("[TopoPRM]   采样点: %zu", sample_points.size());
    ROS_INFO("[TopoPRM]   原始路径: %zu", raw_paths.size());
    ROS_INFO("[TopoPRM]   唯一路径: %zu", unique_paths.size());
    ROS_INFO("[TopoPRM]   最终路径: %zu", topo_paths.size());
    ROS_INFO("[TopoPRM]   🎯 多路径触发: %s", 
             topo_paths.size() > 1 ? "✅ YES (MPPI will run!)" : "❌ NO");
    ROS_INFO("[TopoPRM] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    
    // Visualize
    visualizeTopoPaths(topo_paths);
    
    // Cleanup
    clearGraph();
    
    return !topo_paths.empty();
}

// ============================================================================
// 🔧 LEGACY TOPOLOGICAL PLANNING CODE - COMMENTED OUT FOR TESTING
// ============================================================================
// This entire section (findTopoPaths and 4 path generators) is disabled
// during TGK system validation. Will be permanently removed after testing.
// Backup: topo_prm.cpp.backup_before_legacy_removal
// ============================================================================

#if 1  // 🔧 LEGACY CODE RE-ENABLED as safety fallback (based on test1 analysis: TGK fails ~30%)

vector<TopoPath> TopoPRM::findTopoPathsLegacy(const Vector3d& start, const Vector3d& goal) {
    vector<TopoPath> paths;
    
    ROS_INFO("[TopoPRM] ═══════════════════════════════════════");
    ROS_INFO("[TopoPRM] Finding paths: [%.2f,%.2f,%.2f] → [%.2f,%.2f,%.2f]", 
             start.x(), start.y(), start.z(), goal.x(), goal.y(), goal.z());
    
    // Check direct path first
    vector<Vector3d> direct_path = {start, goal};
    if (isPathValid(direct_path)) {
        double cost = calculatePathCost(direct_path);
        paths.emplace_back(direct_path, cost, 0);
        ROS_INFO("[TopoPRM] ✅ Direct path valid, cost: %.3f", cost);
    } else {
        ROS_INFO("[TopoPRM] ❌ Direct path blocked");
    }
    
    // Find obstacles along direct line
    Vector3d dir = (goal - start).normalized();
    double dist = (goal - start).norm();
    
    vector<Vector3d> obstacle_centers;
    
    // Sample along direct path to find obstacles
    int samples = 0;
    for (double t = step_size_; t < dist; t += step_size_) {
        Vector3d sample_point = start + t * dir;
        samples++;
        if (grid_map_->getInflateOccupancy(sample_point)) {
            obstacle_centers.push_back(sample_point);
        }
    }
    
    ROS_INFO("[TopoPRM] Sampled %d points, found %zu obstacles", samples, obstacle_centers.size());
    
    // Remove duplicate nearby obstacle centers
    vector<Vector3d> filtered_obstacles;
    for (const auto& obs : obstacle_centers) {
        bool is_duplicate = false;
        for (const auto& filtered : filtered_obstacles) {
            if ((obs - filtered).norm() < search_radius_ * 0.5) {
                is_duplicate = true;
                break;
            }
        }
        if (!is_duplicate) {
            filtered_obstacles.push_back(obs);
        }
    }
    
    ROS_INFO("[TopoPRM] After filtering: %zu obstacle centers", filtered_obstacles.size());
    
    if (filtered_obstacles.empty()) {
        ROS_WARN("[TopoPRM] ⚠️ No obstacles found - only direct path available");
        ROS_INFO("[TopoPRM] ═══════════════════════════════════════");
        return paths;
    }
    
    // Generate alternative paths - optimized single strategy
    int path_id = 1;
    int total_attempts = 0;
    int valid_paths = 0;
    
    // For each obstacle, generate tangent paths (proven most effective)
    for (size_t obs_idx = 0; obs_idx < filtered_obstacles.size(); ++obs_idx) {
        const auto& obstacle_center = filtered_obstacles[obs_idx];
        // Tangent paths: 4 cardinal directions around obstacle
        vector<Vector3d> tangent_points = generateTangentPoints(start, goal, obstacle_center);
        ROS_INFO("[TopoPRM] Obstacle #%zu at [%.2f, %.2f, %.2f]: generated %zu tangent points", 
                 obs_idx+1, obstacle_center.x(), obstacle_center.y(), obstacle_center.z(), tangent_points.size());
        
        for (size_t tp_idx = 0; tp_idx < tangent_points.size(); ++tp_idx) {
            const auto& tangent_pt = tangent_points[tp_idx];
            total_attempts++;
            vector<Vector3d> alt_path = {start, tangent_pt, goal};
            
            // Check collision for each segment
            bool start_to_tangent_ok = isLineCollisionFree(start, tangent_pt);
            bool tangent_to_goal_ok = isLineCollisionFree(tangent_pt, goal);
            
            if (start_to_tangent_ok && tangent_to_goal_ok) {
                double cost = calculatePathCost(alt_path);
                paths.emplace_back(alt_path, cost, path_id++);
                valid_paths++;
                ROS_INFO("[TopoPRM]   ✅ Path %d: via [%.2f, %.2f, %.2f], cost=%.3f", 
                         path_id-1, tangent_pt.x(), tangent_pt.y(), tangent_pt.z(), cost);
            } else {
                ROS_WARN("[TopoPRM]   ❌ Rejected tangent #%zu: start→tangent=%s, tangent→goal=%s",
                          tp_idx+1, 
                          start_to_tangent_ok ? "OK" : "COLLISION",
                          tangent_to_goal_ok ? "OK" : "COLLISION");
            }
        }
    }
    
    // ✅ CRITICAL FIX: 为Legacy添加拓扑去重 (解决"好几条一样路径"问题)
    size_t paths_before_dedup = paths.size();
    ROS_INFO("[TopoPRM] 📊 STEP 4: Legacy去重 - 移除重复拓扑路径...");
    
    if (paths.size() > 1) {
        vector<TopoPath> unique_legacy_paths;
        unique_legacy_paths.push_back(paths[0]);  // 保留第一条
        
        for (size_t i = 1; i < paths.size(); ++i) {
            bool is_duplicate = false;
            for (const auto& existing : unique_legacy_paths) {
                // 使用与PRM相同的Hausdorff距离判同
                if (sameTopoPath(paths[i].path, existing.path)) {
                    is_duplicate = true;
                    ROS_INFO("[TopoPRM]   🔄 跳过重复路径 #%d (与路径#%d拓扑相同)", 
                             paths[i].path_id, existing.path_id);
                    break;
                }
            }
            if (!is_duplicate) {
                unique_legacy_paths.push_back(paths[i]);
            }
        }
        
        paths = unique_legacy_paths;
        ROS_INFO("[TopoPRM]   ✅ 去重结果: %zu → %zu 条unique路径 (移除%zu条重复)", 
                 paths_before_dedup, paths.size(), paths_before_dedup - paths.size());
    } else {
        ROS_INFO("[TopoPRM]   ⏭️  路径数≤1,无需去重");
    }
    
    ROS_INFO("[TopoPRM] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    ROS_INFO("[TopoPRM] 📊 Legacy Generation Summary:");
    ROS_INFO("[TopoPRM]   Total attempts: %d", total_attempts);
    ROS_INFO("[TopoPRM]   Valid paths (before dedup): %zu", paths_before_dedup);
    ROS_INFO("[TopoPRM]   Unique paths (after dedup): %zu", paths.size());
    ROS_INFO("[TopoPRM]   Dedup removed: %zu paths", paths_before_dedup - paths.size());
    ROS_INFO("[TopoPRM]   Success rate: %.1f%%", total_attempts > 0 ? 100.0*paths_before_dedup/total_attempts : 0.0);
    ROS_INFO("[TopoPRM]   🎯 Multi-path trigger: %s", paths.size() > 1 ? "✅ YES (MPPI will run!)" : "❌ NO (only 1 path)");
    ROS_INFO("[TopoPRM] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    
    // If no paths found at all, try to generate candidate fallbacks and dump them for offline analysis.
    if (paths.empty()) {
        ROS_WARN("[TopoPRM] No paths found, attempting to generate fallback candidates before interpolation");

        vector<vector<Vector3d>> candidates;

        // Try circular and vertical generators around each filtered obstacle center
        for (const auto& obs : filtered_obstacles) {
            auto circL = generateCircularPath(start, goal, obs, -1);
            auto circR = generateCircularPath(start, goal, obs, 1);
            auto up = generateVerticalPath(start, goal, obs, 1);
            auto down = generateVerticalPath(start, goal, obs, -1);
            if (!circL.empty()) candidates.push_back(circL);
            if (!circR.empty()) candidates.push_back(circR);
            if (!up.empty()) candidates.push_back(up);
            if (!down.empty()) candidates.push_back(down);
        }

        // Also try tangents directly from obstacles
        for (const auto& obs : filtered_obstacles) {
            auto tps = generateTangentPoints(start, goal, obs);
            for (const auto& tp : tps) {
                vector<Vector3d> p = {start, tp, goal};
                if (isPathValid(p)) candidates.push_back(p);
            }
        }

        // Keep up to 3 shortest candidates
        if (!candidates.empty()) {
            sort(candidates.begin(), candidates.end(), [this](const vector<Vector3d>& a, const vector<Vector3d>& b){
                return pathLength(a) < pathLength(b);
            });

            int keep = std::min((size_t)3, candidates.size());
            ROS_WARN("[TopoPRM] Found %zu fallback candidates, keeping %d", candidates.size(), keep);

            // dump to /tmp
            std::string tmpfile = "/tmp/topo_raw_candidates.json";
            std::ofstream ofs(tmpfile);
            if (ofs) {
                ofs << "{\n  \"start\": [" << start.x() << ", " << start.y() << ", " << start.z() << "],\n";
                ofs << "  \"goal\": [" << goal.x() << ", " << goal.y() << ", " << goal.z() << "],\n";
                ofs << "  \"candidates\": [\n";
                for (int i = 0; i < keep; ++i) {
                    ofs << "    { \"length\": " << pathLength(candidates[i]) << ", \"points\": [";
                    for (size_t j = 0; j < candidates[i].size(); ++j) {
                        auto &pt = candidates[i][j];
                        ofs << "["<<pt.x()<<","<<pt.y()<<","<<pt.z()<<"]";
                        if (j+1 < candidates[i].size()) ofs << ", ";
                    }
                    ofs << "] }";
                    if (i+1 < keep) ofs << ",\n";
                }
                ofs << "\n  ]\n}\n";
                ofs.close();
                ROS_WARN("[TopoPRM] Dumped fallback candidates to %s", tmpfile.c_str());
            } else {
                ROS_WARN("[TopoPRM] Failed to open %s for dumping candidates", tmpfile.c_str());
            }

            // Use the shortest candidate as the path to continue the pipeline
            double cost = calculatePathCost(candidates[0]);
            paths.emplace_back(candidates[0], cost, 998);
            ROS_INFO("[TopoPRM] Using fallback candidate with length %.2fm", pathLength(candidates[0]));
        } else {
            ROS_WARN("[TopoPRM] No fallback candidates available, using simple interpolated path");
            vector<Vector3d> simple_path;
            Vector3d direction = (goal - start).normalized();
            double distance = (goal - start).norm();
            int num_points = std::max(3, (int)(distance / (step_size_ * 2.0)));
            for (int i = 0; i <= num_points; ++i) {
                double t = (double)i / num_points;
                Vector3d point = start + t * distance * direction;
                simple_path.push_back(point);
            }
            double cost = calculatePathCost(simple_path);
            paths.emplace_back(simple_path, cost, 999);
            ROS_INFO("[TopoPRM] Added simple interpolated path with %zu points", simple_path.size());
        }
    }
    
    return paths;
}

vector<Vector3d> TopoPRM::generateAlternativePath(const Vector3d& start,
                                                 const Vector3d& goal,
                                                 const Vector3d& obstacle_center,
                                                 int direction) {
    vector<Vector3d> path;
    
    // Calculate avoidance direction
    Vector3d avoidance_dir;
    Vector3d forward_dir = (goal - start).normalized();
    
    switch (direction) {
        case 0: // up
            avoidance_dir = Vector3d(0, 0, 1);
            break;
        case 1: // down  
            avoidance_dir = Vector3d(0, 0, -1);
            break;
        case 2: // left (perpendicular to forward direction)
            avoidance_dir = forward_dir.cross(Vector3d(0, 0, 1)).normalized();
            break;
        case 3: // right
            avoidance_dir = -forward_dir.cross(Vector3d(0, 0, 1)).normalized();
            break;
        default:
            return path; // empty path
    }
    
    // Calculate waypoint to avoid obstacle
    Vector3d waypoint = obstacle_center + avoidance_dir * search_radius_;
    
    // Check if waypoint is valid and try different distances if needed
    bool waypoint_valid = false;
    for (double dist = search_radius_ * 0.5; dist <= search_radius_ * 3.0; dist += search_radius_ * 0.5) {
        waypoint = obstacle_center + avoidance_dir * dist;
        if (!grid_map_->getInflateOccupancy(waypoint)) {
            waypoint_valid = true;
            break;
        }
    }
    
    // If no valid waypoint found, return empty path
    if (!waypoint_valid) {
        ROS_DEBUG("[TopoPRM] Could not find valid waypoint for direction %d", direction);
        return path; // empty path
    }
    
    // Create path: start -> waypoint -> goal
    path.push_back(start);
    path.push_back(waypoint);
    path.push_back(goal);
    
    return path;
}

bool TopoPRM::isPathValid(const vector<Vector3d>& path) {
    if (path.size() < 2) return false;
    
    for (size_t i = 0; i < path.size() - 1; ++i) {
        if (!isLineCollisionFree(path[i], path[i + 1])) {
            ROS_DEBUG("[TopoPRM] Path segment %zu-%zu is blocked", i, i+1);
            return false;
        }
    }
    return true;
}

bool TopoPRM::isLineCollisionFree(const Vector3d& start, const Vector3d& end) {
    Vector3d dir = end - start;
    double dist = dir.norm();
    if (dist < 1e-6) return true;
    
    dir.normalize();
    
    for (double t = 0; t <= dist; t += collision_check_resolution_) {
        Vector3d point = start + t * dir;
        if (grid_map_->getInflateOccupancy(point)) {
            return false;
        }
    }
    return true;
}

double TopoPRM::calculatePathCost(const vector<Vector3d>& path) {
    if (path.size() < 2) return std::numeric_limits<double>::max();
    
    double length_cost = 0.0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        length_cost += (path[i + 1] - path[i]).norm();
    }
    
    double smoothness_cost = calculateSmoothnessCost(path);
    double obstacle_cost = calculateObstacleCost(path);
    
    return length_cost + 2.0 * smoothness_cost + 5.0 * obstacle_cost;
}

double TopoPRM::calculateSmoothnessCost(const vector<Vector3d>& path) {
    if (path.size() < 3) return 0.0;
    
    double smoothness_cost = 0.0;
    for (size_t i = 1; i < path.size() - 1; ++i) {
        Vector3d v1 = (path[i] - path[i - 1]).normalized();
        Vector3d v2 = (path[i + 1] - path[i]).normalized();
        double angle = acos(std::max(-1.0, std::min(1.0, v1.dot(v2))));
        smoothness_cost += angle;
    }
    return smoothness_cost;
}

double TopoPRM::calculateObstacleCost(const vector<Vector3d>& path) {
    double obstacle_cost = 0.0;
    
    for (const auto& point : path) {
        // Check distance to nearest obstacle
        double min_dist = std::numeric_limits<double>::max();
        
        // Sample around the point to find nearest obstacle
        for (double dx = -search_radius_; dx <= search_radius_; dx += step_size_) {
            for (double dy = -search_radius_; dy <= search_radius_; dy += step_size_) {
                for (double dz = -search_radius_; dz <= search_radius_; dz += step_size_) {
                    Vector3d sample = point + Vector3d(dx, dy, dz);
                    if (grid_map_->getInflateOccupancy(sample)) {
                        double dist = Vector3d(dx, dy, dz).norm();
                        min_dist = std::min(min_dist, dist);
                    }
                }
            }
        }
        
        if (min_dist < search_radius_) {
            obstacle_cost += 1.0 / (min_dist + 0.1);
        }
    }
    
    return obstacle_cost;
}

TopoPath TopoPRM::selectBestPath(const vector<TopoPath>& paths) {
    if (paths.empty()) {
        return TopoPath();
    }
    
    // Return the path with minimum cost
    auto best_it = std::min_element(paths.begin(), paths.end(),
        [](const TopoPath& a, const TopoPath& b) {
            return a.cost < b.cost;
        });
    
    return *best_it;
}

void TopoPRM::visualizeTopoPaths(const vector<TopoPath>& paths) {
    ROS_INFO("[TopoPRM] 🎨 Visualizing %zu topological paths with RAINBOW colors", paths.size());
    
    visualization_msgs::MarkerArray marker_array;
    
    // Clear previous markers
    visualization_msgs::Marker clear_marker;
    clear_marker.header.frame_id = frame_id_;
    clear_marker.header.stamp = ros::Time::now();
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    
    // 🎨 固定彩虹色板 (8色) - 鲜艳易区分
    struct Color { double r, g, b; };
    vector<Color> RAINBOW_COLORS = {
        {1.0, 0.0, 0.0},      // #1: 红色 Red
        {1.0, 0.65, 0.0},     // #2: 橙色 Orange
        {1.0, 1.0, 0.0},      // #3: 黄色 Yellow
        {0.0, 1.0, 0.0},      // #4: 绿色 Green
        {0.0, 1.0, 1.0},      // #5: 青色 Cyan
        {0.0, 0.0, 1.0},      // #6: 蓝色 Blue
        {0.5, 0.0, 1.0},      // #7: 紫色 Purple
        {1.0, 0.0, 1.0}       // #8: 品红 Magenta
    };
    
    // Visualize each path with different colors
    for (size_t i = 0; i < paths.size(); ++i) {
        Color color = RAINBOW_COLORS[i % RAINBOW_COLORS.size()];
        
        // 路径线条
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = frame_id_;
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "topo_paths";
        line_marker.id = i;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.pose.orientation.w = 1.0;
        
        line_marker.color.r = color.r;
        line_marker.color.g = color.g;
        line_marker.color.b = color.b;
        line_marker.color.a = 0.6;  // 半透明，突出MPPI优化后路径
        line_marker.scale.x = 0.08;  // 细线 (MPPI会用粗线)
        
        for (const auto& point : paths[i].path) {
            geometry_msgs::Point p;
            p.x = point.x();
            p.y = point.y();
            p.z = point.z();
            line_marker.points.push_back(p);
        }
        
        marker_array.markers.push_back(line_marker);
        
        // 🎨 路径ID文本标注 (起点)
        visualization_msgs::Marker text_marker;
        text_marker.header = line_marker.header;
        text_marker.ns = "topo_labels";
        text_marker.id = i;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        
        const auto& start_pt = paths[i].path.front();
        text_marker.pose.position.x = start_pt.x();
        text_marker.pose.position.y = start_pt.y();
        text_marker.pose.position.z = start_pt.z() + 0.6;  // 起点上方0.6m
        text_marker.pose.orientation.w = 1.0;
        
        text_marker.scale.z = 0.35;  // 文本大小
        text_marker.color.r = color.r;
        text_marker.color.g = color.g;
        text_marker.color.b = color.b;
        text_marker.color.a = 1.0;
        
        std::stringstream ss;
        ss << "Topo#" << (i+1) << "\ncost=" << std::fixed << std::setprecision(1) << paths[i].cost;
        text_marker.text = ss.str();
        
        marker_array.markers.push_back(text_marker);
    }
    
    ROS_INFO("[TopoPRM] About to publish MarkerArray with %zu markers", marker_array.markers.size());
    
    // Check publisher status
    if (topo_paths_pub_.getNumSubscribers() > 0) {
        ROS_INFO("[TopoPRM] Publisher has %u subscribers", topo_paths_pub_.getNumSubscribers());
    } else {
        ROS_WARN("[TopoPRM] Publisher has no subscribers!");
    }
    
    topo_paths_pub_.publish(marker_array);
    
    // Give some time for publishing
    ros::Duration(0.01).sleep();
    
    ROS_INFO("[TopoPRM] Published MarkerArray with %zu markers to topic '/topo_paths'", marker_array.markers.size());
}

// Fast-Planner inspired path generation methods

vector<Vector3d> TopoPRM::generateCircularPath(const Vector3d& start,
                                              const Vector3d& goal,
                                              const Vector3d& obstacle_center,
                                              int side) {
    vector<Vector3d> path;
    
    // Get the direction from start to goal
    Vector3d start_to_goal = goal - start;
    
    // Find perpendicular direction in horizontal plane
    Vector3d horizontal_perp = start_to_goal.cross(Vector3d(0, 0, 1)).normalized();
    if (horizontal_perp.norm() < 1e-3) {
        // Start and goal are vertically aligned, use arbitrary horizontal direction
        horizontal_perp = Vector3d(1, 0, 0);
    }
    
    // Create waypoint that circles around the obstacle
    double avoidance_radius = search_radius_ * 1.2;
    Vector3d waypoint = obstacle_center + side * horizontal_perp * avoidance_radius;
    
    // Check if waypoint is collision-free and adjust if necessary
    bool waypoint_valid = false;
    for (double radius = avoidance_radius; radius <= avoidance_radius * 2.5; radius += search_radius_ * 0.3) {
        waypoint = obstacle_center + side * horizontal_perp * radius;
        if (!grid_map_->getInflateOccupancy(waypoint)) {
            waypoint_valid = true;
            break;
        }
    }
    
    if (!waypoint_valid) {
        return path; // empty path
    }
    
    path.push_back(start);
    path.push_back(waypoint);
    path.push_back(goal);
    
    return path;
}

vector<Vector3d> TopoPRM::generateVerticalPath(const Vector3d& start,
                                              const Vector3d& goal,
                                              const Vector3d& obstacle_center,
                                              int vertical) {
    vector<Vector3d> path;
    
    // Create waypoint above or below the obstacle
    double vertical_offset = search_radius_ * 1.5 * vertical;
    Vector3d waypoint = obstacle_center + Vector3d(0, 0, vertical_offset);
    
    // Ensure waypoint is at a reasonable height
    if (waypoint.z() < 0.3) {  // Minimum height above ground
        waypoint.z() = 0.3;
    } else if (waypoint.z() > 5.0) {  // Maximum reasonable flight height
        waypoint.z() = 5.0;
    }
    
    // Check if waypoint is collision-free
    if (grid_map_->getInflateOccupancy(waypoint)) {
        return path; // empty path
    }
    
    path.push_back(start);
    path.push_back(waypoint);
    path.push_back(goal);
    
    return path;
}

vector<Vector3d> TopoPRM::generateTangentPoints(const Vector3d& start,
                                               const Vector3d& goal,
                                               const Vector3d& obstacle_center) {
    vector<Vector3d> tangent_points;
    
    // Get perpendicular direction to start-goal line
    Vector3d start_to_goal = (goal - start).normalized();
    Vector3d perp_horizontal = start_to_goal.cross(Vector3d(0, 0, 1)).normalized();
    
    if (perp_horizontal.norm() < 1e-3) {
        perp_horizontal = Vector3d(1, 0, 0); // Fallback if vertical
    }
    
    // 🚀 AGGRESSIVE STRATEGY: 大幅增加安全距离
    // 根据test1分析：动态估算+1m仍然导致100%碰撞
    // 新策略：估算障碍物大小 + 3倍安全余量
    double obstacle_actual_radius = estimateObstacleSize(obstacle_center);
    
    // 🚀 PROVEN: 保持原始安全余量策略 (过于激进会导致B-spline失败)
    // 当前接受率12.83%可接受，系统整体性能优秀
    vector<double> safety_margins = {0.3, 0.5, 0.8, 1.0, 1.5, 2.0};
    double avoidance_radius = 0.0;
    double used_safety = 0.0;
    
    // ✅ 额外策略：如果估算半径太小，强制最小绕行距离
    double min_avoidance = search_radius_ * 1.5;  // 最小7.5m (5.0 * 1.5)
    avoidance_radius = std::max(avoidance_radius, min_avoidance);
    
    ROS_INFO("[TopoPRM]   🎯 Obstacle radius: %.2fm, used_safety: +%.2fm → avoidance: %.2fm", 
              obstacle_actual_radius, used_safety, avoidance_radius);
    
    // ✅ OPTIMIZATION: 8个方向，确保不同拓扑路径
    // 核心思想：从障碍物不同侧通过 = 不同拓扑
    Vector3d perp_vertical = start_to_goal.cross(perp_horizontal).normalized();
    
    vector<Vector3d> directions = {
        perp_horizontal,                              // Left (0°)
        -perp_horizontal,                             // Right (180°)
        Vector3d(0, 0, 1),                           // Up (90° vertical)
        Vector3d(0, 0, -1),                          // Down (-90° vertical)
        (perp_horizontal + Vector3d(0,0,1)).normalized(),   // Left-Up (45°)
        (-perp_horizontal + Vector3d(0,0,1)).normalized(),  // Right-Up (135°)
        (perp_horizontal - Vector3d(0,0,1)).normalized(),   // Left-Down (-45°)
        (-perp_horizontal - Vector3d(0,0,1)).normalized()   // Right-Down (-135°)
    };
    
    int accepted = 0;
    int rejected = 0;
    double direct_dist = (goal - start).norm();

    // 🚀 P0 OPTIMIZATION: 改进的tangent生成策略
    // 1. 从宽松safety开始（0.3m），优先尝试更靠近障碍物的路径
    // 2. 放宽路径长度限制到4.0倍（原3.5倍）
    // 3. 不在第一个margin成功后break，收集多个margin的结果以增加拓扑多样性
    for (double margin : safety_margins) {
        double candidate_radius = obstacle_actual_radius + margin;
        candidate_radius = std::max(candidate_radius, search_radius_ * 1.5);

        for (const auto& dir : directions) {
            Vector3d tangent_point = obstacle_center + dir * candidate_radius;

            // Validate: collision-free
            if (grid_map_->getInflateOccupancy(tangent_point)) {
                rejected++;
                continue;
            }

            // 🚀 PROVEN: 路径长度保持4.0倍 (5.0倍过于宽松，质量下降)
            double dist_to_start = (tangent_point - start).norm();
            double dist_to_goal = (tangent_point - goal).norm();

            if (dist_to_start + dist_to_goal > direct_dist * 4.0) {
                rejected++;
                continue;
            }

            // Accept this tangent and record which safety margin we used
            tangent_points.push_back(tangent_point);
            accepted++;
            used_safety = margin;
        }

        // 🚀 CHANGED: 移除early break，尝试所有margin以增加拓扑多样性
        // 但如果已经有足够多的候选点（>16个，即2个margin的8方向），可以提前结束
        if (accepted >= 16) {
            avoidance_radius = obstacle_actual_radius + used_safety;
            break;
        }
    }
    
    // 更新最终使用的avoidance半径
    if (accepted > 0 && avoidance_radius == 0.0) {
        avoidance_radius = obstacle_actual_radius + used_safety;
    }

    ROS_INFO("[TopoPRM]   Tangent generation: accepted=%d, rejected=%d, used_safety=%.2f", accepted, rejected, used_safety);
    
    return tangent_points;
}

// 🚀 NEW FUNCTION: 动态估计障碍物实际大小
// 通过8个方向采样，找到障碍物的实际边界
double TopoPRM::estimateObstacleSize(const Vector3d& obstacle_center) {
    double max_radius = 0.5;  // 最小假设半径：0.5米
    
    // 8个采样方向
    vector<Vector3d> sample_directions = {
        Vector3d(1, 0, 0),    // X+
        Vector3d(-1, 0, 0),   // X-
        Vector3d(0, 1, 0),    // Y+
        Vector3d(0, -1, 0),   // Y-
        Vector3d(0, 0, 1),    // Z+
        Vector3d(0, 0, -1),   // Z-
        Vector3d(0.707, 0.707, 0),   // XY对角线
        Vector3d(-0.707, 0.707, 0)   // XY对角线
    };
    
    for (const auto& dir : sample_directions) {
        double radius = 0.0;
        double step = 0.1;  // 10cm采样步长
        
        // 从中心向外扩展，直到离开障碍物
        while (radius < search_radius_ * 2.0) {  // 最大搜索2倍search_radius
            Vector3d sample_point = obstacle_center + dir * radius;
            
            if (!grid_map_->getInflateOccupancy(sample_point)) {
                // 已经离开障碍物
                break;
            }
            
            radius += step;
        }
        
        max_radius = std::max(max_radius, radius);
    }
    
    ROS_DEBUG("[TopoPRM] Estimated obstacle size: %.2fm (center: [%.2f,%.2f,%.2f])", 
              max_radius, obstacle_center.x(), obstacle_center.y(), obstacle_center.z());
    
    return max_radius;
}

#endif  // End of LEGACY CODE BLOCK - Re-enabled as safety fallback

// ============================================================================
// 🚀 FAST-PLANNER PRM IMPLEMENTATION (Week 1-4)
// ============================================================================

// ============================================================================
// Week 1: 椭球自由空间采样
// ============================================================================
vector<Vector3d> TopoPRM::sampleFreeSpaceInEllipsoid(const Vector3d& start, 
                                                     const Vector3d& goal, 
                                                     int num_samples) {
    vector<Vector3d> free_points;
    
    Vector3d center = 0.5 * (start + goal);
    double semi_major_axis = 0.5 * (goal - start).norm() + sample_inflate_;
    
    // 构建椭球坐标系
    Vector3d x_axis = (goal - start).normalized();
    Vector3d z_axis(0, 0, 1);
    Vector3d y_axis = x_axis.cross(z_axis).normalized();
    if (y_axis.norm() < 1e-3) {
        y_axis = Vector3d(1, 0, 0);
    }
    z_axis = x_axis.cross(y_axis).normalized();
    
    Matrix3d rotation;
    rotation.col(0) = x_axis;
    rotation.col(1) = y_axis;
    rotation.col(2) = z_axis;
    
    // 随机采样
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    
    int valid_count = 0;
    int attempts = 0;
    int max_attempts = num_samples * 10;  // 最多尝试10倍
    
    // 🚀 OPTIMIZED: 收集被拒绝的点（靠近障碍）用于边缘采样
    vector<Vector3d> rejected_points;
    
    while (valid_count < num_samples && attempts < max_attempts) {
        attempts++;
        
        // 在椭球内均匀采样
        double theta = 2.0 * M_PI * dis(gen);
        double phi = acos(2.0 * dis(gen) - 1.0);
        double r = pow(dis(gen), 1.0/3.0) * semi_major_axis;
        
        Vector3d pt_local(
            r * sin(phi) * cos(theta),
            r * sin(phi) * sin(theta),
            r * cos(phi)
        );
        
        Vector3d pt_world = rotation * pt_local + center;
        
        // 检查是否在自由空间
        if (isPointFree(pt_world, clearance_)) {
            free_points.push_back(pt_world);
            valid_count++;
        } else {
            // 收集被拒绝的点用于后续边缘采样
            rejected_points.push_back(pt_world);
        }
    }
    
    // 🚀 OPTIMIZED: 添加障碍边缘采样
    int edge_samples = min(50, (int)rejected_points.size());  // 最多50个边缘采样
    for (int i = 0; i < edge_samples; ++i) {
        Vector3d obs_center = rejected_points[i];
        // 生成切线点
        vector<Vector3d> tangents = generateTangentPoints(start, goal, obs_center);
        for (const auto& tangent : tangents) {
            if (isPointFree(tangent, clearance_)) {
                free_points.push_back(tangent);
            }
        }
    }
    
    ROS_DEBUG("[TopoPRM] 椭球采样: %d 次尝试, %d 个有效点 (+%d 边缘采样)", attempts, valid_count, edge_samples);
    
    return free_points;
}

// 🚀 P0 NEW FUNCTION: 边界层采样
// 在椭球边缘（shell）采样，增加远端和侧向连通性
vector<Vector3d> TopoPRM::sampleBoundaryLayer(const Vector3d& start,
                                               const Vector3d& goal,
                                               int num_samples) {
    vector<Vector3d> boundary_points;
    
    Vector3d center = 0.5 * (start + goal);
    double semi_major_axis = 0.5 * (goal - start).norm() + sample_inflate_;
    
    // 构建椭球坐标系
    Vector3d x_axis = (goal - start).normalized();
    Vector3d z_axis(0, 0, 1);
    Vector3d y_axis = x_axis.cross(z_axis).normalized();
    if (y_axis.norm() < 1e-3) {
        y_axis = Vector3d(1, 0, 0);
    }
    z_axis = x_axis.cross(y_axis).normalized();
    
    Matrix3d rotation;
    rotation.col(0) = x_axis;
    rotation.col(1) = y_axis;
    rotation.col(2) = z_axis;
    
    // 随机采样
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    
    int valid_count = 0;
    int attempts = 0;
    int max_attempts = num_samples * 10;
    
    while (valid_count < num_samples && attempts < max_attempts) {
        attempts++;
        
        // 在椭球表面（shell）采样：r ∈ [0.8, 1.0] * semi_major_axis
        double theta = 2.0 * M_PI * dis(gen);
        double phi = acos(2.0 * dis(gen) - 1.0);
        double r = (0.8 + 0.2 * dis(gen)) * semi_major_axis; // 边界层
        
        Vector3d pt_local(
            r * sin(phi) * cos(theta),
            r * sin(phi) * sin(theta),
            r * cos(phi)
        );
        
        Vector3d pt_world = rotation * pt_local + center;
        
        // 检查是否在自由空间（使用更小的clearance以增加接受率）
        if (isPointFree(pt_world, clearance_ * 0.8)) {
            boundary_points.push_back(pt_world);
            valid_count++;
        }
    }
    
    ROS_DEBUG("[TopoPRM] 边界采样: %d 次尝试, %d 个有效点", attempts, valid_count);
    
    return boundary_points;
}

bool TopoPRM::isPointFree(const Vector3d& pt, double min_clearance) {
    // 检查是否在地图范围内
    if (!grid_map_->isInMap(pt)) {
        return false;
    }
    
    // 检查距离障碍物的距离
    double dist = grid_map_->getDistance(pt);
    return dist > min_clearance;
}

// ============================================================================
// Week 2: 可见性图构建
// ============================================================================
void TopoPRM::buildVisibilityGraph(const Vector3d& start, const Vector3d& goal,
                                   const vector<Vector3d>& sample_points) {
    clearGraph();
    
    // 创建起点和终点节点
    GraphNode* start_node = new GraphNode(start, 0);
    GraphNode* goal_node = new GraphNode(goal, 1);
    graph_nodes_.push_back(start_node);
    graph_nodes_.push_back(goal_node);
    
    // 创建采样点节点
    int node_id = 2;
    for (const auto& pt : sample_points) {
        GraphNode* node = new GraphNode(pt, node_id++);
        graph_nodes_.push_back(node);
    }
    
    // ✅ UPGRADED: K=28优化连通性 (K=22在复杂场景下仍有57.1% DFS超时,其中91.7%找到0路径)
    // 数据分析: Test3显示22/24超时案例为连通性失败(0路径),需提升K值保证复杂场景下图连通性
    int K = 28;
    int edge_count = 0;
    for (size_t i = 0; i < graph_nodes_.size(); ++i) {
        // 收集所有其他节点的距离
        vector<pair<double, int>> distances;
        for (size_t j = 0; j < graph_nodes_.size(); ++j) {
            if (i == j) continue;
            double dist = (graph_nodes_[i]->pos - graph_nodes_[j]->pos).norm();
            distances.push_back({dist, j});
        }
        
        // 排序并取前 K 个最近邻
        sort(distances.begin(), distances.end());
    int num_neighbors = min(K, (int)distances.size());
        
        // 对每个 KNN 邻居检查可见性
        for (int k = 0; k < num_neighbors; ++k) {
            int j = distances[k].second;
            Vector3d p1 = graph_nodes_[i]->pos;
            Vector3d p2 = graph_nodes_[j]->pos;
            
            // 检查可见性
            if (isLineCollisionFree(p1, p2)) {
                // 检查是否已连接（避免重复）
                bool already_connected = false;
                for (auto neighbor : graph_nodes_[i]->neighbors) {
                    if (neighbor->id == graph_nodes_[j]->id) {
                        already_connected = true;
                        break;
                    }
                }
                if (!already_connected) {
                    graph_nodes_[i]->neighbors.push_back(graph_nodes_[j]);
                    graph_nodes_[j]->neighbors.push_back(graph_nodes_[i]);
                    edge_count++;
                }
            }
        }
        
        // 🚀 P0 OPTIMIZED: 如果本节点连接数太少，尝试基于半径的备份策略放宽距离限制
        int connected_here = graph_nodes_[i]->neighbors.size();
        if (connected_here < 3) {
            double radius_thresh = search_radius_ * 1.5; // 🚀 放宽备份阈值 1.2→1.5
            for (size_t j = 0; j < graph_nodes_.size(); ++j) {
                if (i == j) continue;
                double dist = (graph_nodes_[i]->pos - graph_nodes_[j]->pos).norm();
                if (dist > radius_thresh) continue;
                // 如果已经连接就跳过
                bool already_connected = false;
                for (auto neighbor : graph_nodes_[i]->neighbors) {
                    if (neighbor->id == graph_nodes_[j]->id) {
                        already_connected = true;
                        break;
                    }
                }
                if (already_connected) continue;

                // 尝试连边
                Vector3d p1 = graph_nodes_[i]->pos;
                Vector3d p2 = graph_nodes_[j]->pos;
                if (isLineCollisionFree(p1, p2)) {
                    graph_nodes_[i]->neighbors.push_back(graph_nodes_[j]);
                    graph_nodes_[j]->neighbors.push_back(graph_nodes_[i]);
                    edge_count++;
                }
            }
        }
    }
    
    // ✅ GLOBAL FIX: 增强图诊断信息
    double avg_degree = graph_nodes_.empty() ? 0.0 : (2.0 * edge_count / graph_nodes_.size());
    ROS_INFO("[TopoPRM]   可见性图构建完成: KNN K=%d", K);
    ROS_INFO("[TopoPRM]   节点数: %zu, 边数: %d, 平均度: %.1f", 
             graph_nodes_.size(), edge_count, avg_degree);
    
    // 🔍 NEW: 起点/终点连通性报告
    if (graph_nodes_.size() >= 2) {
        int start_degree = graph_nodes_[0]->neighbors.size();
        int goal_degree = graph_nodes_[1]->neighbors.size();
        ROS_INFO("[TopoPRM]   起点连接数: %d, 终点连接数: %d", start_degree, goal_degree);
    }
}

// ============================================================================
// Week 3: DFS多路径搜索
// ============================================================================
vector<vector<Vector3d>> TopoPRM::searchMultiplePaths(GraphNode* start_node, 
                                                      GraphNode* goal_node) {
    raw_paths_.clear();
    
    // � NEW: 连通性诊断 - 提前检测起点/终点孤立问题
    int start_degree = start_node->neighbors.size();
    int goal_degree = goal_node->neighbors.size();
    
    ROS_INFO("[TopoPRM] 图连通性诊断: 起点度=%d, 终点度=%d, 总节点=%zu", 
             start_degree, goal_degree, graph_nodes_.size());
    
    if (start_degree == 0 || goal_degree == 0) {
        ROS_WARN("[TopoPRM] ⚠️ 起点(度=%d)或终点(度=%d)孤立,无法执行DFS,直接回退Legacy",
                 start_degree, goal_degree);
        return {};  // 返回空,避免浪费150ms无效搜索
    }
    
    if (start_degree < 3 || goal_degree < 3) {
        ROS_WARN("[TopoPRM] ⚠️ 起点(度=%d)或终点(度=%d)连通性差,DFS可能困难",
                 start_degree, goal_degree);
    }
    
    // �🚀 P0 FIX: 初始化超时控制
    dfs_start_time_ = std::chrono::steady_clock::now();
    dfs_timeout_flag_ = false;
    
    vector<GraphNode*> visited;
    visited.push_back(start_node);
    
    depthFirstSearch(visited, goal_node);
    
    // 转换为Vector3d路径
    vector<vector<Vector3d>> result_paths;
    for (const auto& node_path : raw_paths_) {
        result_paths.push_back(node_path);
    }
    
    // ✅ GLOBAL FIX: 增强超时报告
    auto end_time = std::chrono::steady_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - dfs_start_time_).count();
    if (dfs_timeout_flag_) {
        ROS_WARN("[TopoPRM] ⏱️ DFS超时 (%.1fms > %.1fms),已找到 %zu 条路径 - 图节点:%zu", 
                 elapsed_ms, MAX_DFS_TIME_MS, result_paths.size(), graph_nodes_.size());
    } else {
        ROS_INFO("[TopoPRM] ✅ DFS完成 (%.1fms),找到 %zu 条原始路径", 
                 elapsed_ms, result_paths.size());
    }
    
    return result_paths;
}

void TopoPRM::depthFirstSearch(vector<GraphNode*>& visited, GraphNode* goal_node) {
    // 🚀 P0 FIX: 超时检查（每次递归开始时检查）
    auto now = std::chrono::steady_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(now - dfs_start_time_).count();
    if (elapsed_ms > MAX_DFS_TIME_MS) {
        if (!dfs_timeout_flag_) {
            dfs_timeout_flag_ = true;
            ROS_WARN("[TopoPRM] DFS达到时间限制 (%.1fms)，当前已找到 %zu 条路径", 
                     elapsed_ms, raw_paths_.size());
        }
        return;  // 超时，提前返回
    }
    
    GraphNode* current = visited.back();
    
    // 到达终点
    if (current->id == goal_node->id) {
        vector<Vector3d> path;
        for (auto node : visited) {
            path.push_back(node->pos);
        }
        raw_paths_.push_back(path);
        
        // 限制路径数量
        if (raw_paths_.size() >= (size_t)max_raw_paths_) {
            return;
        }
        return;
    }
    
    // ✅ GLOBAL FIX: 早停优化 - 降到12条(日志显示超时时已有12条,15太高)
    if (raw_paths_.size() >= 12) {
        return;  // 12条raw → 去重后~4-5条,避免超时
    }
    
    // ✅ NEW: 智能早停 - 基于时间和质量折衷
    auto current_time = std::chrono::steady_clock::now();
    double current_elapsed = std::chrono::duration<double, std::milli>(current_time - dfs_start_time_).count();
    if (raw_paths_.size() >= 8 && current_elapsed > 100.0) {
        ROS_DEBUG("[TopoPRM] 智能早停: 已找到%zu条路径,耗时%.1fms", raw_paths_.size(), current_elapsed);
        return;  // 8条路径+100ms → 提前退出,避免边际收益递减
    }
    
    // 递归搜索邻居
    for (auto neighbor : current->neighbors) {
        // 检查是否已访问
        bool already_visited = false;
        for (auto v : visited) {
            if (v->id == neighbor->id) {
                already_visited = true;
                break;
            }
        }
        
        if (already_visited) continue;
        
        // ✅ GLOBAL FIX: 深度限制降低到20 (25→20: 减少低效深层递归)
        if (visited.size() > 20) continue;
        
        // 递归
        visited.push_back(neighbor);
        depthFirstSearch(visited, goal_node);
        
        if (raw_paths_.size() >= (size_t)max_raw_paths_) {
            return;
        }
        
        visited.pop_back();
    }
}

// ============================================================================
// Week 4: 拓扑等价性去重
// ============================================================================
vector<vector<Vector3d>> TopoPRM::pruneEquivalentPaths(
    const vector<vector<Vector3d>>& paths) {
    
    if (paths.empty()) return paths;
    
    vector<vector<Vector3d>> unique_paths;
    unique_paths.push_back(paths[0]);
    
    for (size_t i = 1; i < paths.size(); ++i) {
        bool is_unique = true;
        
        for (const auto& existing : unique_paths) {
            if (sameTopoPath(paths[i], existing)) {
                is_unique = false;
                break;
            }
        }
        
        if (is_unique) {
            unique_paths.push_back(paths[i]);
        }
    }
    
    ROS_DEBUG("[TopoPRM] 拓扑去重: %zu → %zu 路径", paths.size(), unique_paths.size());
    
    return unique_paths;
}

bool TopoPRM::sameTopoPath(const vector<Vector3d>& path1, 
                           const vector<Vector3d>& path2) {
    // 🚀 OPTIMIZED: 使用 Hausdorff 距离判同，更宽容且高效
    vector<Vector3d> pts1 = discretizePath(path1, discretize_points_num_);
    vector<Vector3d> pts2 = discretizePath(path2, discretize_points_num_);
    
    // 计算 Hausdorff 距离
    double hausdorff_dist = 0.0;
    
    // max_{a in pts1} min_{b in pts2} ||a-b||
    for (const auto& a : pts1) {
        double min_dist = numeric_limits<double>::max();
        for (const auto& b : pts2) {
            min_dist = min(min_dist, (a - b).norm());
        }
        hausdorff_dist = max(hausdorff_dist, min_dist);
    }
    
    // max_{b in pts2} min_{a in pts1} ||a-b||
    for (const auto& b : pts2) {
        double min_dist = numeric_limits<double>::max();
        for (const auto& a : pts1) {
            min_dist = min(min_dist, (a - b).norm());
        }
        hausdorff_dist = max(hausdorff_dist, min_dist);
    }
    
    // ✅ GLOBAL FIX: 适度放宽去重阈值 (0.03→0.035: 保留更多拓扑差异路径)
    double path_length = pathLength(path1);
    double threshold = max(0.25, path_length * 0.035);

    ROS_DEBUG("[TopoPRM] sameTopoPath: hausdorff=%.3f, threshold=%.3f, path_len=%.3f",
              hausdorff_dist, threshold, path_length);

    return hausdorff_dist < threshold;
}

vector<Vector3d> TopoPRM::discretizePath(const vector<Vector3d>& path, int pt_num) {
    if (path.size() < 2) return path;
    
    // 计算路径长度
    vector<double> lengths;
    lengths.push_back(0.0);
    for (size_t i = 0; i < path.size() - 1; ++i) {
        double len = (path[i+1] - path[i]).norm();
        lengths.push_back(lengths.back() + len);
    }
    double total_length = lengths.back();
    
    // 均匀采样
    vector<Vector3d> discretized;
    for (int i = 0; i < pt_num; ++i) {
        double target_len = total_length * i / (pt_num - 1);
        
        // 找到对应的段
        size_t seg_idx = 0;
        for (size_t j = 0; j < lengths.size() - 1; ++j) {
            if (target_len >= lengths[j] && target_len <= lengths[j+1]) {
                seg_idx = j;
                break;
            }
        }
        
        // 插值
        double lambda = (target_len - lengths[seg_idx]) / 
                       (lengths[seg_idx+1] - lengths[seg_idx]);
        Vector3d pt = (1 - lambda) * path[seg_idx] + lambda * path[seg_idx+1];
        discretized.push_back(pt);
    }
    
    return discretized;
}

// ============================================================================
// 辅助函数: 路径选择
// ============================================================================
vector<vector<Vector3d>> TopoPRM::selectShortPaths(
    const vector<vector<Vector3d>>& paths) {
    
    if (paths.empty()) return paths;
    
    vector<vector<Vector3d>> short_paths;
    
    // 找到最短路径
    int shortest_idx = shortestPathIndex(paths);
    double min_length = pathLength(paths[shortest_idx]);
    
    // 选择长度在阈值内的路径
    for (const auto& path : paths) {
        double len = pathLength(path);
        if (len < min_length * ratio_to_short_ && short_paths.size() < (size_t)reserve_num_) {
            short_paths.push_back(path);
        }
    }
    
    // 如果太少，至少保留最短的
    if (short_paths.empty()) {
        short_paths.push_back(paths[shortest_idx]);
    }
    
    return short_paths;
}

int TopoPRM::shortestPathIndex(const vector<vector<Vector3d>>& paths) {
    int shortest_idx = 0;
    double min_length = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < paths.size(); ++i) {
        double len = pathLength(paths[i]);
        if (len < min_length) {
            min_length = len;
            shortest_idx = i;
        }
    }
    
    return shortest_idx;
}

double TopoPRM::pathLength(const vector<Vector3d>& path) {
    double length = 0.0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        length += (path[i+1] - path[i]).norm();
    }
    return length;
}

// ============================================================================
// NOTE: Shared utility functions (isPathValid, calculatePathCost, etc.) 
// are defined within the #if 1 block above and used by both TGK and Legacy.
// No need to duplicate them here.
// ============================================================================

} // namespace ego_planner