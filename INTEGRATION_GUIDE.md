# Map Manager集成技术指南

**目标**: 将 map_manager_pub 动态障碍物功能集成到 Topo-MPPI 系统  
**预计时间**: 4周  
**难度**: ⭐⭐⭐ (中等)

---

## 📋 目录

1. [为什么只选 map_manager_pub](#1-为什么只选-map_manager_pub)
2. [需要大改的模块清单](#2-需要大改的模块清单)
3. [详细实施步骤](#3-详细实施步骤)
4. [代码实现示例](#4-代码实现示例)
5. [测试验证方案](#5-测试验证方案)

---

## 1. 为什么只选 map_manager_pub

### 1.1 决策矩阵

| 评估维度 | map_manager_pub | dyn_small_obs_avoidance | 两者都用 |
|---------|----------------|------------------------|---------|
| **集成难度** | ⭐⭐ (简单) | ⭐⭐⭐⭐ (困难) | ⭐⭐⭐⭐⭐ (极难) |
| **开发时间** | 4周 | 8周 | 12周+ |
| **维护成本** | 低 | 高 | 极高 |
| **功能收益** | ✅ 动态避障 | ✅ 小障碍物 | ✅ 全功能 |
| **风险** | 低 | 中 | 高 |
| **与MPPI匹配度** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐ |

### 1.2 核心优势分析

#### map_manager_pub 的关键优势：

```cpp
// ✅ API设计完美匹配MPPI需求
class dynamicMap {
    // 1. 静态碰撞检查（你已有）
    bool isOccupied(Eigen::Vector3d pos);
    
    // 2. 动态障碍物信息（MPPI需要）
    void getDynamicObstacles(
        std::vector<Eigen::Vector3d>& pos,   // 当前位置
        std::vector<Eigen::Vector3d>& vel,   // 当前速度
        std::vector<Eigen::Vector3d>& size   // 包围盒大小
    );
    
    // 3. 预测轨迹（MPPI核心需求）⭐⭐⭐⭐⭐
    void getTPredTraj(
        std::vector<std::vector<std::vector<geometry_msgs::Point>>>& predTraj
    );
    // predTraj[obstacle_id][time_step][xyz]
};
```

**为什么这对MPPI至关重要？**

```
MPPI的本质：在未来时间窗口内采样轨迹并评估代价

你的MPPI horizon: 20步 × 0.1s = 2秒

map_manager预测: 2秒内每个障碍物的轨迹

完美匹配！ ✅
```

#### dyn_small_obs_avoidance 的局限性：

```cpp
// ❌ 主要是地图表示，缺少预测信息
class TimeAccumulatedMap {
    // 只提供点云和距离查询
    double getDistance(Eigen::Vector3d pos);
    
    // ❌ 没有速度信息
    // ❌ 没有预测轨迹
    // ❌ 需要替换整个地图系统
};
```

---

## 2. 需要大改的模块清单

### 2.1 必须修改的文件（按改动量排序）

#### 🔴 重度改动（核心逻辑）

1. **`plan_env/grid_map.h` + `grid_map.cpp`** (改动量: ⭐⭐⭐⭐)
   ```
   需要添加:
   - 动态障碍物存储结构
   - 预测轨迹管理
   - 时变距离查询接口
   - 可视化发布器
   
   预计代码增加: ~500行
   ```

2. **`path_searching/mppi_planner.cpp`** (改动量: ⭐⭐⭐⭐)
   ```
   需要修改:
   - calculateTrajectoryCost() 函数
   - 添加时变碰撞检查
   - 动态障碍物代价项
   - 采样策略调整（可选）
   
   预计代码增加: ~300行
   ```

3. **`bspline_opt/bspline_optimizer.cpp`** (改动量: ⭐⭐⭐)
   ```
   需要修改:
   - 优化约束添加时变障碍物
   - distanceSurfaceCallback 增强
   - 碰撞检查升级
   
   预计代码修改: ~200行
   ```

#### 🟡 中度改动（接口适配）

4. **`plan_manage/planner_manager.cpp`** (改动量: ⭐⭐)
   ```
   需要添加:
   - map_manager接口调用
   - 动态障碍物数据传递
   - 可视化触发
   
   预计代码增加: ~100行
   ```

5. **新增: `plan_env/dynamic_map_interface.h/.cpp`** (全新模块)
   ```
   封装map_manager_pub的ROS接口:
   - 订阅动态障碍物topic
   - 数据格式转换
   - 缓存管理
   
   预计代码: ~400行
   ```

#### 🟢 轻度改动（配置和消息）

6. **`plan_manage/launch/*.launch`** (改动量: ⭐)
   ```
   添加:
   - map_manager节点启动
   - topic映射
   - 参数配置
   ```

7. **新增: `plan_manage/msg/DynamicObstacle.msg`** (可选)
   ```
   如果不直接用map_manager的消息格式
   ```

---

### 2.2 模块依赖关系图

```
┌─────────────────────────────────────────────────┐
│         map_manager_pub (外部)                   │
│  - 动态障碍物检测                                │
│  - 跟踪                                          │
│  - 轨迹预测                                      │
└──────────────────┬──────────────────────────────┘
                   │ ROS Topics:
                   │ /dynamic_map/obstacles
                   │ /dynamic_map/predictions
                   ▼
┌─────────────────────────────────────────────────┐
│   DynamicMapInterface (新增)                     │
│  - 订阅外部障碍物信息                            │
│  - 数据格式转换                                  │
│  - 缓存管理                                      │
└──────────────────┬──────────────────────────────┘
                   │ API调用
                   ▼
┌─────────────────────────────────────────────────┐
│         GridMap (扩展)                           │
│  原有: ESDF + 占据栅格                           │
│  新增: + 动态障碍物存储                          │
│       + 预测轨迹管理                             │
│       + 时变距离查询                             │
└──────────┬────────────┬─────────────────────────┘
           │            │
           ▼            ▼
    ┌──────────┐  ┌──────────────┐
    │   MPPI   │  │  B-spline    │
    │  (修改)  │  │   (修改)     │
    └──────────┘  └──────────────┘
```

---

## 3. 详细实施步骤

### Week 1: 基础设施搭建

#### Day 1-2: 安装和测试 map_manager_pub

```bash
# 1. 克隆仓库
cd ~/catkin_ws/src
git clone https://github.com/Shawn207/map_manager_pub.git

# 2. 安装依赖
sudo apt-get install ros-noetic-pcl-ros ros-noetic-cv-bridge

# 3. 编译
cd ~/catkin_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES="map_manager"

# 4. 测试（用他们的demo）
roslaunch map_manager dynamic_map.launch
```

**验证**: 能看到动态障碍物检测和可视化

---

#### Day 3-4: 创建接口层

**文件**: `plan_env/include/plan_env/dynamic_map_interface.h`

```cpp
#ifndef _DYNAMIC_MAP_INTERFACE_H_
#define _DYNAMIC_MAP_INTERFACE_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <vector>
#include <deque>
#include <map>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

namespace ego_planner {

class DynamicMapInterface {
public:
    struct DynamicObstacle {
        int id;
        double timestamp;
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d size;           // bounding box
        double confidence;
        
        // 预测轨迹 [time_step] -> position
        std::vector<Eigen::Vector3d> predicted_trajectory;
        std::vector<double> predicted_timestamps;
        
        DynamicObstacle() 
            : id(-1), timestamp(0.0), 
              position(Eigen::Vector3d::Zero()),
              velocity(Eigen::Vector3d::Zero()),
              size(Eigen::Vector3d(0.5, 0.5, 0.5)),
              confidence(1.0) {}
    };

private:
    ros::NodeHandle nh_;
    
    // 订阅者
    ros::Subscriber obstacles_sub_;
    ros::Subscriber predictions_sub_;
    
    // 发布者（调试用）
    ros::Publisher debug_markers_pub_;
    
    // 数据存储
    std::map<int, DynamicObstacle> obstacles_;  // id -> obstacle
    std::mutex obstacles_mutex_;
    
    // 参数
    double obstacle_timeout_;        // 障碍物超时时间
    double prediction_horizon_;      // 预测时域
    double prediction_dt_;           // 预测时间步长
    
    // 回调函数
    void obstaclesCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
    void predictionsCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
    
public:
    typedef std::shared_ptr<DynamicMapInterface> Ptr;
    
    DynamicMapInterface() 
        : obstacle_timeout_(2.0), 
          prediction_horizon_(2.0),
          prediction_dt_(0.1) {}
    
    ~DynamicMapInterface() {}
    
    // 初始化
    void init(ros::NodeHandle& nh);
    
    // 更新（定期调用，移除过期障碍物）
    void update(double current_time);
    
    // 查询接口
    bool getObstacles(std::vector<DynamicObstacle>& obstacles) const;
    bool getObstacleById(int id, DynamicObstacle& obstacle) const;
    
    // 预测接口
    bool predictObstaclePosition(int id, double future_time, 
                                Eigen::Vector3d& pred_pos) const;
    
    // 碰撞检查
    double getMinDistanceAtTime(const Eigen::Vector3d& pos, 
                               double time) const;
    
    bool isInDynamicCollision(const Eigen::Vector3d& pos, 
                             double time,
                             double safety_margin = 0.3) const;
    
    // 轨迹批量检查
    bool checkTrajectoryCollision(
        const std::vector<Eigen::Vector3d>& positions,
        const std::vector<double>& timestamps,
        double safety_margin = 0.3) const;
    
    // 可视化
    void publishDebugMarkers();
    
    // 统计信息
    int getObstacleCount() const { return obstacles_.size(); }
    void printStatistics() const;
};

}  // namespace ego_planner

#endif
```

**文件**: `plan_env/src/dynamic_map_interface.cpp`

```cpp
#include <plan_env/dynamic_map_interface.h>
#include <ros/ros.h>

namespace ego_planner {

void DynamicMapInterface::init(ros::NodeHandle& nh) {
    nh_ = nh;
    
    // 读取参数
    nh.param("dynamic_map/obstacle_timeout", obstacle_timeout_, 2.0);
    nh.param("dynamic_map/prediction_horizon", prediction_horizon_, 2.0);
    nh.param("dynamic_map/prediction_dt", prediction_dt_, 0.1);
    
    // 订阅map_manager的输出
    // 注意：这里的topic名字需要根据实际map_manager配置调整
    obstacles_sub_ = nh.subscribe("/dynamic_map/box_visualization_marker", 
                                  10, 
                                  &DynamicMapInterface::obstaclesCallback, 
                                  this);
    
    predictions_sub_ = nh.subscribe("/dynamic_map/traj_marker",
                                   10,
                                   &DynamicMapInterface::predictionsCallback,
                                   this);
    
    // 发布调试信息
    debug_markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        "/topo_mppi/dynamic_obstacles_debug", 10);
    
    ROS_INFO("[DynamicMapInterface] Initialized with:");
    ROS_INFO("  - Obstacle timeout: %.2f s", obstacle_timeout_);
    ROS_INFO("  - Prediction horizon: %.2f s", prediction_horizon_);
    ROS_INFO("  - Prediction dt: %.2f s", prediction_dt_);
}

void DynamicMapInterface::obstaclesCallback(
    const visualization_msgs::MarkerArray::ConstPtr& msg) {
    
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    
    // map_manager发送的是CUBE_LIST类型的marker
    for (const auto& marker : msg->markers) {
        if (marker.action == visualization_msgs::Marker::DELETE || 
            marker.action == visualization_msgs::Marker::DELETEALL) {
            continue;
        }
        
        int id = marker.id;
        double current_time = ros::Time::now().toSec();
        
        // 更新或创建障碍物
        if (obstacles_.find(id) == obstacles_.end()) {
            obstacles_[id] = DynamicObstacle();
            obstacles_[id].id = id;
        }
        
        auto& obs = obstacles_[id];
        obs.timestamp = current_time;
        
        // 从marker提取信息
        obs.position.x() = marker.pose.position.x;
        obs.position.y() = marker.pose.position.y;
        obs.position.z() = marker.pose.position.z;
        
        obs.size.x() = marker.scale.x;
        obs.size.y() = marker.scale.y;
        obs.size.z() = marker.scale.z;
        
        // 速度需要从历史位置估计或从其他topic获取
        // 这里简化处理，实际应该订阅专门的速度topic
        obs.confidence = 1.0;
    }
    
    ROS_DEBUG("[DynamicMapInterface] Updated %zu obstacles", obstacles_.size());
}

void DynamicMapInterface::predictionsCallback(
    const visualization_msgs::MarkerArray::ConstPtr& msg) {
    
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    
    // map_manager的预测轨迹以LINE_STRIP形式发布
    for (const auto& marker : msg->markers) {
        if (marker.type != visualization_msgs::Marker::LINE_STRIP) {
            continue;
        }
        
        int id = marker.id;
        if (obstacles_.find(id) == obstacles_.end()) {
            continue;  // 没有对应的障碍物
        }
        
        auto& obs = obstacles_[id];
        obs.predicted_trajectory.clear();
        obs.predicted_timestamps.clear();
        
        // 提取预测轨迹点
        for (size_t i = 0; i < marker.points.size(); ++i) {
            Eigen::Vector3d pred_pos(
                marker.points[i].x,
                marker.points[i].y,
                marker.points[i].z
            );
            obs.predicted_trajectory.push_back(pred_pos);
            obs.predicted_timestamps.push_back(i * prediction_dt_);
        }
    }
}

void DynamicMapInterface::update(double current_time) {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    
    // 移除超时的障碍物
    auto it = obstacles_.begin();
    while (it != obstacles_.end()) {
        if (current_time - it->second.timestamp > obstacle_timeout_) {
            ROS_DEBUG("[DynamicMapInterface] Removing timeout obstacle %d", it->first);
            it = obstacles_.erase(it);
        } else {
            ++it;
        }
    }
}

bool DynamicMapInterface::getObstacles(
    std::vector<DynamicObstacle>& obstacles) const {
    
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    
    obstacles.clear();
    for (const auto& pair : obstacles_) {
        obstacles.push_back(pair.second);
    }
    
    return !obstacles.empty();
}

bool DynamicMapInterface::predictObstaclePosition(
    int id, double future_time, Eigen::Vector3d& pred_pos) const {
    
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    
    auto it = obstacles_.find(id);
    if (it == obstacles_.end()) {
        return false;
    }
    
    const auto& obs = it->second;
    
    // 如果有预测轨迹，使用插值
    if (!obs.predicted_trajectory.empty()) {
        // 查找对应的时间步
        size_t idx = static_cast<size_t>(future_time / prediction_dt_);
        if (idx < obs.predicted_trajectory.size()) {
            pred_pos = obs.predicted_trajectory[idx];
            return true;
        }
    }
    
    // 否则使用线性外推
    pred_pos = obs.position + obs.velocity * future_time;
    return true;
}

double DynamicMapInterface::getMinDistanceAtTime(
    const Eigen::Vector3d& pos, double time) const {
    
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    
    double min_dist = std::numeric_limits<double>::max();
    
    for (const auto& pair : obstacles_) {
        const auto& obs = pair.second;
        
        // 预测障碍物在time时刻的位置
        Eigen::Vector3d pred_pos;
        if (!predictObstaclePosition(obs.id, time, pred_pos)) {
            continue;
        }
        
        // 计算到包围盒的距离（简化为到中心距离减去尺寸）
        double dist = (pos - pred_pos).norm();
        dist -= obs.size.norm() / 2.0;  // 考虑障碍物大小
        
        min_dist = std::min(min_dist, dist);
    }
    
    return min_dist;
}

bool DynamicMapInterface::isInDynamicCollision(
    const Eigen::Vector3d& pos, 
    double time,
    double safety_margin) const {
    
    double dist = getMinDistanceAtTime(pos, time);
    return dist < safety_margin;
}

bool DynamicMapInterface::checkTrajectoryCollision(
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<double>& timestamps,
    double safety_margin) const {
    
    if (positions.size() != timestamps.size()) {
        ROS_ERROR("[DynamicMapInterface] Position and timestamp size mismatch");
        return true;  // 保守起见返回碰撞
    }
    
    for (size_t i = 0; i < positions.size(); ++i) {
        if (isInDynamicCollision(positions[i], timestamps[i], safety_margin)) {
            return true;  // 检测到碰撞
        }
    }
    
    return false;  // 无碰撞
}

void DynamicMapInterface::publishDebugMarkers() {
    // 可视化当前跟踪的障碍物和预测轨迹
    // 实现略...
}

void DynamicMapInterface::printStatistics() const {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    
    ROS_INFO("[DynamicMapInterface] Statistics:");
    ROS_INFO("  - Total obstacles: %zu", obstacles_.size());
    
    for (const auto& pair : obstacles_) {
        const auto& obs = pair.second;
        ROS_INFO("    Obstacle %d: pos=(%.2f, %.2f, %.2f), vel=(%.2f, %.2f, %.2f), pred_points=%zu",
                 obs.id,
                 obs.position.x(), obs.position.y(), obs.position.z(),
                 obs.velocity.x(), obs.velocity.y(), obs.velocity.z(),
                 obs.predicted_trajectory.size());
    }
}

}  // namespace ego_planner
```

---

#### Day 5: 编译和单元测试

```bash
# 1. 修改CMakeLists.txt
# plan_env/CMakeLists.txt 添加:
add_library(dynamic_map_interface
    src/dynamic_map_interface.cpp
)
target_link_libraries(dynamic_map_interface
    ${catkin_LIBRARIES}
)

# 2. 编译
cd ~/catkin_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES="plan_env"

# 3. 单元测试
rosrun plan_env test_dynamic_map_interface
```

---

### Week 2: GridMap扩展

#### Day 6-8: 扩展GridMap

**文件修改**: `plan_env/include/plan_env/grid_map.h`

```cpp
class GridMap {
private:
    // 原有成员...
    
    // 🆕 新增：动态障碍物接口
    DynamicMapInterface::Ptr dynamic_map_interface_;
    
    // 🆕 缓存：避免重复查询
    mutable std::map<std::pair<Eigen::Vector3d, double>, double> 
        dynamic_distance_cache_;
    mutable std::mutex cache_mutex_;

public:
    // 🆕 新增接口
    void setDynamicMapInterface(DynamicMapInterface::Ptr interface) {
        dynamic_map_interface_ = interface;
    }
    
    // 🆕 时变距离查询（考虑动态障碍物）
    double getDistanceAtTime(const Eigen::Vector3d& pos, double time);
    
    // 🆕 碰撞检查（静态+动态）
    bool isOccupiedAtTime(const Eigen::Vector3d& pos, 
                         double time,
                         double safety_margin = 0.3);
    
    // 🆕 轨迹碰撞检查
    bool isTrajectoryInCollision(
        const std::vector<Eigen::Vector3d>& positions,
        const std::vector<double>& timestamps,
        double safety_margin = 0.3);
    
    // 🆕 获取动态障碍物信息
    bool getDynamicObstacles(
        std::vector<DynamicMapInterface::DynamicObstacle>& obstacles);
};
```

**文件修改**: `plan_env/src/grid_map.cpp`

```cpp
double GridMap::getDistanceAtTime(const Eigen::Vector3d& pos, double time) {
    // 1. 静态障碍物距离（原有功能）
    double static_dist = getDistance(pos);
    
    // 2. 动态障碍物距离（新增）
    double dynamic_dist = std::numeric_limits<double>::max();
    if (dynamic_map_interface_) {
        dynamic_dist = dynamic_map_interface_->getMinDistanceAtTime(pos, time);
    }
    
    // 3. 返回最小距离
    return std::min(static_dist, dynamic_dist);
}

bool GridMap::isOccupiedAtTime(const Eigen::Vector3d& pos, 
                              double time,
                              double safety_margin) {
    // 检查静态和动态障碍物
    double dist = getDistanceAtTime(pos, time);
    return dist < safety_margin;
}

bool GridMap::isTrajectoryInCollision(
    const std::vector<Eigen::Vector3d>& positions,
    const std::vector<double>& timestamps,
    double safety_margin) {
    
    // 静态检查
    for (const auto& pos : positions) {
        if (getInflateOccupancy(pos) == 1) {
            return true;
        }
    }
    
    // 动态检查
    if (dynamic_map_interface_) {
        return dynamic_map_interface_->checkTrajectoryCollision(
            positions, timestamps, safety_margin);
    }
    
    return false;
}

bool GridMap::getDynamicObstacles(
    std::vector<DynamicMapInterface::DynamicObstacle>& obstacles) {
    
    if (!dynamic_map_interface_) {
        return false;
    }
    
    return dynamic_map_interface_->getObstacles(obstacles);
}
```

---

### Week 3: MPPI代价函数改进

#### Day 9-12: 修改MPPI

**文件**: `path_searching/src/mppi_planner.cpp`

```cpp
double MPPIPlanner::calculateTrajectoryCost(const MPPITrajectory& trajectory,
                                          const Vector3d& goal_pos,
                                          const Vector3d& goal_vel) {
    double total_cost = 0.0;
    
    // ========================================
    // 1. 静态障碍物代价（原有）
    // ========================================
    for (int t = 0; t < trajectory.size(); ++t) {
        double dist = grid_map_->getDistance(trajectory.positions[t]);
        
        if (dist < 0.0) {
            return std::numeric_limits<double>::max();  // 碰撞
        }
        
        double obs_cost = w_obstacle_ * exp(-dist / 0.5);
        total_cost += obs_cost;
    }
    
    // ========================================
    // 🆕 2. 动态障碍物代价（新增）
    // ========================================
    std::vector<DynamicMapInterface::DynamicObstacle> dyn_obs;
    if (grid_map_->getDynamicObstacles(dyn_obs) && !dyn_obs.empty()) {
        
        for (int t = 0; t < trajectory.size(); ++t) {
            double time_in_future = t * dt_;
            
            for (const auto& obs : dyn_obs) {
                // 预测障碍物位置
                Eigen::Vector3d pred_obs_pos;
                if (obs.predicted_trajectory.empty()) {
                    // 无预测轨迹，使用线性外推
                    pred_obs_pos = obs.position + obs.velocity * time_in_future;
                } else {
                    // 使用预测轨迹
                    size_t idx = static_cast<size_t>(time_in_future / 0.1);
                    if (idx < obs.predicted_trajectory.size()) {
                        pred_obs_pos = obs.predicted_trajectory[idx];
                    } else {
                        pred_obs_pos = obs.predicted_trajectory.back();
                    }
                }
                
                // 计算距离（考虑障碍物大小）
                double dist = (trajectory.positions[t] - pred_obs_pos).norm();
                dist -= obs.size.norm() / 2.0;
                
                // 碰撞检查
                if (dist < 0.2) {  // 安全边界
                    return std::numeric_limits<double>::max();
                }
                
                // 接近惩罚（2米内）
                if (dist < 2.0) {
                    double proximity_cost = w_dynamic_obs_ * (2.0 - dist) / 2.0;
                    total_cost += proximity_cost;
                }
                
                // 指数衰减代价
                total_cost += w_dynamic_obs_ * 0.5 * exp(-dist / 1.0);
            }
        }
    }
    
    // ========================================
    // 3. 其他代价项（原有）
    // ========================================
    total_cost += w_smoothness_ * smoothnessCost(trajectory);
    total_cost += w_goal_ * goalCost(trajectory, goal_pos, goal_vel);
    total_cost += w_velocity_ * velocityCost(trajectory, goal_vel);
    
    return total_cost;
}
```

**添加新的权重参数**:

```cpp
// mppi_planner.h
class MPPIPlanner {
private:
    double w_dynamic_obs_;  // 🆕 动态障碍物权重
    
public:
    void setCostWeights(double w_obs, double w_smooth, double w_goal, 
                       double w_vel, double w_dyn_obs) {
        w_obstacle_ = w_obs;
        w_smoothness_ = w_smooth;
        w_goal_ = w_goal;
        w_velocity_ = w_vel;
        w_dynamic_obs_ = w_dyn_obs;  // 🆕
    }
};
```

---

### Week 4: B-spline约束和测试

#### Day 13-15: B-spline优化器改进

**文件**: `bspline_opt/src/bspline_optimizer.cpp`

```cpp
void BsplineOptimizer::combineCostRebound(/* ... */) {
    // 原有代价...
    
    // 🆕 添加动态障碍物约束
    addDynamicObstacleConstraints();
}

void BsplineOptimizer::addDynamicObstacleConstraints() {
    std::vector<DynamicMapInterface::DynamicObstacle> dyn_obs;
    if (!grid_map_->getDynamicObstacles(dyn_obs)) {
        return;  // 无动态障碍物
    }
    
    for (int i = 0; i < cps_.size; ++i) {
        double t = i * bspline_interval_;  // 这个点对应的时间
        
        Eigen::Vector3d pos = cps_.points.col(i);
        
        // 检查与所有动态障碍物的距离
        for (const auto& obs : dyn_obs) {
            Eigen::Vector3d pred_pos;
            if (!obs.predicted_trajectory.empty()) {
                size_t idx = static_cast<size_t>(t / 0.1);
                if (idx < obs.predicted_trajectory.size()) {
                    pred_pos = obs.predicted_trajectory[idx];
                } else {
                    continue;
                }
            } else {
                pred_pos = obs.position + obs.velocity * t;
            }
            
            double dist = (pos - pred_pos).norm();
            double safe_dist = obs.size.norm() / 2.0 + 0.3;  // 安全边界
            
            if (dist < safe_dist) {
                // 添加排斥力，将控制点推离障碍物
                Eigen::Vector3d gradient = (pos - pred_pos).normalized();
                double cost_weight = 100.0 * (safe_dist - dist);
                
                g_q_.col(i) += cost_weight * gradient;
            }
        }
    }
}
```

---

#### Day 16-20: 系统集成测试

**测试脚本**: `test_dynamic_integration.launch`

```xml
<launch>
    <!-- 1. 启动map_manager -->
    <include file="$(find map_manager)/launch/dynamic_map.launch" />
    
    <!-- 2. 启动你的规划器 -->
    <include file="$(find plan_manage)/launch/run_in_sim.launch">
        <arg name="use_dynamic_obstacles" value="true"/>
    </include>
    
    <!-- 3. 启动测试场景 -->
    <node pkg="rosbag" type="play" name="test_bag" 
          args="$(find plan_manage)/test_bags/dynamic_obstacle_test.bag"/>
    
    <!-- 4. 可视化 -->
    <node pkg="rviz" type="rviz" name="rviz" 
          args="-d $(find plan_manage)/launch/dynamic_test.rviz"/>
</launch>
```

---

## 4. 代码实现示例（完整）

### 4.1 planner_manager集成

```cpp
// planner_manager.cpp

void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, 
                                       PlanningVisualization::Ptr vis) {
    // 原有初始化...
    
    // 🆕 初始化动态地图接口
    dynamic_map_interface_.reset(new DynamicMapInterface);
    dynamic_map_interface_->init(nh);
    
    // 🆕 将接口传递给GridMap
    grid_map_->setDynamicMapInterface(dynamic_map_interface_);
    
    ROS_INFO("[PlannerManager] Dynamic map interface initialized");
}

bool EGOPlannerManager::reboundReplan(/* ... */) {
    // 🆕 更新动态地图
    double current_time = ros::Time::now().toSec();
    dynamic_map_interface_->update(current_time);
    
    // 🆕 打印统计信息（调试用）
    if (count % 10 == 0) {  // 每10次打印一次
        dynamic_map_interface_->printStatistics();
    }
    
    // 原有规划逻辑...
}
```

---

## 5. 测试验证方案

### 5.1 单元测试清单

```bash
# 测试1: DynamicMapInterface订阅
rostest plan_env test_dynamic_map_subscribe.test

# 测试2: GridMap时变距离查询
rostest plan_env test_grid_map_time_query.test

# 测试3: MPPI动态代价计算
rostest path_searching test_mppi_dynamic_cost.test

# 测试4: B-spline动态约束
rostest bspline_opt test_bspline_dynamic_constraints.test
```

### 5.2 集成测试场景

#### 场景1: 单个匀速障碍物
```yaml
test_name: single_constant_velocity
obstacle:
  position: [5, 0, 1]
  velocity: [0.5, 0, 0]
  size: [0.5, 0.5, 1.0]
expected:
  - no_collision: true
  - planning_success: true
  - avoidance_distance: > 0.5m
```

#### 场景2: 多个障碍物交叉
```yaml
test_name: multiple_crossing
obstacles:
  - {pos: [3, -2, 1], vel: [0, 0.5, 0]}
  - {pos: [3, 2, 1], vel: [0, -0.5, 0]}
expected:
  - no_collision: true
  - planning_time: < 200ms
```

#### 场景3: 高速接近障碍物
```yaml
test_name: high_speed_approach
obstacle:
  position: [10, 0, 1]
  velocity: [-2.0, 0, 0]  # 高速接近
expected:
  - emergency_stop: false
  - avoidance_maneuver: true
  - min_distance: > 0.3m
```

---

## 6. 性能指标

### 6.1 预期性能

| 指标 | 改进前 | 改进后 | 提升 |
|------|-------|-------|------|
| **动态避障能力** | ❌ | ✅ | N/A |
| **MPPI规划时间** | 100ms | 120ms | -20% ⚠️ |
| **总规划时间** | 200ms | 240ms | -20% ⚠️ |
| **安全性** | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | +67% |
| **适用场景** | 静态 | 静态+动态 | +100% |

**注意**: 计算时间增加是正常的，因为增加了动态障碍物处理。通过Week 1的并行优化可以补偿。

### 6.2 优化建议

如果性能不达标，可以：

1. **减少预测horizon**: 从2s降至1.5s
2. **降低预测频率**: 从10Hz降至5Hz
3. **增加MPPI并行度**: 使用Week 1的OpenMP优化
4. **缓存优化**: 添加距离查询缓存

---

## 7. 常见问题

### Q1: map_manager检测不到障碍物？

**检查清单**:
```bash
# 1. 确认topic是否发布
rostopic list | grep dynamic_map

# 2. 查看消息内容
rostopic echo /dynamic_map/box_visualization_marker

# 3. 检查相机数据
rostopic echo /camera/depth/image_raw

# 4. 查看map_manager日志
rosnode info /dynamic_map_node
```

### Q2: MPPI规划时间过长？

**优化步骤**:
1. 减少采样数: 1000 → 500
2. 启用并行化（Week 1改进2）
3. 减少horizon: 20 → 15步
4. 增加cache

### Q3: 频繁误检碰撞？

**调整参数**:
```yaml
# dynamic_map_param.yaml
obstacle_confidence_threshold: 0.8  # 提高置信度阈值
safety_margin: 0.5  # 增大安全边界
prediction_uncertainty: 0.3  # 考虑预测不确定性
```

---

## 8. 总结

### 8.1 集成完成标志

✅ **Week 1**: DynamicMapInterface正常订阅和解析数据  
✅ **Week 2**: GridMap能查询时变距离  
✅ **Week 3**: MPPI能避开动态障碍物  
✅ **Week 4**: 完整系统通过所有测试场景

### 8.2 下一步

完成map_manager集成后，可以考虑：

1. **性能优化**: 参考PROJECT_ANALYSIS_AND_IMPROVEMENT.md的改进1-3
2. **细小障碍物**: 如果真有需求，再考虑dyn_small_obs_avoidance
3. **学习优化**: MPPI参数自适应调整
4. **GPU加速**: 长期考虑

### 8.3 联系支持

遇到问题？
- 查看 `/docs/FAQ.md`
- 提交 GitHub Issue
- 联系开发者邮件列表

---

**文档版本**: 1.0  
**最后更新**: 2025年10月26日  
**作者**: GitHub Copilot & Project Team
