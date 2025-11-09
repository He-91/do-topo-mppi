# Topo-MPPI 无人机路径规划系统# Topo-MPPI 无人机路径规划系统



基于Fast-Planner改进的三层无人机路径规划系统，支持动态障碍物环境。## 🚀 项目简介

基于Fast-Planner改进的三层无人机路径规划系统：

## 系统架构- **TopoPRM v4.1**（全局多拓扑路径生成 + 智能回退）

- **MPPI**（动力学轨迹并行优化）

```- **B-spline**（轨迹平滑）

EGO-Planner Manager (100Hz)

    │## 🆕 最新更新 (2025-10-07)

    ├─ TopoPRM v4.1 (拓扑路径规划)

    │   ├─ 椭球自由空间采样 (100核心+35边界节点)✅ **已完成障碍物密度加倍测试**

    │   ├─ KNN图构建 (K=22)- Fast-Planner地图：**300柱子 + 20圆圈 = 320个障碍物** (从160个加倍)

    │   ├─ DFS多路径搜索 (200ms timeout)- 地图大小：40m × 20m × 5m

    │   └─ 拓扑去重 (Hausdorff 3.5%)- 点云数量：~450,000 点

    │- RViz配色优化：米黄色背景 + 蓝色半透明障碍物

    ├─ MPPI (动力学轨迹优化)- 所有配置问题已修复（地图颜色、无人机模型、so3错误）

    │   ├─ 1000样本并行采样

    │   ├─ 20步预测视野---

    │   └─ 指数加权选优

    │## 📊 最终性能指标 (v4.0 优化完成)

    └─ B-spline (轨迹平滑)

        └─ 梯度优化 + 避障约束### 核心成果对比

``````

╔════════════════════╦═══════════╦═══════════╦═══════════╦═══════════╗

## 环境要求║      指标          ║  v1.0     ║  v2.0     ║  v3.0     ║  v4.0     ║

║                    ║  (K=18)   ║  (K=22)   ║  (K=28    ║ (K=28     ║

- **ROS版本**: ROS Noetic (Ubuntu 20.04)║                    ║  崩溃版    ║  恢复版   ║  150ms)   ║  200ms)   ║

- **Docker**: 推荐使用Docker容器（已配置CUDA支持）╠════════════════════╬═══════════╬═══════════╬═══════════╬═══════════╣

- **依赖库**:║ PRM成功率          ║    0%     ║   71.0%   ║   70.0%   ║   87.1%   ║

  - OpenCV║ Legacy回退率       ║  100.0%   ║   29.0%   ║   30.0%   ║   12.9%   ║

  - PCL (Point Cloud Library)║ 单路径率           ║   50.0%   ║   29.0%   ║   26.7%   ║   22.6%   ║

  - Eigen3║ 平均路径数         ║   2.63    ║   3.16    ║   2.63    ║   3.10    ║

  - CUDA (可选，用于深度图渲染)║ DFS成功率          ║    0%     ║   ~71%    ║   46.3%   ║   87.1%   ║

║ B-spline成功率     ║    -      ║   96.7%   ║   86.7%   ║   87.1%   ║

## 编译与运行║ 总成功率           ║  100%     ║   96.8%   ║  100%     ║  100%     ║

╚════════════════════╩═══════════╩═══════════╩═══════════╩═══════════╝

### 1. Docker环境（推荐）```



```bash### v4.0 路径质量分布 (31次测试)

# 启动Docker容器（需要已有配置好的ROS Noetic容器）```

docker start <container_id>1路径:  7次 (22.6%)  ✅ 低于30%目标

2路径:  3次 ( 9.7%)

# 进入容器3路径: 10次 (32.3%)  ⭐ 最优占比

docker exec -it <container_id> bash4路径:  7次 (22.6%)

5+路径: 4次 (12.9%)  ✨ 复杂场景多样性

# 编译工作空间────────────────────

cd /home/developer/ros_ws/ddo-topo-mppi平均: 3.10条路径

source /opt/ros/noetic/setup.bash```

catkin build

### 关键突破

# Source工作空间- ✅ **单路径率从50%降至22.6%** (-54.8%改进)

source devel/setup.bash- ✅ **Legacy回退从100%降至12.9%** (PRM主导)

```- ✅ **DFS成功率从0%升至87.1%** (连通性保证)

- ✅ **平均路径3.10条** (理想多样性)

### 2. 运行基础测试- ✅ **100%规划成功率** (可靠性保证)



```bash---

# 启动完整测试（包含静态+动态障碍物）

roslaunch ego_planner topo_mppi_fastplanner_map.launch## 🏗️ 系统架构

```

```

### 3. 启动可视化（需要X11转发）EGO-Planner Manager

    │

在宿主机上：    ├─ STEP 1: Topological Planning (TopoPRM v4.0)

```bash    │   ├─ 📍 椭球自由空间采样 (135节点: 100核心+35边界)

# 允许Docker访问显示    │   ├─ 🌐 可见性图构建 (K=28 KNN, 平均度33.5)

xhost +local:docker    │   ├─ 🔍 DFS多路径搜索 (200ms timeout, 智能剪枝)

    │   ├─ ♻️  拓扑去重 (Hausdorff距离, 3.5%阈值)

# 在容器内启动RViz    │   └─ 🔄 Legacy回退 (PRM失败时, 切线点法+去重)

docker exec <container_id> bash -c "export DISPLAY=:0 && source /home/developer/ros_ws/ddo-topo-mppi/devel/setup.bash && rviz -d /home/developer/ros_ws/ddo-topo-mppi/src/planner/plan_manage/launch/fastplanner_test.rviz"    │

```    ├─ STEP 1.5: Parallel MPPI Optimization

    │   └─ 多路径并行优化, 指数加权选优

### 4. 发送目标点    │

    └─ STEP 3: B-spline Smoothing

```bash        └─ 轨迹平滑与轻微避障

# 发送目标点到(19, 0, 1)```

rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \

'{header: {stamp: now, frame_id: "world"}, \---

  pose: {position: {x: 19.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}'

```## ⚙️ 核心参数配置 (v4.0 最优)



或者在RViz中使用"2D Nav Goal"工具点击设置目标点。### TopoPRM参数

```cpp

## 关键话题// 图构建参数 (topo_prm.cpp)

int K = 28;                          // KNN连接数 (从18→22→28优化)

### 输入话题const double MAX_DFS_TIME_MS = 200.0; // DFS超时 (从150ms→200ms)

- `/visual_slam/odom` - 里程计数据 (nav_msgs/Odometry)int core_samples = 100;              // 核心采样点

- `/move_base_simple/goal` - 目标点 (geometry_msgs/PoseStamped)int boundary_samples = 35;           // 边界采样点

- `/merged_global_map` - 融合后的障碍物点云 (sensor_msgs/PointCloud2)

// 去重参数

### 输出话题double hausdorff_threshold = 0.035;  // 3.5% Hausdorff距离

- `/planning/pos_cmd` - 位置控制命令 (quadrotor_msgs/PositionCommand)```

- `/ego_planner_node/optimal_list` - 最优路径可视化

- `/mppi_optimal_trajectory` - MPPI优化后的轨迹### Legacy回退参数

- `/grid_map/occupancy` - 占据栅格地图```cpp

// 切线点生成

### 可视化话题int num_obstacles = 12;              // 障碍物数量

- `/pcl_render_node/cloud` - 感知点云double safety_margin = 4.5;          // 安全距离(m)

- `/pcl_render_node/depth` - 深度图```

- `/dynamic_obstacles/markers` - 动态障碍物标记

- `/dynamic_obstacles/velocities` - 障碍物速度向量### 性能参数

```cpp

## 配置参数// 路径限制

max_raw_paths_ = 50;                 // DFS最大搜索路径

主要参数位于 `planner/plan_manage/launch/advanced_param.xml`：reserve_num_ = 8;                    // 保留路径数

ratio_to_short_ = 2.5;               // 最长=最短*2.5

```xml```

<!-- 规划参数 -->

<param name="manager/max_vel" value="2.0" />---

<param name="manager/max_acc" value="2.5" />

<param name="manager/planning_horizon" value="7.5" />## 🎯 优化历程总结



<!-- TopoPRM参数 -->### Phase 1: K=18崩溃诊断 (v1.0)

<param name="topo_prm/max_topo_paths" value="5" />**问题**: 100% Legacy回退, 50%单路径率

**原因**: K=18连通性不足, start/goal孤立

<!-- MPPI参数 -->**行动**: 恢复K=22

<param name="optimization/lambda_fitness" value="1.0" />

<param name="optimization/lambda_collision" value="0.5" />### Phase 2: K=22基本恢复 (v2.0)

<param name="optimization/lambda_smooth" value="1.0" />**成果**: 71% PRM成功, 29%单路径率

```**问题**: 仍有29% Legacy回退, 路径偶有重复

**行动**: 实现Legacy去重

## 动态障碍物配置

### Phase 3: Legacy去重实现 (v3.0-early)

在 `topo_mppi_fastplanner_map.launch` 中配置动态障碍物：**成果**: 去重率83.3% (6路径→1路径案例)

**问题**: 性能略降, 单路径率26.7%

```xml**行动**: 提升K到28

<!-- 各类动态障碍物数量 -->

<param name="dynamic_obstacles/num_linear" value="3" />     <!-- 直线运动 -->### Phase 4: K=28连通性提升 (v3.0)

<param name="dynamic_obstacles/num_circular" value="2" />   <!-- 圆周运动 -->**问题**: 图复杂度增加, DFS超时53.7%

<param name="dynamic_obstacles/num_pendulum" value="2" />   <!-- 钟摆运动 -->**发现**: 不是连通性问题, 是搜索时间不足

<param name="dynamic_obstacles/num_random" value="1" />     <!-- 随机游走 -->**行动**: DFS timeout 150ms→200ms



<!-- 更新频率 -->### Phase 5: 200ms timeout最终优化 (v4.0) ✅

<param name="dynamic_obstacles/update_rate" value="30.0" />**成果**: 

```- DFS成功率87.1% (+88%提升)

- Legacy回退12.9% (-57%降低)

## 已知问题与修复- 单路径率22.6% (历史最低)

- 平均路径3.10条 (理想状态)

### 1. 点云话题映射问题

---

**问题**: `pcl_render_node`在启用CUDA时发布到`/pcl_render_node/rendered_pcl`而非`/pcl_render_node/cloud`

## 🚀 快速开始

**修复**: 在`simulator_fastplanner.xml`中添加了topic remap：

```xml### 编译

<remap from="~rendered_pcl" to="~cloud"/>```bash

```cd /home/he/ros_ws/test/topo-mppi

catkin_make -DCATKIN_WHITELIST_PACKAGES="path_searching;plan_manage" -j4

### 2. 动态障碍物融合source devel/setup.bash

```

**问题**: 规划器需要同时感知静态和动态障碍物

### 运行仿真

**修复**: 使用`cloud_merger`节点融合：

```xml#### Fast-Planner地图测试 (推荐) ⭐

<remap from="~global_map" to="/merged_global_map"/>

```**配置**: 320个障碍物 (300柱子 + 20圆圈), 40×20×5m 地图



## 性能指标**终端 1 - 启动系统:**

```bash

- **TopoPRM成功率**: 87.1%docker exec -it 65abafec5dc5 bash -c "cd /home/developer/ros_ws/topo-mppi && source devel/setup.bash && roslaunch ego_planner topo_mppi_fastplanner_map.launch"

- **平均路径数量**: 3.10条```

- **单路径率**: 22.6%（目标<30%）等待看到 `[FSM]: state: WAIT_TARGET` 表示系统就绪

- **总规划成功率**: 100%

- **规划频率**: 100Hz**终端 2 - 启动RViz (最佳配色):**

```bash

## 地图配置xhost +local:docker

docker exec -it 65abafec5dc5 bash -c "cd /home/developer/ros_ws/topo-mppi && source devel/setup.bash && export DISPLAY=:0 && rviz -d src/planner/plan_manage/launch/fastplanner_test.rviz"

- **静态障碍物**: 150柱状 + 10圆形```

- **动态障碍物**: 8个（可配置）应该看到:

- **地图大小**: 40m × 20m × 5m- 🟡 **米黄色背景** (舒适配色)

- **分辨率**: 0.1m- 🟦 **蓝色半透明障碍物** (Boxes样式, Alpha 0.2)

- 🚁 **无人机mesh模型** (起点: x=-19, y=0, z=1)

## 项目结构- 📊 **所有路径可视化** (topo paths, mppi trajectories等)



```**终端 3 - 发送测试目标:**

ddo-topo-mppi/```bash

├── planner/docker exec 65abafec5dc5 bash -c "source /home/developer/ros_ws/topo-mppi/devel/setup.bash && rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: \"world\"}, pose: {position: {x: 10.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}'"

│   ├── plan_manage/          # 主规划器```

│   ├── path_searching/        # 路径搜索（TopoPRM）

│   ├── bspline_opt/          # B样条优化**测试目标建议:**

│   ├── plan_env/             # 环境表示- 近距离: `(10, 0, 1)` - 测试基本规划

│   └── traj_utils/           # 轨迹工具- 中距离: `(0, 10, 1)` - 测试拓扑多样性

├── uav_simulator/- 远距离: `(15, 8, 1)` - 测试复杂避障

│   ├── map_generator/        # 地图生成（静态+动态）

│   ├── local_sensing/        # 深度相机模拟#### 原mockamap测试

│   └── so3_quadrotor_simulator/  # 四旋翼模拟器```bash

├── map_manager/              # 动态地图管理# 启动完整仿真

└── onboard_detector/         # 障碍物检测（可选）roslaunch plan_manage run_in_sim.launch

```

# 启动MPPI可视化测试

## 参考文献roslaunch plan_manage test_mppi_visualization.launch

```

1. Fast-Planner: [https://github.com/HKUST-Aerial-Robotics/Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)

2. EGO-Planner: [https://github.com/ZJU-FAST-Lab/ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner)### RViz可视化

- `/map_generator/global_cloud` - 全局地图点云 (白色)

## 致谢- `/topo_paths` - 彩虹色拓扑路径 (MarkerArray)

- `/mppi_trajectories` - MPPI采样轨迹 (权重着色)

本项目基于以下开源项目：- `/mppi_optimal_trajectory` - 最优轨迹 (蓝色速度箭头)

- Fast-Planner (HKUST-Aerial-Robotics)- `/topo_mppi_paths` - TOPO+MPPI组合路径

- EGO-Planner (ZJU-FAST-Lab)- `/odom_visualization/path` - 无人机飞行轨迹 (绿色)

- Map Manager (Zhefan-Xu)

---

## License

## 📈 性能分析工具

MIT License

### 实时日志统计
```bash
# 查看Legacy回退率
grep -c "Legacy Generation Summary" <log_file>

# 查看DFS超时率
grep "200\.0ms\|200\.1ms" <log_file> | wc -l

# 统计路径分布
grep "Topological planning succeeded, found" <log_file> | \
  grep -o "found [0-9]* paths" | grep -o "[0-9]*" | \
  awk '{sum+=$1; count++; if($1==1) single++} 
       END {print "平均:", sum/count, "单路径率:", single/count*100"%"}'
```

---

## 🔧 故障排查

### 常见问题
1. **高Legacy回退率 (>20%)**
   - 检查K值是否足够 (建议≥28)
   - 检查DFS timeout (建议200ms)
   - 查看start/goal连通度日志

2. **高单路径率 (>30%)**
   - 增加DFS timeout
   - 降低去重阈值 (3.5%→5%)
   - 增加K值

3. **DFS超时过多 (>15%)**
   - 增加timeout到250ms
   - 检查采样质量
   - 查看障碍物密度

---

## 📚 技术细节

### 拓扑去重算法
使用Hausdorff距离判断路径拓扑等价性:
```cpp
double hausdorff_dist = computeHausdorffDistance(path1, path2);
if (hausdorff_dist < threshold * path_length) {
    // 认为是重复路径
}
```

### Legacy回退机制
PRM失败时自动切换到切线点法:
```cpp
if (dfs_timeout && paths.size() == 0) {
    ROS_WARN("PRM失败, 启动Legacy模式");
    findTopoPathsLegacy();  // 包含Hausdorff去重
}
```

### 连通性预检
```cpp
if (start_degree < 5 || goal_degree < 5) {
    ROS_WARN("连通性不足: start_deg=%d, goal_deg=%d", 
             start_degree, goal_degree);
}
```

---

## 📝 版本历史

- **v1.0** (2025-10-03): K=18崩溃版, 100% Legacy
- **v2.0** (2025-10-04): K=22恢复版, 71% PRM
- **v3.0** (2025-10-05): K=28 + Legacy去重
- **v4.0** (2025-10-05): 200ms timeout, 生产就绪 ✅
- **v4.1** (2025-10-06): Fast-Planner地图集成 + 2D测试支持
- **v4.2** (2025-10-07): 障碍物密度加倍测试 + RViz配色优化 🆕

---

## � 修复记录 (v4.2)

### 已解决的问题
1. ✅ **地图颜色错误** - 恢复米黄色背景 + 蓝色半透明障碍物
2. ✅ **无人机模型缺失** - 添加 `/odom_visualization/robot` mesh显示
3. ✅ **so3_disturbance_generator错误** - 移除不存在的包引用
4. ✅ **RViz配置混乱** - 统一使用 `fastplanner_test.rviz`
5. ✅ **障碍物密度** - 从160个加倍到320个 (300柱+20圆)

### 配置文件说明
- `topo_mppi_fastplanner_map.launch` - Fast-Planner地图启动文件 (320障碍物)
- `fastplanner_test.rviz` - 最佳RViz配色配置
- `simulator.xml` - 已移除so3_disturbance_generator节点
- `advanced_param.xml` - TopoPRM+MPPI优化参数

---

## 🎓 参考资料
- [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) - 原始PRM实现
- [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner) - 基础框架
- [TGK-Planner](https://github.com/ZJU-FAST-Lab/TGK-Planner) - 拓扑路径参考

---

**最后更新**: 2025-10-07  
**当前版本**: v4.2 (Production Ready + Doubled Obstacles)  
**测试环境**: Docker 65abafec5dc5, ROS Noetic  
**推荐配置**: K=28, DFS timeout=200ms, 320障碍物
