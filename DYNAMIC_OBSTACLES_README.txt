# 动态障碍物仿真环境使用说明

## ✅ 已完成的工作

### 1. 动态障碍物生成器 (`dynamic_obstacle_generator`)
- **位置**: `uav_simulator/map_generator/src/dynamic_obstacle_generator.cpp`
- **功能**: 生成多种运动模式的动态障碍物
  - 直线运动 (LINEAR): 往返直线运动
  - 圆周运动 (CIRCULAR): 绕中心点旋转
  - 钟摆运动 (PENDULUM): 往复摆动
  - 随机游走 (RANDOM_WALK): 随机目标点导航

### 2. 点云融合节点 (`cloud_merger`)
- **位置**: `uav_simulator/map_generator/src/cloud_merger.cpp`
- **功能**: 合并静态和动态障碍物点云
- **输入**: 
  - `/map_generator/global_cloud` (静态障碍物)
  - `/dynamic_obstacles/cloud` (动态障碍物)
- **输出**: `/pcl_render_node/cloud` (融合点云)

### 3. Launch文件
- `simple_dynamic_test.launch`: 简单测试（仅动态障碍物+RViz）
- `dynamic_obstacles_test.launch`: 完整测试（包含静态地图+规划器）

## 🚀 快速开始

### 方式1: 仅测试动态障碍物（推荐先用这个）

```bash
# 在Docker容器中
source ~/ros_ws/ddo-topo-mppi/devel/setup.bash
roslaunch ego_planner simple_dynamic_test.launch
```

### 方式2: 完整测试（需要集成你的动态避障算法后）

```bash
source ~/ros_ws/ddo-topo-mppi/devel/setup.bash
roslaunch ego_planner dynamic_obstacles_test.launch
```

## 📊 ROS话题

### 动态障碍物相关
- `/dynamic_obstacles/cloud` - 动态障碍物点云 (sensor_msgs/PointCloud2)
- `/dynamic_obstacles/markers` - 可视化标记 (visualization_msgs/MarkerArray)
- `/dynamic_obstacles/velocities` - 速度向量可视化 (visualization_msgs/MarkerArray)

### 融合点云
- `/pcl_render_node/cloud` - 融合后的总点云（供规划器使用）

## ⚙️ 参数配置

在launch文件中可调整：

```xml
<!-- 动态障碍物数量 -->
<param name="dynamic_obstacles/num_linear" value="3" />     <!-- 直线运动 -->
<param name="dynamic_obstacles/num_circular" value="2" />   <!-- 圆周运动 -->
<param name="dynamic_obstacles/num_pendulum" value="2" />   <!-- 钟摆运动 -->
<param name="dynamic_obstacles/num_random" value="1" />     <!-- 随机游走 -->

<!-- 更新频率 -->
<param name="dynamic_obstacles/update_rate" value="20.0" />  <!-- Hz -->

<!-- 分辨率 -->
<param name="dynamic_obstacles/resolution" value="0.15" />   <!-- 米 -->
```

## 🔧 下一步：集成你的动态避障算法

你提到有现成的动态避障算法。集成步骤：

1. **确认算法输入需求**
   - 障碍物位置？速度？轨迹预测？
   - 需要的数据格式？

2. **可能需要的修改**
   - 扩展`GridMap`支持障碍物速度缓存
   - 添加障碍物轨迹预测模块
   - 修改MPPI代价函数考虑动态障碍物

3. **集成点**
   - 你的算法可以订阅`/dynamic_obstacles/velocities`获取速度信息
   - 或者我可以添加新的话题发布障碍物状态（位置+速度+ID）

## 📝 文件清单

```
uav_simulator/map_generator/
├── src/
│   ├── dynamic_obstacle_generator.cpp  # 动态障碍物生成器
│   ├── cloud_merger.cpp                # 点云融合节点
│   └── random_forest_sensing.cpp       # 原有静态地图生成器
├── CMakeLists.txt                      # 已更新（添加新节点）

planner/plan_manage/launch/
├── simple_dynamic_test.launch          # 简单测试launch
├── simple_dynamic_test.rviz            # 简单测试RViz配置
├── dynamic_obstacles_test.launch       # 完整测试launch
└── dynamic_test.rviz                   # 完整测试RViz配置
```

## 🎯 当前状态

✅ 编译成功  
✅ 动态障碍物生成器可运行  
✅ 点云融合节点已实现  
⏳ 等待集成动态避障算法  
⏳ 等待测试验证

告诉我你的算法需要什么输入，我来帮你完成集成！
