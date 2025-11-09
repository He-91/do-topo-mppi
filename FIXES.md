# 系统修复与配置说明

## 修复日期: 2025-11-10

## 关键修复

### 1. 点云话题映射问题

#### 问题描述
`pcl_render_node`在启用CUDA编译时，发布点云到`/pcl_render_node/rendered_pcl`话题，但规划器订阅的是`/pcl_render_node/cloud`话题，导致深度图和点云数据无法正常传递给规划器。

#### 根本原因
- 项目中存在两个版本的pcl_render_node源文件：
  - `pointcloud_render_node.cpp` (无CUDA版本) - 发布到 `~cloud`
  - `pcl_render_node.cpp` (CUDA版本) - 发布到 `~rendered_pcl`
- CMakeLists.txt根据CUDA是否启用选择编译不同版本
- 实际环境启用了CUDA，使用了`pcl_render_node.cpp`

#### 修复方案
在 `planner/plan_manage/launch/simulator_fastplanner.xml` 中添加话题重映射：

```xml
<node pkg="local_sensing_node" type="pcl_render_node" name="pcl_render_node" output="screen">
    <!-- ... 其他参数 ... -->
    
    <!-- 修复：将rendered_pcl重映射到cloud，保持接口一致性 -->
    <remap from="~rendered_pcl" to="~cloud"/>
</node>
```

#### 验证方法
```bash
# 检查话题发布者
rostopic info /pcl_render_node/cloud

# 应该看到：
# Publishers: 
#  * /pcl_render_node (http://...)
```

---

### 2. 动态障碍物点云融合

#### 问题描述
规划器需要同时感知静态地图和动态障碍物，但原始配置中`pcl_render_node`只订阅静态地图。

#### 修复方案
修改 `simulator_fastplanner.xml` 中的global_map订阅：

```xml
<!-- 修改前 -->
<remap from="~global_map" to="/map_generator/global_cloud"/>

<!-- 修改后 -->
<remap from="~global_map" to="/merged_global_map"/>
```

#### 数据流说明
```
random_forest (静态地图生成器)
    ↓ /map_generator/global_cloud
cloud_merger (点云融合节点)
    ↑ /dynamic_obstacles/cloud (动态障碍物)
    ↓ /merged_global_map (融合点云)
pcl_render_node (深度渲染)
    ↓ /pcl_render_node/cloud (感知点云)
ego_planner_node (规划器)
```

---

## 编译与部署

### 1. 工作空间路径映射

**重要**: Docker容器路径映射
- 宿主机: `/home/he/ros_ws/test/ddo-topo-mppi/`
- 容器内: `/home/developer/ros_ws/ddo-topo-mppi/`

两者指向同一物理位置，修改源代码后需要在容器内重新编译。

### 2. 完整编译流程

```bash
# 在Docker容器内
cd /home/developer/ros_ws/ddo-topo-mppi
source /opt/ros/noetic/setup.bash
catkin build

# 编译成功输出
[build] Summary: All 20 packages succeeded!
```

### 3. 运行测试

```bash
# 启动完整系统
roslaunch ego_planner topo_mppi_fastplanner_map.launch

# 在另一个终端启动RViz（需要X11转发）
xhost +local:docker  # 宿主机执行
docker exec <container_id> bash -c "export DISPLAY=:0 && rviz -d ..."

# 发送目标点
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
'{header: {stamp: now, frame_id: "world"}, \
  pose: {position: {x: 19.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}'
```

---

## 依赖包编译顺序

由于包之间的依赖关系，首次编译可能遇到`pose_utils`头文件找不到的问题。

### 解决方案
```bash
# 清理并重新编译（会自动处理依赖顺序）
catkin clean -y
catkin build
```

### 依赖关系
- `pose_utils` → `odom_visualization`, `multi_map_server`
- `quadrotor_msgs` → `so3_control`, `ego_planner`
- `map_generator` → `cloud_merger`, `pcl_render_node`

---

## 常见问题

### Q1: RViz深度图不显示
**A**: 检查`/pcl_render_node/depth`话题是否在发布：
```bash
rostopic hz /pcl_render_node/depth
# 应该看到约30Hz的更新率
```

### Q2: 规划器无法获取点云
**A**: 确认话题映射已生效：
```bash
rostopic info /pcl_render_node/cloud
# Publishers应该包含 /pcl_render_node
```

### Q3: 动态障碍物不可见
**A**: 检查融合节点是否正常工作：
```bash
rostopic hz /merged_global_map
# 应该看到约30Hz的更新率

rostopic info /merged_global_map
# Publishers应该包含 /cloud_merger
```

---

## 文件修改清单

### 修改的文件
1. `planner/plan_manage/launch/simulator_fastplanner.xml`
   - 添加 `~rendered_pcl` → `~cloud` 重映射
   - 修改 `~global_map` → `/merged_global_map` 订阅

### 删除的文件（冗余文档）
- `CODE_REVIEW_AND_CORRECTIONS.md`
- `COMPARISON_WITH_FASTPLANNER.md`
- `DECISION_SUMMARY.md`
- `INTEGRATION_GUIDE.md`
- `PERFORMANCE_COMPARISON.md`
- `PROJECT_ANALYSIS_AND_IMPROVEMENT.md`
- `analysis_summary.txt`
- `test.txt`

### 新增/更新的文件
- `README.md` - 完全重写，包含实际使用指南
- `FIXES.md` - 本文件，记录修复细节

---

## 性能验证

### 测试场景
- 静态障碍物: 150柱状 + 10圆形
- 动态障碍物: 8个（3直线+2圆周+2钟摆+1随机）
- 地图: 40m × 20m × 5m

### 预期结果
- 所有ROS节点正常启动
- RViz中可见点云和深度图
- 发送目标点后能生成多条拓扑路径
- 规划成功率: 100%

---

## 维护建议

1. **定期备份**launch文件修改
2. **记录**任何新的话题重映射
3. **测试**修改后的完整功能
4. **更新**本文档和README

---

最后更新: 2025-11-10
维护者: GitHub Copilot
