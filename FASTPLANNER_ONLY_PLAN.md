# 🎯 FastPlanner-Only改造方案

## 一、改造目标

**完全照着FastPlanner改造，只使用FastPlanner的Topo路径规划**

- ❌ 移除所有Legacy回退机制
- ✅ 只保留FastPlanner PRM+DFS算法
- ✅ 优化参数降低DFS超时率（23.5% → <5%）
- ✅ 保证系统鲁棒性（失败时返回空路径，上层处理）

---

## 二、当前问题分析

### 2.1 当前系统架构

```cpp
bool TopoPRM::searchTopoPaths(start, goal, topo_paths) {
    // Step 1: 采样
    sample_points = sampleFreeSpaceInEllipsoid(...);
    
    if (sample_points.size() < 10) {
        return findTopoPathsLegacy(start, goal);  // ❌ Legacy回退1
    }
    
    // Step 2: 构建图
    buildVisibilityGraph(...);
    
    if (graph_nodes_.size() < 3) {
        return findTopoPathsLegacy(start, goal);  // ❌ Legacy回退2
    }
    
    // Step 3: DFS搜索
    raw_paths = searchMultiplePaths(...);
    
    if (raw_paths.empty()) {
        return findTopoPathsLegacy(start, goal);  // ❌ Legacy回退3
    }
    
    // Step 4-5: 去重+选择
    return topo_paths;
}
```

**问题**: 3个Legacy回退点导致系统依赖非FastPlanner算法

### 2.2 DFS超时率分析

**当前性能** (test.txt):
```
总重规划: 17次
DFS超时: 4次 (23.5%)
Legacy救援: 4次 (100%成功)
```

**超时原因**:
1. 障碍物过大: 6.10m半径 + 4.50m安全距离 = 10.6m避让圈
2. 图规模爆炸: 600-670节点, 搜索空间10^16-10^23
3. 狭窄通道: 椭球采样覆盖不足

---

## 三、FastPlanner-Only改造方案

### 3.1 方案A: 激进模式 - 完全移除Legacy

**目标**: 照着FastPlanner原版，失败就失败

```cpp
bool TopoPRM::searchTopoPaths(const Vector3d& start, const Vector3d& goal,
                             vector<TopoPath>& topo_paths) {
    topo_paths.clear();
    
    // Step 1: 采样
    vector<Vector3d> sample_points = sampleFreeSpaceInEllipsoid(start, goal, 100);
    vector<Vector3d> boundary_points = sampleBoundaryLayer(start, goal, 35);
    sample_points.insert(sample_points.end(), boundary_points.begin(), boundary_points.end());
    
    if (sample_points.size() < 10) {
        ROS_ERROR("[TopoPRM] ❌ 采样失败: 仅%zu个点 (需要≥10)", sample_points.size());
        return false;  // ✅ 直接返回失败,不回退Legacy
    }
    
    // Step 2: 构建图
    buildVisibilityGraph(start, goal, sample_points);
    
    if (graph_nodes_.size() < 3) {
        ROS_ERROR("[TopoPRM] ❌ 图构建失败: 仅%zu个节点 (需要≥3)", graph_nodes_.size());
        clearGraph();
        return false;  // ✅ 直接返回失败,不回退Legacy
    }
    
    // Step 3: DFS搜索
    GraphNode* start_node = graph_nodes_[0];
    GraphNode* goal_node = graph_nodes_[1];
    vector<vector<Vector3d>> raw_paths = searchMultiplePaths(start_node, goal_node);
    
    if (raw_paths.empty()) {
        ROS_ERROR("[TopoPRM] ❌ DFS搜索失败: 未找到路径");
        clearGraph();
        return false;  // ✅ 直接返回失败,不回退Legacy
    }
    
    // Step 4-5: 去重+选择
    vector<vector<Vector3d>> unique_paths = pruneEquivalentPaths(raw_paths);
    vector<vector<Vector3d>> selected_paths = selectShortPaths(unique_paths);
    
    // 转换为TopoPath格式
    for (size_t i = 0; i < selected_paths.size(); ++i) {
        double cost = calculatePathCost(selected_paths[i]);
        topo_paths.emplace_back(selected_paths[i], cost, i);
    }
    
    ROS_INFO("[TopoPRM] ✅ FastPlanner成功: %zu条路径", topo_paths.size());
    visualizeTopoPaths(topo_paths);
    clearGraph();
    
    return !topo_paths.empty();
}
```

**优点**:
- ✅ 纯FastPlanner算法,无混合逻辑
- ✅ 代码简洁,易维护

**缺点**:
- ❌ 23.5%场景规划失败
- ❌ 系统鲁棒性下降

---

### 3.2 方案B: 保守模式 - 优化参数降低超时率

**目标**: 通过优化FastPlanner参数,将超时率降到<5%

#### 优化1: 降低障碍物避让距离

```cpp
// planner/path_searching/src/topo_prm.cpp (Line 1255)

// 当前问题:
avoidance_radius = obstacle_radius + 4.50m;  // 10.6m总避让,太大!

// 优化方案:
double safety_factor = 0.3;  // 安全系数30%
avoidance_radius = obstacle_radius * (1.0 + safety_factor);  // 6.10 * 1.3 = 7.93m

// 进一步优化:
avoidance_radius = obstacle_radius + 1.5;  // 6.10 + 1.5 = 7.6m (-28%)
```

**预期效果**:
- 避让圈面积减少 **48%** (10.6m → 7.6m)
- 自由空间增加 **50-80%**
- DFS超时率: 23.5% → **<10%**

#### 优化2: 增加DFS超时时间

```cpp
// planner/path_searching/include/path_searching/topo_prm.h (Line ~120)

// 当前配置:
const double MAX_DFS_TIME_MS = 200.0;  // 200ms

// 优化方案:
const double MAX_DFS_TIME_MS = 400.0;  // 400ms (+100%)
```

**预期效果**:
- 给DFS更多时间探索
- 超时率: 10% → **<5%**

#### 优化3: 降低采样点数

```cpp
// planner/path_searching/src/topo_prm.cpp (Line 65-70)

// 当前配置:
vector<Vector3d> sample_points = sampleFreeSpaceInEllipsoid(start, goal, 100);
vector<Vector3d> boundary_points = sampleBoundaryLayer(start, goal, 35);
// 总采样: 600-670节点

// 优化方案:
vector<Vector3d> sample_points = sampleFreeSpaceInEllipsoid(start, goal, 60);  // 100→60 (-40%)
vector<Vector3d> boundary_points = sampleBoundaryLayer(start, goal, 20);       // 35→20 (-43%)
// 预期节点: 300-400个 (-50%)
```

**预期效果**:
- 图规模减少 **40-50%**
- DFS搜索空间减少 **10^6-10^10倍**
- 超时率: 5% → **<2%**

#### 优化4: 降低KNN连接数

```cpp
// planner/path_searching/src/topo_prm.cpp (Line 1096)

// 当前配置:
int K = 28;  // 平均度33-34

// 优化方案:
int K = 18;  // 平均度20-22 (-35%)
```

**预期效果**:
- 图边数减少 **40-50%**
- DFS搜索空间减少 **10^3-10^6倍**
- 超时率: 2% → **<1%**

#### 优化组合效果

| 优化项 | 当前值 | 优化值 | 效果 |
|-------|-------|-------|------|
| 避让距离 | 10.6m | 7.6m | -28% |
| DFS超时 | 200ms | 400ms | +100% |
| 采样点数 | 600-670 | 300-400 | -50% |
| KNN连接 | K=28 | K=18 | -35% |
| **图规模** | 600节点,11k边 | **350节点,4k边** | **-42%节点,-64%边** |
| **搜索空间** | 10^23 | **10^16** | **减少10^7倍** |
| **超时率** | 23.5% | **<1%** | **-96%** |

---

### 3.3 方案C: 渐进模式 - 先优化后移除

**阶段1**: 优化参数 (1-2天)
1. 实施方案B的4项优化
2. 测试验证超时率<5%
3. 保留Legacy作为安全网

**阶段2**: 移除Legacy (1天)
1. 确认超时率稳定<5%
2. 删除findTopoPathsLegacy()及相关代码
3. 删除3个Legacy回退点
4. 最终验证

---

## 四、推荐方案对比

| 方案 | 实现难度 | 超时率 | 鲁棒性 | 推荐度 |
|-----|---------|-------|-------|-------|
| **方案A: 激进移除** | ⭐ 简单 | 23.5% | ⚠️ 低 | ❌ 不推荐 |
| **方案B: 参数优化** | ⭐⭐ 简单 | <1% | ✅ 高 | 🏆 **强烈推荐** |
| **方案C: 渐进式** | ⭐⭐ 简单 | <5% | ✅ 高 | ✅ 推荐 |

---

## 五、最终推荐: 方案B (参数优化)

### 5.1 实施步骤

#### Step 1: 降低避让距离 (5分钟)

```cpp
// 文件: planner/path_searching/src/topo_prm.cpp
// 位置: Line 1255 (estimateObstacleSize函数后)

// 查找代码:
avoidance_radius = std::max(avoidance_radius, min_avoidance);

// 修改为:
double safety_distance = 1.5;  // 固定安全距离1.5m (原4.5m)
avoidance_radius = obstacle_radius + safety_distance;  // 6.10 + 1.5 = 7.6m
```

#### Step 2: 增加DFS超时 (2分钟)

```cpp
// 文件: planner/path_searching/include/path_searching/topo_prm.h
// 位置: Line ~120

// 查找代码:
const double MAX_DFS_TIME_MS = 200.0;

// 修改为:
const double MAX_DFS_TIME_MS = 400.0;  // 200ms → 400ms
```

#### Step 3: 降低采样点数 (2分钟)

```cpp
// 文件: planner/path_searching/src/topo_prm.cpp
// 位置: Line 65-70

// 查找代码:
vector<Vector3d> sample_points = sampleFreeSpaceInEllipsoid(start, goal, 100);
vector<Vector3d> boundary_points = sampleBoundaryLayer(start, goal, 35);

// 修改为:
vector<Vector3d> sample_points = sampleFreeSpaceInEllipsoid(start, goal, 60);
vector<Vector3d> boundary_points = sampleBoundaryLayer(start, goal, 20);
```

#### Step 4: 降低KNN连接数 (2分钟)

```cpp
// 文件: planner/path_searching/src/topo_prm.cpp
// 位置: Line 1096

// 查找代码:
int K = 28;

// 修改为:
int K = 18;  // 从28降到18
```

#### Step 5: 测试验证 (30分钟)

```bash
# 1. 编译
cd /home/developer/ros_ws/ddo-topo-mppi
catkin_make -DCMAKE_BUILD_TYPE=Release

# 2. 运行测试
roslaunch plan_manage topo_mppi_fastplanner_map.launch

# 3. 观察日志
# 关注:
# - [TopoPRM] 节点数 (应该<400)
# - [WARN] DFS超时 (应该<1次/17次重规划)
# - [TopoPRM] Legacy Generation (应该看不到)
```

---

### 5.2 预期效果

**优化前** (test.txt):
```
节点数: 600-670
边数: 9000-11000
平均度: 33-34
搜索空间: 10^23-10^30
DFS超时: 4/17 (23.5%)
Legacy触发: 4次 (100%依赖)
```

**优化后** (预期):
```
节点数: 300-400 (-50%)
边数: 3000-5000 (-55%)
平均度: 20-22 (-35%)
搜索空间: 10^13-10^16 (-10^7倍)
DFS超时: 0-1/17 (<5%)
Legacy触发: 0次 (不再需要)
```

---

### 5.3 移除Legacy的时机

**触发条件**: 连续3次测试,DFS超时率均<5%

**移除步骤**:

1. **删除Legacy回退逻辑** (3处):

```cpp
// planner/path_searching/src/topo_prm.cpp

// 回退点1 (Line 78):
if (sample_points.size() < 10) {
    ROS_WARN("[TopoPRM] 采样点太少，回退到Legacy方法");
    vector<TopoPath> legacy_paths = findTopoPathsLegacy(start, goal);  // ❌ 删除
    topo_paths = legacy_paths;
    visualizeTopoPaths(topo_paths);
    return !topo_paths.empty();
}
// 改为:
if (sample_points.size() < 10) {
    ROS_ERROR("[TopoPRM] 采样失败: 仅%zu个点", sample_points.size());
    return false;  // ✅ 直接返回失败
}

// 回退点2 (Line 90):
if (graph_nodes_.size() < 3) {
    ROS_WARN("[TopoPRM] 图节点太少，回退到Legacy方法");
    clearGraph();
    vector<TopoPath> legacy_paths = findTopoPathsLegacy(start, goal);  // ❌ 删除
    // ...
}
// 改为:
if (graph_nodes_.size() < 3) {
    ROS_ERROR("[TopoPRM] 图构建失败: 仅%zu个节点", graph_nodes_.size());
    clearGraph();
    return false;  // ✅ 直接返回失败
}

// 回退点3 (Line 104):
if (raw_paths.empty()) {
    ROS_WARN("[TopoPRM] 未找到路径，回退到Legacy方法");
    clearGraph();
    vector<TopoPath> legacy_paths = findTopoPathsLegacy(start, goal);  // ❌ 删除
    // ...
}
// 改为:
if (raw_paths.empty()) {
    ROS_ERROR("[TopoPRM] DFS搜索失败: 未找到路径");
    clearGraph();
    return false;  // ✅ 直接返回失败
}
```

2. **删除Legacy函数定义**:

```cpp
// 文件: planner/path_searching/src/topo_prm.cpp
// 删除 Line 152-380 (整个 #if 1 ... #endif 块)

#if 1  // ❌ 删除整个代码块
vector<TopoPath> TopoPRM::findTopoPathsLegacy(...) {
    // ...全部删除...
}
vector<Vector3d> TopoPRM::generateAlternativePath(...) { ... }
vector<Vector3d> TopoPRM::generateTangentPoints(...) { ... }
// ...其他Legacy函数...
#endif  // ❌ 删除结束
```

3. **删除Legacy函数声明**:

```cpp
// 文件: planner/path_searching/include/path_searching/topo_prm.h
// 删除 Line 100-115

// ❌ 删除以下声明:
std::vector<TopoPath> findTopoPathsLegacy(...);
std::vector<Eigen::Vector3d> generateAlternativePath(...);
std::vector<Eigen::Vector3d> generateTangentPoints(...);
// ...
```

4. **重新编译验证**:

```bash
cd /home/developer/ros_ws/ddo-topo-mppi
catkin_make -DCMAKE_BUILD_TYPE=Release
# 应该编译成功,无链接错误

# 运行测试
roslaunch plan_manage topo_mppi_fastplanner_map.launch
# 观察: 不应该再看到 "Legacy Generation" 日志
```

---

## 六、风险评估与应对

### 6.1 潜在风险

| 风险 | 概率 | 影响 | 应对措施 |
|-----|------|------|---------|
| DFS超时率仍>5% | 中 | 高 | 继续优化参数/增加超时时间 |
| 采样失败率增加 | 低 | 中 | 放宽采样点数限制(<10→<5) |
| 规划成功率下降 | 中 | 高 | 回滚到有Legacy的版本 |
| 性能退化 | 低 | 低 | 图规模减小,性能应该提升 |

### 6.2 回滚计划

**触发条件**: 任意一项指标不达标
- DFS超时率 >10%
- 规划成功率 <95%
- 平均路径数 <2.5

**回滚步骤**:
```bash
cd /home/he/ros_ws/test/ddo-topo-mppi/src/planner/path_searching
git checkout src/topo_prm.cpp  # 恢复到优化前的版本
git checkout include/path_searching/topo_prm.h
catkin_make
```

---

## 七、总结

### 7.1 核心改造思路

> **不是简单删除Legacy,而是通过优化FastPlanner参数使Legacy不再需要**

**关键优化**:
1. 降低避让距离: 10.6m → 7.6m (-28%)
2. 增加DFS超时: 200ms → 400ms (+100%)
3. 降低采样点数: 600-670 → 300-400 (-50%)
4. 降低KNN连接: K=28 → K=18 (-35%)

**预期结果**:
- 图规模减少 **50%**
- 搜索空间减少 **10^7倍**
- DFS超时率: 23.5% → **<1%**
- Legacy触发率: 23.5% → **0%**

### 7.2 实施时间线

- **Day 1 (上午)**: 实施4项参数优化 (15分钟代码修改 + 编译测试)
- **Day 1 (下午)**: 运行完整测试,收集数据 (3-5次完整运行)
- **Day 2**: 分析测试结果,如果超时率<5%则移除Legacy
- **Day 3**: 最终验证,更新文档

### 7.3 成功标准

✅ **核心目标**: 只使用FastPlanner PRM+DFS算法
✅ **性能目标**: DFS超时率<5%, 规划成功率>95%
✅ **代码目标**: 移除所有Legacy相关代码 (约230行)
✅ **文档目标**: 更新README,说明纯FastPlanner配置

---

**准备开始实施?** 我可以立即执行Step 1-4的代码修改! 🚀
