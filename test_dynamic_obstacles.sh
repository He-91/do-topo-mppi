#!/bin/bash
# 动态障碍物测试启动脚本

echo "========================================"
echo "🚀 启动动态障碍物测试环境"
echo "========================================"

# Source ROS环境
source /opt/ros/noetic/setup.bash
source ~/ros_ws/ddo-topo-mppi/devel/setup.bash

# 检查编译
echo "检查可执行文件..."
if [ ! -f ~/ros_ws/ddo-topo-mppi/devel/lib/map_generator/dynamic_obstacle_generator ]; then
    echo "❌ 动态障碍物生成器未编译！"
    exit 1
fi

if [ ! -f ~/ros_ws/ddo-topo-mppi/devel/lib/map_generator/cloud_merger ]; then
    echo "❌ 点云融合节点未编译！"
    exit 1
fi

echo "✅ 所有可执行文件已就绪"
echo ""
echo "启动方式："
echo "1. 完整测试（需要RViz显示）："
echo "   roslaunch ego_planner dynamic_obstacles_test.launch"
echo ""
echo "2. 仅启动动态障碍物生成器："
echo "   roslaunch ego_planner dynamic_obstacles_test.launch"
echo ""
echo "3. 手动测试各组件："
echo "   # Terminal 1: roscore"
echo "   # Terminal 2: rosrun map_generator dynamic_obstacle_generator"
echo "   # Terminal 3: rosrun map_generator cloud_merger"
echo "   # Terminal 4: rviz"
echo ""
echo "========================================"
echo "📊 话题列表："
echo "   静态障碍物: /map_generator/global_cloud"
echo "   动态障碍物: /dynamic_obstacles/cloud"
echo "   融合点云:   /pcl_render_node/cloud"
echo "   可视化标记: /dynamic_obstacles/markers"
echo "   速度向量:   /dynamic_obstacles/velocities"
echo "========================================"
