/**
 * @file dynamic_obstacle_generator.cpp
 * @brief 动态障碍物生成器 - 支持多种运动模式
 * @date 2025-10-29
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Eigen>
#include <random>
#include <vector>

using namespace std;

// 动态障碍物运动模式
enum MotionType {
  LINEAR,           // 直线运动
  CIRCULAR,         // 圆周运动
  PENDULUM,         // 钟摆运动
  RANDOM_WALK,      // 随机游走
  STATIONARY        // 静止（用于对比测试）
};

// 动态障碍物结构
struct DynamicObstacle {
  int id;
  MotionType motion_type;
  Eigen::Vector3d position;      // 当前位置
  Eigen::Vector3d velocity;      // 当前速度
  Eigen::Vector3d start_pos;     // 起始位置（用于某些运动模式）
  double radius;                 // 障碍物半径
  double height;                 // 障碍物高度
  double speed;                  // 运动速度
  double time_offset;            // 时间偏移（用于相位差）
  
  // 圆周运动参数
  Eigen::Vector3d circle_center;
  double circle_radius;
  
  // 钟摆运动参数
  double pendulum_amplitude;
  Eigen::Vector3d pendulum_axis;  // 摆动轴方向
  
  // 随机游走参数
  Eigen::Vector3d random_target;
  double change_target_time;
};

class DynamicObstacleGenerator {
private:
  ros::NodeHandle nh_;
  ros::Publisher cloud_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher velocity_pub_;
  ros::Timer update_timer_;
  
  vector<DynamicObstacle> obstacles_;
  double resolution_;
  double update_rate_;
  double start_time_;
  
  // 地图边界
  double map_x_min_, map_x_max_;
  double map_y_min_, map_y_max_;
  double map_z_min_, map_z_max_;
  
  // 随机数生成器
  std::default_random_engine rng_;
  std::uniform_real_distribution<double> uniform_dist_;
  
public:
  DynamicObstacleGenerator(ros::NodeHandle& nh) : nh_(nh), uniform_dist_(-1.0, 1.0) {
    // 获取参数
    nh_.param("dynamic_obstacles/resolution", resolution_, 0.1);
    nh_.param("dynamic_obstacles/update_rate", update_rate_, 20.0);
    
    // 地图边界
    double map_x_size, map_y_size, map_z_size;
    nh_.param("map/x_size", map_x_size, 40.0);
    nh_.param("map/y_size", map_y_size, 20.0);
    nh_.param("map/z_size", map_z_size, 5.0);
    
    map_x_min_ = -map_x_size / 2.0;
    map_x_max_ = map_x_size / 2.0;
    map_y_min_ = -map_y_size / 2.0;
    map_y_max_ = map_y_size / 2.0;
    map_z_min_ = 0.0;
    map_z_max_ = map_z_size;
    
    // 发布器
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/dynamic_obstacles/cloud", 10);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/dynamic_obstacles/markers", 10);
    velocity_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/dynamic_obstacles/velocities", 10);
    
    // 初始化随机数生成器
    rng_.seed(std::random_device{}());
    
    // 生成动态障碍物
    generateObstacles();
    
    // 启动更新定时器
    start_time_ = ros::Time::now().toSec();
    update_timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), 
                                    &DynamicObstacleGenerator::updateCallback, this);
    
    ROS_INFO("========================================");
    ROS_INFO("🚀 动态障碍物生成器已启动");
    ROS_INFO("   障碍物数量: %zu", obstacles_.size());
    ROS_INFO("   更新频率: %.1f Hz", update_rate_);
    ROS_INFO("   分辨率: %.2f m", resolution_);
    ROS_INFO("========================================");
  }
  
  void generateObstacles() {
    int num_linear, num_circular, num_pendulum, num_random;
    nh_.param("dynamic_obstacles/num_linear", num_linear, 2);
    nh_.param("dynamic_obstacles/num_circular", num_circular, 2);
    nh_.param("dynamic_obstacles/num_pendulum", num_pendulum, 2);
    nh_.param("dynamic_obstacles/num_random", num_random, 1);
    
    int id = 0;
    
    // 1. 直线运动障碍物
    for (int i = 0; i < num_linear; i++) {
      DynamicObstacle obs;
      obs.id = id++;
      obs.motion_type = LINEAR;
      obs.radius = 0.5;
      obs.height = 2.5;
      obs.speed = 0.5 + 0.2 * i;  // 降低速度: 0.5-0.9 m/s
      obs.time_offset = i * 2.0;  // 时间偏移
      
      // 设置起点和运动方向
      if (i % 2 == 0) {
        // 水平方向
        obs.start_pos = Eigen::Vector3d(map_x_min_ + 5, 
                                        map_y_min_ + 3 + i * 4, 
                                        0.0);
        obs.velocity = Eigen::Vector3d(obs.speed, 0, 0);
      } else {
        // 垂直方向
        obs.start_pos = Eigen::Vector3d(5 + i * 3, 
                                        map_y_min_ + 3, 
                                        0.0);
        obs.velocity = Eigen::Vector3d(0, obs.speed, 0);
      }
      obs.position = obs.start_pos;
      obstacles_.push_back(obs);
    }
    
    // 2. 圆周运动障碍物
    for (int i = 0; i < num_circular; i++) {
      DynamicObstacle obs;
      obs.id = id++;
      obs.motion_type = CIRCULAR;
      obs.radius = 0.4;
      obs.height = 2.0;
      obs.speed = 0.2 + 0.1 * i;  // 降低角速度: 0.2-0.3 rad/s
      obs.time_offset = i * M_PI;  // 相位差
      
      // 圆心位置
      obs.circle_center = Eigen::Vector3d(0, -5 + i * 10, 0.0);
      obs.circle_radius = 3.0 + i * 1.0;
      obs.start_pos = obs.circle_center;
      obs.position = obs.circle_center + Eigen::Vector3d(obs.circle_radius, 0, 0);
      
      obstacles_.push_back(obs);
    }
    
    // 3. 钟摆运动障碍物
    for (int i = 0; i < num_pendulum; i++) {
      DynamicObstacle obs;
      obs.id = id++;
      obs.motion_type = PENDULUM;
      obs.radius = 0.45;
      obs.height = 2.2;
      obs.speed = 0.4 + 0.1 * i;  // 降低摆动频率: 0.4-0.5 Hz
      obs.time_offset = i * 1.5;
      
      obs.start_pos = Eigen::Vector3d(-10 + i * 5, 0, 0.0);
      obs.pendulum_amplitude = 4.0;
      obs.pendulum_axis = (i % 2 == 0) ? Eigen::Vector3d(1, 0, 0) : Eigen::Vector3d(0, 1, 0);
      obs.position = obs.start_pos;
      
      obstacles_.push_back(obs);
    }
    
    // 4. 随机游走障碍物
    for (int i = 0; i < num_random; i++) {
      DynamicObstacle obs;
      obs.id = id++;
      obs.motion_type = RANDOM_WALK;
      obs.radius = 0.5;
      obs.height = 2.3;
      obs.speed = 0.5;  // 降低速度: 0.5 m/s
      obs.time_offset = 0;
      
      obs.start_pos = Eigen::Vector3d(
        (map_x_min_ + map_x_max_) / 2.0,
        (map_y_min_ + map_y_max_) / 2.0,
        0.0
      );
      obs.position = obs.start_pos;
      obs.random_target = generateRandomTarget();
      obs.change_target_time = 0;
      
      obstacles_.push_back(obs);
    }
    
    ROS_INFO("生成动态障碍物:");
    ROS_INFO("  直线运动: %d", num_linear);
    ROS_INFO("  圆周运动: %d", num_circular);
    ROS_INFO("  钟摆运动: %d", num_pendulum);
    ROS_INFO("  随机游走: %d", num_random);
  }
  
  Eigen::Vector3d generateRandomTarget() {
    return Eigen::Vector3d(
      map_x_min_ + (map_x_max_ - map_x_min_) * (uniform_dist_(rng_) + 1.0) / 2.0,
      map_y_min_ + (map_y_max_ - map_y_min_) * (uniform_dist_(rng_) + 1.0) / 2.0,
      1.5
    );
  }
  
  void updateCallback(const ros::TimerEvent& event) {
    double current_time = ros::Time::now().toSec() - start_time_;
    
    // 更新每个障碍物的位置
    for (auto& obs : obstacles_) {
      updateObstaclePosition(obs, current_time);
    }
    
    // 发布点云
    publishPointCloud();
    
    // 发布可视化标记
    publishMarkers();
    
    // 发布速度向量
    publishVelocities();
  }
  
  void updateObstaclePosition(DynamicObstacle& obs, double t) {
    t += obs.time_offset;
    
    switch (obs.motion_type) {
      case LINEAR: {
        // 往返直线运动
        Eigen::Vector3d direction = obs.velocity.normalized();
        double distance = obs.speed * t;
        
        // 计算边界
        double max_dist;
        if (fabs(direction.x()) > 0.5) {
          max_dist = map_x_max_ - obs.start_pos.x() - obs.radius - 1.0;
        } else {
          max_dist = map_y_max_ - obs.start_pos.y() - obs.radius - 1.0;
        }
        
        // 往返运动
        double period = 2.0 * max_dist / obs.speed;
        double phase = fmod(distance, period);
        if (phase > max_dist) {
          phase = 2.0 * max_dist - phase;
        }
        
        obs.position = obs.start_pos + direction * phase;
        obs.velocity = direction * obs.speed * (phase < max_dist ? 1.0 : -1.0);
        break;
      }
      
      case CIRCULAR: {
        // 圆周运动
        double angle = obs.speed * t;
        obs.position = obs.circle_center + Eigen::Vector3d(
          obs.circle_radius * cos(angle),
          obs.circle_radius * sin(angle),
          0
        );
        obs.velocity = Eigen::Vector3d(
          -obs.circle_radius * obs.speed * sin(angle),
          obs.circle_radius * obs.speed * cos(angle),
          0
        );
        break;
      }
      
      case PENDULUM: {
        // 钟摆运动
        double offset = obs.pendulum_amplitude * sin(obs.speed * t);
        obs.position = obs.start_pos + obs.pendulum_axis * offset;
        obs.velocity = obs.pendulum_axis * (obs.pendulum_amplitude * obs.speed * cos(obs.speed * t));
        break;
      }
      
      case RANDOM_WALK: {
        // 随机游走
        Eigen::Vector3d direction = (obs.random_target - obs.position);
        double dist = direction.norm();
        
        if (dist < 0.5 || t > obs.change_target_time + 5.0) {
          obs.random_target = generateRandomTarget();
          obs.change_target_time = t;
        }
        
        if (dist > 0.1) {
          direction.normalize();
          obs.velocity = direction * obs.speed;
          obs.position += obs.velocity * (1.0 / update_rate_);
        }
        break;
      }
      
      case STATIONARY:
        obs.velocity.setZero();
        break;
    }
    
    // 边界检查
    obs.position.x() = std::max(map_x_min_ + obs.radius, std::min(map_x_max_ - obs.radius, obs.position.x()));
    obs.position.y() = std::max(map_y_min_ + obs.radius, std::min(map_y_max_ - obs.radius, obs.position.y()));
    // 保持z=0不变，让障碍物中心在地面，一半在地上一半在地下
    obs.position.z() = 0.0;
  }
  
  void publishPointCloud() {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    
    for (const auto& obs : obstacles_) {
      // 生成圆柱形障碍物点云
      int num_theta = ceil(2.0 * M_PI * obs.radius / resolution_);
      int num_height = ceil(obs.height / resolution_);
      
      for (int h = 0; h < num_height; h++) {
        double z = h * resolution_ - obs.height / 2.0;  // 从-height/2开始，中心在position.z
        for (int theta_idx = 0; theta_idx < num_theta; theta_idx++) {
          double theta = 2.0 * M_PI * theta_idx / num_theta;
          
          // 圆柱表面
          pcl::PointXYZ pt;
          pt.x = obs.position.x() + obs.radius * cos(theta);
          pt.y = obs.position.y() + obs.radius * sin(theta);
          pt.z = obs.position.z() + z;
          cloud.points.push_back(pt);
          
          // 填充内部（密集点云）
          int num_r = ceil(obs.radius / resolution_);
          for (int r_idx = 0; r_idx < num_r; r_idx++) {
            double r = r_idx * resolution_;
            pcl::PointXYZ pt_inner;
            pt_inner.x = obs.position.x() + r * cos(theta);
            pt_inner.y = obs.position.y() + r * sin(theta);
            pt_inner.z = obs.position.z() + z;
            cloud.points.push_back(pt_inner);
          }
        }
      }
    }
    
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = "world";
    cloud_msg.header.stamp = ros::Time::now();
    cloud_pub_.publish(cloud_msg);
  }
  
  void publishMarkers() {
    visualization_msgs::MarkerArray marker_array;
    
    for (const auto& obs : obstacles_) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time::now();
      marker.ns = "dynamic_obstacles";
      marker.id = obs.id;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;
      
      marker.pose.position.x = obs.position.x();
      marker.pose.position.y = obs.position.y();
      marker.pose.position.z = obs.position.z();  // 直接使用position.z，不再加height/2
      marker.pose.orientation.w = 1.0;
      
      marker.scale.x = obs.radius * 2.0;
      marker.scale.y = obs.radius * 2.0;
      marker.scale.z = obs.height;
      
      // 根据运动类型设置颜色
      switch (obs.motion_type) {
        case LINEAR:
          marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
          break;
        case CIRCULAR:
          marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
          break;
        case PENDULUM:
          marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0;
          break;
        case RANDOM_WALK:
          marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0;
          break;
        default:
          marker.color.r = 0.5; marker.color.g = 0.5; marker.color.b = 0.5;
      }
      marker.color.a = 0.7;
      marker.lifetime = ros::Duration(0.2);
      
      marker_array.markers.push_back(marker);
    }
    
    marker_pub_.publish(marker_array);
  }
  
  void publishVelocities() {
    visualization_msgs::MarkerArray vel_array;
    
    for (const auto& obs : obstacles_) {
      visualization_msgs::Marker arrow;
      arrow.header.frame_id = "world";
      arrow.header.stamp = ros::Time::now();
      arrow.ns = "obstacle_velocities";
      arrow.id = obs.id;
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::ADD;
      
      geometry_msgs::Point start, end;
      start.x = obs.position.x();
      start.y = obs.position.y();
      start.z = obs.position.z() + obs.height / 2.0;  // 箭头从障碍物顶部开始
      
      end.x = start.x + obs.velocity.x();
      end.y = start.y + obs.velocity.y();
      end.z = start.z + obs.velocity.z();
      
      arrow.points.push_back(start);
      arrow.points.push_back(end);
      
      arrow.scale.x = 0.1;  // 箭头轴直径
      arrow.scale.y = 0.2;  // 箭头头部直径
      
      arrow.color.r = 1.0;
      arrow.color.g = 1.0;
      arrow.color.g = 1.0;
      arrow.color.a = 1.0;
      arrow.lifetime = ros::Duration(0.2);
      
      vel_array.markers.push_back(arrow);
    }
    
    velocity_pub_.publish(vel_array);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynamic_obstacle_generator");
  ros::NodeHandle nh("~");
  
  DynamicObstacleGenerator generator(nh);
  
  ros::spin();
  return 0;
}
