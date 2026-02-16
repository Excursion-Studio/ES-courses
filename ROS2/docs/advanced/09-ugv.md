# 第9章：无人车开发

## 9.1 无人车系统架构

### 9.1.1 系统组成

<div align="center">
<svg width="650" height="320" viewBox="0 0 650 320" xmlns="http://www.w3.org/2000/svg">
  <defs>
    <marker id="arrowhead" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
      <polygon points="0 0, 10 3.5, 0 7" fill="#495057"/>
    </marker>
  </defs>
  
  <!-- 背景 -->
  <rect x="10" y="10" width="630" height="300" fill="#f8f9fa" stroke="#dee2e6" stroke-width="1" rx="4"/>
  <text x="325" y="38" font-family="system-ui, -apple-system, sans-serif" font-size="18" font-weight="600" text-anchor="middle" fill="#212529">无人车系统架构</text>
  
  <!-- 感知层 -->
  <rect x="40" y="60" width="170" height="80" fill="#e3f2fd" stroke="#1976d2" stroke-width="2" rx="5"/>
  <text x="125" y="85" font-family="system-ui, -apple-system, sans-serif" font-size="15" font-weight="600" text-anchor="middle" fill="#1565c0">感知层</text>
  <text x="125" y="105" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#424242">Perception</text>
  
  <!-- 决策层 -->
  <rect x="240" y="60" width="170" height="80" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="2" rx="5"/>
  <text x="325" y="85" font-family="system-ui, -apple-system, sans-serif" font-size="15" font-weight="600" text-anchor="middle" fill="#6a1b9a">决策层</text>
  <text x="325" y="105" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#424242">Decision</text>
  
  <!-- 执行层 -->
  <rect x="440" y="60" width="170" height="80" fill="#fff3e0" stroke="#f57c00" stroke-width="2" rx="5"/>
  <text x="525" y="85" font-family="system-ui, -apple-system, sans-serif" font-size="15" font-weight="600" text-anchor="middle" fill="#ef6c00">执行层</text>
  <text x="525" y="105" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#424242">Execution</text>
  
  <!-- 连接线 -->
  <line x1="210" y1="100" x2="240" y2="100" stroke="#495057" stroke-width="2" marker-end="url(#arrowhead)"/>
  <line x1="410" y1="100" x2="440" y2="100" stroke="#495057" stroke-width="2" marker-end="url(#arrowhead)"/>
  
  <!-- 感知层详情 -->
  <rect x="40" y="160" width="170" height="130" fill="#ffffff" stroke="#1976d2" stroke-width="1.5" rx="4"/>
  <rect x="55" y="175" width="140" height="28" fill="#e3f2fd" stroke="#1976d2" stroke-width="1" rx="3"/>
  <text x="125" y="194" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#1565c0">激光雷达</text>
  
  <rect x="55" y="211" width="140" height="28" fill="#e3f2fd" stroke="#1976d2" stroke-width="1" rx="3"/>
  <text x="125" y="230" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#1565c0">相机</text>
  
  <rect x="55" y="247" width="66" height="28" fill="#e3f2fd" stroke="#1976d2" stroke-width="1" rx="3"/>
  <text x="88" y="266" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#1565c0">IMU</text>
  
  <rect x="129" y="247" width="66" height="28" fill="#e3f2fd" stroke="#1976d2" stroke-width="1" rx="3"/>
  <text x="162" y="266" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#1565c0">GPS</text>
  
  <!-- 决策层详情 -->
  <rect x="240" y="160" width="170" height="130" fill="#ffffff" stroke="#7b1fa2" stroke-width="1.5" rx="4"/>
  <rect x="255" y="175" width="140" height="28" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="1" rx="3"/>
  <text x="325" y="194" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#6a1b9a">路径规划</text>
  
  <rect x="255" y="211" width="140" height="28" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="1" rx="3"/>
  <text x="325" y="230" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#6a1b9a">避障</text>
  
  <rect x="255" y="247" width="66" height="28" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="1" rx="3"/>
  <text x="288" y="266" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#6a1b9a">定位</text>
  
  <rect x="329" y="247" width="66" height="28" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="1" rx="3"/>
  <text x="362" y="266" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#6a1b9a">地图构建</text>
  
  <!-- 执行层详情 -->
  <rect x="440" y="160" width="170" height="130" fill="#ffffff" stroke="#f57c00" stroke-width="1.5" rx="4"/>
  <rect x="455" y="175" width="140" height="28" fill="#fff3e0" stroke="#f57c00" stroke-width="1" rx="3"/>
  <text x="525" y="194" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#ef6c00">电机控制</text>
  
  <rect x="455" y="211" width="140" height="28" fill="#fff3e0" stroke="#f57c00" stroke-width="1" rx="3"/>
  <text x="525" y="230" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#ef6c00">转向控制</text>
  
  <rect x="455" y="247" width="140" height="28" fill="#fff3e0" stroke="#f57c00" stroke-width="1" rx="3"/>
  <text x="525" y="266" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#ef6c00">制动系统</text>
  
  <!-- 垂直连接线 -->
  <line x1="125" y1="140" x2="125" y2="160" stroke="#495057" stroke-width="1.5" stroke-dasharray="5,3"/>
  <line x1="325" y1="140" x2="325" y2="160" stroke="#495057" stroke-width="1.5" stroke-dasharray="5,3"/>
  <line x1="525" y1="140" x2="525" y2="160" stroke="#495057" stroke-width="1.5" stroke-dasharray="5,3"/>
</svg>
</div>

### 9.1.2 ROS2 话题架构

| 话题 | 消息类型 | 方向 | 说明 |
|------|----------|------|------|
| `/cmd_vel` | geometry_msgs/Twist | 发布 | 速度指令 |
| `/odom` | nav_msgs/Odometry | 发布 | 里程计数据 |
| `/scan` | sensor_msgs/LaserScan | 发布 | 激光雷达数据 |
| `/map` | nav_msgs/OccupancyGrid | 发布 | 地图数据 |
| `/camera/image` | sensor_msgs/Image | 发布 | 相机图像 |
| `/imu/data` | sensor_msgs/Imu | 发布 | IMU数据 |
| `/tf` | tf2_msgs/TFMessage | 发布 | 坐标变换 |

## 9.2 导航2 (Nav2) 框架

### 9.2.1 Nav2 架构

<div align="center">
<svg width="650" height="380" viewBox="0 0 650 380" xmlns="http://www.w3.org/2000/svg">
  <defs>
    <marker id="arrowhead" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
      <polygon points="0 0, 10 3.5, 0 7" fill="#495057"/>
    </marker>
  </defs>
  
  <!-- 背景 -->
  <rect x="10" y="10" width="630" height="360" fill="#f8f9fa" stroke="#dee2e6" stroke-width="1" rx="4"/>
  <text x="325" y="38" font-family="system-ui, -apple-system, sans-serif" font-size="18" font-weight="600" text-anchor="middle" fill="#212529">Nav2 系统架构</text>
  
  <!-- Behavior Server -->
  <rect x="50" y="55" width="550" height="65" fill="#e3f2fd" stroke="#1976d2" stroke-width="2" rx="5"/>
  <text x="325" y="80" font-family="system-ui, -apple-system, sans-serif" font-size="15" font-weight="600" text-anchor="middle" fill="#1565c0">Behavior Server</text>
  <text x="325" y="100" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#424242">行为树: 决策逻辑、错误恢复、任务编排</text>
  
  <!-- 连接线 -->
  <line x1="325" y1="120" x2="325" y2="135" stroke="#495057" stroke-width="2" marker-end="url(#arrowhead)"/>
  
  <!-- Planner Server -->
  <rect x="50" y="140" width="550" height="65" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="2" rx="5"/>
  <text x="325" y="165" font-family="system-ui, -apple-system, sans-serif" font-size="15" font-weight="600" text-anchor="middle" fill="#6a1b9a">Planner Server</text>
  <text x="325" y="185" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#424242">路径规划: 全局路径、代价计算</text>
  
  <!-- 连接线 -->
  <line x1="325" y1="205" x2="325" y2="220" stroke="#495057" stroke-width="2" marker-end="url(#arrowhead)"/>
  
  <!-- Controller Server -->
  <rect x="50" y="225" width="550" height="65" fill="#fff3e0" stroke="#f57c00" stroke-width="2" rx="5"/>
  <text x="325" y="250" font-family="system-ui, -apple-system, sans-serif" font-size="15" font-weight="600" text-anchor="middle" fill="#ef6c00">Controller Server</text>
  <text x="325" y="270" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#424242">局部规划: 轨迹跟踪、避障控制</text>
  
  <!-- 连接线 -->
  <line x1="325" y1="290" x2="325" y2="305" stroke="#495057" stroke-width="2" marker-end="url(#arrowhead)"/>
  
  <!-- Costmap 2D -->
  <rect x="50" y="310" width="550" height="50" fill="#e8f5e9" stroke="#388e3c" stroke-width="2" rx="5"/>
  <text x="325" y="330" font-family="system-ui, -apple-system, sans-serif" font-size="15" font-weight="600" text-anchor="middle" fill="#2e7d32">Costmap 2D</text>
  <text x="325" y="348" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#424242">代价地图: 障碍物表示、环境感知</text>
</svg>
</div>

### 9.2.2 安装 Nav2

```bash
# 安装 Nav2 完整包
sudo apt install ros-humble-navigation2

# 安装 Nav2 Bringup
sudo apt install ros-humble-nav2-bringup
```

### 9.2.3 Nav2 配置文件

创建 `config/nav2_params.yaml`：

```yaml
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      max_vel_x: 0.26
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.26
      acc_lim_x: 2.5
      acc_lim_theta: 3.2

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.3
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

### 9.2.4 启动 Nav2

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'map': '/path/to/map.yaml',
            'params_file': '/path/to/nav2_params.yaml'
        }.items()
    )
    
    return LaunchDescription([nav2_launch])
```

## 9.3 SLAM 建图

### 9.3.1 SLAM 概述

SLAM (Simultaneous Localization and Mapping) 是同时定位与建图技术：

| SLAM 类型 | 特点 | 适用场景 |
|-----------|------|----------|
| Gmapping | 基于粒子滤波 | 2D激光雷达 |
| Cartographer | 图优化 | 高精度建图 |
| SLAM Toolbox | 在线/离线 | 实时建图 |
| RTAB-Map | 视觉SLAM | RGB-D相机 |

### 9.3.2 安装 SLAM Toolbox

```bash
sudo apt install ros-humble-slam-toolbox
```

### 9.3.3 SLAM Toolbox 配置

创建 `config/mapper_params_online_async.yaml`：

```yaml
slam_toolbox:
  ros__parameters:
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    mode: mapping
    
    resolution: 0.05
    max_laser_range: 20.0
    minimum_time_interval: 0.5
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1
    
    link_search_space_dimension: 0.5
    link_search_space_resolution: 0.01
    link_search_space_smear_deviation: 0.1
    
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45
```

### 9.3.4 启动建图

```bash
# 启动 SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py

# 手动控制移动建图
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 保存地图
ros2 run nav2_map_server map_saver_cli -f my_map
```

### 9.3.5 SLAM 节点示例

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class SLAMMonitor(Node):
    def __init__(self):
        super().__init__('slam_monitor')
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/pose', self.pose_callback, 10)
        
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        
        self.get_logger().info('SLAM监控节点已启动')

    def scan_callback(self, msg: LaserScan):
        valid_ranges = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max]
        if valid_ranges:
            self.get_logger().debug(
                f'激光数据: {len(valid_ranges)} 个有效点, '
                f'最近: {min(valid_ranges):.2f}m'
            )

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().debug(f'里程计: ({x:.2f}, {y:.2f})')

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(f'定位位置: ({x:.2f}, {y:.2f})')

    def map_callback(self, msg: OccupancyGrid):
        self.get_logger().info(
            f'地图更新: {msg.info.width}x{msg.info.height}, '
            f'分辨率: {msg.info.resolution}m'
        )

def main(args=None):
    rclpy.init(args=args)
    monitor = SLAMMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 9.4 路径规划与避障

### 9.4.1 路径规划类型

| 类型 | 说明 | 算法 |
|------|------|------|
| 全局规划 | 从起点到终点的整体路径 | A*, Dijkstra, RRT |
| 局部规划 | 实时避障和轨迹跟踪 | DWA, TEB, MPC |

### 9.4.2 导航客户端

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        
        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

    def navigate_to(self, x, y, yaw=0.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        
        # 设置朝向
        import math
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2)
        
        self._action_client.wait_for_server()
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('导航目标被拒绝')
            return
        
        self.get_logger().info('导航目标被接受')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('导航完成')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'导航中... 当前位置: ({feedback.current_pose.pose.position.x:.2f}, '
            f'{feedback.current_pose.pose.position.y:.2f})'
        )

def main(args=None):
    rclpy.init(args=args)
    client = NavigationClient()
    client.navigate_to(x=2.0, y=1.0, yaw=0.0)
    rclpy.spin(client)

if __name__ == '__main__':
    main()
```

### 9.4.3 简单避障控制器

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class SimpleObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('simple_obstacle_avoidance')
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('min_distance', 0.5)
        
        self.get_logger().info('简单避障控制器已启动')

    def scan_callback(self, msg: LaserScan):
        linear_speed = self.get_parameter('linear_speed').value
        angular_speed = self.get_parameter('angular_speed').value
        min_distance = self.get_parameter('min_distance').value
        
        # 分析前方障碍物
        front_ranges = msg.ranges[
            len(msg.ranges)//2 - 10 : len(msg.ranges)//2 + 10
        ]
        front_ranges = [r for r in front_ranges if r > msg.range_min]
        
        # 分析左右两侧
        left_ranges = msg.ranges[len(msg.ranges)//4 - 5 : len(msg.ranges)//4 + 5]
        right_ranges = msg.ranges[3*len(msg.ranges)//4 - 5 : 3*len(msg.ranges)//4 + 5]
        
        left_ranges = [r for r in left_ranges if r > msg.range_min]
        right_ranges = [r for r in right_ranges if r > msg.range_min]
        
        cmd = Twist()
        
        if front_ranges and min(front_ranges) < min_distance:
            # 前方有障碍物，转向
            left_min = min(left_ranges) if left_ranges else float('inf')
            right_min = min(right_ranges) if right_ranges else float('inf')
            
            if left_min > right_min:
                cmd.angular.z = angular_speed  # 左转
            else:
                cmd.angular.z = -angular_speed  # 右转
            
            self.get_logger().info(f'前方障碍物，转向避障')
        else:
            # 前方无障碍物，前进
            cmd.linear.x = linear_speed
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 9.5 定位与地图服务

### 9.5.1 AMCL 定位

AMCL (Adaptive Monte Carlo Localization) 是基于粒子滤波的定位算法。

```yaml
amcl:
  ros__parameters:
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan
```

### 9.5.2 初始位姿设置

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        
        self.timer = self.create_timer(1.0, self.publish_initial_pose)
        self.published = False

    def publish_initial_pose(self):
        if self.published:
            return
        
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.pose.position.x = 0.0
        pose.pose.pose.position.y = 0.0
        pose.pose.pose.orientation.w = 1.0
        
        # 设置协方差
        pose.pose.covariance = [0.0] * 36
        pose.pose.covariance[0] = 0.25   # x
        pose.pose.covariance[7] = 0.25   # y
        pose.pose.covariance[35] = 0.07  # yaw
        
        self.pose_pub.publish(pose)
        self.get_logger().info('发布初始位姿')
        self.published = True

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 9.6 实践项目：自主导航小车

### 9.6.1 项目结构

```
ugv_project/
├── ugv_description/
│   ├── urdf/
│   │   └── robot.urdf.xacro
│   ├── meshes/
│   └── launch/
│       └── display.launch.py
├── ugv_bringup/
│   ├── launch/
│   │   ├── robot.launch.py
│   │   └── navigation.launch.py
│   └── config/
│       ├── nav2_params.yaml
│       └── slam_params.yaml
├── ugv_control/
│   └── src/
│       ├── diff_drive_controller.py
│       └── obstacle_avoidance.py
└── ugv_navigation/
    └── src/
        ├── slam_node.py
        ├── navigation_client.py
        └── waypoint_follower.py
```

### 9.6.2 差速驱动控制器

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class DifferentialDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_controller')
        
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)
        
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 参数
        self.declare_parameter('wheel_separation', 0.4)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        
        # 状态
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        
        self.last_time = self.get_clock().now()
        
        # 定时更新
        self.timer = self.create_timer(0.02, self.update_odometry)
        
        self.get_logger().info('差速驱动控制器已启动')

    def cmd_callback(self, msg: Twist):
        self.vx = msg.linear.x
        self.vtheta = msg.angular.z

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # 更新位置
        self.x += self.vx * math.cos(self.theta) * dt
        self.y += self.vx * math.sin(self.theta) * dt
        self.theta += self.vtheta * dt
        
        # 发布里程计
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.get_parameter('odom_frame').value
        odom.child_frame_id = self.get_parameter('base_frame').value
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2)
        
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vtheta
        
        self.odom_pub.publish(odom)
        
        # 发布TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.get_parameter('odom_frame').value
        t.child_frame_id = self.get_parameter('base_frame').value
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.theta / 2)
        t.transform.rotation.w = math.cos(self.theta / 2)
        
        self.tf_broadcaster.sendTransform(t)
        
        # 发布轮子关节状态
        wheel_separation = self.get_parameter('wheel_separation').value
        wheel_radius = self.get_parameter('wheel_radius').value
        
        left_wheel_vel = (self.vx - self.vtheta * wheel_separation / 2) / wheel_radius
        right_wheel_vel = (self.vx + self.vtheta * wheel_separation / 2) / wheel_radius
        
        self.left_wheel_pos += left_wheel_vel * dt
        self.right_wheel_pos += right_wheel_vel * dt
        
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [self.left_wheel_pos, self.right_wheel_pos]
        joint_state.velocity = [left_wheel_vel, right_wheel_vel]
        
        self.joint_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    controller = DifferentialDriveController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 9.6.3 航点跟随器

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.waypoints = [
            {'x': 1.0, 'y': 0.0, 'yaw': 0.0},
            {'x': 1.0, 'y': 1.0, 'yaw': math.pi/2},
            {'x': 0.0, 'y': 1.0, 'yaw': math.pi},
            {'x': 0.0, 'y': 0.0, 'yaw': -math.pi/2},
        ]
        
        self.current_waypoint = 0
        self.is_navigating = False
        
        self.timer = self.create_timer(1.0, self.navigate_waypoints)
        
        self.get_logger().info(f'航点跟随器已启动，共 {len(self.waypoints)} 个航点')

    def navigate_waypoints(self):
        if self.is_navigating:
            return
        
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info('所有航点已完成')
            self.timer.cancel()
            return
        
        wp = self.waypoints[self.current_waypoint]
        self.navigate_to(wp['x'], wp['y'], wp['yaw'])
        self.is_navigating = True

    def navigate_to(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2)
        
        self.nav_client.wait_for_server()
        
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'航点 {self.current_waypoint + 1} 被拒绝')
            self.is_navigating = False
            return
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.current_waypoint += 1
        self.is_navigating = False
        
        if self.current_waypoint < len(self.waypoints):
            self.get_logger().info(
                f'航点 {self.current_waypoint} 完成，前往下一个航点'
            )

def main(args=None):
    rclpy.init(args=args)
    follower = WaypointFollower()
    rclpy.spin(follower)
    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 9.7 无人车开发最佳实践

### 9.7.1 安全考虑

1. **速度限制**：设置合理的最大速度
2. **急停功能**：实现紧急停止机制
3. **碰撞检测**：多传感器融合避障
4. **工作区域限制**：定义安全工作区域

### 9.7.2 性能优化

1. **代价地图更新频率**：根据场景调整
2. **规划频率**：平衡精度和计算量
3. **传感器融合**：提高感知精度

### 9.7.3 调试技巧

```bash
# 查看导航状态
ros2 topic echo /behavior_tree_log

# 查看代价地图
ros2 topic echo /local_costmap/costmap

# 查看规划路径
ros2 topic echo /plan

# 手动发布速度
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

---


