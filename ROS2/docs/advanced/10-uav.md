# 第10章：无人机开发

## 10.1 无人机系统架构

### 10.1.1 系统组成

<div align="center">
<svg width="650" height="320" viewBox="0 0 650 320" xmlns="http://www.w3.org/2000/svg">
  <defs>
    <marker id="arrowhead" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
      <polygon points="0 0, 10 3.5, 0 7" fill="#495057"/>
    </marker>
  </defs>

  <!-- 背景 -->
  <rect x="10" y="10" width="630" height="300" fill="#f8f9fa" stroke="#dee2e6" stroke-width="1" rx="4"/>
  <text x="325" y="38" font-family="system-ui, -apple-system, sans-serif" font-size="18" font-weight="600" text-anchor="middle" fill="#212529">无人机系统架构</text>

  <!-- 飞控层 -->
  <rect x="40" y="60" width="170" height="80" fill="#e3f2fd" stroke="#1976d2" stroke-width="2" rx="5"/>
  <text x="125" y="85" font-family="system-ui, -apple-system, sans-serif" font-size="15" font-weight="600" text-anchor="middle" fill="#1565c0">飞控层</text>
  <text x="125" y="105" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#424242">Flight Controller</text>
  <text x="125" y="125" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#757575">PX4 / ArduPilot</text>

  <!-- 计算层 -->
  <rect x="240" y="60" width="170" height="80" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="2" rx="5"/>
  <text x="325" y="85" font-family="system-ui, -apple-system, sans-serif" font-size="15" font-weight="600" text-anchor="middle" fill="#6a1b9a">计算层</text>
  <text x="325" y="105" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#424242">Compute Platform</text>
  <text x="325" y="125" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#757575">Companion Computer</text>

  <!-- 传感层 -->
  <rect x="440" y="60" width="170" height="80" fill="#fff3e0" stroke="#f57c00" stroke-width="2" rx="5"/>
  <text x="525" y="85" font-family="system-ui, -apple-system, sans-serif" font-size="15" font-weight="600" text-anchor="middle" fill="#ef6c00">传感层</text>
  <text x="525" y="105" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#424242">Sensor Suite</text>
  <text x="525" y="125" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#757575">Multi-sensor Fusion</text>

  <!-- 连接线 -->
  <line x1="210" y1="100" x2="240" y2="100" stroke="#495057" stroke-width="2" marker-end="url(#arrowhead)"/>
  <line x1="410" y1="100" x2="440" y2="100" stroke="#495057" stroke-width="2" marker-end="url(#arrowhead)"/>

  <!-- 飞控层详情 -->
  <rect x="40" y="160" width="170" height="130" fill="#ffffff" stroke="#1976d2" stroke-width="1.5" rx="4"/>
  <rect x="55" y="175" width="140" height="28" fill="#e3f2fd" stroke="#1976d2" stroke-width="1" rx="3"/>
  <text x="125" y="194" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#1565c0">姿态控制</text>

  <rect x="55" y="211" width="140" height="28" fill="#e3f2fd" stroke="#1976d2" stroke-width="1" rx="3"/>
  <text x="125" y="230" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#1565c0">传感器融合</text>

  <rect x="55" y="247" width="140" height="28" fill="#e3f2fd" stroke="#1976d2" stroke-width="1" rx="3"/>
  <text x="125" y="266" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#1565c0">电机控制</text>

  <!-- 计算层详情 -->
  <rect x="240" y="160" width="170" height="130" fill="#ffffff" stroke="#7b1fa2" stroke-width="1.5" rx="4"/>
  <rect x="255" y="175" width="140" height="28" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="1" rx="3"/>
  <text x="325" y="194" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#6a1b9a">ROS2 节点</text>

  <rect x="255" y="211" width="140" height="28" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="1" rx="3"/>
  <text x="325" y="230" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#6a1b9a">路径规划</text>

  <rect x="255" y="247" width="66" height="28" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="1" rx="3"/>
  <text x="288" y="266" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#6a1b9a">视觉处理</text>

  <rect x="329" y="247" width="66" height="28" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="1" rx="3"/>
  <text x="362" y="266" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#6a1b9a">任务管理</text>

  <!-- 传感层详情 -->
  <rect x="440" y="160" width="170" height="130" fill="#ffffff" stroke="#f57c00" stroke-width="1.5" rx="4"/>
  <rect x="455" y="175" width="66" height="28" fill="#fff3e0" stroke="#f57c00" stroke-width="1" rx="3"/>
  <text x="488" y="194" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#ef6c00">IMU</text>

  <rect x="529" y="175" width="66" height="28" fill="#fff3e0" stroke="#f57c00" stroke-width="1" rx="3"/>
  <text x="562" y="194" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#ef6c00">GPS</text>

  <rect x="455" y="211" width="66" height="28" fill="#fff3e0" stroke="#f57c00" stroke-width="1" rx="3"/>
  <text x="488" y="230" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#ef6c00">气压计</text>

  <rect x="529" y="211" width="66" height="28" fill="#fff3e0" stroke="#f57c00" stroke-width="1" rx="3"/>
  <text x="562" y="230" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#ef6c00">激光雷达</text>

  <rect x="455" y="247" width="140" height="28" fill="#fff3e0" stroke="#f57c00" stroke-width="1" rx="3"/>
  <text x="525" y="266" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#ef6c00">相机</text>

  <!-- 垂直连接线 -->
  <line x1="125" y1="140" x2="125" y2="160" stroke="#495057" stroke-width="1.5" stroke-dasharray="5,3"/>
  <line x1="325" y1="140" x2="325" y2="160" stroke="#495057" stroke-width="1.5" stroke-dasharray="5,3"/>
  <line x1="525" y1="140" x2="525" y2="160" stroke="#495057" stroke-width="1.5" stroke-dasharray="5,3"/>
</svg>
</div>

### 10.1.2 ROS2 话题架构

| 话题 | 消息类型 | 方向 | 说明 |
|------|----------|------|------|
| `/fmu/out/vehicle_odometry` | VehicleOdometry | 订阅 | 飞行器里程计 |
| `/fmu/in/trajectory_setpoint` | TrajectorySetpoint | 发布 | 轨迹设定点 |
| `/fmu/in/vehicle_command` | VehicleCommand | 发布 | 飞行器命令 |
| `/fmu/out/vehicle_status` | VehicleStatus | 订阅 | 飞行器状态 |
| `/fmu/in/offboard_control_mode` | OffboardControlMode | 发布 | 离板控制模式 |

## 10.2 PX4 飞控集成

### 10.2.1 PX4 简介

PX4 是一款专业的开源飞控软件，支持多种飞行器类型：

| 飞行器类型 | 说明 |
|------------|------|
| 多旋翼 | 四旋翼、六旋翼、八旋翼 |
| 固定翼 | 常规布局、飞翼 |
| 垂直起降 | 倾转旋翼、尾座式 |
| 飞艇 | 气球、飞艇 |

### 10.2.2 安装 PX4-Autopilot

```bash
# 安装依赖
sudo apt install git cmake ninja-build python3-pip

# 克隆 PX4
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# 安装 Python 依赖
pip3 install --user -r requirements.txt

# 编译 (以 Ubuntu 为例)
make px4_sitl_default gazebo
```

### 10.2.3 安装 PX4-ROS2 桥接

```bash
# 安装 px4_msgs
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git

# 安装 px4_ros_com
git clone https://github.com/PX4/px4_ros_com.git

# 编译
cd ~/ros2_ws
colcon build --packages-select px4_msgs px4_ros_com
source install/setup.bash
```

### 10.2.4 启动仿真

```bash
# 终端1：启动 PX4 SITL 和 Gazebo
cd ~/PX4-Autopilot
make px4_sitl_default gazebo

# 终端2：启动 ROS2 桥接
source ~/ros2_ws/install/setup.bash
ros2 launch px4_ros_com sensor_combined.launch.py
```

## 10.3 仿真环境配置

### 10.3.1 Gazebo 仿真

PX4 内置 Gazebo 仿真环境：

```bash
# 启动标准四旋翼
make px4_sitl_default gazebo

# 启动特定机型
make px4_sitl_default gazebo_iris

# 启动带光流的四旋翼
make px4_sitl_default gazebo_iris_opt_flow
```

### 10.3.2 Gazebo 世界文件

创建自定义世界文件 `worlds/my_world.world`：

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="my_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- 添加障碍物 -->
    <model name="obstacle_1">
      <pose>5 0 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>1 1 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1 1 2</size></box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## 10.4 飞行控制

### 10.4.1 基本飞行控制节点

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import (
    VehicleCommand,
    VehicleStatus,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleLocalPosition
)
import math

class BasicFlightController(Node):
    def __init__(self):
        super().__init__('basic_flight_controller')
        
        # QoS 配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 发布者
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        # 订阅者
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', 
            self.vehicle_status_callback, qos_profile)
        self.vehicle_local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.vehicle_local_pos_callback, qos_profile)
        
        # 状态变量
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.current_pos = [0.0, 0.0, 0.0]
        
        # 定时器
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.offboard_counter = 0
        
        self.get_logger().info('基本飞行控制器已启动')

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def vehicle_local_pos_callback(self, msg: VehicleLocalPosition):
        self.current_pos = [msg.x, msg.y, msg.z]

    def timer_callback(self):
        # 发布离板控制模式
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(offboard_msg)
        
        # 启动后进入离板模式
        if self.offboard_counter == 10:
            self.engage_offboard_mode()
            self.arm()
        
        # 发布轨迹设定点
        self.publish_position_setpoint(0.0, 0.0, -5.0)
        
        self.offboard_counter += 1

    def engage_offboard_mode(self):
        msg = VehicleCommand()
        msg.param1 = 1.0
        msg.param2 = 6.0
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)
        self.get_logger().info('切换到离板模式')

    def arm(self):
        msg = VehicleCommand()
        msg.param1 = 1.0
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)
        self.get_logger().info('发送解锁命令')

    def disarm(self):
        msg = VehicleCommand()
        msg.param1 = 0.0
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)
        self.get_logger().info('发送上锁命令')

    def publish_position_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = BasicFlightController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 10.4.2 速度控制

```python
def publish_velocity_setpoint(self, vx, vy, vz, yaw_rate=0.0):
    """发布速度设定点"""
    offboard_msg = OffboardControlMode()
    offboard_msg.position = False
    offboard_msg.velocity = True
    offboard_msg.acceleration = False
    offboard_msg.attitude = False
    offboard_msg.body_rate = False
    offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
    self.offboard_mode_pub.publish(offboard_msg)
    
    msg = TrajectorySetpoint()
    msg.velocity = [vx, vy, vz]
    msg.yawspeed = yaw_rate
    msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
    self.trajectory_pub.publish(msg)
```

## 10.5 任务规划

### 10.5.1 任务状态机

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from enum import Enum
import math

class MissionState(Enum):
    IDLE = 0
    TAKEOFF = 1
    WAYPOINT_1 = 2
    WAYPOINT_2 = 3
    WAYPOINT_3 = 4
    RETURN_HOME = 5
    LAND = 6
    COMPLETED = 7

class MissionPlanner(Node):
    def __init__(self):
        super().__init__('mission_planner')
        
        self.state = MissionState.IDLE
        self.waypoints = [
            {'x': 5.0, 'y': 0.0, 'z': -5.0},
            {'x': 5.0, 'y': 5.0, 'z': -5.0},
            {'x': 0.0, 'y': 5.0, 'z': -5.0},
            {'x': 0.0, 'y': 0.0, 'z': -5.0},
        ]
        self.current_waypoint_idx = 0
        
        # 位置阈值
        self.position_threshold = 0.5
        
        self.timer = self.create_timer(0.1, self.execute_mission)
        
        self.get_logger().info('任务规划器已启动')

    def execute_mission(self):
        if self.state == MissionState.IDLE:
            self.get_logger().info('等待启动任务')
            # 检查是否已解锁
            self.state = MissionState.TAKEOFF
            
        elif self.state == MissionState.TAKEOFF:
            self.get_logger().info('起飞中...')
            self.publish_position_setpoint(0.0, 0.0, -5.0)
            if self.is_at_position(0.0, 0.0, -5.0):
                self.state = MissionState.WAYPOINT_1
                
        elif self.state == MissionState.WAYPOINT_1:
            wp = self.waypoints[0]
            self.publish_position_setpoint(wp['x'], wp['y'], wp['z'])
            if self.is_at_position(wp['x'], wp['y'], wp['z']):
                self.state = MissionState.WAYPOINT_2
                
        elif self.state == MissionState.WAYPOINT_2:
            wp = self.waypoints[1]
            self.publish_position_setpoint(wp['x'], wp['y'], wp['z'])
            if self.is_at_position(wp['x'], wp['y'], wp['z']):
                self.state = MissionState.WAYPOINT_3
                
        elif self.state == MissionState.WAYPOINT_3:
            wp = self.waypoints[2]
            self.publish_position_setpoint(wp['x'], wp['y'], wp['z'])
            if self.is_at_position(wp['x'], wp['y'], wp['z']):
                self.state = MissionState.RETURN_HOME
                
        elif self.state == MissionState.RETURN_HOME:
            self.publish_position_setpoint(0.0, 0.0, -5.0)
            if self.is_at_position(0.0, 0.0, -5.0):
                self.state = MissionState.LAND
                
        elif self.state == MissionState.LAND:
            self.get_logger().info('降落中...')
            self.publish_position_setpoint(0.0, 0.0, 0.0)
            if self.is_at_position(0.0, 0.0, 0.0):
                self.state = MissionState.COMPLETED
                
        elif self.state == MissionState.COMPLETED:
            self.get_logger().info('任务完成')
            self.timer.cancel()

    def is_at_position(self, x, y, z):
        dx = self.current_pos[0] - x
        dy = self.current_pos[1] - y
        dz = self.current_pos[2] - z
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        return distance < self.position_threshold

    def publish_position_setpoint(self, x, y, z):
        pass

def main(args=None):
    rclpy.init(args=args)
    planner = MissionPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 10.5.2 圆形轨迹飞行

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

class CircularTrajectory(Node):
    def __init__(self):
        super().__init__('circular_trajectory')
        
        # 圆形参数
        self.declare_parameter('radius', 5.0)
        self.declare_parameter('altitude', -5.0)
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        
        self.angle = 0.0
        
        self.timer = self.create_timer(0.1, self.update_trajectory)
        
        self.get_logger().info('圆形轨迹生成器已启动')

    def update_trajectory(self):
        radius = self.get_parameter('radius').value
        altitude = self.get_parameter('altitude').value
        speed = self.get_parameter('speed').value
        center_x = self.get_parameter('center_x').value
        center_y = self.get_parameter('center_y').value
        
        # 计算当前位置
        x = center_x + radius * math.cos(self.angle)
        y = center_y + radius * math.sin(self.angle)
        z = altitude
        
        # 计算朝向（切线方向）
        yaw = self.angle + math.pi / 2
        
        # 发布位置设定点
        self.publish_position_setpoint(x, y, z, yaw)
        
        # 更新角度
        angular_speed = speed / radius
        self.angle += angular_speed * 0.1
        
        if self.angle > 2 * math.pi:
            self.angle -= 2 * math.pi

    def publish_position_setpoint(self, x, y, z, yaw):
        pass

def main(args=None):
    rclpy.init(args=args)
    trajectory = CircularTrajectory()
    rclpy.spin(trajectory)
    trajectory.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 10.6 传感器融合

### 10.6.1 IMU 数据处理

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
import math

class IMUProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')
        
        self.imu_sub = self.create_subscription(
            Imu, '/fmu/out/vehicle_imu', self.imu_callback, 10)
        
        # 低通滤波参数
        self.alpha = 0.1
        self.filtered_accel = [0.0, 0.0, 0.0]
        self.filtered_gyro = [0.0, 0.0, 0.0]
        
        self.get_logger().info('IMU处理器已启动')

    def imu_callback(self, msg: Imu):
        # 原始数据
        accel = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]
        gyro = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ]
        
        # 低通滤波
        self.filtered_accel = [
            self.alpha * accel[i] + (1 - self.alpha) * self.filtered_accel[i]
            for i in range(3)
        ]
        self.filtered_gyro = [
            self.alpha * gyro[i] + (1 - self.alpha) * self.filtered_gyro[i]
            for i in range(3)
        ]
        
        # 计算加速度幅值
        accel_magnitude = math.sqrt(
            self.filtered_accel[0]**2 + 
            self.filtered_accel[1]**2 + 
            self.filtered_accel[2]**2
        )
        
        self.get_logger().debug(
            f'加速度幅值: {accel_magnitude:.2f} m/s²'
        )
```

### 10.6.2 GPS 数据处理

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
import math

class GPSProcessor(Node):
    def __init__(self):
        super().__init__('gps_processor')
        
        self.gps_sub = self.create_subscription(
            NavSatFix, '/fmu/out/vehicle_gps_position', 
            self.gps_callback, 10)
        
        # 参考点（用于转换到局部坐标）
        self.ref_lat = None
        self.ref_lon = None
        
        self.get_logger().info('GPS处理器已启动')

    def gps_callback(self, msg: NavSatFix):
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        
        # 设置参考点
        if self.ref_lat is None:
            self.ref_lat = lat
            self.ref_lon = lon
        
        # 转换到局部坐标
        x, y = self.gps_to_local(lat, lon)
        
        self.get_logger().info(
            f'GPS: ({lat:.6f}, {lon:.6f}), 本地: ({x:.2f}, {y:.2f})'
        )

    def gps_to_local(self, lat, lon):
        """将GPS坐标转换为本地坐标"""
        # 地球半径（米）
        R = 6371000.0
        
        # 计算偏移
        dlat = math.radians(lat - self.ref_lat)
        dlon = math.radians(lon - self.ref_lon)
        
        # 简化的平面近似
        x = R * dlon * math.cos(math.radians(self.ref_lat))
        y = R * dlat
        
        return x, y
```

## 10.7 实践项目：自主飞行无人机

### 10.7.1 项目结构

```
uav_project/
├── uav_description/
│   └── models/
│       └── quadrotor/
├── uav_bringup/
│   └── launch/
│       ├── simulation.launch.py
│       └── flight.launch.py
├── uav_control/
│   └── src/
│       ├── offboard_control.py
│       ├── mission_planner.py
│       └── trajectory_generator.py
├── uav_perception/
│   └── src/
│       ├── camera_processor.py
│       └── obstacle_detector.py
└── uav_mission/
    └── src/
        ├── survey_mission.py
        └── follow_me.py
```

### 10.7.2 完整飞行控制器

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import (
    VehicleCommand,
    VehicleStatus,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleLocalPosition
)
from enum import Enum
import math

class FlightState(Enum):
    INIT = 0
    ARMING = 1
    TAKEOFF = 2
    MISSION = 3
    RETURN = 4
    LANDING = 5
    DISARMED = 6

class UAVController(Node):
    def __init__(self):
        super().__init__('uav_controller')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 发布者
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        
        # 订阅者
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.status_callback, qos)
        self.pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.pos_callback, qos)
        
        # 参数
        self.declare_parameter('takeoff_altitude', 5.0)
        self.declare_parameter('mission_timeout', 60.0)
        
        # 状态
        self.state = FlightState.INIT
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.position = [0.0, 0.0, 0.0]
        self.home_position = [0.0, 0.0, 0.0]
        
        # 任务航点
        self.mission_waypoints = [
            [5.0, 0.0, -5.0],
            [5.0, 5.0, -5.0],
            [0.0, 5.0, -5.0],
        ]
        self.current_wp_idx = 0
        
        # 定时器
        self.timer = self.create_timer(0.1, self.control_loop)
        self.offboard_counter = 0
        
        self.get_logger().info('无人机控制器已启动')

    def status_callback(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def pos_callback(self, msg: VehicleLocalPosition):
        self.position = [msg.x, msg.y, msg.z]

    def control_loop(self):
        # 发布离板控制模式
        self.publish_offboard_mode()
        
        # 状态机
        if self.state == FlightState.INIT:
            self.handle_init()
        elif self.state == FlightState.ARMING:
            self.handle_arming()
        elif self.state == FlightState.TAKEOFF:
            self.handle_takeoff()
        elif self.state == FlightState.MISSION:
            self.handle_mission()
        elif self.state == FlightState.RETURN:
            self.handle_return()
        elif self.state == FlightState.LANDING:
            self.handle_landing()
        
        self.offboard_counter += 1

    def handle_init(self):
        if self.offboard_counter > 20:
            self.state = FlightState.ARMING
            self.get_logger().info('开始解锁')

    def handle_arming(self):
        if self.offboard_counter % 10 == 0:
            self.arm()
            self.engage_offboard()
        
        if self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.home_position = self.position.copy()
            self.state = FlightState.TAKEOFF
            self.get_logger().info('已解锁，开始起飞')

    def handle_takeoff(self):
        takeoff_alt = self.get_parameter('takeoff_altitude').value
        self.publish_position(0.0, 0.0, -takeoff_alt)
        
        if self.is_at_position(0.0, 0.0, -takeoff_alt):
            self.state = FlightState.MISSION
            self.get_logger().info('起飞完成，开始任务')

    def handle_mission(self):
        if self.current_wp_idx >= len(self.mission_waypoints):
            self.state = FlightState.RETURN
            self.get_logger().info('任务完成，返航')
            return
        
        wp = self.mission_waypoints[self.current_wp_idx]
        self.publish_position(wp[0], wp[1], wp[2])
        
        if self.is_at_position(wp[0], wp[1], wp[2]):
            self.current_wp_idx += 1
            self.get_logger().info(f'到达航点 {self.current_wp_idx}')

    def handle_return(self):
        self.publish_position(
            self.home_position[0],
            self.home_position[1],
            -self.get_parameter('takeoff_altitude').value
        )
        
        if self.is_at_position(
            self.home_position[0],
            self.home_position[1],
            -self.get_parameter('takeoff_altitude').value
        ):
            self.state = FlightState.LANDING
            self.get_logger().info('到达起点，开始降落')

    def handle_landing(self):
        self.publish_position(
            self.home_position[0],
            self.home_position[1],
            0.0
        )
        
        if self.is_at_position(
            self.home_position[0],
            self.home_position[1],
            0.0,
            threshold=0.3
        ):
            self.disarm()
            self.state = FlightState.DISARMED
            self.get_logger().info('降落完成')
            self.timer.cancel()

    def is_at_position(self, x, y, z, threshold=0.5):
        dx = self.position[0] - x
        dy = self.position[1] - y
        dz = self.position[2] - z
        return math.sqrt(dx*dx + dy*dy + dz*dz) < threshold

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.mode_pub.publish(msg)

    def publish_position(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.traj_pub.publish(msg)

    def arm(self):
        msg = VehicleCommand()
        msg.param1 = 1.0
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def disarm(self):
        msg = VehicleCommand()
        msg.param1 = 0.0
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def engage_offboard(self):
        msg = VehicleCommand()
        msg.param1 = 1.0
        msg.param2 = 6.0
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = UAVController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 10.8 无人机开发最佳实践

### 10.8.1 安全考虑

1. **地理围栏**：设置飞行区域限制
2. **低电量返航**：自动返航机制
3. **失控保护**：信号丢失时的安全措施
4. **避障系统**：多传感器融合避障

### 10.8.2 调试技巧

```bash
# 查看 PX4 日志
cd ~/PX4-Autopilot/build/px4_sitl_default/rootfs/log

# 查看 MAVLink 消息
mavproxy.py --master=tcp:127.0.0.1:5760

# QGroundControl 连接
# 打开 QGroundControl，自动连接仿真
```

### 10.8.3 常见问题

| 问题 | 解决方案 |
|------|----------|
| 无法解锁 | 检查安全开关和预检状态 |
| 离板模式失败 | 确保持续发布设定点 |
| 定位漂移 | 检查GPS和EKF状态 |
| 飞行不稳定 | 调整PID参数 |

---


