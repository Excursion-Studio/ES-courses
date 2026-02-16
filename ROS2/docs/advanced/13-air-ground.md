# 第13章：空地协同

## 13.1 空地协同系统概述

### 13.1.1 系统架构

<div align="center">
<svg width="650" height="380" viewBox="0 0 650 380" xmlns="http://www.w3.org/2000/svg">
  <defs>
    <marker id="arrowhead" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
      <polygon points="0 0, 10 3.5, 0 7" fill="#495057"/>
    </marker>
  </defs>

  <!-- 背景 -->
  <rect x="10" y="10" width="630" height="360" fill="#f8f9fa" stroke="#dee2e6" stroke-width="1" rx="4"/>
  <text x="325" y="38" font-family="system-ui, -apple-system, sans-serif" font-size="18" font-weight="600" text-anchor="middle" fill="#212529">空地协同系统架构</text>

  <!-- 协调中心 -->
  <rect x="255" y="55" width="140" height="60" fill="#37474f" stroke="#263238" stroke-width="2" rx="5"/>
  <text x="325" y="80" font-family="system-ui, -apple-system, sans-serif" font-size="15" font-weight="600" text-anchor="middle" fill="#ffffff">协调中心</text>
  <text x="325" y="100" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#b0bec5">Coordinator</text>

  <!-- 空中平台 UAV -->
  <rect x="50" y="145" width="140" height="80" fill="#e3f2fd" stroke="#1976d2" stroke-width="2" rx="5"/>
  <text x="120" y="170" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#1565c0">空中平台</text>
  <text x="120" y="190" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#424242">UAV</text>
  <text x="120" y="210" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#757575">无人机</text>

  <!-- 地面平台 UGV -->
  <rect x="255" y="145" width="140" height="80" fill="#e8f5e9" stroke="#388e3c" stroke-width="2" rx="5"/>
  <text x="325" y="170" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#2e7d32">地面平台</text>
  <text x="325" y="190" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#424242">UGV</text>
  <text x="325" y="210" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#757575">无人车</text>

  <!-- 指挥中心 -->
  <rect x="460" y="145" width="140" height="80" fill="#fff3e0" stroke="#f57c00" stroke-width="2" rx="5"/>
  <text x="530" y="170" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#ef6c00">指挥中心</text>
  <text x="530" y="190" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#424242">Command</text>
  <text x="530" y="210" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#757575">远程监控</text>

  <!-- 连接线 协调中心到各平台 -->
  <line x1="325" y1="115" x2="120" y2="145" stroke="#495057" stroke-width="1.5" stroke-dasharray="5,3"/>
  <line x1="325" y1="115" x2="325" y2="145" stroke="#495057" stroke-width="1.5" stroke-dasharray="5,3"/>
  <line x1="325" y1="115" x2="530" y2="145" stroke="#495057" stroke-width="1.5" stroke-dasharray="5,3"/>

  <!-- 平台间通信 -->
  <line x1="190" y1="185" x2="255" y2="185" stroke="#1976d2" stroke-width="2" marker-end="url(#arrowhead)"/>
  <line x1="255" y1="200" x2="190" y2="200" stroke="#388e3c" stroke-width="2" marker-end="url(#arrowhead)"/>

  <!-- 通信层 -->
  <rect x="50" y="255" width="550" height="90" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="2" rx="5"/>
  <text x="325" y="280" font-family="system-ui, -apple-system, sans-serif" font-size="15" font-weight="600" text-anchor="middle" fill="#6a1b9a">通信层</text>

  <!-- 通信方式 -->
  <rect x="70" y="295" width="100" height="40" fill="#ffffff" stroke="#7b1fa2" stroke-width="1" rx="3"/>
  <text x="120" y="320" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#6a1b9a">WiFi</text>

  <rect x="190" y="295" width="100" height="40" fill="#ffffff" stroke="#7b1fa2" stroke-width="1" rx="3"/>
  <text x="240" y="320" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#6a1b9a">4G / 5G</text>

  <rect x="310" y="295" width="100" height="40" fill="#ffffff" stroke="#7b1fa2" stroke-width="1" rx="3"/>
  <text x="360" y="320" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#6a1b9a">专网通信</text>

  <rect x="430" y="295" width="100" height="40" fill="#ffffff" stroke="#7b1fa2" stroke-width="1" rx="3"/>
  <text x="480" y="320" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#6a1b9a">卫星链路</text>

  <!-- 垂直连接线 -->
  <line x1="120" y1="225" x2="120" y2="255" stroke="#495057" stroke-width="1.5" stroke-dasharray="5,3"/>
  <line x1="325" y1="225" x2="325" y2="255" stroke="#495057" stroke-width="1.5" stroke-dasharray="5,3"/>
  <line x1="530" y1="225" x2="530" y2="255" stroke="#495057" stroke-width="1.5" stroke-dasharray="5,3"/>
</svg>
</div>

### 13.1.2 协同模式

| 模式 | 描述 | 典型应用 |
|------|------|----------|
| 空中侦察 | UAV侦察，UGV执行 | 目标搜索、危险区域探测 |
| 地面引导 | UGV引导，UAV跟随 | 物资运输、编队行进 |
| 协同作业 | 空地同时作业 | 灾害救援、农业植保 |
| 中继通信 | UAV作为通信中继 | 大范围通信覆盖 |

### 13.1.3 ROS2 话题架构

<div align="center">
<svg width="650" height="400" viewBox="0 0 650 400" xmlns="http://www.w3.org/2000/svg">
  <defs>
    <marker id="arrowhead" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
      <polygon points="0 0, 10 3.5, 0 7" fill="#495057"/>
    </marker>
  </defs>

  <!-- 背景 -->
  <rect x="10" y="10" width="630" height="380" fill="#f8f9fa" stroke="#dee2e6" stroke-width="1" rx="4"/>
  <text x="325" y="38" font-family="system-ui, -apple-system, sans-serif" font-size="18" font-weight="600" text-anchor="middle" fill="#212529">空地协同话题架构</text>

  <!-- UAV 话题空间 -->
  <rect x="40" y="55" width="180" height="140" fill="#e3f2fd" stroke="#1976d2" stroke-width="2" rx="5"/>
  <text x="130" y="80" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#1565c0">/uav/</text>
  
  <rect x="55" y="95" width="150" height="22" fill="#ffffff" stroke="#1976d2" stroke-width="1" rx="3"/>
  <text x="130" y="110" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#424242">odom</text>
  
  <rect x="55" y="122" width="150" height="22" fill="#ffffff" stroke="#1976d2" stroke-width="1" rx="3"/>
  <text x="130" y="137" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#424242">cmd_vel</text>
  
  <rect x="55" y="149" width="150" height="22" fill="#ffffff" stroke="#1976d2" stroke-width="1" rx="3"/>
  <text x="130" y="164" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#424242">scan (下视激光)</text>
  
  <rect x="55" y="176" width="72" height="22" fill="#ffffff" stroke="#1976d2" stroke-width="1" rx="3"/>
  <text x="91" y="191" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#424242">battery</text>
  
  <rect x="133" y="176" width="72" height="22" fill="#ffffff" stroke="#1976d2" stroke-width="1" rx="3"/>
  <text x="169" y="191" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#424242">mission_state</text>

  <!-- UGV 话题空间 -->
  <rect x="430" y="55" width="180" height="140" fill="#e8f5e9" stroke="#388e3c" stroke-width="2" rx="5"/>
  <text x="520" y="80" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#2e7d32">/ugv/</text>
  
  <rect x="445" y="95" width="150" height="22" fill="#ffffff" stroke="#388e3c" stroke-width="1" rx="3"/>
  <text x="520" y="110" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#424242">odom</text>
  
  <rect x="445" y="122" width="150" height="22" fill="#ffffff" stroke="#388e3c" stroke-width="1" rx="3"/>
  <text x="520" y="137" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#424242">cmd_vel</text>
  
  <rect x="445" y="149" width="150" height="22" fill="#ffffff" stroke="#388e3c" stroke-width="1" rx="3"/>
  <text x="520" y="164" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#424242">scan (水平激光)</text>
  
  <rect x="445" y="176" width="72" height="22" fill="#ffffff" stroke="#388e3c" stroke-width="1" rx="3"/>
  <text x="481" y="191" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#424242">battery</text>
  
  <rect x="523" y="176" width="72" height="22" fill="#ffffff" stroke="#388e3c" stroke-width="1" rx="3"/>
  <text x="559" y="191" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#424242">mission_state</text>

  <!-- Coordinator 话题空间 -->
  <rect x="40" y="215" width="180" height="130" fill="#fff3e0" stroke="#f57c00" stroke-width="2" rx="5"/>
  <text x="130" y="240" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#ef6c00">/coordinator/</text>
  
  <rect x="55" y="255" width="150" height="22" fill="#ffffff" stroke="#f57c00" stroke-width="1" rx="3"/>
  <text x="130" y="270" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#424242">task_allocation</text>
  
  <rect x="55" y="282" width="150" height="22" fill="#ffffff" stroke="#f57c00" stroke-width="1" rx="3"/>
  <text x="130" y="297" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#424242">formation_command</text>
  
  <rect x="55" y="309" width="72" height="22" fill="#ffffff" stroke="#f57c00" stroke-width="1" rx="3"/>
  <text x="91" y="324" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#424242">relay_status</text>
  
  <rect x="133" y="309" width="72" height="22" fill="#ffffff" stroke="#f57c00" stroke-width="1" rx="3"/>
  <text x="169" y="324" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#424242">conflict_resolution</text>

  <!-- Shared 话题空间 -->
  <rect x="430" y="215" width="180" height="130" fill="#fce4ec" stroke="#c2185b" stroke-width="2" rx="5"/>
  <text x="520" y="240" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#ad1457">/shared/</text>
  
  <rect x="445" y="255" width="150" height="22" fill="#ffffff" stroke="#c2185b" stroke-width="1" rx="3"/>
  <text x="520" y="270" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#424242">global_map</text>
  
  <rect x="445" y="282" width="150" height="22" fill="#ffffff" stroke="#c2185b" stroke-width="1" rx="3"/>
  <text x="520" y="297" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#424242">target_position</text>
  
  <rect x="445" y="309" width="150" height="22" fill="#ffffff" stroke="#c2185b" stroke-width="1" rx="3"/>
  <text x="520" y="324" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#424242">mission_status</text>

  <!-- 中间连接说明 -->
  <rect x="235" y="155" width="180" height="40" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="1.5" rx="4"/>
  <text x="325" y="180" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#6a1b9a">DDS 通信总线</text>
</svg>
</div>

## 13.2 协调中心设计

### 13.2.1 协调器节点

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import json
import math

class AirGroundCoordinator(Node):
    """
    空地协同协调器
    """
    def __init__(self):
        super().__init__('air_ground_coordinator')
        
        # 平台状态
        self.uav_state = {
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'velocity': {'vx': 0.0, 'vy': 0.0, 'vz': 0.0},
            'battery': 100.0,
            'status': 'idle',
            'mission': None
        }
        
        self.ugv_state = {
            'position': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            'velocity': {'vx': 0.0, 'vy': 0.0},
            'battery': 100.0,
            'status': 'idle',
            'mission': None
        }
        
        # 目标信息
        self.targets = []
        self.current_target = None
        
        # 订阅 UAV 状态
        self.uav_odom_sub = self.create_subscription(
            Odometry, '/uav/odom',
            self.uav_odom_callback, 10)
        
        self.uav_battery_sub = self.create_subscription(
            String, '/uav/battery',
            self.uav_battery_callback, 10)
        
        # 订阅 UGV 状态
        self.ugv_odom_sub = self.create_subscription(
            Odometry, '/ugv/odom',
            self.ugv_odom_callback, 10)
        
        self.ugv_battery_sub = self.create_subscription(
            String, '/ugv/battery',
            self.ugv_battery_callback, 10)
        
        # 发布命令
        self.uav_cmd_pub = self.create_publisher(
            PoseStamped, '/uav/target_pose', 10)
        self.ugv_cmd_pub = self.create_publisher(
            PoseStamped, '/ugv/target_pose', 10)
        
        # 协同状态发布
        self.status_pub = self.create_publisher(
            String, '/coordinator/status', 10)
        
        # 定时器
        self.timer = self.create_timer(0.5, self.coordinate)
        
        self.get_logger().info('空地协同协调器已启动')

    def uav_odom_callback(self, msg: Odometry):
        """更新 UAV 状态"""
        self.uav_state['position']['x'] = msg.pose.pose.position.x
        self.uav_state['position']['y'] = msg.pose.pose.position.y
        self.uav_state['position']['z'] = msg.pose.pose.position.z
        self.uav_state['velocity']['vx'] = msg.twist.twist.linear.x
        self.uav_state['velocity']['vy'] = msg.twist.twist.linear.y
        self.uav_state['velocity']['vz'] = msg.twist.twist.linear.z

    def uav_battery_callback(self, msg: String):
        """更新 UAV 电量"""
        self.uav_state['battery'] = float(msg.data)

    def ugv_odom_callback(self, msg: Odometry):
        """更新 UGV 状态"""
        self.ugv_state['position']['x'] = msg.pose.pose.position.x
        self.ugv_state['position']['y'] = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        self.ugv_state['position']['yaw'] = math.atan2(
            2.0 * (q.w * q.z), 1.0 - 2.0 * q.z * q.z)

    def ugv_battery_callback(self, msg: String):
        """更新 UGV 电量"""
        self.ugv_state['battery'] = float(msg.data)

    def coordinate(self):
        """协同控制主循环"""
        # 检查平台状态
        self.check_platform_status()
        
        # 执行协同任务
        if self.current_target:
            self.execute_coordinated_mission()
        
        # 发布状态
        self.publish_status()

    def check_platform_status(self):
        """检查平台状态"""
        # 检查电量
        if self.uav_state['battery'] < 20:
            self.get_logger().warn('UAV 电量低，准备返航')
            self.recall_uav()
        
        if self.ugv_state['battery'] < 15:
            self.get_logger().warn('UGV 电量低，准备返航')
            self.recall_ugv()

    def execute_coordinated_mission(self):
        """执行协同任务"""
        target = self.current_target
        
        # 计算距离
        uav_dist = self.calculate_distance(
            self.uav_state['position'], target)
        ugv_dist = self.calculate_distance_2d(
            self.ugv_state['position'], target)
        
        # 根据任务类型分配
        if target['type'] == 'reconnaissance':
            # 侦察任务：UAV优先
            if uav_dist < 5.0:
                self.uav_state['status'] = 'executing'
            else:
                self.send_uav_to_target(target)
        
        elif target['type'] == 'delivery':
            # 运输任务：UGV执行，UAV护航
            self.send_ugv_to_target(target)
            self.uav_follow_ugv()
        
        elif target['type'] == 'search_rescue':
            # 搜救任务：空地协同
            self.coordinate_search_rescue(target)

    def coordinate_search_rescue(self, target):
        """协同搜救"""
        # UAV 进行大范围搜索
        search_pattern = self.generate_search_pattern(
            target['center'], target['radius'])
        
        # 分配搜索区域
        uav_area = search_pattern[:len(search_pattern)//2]
        ugv_area = search_pattern[len(search_pattern)//2:]
        
        # 发送搜索任务
        if self.uav_state['status'] == 'idle':
            self.send_uav_search_mission(uav_area)
        
        if self.ugv_state['status'] == 'idle':
            self.send_ugv_search_mission(ugv_area)

    def generate_search_pattern(self, center, radius):
        """生成搜索模式"""
        waypoints = []
        num_spirals = 5
        points_per_spiral = 20
        
        for i in range(num_spirals * points_per_spiral):
            t = i / points_per_spiral
            r = radius * t / num_spirals
            theta = 2 * math.pi * t
            
            x = center['x'] + r * math.cos(theta)
            y = center['y'] + r * math.sin(theta)
            waypoints.append({'x': x, 'y': y})
        
        return waypoints

    def send_uav_to_target(self, target):
        """发送 UAV 到目标"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = target['x']
        pose.pose.position.y = target['y']
        pose.pose.position.z = target.get('z', 5.0)
        
        self.uav_cmd_pub.publish(pose)
        self.uav_state['status'] = 'moving'

    def send_ugv_to_target(self, target):
        """发送 UGV 到目标"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = target['x']
        pose.pose.position.y = target['y']
        
        self.ugv_cmd_pub.publish(pose)
        self.ugv_state['status'] = 'moving'

    def uav_follow_ugv(self):
        """UAV 跟随 UGV"""
        ugv_pos = self.ugv_state['position']
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = ugv_pos['x']
        pose.pose.position.y = ugv_pos['y']
        pose.pose.position.z = 10.0  # 保持高度
        
        self.uav_cmd_pub.publish(pose)

    def recall_uav(self):
        """召回 UAV"""
        home = {'x': 0.0, 'y': 0.0, 'z': 2.0}
        self.send_uav_to_target(home)

    def recall_ugv(self):
        """召回 UGV"""
        home = {'x': 0.0, 'y': 0.0}
        self.send_ugv_to_target(home)

    def calculate_distance(self, pos1, pos2):
        """计算3D距离"""
        dx = pos1['x'] - pos2['x']
        dy = pos1['y'] - pos2['y']
        dz = pos1.get('z', 0) - pos2.get('z', 0)
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def calculate_distance_2d(self, pos1, pos2):
        """计算2D距离"""
        dx = pos1['x'] - pos2['x']
        dy = pos1['y'] - pos2['y']
        return math.sqrt(dx*dx + dy*dy)

    def publish_status(self):
        """发布协同状态"""
        status = {
            'uav': self.uav_state,
            'ugv': self.ugv_state,
            'current_target': self.current_target,
            'timestamp': self.get_clock().now().nanoseconds
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def add_target(self, target):
        """添加目标"""
        self.targets.append(target)
        self.get_logger().info(f"添加目标: {target['id']}")


def main(args=None):
    rclpy.init(args=args)
    coordinator = AirGroundCoordinator()
    
    # 添加示例目标
    coordinator.add_target({
        'id': 'target_1',
        'type': 'reconnaissance',
        'x': 50.0,
        'y': 30.0,
        'z': 5.0
    })
    
    rclpy.spin(coordinator)
    coordinator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 13.3 空中侦察与地面执行

### 13.3.1 UAV 侦察节点

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker, MarkerArray
import cv2
import numpy as np
import math

class UAVReconnaissance(Node):
    """
    UAV 侦察节点 - 目标检测与跟踪
    """
    def __init__(self):
        super().__init__('uav_reconnaissance')
        
        self.bridge = CvBridge()
        
        # 相机参数
        self.declare_parameter('camera_fov', 1.047)  # 60度
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        
        # 检测目标
        self.detected_targets = []
        self.tracking_target = None
        
        # 订阅
        self.image_sub = self.create_subscription(
            Image, '/uav/camera/image_raw',
            self.image_callback, 10)
        
        self.altitude_sub = self.create_subscription(
            LaserScan, '/uav/scan_down',
            self.altitude_callback, 10)
        
        # 发布
        self.target_pub = self.create_publisher(
            PoseStamped, '/uav/detected_target', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/uav/target_markers', 10)
        
        # 当前高度
        self.altitude = 5.0
        
        self.get_logger().info('UAV 侦察节点已启动')

    def image_callback(self, msg: Image):
        """处理图像"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')
            return
        
        # 目标检测
        targets = self.detect_targets(cv_image)
        
        # 发布检测结果
        for target in targets:
            self.publish_target(target, msg.header)

    def altitude_callback(self, msg: LaserScan):
        """更新高度"""
        if msg.ranges:
            valid_ranges = [r for r in msg.ranges if r > 0.1]
            if valid_ranges:
                self.altitude = np.mean(valid_ranges)

    def detect_targets(self, image):
        """检测目标"""
        targets = []
        
        # 转换到 HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 检测红色目标（示例）
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # 形态学处理
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # 最小面积阈值
                # 计算中心
                M = cv2.moments(contour)
                if M['m00'] > 0:
                    cx = M['m10'] / M['m00']
                    cy = M['m01'] / M['m00']
                    
                    # 转换到世界坐标
                    world_pos = self.pixel_to_world(cx, cy)
                    
                    targets.append({
                        'pixel_x': cx,
                        'pixel_y': cy,
                        'world_x': world_pos[0],
                        'world_y': world_pos[1],
                        'area': area
                    })
        
        return targets

    def pixel_to_world(self, px, py):
        """像素坐标转世界坐标"""
        fov = self.get_parameter('camera_fov').value
        width = self.get_parameter('image_width').value
        height = self.get_parameter('image_height').value
        
        # 相机角度
        angle_x = (px - width/2) / width * fov
        angle_y = (py - height/2) / height * fov * (height/width)
        
        # 假设相机朝下
        world_x = self.altitude * math.tan(angle_x)
        world_y = self.altitude * math.tan(angle_y)
        
        return (world_x, world_y)

    def publish_target(self, target, header):
        """发布目标位置"""
        pose = PoseStamped()
        pose.header = header
        pose.header.frame_id = 'uav_base_link'
        pose.pose.position.x = target['world_x']
        pose.pose.position.y = target['world_y']
        pose.pose.position.z = 0.0
        
        self.target_pub.publish(pose)
        
        # 发布可视化标记
        marker = Marker()
        marker.header = pose.header
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = target['world_x']
        marker.pose.position.y = target['world_y']
        marker.pose.position.z = 0.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    reconnaissance = UAVReconnaissance()
    rclpy.spin(reconnaissance)
    reconnaissance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 13.3.2 UGV 执行节点

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
import json

class UGVExecutor(Node):
    """
    UGV 执行节点 - 接收 UAV 侦察结果并执行任务
    """
    def __init__(self):
        super().__init__('ugv_executor')
        
        # 任务队列
        self.task_queue = []
        self.current_task = None
        
        # 订阅目标
        self.target_sub = self.create_subscription(
            PoseStamped, '/uav/detected_target',
            self.target_callback, 10)
        
        # 订阅任务
        self.task_sub = self.create_subscription(
            String, '/ugv/task',
            self.task_callback, 10)
        
        # 导航客户端
        self.nav_client = ActionClient(
            self, NavigateToPose, '/ugv/navigate_to_pose')
        
        # 速度发布
        self.cmd_pub = self.create_publisher(
            Twist, '/ugv/cmd_vel', 10)
        
        # 状态发布
        self.status_pub = self.create_publisher(
            String, '/ugv/executor_status', 10)
        
        self.get_logger().info('UGV 执行节点已启动')

    def target_callback(self, msg: PoseStamped):
        """接收目标"""
        task = {
            'type': 'approach',
            'target': {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y
            },
            'priority': 1
        }
        
        self.task_queue.append(task)
        self.task_queue.sort(key=lambda t: t['priority'], reverse=True)
        
        self.get_logger().info(
            f"收到目标: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")
        
        # 如果空闲，开始执行
        if self.current_task is None:
            self.execute_next_task()

    def task_callback(self, msg: String):
        """接收任务"""
        task = json.loads(msg.data)
        self.task_queue.append(task)

    def execute_next_task(self):
        """执行下一个任务"""
        if not self.task_queue:
            return
        
        self.current_task = self.task_queue.pop(0)
        
        if self.current_task['type'] == 'approach':
            self.approach_target(self.current_task['target'])
        elif self.current_task['type'] == 'patrol':
            self.start_patrol(self.current_task['waypoints'])

    def approach_target(self, target):
        """接近目标"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target['x']
        goal_msg.pose.pose.position.y = target['y']
        
        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.nav_response_callback)
        
        self.get_logger().info(f"接近目标: ({target['x']:.2f}, {target['y']:.2f})")

    def nav_response_callback(self, future):
        """导航响应"""
        goal_handle = future.result()
        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.nav_result_callback)
        else:
            self.get_logger().error('导航请求被拒绝')
            self.current_task = None
            self.execute_next_task()

    def nav_result_callback(self, future):
        """导航完成"""
        self.get_logger().info('到达目标位置')
        
        # 执行到达后的操作
        if self.current_task and self.current_task.get('action'):
            self.execute_action(self.current_task['action'])
        
        self.current_task = None
        self.execute_next_task()

    def execute_action(self, action):
        """执行动作"""
        if action == 'scan':
            self.get_logger().info('扫描目标区域')
        elif action == 'pickup':
            self.get_logger().info('拾取目标')
        elif action == 'drop':
            self.get_logger().info('放置目标')


def main(args=None):
    rclpy.init(args=args)
    executor = UGVExecutor()
    rclpy.spin(executor)
    executor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 13.4 通信中继

### 13.4.1 UAV 通信中继节点

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json
import math

class CommunicationRelay(Node):
    """
    UAV 通信中继节点 - 扩展通信范围
    """
    def __init__(self):
        super().__init__('communication_relay')
        
        # 通信参数
        self.declare_parameter('max_range', 100.0)  # 最大通信距离
        self.declare_parameter('relay_height', 20.0)  # 中继高度
        
        # 平台位置
        self.uav_position = None
        self.ugv_position = None
        self.base_station = {'x': 0.0, 'y': 0.0}
        
        # 订阅位置
        self.uav_pos_sub = self.create_subscription(
            PoseStamped, '/uav/current_pose',
            self.uav_pos_callback, 10)
        
        self.ugv_pos_sub = self.create_subscription(
            PoseStamped, '/ugv/current_pose',
            self.ugv_pos_callback, 10)
        
        # 发布中继位置
        self.relay_pos_pub = self.create_publisher(
            PoseStamped, '/uav/relay_position', 10)
        
        # 状态发布
        self.status_pub = self.create_publisher(
            String, '/relay/status', 10)
        
        # 定时更新
        self.timer = self.create_timer(1.0, self.update_relay_position)
        
        self.get_logger().info('通信中继节点已启动')

    def uav_pos_callback(self, msg: PoseStamped):
        """更新 UAV 位置"""
        self.uav_position = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z
        }

    def ugv_pos_callback(self, msg: PoseStamped):
        """更新 UGV 位置"""
        self.ugv_position = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y
        }

    def update_relay_position(self):
        """更新中继位置"""
        if self.ugv_position is None:
            return
        
        max_range = self.get_parameter('max_range').value
        relay_height = self.get_parameter('relay_height').value
        
        # 计算 UGV 到基站的距离
        dist_to_base = math.sqrt(
            (self.ugv_position['x'] - self.base_station['x'])**2 +
            (self.ugv_position['y'] - self.base_station['y'])**2
        )
        
        # 判断是否需要中继
        if dist_to_base > max_range * 0.8:
            # 计算中继位置（在 UGV 和基站之间）
            ratio = max_range * 0.5 / dist_to_base
            
            relay_x = self.base_station['x'] + \
                (self.ugv_position['x'] - self.base_station['x']) * ratio
            relay_y = self.base_station['y'] + \
                (self.ugv_position['y'] - self.base_station['y']) * ratio
            
            # 发布中继位置
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = relay_x
            pose.pose.position.y = relay_y
            pose.pose.position.z = relay_height
            
            self.relay_pos_pub.publish(pose)
            
            # 发布状态
            status = {
                'mode': 'relay',
                'ugv_distance': dist_to_base,
                'relay_position': {'x': relay_x, 'y': relay_y}
            }
            msg = String()
            msg.data = json.dumps(status)
            self.status_pub.publish(msg)
            
            self.get_logger().info(
                f'中继模式: UGV 距离 {dist_to_base:.1f}m, '
                f'中继位置 ({relay_x:.1f}, {relay_y:.1f})'
            )
        else:
            # 不需要中继
            status = {
                'mode': 'normal',
                'ugv_distance': dist_to_base
            }
            msg = String()
            msg.data = json.dumps(status)
            self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=None)
    relay = CommunicationRelay()
    rclpy.spin(relay)
    relay.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 13.5 协同建图

### 13.5.1 空地协同 SLAM

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math

class AirGroundSLAM(Node):
    """
    空地协同 SLAM - 融合空中和地面传感器数据
    """
    def __init__(self):
        super().__init__('air_ground_slam')
        
        # 地图参数
        self.map_width = 200
        self.map_height = 200
        self.map_resolution = 0.1
        self.map_origin = [-10.0, -10.0]
        
        # 地图数据
        self.uav_map = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        self.ugv_map = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        self.fused_map = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        
        # 位姿
        self.uav_pose = None
        self.ugv_pose = None
        
        # 订阅
        self.uav_scan_sub = self.create_subscription(
            LaserScan, '/uav/scan_down',
            self.uav_scan_callback, 10)
        
        self.ugv_scan_sub = self.create_subscription(
            LaserScan, '/ugv/scan',
            self.ugv_scan_callback, 10)
        
        self.uav_odom_sub = self.create_subscription(
            Odometry, '/uav/odom',
            self.uav_odom_callback, 10)
        
        self.ugv_odom_sub = self.create_subscription(
            Odometry, '/ugv/odom',
            self.ugv_odom_callback, 10)
        
        # 发布
        self.map_pub = self.create_publisher(
            OccupancyGrid, '/fused_map', 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 定时发布
        self.timer = self.create_timer(0.5, self.publish_map)
        
        self.get_logger().info('空地协同 SLAM 已启动')

    def uav_odom_callback(self, msg: Odometry):
        """更新 UAV 位姿"""
        self.uav_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'yaw': self.quaternion_to_yaw(msg.pose.pose.orientation)
        }

    def ugv_odom_callback(self, msg: Odometry):
        """更新 UGV 位姿"""
        self.ugv_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': self.quaternion_to_yaw(msg.pose.pose.orientation)
        }

    def uav_scan_callback(self, msg: LaserScan):
        """处理 UAV 激光数据"""
        if self.uav_pose is None:
            return
        
        self.update_map(self.uav_map, msg, self.uav_pose, is_aerial=True)

    def ugv_scan_callback(self, msg: LaserScan):
        """处理 UGV 激光数据"""
        if self.ugv_pose is None:
            return
        
        self.update_map(self.ugv_map, msg, self.ugv_pose, is_aerial=False)

    def update_map(self, map_data, scan, pose, is_aerial=False):
        """更新地图"""
        angle = scan.angle_min
        
        for i, r in enumerate(scan.ranges):
            if r < scan.range_min or r > scan.range_max:
                angle += scan.angle_increment
                continue
            
            # 计算全局坐标
            if is_aerial:
                # 空中视角：激光朝下
                global_x = pose['x'] + r * math.cos(angle + pose['yaw'])
                global_y = pose['y'] + r * math.sin(angle + pose['yaw'])
            else:
                # 地面视角：水平激光
                global_x = pose['x'] + r * math.cos(angle + pose['yaw'])
                global_y = pose['y'] + r * math.sin(angle + pose['yaw'])
            
            # 转换到地图坐标
            map_x = int((global_x - self.map_origin[0]) / self.map_resolution)
            map_y = int((global_y - self.map_origin[1]) / self.map_resolution)
            
            # 检查边界
            if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                map_data[map_y, map_x] = 100  # 障碍物
            
            angle += scan.angle_increment

    def fuse_maps(self):
        """融合地图"""
        # 简单融合策略
        for y in range(self.map_height):
            for x in range(self.map_width):
                if self.uav_map[y, x] > 50 or self.ugv_map[y, x] > 50:
                    self.fused_map[y, x] = 100
                elif self.uav_map[y, x] == 0 and self.ugv_map[y, x] == 0:
                    self.fused_map[y, x] = -1  # 未知
                else:
                    self.fused_map[y, x] = 0  # 空闲

    def publish_map(self):
        """发布融合地图"""
        self.fuse_maps()
        
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = 'map'
        map_msg.header.stamp = self.get_clock().now().to_msg()
        
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.resolution = self.map_resolution
        map_msg.info.origin.position.x = self.map_origin[0]
        map_msg.info.origin.position.y = self.map_origin[1]
        
        map_msg.data = self.fused_map.flatten().tolist()
        
        self.map_pub.publish(map_msg)

    def quaternion_to_yaw(self, q):
        return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * q.z * q.z)


def main(args=None):
    rclpy.init(args=args)
    slam = AirGroundSLAM()
    rclpy.spin(slam)
    slam.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 13.6 空地协同 Launch 配置

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # 协调器
    coordinator = Node(
        package='air_ground_cooperation',
        executable='coordinator',
        name='air_ground_coordinator',
        output='screen'
    )
    
    # UAV 节点组
    uav_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uav_package'),
                'launch', 'uav.launch.py'
            ])
        ])
    )
    
    # UGV 节点组
    ugv_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ugv_package'),
                'launch', 'ugv.launch.py'
            ])
        ])
    )
    
    # 通信中继
    relay = Node(
        package='air_ground_cooperation',
        executable='communication_relay',
        name='communication_relay',
        output='screen'
    )
    
    # 协同 SLAM
    slam = Node(
        package='air_ground_cooperation',
        executable='air_ground_slam',
        name='air_ground_slam',
        output='screen'
    )
    
    return LaunchDescription([
        coordinator,
        uav_nodes,
        ugv_nodes,
        relay,
        slam
    ])
```

---


