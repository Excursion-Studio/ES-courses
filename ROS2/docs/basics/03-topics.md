# 第3章：话题 (Topic)

## 3.1 话题概念

话题 (Topic) 是 ROS2 中节点间异步通信的主要方式。发布者 (Publisher) 向话题发送消息，订阅者 (Subscriber) 从话题接收消息。

### 3.1.1 话题通信模型

<div align="center">
<svg width="700" height="280" viewBox="0 0 700 280" xmlns="http://www.w3.org/2000/svg">
  <defs>
    <marker id="arrow" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
      <polygon points="0 0, 10 3.5, 0 7" fill="#495057"/>
    </marker>
    <marker id="arrowBlue" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
      <polygon points="0 0, 10 3.5, 0 7" fill="#1976d2"/>
    </marker>
    <marker id="arrowOrange" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
      <polygon points="0 0, 10 3.5, 0 7" fill="#f57c00"/>
    </marker>
  </defs>
  
  <!-- 节点 A (发布者) -->
  <rect x="30" y="30" width="150" height="100" fill="#e3f2fd" stroke="#1976d2" stroke-width="2" rx="6"/>
  <text x="105" y="60" font-family="system-ui, -apple-system, sans-serif" font-size="16" font-weight="600" text-anchor="middle" fill="#1565c0">节点 A</text>
  <text x="105" y="85" font-family="system-ui, -apple-system, sans-serif" font-size="13" text-anchor="middle" fill="#424242">(发布者)</text>
  <text x="105" y="110" font-family="monospace" font-size="11" text-anchor="middle" fill="#757575">Publisher</text>
  
  <!-- 节点 B (订阅者) -->
  <rect x="520" y="30" width="150" height="100" fill="#fff3e0" stroke="#f57c00" stroke-width="2" rx="6"/>
  <text x="595" y="60" font-family="system-ui, -apple-system, sans-serif" font-size="16" font-weight="600" text-anchor="middle" fill="#ef6c00">节点 B</text>
  <text x="595" y="85" font-family="system-ui, -apple-system, sans-serif" font-size="13" text-anchor="middle" fill="#424242">(订阅者)</text>
  <text x="595" y="110" font-family="monospace" font-size="11" text-anchor="middle" fill="#757575">Subscriber</text>
  
  <!-- 话题框 -->
  <rect x="200" y="160" width="300" height="100" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="2" rx="6"/>
  <text x="350" y="185" font-family="system-ui, -apple-system, sans-serif" font-size="15" font-weight="600" text-anchor="middle" fill="#6a1b9a">话题 (Topic)</text>
  <text x="350" y="205" font-family="monospace" font-size="12" text-anchor="middle" fill="#4a148c">/sensor/temperature</text>
  
  <!-- 消息队列 -->
  <rect x="220" y="215" width="50" height="30" fill="#e8f5e9" stroke="#388e3c" stroke-width="1" rx="3"/>
  <text x="245" y="235" font-family="monospace" font-size="10" text-anchor="middle" fill="#2e7d32">msg1</text>
  
  <rect x="280" y="215" width="50" height="30" fill="#e8f5e9" stroke="#388e3c" stroke-width="1" rx="3"/>
  <text x="305" y="235" font-family="monospace" font-size="10" text-anchor="middle" fill="#2e7d32">msg2</text>
  
  <rect x="340" y="215" width="50" height="30" fill="#e8f5e9" stroke="#388e3c" stroke-width="1" rx="3"/>
  <text x="365" y="235" font-family="monospace" font-size="10" text-anchor="middle" fill="#2e7d32">msg3</text>
  
  <rect x="400" y="215" width="50" height="30" fill="#e8f5e9" stroke="#388e3c" stroke-width="1" rx="3"/>
  <text x="425" y="235" font-family="monospace" font-size="10" text-anchor="middle" fill="#2e7d32">...</text>
  
  <!-- 发布箭头 -->
  <path d="M 105 130 L 105 145 Q 105 155 115 155 L 200 155 Q 210 155 210 165 L 210 175" stroke="#1976d2" stroke-width="2" fill="none" marker-end="url(#arrowBlue)"/>
  <text x="140" y="150" font-family="system-ui, -apple-system, sans-serif" font-size="10" fill="#1976d2">publish</text>
  
  <!-- 订阅箭头 -->
  <path d="M 595 130 L 595 145 Q 595 155 585 155 L 500 155 Q 490 155 490 165 L 490 175" stroke="#f57c00" stroke-width="2" fill="none" marker-end="url(#arrowOrange)"/>
  <text x="530" y="150" font-family="system-ui, -apple-system, sans-serif" font-size="10" fill="#f57c00">subscribe</text>
  
  <!-- 数据流箭头 -->
  <line x1="460" y1="230" x2="510" y2="230" stroke="#7b1fa2" stroke-width="2" marker-end="url(#arrow)"/>
</svg>
</div>

### 3.1.2 话题特点

| 特点 | 说明 |
|------|------|
| 异步通信 | 发布者不需要等待订阅者响应 |
| 多对多 | 一个话题可以有多个发布者和订阅者 |
| 松耦合 | 发布者和订阅者相互独立 |
| 类型安全 | 消息必须符合定义的类型 |

## 3.2 标准消息类型

### 3.2.1 常用消息类型

| 消息类型 | 包 | 用途 |
|----------|-----|------|
| `std_msgs/String` | std_msgs | 字符串消息 |
| `std_msgs/Int32` | std_msgs | 整数消息 |
| `std_msgs/Float32` | std_msgs | 浮点数消息 |
| `std_msgs/Bool` | std_msgs | 布尔消息 |
| `geometry_msgs/Twist` | geometry_msgs | 速度指令 |
| `geometry_msgs/Pose` | geometry_msgs | 位姿信息 |
| `sensor_msgs/Image` | sensor_msgs | 图像数据 |
| `sensor_msgs/LaserScan` | sensor_msgs | 激光雷达数据 |
| `nav_msgs/Odometry` | nav_msgs | 里程计数据 |

### 3.2.2 查看消息类型

```bash
# 列出所有消息类型
ros2 interface list

# 查看消息定义
ros2 interface show std_msgs/msg/String

# 查看几何消息
ros2 interface show geometry_msgs/msg/Twist
```

输出示例：
```
std_msgs/msg/String:
string data

geometry_msgs/msg/Twist:
Vector3 linear
  float64 x
  float64 y
  float64 z
Vector3 angular
  float64 x
  float64 y
  float64 z
```

## 3.3 发布者与订阅者

### 3.3.1 Python 实现

#### 发布者节点

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 订阅者节点

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3.3.2 C++ 实现

#### 发布者节点

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher() : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello World: " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclpy::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
```

#### 订阅者节点

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber() : Node("minimal_subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic",
            10,
            std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
```

## 3.4 自定义消息类型

### 3.4.1 创建消息定义文件

创建功能包：
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_interfaces
mkdir -p my_interfaces/msg
```

创建消息文件 `my_interfaces/msg/RobotStatus.msg`：

```
# 机器人状态消息
string name                    # 机器人名称
float64 x                      # X坐标
float64 y                      # Y坐标
float64 theta                  # 朝向角度
bool is_active                 # 是否激活
float64 battery_level          # 电池电量 (0-100)
string[] sensors               # 传感器列表
```

### 3.4.2 配置功能包

#### package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_interfaces</name>
  <version>0.0.1</version>
  <description>Custom message definitions</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  
  <exec_depend>rosidl_default_runtime</exec_depend>
  
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

#### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
```

### 3.4.3 构建并使用

```bash
cd ~/ros2_ws
colcon build --packages-select my_interfaces
source install/setup.bash

# 查看生成的消息
ros2 interface show my_interfaces/msg/RobotStatus
```

### 3.4.4 使用自定义消息

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_interfaces.msg import RobotStatus

class RobotStatusPublisher(Node):
    def __init__(self):
        super().__init__('robot_status_publisher')
        
        self.publisher = self.create_publisher(RobotStatus, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)
    
    def publish_status(self):
        msg = RobotStatus()
        msg.name = 'Robot_001'
        msg.x = 1.5
        msg.y = 2.3
        msg.theta = 0.785
        msg.is_active = True
        msg.battery_level = 85.5
        msg.sensors = ['lidar', 'camera', 'imu']
        
        self.publisher.publish(msg)
        self.get_logger().info(f'发布机器人状态: {msg.name}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3.5 QoS (服务质量)

### 3.5.1 QoS 策略

| 策略 | 选项 | 说明 |
|------|------|------|
| **Reliability** | RELIABLE | 保证消息可靠传递 |
| | BEST_EFFORT | 尽力传递，可能丢失 |
| **Durability** | VOLATILE | 只传递给当前订阅者 |
| | TRANSIENT_LOCAL | 新订阅者可获取历史消息 |
| **History** | KEEP_LAST(N) | 保留最近N条消息 |
| | KEEP_ALL | 保留所有消息 |
| **Depth** | 数字 | 队列深度 |

### 3.5.2 QoS 配置示例

```python
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        
        # 传感器数据 QoS (高性能)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # 配置数据 QoS (可靠)
        config_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        self.sensor_pub = self.create_publisher(
            sensor_msgs.msg.Image, 'image_raw', sensor_qos)
        
        self.config_sub = self.create_subscription(
            std_msgs.msg.String, 'config', self.config_callback, config_qos)
```

### 3.5.3 QoS 兼容性

```
发布者 QoS          订阅者 QoS          兼容性
─────────────────────────────────────────────
RELIABLE        ◄──►  RELIABLE           ✓
RELIABLE        ◄──►  BEST_EFFORT        ✓ (降级)
BEST_EFFORT     ◄──►  RELIABLE           ✗
BEST_EFFORT     ◄──►  BEST_EFFORT        ✓
```

## 3.6 话题命令行工具

### 3.6.1 常用命令

```bash
# 列出所有话题
ros2 topic list

# 列出话题详细信息
ros2 topic list -t

# 查看话题信息
ros2 topic info /topic_name

# 查看话题类型
ros2 topic type /topic_name

# 查看话题消息
ros2 topic echo /topic_name

# 发布消息
ros2 topic pub /topic_name std_msgs/msg/String "data: 'Hello'"

# 发布消息 (持续)
ros2 topic pub -r 1 /topic_name std_msgs/msg/String "data: 'Hello'"

# 查看发布频率
ros2 topic hz /topic_name

# 查看带宽
ros2 topic bw /topic_name
```

### 3.6.2 实际示例

```bash
# 发布速度指令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"

# 查看激光雷达数据
ros2 topic echo /scan

# 检查话题发布频率
ros2 topic hz /odom
```

## 3.7 实践示例：传感器数据发布与订阅

### 3.7.1 模拟传感器节点

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from math import pi
import random

class FakeLaserNode(Node):
    def __init__(self):
        super().__init__('fake_laser_node')
        
        self.publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)
        
        self.angle_min = -pi
        self.angle_max = pi
        self.angle_increment = pi / 180
        self.range_min = 0.1
        self.range_max = 10.0
        
    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        
        num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)
        msg.ranges = [random.uniform(0.5, 8.0) for _ in range(num_readings)]
        
        self.publisher.publish(msg)
        self.get_logger().debug(f'发布激光数据: {num_readings} 个点')

def main(args=None):
    rclpy.init(args=args)
    node = FakeLaserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3.7.2 数据处理节点

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserProcessorNode(Node):
    def __init__(self):
        super().__init__('laser_processor_node')
        
        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
    
    def scan_callback(self, msg: LaserScan):
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        
        if valid_ranges:
            min_distance = min(valid_ranges)
            max_distance = max(valid_ranges)
            avg_distance = sum(valid_ranges) / len(valid_ranges)
            
            self.get_logger().info(
                f'激光数据 - 最小: {min_distance:.2f}m, '
                f'最大: {max_distance:.2f}m, '
                f'平均: {avg_distance:.2f}m'
            )

def main(args=None):
    rclpy.init(args=args)
    node = LaserProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3.8 话题最佳实践

### 3.8.1 命名规范

```bash
# 推荐：使用命名空间
/robot/sensors/lidar
/robot/sensors/camera
/robot/cmd_vel

# 避免：无命名空间
/lidar
/camera
/cmd_vel
```

### 3.8.2 QoS 选择指南

| 应用场景 | Reliability | Durability |
|----------|-------------|------------|
| 传感器数据 | BEST_EFFORT | VOLATILE |
| 控制指令 | RELIABLE | VOLATILE |
| 配置参数 | RELIABLE | TRANSIENT_LOCAL |
| 状态信息 | RELIABLE | TRANSIENT_LOCAL |

### 3.8.3 性能优化

1. **合理设置队列深度**：根据消息频率和消费速度设置
2. **选择合适的 QoS**：传感器用 BEST_EFFORT，控制用 RELIABLE
3. **避免消息过大**：必要时压缩或分片传输

---


