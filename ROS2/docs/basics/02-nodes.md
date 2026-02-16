# 第2章：节点 (Node)

## 2.1 节点概念

节点 (Node) 是 ROS2 中执行计算的基本单元。每个节点都是一个独立的进程，负责特定的功能，如传感器数据处理、运动控制、路径规划等。

### 2.1.1 节点的特点

- **独立性**：每个节点是独立的进程
- **单一职责**：一个节点专注于一个功能
- **可组合性**：多个节点协同完成复杂任务
- **可发现性**：节点可以相互发现并通信

### 2.1.2 节点架构图

<div align="center">
<svg width="700" height="260" viewBox="0 0 700 260" xmlns="http://www.w3.org/2000/svg">
  <defs>
    <marker id="arrow" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
      <polygon points="0 0, 10 3.5, 0 7" fill="#495057"/>
    </marker>
  </defs>
  
  <!-- 背景 -->
  <rect x="10" y="10" width="680" height="240" fill="#f8f9fa" stroke="#dee2e6" stroke-width="1" rx="4"/>
  <text x="350" y="35" font-family="system-ui, -apple-system, sans-serif" font-size="16" font-weight="600" text-anchor="middle" fill="#212529">ROS2 节点通信架构</text>
  
  <!-- 节点 A - 传感器 -->
  <rect x="30" y="55" width="130" height="80" fill="#e3f2fd" stroke="#1976d2" stroke-width="2" rx="6"/>
  <text x="95" y="80" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#1565c0">节点 A</text>
  <text x="95" y="100" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#424242">传感器</text>
  <text x="95" y="120" font-family="monospace" font-size="10" text-anchor="middle" fill="#757575">Publisher</text>
  
  <!-- 节点 B - 处理器 -->
  <rect x="200" y="55" width="130" height="80" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="2" rx="6"/>
  <text x="265" y="80" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#6a1b9a">节点 B</text>
  <text x="265" y="100" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#424242">处理器</text>
  <text x="265" y="120" font-family="monospace" font-size="10" text-anchor="middle" fill="#757575">Pub/Sub</text>
  
  <!-- 节点 C - 控制器 -->
  <rect x="370" y="55" width="130" height="80" fill="#e8f5e9" stroke="#388e3c" stroke-width="2" rx="6"/>
  <text x="435" y="80" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#2e7d32">节点 C</text>
  <text x="435" y="100" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#424242">控制器</text>
  <text x="435" y="120" font-family="monospace" font-size="10" text-anchor="middle" fill="#757575">Pub/Sub</text>
  
  <!-- 节点 D - 执行器 -->
  <rect x="540" y="55" width="130" height="80" fill="#fff3e0" stroke="#f57c00" stroke-width="2" rx="6"/>
  <text x="605" y="80" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#ef6c00">节点 D</text>
  <text x="605" y="100" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#424242">执行器</text>
  <text x="605" y="120" font-family="monospace" font-size="10" text-anchor="middle" fill="#757575">Subscriber</text>
  
  <!-- 数据流箭头 -->
  <line x1="160" y1="95" x2="200" y2="95" stroke="#1976d2" stroke-width="2" marker-end="url(#arrow)"/>
  <line x1="330" y1="95" x2="370" y2="95" stroke="#7b1fa2" stroke-width="2" marker-end="url(#arrow)"/>
  <line x1="500" y1="95" x2="540" y2="95" stroke="#388e3c" stroke-width="2" marker-end="url(#arrow)"/>
  
  <!-- DDS 通信层 -->
  <rect x="30" y="160" width="640" height="70" fill="#eceff1" stroke="#546e7a" stroke-width="2" rx="5"/>
  <text x="350" y="185" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#37474f">DDS 通信层 (Data Distribution Service)</text>
  
  <!-- DDS 功能 -->
  <rect x="60" y="195" width="90" height="28" fill="#ffffff" stroke="#78909c" stroke-width="1" rx="3"/>
  <text x="105" y="214" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#455a64">话题发现</text>
  
  <rect x="170" y="195" width="90" height="28" fill="#ffffff" stroke="#78909c" stroke-width="1" rx="3"/>
  <text x="215" y="214" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#455a64">消息传输</text>
  
  <rect x="280" y="195" width="90" height="28" fill="#ffffff" stroke="#78909c" stroke-width="1" rx="3"/>
  <text x="325" y="214" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#455a64">QoS 管理</text>
  
  <rect x="390" y="195" width="90" height="28" fill="#ffffff" stroke="#78909c" stroke-width="1" rx="3"/>
  <text x="435" y="214" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#455a64">安全认证</text>
  
  <rect x="500" y="195" width="90" height="28" fill="#ffffff" stroke="#78909c" stroke-width="1" rx="3"/>
  <text x="545" y="214" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#455a64">序列化</text>
  
  <!-- 节点到 DDS 连接线 -->
  <line x1="95" y1="135" x2="95" y2="160" stroke="#1976d2" stroke-width="1.5" stroke-dasharray="4,2"/>
  <line x1="265" y1="135" x2="265" y2="160" stroke="#7b1fa2" stroke-width="1.5" stroke-dasharray="4,2"/>
  <line x1="435" y1="135" x2="435" y2="160" stroke="#388e3c" stroke-width="1.5" stroke-dasharray="4,2"/>
  <line x1="605" y1="135" x2="605" y2="160" stroke="#f57c00" stroke-width="1.5" stroke-dasharray="4,2"/>
</svg>
</div>

## 2.2 创建第一个 Python 节点

### 2.2.1 创建功能包

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_first_node
```

### 2.2.2 编写节点代码

创建文件 `my_first_node/my_first_node/my_node.py`：

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self):
        super().__init__('my_first_node')
        self.get_logger().info('你好，ROS2！我是第一个节点')
        
        self.counter = 0
        self.timer = self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f'节点运行中... 计数: {self.counter}')

def main(args=None):
    rclpy.init(args=args)
    
    node = MyFirstNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2.2.3 配置 setup.py

```python
from setuptools import setup

package_name = 'my_first_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='My first ROS2 node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_first_node.my_node:main',
        ],
    },
)
```

### 2.2.4 构建与运行

```bash
cd ~/ros2_ws
colcon build --packages-select my_first_node
source install/setup.bash
ros2 run my_first_node my_node
```

## 2.3 创建第一个 C++ 节点

### 2.3.1 创建功能包

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_cpp_node
```

### 2.3.2 编写节点代码

创建文件 `my_cpp_node/src/my_node.cpp`：

```cpp
#include "rclcpp/rclcpp.hpp"

class MyFirstNode : public rclcpp::Node
{
public:
    MyFirstNode() : Node("my_first_node"), counter_(0)
    {
        RCLCPP_INFO(this->get_logger(), "你好，ROS2！我是第一个C++节点");
        
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MyFirstNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        counter_++;
        RCLCPP_INFO(this->get_logger(), "节点运行中... 计数: %d", counter_);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyFirstNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### 2.3.3 配置 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_cpp_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp)

install(TARGETS
  my_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

### 2.3.4 配置 package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_cpp_node</name>
  <version>0.0.1</version>
  <description>My first C++ ROS2 node</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### 2.3.5 构建与运行

```bash
cd ~/ros2_ws
colcon build --packages-select my_cpp_node
source install/setup.bash
ros2 run my_cpp_node my_node
```

## 2.4 节点生命周期

ROS2 提供了生命周期节点 (Lifecycle Node)，允许管理节点的状态转换。

### 2.4.1 生命周期状态

<div align="center">
<svg width="600" height="480" viewBox="0 0 600 480" xmlns="http://www.w3.org/2000/svg">
  <defs>
    <marker id="arrowLife" markerWidth="8" markerHeight="6" refX="7" refY="3" orient="auto">
      <polygon points="0 0, 8 3, 0 6" fill="#495057"/>
    </marker>
  </defs>
  
  <!-- 背景 -->
  <rect x="10" y="10" width="580" height="460" fill="#f8f9fa" stroke="#dee2e6" stroke-width="1" rx="4"/>
  <text x="300" y="40" font-family="system-ui, -apple-system, sans-serif" font-size="18" font-weight="600" text-anchor="middle" fill="#212529">节点生命周期状态图</text>
  
  <!-- Unconfigured -->
  <rect x="235" y="60" width="130" height="45" fill="#fff3cd" stroke="#ffc107" stroke-width="2" rx="5"/>
  <text x="300" y="88" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#856404">Unconfigured</text>
  
  <!-- Inactive 1 -->
  <rect x="235" y="150" width="130" height="45" fill="#d1ecf1" stroke="#17a2b8" stroke-width="2" rx="5"/>
  <text x="300" y="178" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#0c5460">Inactive</text>
  
  <!-- Active -->
  <rect x="235" y="240" width="130" height="45" fill="#d4edda" stroke="#28a745" stroke-width="2" rx="5"/>
  <text x="300" y="268" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#155724">Active</text>
  
  <!-- Inactive 2 -->
  <rect x="235" y="330" width="130" height="45" fill="#d1ecf1" stroke="#17a2b8" stroke-width="2" rx="5"/>
  <text x="300" y="358" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#0c5460">Inactive</text>
  
  <!-- Finalized -->
  <rect x="235" y="410" width="130" height="45" fill="#f8d7da" stroke="#dc3545" stroke-width="2" rx="5"/>
  <text x="300" y="438" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#721c24">Finalized</text>
  
  <!-- 连接箭头 Unconfigured -> Inactive -->
  <line x1="300" y1="105" x2="300" y2="150" stroke="#495057" stroke-width="1.5" marker-end="url(#arrowLife)"/>
  <text x="340" y="130" font-family="system-ui, -apple-system, sans-serif" font-size="11" font-weight="500" fill="#495057">configure()</text>
  
  <!-- 连接箭头 Inactive -> Active -->
  <line x1="300" y1="195" x2="300" y2="240" stroke="#495057" stroke-width="1.5" marker-end="url(#arrowLife)"/>
  <text x="345" y="220" font-family="system-ui, -apple-system, sans-serif" font-size="11" font-weight="500" fill="#495057">activate()</text>
  
  <!-- 连接箭头 Active -> Inactive (从右侧) -->
  <path d="M 365 262 L 430 262 L 430 172 L 365 172" stroke="#495057" stroke-width="1.5" fill="none" marker-end="url(#arrowLife)"/>
  <text x="450" y="217" font-family="system-ui, -apple-system, sans-serif" font-size="11" font-weight="500" fill="#495057">deactivate()</text>
  
  <!-- 连接箭头 Inactive -> Inactive (从左侧) -->
  <path d="M 235 172 L 150 172 L 150 352 L 235 352" stroke="#495057" stroke-width="1.5" fill="none" marker-end="url(#arrowLife)"/>
  <text x="110" y="262" font-family="system-ui, -apple-system, sans-serif" font-size="11" font-weight="500" fill="#495057">cleanup()</text>
  
  <!-- 连接箭头 Inactive -> Finalized -->
  <line x1="300" y1="375" x2="300" y2="410" stroke="#495057" stroke-width="1.5" marker-end="url(#arrowLife)"/>
  <text x="340" y="395" font-family="system-ui, -apple-system, sans-serif" font-size="11" font-weight="500" fill="#495057">cleanup()</text>
</svg>
</div>

### 2.4.2 生命周期节点示例 (Python)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn

class MyLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('my_lifecycle_node')
    
    def on_configure(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('配置节点...')
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('激活节点...')
        return super().on_activate(previous_state)
    
    def on_deactivate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('停用节点...')
        return super().on_deactivate(previous_state)
    
    def on_cleanup(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('清理节点...')
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('关闭节点...')
        return TransitionCallbackReturn.SUCCESS

def main(args=None):
    rclpy.init(args=args)
    node = MyLifecycleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 2.5 节点命令行工具

### 2.5.1 列出所有节点

```bash
ros2 node list
```

输出示例：
```
/my_first_node
/talker
/listener
```

### 2.5.2 查看节点信息

```bash
ros2 node info /my_first_node
```

输出示例：
```
/my_first_node
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /rosout: rcl_interfaces/msg/Log
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Services:
    /my_first_node/get_type_description: type_description_interfaces/srv/GetTypeDescription
    /my_first_node/get_parameters: rcl_interfaces/srv/GetParameters
    ...
```

### 2.5.3 管理生命周期节点

```bash
# 查看生命周期状态
ros2 lifecycle list /my_lifecycle_node

# 触发状态转换
ros2 lifecycle set /my_lifecycle_node configure
ros2 lifecycle set /my_lifecycle_node activate
ros2 lifecycle set /my_lifecycle_node deactivate
```

## 2.6 实践示例：简单发布者/订阅者节点

### 2.6.1 发布者节点 (Python)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        
        self.publisher = self.create_publisher(String, 'topic', 10)
        
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.counter = 0
    
    def timer_callback(self):
        msg = String()
        msg.data = f'你好，这是第 {self.counter} 条消息'
        self.publisher.publish(msg)
        self.get_logger().info(f'发布: "{msg.data}"')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2.6.2 订阅者节点 (Python)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
    
    def listener_callback(self, msg):
        self.get_logger().info(f'收到: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2.6.3 运行示例

```bash
# 终端1：运行发布者
ros2 run my_first_node publisher_node

# 终端2：运行订阅者
ros2 run my_first_node subscriber_node
```

## 2.7 节点最佳实践

### 2.7.1 命名规范

| 类型 | 规范 | 示例 |
|------|------|------|
| 节点名 | 小写下划线 | `sensor_driver` |
| 话题名 | 小写下划线 | `/camera/image_raw` |
| 服务名 | 小写下划线 | `/get_position` |
| 动作名 | 小写下划线 | `/navigate_to_goal` |

### 2.7.2 代码规范

1. **使用类继承 Node**：保持代码结构清晰
2. **合理使用日志级别**：
   - `DEBUG`：调试信息
   - `INFO`：一般信息
   - `WARN`：警告信息
   - `ERROR`：错误信息
3. **资源清理**：在节点销毁时释放资源
4. **异常处理**：使用 try-except 捕获异常

---


