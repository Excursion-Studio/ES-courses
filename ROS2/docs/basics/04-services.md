# 第4章：服务 (Service)

## 4.1 服务概念

服务 (Service) 是 ROS2 中节点间同步通信的方式。客户端发送请求，服务端处理后返回响应，类似于函数调用的模式。

### 4.1.1 服务通信模型

<div align="center">
<svg width="650" height="260" viewBox="0 0 650 260" xmlns="http://www.w3.org/2000/svg">
  <defs>
    <marker id="arrowBlue" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
      <polygon points="0 0, 10 3.5, 0 7" fill="#1976d2"/>
    </marker>
    <marker id="arrowGreen" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
      <polygon points="0 0, 10 3.5, 0 7" fill="#388e3c"/>
    </marker>
  </defs>
  
  <!-- 背景 -->
  <rect x="10" y="10" width="630" height="240" fill="#f8f9fa" stroke="#dee2e6" stroke-width="1" rx="4"/>
  
  <!-- 客户端 -->
  <rect x="40" y="40" width="170" height="100" fill="#e3f2fd" stroke="#1976d2" stroke-width="2" rx="6"/>
  <text x="125" y="70" font-family="system-ui, -apple-system, sans-serif" font-size="16" font-weight="600" text-anchor="middle" fill="#1565c0">客户端</text>
  <text x="125" y="95" font-family="monospace" font-size="12" text-anchor="middle" fill="#424242">(Client)</text>
  <text x="125" y="120" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#757575">服务调用方</text>
  
  <!-- 服务端 -->
  <rect x="440" y="40" width="170" height="100" fill="#fff3e0" stroke="#f57c00" stroke-width="2" rx="6"/>
  <text x="525" y="70" font-family="system-ui, -apple-system, sans-serif" font-size="16" font-weight="600" text-anchor="middle" fill="#ef6c00">服务端</text>
  <text x="525" y="95" font-family="monospace" font-size="12" text-anchor="middle" fill="#424242">(Server)</text>
  <text x="525" y="120" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#757575">服务提供方</text>
  
  <!-- 请求箭头 -->
  <line x1="220" y1="75" x2="430" y2="75" stroke="#1976d2" stroke-width="2.5" marker-end="url(#arrowBlue)"/>
  <text x="325" y="65" font-family="system-ui, -apple-system, sans-serif" font-size="12" font-weight="600" text-anchor="middle" fill="#1976d2">请求 (Request)</text>
  
  <!-- 响应箭头 -->
  <line x1="430" y1="115" x2="220" y2="115" stroke="#388e3c" stroke-width="2.5" marker-end="url(#arrowGreen)"/>
  <text x="325" y="135" font-family="system-ui, -apple-system, sans-serif" font-size="12" font-weight="600" text-anchor="middle" fill="#388e3c">响应 (Response)</text>
  
  <!-- 处理框 -->
  <rect x="460" y="155" width="130" height="45" fill="#e8f5e9" stroke="#388e3c" stroke-width="1.5" rx="4"/>
  <text x="525" y="183" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#2e7d32">处理请求</text>
  
  <!-- 处理连接线 -->
  <line x1="525" y1="140" x2="525" y2="155" stroke="#495057" stroke-width="1.5" stroke-dasharray="4,2"/>
  
  <!-- 同步标签 -->
  <rect x="240" y="180" width="130" height="35" fill="#fce4ec" stroke="#c2185b" stroke-width="1.5" rx="4"/>
  <text x="305" y="203" font-family="system-ui, -apple-system, sans-serif" font-size="12" font-weight="600" text-anchor="middle" fill="#ad1457">同步通信</text>
  
  <!-- 阻塞标签 -->
  <rect x="60" y="180" width="130" height="35" fill="#fff8e1" stroke="#ffa000" stroke-width="1.5" rx="4"/>
  <text x="125" y="203" font-family="system-ui, -apple-system, sans-serif" font-size="12" font-weight="600" text-anchor="middle" fill="#ff6f00">阻塞等待</text>
</svg>
</div>

### 4.1.2 服务 vs 话题对比

| 特性 | 话题 (Topic) | 服务 (Service) |
|------|--------------|----------------|
| 通信模式 | 异步 | 同步 |
| 数据流向 | 单向 | 双向 |
| 参与者 | 多对多 | 一对一 |
| 适用场景 | 持续数据流 | 请求-响应操作 |
| 实时性 | 高 | 中等 |

### 4.1.3 适用场景

- ✅ 获取传感器单次数据
- ✅ 执行一次性操作（如抓取物体）
- ✅ 查询机器人状态
- ✅ 参数配置
- ❌ 持续数据传输（用话题）
- ❌ 长时间任务（用动作）

## 4.2 标准服务类型

### 4.2.1 常用服务类型

| 服务类型 | 包 | 用途 |
|----------|-----|------|
| `std_srvs/Empty` | std_srvs | 无参数调用 |
| `std_srvs/SetBool` | std_srvs | 设置布尔值 |
| `std_srvs/Trigger` | std_srvs | 触发操作 |
| `example_interfaces/AddTwoInts` | example_interfaces | 加法示例 |

### 4.2.2 查看服务类型

```bash
# 列出所有服务类型
ros2 interface list | grep srv

# 查看服务定义
ros2 interface show std_srvs/srv/SetBool

# 查看 Trigger 服务
ros2 interface show std_srvs/srv/Trigger
```

输出示例：
```
std_srvs/srv/SetBool:
bool data
---
bool success
string message

std_srvs/srv/Trigger:
---
bool success
string message
```

## 4.3 服务端与客户端

### 4.3.1 Python 实现

#### 服务端节点

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        
        self.srv = self.create_service(
            AddTwoInts, 
            'add_two_ints', 
            self.add_two_ints_callback
        )
        self.get_logger().info('服务已启动: add_two_ints')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'收到请求: {request.a} + {request.b}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 客户端节点

```python
#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务启动...')
        
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    
    minimal_client = MinimalClientAsync()
    
    a = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    b = int(sys.argv[2]) if len(sys.argv) > 2 else 3
    
    response = minimal_client.send_request(a, b)
    minimal_client.get_logger().info(
        f'请求: {a} + {b} = {response.sum}'
    )
    
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4.3.2 C++ 实现

#### 服务端节点

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class MinimalService : public rclcpp::Node
{
public:
    MinimalService() : Node("minimal_service")
    {
        service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&MinimalService::add, this, std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "服务已启动: add_two_ints");
    }

private:
    void add(
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "收到请求: %ld + %ld", request->a, request->b);
    }
    
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalService>());
    rclcpp::shutdown();
    return 0;
}
```

#### 客户端节点

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class MinimalClient : public rclcpp::Node
{
public:
    MinimalClient() : Node("minimal_client")
    {
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    }

    void send_request(int64_t a, int64_t b)
    {
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "等待服务时被中断");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "等待服务启动...");
        }

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        auto future_result = client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "请求: %ld + %ld = %ld", a, b, future_result.get()->sum);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "服务调用失败");
        }
    }

private:
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto client = std::make_shared<MinimalClient>();
    int64_t a = (argc > 1) ? atoll(argv[1]) : 2;
    int64_t b = (argc > 2) ? atoll(argv[2]) : 3;
    
    client->send_request(a, b);
    
    rclcpp::shutdown();
    return 0;
}
```

## 4.4 自定义服务类型

### 4.4.1 创建服务定义文件

创建文件 `my_interfaces/srv/GetRobotPose.srv`：

```
# 请求 (Request)
string robot_name          # 机器人名称
---
# 响应 (Response)
float64 x                  # X坐标
float64 y                  # Y坐标
float64 theta              # 朝向角度
bool success               # 是否成功
string message             # 状态消息
```

### 4.4.2 配置功能包

#### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "srv/GetRobotPose.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
```

#### package.xml

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

### 4.4.3 构建并使用

```bash
cd ~/ros2_ws
colcon build --packages-select my_interfaces
source install/setup.bash

# 查看生成的服务
ros2 interface show my_interfaces/srv/GetRobotPose
```

## 4.5 服务命令行工具

### 4.5.1 常用命令

```bash
# 列出所有服务
ros2 service list

# 查看服务类型
ros2 service type /add_two_ints

# 查看服务接口
ros2 interface show example_interfaces/srv/AddTwoInts

# 调用服务
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"

# 查找使用某类型的所有服务
ros2 service find example_interfaces/srv/AddTwoInts
```

### 4.5.2 实际示例

```bash
# 调用触发服务
ros2 service call /trigger std_srvs/srv/Trigger

# 设置布尔参数
ros2 service call /enable std_srvs/srv/SetBool "{data: true}"

# 获取机器人位置
ros2 service call /get_pose my_interfaces/srv/GetRobotPose "{robot_name: 'robot_001'}"
```

## 4.6 实践示例：计算服务与数据查询服务

### 4.6.1 机器人状态查询服务

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_interfaces.srv import GetRobotPose
from geometry_msgs.msg import Pose

class RobotPoseServer(Node):
    def __init__(self):
        super().__init__('robot_pose_server')
        
        self.robots = {
            'robot_001': {'x': 1.0, 'y': 2.0, 'theta': 0.5},
            'robot_002': {'x': 3.0, 'y': 1.5, 'theta': 1.2},
            'robot_003': {'x': 0.5, 'y': 4.0, 'theta': -0.3},
        }
        
        self.srv = self.create_service(
            GetRobotPose, 
            'get_robot_pose', 
            self.get_pose_callback
        )
        self.get_logger().info('机器人位置服务已启动')

    def get_pose_callback(self, request, response):
        robot_name = request.robot_name
        
        if robot_name in self.robots:
            pose = self.robots[robot_name]
            response.x = pose['x']
            response.y = pose['y']
            response.theta = pose['theta']
            response.success = True
            response.message = f'成功获取 {robot_name} 的位置'
        else:
            response.x = 0.0
            response.y = 0.0
            response.theta = 0.0
            response.success = False
            response.message = f'未找到机器人: {robot_name}'
        
        self.get_logger().info(f'查询请求: {robot_name}')
        return response

def main(args=None):
    rclpy.init(args=args)
    server = RobotPoseServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4.6.2 客户端调用示例

```python
#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from my_interfaces.srv import GetRobotPose

class RobotPoseClient(Node):
    def __init__(self):
        super().__init__('robot_pose_client')
        self.cli = self.create_client(GetRobotPose, 'get_robot_pose')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务启动...')
    
    def query_pose(self, robot_name):
        request = GetRobotPose.Request()
        request.robot_name = robot_name
        
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    client = RobotPoseClient()
    
    robot_name = sys.argv[1] if len(sys.argv) > 1 else 'robot_001'
    
    response = client.query_pose(robot_name)
    
    if response.success:
        client.get_logger().info(
            f'{robot_name} 位置: ({response.x:.2f}, {response.y:.2f}), '
            f'朝向: {response.theta:.2f} rad'
        )
    else:
        client.get_logger().error(response.message)
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4.6.3 运行示例

```bash
# 终端1：启动服务端
ros2 run my_package robot_pose_server

# 终端2：调用服务
ros2 service call /get_robot_pose my_interfaces/srv/GetRobotPose "{robot_name: 'robot_001'}"

# 终端3：运行客户端
ros2 run my_package robot_pose_client robot_002
```

## 4.7 异步服务调用

### 4.7.1 异步客户端示例

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import asyncio

class AsyncServiceClient(Node):
    def __init__(self):
        super().__init__('async_service_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
    
    async def call_service_async(self, a, b):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务...')
        
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        future = self.cli.call_async(request)
        
        def callback(future):
            response = future.result()
            self.get_logger().info(f'异步结果: {a} + {b} = {response.sum}')
        
        future.add_done_callback(callback)
        return future

def main(args=None):
    rclpy.init(args=args)
    client = AsyncServiceClient()
    
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(client,), daemon=True)
    spin_thread.start()
    
    import time
    for i in range(5):
        future = client.call_service_async(i, i * 2)
        time.sleep(0.5)
    
    time.sleep(2)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 4.8 服务最佳实践

### 4.8.1 设计原则

1. **快速响应**：服务应尽快返回结果
2. **幂等性**：相同请求应产生相同结果
3. **错误处理**：返回明确的成功/失败状态
4. **合理命名**：服务名应清晰表达功能

### 4.8.2 命名规范

```bash
# 推荐：动词开头
/get_pose
/set_velocity
/reset_position
/calculate_path

# 避免：名词
/pose
/velocity
/position
```

### 4.8.3 常见问题

| 问题 | 解决方案 |
|------|----------|
| 服务调用超时 | 检查服务是否启动，增加超时时间 |
| 客户端阻塞 | 使用异步调用 `call_async` |
| 服务响应慢 | 考虑使用动作 (Action) |

---


