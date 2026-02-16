# 第5章：参数 (Parameter)

## 5.1 参数系统概述

参数 (Parameter) 是 ROS2 中用于配置节点的机制。每个节点可以声明、获取、设置参数，实现运行时配置。

### 5.1.1 参数特点

| 特点 | 说明 |
|------|------|
| 类型安全 | 支持多种数据类型 |
| 运行时修改 | 无需重启节点 |
| 外部配置 | 可通过文件或命令行设置 |
| 回调机制 | 参数变化时触发回调 |

### 5.1.2 参数类型

| 类型 | Python | C++ | 说明 |
|------|--------|-----|------|
| 布尔 | bool | bool | true/false |
| 整数 | int | int64_t | 整数值 |
| 浮点 | float | double | 浮点数 |
| 字符串 | str | std::string | 文本 |
| 字节数组 | bytes | std::vector<uint8_t> | 二进制数据 |
| 布尔数组 | list[bool] | std::vector<bool> | 布尔列表 |
| 整数数组 | list[int] | std::vector<int64_t> | 整数列表 |
| 浮点数组 | list[float] | std::vector<double> | 浮点列表 |
| 字符串数组 | list[str] | std::vector<std::string> | 字符串列表 |

## 5.2 参数声明与获取

### 5.2.1 Python 实现

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # 声明参数（带默认值）
        self.declare_parameter('my_string', 'Hello ROS2')
        self.declare_parameter('my_int', 42)
        self.declare_parameter('my_float', 3.14)
        self.declare_parameter('my_bool', True)
        self.declare_parameter('my_array', [1, 2, 3, 4, 5])
        
        # 获取参数值
        my_string = self.get_parameter('my_string').value
        my_int = self.get_parameter('my_int').value
        my_float = self.get_parameter('my_float').value
        my_bool = self.get_parameter('my_bool').value
        my_array = self.get_parameter('my_array').value
        
        self.get_logger().info(f'字符串参数: {my_string}')
        self.get_logger().info(f'整数参数: {my_int}')
        self.get_logger().info(f'浮点参数: {my_float}')
        self.get_logger().info(f'布尔参数: {my_bool}')
        self.get_logger().info(f'数组参数: {my_array}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 5.2.2 C++ 实现

```cpp
#include "rclcpp/rclcpp.hpp"

class ParameterNode : public rclcpp::Node
{
public:
    ParameterNode() : Node("parameter_node")
    {
        // 声明参数
        this->declare_parameter("my_string", "Hello ROS2");
        this->declare_parameter("my_int", 42);
        this->declare_parameter("my_float", 3.14);
        this->declare_parameter("my_bool", true);
        
        // 获取参数值
        std::string my_string = this->get_parameter("my_string").as_string();
        int64_t my_int = this->get_parameter("my_int").as_int();
        double my_float = this->get_parameter("my_float").as_double();
        bool my_bool = this->get_parameter("my_bool").as_bool();
        
        RCLCPP_INFO(this->get_logger(), "字符串参数: %s", my_string.c_str());
        RCLCPP_INFO(this->get_logger(), "整数参数: %ld", my_int);
        RCLCPP_INFO(this->get_logger(), "浮点参数: %.2f", my_float);
        RCLCPP_INFO(this->get_logger(), "布尔参数: %s", my_bool ? "true" : "false");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParameterNode>());
    rclcpp::shutdown();
    return 0;
}
```

## 5.3 参数动态更新

### 5.3.1 参数回调机制

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

class DynamicParameterNode(Node):
    def __init__(self):
        super().__init__('dynamic_parameter_node')
        
        # 声明参数
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('message', 'Hello')
        self.declare_parameter('enabled', True)
        
        # 注册参数变化回调
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # 创建定时器
        self.timer = self.create_timer(
            self.get_parameter('frequency').value,
            self.timer_callback
        )
        
        self.get_logger().info('动态参数节点已启动')

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'frequency':
                if param.value <= 0:
                    self.get_logger().error('频率必须大于0')
                    return SetParametersResult(successful=False)
                # 更新定时器
                self.timer.cancel()
                self.timer = self.create_timer(param.value, self.timer_callback)
                self.get_logger().info(f'频率更新为: {param.value} Hz')
            
            elif param.name == 'message':
                self.get_logger().info(f'消息更新为: {param.value}')
            
            elif param.name == 'enabled':
                self.get_logger().info(f'启用状态: {param.value}')
        
        return SetParametersResult(successful=True)

    def timer_callback(self):
        if self.get_parameter('enabled').value:
            msg = self.get_parameter('message').value
            self.get_logger().info(f'定时消息: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = DynamicParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 5.3.2 C++ 参数回调

```cpp
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

class DynamicParameterNode : public rclcpp::Node
{
public:
    DynamicParameterNode() : Node("dynamic_parameter_node")
    {
        // 声明参数
        this->declare_parameter("frequency", 1.0);
        this->declare_parameter("message", "Hello");
        this->declare_parameter("enabled", true);
        
        // 注册回调
        callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&DynamicParameterNode::parameterCallback, this, std::placeholders::_1));
        
        // 创建定时器
        double freq = this->get_parameter("frequency").as_double();
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / freq),
            std::bind(&DynamicParameterNode::timerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "动态参数节点已启动");
    }

private:
    rcl_interfaces::msg::SetParametersResult parameterCallback(
        const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        
        for (const auto &param : params)
        {
            if (param.get_name() == "frequency")
            {
                double freq = param.as_double();
                if (freq <= 0)
                {
                    result.successful = false;
                    result.reason = "频率必须大于0";
                    return result;
                }
                
                timer_->cancel();
                timer_ = this->create_wall_timer(
                    std::chrono::duration<double>(1.0 / freq),
                    std::bind(&DynamicParameterNode::timerCallback, this));
                
                RCLCPP_INFO(this->get_logger(), "频率更新为: %.2f Hz", freq);
            }
            else if (param.get_name() == "message")
            {
                RCLCPP_INFO(this->get_logger(), "消息更新为: %s", param.as_string().c_str());
            }
            else if (param.get_name() == "enabled")
            {
                RCLCPP_INFO(this->get_logger(), "启用状态: %s", param.as_bool() ? "true" : "false");
            }
        }
        
        return result;
    }
    
    void timerCallback()
    {
        if (this->get_parameter("enabled").as_bool())
        {
            RCLCPP_INFO(this->get_logger(), "定时消息: %s", 
                this->get_parameter("message").as_string().c_str());
        }
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicParameterNode>());
    rclcpp::shutdown();
    return 0;
}
```

## 5.4 参数描述与约束

### 5.4.1 参数描述

```python
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, FloatingPointRange

class DescribedParameterNode(Node):
    def __init__(self):
        super().__init__('described_parameter_node')
        
        # 带描述的参数
        speed_descriptor = ParameterDescriptor(
            name='speed',
            type=3,  # PARAMETER_DOUBLE
            description='机器人移动速度 (m/s)',
            additional_constraints='必须在 0.0 到 5.0 之间',
            read_only=False
        )
        
        # 带范围约束的参数
        speed_range = FloatingPointRange()
        speed_range.from_value = 0.0
        speed_range.to_value = 5.0
        speed_range.step = 0.1
        speed_descriptor.floating_point_range = [speed_range]
        
        self.declare_parameter('speed', 1.0, speed_descriptor)
        
        # 整数范围参数
        mode_descriptor = ParameterDescriptor(
            name='mode',
            type=2,  # PARAMETER_INTEGER
            description='工作模式'
        )
        mode_range = IntegerRange()
        mode_range.from_value = 0
        mode_range.to_value = 3
        mode_range.step = 1
        mode_descriptor.integer_range = [mode_range]
        
        self.declare_parameter('mode', 0, mode_descriptor)
```

## 5.5 YAML 配置文件

### 5.5.1 配置文件格式

创建文件 `config/params.yaml`：

```yaml
parameter_node:
  ros__parameters:
    my_string: "Hello from YAML"
    my_int: 100
    my_float: 2.718
    my_bool: false
    my_array: [10, 20, 30, 40, 50]

controller_node:
  ros__parameters:
    kp: 1.5
    ki: 0.1
    kd: 0.05
    max_speed: 2.0
    enabled: true
```

### 5.5.2 命名空间配置

```yaml
robot:
  sensor_node:
    ros__parameters:
      frame_id: "laser_link"
      scan_frequency: 10.0
  
  controller_node:
    ros__parameters:
      kp: 1.0
      ki: 0.1
      kd: 0.01
```

### 5.5.3 加载配置文件

#### 通过命令行加载

```bash
ros2 run my_package parameter_node --ros-args --params-file config/params.yaml
```

#### 通过 Launch 文件加载

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('my_package'),
        'config',
        'params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='my_package',
            executable='parameter_node',
            name='parameter_node',
            parameters=[config_file]
        )
    ])
```

## 5.6 参数命令行工具

### 5.6.1 常用命令

```bash
# 列出节点的所有参数
ros2 param list

# 获取参数值
ros2 param get /parameter_node my_string

# 设置参数值
ros2 param set /parameter_node my_int 200

# 查看参数描述
ros2 param describe /parameter_node speed

# 导出参数到文件
ros2 param dump /parameter_node

# 从文件加载参数
ros2 param load /parameter_node params.yaml
```

### 5.6.2 实际示例

```bash
# 启动节点
ros2 run my_package dynamic_parameter_node

# 查看参数列表
ros2 param list /dynamic_parameter_node

# 修改频率
ros2 param set /dynamic_parameter_node frequency 2.0

# 修改消息
ros2 param set /dynamic_parameter_node message "ROS2 is awesome"

# 禁用
ros2 param set /dynamic_parameter_node enabled false

# 导出当前参数
ros2 param dump /dynamic_parameter_node > current_params.yaml
```

## 5.7 实践示例：可配置的控制器节点

### 5.7.1 PID 控制器节点

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller')
        
        # 声明 PID 参数
        self.declare_parameter('kp', 1.0, 
            ParameterDescriptor(description='比例增益'))
        self.declare_parameter('ki', 0.1, 
            ParameterDescriptor(description='积分增益'))
        self.declare_parameter('kd', 0.05, 
            ParameterDescriptor(description='微分增益'))
        self.declare_parameter('max_output', 1.0, 
            ParameterDescriptor(description='最大输出'))
        self.declare_parameter('target', 0.0, 
            ParameterDescriptor(description='目标值'))
        
        # PID 状态
        self.integral = 0.0
        self.last_error = 0.0
        
        # 订阅当前值
        self.current_sub = self.create_subscription(
            Float64, 'current_value', self.current_callback, 10)
        
        # 发布控制输出
        self.output_pub = self.create_publisher(Float64, 'control_output', 10)
        
        # 参数回调
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.get_logger().info('PID控制器已启动')
        self.log_parameters()

    def parameter_callback(self, params):
        for param in params:
            self.get_logger().info(f'参数 {param.name} 更新为 {param.value}')
        self.log_parameters()
        return SetParametersResult(successful=True)

    def log_parameters(self):
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        self.get_logger().info(f'PID参数: Kp={kp}, Ki={ki}, Kd={kd}')

    def current_callback(self, msg: Float64):
        target = self.get_parameter('target').value
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        max_output = self.get_parameter('max_output').value
        
        # 计算 PID
        error = target - msg.data
        self.integral += error
        derivative = error - self.last_error
        
        output = kp * error + ki * self.integral + kd * derivative
        
        # 限幅
        output = max(-max_output, min(max_output, output))
        
        # 发布输出
        out_msg = Float64()
        out_msg.data = output
        self.output_pub.publish(out_msg)
        
        self.last_error = error

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 5.7.2 配置文件

```yaml
pid_controller:
  ros__parameters:
    kp: 2.0
    ki: 0.5
    kd: 0.1
    max_output: 5.0
    target: 10.0
```

## 5.8 参数最佳实践

### 5.8.1 设计原则

1. **合理默认值**：提供合理的默认参数值
2. **参数描述**：为每个参数添加描述
3. **范围约束**：对数值参数设置合理范围
4. **验证逻辑**：在回调中验证参数有效性

### 5.8.2 命名规范

```python
# 推荐：使用有意义的名称
self.declare_parameter('motor_max_speed', 2.0)
self.declare_parameter('sensor_frequency', 10.0)
self.declare_parameter('controller_kp', 1.0)

# 避免：无意义或过长的名称
self.declare_parameter('param1', 2.0)
self.declare_parameter('this_is_a_very_long_parameter_name', 10.0)
```

### 5.8.3 配置管理

```yaml
# 推荐：按功能模块组织配置
robot:
  navigation:
    ros__parameters:
      max_speed: 1.0
      acceleration: 0.5
  
  sensors:
    ros__parameters:
      lidar_frequency: 10.0
      camera_resolution: [640, 480]
```

---


