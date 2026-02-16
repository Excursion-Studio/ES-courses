# 第7章：工具与调试

## 7.1 Launch 文件

Launch 文件用于同时启动多个节点，配置参数和命名空间。

### 7.1.1 Python Launch 文件

创建文件 `launch/my_launch.py`：

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间'
    )
    
    # 获取配置文件路径
    config_file = os.path.join(
        get_package_share_directory('my_package'),
        'config',
        'params.yaml'
    )
    
    # 创建节点
    node1 = Node(
        package='my_package',
        executable='publisher_node',
        name='my_publisher',
        parameters=[config_file],
        output='screen'
    )
    
    node2 = Node(
        package='my_package',
        executable='subscriber_node',
        name='my_subscriber',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/topic', '/my_topic')
        ],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        node1,
        node2
    ])
```

### 7.1.2 带命名空间的 Launch

```python
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction

def generate_launch_description():
    # 机器人1的节点组
    robot1_nodes = GroupAction(
        actions=[
            PushRosNamespace('robot1'),
            Node(
                package='my_package',
                executable='sensor_node',
                name='sensor'
            ),
            Node(
                package='my_package',
                executable='controller_node',
                name='controller'
            )
        ]
    )
    
    # 机器人2的节点组
    robot2_nodes = GroupAction(
        actions=[
            PushRosNamespace('robot2'),
            Node(
                package='my_package',
                executable='sensor_node',
                name='sensor'
            ),
            Node(
                package='my_package',
                executable='controller_node',
                name='controller'
            )
        ]
    )
    
    return LaunchDescription([
        robot1_nodes,
        robot2_nodes
    ])
```

### 7.1.3 包含其他 Launch 文件

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 包含其他包的launch文件
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'True',
            'autostart': 'True'
        }.items()
    )
    
    return LaunchDescription([
        nav2_launch
    ])
```

### 7.1.4 运行 Launch 文件

```bash
# 运行launch文件
ros2 launch my_package my_launch.py

# 传递参数
ros2 launch my_package my_launch.py use_sim_time:=true

# 指定参数文件
ros2 launch my_package my_launch.py params_file:=/path/to/params.yaml
```

## 7.2 ros2 命令行工具大全

### 7.2.1 节点命令

```bash
# 列出所有节点
ros2 node list

# 查看节点信息
ros2 node info /node_name

# 运行节点
ros2 run package_name node_name

# 运行节点并设置参数
ros2 run package_name node_name --ros-args -p param_name:=value
```

### 7.2.2 话题命令

```bash
# 列出所有话题
ros2 topic list

# 查看话题信息
ros2 topic info /topic_name

# 查看话题类型
ros2 topic type /topic_name

# 显示话题消息
ros2 topic echo /topic_name

# 发布消息
ros2 topic pub /topic_name msg_type "data"

# 查看发布频率
ros2 topic hz /topic_name

# 查看带宽
ros2 topic bw /topic_name
```

### 7.2.3 服务命令

```bash
# 列出所有服务
ros2 service list

# 查看服务类型
ros2 service type /service_name

# 调用服务
ros2 service call /service_name srv_type "request_data"

# 查找服务
ros2 service find srv_type
```

### 7.2.4 参数命令

```bash
# 列出参数
ros2 param list

# 获取参数值
ros2 param get /node_name param_name

# 设置参数值
ros2 param set /node_name param_name value

# 查看参数描述
ros2 param describe /node_name param_name

# 导出参数
ros2 param dump /node_name

# 加载参数
ros2 param load /node_name params.yaml
```

### 7.2.5 动作命令

```bash
# 列出所有动作
ros2 action list

# 查看动作信息
ros2 action info /action_name

# 发送动作目标
ros2 action send_goal /action_name action_type "goal_data"

# 发送目标并显示反馈
ros2 action send_goal --feedback /action_name action_type "goal_data"
```

### 7.2.6 包管理命令

```bash
# 列出所有包
ros2 pkg list

# 查看包路径
ros2 pkg prefix package_name

# 查看包的可执行文件
ros2 pkg executables package_name

# 创建包
ros2 pkg create --build-type ament_python new_package

# 查看包的xml信息
ros2 pkg xml package_name
```

### 7.2.7 接口命令

```bash
# 列出所有接口
ros2 interface list

# 查看接口定义
ros2 interface show msg/srv/action_type

# 查看消息
ros2 interface show std_msgs/msg/String

# 查看服务
ros2 interface show example_interfaces/srv/AddTwoInts

# 查看动作
ros2 interface show nav2_msgs/action/NavigateToPose
```

### 7.2.8 构建命令

```bash
# 构建工作空间
colcon build

# 构建特定包
colcon build --packages-select package_name

# 构建并忽略某些包
colcon build --packages-ignore package_name

# 并行构建
colcon build --parallel-workers 4

# 构建并运行测试
colcon build --cmake-args -DBUILD_TESTING=ON

# 生成symlink安装（开发模式）
colcon build --symlink-install
```

## 7.3 rqt 可视化工具

### 7.3.1 启动 rqt

```bash
# 启动rqt主界面
rqt

# 启动特定插件
rqt_graph
rqt_plot
rqt_console
rqt_image_view
```

### 7.3.2 常用插件

| 插件 | 功能 |
|------|------|
| **rqt_graph** | 显示节点和话题关系图 |
| **rqt_plot** | 实时绘制话题数据曲线 |
| **rqt_console** | 查看日志输出 |
| **rqt_image_view** | 显示图像话题 |
| **rqt_reconfigure** | 动态修改参数 |
| **rqt_service_caller** | 调用服务 |
| **rqt_publisher** | 发布话题消息 |
| **rqt_top** | 查看节点资源使用 |

### 7.3.3 rqt_graph 使用

```bash
# 启动节点图
rqt_graph

# 隐藏隐藏话题
# 在界面中取消 "Hide Debug" 选项
```

### 7.3.4 rqt_plot 使用

```bash
# 命令行方式绘制
rqt_plot /topic_name/field

# 示例：绘制速度
rqt_plot /cmd_vel/linear/x
```

## 7.4 日志与调试

### 7.4.1 日志级别

| 级别 | 说明 | 使用场景 |
|------|------|----------|
| DEBUG | 调试信息 | 开发调试 |
| INFO | 一般信息 | 正常运行状态 |
| WARN | 警告信息 | 潜在问题 |
| ERROR | 错误信息 | 功能异常 |
| FATAL | 致命错误 | 系统崩溃 |

### 7.4.2 设置日志级别

```bash
# 命令行设置
ros2 run my_package my_node --ros-args --log-level debug

# 设置特定logger
ros2 run my_package my_node --ros-args --log-level my_node:=debug
```

### 7.4.3 Python 日志使用

```python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # 不同级别的日志
        self.get_logger().debug('调试信息')
        self.get_logger().info('一般信息')
        self.get_logger().warn('警告信息')
        self.get_logger().error('错误信息')
        self.get_logger().fatal('致命错误')
        
        # 设置日志级别
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
```

### 7.4.4 C++ 日志使用

```cpp
// 不同级别的日志
RCLCPP_DEBUG(this->get_logger(), "调试信息");
RCLCPP_INFO(this->get_logger(), "一般信息");
RCLCPP_WARN(this->get_logger(), "警告信息");
RCLCPP_ERROR(this->get_logger(), "错误信息");
RCLCPP_FATAL(this->get_logger(), "致命错误");

// 带参数的日志
RCLCPP_INFO(this->get_logger(), "速度: %.2f m/s", speed);
RCLCPP_INFO(this->get_logger(), "位置: (%.2f, %.2f)", x, y);

// 条件日志
RCLCPP_INFO_EXPRESSION(this->get_logger(), count > 10, "计数超过10");
RCLCPP_INFO_ONCE(this->get_logger(), "只打印一次");
RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "每秒打印一次");
```

### 7.4.5 查看日志

```bash
# 使用rqt_console查看日志
rqt_console

# 过滤日志
ros2 topic echo /rosout --filter "msg.level == 40"  # 只显示ERROR
```

## 7.5 包管理与依赖

### 7.5.1 package.xml 详解

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_package</name>
  <version>1.0.0</version>
  <description>我的ROS2功能包</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>Apache-2.0</license>
  
  <!-- 构建工具依赖 -->
  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <!-- 构建依赖 -->
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  
  <!-- 测试依赖 -->
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  
  <!-- 导出配置 -->
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### 7.5.2 依赖类型说明

| 依赖类型 | 说明 |
|----------|------|
| `buildtool_depend` | 构建工具依赖 |
| `build_depend` | 构建时依赖 |
| `build_export_depend` | 构建导出依赖 |
| `exec_depend` | 运行时依赖 |
| `depend` | 构建和运行都依赖 |
| `test_depend` | 测试依赖 |

### 7.5.3 使用 rosdep 安装依赖

```bash
# 初始化rosdep
sudo rosdep init
rosdep update

# 安装依赖
rosdep install --from-paths src --ignore-src -y

# 跳过某些包
rosdep install --from-paths src --ignore-src -y --skip-keys="package_name"
```

## 7.6 常见调试技巧

### 7.6.1 检查环境

```bash
# 查看ROS版本
printenv | grep ROS

# 查看ROS_DISTRO
echo $ROS_DISTRO

# 查看工作空间
echo $AMENT_PREFIX_PATH
```

### 7.6.2 检查节点状态

```bash
# 查看节点是否运行
ros2 node list

# 查看节点详细信息
ros2 node info /node_name

# 检查话题连接
ros2 topic info /topic_name -v
```

### 7.6.3 网络问题排查

```bash
# 检查DDS实现
echo $RMW_IMPLEMENTATION

# 设置DDS实现
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 检查网络接口
ros2 topic info /topic_name -v
```

### 7.6.4 常见问题解决

| 问题 | 解决方案 |
|------|----------|
| 找不到包 | `source install/setup.bash` |
| 节点无法通信 | 检查DDS配置和防火墙 |
| 参数不生效 | 检查YAML格式和命名空间 |
| Launch失败 | 检查文件路径和语法 |

## 7.7 性能分析工具

### 7.7.1 检查话题频率

```bash
# 查看发布频率
ros2 topic hz /topic_name

# 查看延迟
ros2 topic delay /topic_name
```

### 7.7.2 检查带宽

```bash
# 查看话题带宽
ros2 topic bw /topic_name
```

### 7.7.3 使用 rqt_top

```bash
# 查看节点资源使用
rqt_top
```

---

**基础教程完成！**


