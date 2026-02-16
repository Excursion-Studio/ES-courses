# 第11章：仿真与可视化

## 11.1 Gazebo 仿真环境

### 11.1.1 Gazebo 简介

Gazebo 是一款强大的机器人仿真软件，支持：

- 物理引擎（ODE、Bullet、Simbody）
- 多种传感器仿真
- 插件扩展
- ROS2 集成

### 11.1.2 安装 Gazebo

```bash
# 安装 Gazebo Harmonic (推荐 ROS2 Humble)
sudo apt install ros-humble-gazebo-ros-pkgs

# 安装 Gazebo ROS2 控制
sudo apt install ros-humble-gazebo-ros2-control

# 安装 Gazebo 插件
sudo apt install ros-humble-gazebo-plugins
```

### 11.1.3 启动 Gazebo

```bash
# 启动空世界
ros2 launch gazebo_ros gazebo.launch.py

# 启动带参数
ros2 launch gazebo_ros gazebo.launch.py world:=empty.world

# 直接运行
gazebo
```

### 11.1.4 创建自定义世界

创建文件 `worlds/my_world.world`：

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="my_world">
    <!-- 物理设置 -->
    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <gravity>0 0 -9.81</gravity>
    </physics>
    
    <!-- 场景设置 -->
    <scene>
      <ambient>0.4 0.4 0.4 1.0</ambient>
      <background>0.7 0.7 0.7 1.0</background>
      <shadows>true</shadows>
    </scene>
    
    <!-- 太阳 -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- 地面 -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- 建筑物 -->
    <model name="building_1">
      <pose>10 5 2.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>5 5 5</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>5 5 5</size></box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- 墙壁 -->
    <model name="wall_1">
      <pose>0 10 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>20 0.2 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>20 0.2 2</size></box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### 11.1.5 Launch 文件启动

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    world_file = os.path.join(
        os.path.dirname(__file__), '..', 'worlds', 'my_world.world'
    )
    
    return LaunchDescription([
        # 启动 Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so'],
            output='screen'
        ),
        
        # 生成机器人
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'my_robot',
                '-file', '/path/to/robot.urdf',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1'
            ],
            output='screen'
        )
    ])
```

## 11.2 RViz2 可视化

### 11.2.1 RViz2 简介

RViz2 是 ROS2 的 3D 可视化工具，支持：

- 传感器数据可视化
- 机器人模型显示
- 地图和路径显示
- 坐标变换可视化

### 11.2.2 启动 RViz2

```bash
# 启动 RViz2
rviz2

# 加载配置文件
rviz2 -d /path/to/config.rviz
```

### 11.2.3 RViz2 配置文件

创建 `config/my_config.rviz`：

```yaml
Panels:
  - Class: rviz_common/Displays
  - Class: rviz_common/Views
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Name: Grid
      Reference Frame: <Fixed Frame>
      Plane Cell Count: 10
      
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Description Topic: /robot_description
      Name: RobotModel
      Visual Enabled: true
      
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Class: rviz_default_plugins/LaserScan
      Color: 255; 0; 0
      Name: LaserScan
      Size (m): 0.05
      Topic: /scan
      
    - Alpha: 0.7
      Class: rviz_default_plugins/Map
      Name: Map
      Topic: /map
      
    - Alpha: 1
      Class: rviz_default_plugins/TF
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      
    - Alpha: 1
      Buffer Length: 1
      Class: rviz_default_plugins/Path
      Color: 25; 255; 0
      Name: Path
      Topic: /plan
      
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
    
  Views:
    Current:
      Class: rviz_default_plugins/TopDownOrtho
      Enable Stereo Rendering: true
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Angle: 0
      Scale: 50
      Target Frame: base_link
      Value: TopDownOrtho (rviz_default_plugins)
      X: 0
      Y: 0
Window Geometry:
  Displays:
    collapsed: false
  Views:
    collapsed: false
```

### 11.2.4 RViz2 Marker 可视化

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        
        self.timer = self.create_timer(1.0, self.publish_markers)
        self.counter = 0
        
        self.get_logger().info('Marker发布器已启动')

    def publish_markers(self):
        # 发布单个箭头
        arrow = Marker()
        arrow.header.frame_id = 'map'
        arrow.header.stamp = self.get_clock().now().to_msg()
        arrow.ns = 'arrow'
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        
        start = Point(x=0.0, y=0.0, z=0.0)
        end = Point(x=1.0, y=1.0, z=0.0)
        arrow.points = [start, end]
        
        arrow.scale.x = 0.1
        arrow.scale.y = 0.2
        arrow.scale.z = 0.0
        
        arrow.color.r = 1.0
        arrow.color.g = 0.0
        arrow.color.b = 0.0
        arrow.color.a = 1.0
        
        self.marker_pub.publish(arrow)
        
        # 发布 Marker Array (多个球体)
        marker_array = MarkerArray()
        
        for i in range(5):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'spheres'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            angle = i * 2 * math.pi / 5 + self.counter * 0.1
            marker.pose.position.x = 2.0 * math.cos(angle)
            marker.pose.position.y = 2.0 * math.sin(angle)
            marker.pose.position.z = 0.5
            
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            marker.color.r = float(i) / 5.0
            marker.color.g = 1.0 - float(i) / 5.0
            marker.color.b = 0.5
            marker.color.a = 1.0
            
            marker_array.markers.append(marker)
        
        self.marker_array_pub.publish(marker_array)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    publisher = MarkerPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 11.2.5 Marker 类型

| 类型 | 说明 | 用途 |
|------|------|------|
| ARROW | 箭头 | 方向指示 |
| CUBE | 立方体 | 物体表示 |
| SPHERE | 球体 | 点位标记 |
| CYLINDER | 圆柱 | 柱状物体 |
| LINE_STRIP | 连续线 | 路径显示 |
| LINE_LIST | 线段列表 | 多条线段 |
| CUBE_LIST | 立方体列表 | 点云表示 |
| SPHERE_LIST | 球体列表 | 多点标记 |
| POINTS | 点集 | 点云数据 |
| TEXT_VIEW_FACING | 文字 | 标签显示 |
| MESH_RESOURCE | 网格模型 | 复杂模型 |
| TRIANGLE_LIST | 三角形列表 | 自定义形状 |

## 11.3 自定义插件开发

### 11.3.1 Gazebo 插件结构

```cpp
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace gazebo
{
class MyPlugin : public ModelPlugin
{
public:
    MyPlugin() : ModelPlugin()
    {
        printf("MyPlugin loaded\n");
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // 保存模型指针
        this->model = _parent;
        
        // 获取参数
        if (_sdf->HasElement("robotNamespace"))
        {
            this->namespace_ = _sdf->Get<std::string>("robotNamespace");
        }
        
        // 初始化 ROS 节点
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_plugin",
                      ros::init_options::NoSigintHandler);
        }
        
        this->ros_node.reset(new ros::NodeHandle(this->namespace_));
        
        // 创建订阅者
        this->ros_sub = this->ros_node->subscribe<std_msgs::Float64>(
            "command", 1,
            std::bind(&MyPlugin::OnRosMsg, this, std::placeholders::_1));
        
        // 创建发布者
        this->ros_pub = this->ros_node->advertise<std_msgs::Float64>(
            "state", 1);
        
        // 连接更新事件
        this->update_connection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&MyPlugin::OnUpdate, this));
    }

    void OnUpdate()
    {
        // 每个仿真步调用
        // 发布状态
        std_msgs::Float64 msg;
        msg.data = this->model->WorldPose().Pos().X();
        this->ros_pub.publish(msg);
    }

    void OnRosMsg(const std_msgs::Float64::ConstPtr &_msg)
    {
        // 处理 ROS 消息
        this->model->SetLinearVel(ignition::math::Vector3d(
            _msg->data, 0, 0));
    }

private:
    physics::ModelPtr model;
    std::string namespace_;
    std::unique_ptr<ros::NodeHandle> ros_node;
    ros::Subscriber ros_sub;
    ros::Publisher ros_pub;
    event::ConnectionPtr update_connection;
};

GZ_REGISTER_MODEL_PLUGIN(MyPlugin)
}
```

### 11.3.2 在 URDF 中使用插件

```xml
<gazebo>
    <plugin name="my_plugin" filename="libmy_plugin.so">
        <robotNamespace>my_robot</robotNamespace>
    </plugin>
</gazebo>
```

### 11.3.3 CMakeLists.txt 配置

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_gazebo_plugins)

find_package(gazebo REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(my_plugin SHARED src/my_plugin.cpp)
target_include_directories(my_plugin PUBLIC
    ${GAZEBO_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
)
target_link_libraries(my_plugin
    ${GAZEBO_LIBRARIES}
    ${rclcpp_LIBRARIES}
)

ament_target_dependencies(my_plugin rclcpp std_msgs)

install(TARGETS my_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_package()
```

## 11.4 多机器人仿真

### 11.4.1 多机器人 Launch 文件

```python
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf', 'robot.urdf'
    )
    
    robots = [
        {'name': 'robot1', 'x': 0.0, 'y': 0.0},
        {'name': 'robot2', 'x': 2.0, 'y': 0.0},
        {'name': 'robot3', 'x': 4.0, 'y': 0.0},
    ]
    
    launch_entities = []
    
    for robot in robots:
        robot_group = GroupAction([
            PushRosNamespace(robot['name']),
            
            # 发布机器人描述
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': open(urdf_file).read()}]
            ),
            
            # 在 Gazebo 中生成
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', robot['name'],
                    '-file', urdf_file,
                    '-robot_namespace', robot['name'],
                    '-x', str(robot['x']),
                    '-y', str(robot['y']),
                    '-z', '0.1'
                ],
                output='screen'
            )
        ])
        
        launch_entities.append(robot_group)
    
    return LaunchDescription(launch_entities)
```

### 11.4.2 多机器人话题命名

```
/robot1/cmd_vel
/robot1/odom
/robot1/scan
/robot2/cmd_vel
/robot2/odom
/robot2/scan
...
```

### 11.4.3 多机器人 TF 树

<div align="center">
<svg width="650" height="320" viewBox="0 0 650 320" xmlns="http://www.w3.org/2000/svg">
  <defs>
    <marker id="arrowTF" markerWidth="8" markerHeight="6" refX="7" refY="3" orient="auto">
      <polygon points="0 0, 8 3, 0 6" fill="#495057"/>
    </marker>
  </defs>
  
  <!-- 背景 -->
  <rect x="10" y="10" width="630" height="300" fill="#f8f9fa" stroke="#dee2e6" stroke-width="1" rx="4"/>
  <text x="325" y="38" font-family="system-ui, -apple-system, sans-serif" font-size="18" font-weight="600" text-anchor="middle" fill="#212529">多机器人 TF 树</text>
  
  <!-- map 根节点 -->
  <rect x="290" y="55" width="70" height="35" fill="#e3f2fd" stroke="#1976d2" stroke-width="2" rx="4"/>
  <text x="325" y="78" font-family="system-ui, -apple-system, sans-serif" font-size="13" font-weight="600" text-anchor="middle" fill="#1565c0">map</text>
  
  <!-- robot1 -->
  <rect x="50" y="130" width="120" height="35" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="2" rx="4"/>
  <text x="110" y="153" font-family="system-ui, -apple-system, sans-serif" font-size="12" font-weight="600" text-anchor="middle" fill="#6a1b9a">robot1/base_link</text>
  
  <rect x="20" y="195" width="80" height="30" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="1.5" rx="3"/>
  <text x="60" y="215" font-family="system-ui, -apple-system, sans-serif" font-size="10" font-weight="500" text-anchor="middle" fill="#6a1b9a">robot1/base_scan</text>
  
  <rect x="120" y="195" width="80" height="30" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="1.5" rx="3"/>
  <text x="160" y="215" font-family="system-ui, -apple-system, sans-serif" font-size="10" font-weight="500" text-anchor="middle" fill="#6a1b9a">robot1/base_footprint</text>
  
  <!-- robot2 -->
  <rect x="265" y="130" width="120" height="35" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="2" rx="4"/>
  <text x="325" y="153" font-family="system-ui, -apple-system, sans-serif" font-size="12" font-weight="600" text-anchor="middle" fill="#6a1b9a">robot2/base_link</text>
  
  <rect x="235" y="195" width="80" height="30" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="1.5" rx="3"/>
  <text x="275" y="215" font-family="system-ui, -apple-system, sans-serif" font-size="10" font-weight="500" text-anchor="middle" fill="#6a1b9a">robot2/base_scan</text>
  
  <rect x="335" y="195" width="80" height="30" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="1.5" rx="3"/>
  <text x="375" y="215" font-family="system-ui, -apple-system, sans-serif" font-size="10" font-weight="500" text-anchor="middle" fill="#6a1b9a">robot2/base_footprint</text>
  
  <!-- robot3 -->
  <rect x="480" y="130" width="120" height="35" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="2" rx="4"/>
  <text x="540" y="153" font-family="system-ui, -apple-system, sans-serif" font-size="12" font-weight="600" text-anchor="middle" fill="#6a1b9a">robot3/base_link</text>
  
  <rect x="450" y="195" width="80" height="30" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="1.5" rx="3"/>
  <text x="490" y="215" font-family="system-ui, -apple-system, sans-serif" font-size="10" font-weight="500" text-anchor="middle" fill="#6a1b9a">robot3/base_scan</text>
  
  <rect x="550" y="195" width="80" height="30" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="1.5" rx="3"/>
  <text x="590" y="215" font-family="system-ui, -apple-system, sans-serif" font-size="10" font-weight="500" text-anchor="middle" fill="#6a1b9a">robot3/base_footprint</text>
  
  <!-- 连接线 - robot1 -->
  <line x1="325" y1="90" x2="325" y2="110" stroke="#495057" stroke-width="1.5"/>
  <line x1="325" y1="110" x2="110" y2="110" stroke="#495057" stroke-width="1.5"/>
  <line x1="110" y1="110" x2="110" y2="130" stroke="#495057" stroke-width="1.5" marker-end="url(#arrowTF)"/>
  
  <line x1="110" y1="165" x2="110" y2="175" stroke="#495057" stroke-width="1.2"/>
  <line x1="110" y1="175" x2="60" y2="175" stroke="#495057" stroke-width="1.2"/>
  <line x1="60" y1="175" x2="60" y2="195" stroke="#495057" stroke-width="1.2" marker-end="url(#arrowTF)"/>
  
  <line x1="110" y1="175" x2="160" y2="175" stroke="#495057" stroke-width="1.2"/>
  <line x1="160" y1="175" x2="160" y2="195" stroke="#495057" stroke-width="1.2" marker-end="url(#arrowTF)"/>
  
  <!-- 连接线 - robot2 -->
  <line x1="325" y1="110" x2="325" y2="130" stroke="#495057" stroke-width="1.5" marker-end="url(#arrowTF)"/>
  
  <line x1="325" y1="165" x2="325" y2="175" stroke="#495057" stroke-width="1.2"/>
  <line x1="325" y1="175" x2="275" y2="175" stroke="#495057" stroke-width="1.2"/>
  <line x1="275" y1="175" x2="275" y2="195" stroke="#495057" stroke-width="1.2" marker-end="url(#arrowTF)"/>
  
  <line x1="325" y1="175" x2="375" y2="175" stroke="#495057" stroke-width="1.2"/>
  <line x1="375" y1="175" x2="375" y2="195" stroke="#495057" stroke-width="1.2" marker-end="url(#arrowTF)"/>
  
  <!-- 连接线 - robot3 -->
  <line x1="325" y1="110" x2="540" y2="110" stroke="#495057" stroke-width="1.5"/>
  <line x1="540" y1="110" x2="540" y2="130" stroke="#495057" stroke-width="1.5" marker-end="url(#arrowTF)"/>
  
  <line x1="540" y1="165" x2="540" y2="175" stroke="#495057" stroke-width="1.2"/>
  <line x1="540" y1="175" x2="490" y2="175" stroke="#495057" stroke-width="1.2"/>
  <line x1="490" y1="175" x2="490" y2="195" stroke="#495057" stroke-width="1.2" marker-end="url(#arrowTF)"/>
  
  <line x1="540" y1="175" x2="590" y2="175" stroke="#495057" stroke-width="1.2"/>
  <line x1="590" y1="175" x2="590" y2="195" stroke="#495057" stroke-width="1.2" marker-end="url(#arrowTF)"/>
</svg>
</div>

## 11.5 传感器仿真

### 11.5.1 激光雷达仿真

```xml
<gazebo reference="laser_link">
    <sensor type="ray" name="laser">
        <pose>0 0 0.05 0 0 0</pose>
        <visualize>true</visualize>
        <update_rate>10</update_rate>
        <ray>
            <scan>
                <horizontal>
                    <samples>360</samples>
                    <resolution>1</resolution>
                    <min_angle>-3.14159</min_angle>
                    <max_angle>3.14159</max_angle>
                </horizontal>
            </scan>
            <range>
                <min>0.1</min>
                <max>10.0</max>
                <resolution>0.01</resolution>
            </range>
            <noise>
                <type>gaussian</type>
                <mean>0.0</mean>
                <stddev>0.01</stddev>
            </noise>
        </ray>
        <plugin name="laser_plugin" filename="libgazebo_ros_ray_sensor.so">
            <ros>
                <namespace>/</namespace>
                <remapping>~/out:=scan</remapping>
            </ros>
            <output_type>sensor_msgs/LaserScan</output_type>
            <frame_name>laser_link</frame_name>
        </plugin>
    </sensor>
</gazebo>
```

### 11.5.2 相机仿真

```xml
<gazebo reference="camera_link">
    <sensor type="camera" name="camera">
        <update_rate>30.0</update_rate>
        <camera>
            <horizontal_fov>1.3962634</horizontal_fov>
            <image>
                <width>640</width>
                <height>480</height>
                <format>R8G8B8</format>
            </image>
            <clip>
                <near>0.02</near>
                <far>300</far>
            </clip>
            <noise>
                <type>gaussian</type>
                <mean>0.0</mean>
                <stddev>0.007</stddev>
            </noise>
        </camera>
        <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
            <ros>
                <namespace>/</namespace>
                <remapping>image_raw:=camera/image_raw</remapping>
            </ros>
            <camera_name>camera</camera_name>
            <frame_name>camera_link</frame_name>
        </plugin>
    </sensor>
</gazebo>
```

### 11.5.3 IMU 仿真

```xml
<gazebo reference="imu_link">
    <sensor type="imu" name="imu">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <imu>
            <angular_velocity>
                <x>
                    <noise type="gaussian">
                        <mean>0.0</mean>
                        <stddev>2e-4</stddev>
                    </noise>
                </x>
                <y>
                    <noise type="gaussian">
                        <mean>0.0</mean>
                        <stddev>2e-4</stddev>
                    </noise>
                </y>
                <z>
                    <noise type="gaussian">
                        <mean>0.0</mean>
                        <stddev>2e-4</stddev>
                    </noise>
                </z>
            </angular_velocity>
            <linear_acceleration>
                <x>
                    <noise type="gaussian">
                        <mean>0.0</mean>
                        <stddev>1.7e-2</stddev>
                    </noise>
                </x>
                <y>
                    <noise type="gaussian">
                        <mean>0.0</mean>
                        <stddev>1.7e-2</stddev>
                    </noise>
                </y>
                <z>
                    <noise type="gaussian">
                        <mean>0.0</mean>
                        <stddev>1.7e-2</stddev>
                    </noise>
                </z>
            </linear_acceleration>
        </imu>
        <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
            <ros>
                <namespace>/</namespace>
                <remapping>~/out:=imu/data</remapping>
            </ros>
            <initial_orientation_as_reference>false</initial_orientation_as_reference>
            <frame_name>imu_link</frame_name>
        </plugin>
    </sensor>
</gazebo>
```

## 11.6 仿真最佳实践

### 11.6.1 性能优化

1. **简化模型**：减少网格面数
2. **调整更新频率**：根据需求设置传感器频率
3. **使用 LOD**：细节层次距离
4. **关闭不必要的可视化**

### 11.6.2 调试技巧

```bash
# 查看 Gazebo 话题
gz topic -l

# 查看 Gazebo 模型
gz model -l

# 查看仿真统计
gz stats

# 录制仿真
gz record -a

# 回放仿真
gz playback -f state.log
```

### 11.6.3 常见问题

| 问题 | 解决方案 |
|------|----------|
| 模型下沉 | 增加碰撞检测精度 |
| 传感器无数据 | 检查话题映射和 frame_id |
| 仿真卡顿 | 降低物理引擎精度 |
| 模型漂移 | 检查惯性参数设置 |

---


