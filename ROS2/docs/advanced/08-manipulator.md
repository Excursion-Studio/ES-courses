# 第8章：机械臂开发

## 8.1 机械臂运动学基础

### 8.1.1 坐标系统

机械臂开发涉及多个坐标系：

| 坐标系 | 说明 |
|--------|------|
| 世界坐标系 (World) | 全局参考坐标系 |
| 基座坐标系 (Base) | 机械臂安装位置 |
| 关节坐标系 (Joint) | 每个关节的局部坐标系 |
| 末端坐标系 (End Effector) | 机械臂末端执行器坐标系 |
| 工具坐标系 (Tool) | 工具尖端坐标系 |

### 8.1.2 正运动学与逆运动学

```
正运动学 (Forward Kinematics):
    关节角度 → 末端位姿
    θ₁, θ₂, ... θₙ → (x, y, z, roll, pitch, yaw)

逆运动学 (Inverse Kinematics):
    末端位姿 → 关节角度
    (x, y, z, roll, pitch, yaw) → θ₁, θ₂, ... θₙ
```

### 8.1.3 DH 参数

Denavit-Hartenberg 参数用于描述机械臂运动学：

| 参数 | 说明 |
|------|------|
| θ (theta) | 关节角度 |
| d | 连杆偏移 |
| a | 连杆长度 |
| α (alpha) | 连杆扭转角 |

## 8.2 URDF 模型创建

### 8.2.1 URDF 基本结构

```xml
<?xml version="1.0"?>
<robot name="my_robot_arm">
  
  <!-- 材料 -->
  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>
  
  <!-- 基座连杆 -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  
  <!-- 连杆1 -->
  <link name="link1">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <origin xyz="0 0 0.15"/>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <origin xyz="0 0 0.15"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0.15"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  
  <!-- 关节1 -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>
  
</robot>
```

### 8.2.2 6自由度机械臂 URDF

```xml
<?xml version="1.0"?>
<robot name="arm_6dof">
  
  <!-- 材料 -->
  <material name="orange"><color rgba="1 0.5 0 1"/></material>
  <material name="grey"><color rgba="0.5 0.5 0.5 1"/></material>
  
  <!-- 基座 -->
  <link name="base_link">
    <visual>
      <geometry><cylinder radius="0.08" length="0.05"/></geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry><cylinder radius="0.08" length="0.05"/></geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  
  <!-- 连杆1 - 肩部旋转 -->
  <link name="link1">
    <visual>
      <geometry><cylinder radius="0.04" length="0.15"/></geometry>
      <origin xyz="0 0 0.075"/>
      <material name="orange"/>
    </visual>
    <collision>
      <geometry><cylinder radius="0.04" length="0.15"/></geometry>
      <origin xyz="0 0 0.075"/>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 0.075"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="50" velocity="2.0"/>
  </joint>
  
  <!-- 连杆2 - 肩部俯仰 -->
  <link name="link2">
    <visual>
      <geometry><box size="0.04 0.04 0.25"/></geometry>
      <origin xyz="0 0 0.125"/>
      <material name="orange"/>
    </visual>
    <collision>
      <geometry><box size="0.04 0.04 0.25"/></geometry>
      <origin xyz="0 0 0.125"/>
    </collision>
    <inertial>
      <mass value="0.6"/>
      <origin xyz="0 0 0.125"/>
      <inertia ixx="0.004" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.15"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/>
  </joint>
  
  <!-- 连杆3 - 肘部 -->
  <link name="link3">
    <visual>
      <geometry><box size="0.04 0.04 0.20"/></geometry>
      <origin xyz="0 0 0.10"/>
      <material name="orange"/>
    </visual>
    <collision>
      <geometry><box size="0.04 0.04 0.20"/></geometry>
      <origin xyz="0 0 0.10"/>
    </collision>
    <inertial>
      <mass value="0.4"/>
      <origin xyz="0 0 0.10"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.0005"/>
    </inertial>
  </link>
  
  <joint name="joint3" type="revolute">
    <parent link="link2"/>
    <child link="link3"/>
    <origin xyz="0 0 0.25"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="2.5" effort="30" velocity="2.0"/>
  </joint>
  
  <!-- 连杆4 - 腕部旋转 -->
  <link name="link4">
    <visual>
      <geometry><cylinder radius="0.03" length="0.08"/></geometry>
      <origin xyz="0 0 0.04"/>
      <material name="orange"/>
    </visual>
    <collision>
      <geometry><cylinder radius="0.03" length="0.08"/></geometry>
      <origin xyz="0 0 0.04"/>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <origin xyz="0 0 0.04"/>
      <inertia ixx="0.0003" ixy="0" ixz="0" iyy="0.0003" iyz="0" izz="0.0001"/>
    </inertial>
  </link>
  
  <joint name="joint4" type="revolute">
    <parent link="link3"/>
    <child link="link4"/>
    <origin xyz="0 0 0.20"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="20" velocity="2.0"/>
  </joint>
  
  <!-- 连杆5 - 腕部俯仰 -->
  <link name="link5">
    <visual>
      <geometry><cylinder radius="0.025" length="0.06"/></geometry>
      <origin xyz="0 0 0.03"/>
      <material name="orange"/>
    </visual>
    <collision>
      <geometry><cylinder radius="0.025" length="0.06"/></geometry>
      <origin xyz="0 0 0.03"/>
    </collision>
    <inertial>
      <mass value="0.15"/>
      <origin xyz="0 0 0.03"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.00005"/>
    </inertial>
  </link>
  
  <joint name="joint5" type="revolute">
    <parent link="link4"/>
    <child link="link5"/>
    <origin xyz="0 0 0.08"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="15" velocity="2.0"/>
  </joint>
  
  <!-- 连杆6 - 腕部滚转 -->
  <link name="link6">
    <visual>
      <geometry><cylinder radius="0.02" length="0.04"/></geometry>
      <origin xyz="0 0 0.02"/>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry><cylinder radius="0.02" length="0.04"/></geometry>
      <origin xyz="0 0 0.02"/>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0.02"/>
      <inertia ixx="0.00005" ixy="0" ixz="0" iyy="0.00005" iyz="0" izz="0.00002"/>
    </inertial>
  </link>
  
  <joint name="joint6" type="revolute">
    <parent link="link5"/>
    <child link="link6"/>
    <origin xyz="0 0 0.06"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="2.0"/>
  </joint>
  
  <!-- 末端执行器 -->
  <link name="end_effector"/>
  
  <joint name="end_effector_joint" type="fixed">
    <parent link="link6"/>
    <child link="end_effector"/>
    <origin xyz="0 0 0.04"/>
  </joint>
  
</robot>
```

### 8.2.3 使用 Xacro 简化

```xml
<?xml version="1.0"?>
<robot name="arm_6dof" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <!-- 可复用的宏 -->
  <xacro:macro name="arm_link" params="name length radius color">
    <link name="${name}">
      <visual>
        <geometry><cylinder radius="${radius}" length="${length}"/></geometry>
        <origin xyz="0 0 ${length/2}"/>
        <material name="${color}"/>
      </visual>
      <collision>
        <geometry><cylinder radius="${radius}" length="${length}"/></geometry>
        <origin xyz="0 0 ${length/2}"/>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <origin xyz="0 0 ${length/2}"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
      </inertial>
    </link>
  </xacro:macro>
  
  <xacro:macro name="arm_joint" params="name parent child origin_z axis lower upper">
    <joint name="${name}" type="revolute">
      <parent link="${parent}"/>
      <child link="${child}"/>
      <origin xyz="0 0 ${origin_z}"/>
      <axis xyz="${axis}"/>
      <limit lower="${lower}" upper="${upper}" effort="50" velocity="2.0"/>
    </joint>
  </xacro:macro>
  
  <!-- 使用宏创建机械臂 -->
  <xacro:arm_link name="link1" length="0.15" radius="0.04" color="orange"/>
  <xacro:arm_joint name="joint1" parent="base_link" child="link1" 
                   origin_z="0.05" axis="0 0 1" lower="-3.14" upper="3.14"/>
  
</robot>
```

## 8.3 MoveIt2 运动规划框架

### 8.3.1 MoveIt2 架构

<div align="center">
<svg width="650" height="450" viewBox="0 0 650 450" xmlns="http://www.w3.org/2000/svg">
  <defs>
    <marker id="arrowhead" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
      <polygon points="0 0, 10 3.5, 0 7" fill="#495057"/>
    </marker>
  </defs>

  <!-- 背景 -->
  <rect x="10" y="10" width="630" height="430" fill="#f8f9fa" stroke="#dee2e6" stroke-width="1" rx="4"/>
  <text x="325" y="38" font-family="system-ui, -apple-system, sans-serif" font-size="18" font-weight="600" text-anchor="middle" fill="#212529">MoveIt2 架构</text>

  <!-- 用户接口层 -->
  <rect x="40" y="55" width="120" height="60" fill="#e3f2fd" stroke="#1976d2" stroke-width="2" rx="5"/>
  <text x="100" y="80" font-family="system-ui, -apple-system, sans-serif" font-size="13" font-weight="600" text-anchor="middle" fill="#1565c0">用户接口</text>
  <text x="100" y="100" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#757575">C++ API</text>

  <rect x="185" y="55" width="120" height="60" fill="#e3f2fd" stroke="#1976d2" stroke-width="2" rx="5"/>
  <text x="245" y="80" font-family="system-ui, -apple-system, sans-serif" font-size="13" font-weight="600" text-anchor="middle" fill="#1565c0">RViz插件</text>
  <text x="245" y="100" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#757575">可视化界面</text>

  <rect x="330" y="55" width="120" height="60" fill="#e3f2fd" stroke="#1976d2" stroke-width="2" rx="5"/>
  <text x="390" y="80" font-family="system-ui, -apple-system, sans-serif" font-size="13" font-weight="600" text-anchor="middle" fill="#1565c0">Python API</text>
  <text x="390" y="100" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#757575">脚本接口</text>

  <rect x="475" y="55" width="120" height="60" fill="#e3f2fd" stroke="#1976d2" stroke-width="2" rx="5"/>
  <text x="535" y="80" font-family="system-ui, -apple-system, sans-serif" font-size="13" font-weight="600" text-anchor="middle" fill="#1565c0">ROS2 服务</text>
  <text x="535" y="100" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#757575">Action/Topic</text>

  <!-- 连接线到 MoveGroup -->
  <line x1="100" y1="115" x2="245" y2="140" stroke="#495057" stroke-width="1.5" stroke-dasharray="4,2"/>
  <line x1="245" y1="115" x2="325" y2="140" stroke="#495057" stroke-width="1.5" stroke-dasharray="4,2"/>
  <line x1="390" y1="115" x2="325" y2="140" stroke="#495057" stroke-width="1.5" stroke-dasharray="4,2"/>
  <line x1="535" y1="115" x2="405" y2="140" stroke="#495057" stroke-width="1.5" stroke-dasharray="4,2"/>

  <!-- MoveGroup 核心 -->
  <rect x="60" y="150" width="530" height="70" fill="#7b1fa2" stroke="#4a148c" stroke-width="2" rx="5"/>
  <text x="325" y="180" font-family="system-ui, -apple-system, sans-serif" font-size="16" font-weight="600" text-anchor="middle" fill="#ffffff">MoveGroup (核心组件)</text>
  <text x="325" y="205" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#e1bee7">运动规划、场景管理、轨迹执行</text>

  <!-- 功能模块层 -->
  <rect x="60" y="240" width="160" height="80" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="2" rx="5"/>
  <text x="140" y="265" font-family="system-ui, -apple-system, sans-serif" font-size="13" font-weight="600" text-anchor="middle" fill="#6a1b9a">运动规划</text>
  <text x="140" y="285" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#424242">Planning</text>
  <text x="140" y="305" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#757575">路径生成</text>

  <rect x="245" y="240" width="160" height="80" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="2" rx="5"/>
  <text x="325" y="265" font-family="system-ui, -apple-system, sans-serif" font-size="13" font-weight="600" text-anchor="middle" fill="#6a1b9a">场景规划</text>
  <text x="325" y="285" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#424242">Scene</text>
  <text x="325" y="305" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#757575">环境建模</text>

  <rect x="430" y="240" width="160" height="80" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="2" rx="5"/>
  <text x="510" y="265" font-family="system-ui, -apple-system, sans-serif" font-size="13" font-weight="600" text-anchor="middle" fill="#6a1b9a">轨迹执行</text>
  <text x="510" y="285" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#424242">Execution</text>
  <text x="510" y="305" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#757575">控制器接口</text>

  <!-- 连接线 -->
  <line x1="140" y1="220" x2="140" y2="240" stroke="#495057" stroke-width="1.5" stroke-dasharray="4,2"/>
  <line x1="325" y1="220" x2="325" y2="240" stroke="#495057" stroke-width="1.5" stroke-dasharray="4,2"/>
  <line x1="510" y1="220" x2="510" y2="240" stroke="#495057" stroke-width="1.5" stroke-dasharray="4,2"/>

  <!-- 底层实现层 -->
  <rect x="60" y="340" width="160" height="80" fill="#fff3e0" stroke="#f57c00" stroke-width="2" rx="5"/>
  <text x="140" y="365" font-family="system-ui, -apple-system, sans-serif" font-size="13" font-weight="600" text-anchor="middle" fill="#ef6c00">规划器库</text>
  <text x="140" y="385" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#424242">OMPL / Pilz</text>
  <text x="140" y="405" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#757575">运动学求解</text>

  <rect x="245" y="340" width="160" height="80" fill="#fff3e0" stroke="#f57c00" stroke-width="2" rx="5"/>
  <text x="325" y="365" font-family="system-ui, -apple-system, sans-serif" font-size="13" font-weight="600" text-anchor="middle" fill="#ef6c00">碰撞检测</text>
  <text x="325" y="385" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#424242">FCL / Bullet</text>
  <text x="325" y="405" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#757575">安全检测</text>

  <rect x="430" y="340" width="160" height="80" fill="#fff3e0" stroke="#f57c00" stroke-width="2" rx="5"/>
  <text x="510" y="365" font-family="system-ui, -apple-system, sans-serif" font-size="13" font-weight="600" text-anchor="middle" fill="#ef6c00">控制器</text>
  <text x="510" y="385" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#424242">Controller</text>
  <text x="510" y="405" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#757575">ros2_control</text>

  <!-- 连接线 -->
  <line x1="140" y1="320" x2="140" y2="340" stroke="#495057" stroke-width="1.5" stroke-dasharray="4,2"/>
  <line x1="325" y1="320" x2="325" y2="340" stroke="#495057" stroke-width="1.5" stroke-dasharray="4,2"/>
  <line x1="510" y1="320" x2="510" y2="340" stroke="#495057" stroke-width="1.5" stroke-dasharray="4,2"/>
</svg>
</div>

### 8.3.2 安装 MoveIt2

```bash
# 安装 MoveIt2
sudo apt install ros-humble-moveit

# 安装 MoveIt2 可视化工具
sudo apt install ros-humble-moveit-ros-visualization
```

### 8.3.3 配置 MoveIt2

使用 MoveIt Setup Assistant：

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

配置步骤：
1. 加载 URDF 模型
2. 生成碰撞矩阵
3. 定义规划组
4. 设置末端执行器
5. 配置控制器
6. 生成配置文件

### 8.3.4 MoveIt2 Python 接口

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from moveit.planning import PlanningComponent

class MoveItArmController(Node):
    def __init__(self):
        super().__init__('moveit_arm_controller')
        
        # 初始化 MoveItPy
        self.moveit = MoveItPy(node_name="moveit_py")
        self.arm = self.moveit.get_planning_component("arm")
        self.robot_model = self.moveit.get_robot_model()
        self.planning_scene_monitor = self.moveit.get_planning_scene_monitor()
        
        self.get_logger().info('MoveIt2 机械臂控制器已启动')

    def move_to_pose(self, pose):
        """移动到指定位姿"""
        self.arm.set_start_state_to_current_state()
        
        # 设置目标位姿
        robot_state = RobotState(self.robot_model)
        robot_state.set_pose_target(pose, "end_effector")
        self.arm.set_goal_state(robot_state=robot_state)
        
        # 规划轨迹
        plan_result = self.arm.plan()
        
        if plan_result:
            # 执行轨迹
            self.moveit.execute(plan_result.trajectory, controllers=[])
            self.get_logger().info('轨迹执行完成')
            return True
        else:
            self.get_logger().error('规划失败')
            return False

    def move_to_joint_positions(self, joint_positions):
        """移动到指定关节位置"""
        self.arm.set_start_state_to_current_state()
        
        robot_state = RobotState(self.robot_model)
        robot_state.set_joint_group_positions("arm", joint_positions)
        self.arm.set_goal_state(robot_state=robot_state)
        
        plan_result = self.arm.plan()
        
        if plan_result:
            self.moveit.execute(plan_result.trajectory, controllers=[])
            return True
        return False

def main(args=None):
    rclpy.init(args=args)
    controller = MoveItArmController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 8.4 轨迹规划与执行

### 8.4.1 轨迹规划类型

| 规划器 | 特点 | 适用场景 |
|--------|------|----------|
| RRT | 快速随机树 | 通用场景 |
| RRTConnect | 双向RRT | 快速规划 |
| PRM | 概率路图 | 多次查询 |
| Pilz | 确定性规划 | 工业应用 |
| LERP | 线性插值 | 简单运动 |

### 8.4.2 轨迹规划示例

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy, PlanningComponent
from geometry_msgs.msg import PoseStamped
import math

class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('trajectory_planner')
        
        self.moveit = MoveItPy(node_name="moveit_py")
        self.arm = self.moveit.get_planning_component("arm")
        
    def plan_cartesian_path(self, waypoints):
        """规划笛卡尔路径"""
        self.arm.set_start_state_to_current_state()
        
        # 设置路径约束
        plan_result = self.arm.plan(
            multi_plan_parameters=waypoints,
            planner_id="RRTConnect"
        )
        
        return plan_result

    def execute_linear_motion(self, start_pose, end_pose, steps=10):
        """执行直线运动"""
        waypoints = []
        
        for i in range(steps + 1):
            t = i / steps
            pose = PoseStamped()
            pose.header.frame_id = "base_link"
            
            # 线性插值
            pose.pose.position.x = start_pose.pose.position.x + \
                t * (end_pose.pose.position.x - start_pose.pose.position.x)
            pose.pose.position.y = start_pose.pose.position.y + \
                t * (end_pose.pose.position.y - start_pose.pose.position.y)
            pose.pose.position.z = start_pose.pose.position.z + \
                t * (end_pose.pose.position.z - start_pose.pose.position.z)
            
            # 保持姿态不变
            pose.pose.orientation = start_pose.pose.orientation
            
            waypoints.append(pose)
        
        return self.plan_cartesian_path(waypoints)

    def plan_circular_motion(self, center, radius, angle_range, steps=20):
        """规划圆弧运动"""
        waypoints = []
        
        for i in range(steps + 1):
            angle = angle_range[0] + (angle_range[1] - angle_range[0]) * i / steps
            
            pose = PoseStamped()
            pose.header.frame_id = "base_link"
            pose.pose.position.x = center.x + radius * math.cos(angle)
            pose.pose.position.y = center.y + radius * math.sin(angle)
            pose.pose.position.z = center.z
            
            waypoints.append(pose)
        
        return self.plan_cartesian_path(waypoints)

def main(args=None):
    rclpy.init(args=args)
    planner = TrajectoryPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 8.5 抓取与放置操作

### 8.5.1 抓取动作定义

```
# PickAndPlace.action

# 目标
geometry_msgs/PoseStamped target_pose    # 目标物体位姿
geometry_msgs/PoseStamped place_pose     # 放置位置
float64 approach_distance                # 接近距离
float64 retreat_distance                 # 退出距离
---
# 结果
bool success
string message
---
# 反馈
string current_stage                     # 当前阶段
float64 progress                         # 进度
```

### 8.5.2 抓取流程

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from my_interfaces.action import PickAndPlace

class PickAndPlaceServer(Node):
    def __init__(self):
        super().__init__('pick_and_place_server')
        
        self._action_server = ActionServer(
            self,
            PickAndPlace,
            'pick_and_place',
            self.execute_callback
        )
        
        self.get_logger().info('抓取放置服务器已启动')

    def execute_callback(self, goal_handle):
        target = goal_handle.request.target_pose
        place = goal_handle.request.place_pose
        approach = goal_handle.request.approach_distance
        retreat = goal_handle.request.retreat_distance
        
        feedback = PickAndPlace.Feedback()
        result = PickAndPlace.Result()
        
        # 阶段1: 移动到接近位置
        feedback.current_stage = "approaching"
        feedback.progress = 0.0
        goal_handle.publish_feedback(feedback)
        self.move_to_approach_pose(target, approach)
        
        # 阶段2: 下降到目标
        feedback.current_stage = "descending"
        feedback.progress = 25.0
        goal_handle.publish_feedback(feedback)
        self.move_to_target(target)
        
        # 阶段3: 闭合夹爪
        feedback.current_stage = "gripping"
        feedback.progress = 50.0
        goal_handle.publish_feedback(feedback)
        self.close_gripper()
        
        # 阶段4: 提升物体
        feedback.current_stage = "lifting"
        feedback.progress = 65.0
        goal_handle.publish_feedback(feedback)
        self.lift_object(target, retreat)
        
        # 阶段5: 移动到放置位置
        feedback.current_stage = "moving"
        feedback.progress = 80.0
        goal_handle.publish_feedback(feedback)
        self.move_to_place(place)
        
        # 阶段6: 放置物体
        feedback.current_stage = "placing"
        feedback.progress = 90.0
        goal_handle.publish_feedback(feedback)
        self.open_gripper()
        
        # 完成
        feedback.current_stage = "completed"
        feedback.progress = 100.0
        goal_handle.publish_feedback(feedback)
        
        goal_handle.succeed()
        result.success = True
        result.message = "抓取放置完成"
        
        return result

    def move_to_approach_pose(self, target, distance):
        self.get_logger().info('移动到接近位置')
        # 实现移动逻辑

    def move_to_target(self, target):
        self.get_logger().info('移动到目标位置')
        # 实现移动逻辑

    def close_gripper(self):
        self.get_logger().info('闭合夹爪')
        # 发布夹爪控制命令

    def lift_object(self, target, distance):
        self.get_logger().info('提升物体')
        # 实现提升逻辑

    def move_to_place(self, place):
        self.get_logger().info('移动到放置位置')
        # 实现移动逻辑

    def open_gripper(self):
        self.get_logger().info('打开夹爪')
        # 发布夹爪控制命令

def main(args=None):
    rclpy.init(args=args)
    server = PickAndPlaceServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 8.6 实践项目：6自由度机械臂控制

### 8.6.1 项目结构

```
robot_arm_project/
├── robot_arm_description/
│   ├── urdf/
│   │   └── arm_6dof.urdf.xacro
│   ├── meshes/
│   └── launch/
│       └── display.launch.py
├── robot_arm_moveit_config/
│   ├── config/
│   │   ├── kinematics.yaml
│   │   └── ompl_planning.yaml
│   └── launch/
│       └── move_group.launch.py
├── robot_arm_control/
│   ├── src/
│   │   ├── arm_controller.py
│   │   └── gripper_controller.py
│   └── launch/
│       └── control.launch.py
└── robot_arm_application/
    └── src/
        └── pick_and_place.py
```

### 8.6.2 启动文件

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # URDF 文件路径
    urdf_file = os.path.join(
        get_package_share_directory('robot_arm_description'),
        'urdf', 'arm_6dof.urdf'
    )
    
    # MoveIt2 配置
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('robot_arm_moveit_config'),
                'launch', 'move_group.launch.py'
            )
        ])
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('robot_arm_moveit_config'),
            'config', 'moveit.rviz'
        )]
    )
    
    # 控制器节点
    controller_node = Node(
        package='robot_arm_control',
        executable='arm_controller',
        output='screen'
    )
    
    return LaunchDescription([
        moveit_launch,
        rviz_node,
        controller_node
    ])
```

## 8.7 机械臂开发最佳实践

### 8.7.1 安全考虑

1. **速度限制**：设置合理的关节速度限制
2. **工作空间限制**：定义安全工作区域
3. **碰撞检测**：启用碰撞检测功能
4. **急停机制**：实现紧急停止功能

### 8.7.2 性能优化

1. **运动学缓存**：缓存运动学计算结果
2. **轨迹平滑**：使用轨迹平滑算法
3. **实时控制**：使用实时操作系统

### 8.7.3 调试技巧

```bash
# 查看机械臂状态
ros2 topic echo /joint_states

# 查看规划场景
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene

# 检查运动学
ros2 run moveit_kinematics test_kinematics
```

---


