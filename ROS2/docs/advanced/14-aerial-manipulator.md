# 第14章：空中机械臂

## 14.1 空中机械臂概述

### 14.1.1 系统组成

<div align="center">
<svg width="650" height="400" viewBox="0 0 650 400" xmlns="http://www.w3.org/2000/svg">
  <defs>
    <marker id="arrowhead" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
      <polygon points="0 0, 10 3.5, 0 7" fill="#495057"/>
    </marker>
  </defs>

  <!-- 背景 -->
  <rect x="10" y="10" width="630" height="380" fill="#f8f9fa" stroke="#dee2e6" stroke-width="1" rx="4"/>
  <text x="325" y="38" font-family="system-ui, -apple-system, sans-serif" font-size="18" font-weight="600" text-anchor="middle" fill="#212529">空中机械臂系统</text>

  <!-- 飞行平台外框 -->
  <rect x="40" y="55" width="570" height="320" fill="#e3f2fd" stroke="#1976d2" stroke-width="2" rx="6"/>
  <text x="325" y="80" font-family="system-ui, -apple-system, sans-serif" font-size="15" font-weight="600" text-anchor="middle" fill="#1565c0">飞行平台 (UAV)</text>

  <!-- 机械臂系统 -->
  <rect x="60" y="100" width="530" height="150" fill="#ffffff" stroke="#7b1fa2" stroke-width="2" rx="5"/>
  <text x="325" y="125" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#6a1b9a">机械臂系统</text>

  <!-- 关节1 -->
  <rect x="100" y="145" width="90" height="70" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="1.5" rx="4"/>
  <text x="145" y="170" font-family="system-ui, -apple-system, sans-serif" font-size="13" font-weight="600" text-anchor="middle" fill="#6a1b9a">关节1</text>
  <text x="145" y="190" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#9c27b0">俯仰</text>
  <text x="145" y="205" font-family="system-ui, -apple-system, sans-serif" font-size="9" text-anchor="middle" fill="#757575">Pitch</text>

  <!-- 关节2 -->
  <rect x="220" y="145" width="90" height="70" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="1.5" rx="4"/>
  <text x="265" y="170" font-family="system-ui, -apple-system, sans-serif" font-size="13" font-weight="600" text-anchor="middle" fill="#6a1b9a">关节2</text>
  <text x="265" y="190" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#9c27b0">俯仰/滚转</text>
  <text x="265" y="205" font-family="system-ui, -apple-system, sans-serif" font-size="9" text-anchor="middle" fill="#757575">Pitch/Roll</text>

  <!-- 关节3 -->
  <rect x="340" y="145" width="90" height="70" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="1.5" rx="4"/>
  <text x="385" y="170" font-family="system-ui, -apple-system, sans-serif" font-size="13" font-weight="600" text-anchor="middle" fill="#6a1b9a">关节3</text>
  <text x="385" y="190" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#9c27b0">俯仰</text>
  <text x="385" y="205" font-family="system-ui, -apple-system, sans-serif" font-size="9" text-anchor="middle" fill="#757575">Pitch</text>

  <!-- 末端执行器 -->
  <rect x="460" y="145" width="110" height="70" fill="#fce4ec" stroke="#c2185b" stroke-width="2" rx="4"/>
  <text x="515" y="170" font-family="system-ui, -apple-system, sans-serif" font-size="13" font-weight="600" text-anchor="middle" fill="#ad1457">末端执行器</text>
  <text x="515" y="190" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#e91e63">夹爪/工具</text>
  <text x="515" y="205" font-family="system-ui, -apple-system, sans-serif" font-size="9" text-anchor="middle" fill="#757575">End Effector</text>

  <!-- 关节连接线 -->
  <line x1="190" y1="180" x2="220" y2="180" stroke="#7b1fa2" stroke-width="3"/>
  <line x1="310" y1="180" x2="340" y2="180" stroke="#7b1fa2" stroke-width="3"/>
  <line x1="430" y1="180" x2="460" y2="180" stroke="#c2185b" stroke-width="3"/>

  <!-- 飞控 -->
  <rect x="80" y="270" width="140" height="85" fill="#ffffff" stroke="#1976d2" stroke-width="1.5" rx="4"/>
  <text x="150" y="295" font-family="system-ui, -apple-system, sans-serif" font-size="13" font-weight="600" text-anchor="middle" fill="#1565c0">飞控系统</text>
  <rect x="95" y="310" width="110" height="35" fill="#e3f2fd" stroke="#1976d2" stroke-width="1" rx="3"/>
  <text x="150" y="330" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#1565c0">PX4</text>

  <!-- 传感器 -->
  <rect x="255" y="270" width="140" height="85" fill="#ffffff" stroke="#388e3c" stroke-width="1.5" rx="4"/>
  <text x="325" y="295" font-family="system-ui, -apple-system, sans-serif" font-size="13" font-weight="600" text-anchor="middle" fill="#2e7d32">传感器</text>
  <rect x="270" y="310" width="55" height="35" fill="#e8f5e9" stroke="#388e3c" stroke-width="1" rx="3"/>
  <text x="297" y="330" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#2e7d32">IMU</text>
  <rect x="330" y="310" width="55" height="35" fill="#e8f5e9" stroke="#388e3c" stroke-width="1" rx="3"/>
  <text x="357" y="330" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#2e7d32">GPS</text>

  <!-- 计算机 -->
  <rect x="430" y="270" width="140" height="85" fill="#ffffff" stroke="#f57c00" stroke-width="1.5" rx="4"/>
  <text x="500" y="295" font-family="system-ui, -apple-system, sans-serif" font-size="13" font-weight="600" text-anchor="middle" fill="#ef6c00">机载计算机</text>
  <rect x="445" y="310" width="110" height="35" fill="#fff3e0" stroke="#f57c00" stroke-width="1" rx="3"/>
  <text x="500" y="330" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#ef6c00">ROS2</text>
</svg>
</div>

### 14.1.2 技术挑战

| 挑战 | 描述 | 解决方案 |
|------|------|----------|
| 耦合动力学 | 机械臂运动影响飞行稳定性 | 动力学建模与补偿 |
| 负载变化 | 抓取后质量变化 | 自适应控制 |
| 精确定位 | 空中悬停精度要求高 | 视觉伺服 |
| 通信延迟 | 远程操控延迟 | 预测控制 |

### 14.1.3 应用场景

- 电力线路巡检与维护
- 高空作业与维修
- 灾害救援物资投放
- 空中采样与检测

## 14.2 系统建模

### 14.2.1 动力学模型

```python
#!/usr/bin/env python3
import numpy as np
from dataclasses import dataclass
from typing import List

@dataclass
class Link:
    """机械臂连杆参数"""
    mass: float          # 质量 (kg)
    length: float        # 长度 (m)
    com: np.ndarray      # 质心位置 [x, y, z]
    inertia: np.ndarray  # 惯性张量 (3x3)

@dataclass
class Joint:
    """关节参数"""
    type: str            # 'revolute' or 'prismatic'
    axis: np.ndarray     # 旋转轴
    lower_limit: float   # 下限
    upper_limit: float   # 上限
    damping: float       # 阻尼

class AerialManipulatorDynamics:
    """
    空中机械臂动力学模型
    """
    def __init__(self):
        # UAV 参数
        self.uav_mass = 2.0  # kg
        self.uav_inertia = np.diag([0.02, 0.02, 0.04])  # kg*m^2
        
        # 机械臂参数
        self.links = [
            Link(mass=0.2, length=0.15, 
                 com=np.array([0.075, 0, 0]),
                 inertia=np.diag([0.0001, 0.0001, 0.0001])),
            Link(mass=0.15, length=0.12,
                 com=np.array([0.06, 0, 0]),
                 inertia=np.diag([0.00005, 0.00005, 0.00005])),
            Link(mass=0.1, length=0.1,
                 com=np.array([0.05, 0, 0]),
                 inertia=np.diag([0.00003, 0.00003, 0.00003])),
        ]
        
        self.joints = [
            Joint(type='revolute', axis=np.array([0, 1, 0]),
                  lower_limit=-np.pi/2, upper_limit=np.pi/2, damping=0.1),
            Joint(type='revolute', axis=np.array([0, 1, 0]),
                  lower_limit=-np.pi/2, upper_limit=np.pi/2, damping=0.1),
            Joint(type='revolute', axis=np.array([0, 1, 0]),
                  lower_limit=-np.pi, upper_limit=np.pi, damping=0.1),
        ]
        
        # 重力
        self.g = 9.81
        
    def compute_total_mass(self, joint_angles: np.ndarray, 
                           payload_mass: float = 0.0) -> float:
        """计算总质量"""
        total = self.uav_mass + payload_mass
        for link in self.links:
            total += link.mass
        return total
    
    def compute_com(self, joint_angles: np.ndarray) -> np.ndarray:
        """计算系统质心"""
        # UAV 质心
        com = np.zeros(3)
        total_mass = self.uav_mass
        
        # 各连杆质心贡献
        for i, (link, angle) in enumerate(zip(self.links, joint_angles)):
            # 计算连杆位置
            link_pos = self.compute_link_position(i, joint_angles)
            link_com = link_pos + link.com
            
            com = (com * total_mass + link_com * link.mass) / (total_mass + link.mass)
            total_mass += link.mass
        
        return com
    
    def compute_link_position(self, link_index: int, 
                              joint_angles: np.ndarray) -> np.ndarray:
        """计算连杆位置"""
        position = np.array([0, 0, 0])  # 从 UAV 中心开始
        
        for i in range(link_index + 1):
            angle = joint_angles[i]
            length = self.links[i].length
            
            # 简化的 2D 计算
            if i == 0:
                position[0] += length * np.cos(angle)
                position[2] -= length * np.sin(angle)
            else:
                prev_angle = sum(joint_angles[:i])
                position[0] += length * np.cos(prev_angle + angle)
                position[2] -= length * np.sin(prev_angle + angle)
        
        return position
    
    def compute_reaction_forces(self, joint_angles: np.ndarray,
                                joint_velocities: np.ndarray,
                                joint_accelerations: np.ndarray) -> dict:
        """计算机械臂运动对 UAV 的反作用力"""
        # 惯性力
        inertial_force = np.zeros(3)
        inertial_torque = np.zeros(3)
        
        for i, (link, angle, vel, acc) in enumerate(
            zip(self.links, joint_angles, joint_velocities, joint_accelerations)):
            
            # 连杆位置和加速度
            link_pos = self.compute_link_position(i, joint_angles)
            
            # 线性加速度产生的惯性力
            linear_acc = acc * link.length  # 简化
            inertial_force += link.mass * linear_acc
            
            # 角加速度产生的惯性力矩
            inertial_torque += link.inertia @ np.array([0, acc, 0])
        
        return {
            'force': inertial_force,
            'torque': inertial_torque
        }
    
    def compute_gravity_compensation(self, joint_angles: np.ndarray) -> np.ndarray:
        """计算重力补偿力矩"""
        torques = np.zeros(len(joint_angles))
        
        for i in range(len(joint_angles)):
            # 计算从该关节到末端的所有质量
            total_moment = 0.0
            
            for j in range(i, len(self.links)):
                link = self.links[j]
                # 计算力臂
                arm = self.compute_lever_arm(i, j, joint_angles)
                total_moment += link.mass * self.g * arm
            
            torques[i] = total_moment
        
        return torques
    
    def compute_lever_arm(self, joint_idx: int, link_idx: int,
                          joint_angles: np.ndarray) -> float:
        """计算力臂"""
        # 简化计算
        arm = 0.0
        for k in range(joint_idx, link_idx + 1):
            angle = sum(joint_angles[:k+1])
            arm += self.links[k].length * np.cos(angle)
        return arm


def main():
    dynamics = AerialManipulatorDynamics()
    
    # 测试
    joint_angles = np.array([0.3, -0.2, 0.1])
    joint_velocities = np.array([0.1, 0.0, 0.0])
    joint_accelerations = np.array([0.5, 0.0, 0.0])
    
    print(f"总质量: {dynamics.compute_total_mass(joint_angles):.2f} kg")
    print(f"质心位置: {dynamics.compute_com(joint_angles)}")
    print(f"反作用力: {dynamics.compute_reaction_forces(joint_angles, joint_velocities, joint_accelerations)}")
    print(f"重力补偿: {dynamics.compute_gravity_compensation(joint_angles)}")


if __name__ == '__main__':
    main()
```

### 14.2.2 URDF 模型

```xml
<?xml version="1.0"?>
<robot name="aerial_manipulator">
  
  <!-- UAV 基座 -->
  <link name="base_link">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.04"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.15" length="0.08"/>
      </geometry>
      <origin xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.15" length="0.08"/>
      </geometry>
    </collision>
  </link>
  
  <!-- 机械臂安装座 -->
  <link name="arm_base">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.03" length="0.02"/>
      </geometry>
      <origin xyz="0 0 -0.01"/>
    </visual>
  </link>
  
  <joint name="arm_base_joint" type="fixed">
    <parent link="base_link"/>
    <child link="arm_base"/>
    <origin xyz="0.1 0 -0.04"/>
  </joint>
  
  <!-- 连杆1 -->
  <link name="link1">
    <inertial>
      <mass value="0.2"/>
      <origin xyz="0.075 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.15 0.03 0.03"/>
      </geometry>
      <origin xyz="0.075 0 0"/>
    </visual>
  </link>
  
  <joint name="joint1" type="revolute">
    <parent link="arm_base"/>
    <child link="link1"/>
    <origin xyz="0 0 -0.02"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="2.0"/>
  </joint>
  
  <!-- 连杆2 -->
  <link name="link2">
    <inertial>
      <mass value="0.15"/>
      <origin xyz="0.06 0 0"/>
      <inertia ixx="0.00005" ixy="0" ixz="0" iyy="0.00005" iyz="0" izz="0.00005"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.12 0.025 0.025"/>
      </geometry>
      <origin xyz="0.06 0 0"/>
    </visual>
  </link>
  
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0.15 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="8" velocity="2.0"/>
  </joint>
  
  <!-- 连杆3 -->
  <link name="link3">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0.05 0 0"/>
      <inertia ixx="0.00003" ixy="0" ixz="0" iyy="0.00003" iyz="0" izz="0.00003"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.1 0.02 0.02"/>
      </geometry>
      <origin xyz="0.05 0 0"/>
    </visual>
  </link>
  
  <joint name="joint3" type="revolute">
    <parent link="link2"/>
    <child link="link3"/>
    <origin xyz="0.12 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="5" velocity="2.0"/>
  </joint>
  
  <!-- 末端执行器 -->
  <link name="end_effector">
    <inertial>
      <mass value="0.05"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.025"/>
    </visual>
  </link>
  
  <joint name="end_effector_joint" type="fixed">
    <parent link="link3"/>
    <child link="end_effector"/>
    <origin xyz="0.1 0 0"/>
  </joint>
  
</robot>
```

## 14.3 飞行控制与机械臂协调

### 14.3.1 协调控制器

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Wrench
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import math

class CoordinatedController(Node):
    """
    飞行平台与机械臂协调控制器
    """
    def __init__(self):
        super().__init__('coordinated_controller')
        
        # 状态
        self.uav_pose = None
        self.uav_velocity = None
        self.joint_states = None
        self.end_effector_pose = None
        
        # 目标
        self.uav_target = None
        self.ee_target = None
        
        # 控制参数
        self.declare_parameter('uav_kp', 2.0)
        self.declare_parameter('uav_kd', 1.0)
        self.declare_parameter('arm_kp', 5.0)
        self.declare_parameter('arm_kd', 0.5)
        
        # 订阅
        self.uav_pose_sub = self.create_subscription(
            PoseStamped, '/uav/pose',
            self.uav_pose_callback, 10)
        
        self.uav_vel_sub = self.create_subscription(
            Twist, '/uav/velocity',
            self.uav_vel_callback, 10)
        
        self.joint_state_sub = self.create_subscription(
            JointState, '/arm/joint_states',
            self.joint_state_callback, 10)
        
        self.ee_target_sub = self.create_subscription(
            PoseStamped, '/end_effector/target',
            self.ee_target_callback, 10)
        
        # 发布
        self.uav_thrust_pub = self.create_publisher(
            Wrench, '/uav/thrust', 10)
        
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray, '/arm/joint_commands', 10)
        
        self.ee_pose_pub = self.create_publisher(
            PoseStamped, '/end_effector/pose', 10)
        
        # 定时控制
        self.timer = self.create_timer(0.02, self.control_loop)  # 50Hz
        
        # 动力学模型
        self.dynamics = AerialManipulatorDynamics()
        
        self.get_logger().info('协调控制器已启动')

    def uav_pose_callback(self, msg: PoseStamped):
        self.uav_pose = msg

    def uav_vel_callback(self, msg: Twist):
        self.uav_velocity = msg

    def joint_state_callback(self, msg: JointState):
        self.joint_states = msg
        self.update_ee_pose()

    def ee_target_callback(self, msg: PoseStamped):
        self.ee_target = msg

    def update_ee_pose(self):
        """更新末端执行器位姿"""
        if self.joint_states is None or self.uav_pose is None:
            return
        
        # 正运动学计算
        joint_angles = np.array(self.joint_states.position[:3])
        ee_local = self.forward_kinematics(joint_angles)
        
        # 转换到世界坐标
        self.end_effector_pose = PoseStamped()
        self.end_effector_pose.header.frame_id = 'map'
        self.end_effector_pose.pose.position.x = \
            self.uav_pose.pose.position.x + ee_local[0]
        self.end_effector_pose.pose.position.y = \
            self.uav_pose.pose.position.y + ee_local[1]
        self.end_effector_pose.pose.position.z = \
            self.uav_pose.pose.position.z + ee_local[2]
        
        self.ee_pose_pub.publish(self.end_effector_pose)

    def forward_kinematics(self, joint_angles: np.ndarray) -> np.ndarray:
        """正运动学"""
        # 简化的 2D 正运动学
        x = 0.1  # 基座偏移
        z = -0.04
        
        cumulative_angle = 0
        lengths = [0.15, 0.12, 0.1]  # 连杆长度
        
        for angle, length in zip(joint_angles, lengths):
            cumulative_angle += angle
            x += length * np.cos(cumulative_angle)
            z -= length * np.sin(cumulative_angle)
        
        return np.array([x, 0, z])

    def inverse_kinematics(self, target_local: np.ndarray) -> np.ndarray:
        """逆运动学"""
        # 简化的数值解法
        target_x, target_z = target_local[0], target_local[2]
        
        # 初始猜测
        q = np.array([0.0, 0.0, 0.0])
        
        for _ in range(100):
            current = self.forward_kinematics(q)
            error = np.array([target_x - current[0], target_z - current[2]])
            
            if np.linalg.norm(error) < 0.001:
                break
            
            # 雅可比矩阵（数值计算）
            J = self.compute_jacobian(q)
            
            # 更新
            dq = np.linalg.pinv(J) @ error
            q += dq * 0.5
            
            # 关节限位
            q = np.clip(q, [-1.57, -1.57, -3.14], [1.57, 1.57, 3.14])
        
        return q

    def compute_jacobian(self, joint_angles: np.ndarray) -> np.ndarray:
        """计算雅可比矩阵"""
        J = np.zeros((2, 3))
        delta = 0.001
        
        for i in range(3):
            q_plus = joint_angles.copy()
            q_plus[i] += delta
            
            p = self.forward_kinematics(joint_angles)
            p_plus = self.forward_kinematics(q_plus)
            
            J[0, i] = (p_plus[0] - p[0]) / delta
            J[1, i] = (p_plus[2] - p[2]) / delta
        
        return J

    def control_loop(self):
        """协调控制主循环"""
        if self.uav_pose is None or self.joint_states is None:
            return
        
        if self.ee_target is None:
            return
        
        # 计算末端误差
        ee_error = np.array([
            self.ee_target.pose.position.x - self.end_effector_pose.pose.position.x,
            self.ee_target.pose.position.y - self.end_effector_pose.pose.position.y,
            self.ee_target.pose.position.z - self.end_effector_pose.pose.position.z
        ])
        
        # 分解控制：UAV 负责大范围移动，机械臂负责精细调整
        uav_contribution, arm_contribution = self.decompose_control(ee_error)
        
        # UAV 控制
        self.control_uav(uav_contribution)
        
        # 机械臂控制
        self.control_arm(arm_contribution)

    def decompose_control(self, error: np.ndarray) -> tuple:
        """分解控制任务"""
        # UAV 负责大于阈值的误差
        threshold = 0.3  # 米
        
        uav_contribution = np.zeros(3)
        arm_contribution = error.copy()
        
        for i in range(3):
            if abs(error[i]) > threshold:
                uav_contribution[i] = error[i] - np.sign(error[i]) * threshold
                arm_contribution[i] = np.sign(error[i]) * threshold
        
        return uav_contribution, arm_contribution

    def control_uav(self, error: np.ndarray):
        """UAV 位置控制"""
        if self.uav_velocity is None:
            return
        
        kp = self.get_parameter('uav_kp').value
        kd = self.get_parameter('uav_kd').value
        
        # PD 控制
        thrust = Wrench()
        thrust.force.x = kp * error[0] - kd * self.uav_velocity.linear.x
        thrust.force.y = kp * error[1] - kd * self.uav_velocity.linear.y
        thrust.force.z = kp * error[2] - kd * self.uav_velocity.linear.z
        
        # 添加重力补偿
        joint_angles = np.array(self.joint_states.position[:3])
        total_mass = self.dynamics.compute_total_mass(joint_angles)
        thrust.force.z += total_mass * 9.81
        
        # 添加机械臂动力学补偿
        reaction = self.dynamics.compute_reaction_forces(
            joint_angles,
            np.array(self.joint_states.velocity[:3]),
            np.zeros(3)  # 假设加速度为0
        )
        thrust.torque.x = -reaction['torque'][0]
        thrust.torque.y = -reaction['torque'][1]
        
        self.uav_thrust_pub.publish(thrust)

    def control_arm(self, error: np.ndarray):
        """机械臂控制"""
        joint_angles = np.array(self.joint_states.position[:3])
        joint_velocities = np.array(self.joint_states.velocity[:3])
        
        # 计算目标关节角度
        target_local = np.array([
            error[0] + self.forward_kinematics(joint_angles)[0],
            0,
            error[2] + self.forward_kinematics(joint_angles)[2]
        ])
        
        target_joints = self.inverse_kinematics(target_local)
        
        # PD 控制
        kp = self.get_parameter('arm_kp').value
        kd = self.get_parameter('arm_kd').value
        
        joint_error = target_joints - joint_angles
        
        cmd = Float64MultiArray()
        cmd.data = (kp * joint_error - kd * joint_velocities).tolist()
        
        self.joint_cmd_pub.publish(cmd)


class AerialManipulatorDynamics:
    """简化的动力学模型"""
    def __init__(self):
        self.uav_mass = 2.0
        self.link_masses = [0.2, 0.15, 0.1]
    
    def compute_total_mass(self, joint_angles):
        return self.uav_mass + sum(self.link_masses)
    
    def compute_reaction_forces(self, angles, velocities, accelerations):
        # 简化计算
        return {'force': np.zeros(3), 'torque': np.zeros(3)}


def main(args=None):
    rclpy.init(args=args)
    controller = CoordinatedController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 14.4 视觉伺服抓取

### 14.4.1 视觉伺服控制器

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped, Twist
from cv_bridge import CvBridge
from std_msgs.msg import Float64MultiArray
import cv2
import numpy as np
import math

class VisualServoController(Node):
    """
    基于视觉的伺服控制器
    """
    def __init__(self):
        super().__init__('visual_servo_controller')
        
        self.bridge = CvBridge()
        
        # 相机参数
        self.declare_parameter('camera_matrix', [500.0, 0.0, 320.0,
                                                  0.0, 500.0, 240.0,
                                                  0.0, 0.0, 1.0])
        self.declare_parameter('target_size', 0.1)  # 目标尺寸（米）
        
        # 状态
        self.current_image = None
        self.joint_states = None
        self.target_detected = False
        self.target_pose = None
        
        # 订阅
        self.image_sub = self.create_subscription(
            Image, '/arm_camera/image_raw',
            self.image_callback, 10)
        
        self.joint_sub = self.create_subscription(
            JointState, '/arm/joint_states',
            self.joint_callback, 10)
        
        # 发布
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray, '/arm/joint_commands', 10)
        
        self.target_pose_pub = self.create_publisher(
            PoseStamped, '/detected_target', 10)
        
        self.debug_pub = self.create_publisher(
            Image, '/visual_servo/debug', 10)
        
        # 控制参数
        self.lambda_gain = 0.5  # 视觉伺服增益
        
        # 定时控制
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('视觉伺服控制器已启动')

    def image_callback(self, msg: Image):
        """处理图像"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')

    def joint_callback(self, msg: JointState):
        """更新关节状态"""
        self.joint_states = msg

    def detect_target(self, image):
        """检测目标"""
        if image is None:
            return None
        
        # 转换到 HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 检测特定颜色目标（示例：绿色）
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])
        
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # 形态学处理
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # 找最大轮廓
        max_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(max_contour)
        
        if area < 100:
            return None
        
        # 计算中心和外接矩形
        M = cv2.moments(max_contour)
        if M['m00'] == 0:
            return None
        
        cx = M['m10'] / M['m00']
        cy = M['m01'] / M['m00']
        
        rect = cv2.minAreaRect(max_contour)
        (rx, ry), (rw, rh), angle = rect
        
        return {
            'center': (cx, cy),
            'area': area,
            'rect': rect,
            'contour': max_contour
        }

    def estimate_target_pose(self, detection, image):
        """估计目标位姿"""
        if detection is None:
            return None
        
        cx, cy = detection['center']
        
        # 获取相机参数
        K = np.array(self.get_parameter('camera_matrix').value).reshape(3, 3)
        target_size = self.get_parameter('target_size').value
        
        # 像素到相机坐标转换
        fx, fy = K[0, 0], K[1, 1]
        cx0, cy0 = K[0, 2], K[1, 2]
        
        # 估计深度（基于目标尺寸）
        rect = detection['rect']
        pixel_size = max(rect[1])
        estimated_depth = target_size * fx / pixel_size
        
        # 计算 3D 位置
        x = (cx - cx0) * estimated_depth / fx
        y = (cy - cy0) * estimated_depth / fy
        z = estimated_depth
        
        return {
            'position': np.array([x, y, z]),
            'pixel_center': (cx, cy),
            'depth': estimated_depth
        }

    def compute_visual_servo(self, target_pose, image_size):
        """计算视觉伺服控制"""
        if target_pose is None:
            return None
        
        cx, cy = target_pose['pixel_center']
        width, height = image_size[1], image_size[0]
        
        # 图像中心
        center_x, center_y = width / 2, height / 2
        
        # 图像误差
        error_x = (cx - center_x) / width
        error_y = (cy - center_y) / height
        
        # 计算期望的末端执行器速度
        # 简化：直接映射到关节速度
        
        # 相机到末端执行器的变换（假设相机安装在末端）
        v_x = -self.lambda_gain * error_x
        v_y = -self.lambda_gain * error_y
        
        return {
            'v_x': v_x,
            'v_y': v_y,
            'error': (error_x, error_y)
        }

    def control_loop(self):
        """控制主循环"""
        if self.current_image is None or self.joint_states is None:
            return
        
        # 检测目标
        detection = self.detect_target(self.current_image)
        
        if detection is None:
            self.target_detected = False
            return
        
        self.target_detected = True
        
        # 估计目标位姿
        target_pose = self.estimate_target_pose(detection, self.current_image)
        
        # 计算视觉伺服控制
        servo_cmd = self.compute_visual_servo(
            target_pose, self.current_image.shape)
        
        if servo_cmd is None:
            return
        
        # 发布关节命令
        cmd = Float64MultiArray()
        
        # 简化：直接使用误差控制关节
        joint_angles = np.array(self.joint_states.position[:3])
        
        # 调整关节角度
        cmd.data = [
            joint_angles[0] + servo_cmd['v_x'] * 0.1,
            joint_angles[1] - servo_cmd['v_y'] * 0.1,
            joint_angles[2]
        ]
        
        self.joint_cmd_pub.publish(cmd)
        
        # 发布调试图像
        debug_image = self.current_image.copy()
        cv2.drawContours(debug_image, [detection['contour']], -1, (0, 255, 0), 2)
        cv2.circle(debug_image, 
                   (int(detection['center'][0]), int(detection['center'][1])),
                   5, (0, 0, 255), -1)
        
        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
        self.debug_pub.publish(debug_msg)
        
        # 发布目标位姿
        pose = PoseStamped()
        pose.header.frame_id = 'arm_camera'
        pose.pose.position.x = target_pose['position'][0]
        pose.pose.position.y = target_pose['position'][1]
        pose.pose.position.z = target_pose['position'][2]
        self.target_pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    controller = VisualServoController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 14.5 抓取动作规划

### 14.5.1 抓取规划器

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped, Wrench
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import math

class GraspPlanner(Node):
    """
    空中抓取动作规划器
    """
    def __init__(self):
        super().__init__('grasp_planner')
        
        # 状态
        self.uav_pose = None
        self.joint_states = None
        self.target_pose = None
        
        # 抓取参数
        self.declare_parameter('approach_distance', 0.2)
        self.declare_parameter('grasp_force', 5.0)
        self.declare_parameter('retreat_distance', 0.3)
        
        # 订阅
        self.uav_pose_sub = self.create_subscription(
            PoseStamped, '/uav/pose',
            self.uav_pose_callback, 10)
        
        self.joint_sub = self.create_subscription(
            JointState, '/arm/joint_states',
            self.joint_callback, 10)
        
        self.target_sub = self.create_subscription(
            PoseStamped, '/grasp_target',
            self.target_callback, 10)
        
        # 发布
        self.uav_cmd_pub = self.create_publisher(
            PoseStamped, '/uav/target_pose', 10)
        
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray, '/arm/joint_commands', 10)
        
        self.gripper_cmd_pub = self.create_publisher(
            Float64MultiArray, '/gripper/command', 10)
        
        # 状态机
        self.state = 'IDLE'
        self.state_timer = None
        
        self.get_logger().info('抓取规划器已启动')

    def uav_pose_callback(self, msg: PoseStamped):
        self.uav_pose = msg

    def joint_callback(self, msg: JointState):
        self.joint_states = msg

    def target_callback(self, msg: PoseStamped):
        self.target_pose = msg
        if self.state == 'IDLE':
            self.start_grasp_sequence()

    def start_grasp_sequence(self):
        """开始抓取序列"""
        self.state = 'APPROACH'
        self.get_logger().info('开始抓取序列')
        self.execute_approach()

    def execute_approach(self):
        """执行接近阶段"""
        if self.target_pose is None:
            return
        
        approach_dist = self.get_parameter('approach_distance').value
        
        # 计算接近点（目标上方）
        approach_pose = PoseStamped()
        approach_pose.header.frame_id = 'map'
        approach_pose.pose.position.x = self.target_pose.pose.position.x
        approach_pose.pose.position.y = self.target_pose.pose.position.y
        approach_pose.pose.position.z = self.target_pose.pose.position.z + approach_dist
        
        self.uav_cmd_pub.publish(approach_pose)
        
        # 设置机械臂为准备姿态
        self.set_arm_pregrasp()
        
        # 检查是否到达
        self.check_approach_timer = self.create_timer(
            0.5, self.check_approach_complete)

    def check_approach_complete(self):
        """检查接近是否完成"""
        if self.uav_pose is None:
            return
        
        dist = self.calculate_distance_2d(
            self.uav_pose.pose.position,
            self.target_pose.pose.position
        )
        
        if dist < 0.1:
            self.check_approach_timer.cancel()
            self.state = 'DESCEND'
            self.execute_descend()

    def execute_descend(self):
        """执行下降阶段"""
        self.get_logger().info('下降到抓取位置')
        
        # 移动到目标位置
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = 'map'
        grasp_pose.pose.position.x = self.target_pose.pose.position.x
        grasp_pose.pose.position.y = self.target_pose.pose.position.y
        grasp_pose.pose.position.z = self.target_pose.pose.position.z + 0.05
        
        self.uav_cmd_pub.publish(grasp_pose)
        
        # 检查下降完成
        self.check_descend_timer = self.create_timer(
            0.5, self.check_descend_complete)

    def check_descend_complete(self):
        """检查下降是否完成"""
        if self.uav_pose is None:
            return
        
        target_z = self.target_pose.pose.position.z + 0.05
        current_z = self.uav_pose.pose.position.z
        
        if abs(current_z - target_z) < 0.05:
            self.check_descend_timer.cancel()
            self.state = 'GRASP'
            self.execute_grasp()

    def execute_grasp(self):
        """执行抓取"""
        self.get_logger().info('执行抓取')
        
        # 关闭夹爪
        gripper_cmd = Float64MultiArray()
        gripper_cmd.data = [0.0]  # 关闭
        self.gripper_cmd_pub.publish(gripper_cmd)
        
        # 等待抓取完成
        self.grasp_timer = self.create_timer(1.0, self.grasp_complete)

    def grasp_complete(self):
        """抓取完成"""
        self.grasp_timer.cancel()
        self.state = 'RETREAT'
        self.execute_retreat()

    def execute_retreat(self):
        """执行撤退"""
        self.get_logger().info('撤退')
        
        retreat_dist = self.get_parameter('retreat_distance').value
        
        # 上升到安全高度
        retreat_pose = PoseStamped()
        retreat_pose.header.frame_id = 'map'
        retreat_pose.pose.position.x = self.uav_pose.pose.position.x
        retreat_pose.pose.position.y = self.uav_pose.pose.position.y
        retreat_pose.pose.position.z = self.uav_pose.pose.position.z + retreat_dist
        
        self.uav_cmd_pub.publish(retreat_pose)
        
        # 检查撤退完成
        self.check_retreat_timer = self.create_timer(
            0.5, self.check_retreat_complete)

    def check_retreat_complete(self):
        """检查撤退是否完成"""
        if self.uav_pose is None:
            return
        
        if self.uav_pose.pose.position.z > 1.0:  # 安全高度
            self.check_retreat_timer.cancel()
            self.state = 'TRANSPORT'
            self.get_logger().info('抓取完成，准备运输')

    def set_arm_pregrasp(self):
        """设置机械臂预抓取姿态"""
        cmd = Float64MultiArray()
        cmd.data = [0.3, -0.5, 0.2]  # 预抓取关节角度
        self.joint_cmd_pub.publish(cmd)

    def calculate_distance_2d(self, pos1, pos2):
        """计算 2D 距离"""
        dx = pos1.x - pos2.x
        dy = pos1.y - pos2.y
        return math.sqrt(dx*dx + dy*dy)


def main(args=None):
    rclpy.init(args=args)
    planner = GraspPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 14.6 空中机械臂 Launch 配置

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    
    # UAV 节点
    uav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uav_package'),
                'launch', 'uav.launch.py'
            ])
        ])
    )
    
    # 机械臂控制器
    arm_controller = Node(
        package='aerial_manipulator',
        executable='arm_controller',
        name='arm_controller',
        output='screen'
    )
    
    # 协调控制器
    coordinated_controller = Node(
        package='aerial_manipulator',
        executable='coordinated_controller',
        name='coordinated_controller',
        output='screen'
    )
    
    # 视觉伺服
    visual_servo = Node(
        package='aerial_manipulator',
        executable='visual_servo_controller',
        name='visual_servo_controller',
        output='screen'
    )
    
    # 抓取规划器
    grasp_planner = Node(
        package='aerial_manipulator',
        executable='grasp_planner',
        name='grasp_planner',
        output='screen'
    )
    
    return LaunchDescription([
        uav_launch,
        arm_controller,
        coordinated_controller,
        visual_servo,
        grasp_planner
    ])
```

## 14.7 安全与故障处理

### 14.7.1 安全监控节点

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Wrench
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import String
import json
import math

class SafetyMonitor(Node):
    """
    空中机械臂安全监控
    """
    def __init__(self):
        super().__init__('safety_monitor')
        
        # 安全参数
        self.declare_parameter('max_joint_velocity', 2.0)
        self.declare_parameter('max_joint_torque', 10.0)
        self.declare_parameter('max_tilt_angle', 0.5)  # 弧度
        self.declare_parameter('min_altitude', 0.5)
        self.declare_parameter('max_payload', 0.5)
        
        # 状态
        self.joint_states = None
        self.uav_pose = None
        self.imu_data = None
        
        # 订阅
        self.joint_sub = self.create_subscription(
            JointState, '/arm/joint_states',
            self.joint_callback, 10)
        
        self.uav_pose_sub = self.create_subscription(
            PoseStamped, '/uav/pose',
            self.uav_pose_callback, 10)
        
        self.imu_sub = self.create_subscription(
            Imu, '/uav/imu',
            self.imu_callback, 10)
        
        # 发布
        self.emergency_pub = self.create_publisher(
            String, '/emergency_stop', 10)
        
        self.status_pub = self.create_publisher(
            String, '/safety_status', 10)
        
        # 定时检查
        self.timer = self.create_timer(0.1, self.safety_check)
        
        self.get_logger().info('安全监控已启动')

    def joint_callback(self, msg: JointState):
        self.joint_states = msg

    def uav_pose_callback(self, msg: PoseStamped):
        self.uav_pose = msg

    def imu_callback(self, msg: Imu):
        self.imu_data = msg

    def safety_check(self):
        """安全检查"""
        warnings = []
        errors = []
        
        # 检查关节速度
        if self.joint_states:
            max_vel = self.get_parameter('max_joint_velocity').value
            for i, vel in enumerate(self.joint_states.velocity):
                if abs(vel) > max_vel:
                    warnings.append(f'关节{i+1}速度超限: {vel:.2f}')
        
        # 检查飞行姿态
        if self.imu_data:
            max_tilt = self.get_parameter('max_tilt_angle').value
            
            # 从四元数计算倾斜角
            q = self.imu_data.orientation
            roll = math.atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y))
            pitch = math.asin(2*(q.w*q.y - q.z*q.x))
            
            if abs(roll) > max_tilt or abs(pitch) > max_tilt:
                errors.append(f'姿态超限: roll={roll:.2f}, pitch={pitch:.2f}')
        
        # 检查高度
        if self.uav_pose:
            min_alt = self.get_parameter('min_altitude').value
            if self.uav_pose.pose.position.z < min_alt:
                errors.append(f'高度过低: {self.uav_pose.pose.position.z:.2f}m')
        
        # 发布状态
        status = {
            'warnings': warnings,
            'errors': errors,
            'safe': len(errors) == 0
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)
        
        # 紧急停止
        if errors:
            self.get_logger().error(f'安全错误: {errors}')
            emergency_msg = String()
            emergency_msg.data = json.dumps({
                'reason': errors[0],
                'action': 'emergency_stop'
            })
            self.emergency_pub.publish(emergency_msg)


def main(args=None):
    rclpy.init(args=args)
    monitor = SafetyMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 总结

本章介绍了空中机械臂系统的核心技术：

1. **系统建模**：动力学模型、URDF 模型
2. **协调控制**：飞行平台与机械臂的协调
3. **视觉伺服**：基于视觉的精确抓取
4. **抓取规划**：完整的抓取动作序列
5. **安全监控**：故障检测与处理

空中机械臂是一个复杂的机电系统，需要综合考虑：
- 飞行稳定性
- 机械臂运动学/动力学
- 视觉感知
- 安全保障

建议进一步学习：
- PX4 飞控高级配置
- MoveIt2 运动规划
- 视觉伺服理论
- 空气动力学

---

**教程完成！** 恭喜你完成了 ROS2 完整教程的学习！
