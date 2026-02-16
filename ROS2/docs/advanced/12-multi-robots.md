# 第12章：多机器人协作

## 12.1 多机器人系统概述

### 12.1.1 协作架构模式

<div align="center">
<svg width="700" height="520" viewBox="0 0 700 520" xmlns="http://www.w3.org/2000/svg">
  <defs>
    <marker id="arrowhead" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
      <polygon points="0 0, 10 3.5, 0 7" fill="#495057"/>
    </marker>
  </defs>

  <!-- 背景 -->
  <rect x="10" y="10" width="680" height="500" fill="#f8f9fa" stroke="#dee2e6" stroke-width="1" rx="4"/>
  <text x="350" y="38" font-family="system-ui, -apple-system, sans-serif" font-size="18" font-weight="600" text-anchor="middle" fill="#212529">多机器人协作架构</text>

  <!-- 集中式架构 -->
  <rect x="30" y="55" width="640" height="130" fill="#e3f2fd" stroke="#1976d2" stroke-width="1.5" rx="5"/>
  <text x="350" y="75" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#1565c0">集中式架构 (Centralized)</text>

  <!-- 中央控制节点 -->
  <rect x="280" y="90" width="140" height="50" fill="#1976d2" stroke="#0d47a1" stroke-width="1.5" rx="4"/>
  <text x="350" y="110" font-family="system-ui, -apple-system, sans-serif" font-size="13" font-weight="600" text-anchor="middle" fill="#ffffff">中央控制节点</text>
  <text x="350" y="128" font-family="system-ui, -apple-system, sans-serif" font-size="9" text-anchor="middle" fill="#bbdefb">全局信息汇总 / 任务分配</text>

  <!-- 机器人1 -->
  <rect x="60" y="155" width="80" height="35" fill="#ffffff" stroke="#1976d2" stroke-width="1.5" rx="3"/>
  <text x="100" y="177" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#1565c0">机器人1</text>

  <!-- 机器人2 -->
  <rect x="310" y="155" width="80" height="35" fill="#ffffff" stroke="#1976d2" stroke-width="1.5" rx="3"/>
  <text x="350" y="177" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#1565c0">机器人2</text>

  <!-- 机器人3 -->
  <rect x="560" y="155" width="80" height="35" fill="#ffffff" stroke="#1976d2" stroke-width="1.5" rx="3"/>
  <text x="600" y="177" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#1565c0">机器人3</text>

  <!-- 连接线 -->
  <line x1="100" y1="155" x2="310" y2="140" stroke="#495057" stroke-width="1.5" stroke-dasharray="4,2"/>
  <line x1="350" y1="140" x2="350" y2="155" stroke="#495057" stroke-width="1.5" stroke-dasharray="4,2"/>
  <line x1="600" y1="155" x2="420" y2="140" stroke="#495057" stroke-width="1.5" stroke-dasharray="4,2"/>

  <!-- 分布式架构 -->
  <rect x="30" y="200" width="640" height="130" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="1.5" rx="5"/>
  <text x="350" y="220" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#6a1b9a">分布式架构 (Distributed)</text>

  <!-- 机器人A -->
  <rect x="100" y="245" width="80" height="40" fill="#ffffff" stroke="#7b1fa2" stroke-width="1.5" rx="3"/>
  <text x="140" y="265" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#6a1b9a">机器人A</text>
  <text x="140" y="278" font-family="system-ui, -apple-system, sans-serif" font-size="9" text-anchor="middle" fill="#9c27b0">自主决策</text>

  <!-- 机器人B -->
  <rect x="310" y="245" width="80" height="40" fill="#ffffff" stroke="#7b1fa2" stroke-width="1.5" rx="3"/>
  <text x="350" y="265" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#6a1b9a">机器人B</text>
  <text x="350" y="278" font-family="system-ui, -apple-system, sans-serif" font-size="9" text-anchor="middle" fill="#9c27b0">自主决策</text>

  <!-- 机器人C -->
  <rect x="520" y="245" width="80" height="40" fill="#ffffff" stroke="#7b1fa2" stroke-width="1.5" rx="3"/>
  <text x="560" y="265" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#6a1b9a">机器人C</text>
  <text x="560" y="278" font-family="system-ui, -apple-system, sans-serif" font-size="9" text-anchor="middle" fill="#9c27b0">自主决策</text>

  <!-- 机器人D -->
  <rect x="205" y="300" width="80" height="40" fill="#ffffff" stroke="#7b1fa2" stroke-width="1.5" rx="3"/>
  <text x="245" y="320" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#6a1b9a">机器人D</text>
  <text x="245" y="333" font-family="system-ui, -apple-system, sans-serif" font-size="9" text-anchor="middle" fill="#9c27b0">自主决策</text>

  <!-- 机器人E -->
  <rect x="415" y="300" width="80" height="40" fill="#ffffff" stroke="#7b1fa2" stroke-width="1.5" rx="3"/>
  <text x="455" y="320" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#6a1b9a">机器人E</text>
  <text x="455" y="333" font-family="system-ui, -apple-system, sans-serif" font-size="9" text-anchor="middle" fill="#9c27b0">自主决策</text>

  <!-- 通信连线 -->
  <line x1="180" y1="265" x2="310" y2="265" stroke="#7b1fa2" stroke-width="1.5" marker-end="url(#arrowhead)"/>
  <line x1="390" y1="265" x2="520" y2="265" stroke="#7b1fa2" stroke-width="1.5" marker-end="url(#arrowhead)"/>
  <line x1="285" y1="320" x2="205" y2="320" stroke="#7b1fa2" stroke-width="1.5" marker-end="url(#arrowhead)"/>
  <line x1="495" y1="320" x2="415" y2="320" stroke="#7b1fa2" stroke-width="1.5" marker-end="url(#arrowhead)"/>
  <line x1="140" y1="285" x2="245" y2="300" stroke="#7b1fa2" stroke-width="1" stroke-dasharray="3,2"/>
  <line x1="560" y1="285" x2="455" y2="300" stroke="#7b1fa2" stroke-width="1" stroke-dasharray="3,2"/>

  <text x="350" y="295" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#6a1b9a">点对点通信 / 协商</text>

  <!-- 混合式架构 -->
  <rect x="30" y="345" width="640" height="150" fill="#fff3e0" stroke="#f57c00" stroke-width="1.5" rx="5"/>
  <text x="350" y="365" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#ef6c00">混合式架构 (Hybrid)</text>

  <!-- 协调服务器 -->
  <rect x="280" y="380" width="140" height="50" fill="#f57c00" stroke="#e65100" stroke-width="1.5" rx="4"/>
  <text x="350" y="400" font-family="system-ui, -apple-system, sans-serif" font-size="13" font-weight="600" text-anchor="middle" fill="#ffffff">协调服务器</text>
  <text x="350" y="418" font-family="system-ui, -apple-system, sans-serif" font-size="9" text-anchor="middle" fill="#ffe0b2">全局协调</text>

  <!-- 组A -->
  <rect x="60" y="445" width="120" height="40" fill="#ffffff" stroke="#f57c00" stroke-width="1.5" rx="3"/>
  <text x="120" y="460" font-family="system-ui, -apple-system, sans-serif" font-size="12" font-weight="600" text-anchor="middle" fill="#ef6c00">组 A</text>
  <text x="120" y="475" font-family="system-ui, -apple-system, sans-serif" font-size="9" text-anchor="middle" fill="#757575">组内分布式协作</text>

  <!-- 组B -->
  <rect x="520" y="445" width="120" height="40" fill="#ffffff" stroke="#f57c00" stroke-width="1.5" rx="3"/>
  <text x="580" y="460" font-family="system-ui, -apple-system, sans-serif" font-size="12" font-weight="600" text-anchor="middle" fill="#ef6c00">组 B</text>
  <text x="580" y="475" font-family="system-ui, -apple-system, sans-serif" font-size="9" text-anchor="middle" fill="#757575">组内分布式协作</text>

  <!-- 组内机器人 -->
  <circle cx="95" cy="465" r="6" fill="#f57c00"/>
  <circle cx="120" cy="465" r="6" fill="#f57c00"/>
  <circle cx="145" cy="465" r="6" fill="#f57c00"/>
  <line x1="101" y1="465" x2="114" y2="465" stroke="#f57c00" stroke-width="1.5"/>
  <line x1="126" y1="465" x2="139" y2="465" stroke="#f57c00" stroke-width="1.5"/>

  <circle cx="555" cy="465" r="6" fill="#f57c00"/>
  <circle cx="580" cy="465" r="6" fill="#f57c00"/>
  <circle cx="605" cy="465" r="6" fill="#f57c00"/>
  <line x1="561" y1="465" x2="574" y2="465" stroke="#f57c00" stroke-width="1.5"/>
  <line x1="586" y1="465" x2="599" y2="465" stroke="#f57c00" stroke-width="1.5"/>

  <!-- 连接线 -->
  <line x1="180" y1="455" x2="280" y2="430" stroke="#495057" stroke-width="1.5" stroke-dasharray="4,2"/>
  <line x1="520" y1="455" x2="420" y2="430" stroke="#495057" stroke-width="1.5" stroke-dasharray="4,2"/>
</svg>
</div>

### 12.1.2 协作模式对比

| 架构模式 | 优点 | 缺点 | 适用场景 |
|----------|------|------|----------|
| 集中式 | 全局最优、易管理 | 单点故障、扩展性差 | 小规模、结构化环境 |
| 分布式 | 鲁棒性强、可扩展 | 难以全局最优 | 大规模、开放环境 |
| 混合式 | 兼顾两者优点 | 实现复杂 | 中大规模、复杂任务 |

### 12.1.3 ROS2 多机器人通信

<div align="center">
<svg width="700" height="380" viewBox="0 0 700 380" xmlns="http://www.w3.org/2000/svg">
  <defs>
    <marker id="arrowhead" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
      <polygon points="0 0, 10 3.5, 0 7" fill="#495057"/>
    </marker>
  </defs>

  <!-- 背景 -->
  <rect x="10" y="10" width="680" height="360" fill="#f8f9fa" stroke="#dee2e6" stroke-width="1" rx="4"/>
  <text x="350" y="38" font-family="system-ui, -apple-system, sans-serif" font-size="18" font-weight="600" text-anchor="middle" fill="#212529">ROS2 多机器人命名空间</text>

  <!-- robot1 命名空间 -->
  <rect x="30" y="55" width="200" height="180" fill="#e3f2fd" stroke="#1976d2" stroke-width="2" rx="5"/>
  <text x="130" y="80" font-family="monospace" font-size="14" font-weight="600" text-anchor="middle" fill="#1565c0">/robot1/</text>
  
  <rect x="45" y="95" width="170" height="25" fill="#ffffff" stroke="#1976d2" stroke-width="1" rx="3"/>
  <text x="130" y="113" font-family="monospace" font-size="12" text-anchor="middle" fill="#424242">cmd_vel</text>
  
  <rect x="45" y="125" width="170" height="25" fill="#ffffff" stroke="#1976d2" stroke-width="1" rx="3"/>
  <text x="130" y="143" font-family="monospace" font-size="12" text-anchor="middle" fill="#424242">odom</text>
  
  <rect x="45" y="155" width="170" height="25" fill="#ffffff" stroke="#1976d2" stroke-width="1" rx="3"/>
  <text x="130" y="173" font-family="monospace" font-size="12" text-anchor="middle" fill="#424242">scan</text>
  
  <rect x="45" y="185" width="170" height="40" fill="#bbdefb" stroke="#1976d2" stroke-width="1" rx="3"/>
  <text x="130" y="203" font-family="monospace" font-size="11" text-anchor="middle" fill="#1565c0">navigation/</text>
  <text x="130" y="218" font-family="monospace" font-size="10" text-anchor="middle" fill="#757575">plan, goal</text>

  <!-- robot2 命名空间 -->
  <rect x="250" y="55" width="200" height="180" fill="#e8f5e9" stroke="#388e3c" stroke-width="2" rx="5"/>
  <text x="350" y="80" font-family="monospace" font-size="14" font-weight="600" text-anchor="middle" fill="#2e7d32">/robot2/</text>
  
  <rect x="265" y="95" width="170" height="25" fill="#ffffff" stroke="#388e3c" stroke-width="1" rx="3"/>
  <text x="350" y="113" font-family="monospace" font-size="12" text-anchor="middle" fill="#424242">cmd_vel</text>
  
  <rect x="265" y="125" width="170" height="25" fill="#ffffff" stroke="#388e3c" stroke-width="1" rx="3"/>
  <text x="350" y="143" font-family="monospace" font-size="12" text-anchor="middle" fill="#424242">odom</text>
  
  <rect x="265" y="155" width="170" height="25" fill="#ffffff" stroke="#388e3c" stroke-width="1" rx="3"/>
  <text x="350" y="173" font-family="monospace" font-size="12" text-anchor="middle" fill="#424242">scan</text>
  
  <rect x="265" y="185" width="170" height="40" fill="#c8e6c9" stroke="#388e3c" stroke-width="1" rx="3"/>
  <text x="350" y="203" font-family="monospace" font-size="11" text-anchor="middle" fill="#2e7d32">navigation/</text>
  <text x="350" y="218" font-family="monospace" font-size="10" text-anchor="middle" fill="#757575">plan, goal</text>

  <!-- coordinator 命名空间 -->
  <rect x="470" y="55" width="200" height="180" fill="#fff3e0" stroke="#f57c00" stroke-width="2" rx="5"/>
  <text x="570" y="80" font-family="monospace" font-size="14" font-weight="600" text-anchor="middle" fill="#ef6c00">/coordinator/</text>
  
  <rect x="485" y="95" width="170" height="25" fill="#ffffff" stroke="#f57c00" stroke-width="1" rx="3"/>
  <text x="570" y="113" font-family="monospace" font-size="12" text-anchor="middle" fill="#424242">task_assignment</text>
  
  <rect x="485" y="125" width="170" height="25" fill="#ffffff" stroke="#f57c00" stroke-width="1" rx="3"/>
  <text x="570" y="143" font-family="monospace" font-size="12" text-anchor="middle" fill="#424242">formation_control</text>
  
  <rect x="485" y="155" width="170" height="25" fill="#ffffff" stroke="#f57c00" stroke-width="1" rx="3"/>
  <text x="570" y="173" font-family="monospace" font-size="12" text-anchor="middle" fill="#424242">conflict_resolution</text>

  <!-- shared 命名空间 -->
  <rect x="250" y="250" width="200" height="100" fill="#fce4ec" stroke="#c2185b" stroke-width="2" rx="5"/>
  <text x="350" y="275" font-family="monospace" font-size="14" font-weight="600" text-anchor="middle" fill="#ad1457">/shared/</text>
  
  <rect x="265" y="290" width="80" height="25" fill="#ffffff" stroke="#c2185b" stroke-width="1" rx="3"/>
  <text x="305" y="308" font-family="monospace" font-size="11" text-anchor="middle" fill="#424242">map</text>
  
  <rect x="355" y="290" width="80" height="25" fill="#ffffff" stroke="#c2185b" stroke-width="1" rx="3"/>
  <text x="395" y="308" font-family="monospace" font-size="11" text-anchor="middle" fill="#424242">tf</text>
  
  <rect x="265" y="320" width="170" height="25" fill="#ffffff" stroke="#c2185b" stroke-width="1" rx="3"/>
  <text x="350" y="338" font-family="monospace" font-size="11" text-anchor="middle" fill="#424242">global_state</text>

  <!-- 连接线 -->
  <line x1="130" y1="235" x2="305" y2="250" stroke="#495057" stroke-width="1.5" stroke-dasharray="5,3"/>
  <line x1="350" y1="235" x2="350" y2="250" stroke="#495057" stroke-width="1.5" stroke-dasharray="5,3"/>
  <line x1="570" y1="235" x2="395" y2="250" stroke="#495057" stroke-width="1.5" stroke-dasharray="5,3"/>
</svg>
</div>

## 12.2 任务分配系统

### 12.2.1 任务分配算法

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
import json
import math

class TaskAllocator(Node):
    """
    基于拍卖机制的任务分配器
    """
    def __init__(self):
        super().__init__('task_allocator')
        
        # 机器人状态
        self.robots = {}
        self.robot_clients = {}
        
        # 任务队列
        self.task_queue = []
        self.completed_tasks = []
        
        # 拍卖参数
        self.auction_timeout = 2.0  # 拍卖超时时间
        self.current_auction = None
        self.bids = {}
        
        # 订阅机器人状态
        self.status_sub = self.create_subscription(
            String, '/robot_status', self.robot_status_callback, 10)
        
        # 发布任务
        self.task_pub = self.create_publisher(String, '/task_assignment', 10)
        
        # 定时器
        self.timer = self.create_timer(0.5, self.process_tasks)
        
        self.get_logger().info('任务分配器已启动')

    def robot_status_callback(self, msg: String):
        """更新机器人状态"""
        status = json.loads(msg.data)
        robot_id = status['robot_id']
        self.robots[robot_id] = {
            'position': status['position'],
            'battery': status['battery'],
            'status': status['status'],
            'current_task': status.get('current_task', None)
        }

    def add_task(self, task_id, task_type, location, priority=0):
        """添加任务到队列"""
        task = {
            'id': task_id,
            'type': task_type,
            'location': location,  # {'x': float, 'y': float}
            'priority': priority,
            'status': 'pending',
            'assigned_to': None
        }
        self.task_queue.append(task)
        self.task_queue.sort(key=lambda t: t['priority'], reverse=True)
        self.get_logger().info(f'添加任务: {task_id}')

    def process_tasks(self):
        """处理任务分配"""
        # 找待分配任务
        pending_tasks = [t for t in self.task_queue if t['status'] == 'pending']
        if not pending_tasks:
            return
        
        # 找空闲机器人
        idle_robots = [
            rid for rid, r in self.robots.items() 
            if r['status'] == 'idle' and r['battery'] > 20
        ]
        if not idle_robots:
            return
        
        # 分配任务
        task = pending_tasks[0]
        best_robot = self.find_best_robot(task, idle_robots)
        
        if best_robot:
            self.assign_task(task, best_robot)

    def find_best_robot(self, task, available_robots):
        """找最佳机器人（基于距离和电量）"""
        best_robot = None
        best_score = float('inf')
        
        for robot_id in available_robots:
            robot = self.robots[robot_id]
            
            # 计算距离
            dx = robot['position']['x'] - task['location']['x']
            dy = robot['position']['y'] - task['location']['y']
            distance = math.sqrt(dx*dx + dy*dy)
            
            # 综合评分（距离 + 电量惩罚）
            score = distance + (100 - robot['battery']) * 0.1
            
            if score < best_score:
                best_score = score
                best_robot = robot_id
        
        return best_robot

    def assign_task(self, task, robot_id):
        """分配任务给机器人"""
        task['status'] = 'assigned'
        task['assigned_to'] = robot_id
        self.robots[robot_id]['status'] = 'busy'
        
        # 发布任务分配消息
        assignment = {
            'task_id': task['id'],
            'robot_id': robot_id,
            'location': task['location'],
            'type': task['type']
        }
        msg = String()
        msg.data = json.dumps(assignment)
        self.task_pub.publish(msg)
        
        self.get_logger().info(f'任务 {task["id"]} 分配给 {robot_id}')

    def task_completed(self, task_id, robot_id):
        """任务完成回调"""
        for task in self.task_queue:
            if task['id'] == task_id:
                task['status'] = 'completed'
                self.completed_tasks.append(task)
                self.task_queue.remove(task)
                break
        
        if robot_id in self.robots:
            self.robots[robot_id]['status'] = 'idle'
        
        self.get_logger().info(f'任务 {task_id} 完成')


class AuctionTaskAllocator(TaskAllocator):
    """
    基于拍卖的任务分配器
    """
    def __init__(self):
        super().__init__()
        
        # 拍卖订阅
        self.bid_sub = self.create_subscription(
            String, '/task_bids', self.bid_callback, 10)
        
        # 拍卖状态
        self.auction_active = False
        self.auction_timer = None

    def start_auction(self, task):
        """开始拍卖"""
        self.current_auction = task
        self.bids = {}
        self.auction_active = True
        
        # 发布拍卖公告
        auction_msg = {
            'type': 'auction_start',
            'task': task
        }
        msg = String()
        msg.data = json.dumps(auction_msg)
        self.task_pub.publish(msg)
        
        # 设置拍卖超时
        self.auction_timer = self.create_timer(
            self.auction_timeout, self.end_auction)
        
        self.get_logger().info(f'开始拍卖任务: {task["id"]}')

    def bid_callback(self, msg: String):
        """接收投标"""
        if not self.auction_active:
            return
        
        bid = json.loads(msg.data)
        if bid['task_id'] == self.current_auction['id']:
            self.bids[bid['robot_id']] = bid['value']

    def end_auction(self):
        """结束拍卖，分配任务"""
        if self.auction_timer:
            self.auction_timer.cancel()
        
        if not self.bids:
            self.get_logger().warn('没有收到投标')
            self.auction_active = False
            return
        
        # 找最低投标者
        winner = min(self.bids, key=self.bids.get)
        self.assign_task(self.current_auction, winner)
        
        # 发布结果
        result_msg = {
            'type': 'auction_result',
            'task_id': self.current_auction['id'],
            'winner': winner
        }
        msg = String()
        msg.data = json.dumps(result_msg)
        self.task_pub.publish(msg)
        
        self.auction_active = False
        self.current_auction = None


def main(args=None):
    rclpy.init(args=args)
    allocator = AuctionTaskAllocator()
    
    # 添加示例任务
    allocator.add_task('task_1', 'pickup', {'x': 5.0, 'y': 3.0}, priority=2)
    allocator.add_task('task_2', 'delivery', {'x': 2.0, 'y': 7.0}, priority=1)
    allocator.add_task('task_3', 'patrol', {'x': 8.0, 'y': 1.0}, priority=0)
    
    rclpy.spin(allocator)
    allocator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 12.2.2 机器人代理节点

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
import json
import math

class RobotAgent(Node):
    """
    机器人代理节点 - 负责单机器人的任务执行和状态上报
    """
    def __init__(self, robot_id):
        super().__init__(f'{robot_id}_agent')
        
        self.robot_id = robot_id
        self.namespace = f'/{robot_id}'
        
        # 状态
        self.position = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.battery = 100.0
        self.status = 'idle'  # idle, busy, charging, error
        self.current_task = None
        
        # 订阅
        self.odom_sub = self.create_subscription(
            Odometry, f'{self.namespace}/odom',
            self.odom_callback, 10)
        
        self.task_sub = self.create_subscription(
            String, '/task_assignment',
            self.task_callback, 10)
        
        self.auction_sub = self.create_subscription(
            String, '/task_assignment',
            self.auction_callback, 10)
        
        # 发布
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.cmd_vel_pub = self.create_publisher(
            Twist, f'{self.namespace}/cmd_vel', 10)
        self.bid_pub = self.create_publisher(String, '/task_bids', 10)
        
        # 导航客户端
        self.nav_client = ActionClient(
            self, NavigateToPose, f'{self.namespace}/navigate_to_pose')
        
        # 定时器
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info(f'机器人代理 {robot_id} 已启动')

    def odom_callback(self, msg: Odometry):
        """更新位置"""
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y
        
        # 计算yaw
        q = msg.pose.pose.orientation
        self.position['yaw'] = math.atan2(
            2.0 * (q.w * q.z), 1.0 - 2.0 * q.z * q.z)

    def publish_status(self):
        """发布状态"""
        status = {
            'robot_id': self.robot_id,
            'position': self.position,
            'battery': self.battery,
            'status': self.status,
            'current_task': self.current_task
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def task_callback(self, msg: String):
        """接收任务分配"""
        assignment = json.loads(msg.data)
        
        if assignment['robot_id'] != self.robot_id:
            return
        
        self.execute_task(assignment)

    def auction_callback(self, msg: String):
        """参与拍卖"""
        auction = json.loads(msg.data)
        
        if auction['type'] == 'auction_start':
            self.submit_bid(auction['task'])
        elif auction['type'] == 'auction_result':
            if auction['winner'] == self.robot_id:
                self.get_logger().info('赢得拍卖!')

    def submit_bid(self, task):
        """提交投标"""
        # 计算投标值（距离）
        dx = self.position['x'] - task['location']['x']
        dy = self.position['y'] - task['location']['y']
        distance = math.sqrt(dx*dx + dy*dy)
        
        # 考虑电量因素
        bid_value = distance * (1 + (100 - self.battery) * 0.01)
        
        bid = {
            'robot_id': self.robot_id,
            'task_id': task['id'],
            'value': bid_value
        }
        msg = String()
        msg.data = json.dumps(bid)
        self.bid_pub.publish(msg)
        
        self.get_logger().info(f'投标任务 {task["id"]}: {bid_value:.2f}')

    def execute_task(self, task):
        """执行任务"""
        self.status = 'busy'
        self.current_task = task['task_id']
        
        self.get_logger().info(f'执行任务: {task["task_id"]}')
        
        # 发送导航目标
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = task['location']['x']
        goal_msg.pose.pose.position.y = task['location']['y']
        
        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.nav_response_callback)

    def nav_response_callback(self, future):
        """导航响应"""
        goal_handle = future.result()
        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.nav_result_callback)
        else:
            self.status = 'error'
            self.current_task = None

    def nav_result_callback(self, future):
        """导航完成"""
        self.status = 'idle'
        self.current_task = None
        self.get_logger().info('任务完成')


def main(args=None):
    rclpy.init(args=args)
    
    import sys
    robot_id = sys.argv[1] if len(sys.argv) > 1 else 'robot1'
    
    agent = RobotAgent(robot_id)
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 12.3 编队控制

### 12.3.1 编队形状定义

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
from enum import Enum

class FormationType(Enum):
    LINE = "line"
    COLUMN = "column"
    TRIANGLE = "triangle"
    DIAMOND = "diamond"
    CIRCLE = "circle"

class FormationController(Node):
    """
    多机器人编队控制器
    """
    def __init__(self):
        super().__init__('formation_controller')
        
        # 编队参数
        self.declare_parameter('formation_type', 'triangle')
        self.declare_parameter('spacing', 2.0)
        self.declare_parameter('leader_id', 'robot1')
        
        # 机器人列表
        self.robots = ['robot1', 'robot2', 'robot3', 'robot4']
        self.robot_poses = {}
        self.robot_vel_pubs = {}
        
        # 订阅每个机器人的位置
        for robot in self.robots:
            self.create_subscription(
                Odometry, f'/{robot}/odom',
                lambda msg, r=robot: self.odom_callback(msg, r),
                10
            )
        
        # 发布速度命令
        for robot in self.robots:
            self.robot_vel_pubs[robot] = self.create_publisher(
                Twist, f'/{robot}/cmd_vel', 10)
        
        # 领航者目标订阅
        self.target_sub = self.create_subscription(
            PoseStamped, '/formation_target',
            self.target_callback, 10)
        
        # 编队目标
        self.formation_target = None
        
        # 控制参数
        self.kp_linear = 0.5
        self.kp_angular = 2.0
        self.max_linear = 0.5
        self.max_angular = 1.0
        
        # 定时控制
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('编队控制器已启动')

    def odom_callback(self, msg: Odometry, robot_id):
        """更新机器人位置"""
        self.robot_poses[robot_id] = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': self.quaternion_to_yaw(msg.pose.pose.orientation)
        }

    def quaternion_to_yaw(self, q):
        return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * q.z * q.z)

    def target_callback(self, msg: PoseStamped):
        """更新编队目标"""
        self.formation_target = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'yaw': self.quaternion_to_yaw(msg.pose.orientation)
        }

    def get_formation_offsets(self, formation_type, spacing):
        """获取编队偏移量"""
        if formation_type == FormationType.LINE:
            return [
                (0.0, 0.0),
                (0.0, spacing),
                (0.0, -spacing),
                (0.0, 2*spacing)
            ]
        elif formation_type == FormationType.COLUMN:
            return [
                (0.0, 0.0),
                (-spacing, 0.0),
                (-2*spacing, 0.0),
                (-3*spacing, 0.0)
            ]
        elif formation_type == FormationType.TRIANGLE:
            return [
                (0.0, 0.0),
                (-spacing, spacing/2),
                (-spacing, -spacing/2),
                (-2*spacing, 0.0)
            ]
        elif formation_type == FormationType.DIAMOND:
            return [
                (0.0, 0.0),
                (-spacing, spacing),
                (-spacing, -spacing),
                (-2*spacing, 0.0)
            ]
        elif formation_type == FormationType.CIRCLE:
            offsets = []
            for i, robot in enumerate(self.robots):
                angle = 2 * math.pi * i / len(self.robots)
                offsets.append((
                    -spacing * math.cos(angle),
                    spacing * math.sin(angle)
                ))
            return offsets
        else:
            return [(0.0, 0.0)] * len(self.robots)

    def control_loop(self):
        """编队控制主循环"""
        if not self.formation_target:
            return
        
        leader_id = self.get_parameter('leader_id').value
        if leader_id not in self.robot_poses:
            return
        
        formation_type = FormationType(self.get_parameter('formation_type').value)
        spacing = self.get_parameter('spacing').value
        
        leader_pose = self.robot_poses[leader_id]
        leader_yaw = leader_pose['yaw']
        
        offsets = self.get_formation_offsets(formation_type, spacing)
        
        for i, robot in enumerate(self.robots):
            if robot not in self.robot_poses:
                continue
            
            if robot == leader_id:
                # 领航者直接跟踪目标
                self.track_target(robot, self.formation_target)
            else:
                # 跟随者保持编队位置
                offset = offsets[i]
                
                # 计算目标位置（相对于领航者）
                target_x = leader_pose['x'] + \
                    offset[0] * math.cos(leader_yaw) - \
                    offset[1] * math.sin(leader_yaw)
                target_y = leader_pose['y'] + \
                    offset[0] * math.sin(leader_yaw) + \
                    offset[1] * math.cos(leader_yaw)
                
                target = {'x': target_x, 'y': target_y}
                self.track_target(robot, target)

    def track_target(self, robot_id, target):
        """跟踪目标位置"""
        current = self.robot_poses[robot_id]
        
        # 计算误差
        dx = target['x'] - current['x']
        dy = target['y'] - current['y']
        distance = math.sqrt(dx*dx + dy*dy)
        
        # 计算目标朝向
        target_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(target_yaw - current['yaw'])
        
        # 计算速度命令
        cmd = Twist()
        
        if distance > 0.1:
            cmd.linear.x = min(self.max_linear, self.kp_linear * distance)
            cmd.angular.z = max(-self.max_angular, 
                min(self.max_angular, self.kp_angular * yaw_error))
        
        self.robot_vel_pubs[robot_id].publish(cmd)

    def normalize_angle(self, angle):
        """归一化角度到 [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def change_formation(self, formation_type: FormationType):
        """切换编队形状"""
        self.set_parameters([
            rclpy.parameter.Parameter(
                'formation_type', 
                rclpy.parameter.Parameter.Type.STRING, 
                formation_type.value
            )
        ])
        self.get_logger().info(f'切换编队: {formation_type.value}')


def main(args=None):
    rclpy.init(args=args)
    controller = FormationController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 12.3.2 虚拟结构法

```python
class VirtualStructureController(Node):
    """
    基于虚拟结构的编队控制
    """
    def __init__(self):
        super().__init__('virtual_structure_controller')
        
        # 虚拟结构参数
        self.structure_center = [0.0, 0.0]
        self.structure_yaw = 0.0
        self.structure_velocity = [0.0, 0.0]
        
        # 机器人在结构中的相对位置
        self.relative_positions = {
            'robot1': [0.5, 0.0],
            'robot2': [0.0, 0.5],
            'robot3': [0.0, -0.5],
            'robot4': [-0.5, 0.0]
        }
        
        # 控制增益
        self.k_struct = 1.0  # 结构保持增益
        self.k_center = 0.5  # 中心跟踪增益
        
    def compute_virtual_structure(self):
        """计算虚拟结构位置"""
        # 更新虚拟结构中心位置
        # 这里可以根据任务需求更新
        
        # 计算每个机器人的目标位置
        targets = {}
        for robot, rel_pos in self.relative_positions.items():
            target_x = self.structure_center[0] + \
                rel_pos[0] * math.cos(self.structure_yaw) - \
                rel_pos[1] * math.sin(self.structure_yaw)
            target_y = self.structure_center[1] + \
                rel_pos[0] * math.sin(self.structure_yaw) + \
                rel_pos[1] * math.cos(self.structure_yaw)
            targets[robot] = [target_x, target_y]
        
        return targets
```

## 12.4 冲突检测与解决

### 12.4.1 碰撞避免

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
import math
import numpy as np

class CollisionAvoidance(Node):
    """
    多机器人碰撞避免系统
    """
    def __init__(self):
        super().__init__('collision_avoidance')
        
        # 机器人参数
        self.robots = ['robot1', 'robot2', 'robot3']
        self.robot_radius = 0.3  # 机器人半径
        self.safety_margin = 0.2  # 安全边距
        
        # 机器人状态
        self.robot_states = {}
        
        # 订阅
        for robot in self.robots:
            self.create_subscription(
                Odometry, f'/{robot}/odom',
                lambda msg, r=robot: self.odom_callback(msg, r),
                10
            )
            self.create_subscription(
                Twist, f'/{robot}/cmd_vel_raw',
                lambda msg, r=robot: self.cmd_vel_callback(msg, r),
                10
            )
        
        # 发布
        self.cmd_vel_pubs = {}
        for robot in self.robots:
            self.cmd_vel_pubs[robot] = self.create_publisher(
                Twist, f'/{robot}/cmd_vel', 10)
        
        # 可视化
        self.marker_pub = self.create_publisher(
            MarkerArray, '/collision_markers', 10)
        
        self.timer = self.create_timer(0.1, self.check_collisions)
        
        self.get_logger().info('碰撞避免系统已启动')

    def odom_callback(self, msg: Odometry, robot_id):
        """更新机器人状态"""
        self.robot_states[robot_id] = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': self.quaternion_to_yaw(msg.pose.pose.orientation),
            'vx': msg.twist.twist.linear.x,
            'vy': msg.twist.twist.linear.y,
            'vyaw': msg.twist.twist.angular.z,
            'cmd_vel': Twist()  # 将在 cmd_vel_callback 中更新
        }

    def cmd_vel_callback(self, msg: Twist, robot_id):
        """接收原始速度命令"""
        if robot_id in self.robot_states:
            self.robot_states[robot_id]['cmd_vel'] = msg

    def quaternion_to_yaw(self, q):
        return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * q.z * q.z)

    def check_collisions(self):
        """检测并解决碰撞"""
        if len(self.robot_states) < 2:
            return
        
        # 检测所有机器人对
        for i, robot1 in enumerate(self.robots):
            for robot2 in self.robots[i+1:]:
                if robot1 not in self.robot_states or robot2 not in self.robot_states:
                    continue
                
                state1 = self.robot_states[robot1]
                state2 = self.robot_states[robot2]
                
                # 计算距离
                dx = state1['x'] - state2['x']
                dy = state1['y'] - state2['y']
                distance = math.sqrt(dx*dx + dy*dy)
                
                # 安全距离
                safe_distance = 2 * self.robot_radius + self.safety_margin
                
                if distance < safe_distance:
                    # 碰撞风险，调整速度
                    self.resolve_collision(robot1, robot2, distance, safe_distance)

    def resolve_collision(self, robot1, robot2, distance, safe_distance):
        """解决碰撞"""
        state1 = self.robot_states[robot1]
        state2 = self.robot_states[robot2]
        
        # 计算排斥方向
        dx = state1['x'] - state2['x']
        dy = state1['y'] - state2['y']
        norm = math.sqrt(dx*dx + dy*dy)
        if norm < 0.01:
            dx, dy = 1.0, 0.0
        else:
            dx, dy = dx/norm, dy/norm
        
        # 计算排斥力
        repulsion = (safe_distance - distance) / safe_distance
        
        # 调整 robot1 速度
        cmd1 = Twist()
        cmd1.linear.x = state1['cmd_vel'].linear.x + dx * repulsion * 0.3
        cmd1.linear.y = state1['cmd_vel'].linear.y + dy * repulsion * 0.3
        cmd1.angular.z = state1['cmd_vel'].angular.z
        
        # 调整 robot2 速度
        cmd2 = Twist()
        cmd2.linear.x = state2['cmd_vel'].linear.x - dx * repulsion * 0.3
        cmd2.linear.y = state2['cmd_vel'].linear.y - dy * repulsion * 0.3
        cmd2.angular.z = state2['cmd_vel'].angular.z
        
        # 发布调整后的速度
        self.cmd_vel_pubs[robot1].publish(cmd1)
        self.cmd_vel_pubs[robot2].publish(cmd2)
        
        self.get_logger().warn(
            f'碰撞风险: {robot1} - {robot2}, 距离: {distance:.2f}m'
        )

    def predict_collision(self, robot1, robot2, time_horizon=2.0):
        """预测未来碰撞"""
        state1 = self.robot_states[robot1]
        state2 = self.robot_states[robot2]
        
        # 预测未来位置
        for t in np.arange(0, time_horizon, 0.1):
            future_x1 = state1['x'] + state1['vx'] * t
            future_y1 = state1['y'] + state1['vy'] * t
            future_x2 = state2['x'] + state2['vx'] * t
            future_y2 = state2['y'] + state2['vy'] * t
            
            distance = math.sqrt(
                (future_x1 - future_x2)**2 + 
                (future_y1 - future_y2)**2
            )
            
            if distance < 2 * self.robot_radius + self.safety_margin:
                return True, t
        
        return False, 0.0


class ORCA(CollisionAvoidance):
    """
    ORCA (Optimal Reciprocal Collision Avoidance) 算法
    """
    def __init__(self):
        super().__init__()
        
        self.time_horizon = 2.0  # 时间视野
        
    def compute_orca_velocity(self, robot_id):
        """计算 ORCA 速度"""
        if robot_id not in self.robot_states:
            return Twist()
        
        state = self.robot_states[robot_id]
        current_vel = np.array([state['vx'], state['vy']])
        
        # 计算与其他机器人的 ORCA 约束
        orca_planes = []
        
        for other_robot, other_state in self.robot_states.items():
            if other_robot == robot_id:
                continue
            
            # 相对位置和速度
            relative_pos = np.array([
                other_state['x'] - state['x'],
                other_state['y'] - state['y']
            ])
            relative_vel = current_vel - np.array([
                other_state['vx'], other_state['vy']
            ])
            
            distance = np.linalg.norm(relative_pos)
            combined_radius = 2 * self.robot_radius + self.safety_margin
            
            if distance < combined_radius:
                # 已经碰撞
                direction = -relative_pos / distance
                new_vel = direction * 0.1
                cmd = Twist()
                cmd.linear.x = new_vel[0]
                cmd.linear.y = new_vel[1]
                return cmd
            
            # 计算 ORCA 平面
            inv_time_horizon = 1.0 / self.time_horizon
            
            if distance < combined_radius * inv_time_horizon:
                u = relative_pos / distance * combined_radius * inv_time_horizon - relative_vel
            else:
                u = relative_pos / distance * combined_radius * inv_time_horizon - relative_vel
            
            orca_planes.append(u / 2)
        
        # 找到满足所有约束的最优速度
        new_vel = current_vel
        for u in orca_planes:
            new_vel = new_vel - u * 0.5
        
        cmd = Twist()
        cmd.linear.x = np.clip(new_vel[0], -0.5, 0.5)
        cmd.linear.y = np.clip(new_vel[1], -0.5, 0.5)
        
        return cmd


def main(args=None):
    rclpy.init(args=args)
    orca = ORCA()
    rclpy.spin(orca)
    orca.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 12.5 分布式共识

### 12.5.1 一致性协议

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import hashlib

class DistributedConsensus(Node):
    """
    分布式一致性协议实现
    """
    def __init__(self, robot_id):
        super().__init__(f'{robot_id}_consensus')
        
        self.robot_id = robot_id
        self.peers = []  # 其他机器人列表
        
        # 共识状态
        self.proposal_id = 0
        self.current_value = None
        self.promised_id = 0
        self.accepted_id = 0
        self.accepted_value = None
        
        # 投票记录
        self.votes = {}
        
        # 订阅
        self.consensus_sub = self.create_subscription(
            String, '/consensus',
            self.consensus_callback, 10)
        
        # 发布
        self.consensus_pub = self.create_publisher(String, '/consensus', 10)
        
        self.get_logger().info(f'共识节点 {robot_id} 已启动')

    def propose(self, value):
        """发起提案"""
        self.proposal_id += 1
        proposal = {
            'type': 'prepare',
            'proposal_id': self.proposal_id,
            'proposer': self.robot_id,
            'value': value
        }
        self.publish_message(proposal)
        self.get_logger().info(f'发起提案: {self.proposal_id}')

    def consensus_callback(self, msg: String):
        """处理共识消息"""
        message = json.loads(msg.data)
        
        if message['type'] == 'prepare':
            self.handle_prepare(message)
        elif message['type'] == 'promise':
            self.handle_promise(message)
        elif message['type'] == 'accept':
            self.handle_accept(message)
        elif message['type'] == 'accepted':
            self.handle_accepted(message)

    def handle_prepare(self, message):
        """处理准备请求"""
        if message['proposal_id'] > self.promised_id:
            self.promised_id = message['proposal_id']
            
            response = {
                'type': 'promise',
                'proposal_id': message['proposal_id'],
                'responder': self.robot_id,
                'accepted_id': self.accepted_id,
                'accepted_value': self.accepted_value
            }
            self.publish_message(response)

    def handle_promise(self, message):
        """处理承诺响应"""
        if message['proposal_id'] == self.proposal_id:
            self.votes[message['responder']] = message
            
            # 检查是否获得多数
            if len(self.votes) > len(self.peers) / 2:
                # 发送接受请求
                accept_msg = {
                    'type': 'accept',
                    'proposal_id': self.proposal_id,
                    'proposer': self.robot_id,
                    'value': self.current_value
                }
                self.publish_message(accept_msg)

    def handle_accept(self, message):
        """处理接受请求"""
        if message['proposal_id'] >= self.promised_id:
            self.accepted_id = message['proposal_id']
            self.accepted_value = message['value']
            
            response = {
                'type': 'accepted',
                'proposal_id': message['proposal_id'],
                'acceptor': self.robot_id,
                'value': message['value']
            }
            self.publish_message(response)

    def handle_accepted(self, message):
        """处理接受确认"""
        self.get_logger().info(
            f"共识达成: proposal_id={message['proposal_id']}, "
            f"value={message['value']}"
        )

    def publish_message(self, message):
        """发布消息"""
        msg = String()
        msg.data = json.dumps(message)
        self.consensus_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    import sys
    robot_id = sys.argv[1] if len(sys.argv) > 1 else 'robot1'
    
    consensus = DistributedConsensus(robot_id)
    rclpy.spin(consensus)
    consensus.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 12.6 多机器人 Launch 配置

### 12.6.1 Launch 文件

```python
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    num_robots = LaunchConfiguration('num_robots')
    
    declare_num_robots = DeclareLaunchArgument(
        'num_robots',
        default_value='3',
        description='Number of robots'
    )
    
    # 任务分配器
    task_allocator = Node(
        package='multi_robot',
        executable='task_allocator',
        name='task_allocator',
        output='screen'
    )
    
    # 编队控制器
    formation_controller = Node(
        package='multi_robot',
        executable='formation_controller',
        name='formation_controller',
        output='screen'
    )
    
    # 碰撞避免
    collision_avoidance = Node(
        package='multi_robot',
        executable='collision_avoidance',
        name='collision_avoidance',
        output='screen'
    )
    
    # 机器人节点
    robot_nodes = []
    for i in range(3):
        robot_id = f'robot{i+1}'
        
        robot_group = GroupAction([
            PushRosNamespace(robot_id),
            
            # 机器人代理
            Node(
                package='multi_robot',
                executable='robot_agent',
                name='robot_agent',
                arguments=[robot_id],
                output='screen'
            ),
            
            # 导航
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[{'controller_frequency': 20.0}]
            ),
        ])
        
        robot_nodes.append(robot_group)
    
    return LaunchDescription([
        declare_num_robots,
        task_allocator,
        formation_controller,
        collision_avoidance,
        *robot_nodes
    ])
```

---


