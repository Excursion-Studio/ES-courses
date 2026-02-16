# 第6章：动作 (Action)

## 6.1 动作概念

动作 (Action) 是 ROS2 中用于处理长时间任务的通信方式。它结合了话题和服务的特点，支持目标发送、进度反馈和结果返回。

### 6.1.1 动作通信模型

<div align="center">
<svg width="700" height="320" viewBox="0 0 700 320" xmlns="http://www.w3.org/2000/svg">
  <defs>
    <marker id="arrowBlue" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
      <polygon points="0 0, 10 3.5, 0 7" fill="#1976d2"/>
    </marker>
    <marker id="arrowPurple" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
      <polygon points="0 0, 10 3.5, 0 7" fill="#7b1fa2"/>
    </marker>
    <marker id="arrowGreen" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
      <polygon points="0 0, 10 3.5, 0 7" fill="#388e3c"/>
    </marker>
  </defs>
  
  <!-- 背景 -->
  <rect x="10" y="10" width="680" height="300" fill="#f8f9fa" stroke="#dee2e6" stroke-width="1" rx="4"/>
  <text x="350" y="35" font-family="system-ui, -apple-system, sans-serif" font-size="16" font-weight="600" text-anchor="middle" fill="#212529">动作通信 (Action)</text>
  
  <!-- 客户端 -->
  <rect x="40" y="55" width="160" height="90" fill="#e3f2fd" stroke="#1976d2" stroke-width="2" rx="6"/>
  <text x="120" y="85" font-family="system-ui, -apple-system, sans-serif" font-size="15" font-weight="600" text-anchor="middle" fill="#1565c0">客户端</text>
  <text x="120" y="105" font-family="monospace" font-size="11" text-anchor="middle" fill="#424242">(Client)</text>
  <text x="120" y="125" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#757575">动作调用方</text>
  
  <!-- 服务端 -->
  <rect x="500" y="55" width="160" height="90" fill="#fff3e0" stroke="#f57c00" stroke-width="2" rx="6"/>
  <text x="580" y="85" font-family="system-ui, -apple-system, sans-serif" font-size="15" font-weight="600" text-anchor="middle" fill="#ef6c00">服务端</text>
  <text x="580" y="105" font-family="monospace" font-size="11" text-anchor="middle" fill="#424242">(Server)</text>
  <text x="580" y="125" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#757575">动作执行方</text>
  
  <!-- Goal 箭头 -->
  <line x1="220" y1="80" x2="490" y2="80" stroke="#1976d2" stroke-width="2.5" marker-end="url(#arrowBlue)"/>
  <text x="355" y="70" font-family="system-ui, -apple-system, sans-serif" font-size="12" font-weight="600" text-anchor="middle" fill="#1976d2">目标 (Goal)</text>
  
  <!-- Feedback 箭头 -->
  <line x1="490" y1="110" x2="220" y2="110" stroke="#7b1fa2" stroke-width="2" marker-end="url(#arrowPurple)"/>
  <line x1="490" y1="125" x2="220" y2="125" stroke="#7b1fa2" stroke-width="2" marker-end="url(#arrowPurple)"/>
  <text x="355" y="145" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#7b1fa2">反馈 (Feedback) × N</text>
  
  <!-- Result 箭头 -->
  <line x1="490" y1="170" x2="220" y2="170" stroke="#388e3c" stroke-width="2.5" marker-end="url(#arrowGreen)"/>
  <text x="355" y="190" font-family="system-ui, -apple-system, sans-serif" font-size="12" font-weight="600" text-anchor="middle" fill="#388e3c">结果 (Result)</text>
  
  <!-- 任务执行框 -->
  <rect x="510" y="200" width="140" height="50" fill="#e8f5e9" stroke="#388e3c" stroke-width="1.5" rx="4"/>
  <text x="580" y="220" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#2e7d32">执行任务</text>
  <text x="580" y="238" font-family="system-ui, -apple-system, sans-serif" font-size="10" text-anchor="middle" fill="#757575">长时间运行</text>
  
  <!-- 任务连接线 -->
  <line x1="580" y1="145" x2="580" y2="200" stroke="#495057" stroke-width="1.5" stroke-dasharray="4,2"/>
  
  <!-- 特性标签 -->
  <rect x="40" y="220" width="130" height="35" fill="#fce4ec" stroke="#c2185b" stroke-width="1.5" rx="4"/>
  <text x="105" y="243" font-family="system-ui, -apple-system, sans-serif" font-size="11" font-weight="600" text-anchor="middle" fill="#ad1457">异步 + 可取消</text>
  
  <rect x="190" y="220" width="130" height="35" fill="#e1f5fe" stroke="#0288d1" stroke-width="1.5" rx="4"/>
  <text x="255" y="243" font-family="system-ui, -apple-system, sans-serif" font-size="11" font-weight="600" text-anchor="middle" fill="#0277bd">进度反馈</text>
  
  <rect x="340" y="220" width="130" height="35" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="1.5" rx="4"/>
  <text x="405" y="243" font-family="system-ui, -apple-system, sans-serif" font-size="11" font-weight="600" text-anchor="middle" fill="#6a1b9a">目标追踪</text>
</svg>
</div>

### 6.1.2 动作 vs 服务 vs 话题

| 特性 | 话题 | 服务 | 动作 |
|------|------|------|------|
| 通信模式 | 异步 | 同步 | 异步 |
| 数据流向 | 单向 | 双向 | 双向 |
| 反馈机制 | 无 | 无 | 有 |
| 可取消 | - | 否 | 是 |
| 适用场景 | 数据流 | 快速操作 | 长时间任务 |

### 6.1.3 适用场景

- ✅ 导航到目标点
- ✅ 机械臂抓取物体
- ✅ 执行巡检任务
- ✅ 长时间计算任务
- ❌ 快速查询（用服务）
- ❌ 持续数据流（用话题）

## 6.2 动作定义

### 6.2.1 动作文件结构

动作定义文件 (`.action`) 包含三部分：

```
# 目标 (Goal)
目标数据类型
---
# 结果 (Result)
结果数据类型
---
# 反馈 (Feedback)
反馈数据类型
```

### 6.2.2 标准动作类型

```bash
# 列出所有动作类型
ros2 interface list | grep action

# 查看动作定义
ros2 interface show nav2_msgs/action/NavigateToPose
```

### 6.2.3 自定义动作

创建文件 `my_interfaces/action/CountTo.action`：

```
# 目标 (Goal)
int64 target_number        # 目标数字
float64 delay              # 延迟时间(秒)
---
# 结果 (Result)
int64 final_count          # 最终计数
bool success               # 是否成功
string message             # 状态消息
---
# 反馈 (Feedback)
int64 current_count        # 当前进度
float64 progress           # 完成百分比
```

### 6.2.4 配置功能包

#### CMakeLists.txt

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "srv/GetRobotPose.srv"
  "action/CountTo.action"
)
```

#### package.xml

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

## 6.3 动作服务器与客户端

### 6.3.1 Python 实现

#### 动作服务器

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from my_interfaces.action import CountTo
import time

class CountToServer(Node):
    def __init__(self):
        super().__init__('count_to_server')
        
        self._action_server = ActionServer(
            self,
            CountTo,
            'count_to',
            self.execute_callback
        )
        
        self.get_logger().info('动作服务器已启动: count_to')

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'收到目标: 数到 {goal_handle.request.target_number}')
        
        target = goal_handle.request.target_number
        delay = goal_handle.request.delay
        
        feedback_msg = CountTo.Feedback()
        result_msg = CountTo.Result()
        
        for i in range(1, target + 1):
            # 检查是否被取消
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result_msg.final_count = i - 1
                result_msg.success = False
                result_msg.message = '动作被取消'
                self.get_logger().info('目标被取消')
                return result_msg
            
            # 发布反馈
            feedback_msg.current_count = i
            feedback_msg.progress = (i / target) * 100.0
            goal_handle.publish_feedback(feedback_msg)
            
            self.get_logger().info(f'进度: {i}/{target} ({feedback_msg.progress:.1f}%)')
            
            time.sleep(delay)
        
        # 返回结果
        goal_handle.succeed()
        result_msg.final_count = target
        result_msg.success = True
        result_msg.message = '计数完成'
        
        return result_msg

def main(args=None):
    rclpy.init(args=args)
    server = CountToServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 动作客户端

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from my_interfaces.action import CountTo

class CountToClient(Node):
    def __init__(self):
        super().__init__('count_to_client')
        
        self._action_client = ActionClient(self, CountTo, 'count_to')

    def send_goal(self, target, delay):
        goal_msg = CountTo.Goal()
        goal_msg.target_number = target
        goal_msg.delay = delay
        
        self._action_client.wait_for_server()
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            return
        
        self.get_logger().info('目标被接受')
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            f'结果: 最终计数={result.final_count}, '
            f'成功={result.success}, 消息={result.message}'
        )
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'反馈: 当前进度={feedback.current_count}, '
            f'完成={feedback.progress:.1f}%'
        )

def main(args=None):
    rclpy.init(args=args)
    
    client = CountToClient()
    client.send_goal(target=10, delay=0.5)
    
    rclpy.spin(client)

if __name__ == '__main__':
    main()
```

### 6.3.2 C++ 实现

#### 动作服务器

```cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_interfaces/action/count_to.hpp"
#include <thread>

class CountToServer : public rclcpp::Node
{
public:
    using CountTo = my_interfaces::action::CountTo;
    using GoalHandleCountTo = rclcpp_action::ServerGoalHandle<CountTo>;

    CountToServer() : Node("count_to_server")
    {
        auto handle_goal = [this](
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const CountTo::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "收到目标: 数到 %ld", goal->target_number);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        };

        auto handle_cancel = [this](
            const std::shared_ptr<GoalHandleCountTo> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "收到取消请求");
            return rclcpp_action::CancelResponse::ACCEPT;
        };

        auto handle_accepted = [this](
            const std::shared_ptr<GoalHandleCountTo> goal_handle)
        {
            std::thread{std::bind(&CountToServer::execute, this, std::placeholders::_1), goal_handle}.detach();
        };

        this->action_server_ = rclcpp_action::create_server<CountTo>(
            this,
            "count_to",
            handle_goal,
            handle_cancel,
            handle_accepted);
        
        RCLCPP_INFO(this->get_logger(), "动作服务器已启动: count_to");
    }

private:
    void execute(const std::shared_ptr<GoalHandleCountTo> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<CountTo::Feedback>();
        auto result = std::make_shared<CountTo::Result>();
        
        for (int64_t i = 1; i <= goal->target_number; i++)
        {
            if (goal_handle->is_canceling())
            {
                result->final_count = i - 1;
                result->success = false;
                result->message = "动作被取消";
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "目标被取消");
                return;
            }
            
            feedback->current_count = i;
            feedback->progress = (static_cast<double>(i) / goal->target_number) * 100.0;
            goal_handle->publish_feedback(feedback);
            
            RCLCPP_INFO(this->get_logger(), "进度: %ld/%ld (%.1f%%)", 
                i, goal->target_number, feedback->progress);
            
            std::this_thread::sleep_for(
                std::chrono::duration<double>(goal->delay));
        }
        
        result->final_count = goal->target_number;
        result->success = true;
        result->message = "计数完成";
        goal_handle->succeed(result);
    }
    
    rclcpp_action::Server<CountTo>::SharedPtr action_server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CountToServer>());
    rclcpp::shutdown();
    return 0;
}
```

#### 动作客户端

```cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_interfaces/action/count_to.hpp"

class CountToClient : public rclcpp::Node
{
public:
    using CountTo = my_interfaces::action::CountTo;

    CountToClient() : Node("count_to_client")
    {
        this->client_ptr_ = rclcpp_action::create_client<CountTo>(this, "count_to");
    }

    void send_goal(int64_t target, double delay)
    {
        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "等待动作服务器超时");
            return;
        }

        auto goal_msg = CountTo::Goal();
        goal_msg.target_number = target;
        goal_msg.delay = delay;

        auto send_goal_options = rclcpp_action::Client<CountTo>::SendGoalOptions();
        send_goal_options.feedback_callback =
            std::bind(&CountToClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&CountToClient::result_callback, this, std::placeholders::_1);

        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "发送目标: 数到 %ld", target);
    }

private:
    void feedback_callback(
        rclcpp_action::ClientGoalHandle<CountTo>::SharedPtr,
        const std::shared_ptr<const CountTo::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "反馈: 当前进度=%ld, 完成=%.1f%%",
            feedback->current_count, feedback->progress);
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<CountTo>::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "结果: 最终计数=%ld, 成功=%s, 消息=%s",
                result.result->final_count,
                result.result->success ? "true" : "false",
                result.result->message.c_str());
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "目标被中止");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "目标被取消");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "未知结果代码");
            break;
        }
        rclcpp::shutdown();
    }
    
    rclcpp_action::Client<CountTo>::SharedPtr client_ptr_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<CountToClient>();
    client->send_goal(10, 0.5);
    rclcpp::spin(client);
    rclcpp::shutdown();
    return 0;
}
```

## 6.4 动作命令行工具

### 6.4.1 常用命令

```bash
# 列出所有动作
ros2 action list

# 查看动作信息
ros2 action info /count_to

# 查看动作类型定义
ros2 interface show my_interfaces/action/CountTo

# 发送动作目标
ros2 action send_goal /count_to my_interfaces/action/CountTo "{target_number: 5, delay: 0.5}"

# 发送目标并显示反馈
ros2 action send_goal --feedback /count_to my_interfaces/action/CountTo "{target_number: 5, delay: 0.5}"
```

### 6.4.2 实际示例

```bash
# 发送导航目标
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}}}}"

# 查看动作状态
ros2 action info /navigate_to_pose
```

## 6.5 实践示例：导航到目标点

### 6.5.1 导航动作定义

创建文件 `my_interfaces/action/Navigate.action`：

```
# 目标 (Goal)
geometry_msgs/PoseStamped target_pose    # 目标位姿
float64 max_speed                        # 最大速度
---
# 结果 (Result)
bool success                             # 是否成功
float64 total_distance                   # 总行驶距离
float64 elapsed_time                     # 耗时
string message                           # 状态消息
---
# 反馈 (Feedback)
float64 distance_remaining               # 剩余距离
geometry_msgs/Pose current_pose          # 当前位姿
float64 speed                            # 当前速度
```

### 6.5.2 简化版导航服务器

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from my_interfaces.action import Navigate
import math
import time

class NavigateServer(Node):
    def __init__(self):
        super().__init__('navigate_server')
        
        self._action_server = ActionServer(
            self,
            Navigate,
            'navigate',
            self.execute_callback
        )
        
        self.get_logger().info('导航服务器已启动')

    def execute_callback(self, goal_handle):
        target = goal_handle.request.target_pose
        max_speed = goal_handle.request.max_speed
        
        self.get_logger().info(
            f'导航到目标: ({target.pose.position.x:.2f}, {target.pose.position.y:.2f})'
        )
        
        feedback_msg = Navigate.Feedback()
        result_msg = Navigate.Result()
        
        # 模拟当前位置
        current_x, current_y = 0.0, 0.0
        target_x = target.pose.position.x
        target_y = target.pose.position.y
        
        start_time = time.time()
        
        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result_msg.success = False
                result_msg.message = '导航被取消'
                return result_msg
            
            # 计算距离
            dx = target_x - current_x
            dy = target_y - current_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < 0.05:  # 到达目标
                break
            
            # 模拟移动
            step = min(max_speed * 0.1, distance)
            ratio = step / distance
            current_x += dx * ratio
            current_y += dy * ratio
            
            # 发布反馈
            feedback_msg.distance_remaining = distance
            feedback_msg.current_pose = PoseStamped()
            feedback_msg.current_pose.pose.position.x = current_x
            feedback_msg.current_pose.pose.position.y = current_y
            feedback_msg.speed = max_speed
            goal_handle.publish_feedback(feedback_msg)
            
            self.get_logger().info(f'剩余距离: {distance:.2f}m')
            time.sleep(0.1)
        
        # 返回结果
        elapsed = time.time() - start_time
        total_dist = math.sqrt(target_x**2 + target_y**2)
        
        goal_handle.succeed()
        result_msg.success = True
        result_msg.total_distance = total_dist
        result_msg.elapsed_time = elapsed
        result_msg.message = '导航完成'
        
        self.get_logger().info(
            f'导航完成: 距离={total_dist:.2f}m, 耗时={elapsed:.2f}s'
        )
        
        return result_msg

def main(args=None):
    rclpy.init(args=args)
    server = NavigateServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 6.5.3 导航客户端

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from my_interfaces.action import Navigate

class NavigateClient(Node):
    def __init__(self):
        super().__init__('navigate_client')
        self._action_client = ActionClient(self, Navigate, 'navigate')

    def send_goal(self, x, y, max_speed=1.0):
        goal_msg = Navigate.Goal()
        goal_msg.target_pose = PoseStamped()
        goal_msg.target_pose.header.frame_id = 'map'
        goal_msg.target_pose.pose.position.x = x
        goal_msg.target_pose.pose.position.y = y
        goal_msg.max_speed = max_speed
        
        self._action_client.wait_for_server()
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            rclpy.shutdown()
            return
        
        self.get_logger().info('目标被接受，开始导航...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            f'导航结果: 成功={result.success}, '
            f'距离={result.total_distance:.2f}m, '
            f'耗时={result.elapsed_time:.2f}s'
        )
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'剩余距离: {feedback.distance_remaining:.2f}m, '
            f'当前速度: {feedback.speed:.2f}m/s'
        )

def main(args=None):
    rclpy.init(args=args)
    client = NavigateClient()
    client.send_goal(x=5.0, y=3.0, max_speed=1.0)
    rclpy.spin(client)

if __name__ == '__main__':
    main()
```

## 6.6 动作最佳实践

### 6.6.1 设计原则

1. **合理的目标粒度**：目标应足够具体且可完成
2. **及时反馈**：定期发布有意义的反馈信息
3. **可取消性**：支持中途取消，清理资源
4. **错误处理**：返回明确的成功/失败状态

### 6.6.2 命名规范

```bash
# 推荐：动词开头
/navigate_to_pose
/follow_path
/grip_object
/execute_trajectory

# 避免：名词
/pose
/path
/object
```

### 6.6.3 常见问题

| 问题 | 解决方案 |
|------|----------|
| 目标被拒绝 | 检查目标参数是否有效 |
| 动作卡住 | 添加超时机制 |
| 反馈不及时 | 减小反馈发布间隔 |
| 取消不响应 | 确保检查 `is_cancel_requested` |

---


