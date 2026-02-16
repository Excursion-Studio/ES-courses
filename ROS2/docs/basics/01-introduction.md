# 第1章：ROS2 概述与环境搭建

## 1.1 什么是 ROS2

ROS2 (Robot Operating System 2) 是一个用于编写机器人软件的灵活框架。尽管名字中包含"操作系统"，但 ROS2 实际上是一套中间件，运行在 Linux、Windows、macOS 等操作系统之上。

### 1.1.1 ROS2 的发展历程

| 版本 | 发布时间 | 代号 | 支持状态 |
|------|----------|------|----------|
| Ardent | 2017年12月 | Ardent Apalone | 已停止支持 |
| Bouncy | 2018年7月 | Bouncy Bolson | 已停止支持 |
| Crystal | 2018年12月 | Crystal Clemmys | 已停止支持 |
| Dashing | 2019年5月 | Dashing Diademata | 已停止支持 |
| Eloquent | 2019年11月 | Elous Elusor | 已停止支持 |
| Foxy | 2020年6月 | Foxy Fitzroy | 已停止支持 |
| Galactic | 2021年5月 | Galactic Geochelone | 已停止支持 |
| **Humble** | 2022年5月 | Humble Hawksbill | **LTS (2027年5月)** |
| Iron | 2023年5月 | Iron Irwini | 支持中 (2024年11月) |
| **Jazzy** | 2024年5月 | Jazzy Jalisco | **LTS (2029年5月)** |

### 1.1.2 ROS2 vs ROS1 对比

| 特性 | ROS1 | ROS2 |
|------|------|------|
| 通信中间件 | 自定义 TCPROS/UDPROS | DDS (Data Distribution Service) |
| 实时性 | 不支持 | 支持 |
| 跨平台 | 仅 Linux | Linux/Windows/macOS |
| 节点生命周期 | 无 | 支持托管节点 |
| 安全性 | 无内置支持 | DDS-Security |
| QoS 配置 | 无 | 多种 QoS 策略 |
| 多机器人 | 困难 | 原生支持 |

## 1.2 ROS2 架构概述

### 1.2.1 分层架构

<div align="center">
<svg width="600" height="320" viewBox="0 0 600 320" xmlns="http://www.w3.org/2000/svg">
  
  <!-- 背景 -->
  <rect x="10" y="10" width="580" height="300" fill="#f8f9fa" stroke="#dee2e6" stroke-width="1" rx="4"/>
  
  <!-- 应用层 -->
  <rect x="30" y="30" width="540" height="55" fill="#e3f2fd" stroke="#1976d2" stroke-width="2" rx="5"/>
  <text x="300" y="52" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#1565c0">应用层 (Application Layer)</text>
  <text x="300" y="72" font-family="system-ui, -apple-system, sans-serif" font-size="12" text-anchor="middle" fill="#424242">节点、话题、服务、动作、参数</text>
  
  <!-- 中间件层 -->
  <rect x="30" y="100" width="540" height="55" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="2" rx="5"/>
  <text x="300" y="122" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#6a1b9a">中间件层 (Middleware Layer)</text>
  <text x="300" y="142" font-family="monospace" font-size="12" text-anchor="middle" fill="#424242">rclcpp / rclpy (Client Library)</text>
  
  <!-- 通信层 -->
  <rect x="30" y="170" width="540" height="55" fill="#e8f5e9" stroke="#388e3c" stroke-width="2" rx="5"/>
  <text x="300" y="192" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#2e7d32">通信层 (Communication Layer)</text>
  <text x="300" y="212" font-family="monospace" font-size="12" text-anchor="middle" fill="#424242">DDS / RMW (ROS Middleware)</text>
  
  <!-- 操作系统层 -->
  <rect x="30" y="240" width="540" height="55" fill="#fff3e0" stroke="#f57c00" stroke-width="2" rx="5"/>
  <text x="300" y="262" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#ef6c00">操作系统层 (OS Layer)</text>
  <text x="300" y="282" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#424242">Linux / Windows / macOS / RTOS</text>
</svg>
</div>

### 1.2.2 核心概念

- **节点 (Node)**：执行计算的基本单元
- **话题 (Topic)**：节点间异步通信的通道
- **服务 (Service)**：节点间同步通信的方式
- **动作 (Action)**：长时间任务的通信方式
- **参数 (Parameter)**：节点的配置数据

## 1.3 环境安装

### 1.3.1 Ubuntu 安装 (推荐)

#### 系统要求
- Ubuntu 22.04 LTS (ROS2 Humble)
- Ubuntu 24.04 LTS (ROS2 Jazzy)

#### 安装步骤

```bash
# 1. 设置语言环境
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. 添加 ROS2 仓库
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. 安装 ROS2
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop

# 4. 安装开发工具
sudo apt install python3-argcomplete python3-colcon-common-extensions

# 5. 环境配置
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 1.3.2 Windows 安装

#### 系统要求
- Windows 10/11 64位

#### 安装步骤

```powershell
# 1. 安装 Chocolatey (Windows 包管理器)
# 以管理员身份运行 PowerShell
Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))

# 2. 安装必要工具
choco install -y python git cmake

# 3. 下载并安装 ROS2
# 访问 https://github.com/ros2/ros2/releases 下载 Windows 安装包

# 4. 配置环境
# 运行 ROS2 命令提示符快捷方式
```

## 1.4 开发环境配置

### 1.4.1 VS Code 配置

#### 推荐扩展

1. **ROS** - Microsoft 官方 ROS 扩展
2. **Python** - Python 语言支持
3. **C/C++** - C++ 语言支持
4. **CMake** - CMake 语法支持
5. **CMake Tools** - CMake 构建工具

#### settings.json 配置

```json
{
    "ros.distro": "humble",
    "python.analysis.extraPaths": [
        "/opt/ros/humble/lib/python3.10/site-packages"
    ],
    "C_Cpp.intelliSenseEngine": "default",
    "cmake.configureOnOpen": false
}
```

### 1.4.2 创建工作空间

```bash
# 创建工作空间目录
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# 构建工作空间
colcon build

# 加载工作空间环境
source install/setup.bash
```

### 1.4.3 创建功能包

```bash
# 进入 src 目录
cd ~/ros2_ws/src

# 创建 Python 功能包
ros2 pkg create --build-type ament_python my_python_pkg

# 创建 C++ 功能包
ros2 pkg create --build-type ament_cmake my_cpp_pkg

# 创建包含节点示例的功能包
ros2 pkg create --build-type ament_python --node-name my_node my_pkg
```

## 1.5 验证安装

### 1.5.1 运行示例节点

```bash
# 终端1：运行发布者节点
ros2 run demo_nodes_cpp talker

# 终端2：运行订阅者节点
ros2 run demo_nodes_cpp listener
```

### 1.5.2 检查环境

```bash
# 查看 ROS2 版本
ros2 --version

# 查看环境变量
printenv | grep -i ROS

# 列出可用功能包
ros2 pkg list
```

## 1.6 常见问题

### Q1: 找不到 ros2 命令

确保已加载 ROS2 环境：
```bash
source /opt/ros/humble/setup.bash
```

### Q2: colcon build 失败

检查依赖是否完整：
```bash
rosdep install --from-paths src --ignore-src -y
```

### Q3: VS Code 无法识别 ROS2 包

确保 `settings.json` 中正确配置了 Python 路径和 ROS 发行版。

---


