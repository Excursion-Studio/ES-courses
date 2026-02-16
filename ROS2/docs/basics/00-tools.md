# 第0章：开发工具基础

在开始 ROS2 开发之前，需要掌握一些基础工具。本章介绍 Linux 系统、Shell 脚本、CMake 构建工具和 Docker 容器技术。

## 0.1 Linux 基础

### 0.1.1 Linux 发行版选择

ROS2 主要支持以下 Linux 发行版：

| ROS2 版本 | 推荐系统 | 支持状态 |
|-----------|----------|----------|
| Humble | Ubuntu 22.04 LTS | LTS (2027年5月) |
| Iron | Ubuntu 22.04 LTS | 2024年11月 |
| Jazzy | Ubuntu 24.04 LTS | LTS (2029年5月) |

### 0.1.2 文件系统结构

```
/                    # 根目录
├── home/            # 用户主目录
│   └── username/    # 用户目录
│       ├── ros2_ws/ # ROS2 工作空间
│       └── .bashrc  # Shell 配置文件
├── opt/ros/         # ROS 安装目录
│   └── humble/      # ROS2 Humble
├── usr/             # 系统程序
│   ├── bin/         # 可执行文件
│   ├── lib/         # 库文件
│   └── include/     # 头文件
├── etc/             # 系统配置
└── tmp/             # 临时文件
```

### 0.1.3 常用命令

#### 文件操作

```bash
# 查看当前目录
pwd

# 列出文件
ls -la

# 切换目录
cd /path/to/directory
cd ~        # 回到主目录
cd ..       # 返回上级目录

# 创建目录
mkdir -p path/to/directory

# 创建文件
touch filename.txt

# 复制文件/目录
cp source.txt dest.txt
cp -r source_dir/ dest_dir/

# 移动/重命名
mv old_name.txt new_name.txt
mv file.txt /new/path/

# 删除文件/目录
rm file.txt
rm -r directory/
rm -rf directory/  # 强制删除（谨慎使用）

# 查看文件内容
cat file.txt
less file.txt
head -n 20 file.txt
tail -n 20 file.txt

# 查找文件
find /path -name "*.txt"
locate filename

# 搜索文件内容
grep "pattern" file.txt
grep -r "pattern" directory/
```

#### 权限管理

```bash
# 查看权限
ls -la

# 修改权限
chmod 755 script.sh      # 数字方式
chmod +x script.sh       # 添加执行权限
chmod u+x script.sh      # 用户添加执行权限
chmod g-w file.txt       # 组移除写权限

# 修改所有者
chown user:group file.txt
chown -R user:group directory/

# 使用 sudo 执行特权命令
sudo apt update
sudo nano /etc/hosts
```

#### 进程管理

```bash
# 查看进程
ps aux
ps aux | grep ros2

# 实时进程监控
top
htop

# 终止进程
kill PID
kill -9 PID              # 强制终止
killall process_name

# 后台运行
ros2 run my_pkg node &   # 后台运行
nohup command &          # 忽略挂断信号

# 查看端口占用
netstat -tulpn
lsof -i :port_number
```

### 0.1.4 环境变量

```bash
# 查看环境变量
echo $PATH
echo $ROS_DISTRO
printenv | grep ROS

# 设置环境变量（临时）
export MY_VAR="value"
export PATH=$PATH:/new/path

# 设置环境变量（永久）
# 编辑 ~/.bashrc
echo 'export MY_VAR="value"' >> ~/.bashrc
source ~/.bashrc

# ROS2 环境配置
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## 0.2 Shell 脚本

### 0.2.1 Shell 基础

```bash
#!/bin/bash
# 这是一个 Shell 脚本示例

# 变量定义（等号两边不能有空格）
NAME="ROS2"
VERSION="Humble"
COUNT=10

# 使用变量
echo "Welcome to $NAME $VERSION"
echo "Count: ${COUNT}"

# 数组
ROBOTS=("robot1" "robot2" "robot3")
echo "First robot: ${ROBOTS[0]}"
echo "All robots: ${ROBOTS[@]}"
echo "Array length: ${#ROBOTS[@]}"

# 命令替换
CURRENT_DATE=$(date +%Y-%m-%d)
CURRENT_DIR=$(pwd)
echo "Today: $CURRENT_DATE"
echo "Directory: $CURRENT_DIR"
```

### 0.2.2 条件判断

```bash
#!/bin/bash

# if 语句
NUM=10

if [ $NUM -gt 5 ]; then
    echo "Number is greater than 5"
elif [ $NUM -eq 5 ]; then
    echo "Number is equal to 5"
else
    echo "Number is less than 5"
fi

# 字符串比较
STR="hello"

if [ "$STR" = "hello" ]; then
    echo "String matches"
fi

if [ -n "$STR" ]; then
    echo "String is not empty"
fi

# 文件判断
FILE="/path/to/file"

if [ -f "$FILE" ]; then
    echo "File exists"
fi

if [ -d "$FILE" ]; then
    echo "Directory exists"
fi

# 常用判断符
# -eq  等于
# -ne  不等于
# -gt  大于
# -lt  小于
# -ge  大于等于
# -le  小于等于
# -f   文件存在
# -d   目录存在
# -z   字符串为空
# -n   字符串非空
```

### 0.2.3 循环

```bash
#!/bin/bash

# for 循环
for i in 1 2 3 4 5; do
    echo "Number: $i"
done

# for 循环（范围）
for i in {1..10}; do
    echo "Count: $i"
done

# for 循环（C风格）
for ((i=0; i<10; i++)); do
    echo "Index: $i"
done

# 遍历数组
ROBOTS=("robot1" "robot2" "robot3")
for robot in "${ROBOTS[@]}"; do
    echo "Robot: $robot"
done

# while 循环
COUNT=0
while [ $COUNT -lt 5 ]; do
    echo "Count: $COUNT"
    COUNT=$((COUNT + 1))
done

# 读取文件
while read line; do
    echo "Line: $line"
done < input.txt
```

### 0.2.4 函数

```bash
#!/bin/bash

# 定义函数
greet() {
    local name=$1
    echo "Hello, $name!"
}

# 带返回值的函数
add() {
    local a=$1
    local b=$2
    local sum=$((a + b))
    echo $sum
}

# 调用函数
greet "ROS2"

result=$(add 5 3)
echo "5 + 3 = $result"

# 带默认参数的函数
launch_robot() {
    local name=${1:-"default_robot"}
    local speed=${2:-1.0}
    echo "Launching $name with speed $speed"
}

launch_robot
launch_robot "robot1" 2.0
```

### 0.2.5 ROS2 常用脚本示例

```bash
#!/bin/bash
# build_workspace.sh - 构建工作空间

WORKSPACE_DIR="${1:-$HOME/ros2_ws}"

echo "Building workspace: $WORKSPACE_DIR"

# 检查目录是否存在
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "Error: Workspace directory not found"
    exit 1
fi

cd "$WORKSPACE_DIR" || exit 1

# 加载环境
source /opt/ros/humble/setup.bash

# 构建
colcon build --symlink-install

# 检查构建结果
if [ $? -eq 0 ]; then
    echo "Build successful!"
    echo "Remember to source: source $WORKSPACE_DIR/install/setup.bash"
else
    echo "Build failed!"
    exit 1
fi
```

```bash
#!/bin/bash
# launch_simulation.sh - 启动仿真环境

# 参数解析
WORLD="${1:-empty}"
ROBOTS="${2:-1}"

echo "Launching simulation..."
echo "World: $WORLD"
echo "Robots: $ROBOTS"

# 加载环境
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 启动 Gazebo
ros2 launch my_simulation simulation.launch.py \
    world:="$WORLD" \
    num_robots:="$ROBOTS"
```

## 0.3 CMake 构建工具

### 0.3.1 CMake 基础

CMake 是 ROS2 C++ 项目的主要构建工具。

#### 基本 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_project)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 添加可执行文件
add_executable(my_node src/main.cpp)

# 链接依赖
ament_target_dependencies(my_node
    rclcpp
    std_msgs
)

# 安装目标
install(TARGETS
    my_node
    DESTINATION lib/${PROJECT_NAME}
)

# 安装目录
install(DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### 0.3.2 编译目标类型

```cmake
# 可执行文件
add_executable(my_node src/main.cpp)

# 静态库
add_library(my_lib STATIC src/lib.cpp)

# 动态库
add_library(my_lib SHARED src/lib.cpp)

# 头文件库
add_library(my_header_lib INTERFACE)
target_include_directories(my_header_lib INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
```

### 0.3.3 依赖管理

```cmake
# 查找包
find_package(PkgConfig REQUIRED)
pkg_check_modules(OPENCV REQUIRED opencv4)

# 包含目录
target_include_directories(my_target
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# 链接库
target_link_libraries(my_target
    ${OPENCV_LIBRARIES}
)

# 编译定义
target_compile_definitions(my_target
    PRIVATE
        MY_DEFINE=1
)

# 编译选项
target_compile_options(my_target
    PRIVATE
        -O3
        -Werror
)
```

### 0.3.4 ROS2 CMake 配置

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_ros2_package)

# 默认为 Release 构建
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# C++ 标准
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 注册组件
add_library(my_component SHARED
    src/my_component.cpp
)

target_include_directories(my_component PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(my_component
    rclcpp
    rclcpp_components
    std_msgs
    geometry_msgs
)

rclcpp_components_register_node(my_component
    PLUGIN "my_package::MyNode"
    EXECUTABLE my_node
)

# 安装
install(TARGETS my_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

install(DIRECTORY
    launch
    config
    urdf
    meshes
    worlds
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### 0.3.5 条件编译

```cmake
# 选项
option(BUILD_TESTING "Build tests" ON)
option(USE_OPENCV "Use OpenCV" OFF)

# 条件编译
if(USE_OPENCV)
    find_package(OpenCV REQUIRED)
    target_compile_definitions(my_target PRIVATE USE_OPENCV)
    target_link_libraries(my_target ${OpenCV_LIBS})
endif()

# 平台判断
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    message(STATUS "Building for x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    message(STATUS "Building for ARM64")
endif()

# Debug/Release 区分
target_compile_definitions(my_target
    PUBLIC
    $<$<CONFIG:Debug>:DEBUG_MODE>
    $<$<CONFIG:Release>:NDEBUG>
)
```

### 0.3.6 自定义 CMake 模块

创建 `cmake/FindMyLibrary.cmake`：

```cmake
# FindMyLibrary.cmake
# 查找 MyLibrary

find_path(MYLIBRARY_INCLUDE_DIR
    NAMES mylibrary.h
    PATHS /usr/local/include /usr/include
)

find_library(MYLIBRARY_LIBRARY
    NAMES mylibrary
    PATHS /usr/local/lib /usr/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MyLibrary
    REQUIRED_VARS MYLIBRARY_LIBRARY MYLIBRARY_INCLUDE_DIR
)

if(MyLibrary_FOUND)
    set(MyLibrary_LIBRARIES ${MYLIBRARY_LIBRARY})
    set(MyLibrary_INCLUDE_DIRS ${MYLIBRARY_INCLUDE_DIR})
endif()

mark_as_advanced(MYLIBRARY_INCLUDE_DIR MYLIBRARY_LIBRARY)
```

使用自定义模块：

```cmake
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(MyLibrary REQUIRED)
```

## 0.4 Docker 容器技术

### 0.4.1 Docker 基础概念

<div align="center">
<svg width="650" height="380" viewBox="0 0 650 380" xmlns="http://www.w3.org/2000/svg">
  <defs>
    <marker id="arrowhead" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
      <polygon points="0 0, 10 3.5, 0 7" fill="#495057"/>
    </marker>
  </defs>
  
  <!-- 背景 -->
  <rect x="10" y="10" width="630" height="360" fill="#f8f9fa" stroke="#dee2e6" stroke-width="1" rx="4"/>
  <text x="325" y="38" font-family="system-ui, -apple-system, sans-serif" font-size="18" font-weight="600" text-anchor="middle" fill="#212529">Docker 架构</text>
  
  <!-- Docker Host 外框 -->
  <rect x="40" y="55" width="570" height="290" fill="#e3f2fd" stroke="#1976d2" stroke-width="2" rx="6"/>
  <text x="325" y="80" font-family="system-ui, -apple-system, sans-serif" font-size="16" font-weight="600" text-anchor="middle" fill="#1565c0">Docker Host</text>
  
  <!-- Container 1 - ROS2 -->
  <rect x="70" y="105" width="140" height="100" fill="#ffffff" stroke="#0288d1" stroke-width="2" rx="5"/>
  <text x="140" y="135" font-family="system-ui, -apple-system, sans-serif" font-size="15" font-weight="600" text-anchor="middle" fill="#0277bd">Container</text>
  <rect x="85" y="150" width="110" height="40" fill="#e1f5fe" stroke="#0288d1" stroke-width="1" rx="3"/>
  <text x="140" y="175" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#01579b">ROS2</text>
  
  <!-- Container 2 - Nav2 -->
  <rect x="255" y="105" width="140" height="100" fill="#ffffff" stroke="#7b1fa2" stroke-width="2" rx="5"/>
  <text x="325" y="135" font-family="system-ui, -apple-system, sans-serif" font-size="15" font-weight="600" text-anchor="middle" fill="#6a1b9a">Container</text>
  <rect x="270" y="150" width="110" height="40" fill="#f3e5f5" stroke="#7b1fa2" stroke-width="1" rx="3"/>
  <text x="325" y="175" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#4a148c">Nav2</text>
  
  <!-- Container 3 - Gazebo -->
  <rect x="440" y="105" width="140" height="100" fill="#ffffff" stroke="#388e3c" stroke-width="2" rx="5"/>
  <text x="510" y="135" font-family="system-ui, -apple-system, sans-serif" font-size="15" font-weight="600" text-anchor="middle" fill="#2e7d32">Container</text>
  <rect x="455" y="150" width="110" height="40" fill="#e8f5e9" stroke="#388e3c" stroke-width="1" rx="3"/>
  <text x="510" y="175" font-family="system-ui, -apple-system, sans-serif" font-size="14" font-weight="600" text-anchor="middle" fill="#1b5e20">Gazebo</text>
  
  <!-- 连接线 -->
  <line x1="140" y1="205" x2="140" y2="230" stroke="#495057" stroke-width="1.5" stroke-dasharray="5,3"/>
  <line x1="325" y1="205" x2="325" y2="230" stroke="#495057" stroke-width="1.5" stroke-dasharray="5,3"/>
  <line x1="510" y1="205" x2="510" y2="230" stroke="#495057" stroke-width="1.5" stroke-dasharray="5,3"/>
  
  <!-- Docker Daemon -->
  <rect x="70" y="230" width="510" height="90" fill="#37474f" stroke="#263238" stroke-width="2" rx="5"/>
  <text x="325" y="260" font-family="system-ui, -apple-system, sans-serif" font-size="17" font-weight="600" text-anchor="middle" fill="#ffffff">Docker Daemon</text>
  
  <!-- Daemon 内部组件 -->
  <rect x="100" y="275" width="100" height="35" fill="#546e7a" stroke="#455a64" stroke-width="1" rx="3"/>
  <text x="150" y="297" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#ffffff">镜像管理</text>
  
  <rect x="225" y="275" width="100" height="35" fill="#546e7a" stroke="#455a64" stroke-width="1" rx="3"/>
  <text x="275" y="297" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#ffffff">容器管理</text>
  
  <rect x="350" y="275" width="100" height="35" fill="#546e7a" stroke="#455a64" stroke-width="1" rx="3"/>
  <text x="400" y="297" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#ffffff">网络管理</text>
  
  <rect x="475" y="275" width="80" height="35" fill="#546e7a" stroke="#455a64" stroke-width="1" rx="3"/>
  <text x="515" y="297" font-family="system-ui, -apple-system, sans-serif" font-size="11" text-anchor="middle" fill="#ffffff">存储</text>
</svg>
</div>

### 0.4.2 Docker 安装

```bash
# Ubuntu 安装 Docker
sudo apt update
sudo apt install -y ca-certificates curl gnupg

# 添加 Docker 官方 GPG 密钥
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | \
    sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# 添加 Docker 仓库
echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/etc/apt/keyrings/docker.gpg] \
    https://download.docker.com/linux/ubuntu \
    $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
    sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# 安装 Docker
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io \
    docker-buildx-plugin docker-compose-plugin

# 添加用户到 docker 组
sudo usermod -aG docker $USER

# 验证安装
docker --version
docker run hello-world
```

### 0.4.3 Docker 基本命令

```bash
# 镜像操作
docker pull ros:humble                     # 拉取镜像
docker images                              # 列出镜像
docker rmi image_id                        # 删除镜像
docker build -t my_image:tag .             # 构建镜像

# 容器操作
docker run -it ros:humble bash            # 运行容器
docker run -d --name my_container image   # 后台运行
docker ps                                  # 列出运行中的容器
docker ps -a                               # 列出所有容器
docker stop container_id                   # 停止容器
docker rm container_id                     # 删除容器
docker exec -it container_id bash          # 进入容器

# 容器管理
docker start container_id                  # 启动容器
docker restart container_id                # 重启容器
docker logs container_id                   # 查看日志
docker cp file.txt container_id:/path/     # 复制文件到容器
docker cp container_id:/path/file.txt ./   # 从容器复制文件

# 清理
docker system prune                        # 清理未使用的资源
docker volume prune                        # 清理未使用的卷
```

### 0.4.4 ROS2 Dockerfile

```dockerfile
# ROS2 Humble 开发环境
FROM ros:humble

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# 安装基础工具
RUN apt-get update && apt-get install -y \
    vim \
    git \
    cmake \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 安装 ROS2 工具
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-moveit \
    && rm -rf /var/lib/apt/lists/*

# 创建工作空间
WORKDIR /ros2_ws

# 设置环境
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# 默认命令
CMD ["bash"]
```

### 0.4.5 Docker Compose

创建 `docker-compose.yml`：

```yaml
version: '3.8'

services:
  ros2:
    image: ros:humble
    container_name: ros2_dev
    privileged: true
    network_mode: host
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=0
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - ./ros2_ws:/ros2_ws
      - ~/.ssh:/root/.ssh:ro
    working_dir: /ros2_ws
    command: bash
    
  gazebo:
    image: ros:humble
    container_name: gazebo_sim
    privileged: true
    network_mode: host
    environment:
      - DISPLAY=${DISPLAY}
      - GAZEBO_MODEL_PATH=/models
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - ./models:/models
    command: >
      bash -c "source /opt/ros/humble/setup.bash &&
               ros2 launch gazebo_ros gazebo.launch.py"
    
  nav2:
    image: ros:humble
    container_name: nav2
    privileged: true
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=0
    volumes:
      - ./config:/config
    command: >
      bash -c "source /opt/ros/humble/setup.bash &&
               ros2 launch nav2_bringup bringup_launch.py"
```

使用 Docker Compose：

```bash
# 启动所有服务
docker compose up -d

# 启动特定服务
docker compose up -d ros2

# 查看服务状态
docker compose ps

# 查看日志
docker compose logs ros2

# 停止服务
docker compose down

# 重新构建
docker compose build
```

### 0.4.6 ROS2 多容器配置

```yaml
version: '3.8'

services:
  # 主控节点
  master:
    build: ./docker/master
    container_name: ros2_master
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=0
    command: ros2 run demo_nodes_cpp talker
    
  # 机器人1
  robot1:
    build: ./docker/robot
    container_name: robot1
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=0
      - ROBOT_NAME=robot1
    command: >
      bash -c "source /opt/ros/humble/setup.bash &&
               ros2 run my_pkg robot_node"
    
  # 机器人2
  robot2:
    build: ./docker/robot
    container_name: robot2
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=0
      - ROBOT_NAME=robot2
    command: >
      bash -c "source /opt/ros/humble/setup.bash &&
               ros2 run my_pkg robot_node"
    
  # 可视化
  rviz:
    image: ros:humble
    container_name: rviz
    network_mode: host
    environment:
      - DISPLAY=${DISPLAY}
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - ./config:/config
    command: rviz2
```

### 0.4.7 Docker 开发最佳实践

#### 多阶段构建

```dockerfile
# 构建阶段
FROM ros:humble AS builder

WORKDIR /ros2_ws
COPY src/ src/

RUN . /opt/ros/humble/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 运行阶段
FROM ros:humble

WORKDIR /ros2_ws

COPY --from=builder /ros2_ws/install /ros2_ws/install

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
```

#### 开发环境 Dockerfile

```dockerfile
FROM ros:humble

# 安装开发工具
RUN apt-get update && apt-get install -y \
    gdb \
    valgrind \
    clang-format \
    python3-debugpy \
    && rm -rf /var/lib/apt/lists/*

# 配置开发环境
RUN echo "alias cb='colcon build --symlink-install'" >> ~/.bashrc && \
    echo "alias st='source install/setup.bash'" >> ~/.bashrc

# 挂载工作空间
VOLUME ["/ros2_ws"]
WORKDIR /ros2_ws
```

#### 运行带 GPU 支持

```bash
# 使用 NVIDIA Docker
docker run -it --rm \
    --gpus all \
    --network host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    ros:humble \
    bash
```

## 0.5 开发环境配置清单

### 0.5.1 推荐软件

| 类别 | 软件 | 用途 |
|------|------|------|
| IDE | VS Code | 代码编辑 |
| IDE | CLion | C++ 开发 |
| 终端 | Terminator | 多终端管理 |
| 终端 | tmux | 终端复用 |
| 版本控制 | Git | 代码管理 |
| 文档 | Typora | Markdown 编辑 |

### 0.5.2 VS Code 扩展

```
- ROS (Microsoft)
- Python (Microsoft)
- C/C++ (Microsoft)
- CMake (twxs)
- CMake Tools (Microsoft)
- Docker (Microsoft)
- YAML (Red Hat)
- GitLens (GitKraken)
```

### 0.5.3 常用别名配置

```bash
# 添加到 ~/.bashrc

# ROS2 别名
alias sb='source ~/.bashrc'
alias sws='source ~/ros2_ws/install/setup.bash'
alias cb='cd ~/ros2_ws && colcon build --symlink-install'
alias cbs='cd ~/ros2_ws && colcon build --symlink-install --packages-select'
alias ct='colcon test && colcon test-result --verbose'

# Docker 别名
alias dps='docker ps'
alias dpsa='docker ps -a'
alias di='docker images'
alias drm='docker rm $(docker ps -aq)'
alias drmi='docker rmi $(docker images -q)'
alias dprune='docker system prune -af'

# Git 别名
alias gs='git status'
alias ga='git add'
alias gc='git commit -m'
alias gp='git push'
alias gl='git log --oneline --graph'
```

---


