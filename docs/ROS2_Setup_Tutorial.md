# ROS2机械臂仿真完整配置教程

## 1. WSL2 Ubuntu 24.04 环境准备

### 1.1 确保WSL2已安装并运行Ubuntu 24.04
```bash
# 检查WSL版本
wsl --version

# 如果需要安装Ubuntu 24.04
wsl --install -d Ubuntu-24.04
```

### 1.2 更新系统
```bash
sudo apt update && sudo apt upgrade -y
```

## 2. ROS2 Humble安装

### 2.1 自动安装（推荐）
```bash
# 使用提供的安装脚本
chmod +x setup_ros2_workspace.sh
./setup_ros2_workspace.sh
```

### 2.2 手动安装步骤
```bash
# 1. 安装必要工具
sudo apt install -y curl gnupg2 lsb-release software-properties-common

# 2. 添加ROS2仓库
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. 更新包列表
sudo apt update

# 4. 安装ROS2 Humble
sudo apt install -y ros-humble-desktop-full

# 5. 安装额外包
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-rviz2 \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool
```

## 3. 工作空间设置

### 3.1 创建工作空间
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 3.2 初始化rosdep
```bash
sudo rosdep init
rosdep update
```

### 3.3 设置环境变量
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 4. 机械臂包配置

### 4.1 复制arm6包到工作空间
```bash
# 将arm6文件夹复制到工作空间
cp -r /path/to/arm6 ~/ros2_ws/src/

# 或者如果在Windows中，使用WSL路径
cp -r /mnt/f/F\ Download/simulation/arm6 ~/ros2_ws/src/
```

### 4.2 更新包配置文件
```bash
cd ~/ros2_ws/src/arm6

# 替换为ROS2版本的配置文件
mv package_ros2.xml package.xml
mv CMakeLists_ros2.txt CMakeLists.txt
```

## 5. 构建和测试

### 5.1 安装依赖
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 5.2 构建包
```bash
colcon build --packages-select arm6
source install/setup.bash
```

### 5.3 测试URDF可视化
```bash
# 启动RViz2显示机械臂
ros2 launch arm6 display_ros2.launch.py
```

### 5.4 测试Gazebo仿真
```bash
# 启动Gazebo仿真
ros2 launch arm6 gazebo_ros2.launch.py
```

## 6. 常见问题解决

### 6.1 如果遇到权限问题
```bash
sudo chown -R $USER:$USER ~/ros2_ws
```

### 6.2 如果Gazebo启动失败
```bash
# 安装Gazebo Classic
sudo apt install -y gazebo
```

### 6.3 如果找不到包
```bash
# 重新source环境
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

## 7. 下一步操作

安装完成后，您可以：
1. 在RViz2中可视化机械臂
2. 在Gazebo中进行物理仿真
3. 使用ros2_control控制机械臂关节
4. 编写自定义控制节点
