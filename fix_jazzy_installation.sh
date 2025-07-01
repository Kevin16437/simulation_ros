#!/bin/bash

# 修复ROS2 Jazzy安装脚本
echo "=== 修复ROS2 Jazzy安装 ==="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}1. 修复GPG密钥问题${NC}"
# 重新添加ROS2 GPG密钥
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# 或者使用备用方法
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654

echo -e "\n${BLUE}2. 更新包列表${NC}"
sudo apt update

echo -e "\n${BLUE}3. 设置正确的ROS2环境${NC}"
# 确保使用正确的ROS2版本
export ROS_DISTRO=jazzy
source /opt/ros/jazzy/setup.bash

# 更新bashrc
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi

echo -e "\n${BLUE}4. 安装缺失的包${NC}"
# 安装基本的ROS2包
sudo apt install -y \
    ros-jazzy-desktop \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-rviz2

# 尝试安装Gazebo相关包（可能名称不同）
echo -e "\n${BLUE}5. 安装Gazebo相关包${NC}"
sudo apt install -y \
    ros-jazzy-gazebo-ros \
    ros-jazzy-gazebo-plugins \
    ros-jazzy-gazebo-msgs || echo -e "${YELLOW}某些Gazebo包可能不可用${NC}"

# 安装控制相关包
sudo apt install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager || echo -e "${YELLOW}某些控制包可能不可用${NC}"

echo -e "\n${BLUE}6. 安装开发工具${NC}"
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    cmake

echo -e "\n${BLUE}7. 修复rosdep（使用本地方法）${NC}"
# 如果网络有问题，我们手动创建rosdep配置
sudo mkdir -p /etc/ros/rosdep/sources.list.d/

# 创建本地rosdep配置
sudo tee /etc/ros/rosdep/sources.list.d/20-default.list > /dev/null << 'EOF'
# os-specific listings first
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/osx-homebrew.yaml osx

# generic
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml
gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/releases/fuerte.yaml fuerte

# newer distributions: Groovy, Hydro, Indigo, Jade, Kinetic, Lunar, Melodic
gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/releases/groovy.yaml groovy
gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/releases/hydro.yaml hydro
gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/releases/indigo.yaml indigo
gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/releases/jade.yaml jade
gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/releases/kinetic.yaml kinetic
gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/releases/lunar.yaml lunar
gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/releases/melodic.yaml melodic

# ROS 2 distributions
gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/releases/dashing.yaml dashing
gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/releases/eloquent.yaml eloquent
gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/releases/foxy.yaml foxy
gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/releases/galactic.yaml galactic
gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/releases/humble.yaml humble
gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/releases/iron.yaml iron
gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/releases/jazzy.yaml jazzy
EOF

# 尝试更新rosdep
rosdep update || echo -e "${YELLOW}rosdep更新可能失败，但可以继续${NC}"

echo -e "\n${BLUE}8. 验证安装${NC}"
source /opt/ros/jazzy/setup.bash

# 检查关键包
echo "检查已安装的包..."
if ros2 pkg list | grep -q xacro; then
    echo -e "${GREEN}✓${NC} xacro 已安装"
else
    echo -e "${RED}✗${NC} xacro 未安装"
fi

if ros2 pkg list | grep -q robot_state_publisher; then
    echo -e "${GREEN}✓${NC} robot_state_publisher 已安装"
else
    echo -e "${RED}✗${NC} robot_state_publisher 未安装"
fi

echo -e "\n${GREEN}=== 修复完成 ===${NC}"
echo -e "${YELLOW}请运行以下命令重新加载环境:${NC}"
echo "source ~/.bashrc"
echo ""
echo -e "${BLUE}然后可以尝试构建项目:${NC}"
echo "cd ~/ros2_ws"
echo "source /opt/ros/jazzy/setup.bash"
echo "colcon build --packages-select arm6"
