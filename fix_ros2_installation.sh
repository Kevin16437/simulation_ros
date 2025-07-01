#!/bin/bash

# ROS2安装修复脚本
echo "=== ROS2安装修复脚本 ==="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 检查是否为root用户
if [[ $EUID -eq 0 ]]; then
   echo -e "${RED}请不要以root用户运行此脚本${NC}"
   exit 1
fi

echo -e "${BLUE}检测Ubuntu版本...${NC}"
UBUNTU_VERSION=$(lsb_release -rs)
echo "Ubuntu版本: $UBUNTU_VERSION"

# 根据Ubuntu版本确定ROS2版本
if [[ "$UBUNTU_VERSION" == "24.04" ]]; then
    ROS_DISTRO="jazzy"
    echo -e "${GREEN}推荐ROS2版本: Jazzy (Ubuntu 24.04)${NC}"
elif [[ "$UBUNTU_VERSION" == "22.04" ]]; then
    ROS_DISTRO="humble"
    echo -e "${GREEN}推荐ROS2版本: Humble (Ubuntu 22.04)${NC}"
elif [[ "$UBUNTU_VERSION" == "20.04" ]]; then
    ROS_DISTRO="galactic"
    echo -e "${GREEN}推荐ROS2版本: Galactic (Ubuntu 20.04)${NC}"
else
    echo -e "${YELLOW}未知Ubuntu版本，默认使用Humble${NC}"
    ROS_DISTRO="humble"
fi

echo -e "\n${BLUE}1. 更新系统包${NC}"
sudo apt update

echo -e "\n${BLUE}2. 安装必要工具${NC}"
sudo apt install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    ca-certificates

echo -e "\n${BLUE}3. 添加ROS2仓库${NC}"
# 添加ROS2 GPG密钥
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 添加ROS2仓库
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo -e "\n${BLUE}4. 更新包列表${NC}"
sudo apt update

echo -e "\n${BLUE}5. 安装ROS2 ${ROS_DISTRO}${NC}"
sudo apt install -y ros-${ROS_DISTRO}-desktop-full

echo -e "\n${BLUE}6. 安装开发工具${NC}"
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    cmake \
    git

echo -e "\n${BLUE}7. 安装ARM6项目所需的包${NC}"
sudo apt install -y \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-controller-manager

echo -e "\n${BLUE}8. 初始化rosdep${NC}"
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    sudo rosdep init
fi
rosdep update

echo -e "\n${BLUE}9. 设置环境变量${NC}"
# 备份现有的bashrc
cp ~/.bashrc ~/.bashrc.backup.$(date +%Y%m%d_%H%M%S)

# 移除旧的ROS设置
sed -i '/source \/opt\/ros/d' ~/.bashrc

# 添加新的ROS2设置
echo "" >> ~/.bashrc
echo "# ROS2 ${ROS_DISTRO} setup" >> ~/.bashrc
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# 如果工作空间存在，也添加工作空间设置
if [ -d "$HOME/ros2_ws/install" ]; then
    echo "source $HOME/ros2_ws/install/setup.bash" >> ~/.bashrc
fi

echo -e "\n${BLUE}10. 验证安装${NC}"
source /opt/ros/${ROS_DISTRO}/setup.bash

# 测试ROS2命令
if ros2 --help > /dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} ROS2安装成功"
else
    echo -e "${RED}✗${NC} ROS2安装可能有问题"
fi

# 检查关键包
echo "检查关键包..."
packages_to_check=(
    "xacro"
    "robot_state_publisher"
    "gazebo_ros"
)

for pkg in "${packages_to_check[@]}"; do
    if ros2 pkg list | grep -q "$pkg"; then
        echo -e "${GREEN}✓${NC} $pkg 可用"
    else
        echo -e "${RED}✗${NC} $pkg 不可用"
    fi
done

echo -e "\n${GREEN}=== 修复完成 ===${NC}"
echo -e "${YELLOW}请运行以下命令重新加载环境:${NC}"
echo "source ~/.bashrc"
echo ""
echo -e "${YELLOW}或者重新打开终端${NC}"
echo ""
echo -e "${BLUE}然后可以重新运行部署脚本:${NC}"
echo "./deploy_to_wsl2.sh"
