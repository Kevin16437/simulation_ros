#!/bin/bash

# ARM6机械臂ROS2配置测试脚本
# 此脚本用于测试和验证ARM6机械臂的ROS2配置

echo "=== ARM6机械臂ROS2配置测试 ==="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查函数
check_command() {
    if command -v $1 &> /dev/null; then
        echo -e "${GREEN}✓${NC} $1 已安装"
        return 0
    else
        echo -e "${RED}✗${NC} $1 未找到"
        return 1
    fi
}

check_ros_package() {
    if ros2 pkg list | grep -q $1; then
        echo -e "${GREEN}✓${NC} ROS2包 $1 已安装"
        return 0
    else
        echo -e "${RED}✗${NC} ROS2包 $1 未找到"
        return 1
    fi
}

# 1. 检查基本环境
echo -e "\n${YELLOW}1. 检查基本环境${NC}"
check_command "ros2"
check_command "gazebo"
check_command "rviz2"
check_command "colcon"

# 2. 检查ROS2包
echo -e "\n${YELLOW}2. 检查ROS2包${NC}"
check_ros_package "robot_state_publisher"
check_ros_package "joint_state_publisher_gui"
check_ros_package "gazebo_ros_pkgs"
check_ros_package "ros2_control"
check_ros_package "ros2_controllers"

# 3. 检查工作空间
echo -e "\n${YELLOW}3. 检查工作空间${NC}"
if [ -d "$HOME/ros2_ws" ]; then
    echo -e "${GREEN}✓${NC} ROS2工作空间存在"
    if [ -d "$HOME/ros2_ws/src/arm6" ]; then
        echo -e "${GREEN}✓${NC} arm6包存在于工作空间"
    else
        echo -e "${RED}✗${NC} arm6包不存在于工作空间"
        echo "请将arm6包复制到 ~/ros2_ws/src/"
    fi
else
    echo -e "${RED}✗${NC} ROS2工作空间不存在"
    echo "请运行 mkdir -p ~/ros2_ws/src"
fi

# 4. 检查URDF文件
echo -e "\n${YELLOW}4. 检查URDF文件${NC}"
if [ -f "$HOME/ros2_ws/src/arm6/urdf/arm6.urdf" ]; then
    echo -e "${GREEN}✓${NC} URDF文件存在"
    
    # 检查URDF语法
    if command -v check_urdf &> /dev/null; then
        echo "检查URDF语法..."
        if check_urdf $HOME/ros2_ws/src/arm6/urdf/arm6.urdf; then
            echo -e "${GREEN}✓${NC} URDF语法正确"
        else
            echo -e "${RED}✗${NC} URDF语法错误"
        fi
    else
        echo -e "${YELLOW}!${NC} check_urdf工具未安装，跳过语法检查"
    fi
else
    echo -e "${RED}✗${NC} URDF文件不存在"
fi

# 5. 检查配置文件
echo -e "\n${YELLOW}5. 检查配置文件${NC}"
config_files=(
    "package.xml"
    "CMakeLists.txt"
    "launch/display_ros2.launch.py"
    "launch/gazebo_ros2.launch.py"
    "config/arm_controllers.yaml"
    "config/arm6.rviz"
)

for file in "${config_files[@]}"; do
    if [ -f "$HOME/ros2_ws/src/arm6/$file" ]; then
        echo -e "${GREEN}✓${NC} $file 存在"
    else
        echo -e "${RED}✗${NC} $file 不存在"
    fi
done

# 6. 构建测试
echo -e "\n${YELLOW}6. 构建测试${NC}"
cd $HOME/ros2_ws
source /opt/ros/humble/setup.bash

echo "正在构建arm6包..."
if colcon build --packages-select arm6 --symlink-install; then
    echo -e "${GREEN}✓${NC} arm6包构建成功"
else
    echo -e "${RED}✗${NC} arm6包构建失败"
    exit 1
fi

# 7. 功能测试
echo -e "\n${YELLOW}7. 功能测试${NC}"
source install/setup.bash

# 测试包是否可以找到
if ros2 pkg list | grep -q arm6; then
    echo -e "${GREEN}✓${NC} arm6包可以被ROS2找到"
else
    echo -e "${RED}✗${NC} arm6包无法被ROS2找到"
fi

# 测试launch文件
echo "测试launch文件语法..."
if ros2 launch arm6 display_ros2.launch.py --show-args &> /dev/null; then
    echo -e "${GREEN}✓${NC} display_ros2.launch.py 语法正确"
else
    echo -e "${RED}✗${NC} display_ros2.launch.py 语法错误"
fi

if ros2 launch arm6 gazebo_ros2.launch.py --show-args &> /dev/null; then
    echo -e "${RED}✗${NC} gazebo_ros2.launch.py 语法错误（这是正常的，因为需要Gazebo运行）"
fi

echo -e "\n${YELLOW}=== 测试完成 ===${NC}"
echo -e "\n${YELLOW}下一步操作：${NC}"
echo "1. 如果所有测试通过，可以运行："
echo "   ros2 launch arm6 display_ros2.launch.py"
echo ""
echo "2. 启动Gazebo仿真："
echo "   ros2 launch arm6 gazebo_ros2.launch.py"
echo ""
echo "3. 控制机械臂关节："
echo "   ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller"
