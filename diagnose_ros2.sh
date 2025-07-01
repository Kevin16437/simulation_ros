#!/bin/bash

# ROS2诊断脚本
echo "=== ROS2安装诊断 ==="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}1. 检查操作系统版本${NC}"
lsb_release -a

echo -e "\n${BLUE}2. 检查ROS2命令${NC}"
if command -v ros2 &> /dev/null; then
    echo -e "${GREEN}✓${NC} ros2命令存在"
    which ros2
    
    # 尝试获取版本信息
    echo "尝试获取ROS2版本信息..."
    ros2 --help | head -5
else
    echo -e "${RED}✗${NC} ros2命令不存在"
fi

echo -e "\n${BLUE}3. 检查可能的ROS2安装路径${NC}"
possible_paths=(
    "/opt/ros/humble"
    "/opt/ros/iron"
    "/opt/ros/jazzy"
    "/opt/ros/galactic"
    "/opt/ros/foxy"
)

for path in "${possible_paths[@]}"; do
    if [ -d "$path" ]; then
        echo -e "${GREEN}✓${NC} 找到ROS2安装: $path"
        if [ -f "$path/setup.bash" ]; then
            echo -e "${GREEN}  ✓${NC} setup.bash存在"
        else
            echo -e "${RED}  ✗${NC} setup.bash不存在"
        fi
    else
        echo -e "${YELLOW}!${NC} 未找到: $path"
    fi
done

echo -e "\n${BLUE}4. 检查环境变量${NC}"
echo "ROS_DISTRO: ${ROS_DISTRO:-未设置}"
echo "ROS_VERSION: ${ROS_VERSION:-未设置}"
echo "AMENT_PREFIX_PATH: ${AMENT_PREFIX_PATH:-未设置}"

echo -e "\n${BLUE}5. 检查已安装的ROS2包${NC}"
if dpkg -l | grep -q ros-; then
    echo -e "${GREEN}✓${NC} 找到ROS2相关包:"
    dpkg -l | grep ros- | head -10
    echo "..."
    echo "总计: $(dpkg -l | grep ros- | wc -l) 个ROS2包"
else
    echo -e "${RED}✗${NC} 未找到ROS2包"
fi

echo -e "\n${BLUE}6. 检查特定包${NC}"
packages_to_check=(
    "ros-humble-desktop"
    "ros-humble-xacro"
    "ros-humble-robot-state-publisher"
    "ros-humble-gazebo-ros-pkgs"
    "ros-humble-ros2-control"
)

for pkg in "${packages_to_check[@]}"; do
    if dpkg -l | grep -q "$pkg"; then
        echo -e "${GREEN}✓${NC} $pkg 已安装"
    else
        echo -e "${RED}✗${NC} $pkg 未安装"
    fi
done

echo -e "\n${BLUE}7. 检查rosdep${NC}"
if command -v rosdep &> /dev/null; then
    echo -e "${GREEN}✓${NC} rosdep已安装"
    if [ -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
        echo -e "${GREEN}✓${NC} rosdep已初始化"
    else
        echo -e "${RED}✗${NC} rosdep未初始化"
    fi
else
    echo -e "${RED}✗${NC} rosdep未安装"
fi

echo -e "\n${BLUE}8. 检查colcon${NC}"
if command -v colcon &> /dev/null; then
    echo -e "${GREEN}✓${NC} colcon已安装"
    colcon version
else
    echo -e "${RED}✗${NC} colcon未安装"
fi

echo -e "\n${BLUE}=== 诊断完成 ===${NC}"
