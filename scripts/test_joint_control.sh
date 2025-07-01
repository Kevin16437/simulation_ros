#!/bin/bash

# ARM机械臂关节控制测试脚本
# 用于验证joint_state_publisher_gui是否能正常控制关节

echo "=== ARM机械臂关节控制测试 ==="

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

# 检查ROS2环境
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}错误: ROS2未安装或未正确配置${NC}"
    echo "请先运行 setup_ros2_workspace.sh"
    exit 1
fi

# 检查工作空间
if [ ! -d "$HOME/ros2_ws" ]; then
    echo -e "${RED}错误: ROS2工作空间不存在${NC}"
    echo "请先运行 deploy_to_wsl2.sh"
    exit 1
fi

echo -e "${BLUE}进入工作空间并设置环境...${NC}"
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 检查arm_urdf包是否存在
if ! ros2 pkg list | grep -q arm_urdf; then
    echo -e "${RED}错误: arm_urdf包未找到${NC}"
    echo "请确保包已正确构建"
    exit 1
fi

echo -e "${GREEN}✓${NC} arm_urdf包已找到"

# 检查URDF文件
URDF_FILE="$HOME/ros2_ws/src/arm_urdf/urdf/arm2.0.urdf"
if [ ! -f "$URDF_FILE" ]; then
    echo -e "${RED}错误: URDF文件不存在: $URDF_FILE${NC}"
    exit 1
fi

echo -e "${GREEN}✓${NC} URDF文件存在"

# 检查关节限制
echo -e "${BLUE}检查关节限制配置...${NC}"
if grep -q 'lower="-3.14159"' "$URDF_FILE"; then
    echo -e "${GREEN}✓${NC} 关节限制已正确配置"
else
    echo -e "${RED}✗${NC} 关节限制配置有问题"
    echo "请检查URDF文件中的关节限制设置"
fi

# 测试launch文件语法
echo -e "${BLUE}测试launch文件语法...${NC}"
if timeout 3 ros2 launch arm_urdf display_ros2.launch.py --show-args &>/dev/null; then
    echo -e "${GREEN}✓${NC} launch文件语法正确"
else
    echo -e "${YELLOW}!${NC} launch文件可能有问题（或超时）"
fi

echo -e "\n${YELLOW}=== 测试完成 ===${NC}"
echo -e "${BLUE}如果所有检查都通过，您可以运行以下命令启动可视化：${NC}"
echo "ros2 launch arm_urdf display_ros2.launch.py"
echo ""
echo -e "${BLUE}启动后，您应该能够在joint_state_publisher_gui窗口中看到6个关节滑块${NC}"
echo -e "${BLUE}移动滑块应该能够控制RViz中机械臂的关节位置${NC}"