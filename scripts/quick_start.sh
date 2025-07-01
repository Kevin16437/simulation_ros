#!/bin/bash

# ARM6机械臂快速启动脚本

echo "=== ARM6机械臂快速启动 ==="

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 检查ROS2环境
if ! command -v ros2 &> /dev/null; then
    echo "错误: ROS2未安装或未正确配置"
    echo "请先运行 setup_ros2_workspace.sh"
    exit 1
fi

# 进入工作空间
cd ~/ros2_ws

# Source环境
source /opt/ros/humble/setup.bash
source install/setup.bash

echo -e "${YELLOW}选择启动模式:${NC}"
echo "1. RViz2可视化 (仅显示机械臂模型)"
echo "2. Gazebo仿真 (物理仿真环境)"
echo "3. 运行测试脚本"
echo "4. 构建arm6包"
echo "5. 检查系统状态"

read -p "请输入选择 (1-5): " choice

case $choice in
    1)
        echo -e "${GREEN}启动RViz2可视化...${NC}"
        echo "这将显示机械臂模型，您可以使用GUI控制关节"
        ros2 launch arm6 display_ros2.launch.py
        ;;
    2)
        echo -e "${GREEN}启动Gazebo仿真...${NC}"
        echo "这将启动物理仿真环境"
        echo "注意: 首次启动可能需要较长时间下载模型"
        ros2 launch arm6 gazebo_ros2.launch.py
        ;;
    3)
        echo -e "${GREEN}运行测试脚本...${NC}"
        python3 ~/ros2_ws/src/arm6/scripts/test_arm_movement.py
        ;;
    4)
        echo -e "${GREEN}构建arm6包...${NC}"
        colcon build --packages-select arm6 --symlink-install
        echo "构建完成，请重新source环境:"
        echo "source install/setup.bash"
        ;;
    5)
        echo -e "${GREEN}检查系统状态...${NC}"
        ./test_arm6_setup.sh
        ;;
    *)
        echo "无效选择"
        exit 1
        ;;
esac

echo -e "\n${BLUE}有用的命令:${NC}"
echo "查看所有话题: ros2 topic list"
echo "查看关节状态: ros2 topic echo /joint_states"
echo "控制关节: ros2 topic pub /arm_controller/joint_trajectory ..."
echo "启动rqt: rqt"
