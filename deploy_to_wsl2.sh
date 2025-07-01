#!/bin/bash

# ARM6项目WSL2部署脚本
# 此脚本用于在WSL2环境中部署和测试ARM6机械臂项目

echo "=== ARM6项目WSL2部署脚本 ==="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_NAME="arm6_project"
ROS_WS="$HOME/ros2_ws"

echo -e "${BLUE}脚本目录: $SCRIPT_DIR${NC}"
echo -e "${BLUE}目标工作空间: $ROS_WS${NC}"

# 检查是否在WSL2环境中
check_wsl2() {
    if grep -qi microsoft /proc/version; then
        echo -e "${GREEN}✓${NC} 检测到WSL2环境"
        return 0
    else
        echo -e "${YELLOW}!${NC} 未检测到WSL2环境，继续执行..."
        return 0
    fi
}

# 检查ROS2安装
check_ros2() {
    if command -v ros2 &> /dev/null; then
        echo -e "${GREEN}✓${NC} ROS2已安装"
        ros2 --version
        return 0
    else
        echo -e "${RED}✗${NC} ROS2未安装"
        echo "请先运行 setup_ros2_workspace.sh 安装ROS2"
        return 1
    fi
}

# 创建工作空间
setup_workspace() {
    echo -e "\n${YELLOW}设置ROS2工作空间${NC}"
    
    if [ ! -d "$ROS_WS" ]; then
        mkdir -p "$ROS_WS/src"
        echo -e "${GREEN}✓${NC} 创建工作空间: $ROS_WS"
    else
        echo -e "${YELLOW}!${NC} 工作空间已存在: $ROS_WS"
    fi
    
    # 复制arm6包
    if [ -d "$SCRIPT_DIR/arm6" ]; then
        cp -r "$SCRIPT_DIR/arm6" "$ROS_WS/src/"
        echo -e "${GREEN}✓${NC} 复制arm6包到工作空间"
    else
        echo -e "${RED}✗${NC} 找不到arm6包"
        return 1
    fi
    
    return 0
}

# 安装依赖
install_dependencies() {
    echo -e "\n${YELLOW}安装依赖包${NC}"
    
    # Source ROS2环境
    source /opt/ros/humble/setup.bash
    
    cd "$ROS_WS"
    
    # 使用rosdep安装依赖
    if command -v rosdep &> /dev/null; then
        echo "使用rosdep安装依赖..."
        rosdep install --from-paths src --ignore-src -r -y
    else
        echo -e "${YELLOW}!${NC} rosdep未找到，手动安装关键依赖"
        sudo apt update
        sudo apt install -y \
            ros-humble-robot-state-publisher \
            ros-humble-joint-state-publisher-gui \
            ros-humble-rviz2 \
            ros-humble-gazebo-ros-pkgs \
            ros-humble-ros2-control \
            ros-humble-ros2-controllers \
            ros-humble-controller-manager \
            ros-humble-xacro
    fi
    
    echo -e "${GREEN}✓${NC} 依赖安装完成"
}

# 构建包
build_package() {
    echo -e "\n${YELLOW}构建arm6包${NC}"
    
    cd "$ROS_WS"
    source /opt/ros/humble/setup.bash
    
    # 构建包
    if colcon build --packages-select arm6 --symlink-install; then
        echo -e "${GREEN}✓${NC} arm6包构建成功"
        return 0
    else
        echo -e "${RED}✗${NC} arm6包构建失败"
        return 1
    fi
}

# 测试包
test_package() {
    echo -e "\n${YELLOW}测试arm6包${NC}"
    
    cd "$ROS_WS"
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    
    # 检查包是否可以找到
    if ros2 pkg list | grep -q arm6; then
        echo -e "${GREEN}✓${NC} arm6包可以被ROS2找到"
    else
        echo -e "${RED}✗${NC} arm6包无法被ROS2找到"
        return 1
    fi
    
    # 测试launch文件语法
    echo "测试launch文件..."
    if timeout 5 ros2 launch arm6 display_ros2.launch.py --show-args &>/dev/null; then
        echo -e "${GREEN}✓${NC} display_ros2.launch.py 语法正确"
    else
        echo -e "${YELLOW}!${NC} display_ros2.launch.py 可能有问题（或超时）"
    fi
    
    return 0
}

# 创建启动脚本
create_launch_scripts() {
    echo -e "\n${YELLOW}创建启动脚本${NC}"
    
    # 创建RViz启动脚本
    cat > "$HOME/start_arm6_rviz.sh" << 'EOF'
#!/bin/bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
echo "启动ARM6 RViz2可视化..."
ros2 launch arm6 display_ros2.launch.py
EOF
    chmod +x "$HOME/start_arm6_rviz.sh"
    
    # 创建Gazebo启动脚本
    cat > "$HOME/start_arm6_gazebo.sh" << 'EOF'
#!/bin/bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
echo "启动ARM6 Gazebo仿真..."
ros2 launch arm6 gazebo_ros2.launch.py
EOF
    chmod +x "$HOME/start_arm6_gazebo.sh"
    
    # 创建控制测试脚本
    cat > "$HOME/test_arm6_control.sh" << 'EOF'
#!/bin/bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
echo "启动ARM6控制测试..."
python3 src/arm6/scripts/test_arm_movement.py
EOF
    chmod +x "$HOME/test_arm6_control.sh"
    
    echo -e "${GREEN}✓${NC} 启动脚本已创建在 $HOME 目录"
}

# 主函数
main() {
    echo -e "${BLUE}开始部署ARM6项目到WSL2...${NC}"
    
    # 检查环境
    check_wsl2
    if ! check_ros2; then
        echo -e "${RED}请先安装ROS2环境${NC}"
        exit 1
    fi
    
    # 设置工作空间
    if ! setup_workspace; then
        echo -e "${RED}工作空间设置失败${NC}"
        exit 1
    fi
    
    # 安装依赖
    install_dependencies
    
    # 构建包
    if ! build_package; then
        echo -e "${RED}包构建失败${NC}"
        exit 1
    fi
    
    # 测试包
    test_package
    
    # 创建启动脚本
    create_launch_scripts
    
    # 输出使用说明
    echo -e "\n${GREEN}=== 部署完成！ ===${NC}"
    echo -e "\n${BLUE}使用方法：${NC}"
    echo "1. 启动RViz2可视化："
    echo "   ~/start_arm6_rviz.sh"
    echo ""
    echo "2. 启动Gazebo仿真："
    echo "   ~/start_arm6_gazebo.sh"
    echo ""
    echo "3. 测试机械臂控制："
    echo "   ~/test_arm6_control.sh"
    echo ""
    echo -e "${BLUE}手动命令：${NC}"
    echo "cd ~/ros2_ws && source install/setup.bash"
    echo "ros2 launch arm6 display_ros2.launch.py"
    echo "ros2 launch arm6 gazebo_ros2.launch.py"
    echo ""
    echo -e "${GREEN}项目部署成功！${NC}"
}

# 运行主函数
main "$@"
