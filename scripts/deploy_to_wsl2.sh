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
PROJECT_NAME="arm_urdf_project"
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
    
    # 复制arm_urdf包
    if [ -d "$SCRIPT_DIR/../arm_urdf" ]; then
        cp -r "$SCRIPT_DIR/../arm_urdf" "$ROS_WS/src/"
        echo -e "${GREEN}✓${NC} 复制arm_urdf包到工作空间"
    else
        echo -e "${RED}✗${NC} 找不到arm_urdf包 at $SCRIPT_DIR/../arm_urdf"
        return 1
    fi
    
    return 0
}

# 安装依赖
install_dependencies() {
    echo -e "\n${YELLOW}安装依赖包${NC}"
    
    # Source ROS2环境
    source /opt/ros/jazzy/setup.bash
    
    cd "$ROS_WS"
    
    # 使用rosdep安装依赖
    if command -v rosdep &> /dev/null; then
        echo "使用rosdep安装依赖..."
        rosdep install --from-paths src --ignore-src -r -y
    else
        echo -e "${YELLOW}!${NC} rosdep未找到，手动安装关键依赖"
        sudo apt update
        sudo apt install -y \
            ros-jazzy-robot-state-publisher \
            ros-jazzy-joint-state-publisher-gui \
            ros-jazzy-rviz2 \
            ros-jazzy-gazebo-ros-pkgs \
            ros-jazzy-ros2-control \
            ros-jazzy-ros2-controllers \
            ros-jazzy-controller-manager \
            ros-jazzy-xacro
    fi
    
    echo -e "${GREEN}✓${NC} 依赖安装完成"
}

# 构建包
build_package() {
    echo -e "\n${YELLOW}构建arm_urdf包${NC}"
    
    cd "$ROS_WS"
    source /opt/ros/jazzy/setup.bash
    
    # 构建包
    if colcon build --packages-select arm_urdf --symlink-install; then
        echo -e "${GREEN}✓${NC} arm_urdf包构建成功"
        return 0
    else
        echo -e "${RED}✗${NC} arm_urdf包构建失败"
        return 1
    fi
}

# 测试包
test_package() {
    echo -e "\n${YELLOW}测试arm_urdf包${NC}"
    
    cd "$ROS_WS"
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    
    # 检查包是否可以找到
    if ros2 pkg list | grep -q arm_urdf; then
        echo -e "${GREEN}✓${NC} arm_urdf包可以被ROS2找到"
    else
        echo -e "${RED}✗${NC} arm_urdf包无法被ROS2找到"
        return 1
    fi
    
    # 测试launch文件语法
    echo "测试launch文件..."
    if timeout 5 ros2 launch arm_urdf display_ros2.launch.py --show-args &>/dev/null; then
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
    cat > "$HOME/start_arm_urdf_rviz.sh" << 'EOF'
#!/bin/bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
echo "启动arm_urdf RViz2可视化..."
ros2 launch arm_urdf display_ros2.launch.py
EOF
    chmod +x "$HOME/start_arm_urdf_rviz.sh"
    
    echo -e "${GREEN}✓${NC} RViz启动脚本 (start_arm_urdf_rviz.sh) 已创建在 $HOME 目录"
    echo -e "${YELLOW}!${NC} Gazebo和控制测试脚本未创建，因为相关文件不存在。"
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
    echo "   ~/start_arm_urdf_rviz.sh"
    echo ""
    echo -e "${BLUE}手动命令：${NC}"
    echo "cd ~/ros2_ws && source install/setup.bash"
    echo "ros2 launch arm_urdf display_ros2.launch.py"
    echo ""
    echo -e "${GREEN}项目部署成功！${NC}"
}

# 运行主函数
main "$@"
