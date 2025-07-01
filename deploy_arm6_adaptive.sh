#!/bin/bash

# ARM6自适应部署脚本
echo "=== ARM6自适应部署脚本 ==="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="$HOME/ros2_ws"

# 自动检测ROS2安装
detect_ros2() {
    echo -e "${BLUE}检测ROS2安装...${NC}"
    
    # 检查可能的ROS2版本
    for distro in jazzy humble iron galactic foxy; do
        if [ -f "/opt/ros/$distro/setup.bash" ]; then
            echo -e "${GREEN}✓${NC} 找到ROS2 $distro"
            export ROS_DISTRO=$distro
            export ROS_SETUP_PATH="/opt/ros/$distro/setup.bash"
            return 0
        fi
    done
    
    echo -e "${RED}✗${NC} 未找到ROS2安装"
    return 1
}

# 检查并安装缺失的包
install_missing_packages() {
    echo -e "\n${BLUE}检查并安装缺失的包...${NC}"
    
    source $ROS_SETUP_PATH
    
    # 检查关键包
    missing_packages=()
    
    if ! ros2 pkg list | grep -q xacro; then
        missing_packages+=("ros-${ROS_DISTRO}-xacro")
    fi
    
    if ! ros2 pkg list | grep -q robot_state_publisher; then
        missing_packages+=("ros-${ROS_DISTRO}-robot-state-publisher")
    fi
    
    if ! ros2 pkg list | grep -q joint_state_publisher_gui; then
        missing_packages+=("ros-${ROS_DISTRO}-joint-state-publisher-gui")
    fi
    
    if ! ros2 pkg list | grep -q gazebo_ros; then
        missing_packages+=("ros-${ROS_DISTRO}-gazebo-ros-pkgs")
    fi
    
    if ! ros2 pkg list | grep -q controller_manager; then
        missing_packages+=("ros-${ROS_DISTRO}-ros2-control" "ros-${ROS_DISTRO}-ros2-controllers")
    fi
    
    if [ ${#missing_packages[@]} -gt 0 ]; then
        echo -e "${YELLOW}需要安装以下包:${NC}"
        printf '%s\n' "${missing_packages[@]}"
        
        echo "正在安装..."
        sudo apt update
        sudo apt install -y "${missing_packages[@]}"
    else
        echo -e "${GREEN}✓${NC} 所有必需包已安装"
    fi
}

# 修复CMakeLists.txt以适应可用的包
fix_cmake_for_available_packages() {
    echo -e "\n${BLUE}修复CMakeLists.txt...${NC}"
    
    source $ROS_SETUP_PATH
    
    # 检查xacro是否可用
    if ! ros2 pkg list | grep -q xacro; then
        echo -e "${YELLOW}xacro不可用，从CMakeLists.txt中移除${NC}"
        sed -i '/find_package(xacro REQUIRED)/d' "$ROS_WS/src/arm6/CMakeLists.txt"
    fi
}

# 主函数
main() {
    echo -e "${BLUE}开始自适应部署ARM6项目...${NC}"
    
    # 检测ROS2
    if ! detect_ros2; then
        echo -e "${RED}请先安装ROS2，可以运行:${NC}"
        echo "./fix_ros2_installation.sh"
        exit 1
    fi
    
    echo -e "${GREEN}使用ROS2 $ROS_DISTRO${NC}"
    
    # 创建工作空间
    echo -e "\n${BLUE}设置工作空间...${NC}"
    mkdir -p "$ROS_WS/src"
    
    # 复制arm6包
    if [ -d "$SCRIPT_DIR/arm6" ]; then
        cp -r "$SCRIPT_DIR/arm6" "$ROS_WS/src/"
        echo -e "${GREEN}✓${NC} 复制arm6包到工作空间"
    else
        echo -e "${RED}✗${NC} 找不到arm6包"
        exit 1
    fi
    
    # 安装缺失的包
    install_missing_packages
    
    # 修复CMakeLists.txt
    fix_cmake_for_available_packages
    
    # 初始化rosdep（如果需要）
    if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
        echo -e "\n${BLUE}初始化rosdep...${NC}"
        sudo rosdep init
        rosdep update
    fi
    
    # 安装依赖
    echo -e "\n${BLUE}安装项目依赖...${NC}"
    cd "$ROS_WS"
    source $ROS_SETUP_PATH
    rosdep install --from-paths src --ignore-src -r -y
    
    # 构建包
    echo -e "\n${BLUE}构建arm6包...${NC}"
    if colcon build --packages-select arm6 --symlink-install; then
        echo -e "${GREEN}✓${NC} arm6包构建成功"
    else
        echo -e "${RED}✗${NC} arm6包构建失败"
        echo -e "${YELLOW}尝试查看错误信息并手动修复${NC}"
        exit 1
    fi
    
    # 设置环境
    echo -e "\n${BLUE}设置环境变量...${NC}"
    if ! grep -q "source $ROS_SETUP_PATH" ~/.bashrc; then
        echo "source $ROS_SETUP_PATH" >> ~/.bashrc
    fi
    
    if ! grep -q "source $ROS_WS/install/setup.bash" ~/.bashrc; then
        echo "source $ROS_WS/install/setup.bash" >> ~/.bashrc
    fi
    
    # 创建启动脚本
    echo -e "\n${BLUE}创建启动脚本...${NC}"
    
    cat > "$HOME/start_arm6_rviz.sh" << EOF
#!/bin/bash
cd $ROS_WS
source $ROS_SETUP_PATH
source install/setup.bash
echo "启动ARM6 RViz2可视化..."
ros2 launch arm6 display_ros2.launch.py
EOF
    chmod +x "$HOME/start_arm6_rviz.sh"
    
    cat > "$HOME/start_arm6_gazebo.sh" << EOF
#!/bin/bash
cd $ROS_WS
source $ROS_SETUP_PATH
source install/setup.bash
echo "启动ARM6 Gazebo仿真..."
ros2 launch arm6 gazebo_ros2.launch.py
EOF
    chmod +x "$HOME/start_arm6_gazebo.sh"
    
    # 测试
    echo -e "\n${BLUE}测试安装...${NC}"
    source $ROS_SETUP_PATH
    source "$ROS_WS/install/setup.bash"
    
    if ros2 pkg list | grep -q arm6; then
        echo -e "${GREEN}✓${NC} arm6包可以被ROS2找到"
    else
        echo -e "${RED}✗${NC} arm6包无法被ROS2找到"
    fi
    
    # 完成
    echo -e "\n${GREEN}=== 部署完成！ ===${NC}"
    echo -e "\n${BLUE}使用方法：${NC}"
    echo "1. 重新加载环境: source ~/.bashrc"
    echo "2. 启动RViz2: ~/start_arm6_rviz.sh"
    echo "3. 启动Gazebo: ~/start_arm6_gazebo.sh"
    echo ""
    echo -e "${YELLOW}或者手动运行：${NC}"
    echo "cd $ROS_WS && source install/setup.bash"
    echo "ros2 launch arm6 display_ros2.launch.py"
}

# 运行主函数
main "$@"
