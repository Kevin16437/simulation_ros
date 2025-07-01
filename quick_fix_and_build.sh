#!/bin/bash

# 快速修复和构建脚本
echo "=== ARM6快速修复和构建 ==="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}1. 设置ROS2 Jazzy环境${NC}"
export ROS_DISTRO=jazzy
source /opt/ros/jazzy/setup.bash

echo -e "\n${BLUE}2. 安装必需的包${NC}"
sudo apt update
sudo apt install -y \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-rviz2 \
    python3-colcon-common-extensions

echo -e "\n${BLUE}3. 修复CMakeLists.txt（移除xacro依赖）${NC}"
cd ~/ros2_ws/src/arm6
# 备份原文件
cp CMakeLists.txt CMakeLists.txt.backup

# 创建简化版本
cat > CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(arm6)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

# Install directories
install(
  DIRECTORY
    urdf
    meshes
    launch
    config
    worlds
    scripts
  DESTINATION
    share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
EOF

echo -e "${GREEN}✓${NC} CMakeLists.txt已简化"

echo -e "\n${BLUE}4. 修复package.xml（移除不必要的依赖）${NC}"
# 备份原文件
cp package.xml package.xml.backup

# 创建简化版本
cat > package.xml << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>arm6</name>
  <version>1.0.0</version>
  <description>URDF Description package for arm6 robot arm</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>BSD-3-Clause</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- Dependencies for robot description -->
  <depend>robot_state_publisher</depend>
  <depend>joint_state_publisher</depend>
  <depend>joint_state_publisher_gui</depend>
  <depend>rviz2</depend>

  <!-- Build dependencies -->
  <build_depend>urdf</build_depend>

  <!-- Execution dependencies -->
  <exec_depend>urdf</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

echo -e "${GREEN}✓${NC} package.xml已简化"

echo -e "\n${BLUE}5. 构建项目${NC}"
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash

# 清理之前的构建
rm -rf build/ install/ log/

# 构建arm6包
echo "开始构建..."
if colcon build --packages-select arm6 --symlink-install; then
    echo -e "${GREEN}✓${NC} arm6包构建成功！"
    
    # 设置环境
    source install/setup.bash
    
    # 验证包
    if ros2 pkg list | grep -q arm6; then
        echo -e "${GREEN}✓${NC} arm6包可以被ROS2找到"
    else
        echo -e "${RED}✗${NC} arm6包无法被ROS2找到"
    fi
    
    echo -e "\n${GREEN}=== 构建成功！ ===${NC}"
    echo -e "\n${BLUE}使用方法：${NC}"
    echo "1. 加载环境:"
    echo "   cd ~/ros2_ws && source install/setup.bash"
    echo ""
    echo "2. 启动RViz2可视化:"
    echo "   ros2 launch arm6 display_ros2.launch.py"
    echo ""
    echo "3. 查看可用的launch文件:"
    echo "   ros2 pkg list | grep arm6"
    echo "   ros2 launch arm6 --show-args display_ros2.launch.py"
    
else
    echo -e "${RED}✗${NC} arm6包构建失败"
    echo -e "${YELLOW}查看错误信息并手动修复${NC}"
    exit 1
fi

echo -e "\n${BLUE}6. 创建启动脚本${NC}"
cat > ~/start_arm6_rviz.sh << 'EOF'
#!/bin/bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
echo "启动ARM6 RViz2可视化..."
ros2 launch arm6 display_ros2.launch.py
EOF
chmod +x ~/start_arm6_rviz.sh

echo -e "${GREEN}✓${NC} 启动脚本已创建: ~/start_arm6_rviz.sh"

echo -e "\n${GREEN}=== 全部完成！ ===${NC}"
