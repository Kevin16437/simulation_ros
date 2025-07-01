#!/bin/bash

# 简化的包验证脚本
echo "=== ARM6包简化验证 ==="

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

PACKAGE_DIR="arm6"
ERRORS=0
WARNINGS=0

check_file() {
    if [ -f "$1" ]; then
        echo -e "${GREEN}✓${NC} $1 存在"
        return 0
    else
        echo -e "${RED}✗${NC} $1 不存在"
        ((ERRORS++))
        return 1
    fi
}

check_dir() {
    if [ -d "$1" ]; then
        echo -e "${GREEN}✓${NC} $1 目录存在"
        return 0
    else
        echo -e "${YELLOW}!${NC} $1 目录不存在"
        ((WARNINGS++))
        return 1
    fi
}

echo "检查基本文件结构..."

# 检查必需文件
check_file "$PACKAGE_DIR/package.xml"
check_file "$PACKAGE_DIR/CMakeLists.txt"
check_file "$PACKAGE_DIR/urdf/arm6.urdf"

# 检查launch文件
check_file "$PACKAGE_DIR/launch/display_ros2.launch.py"
check_file "$PACKAGE_DIR/launch/gazebo_ros2.launch.py"

# 检查配置文件
check_file "$PACKAGE_DIR/config/arm_controllers.yaml"
check_file "$PACKAGE_DIR/config/arm6.rviz"

# 检查目录
check_dir "$PACKAGE_DIR/meshes"
check_dir "$PACKAGE_DIR/worlds"
check_dir "$PACKAGE_DIR/scripts"

echo ""
echo "检查package.xml内容..."
if [ -f "$PACKAGE_DIR/package.xml" ]; then
    if grep -q "ament_cmake" "$PACKAGE_DIR/package.xml"; then
        echo -e "${GREEN}✓${NC} 使用 ament_cmake (ROS2)"
    elif grep -q "catkin" "$PACKAGE_DIR/package.xml"; then
        echo -e "${YELLOW}!${NC} 使用 catkin (ROS1)"
        ((WARNINGS++))
    fi
    
    if grep -q "format=\"3\"" "$PACKAGE_DIR/package.xml"; then
        echo -e "${GREEN}✓${NC} 使用 package format 3 (ROS2)"
    elif grep -q "format=\"2\"" "$PACKAGE_DIR/package.xml"; then
        echo -e "${YELLOW}!${NC} 使用 package format 2"
        ((WARNINGS++))
    fi
fi

echo ""
echo "检查URDF文件内容..."
if [ -f "$PACKAGE_DIR/urdf/arm6.urdf" ]; then
    if grep -q "ros2_control" "$PACKAGE_DIR/urdf/arm6.urdf"; then
        echo -e "${GREEN}✓${NC} 包含 ros2_control 配置"
    else
        echo -e "${YELLOW}!${NC} 缺少 ros2_control 配置"
        ((WARNINGS++))
    fi
    
    if grep -q "gazebo_ros2_control" "$PACKAGE_DIR/urdf/arm6.urdf"; then
        echo -e "${GREEN}✓${NC} 使用 gazebo_ros2_control"
    elif grep -q "gazebo_ros_control" "$PACKAGE_DIR/urdf/arm6.urdf"; then
        echo -e "${YELLOW}!${NC} 使用 gazebo_ros_control (ROS1)"
        ((WARNINGS++))
    fi
    
    # 检查关节限制
    joint_count=$(grep -c "<joint" "$PACKAGE_DIR/urdf/arm6.urdf")
    echo -e "${GREEN}✓${NC} 找到 $joint_count 个关节"
    
    # 检查是否有0限制的关节
    zero_limits=$(grep -A 5 "<limit" "$PACKAGE_DIR/urdf/arm6.urdf" | grep -c "upper=\"0\"")
    if [ $zero_limits -gt 0 ]; then
        echo -e "${YELLOW}!${NC} 发现 $zero_limits 个可能有问题的关节限制"
        ((WARNINGS++))
    else
        echo -e "${GREEN}✓${NC} 关节限制看起来正常"
    fi
fi

echo ""
echo "检查网格文件..."
if [ -d "$PACKAGE_DIR/meshes" ]; then
    stl_count=$(find "$PACKAGE_DIR/meshes" -name "*.STL" -o -name "*.stl" | wc -l)
    echo -e "${GREEN}✓${NC} 找到 $stl_count 个 STL 文件"
fi

echo ""
echo "=== 验证总结 ==="
echo -e "错误: ${RED}$ERRORS${NC}"
echo -e "警告: ${YELLOW}$WARNINGS${NC}"

if [ $ERRORS -eq 0 ]; then
    echo -e "${GREEN}✓ 基本验证通过！${NC}"
    echo ""
    echo "项目已准备好在ROS2环境中使用。"
    echo "请按照 USAGE_INSTRUCTIONS.md 中的步骤在WSL2中设置。"
    exit 0
else
    echo -e "${RED}✗ 发现 $ERRORS 个错误，需要修复${NC}"
    exit 1
fi
