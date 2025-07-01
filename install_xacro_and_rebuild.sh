#!/bin/bash

# 安装XACRO并重新构建项目
echo "=== 安装XACRO并重新构建ARM6项目 ==="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

WORKSPACE_DIR="$HOME/ros2_ws"

echo -e "${BLUE}1. 设置ROS2环境${NC}"
source /opt/ros/jazzy/setup.bash

echo -e "\n${BLUE}2. 安装XACRO包${NC}"
sudo apt update
sudo apt install -y ros-jazzy-xacro

echo -e "\n${BLUE}3. 验证XACRO安装${NC}"
if command -v xacro &> /dev/null; then
    echo -e "${GREEN}✓${NC} XACRO命令可用"
    xacro --version
else
    echo -e "${RED}✗${NC} XACRO命令不可用"
fi

if ros2 pkg list | grep -q xacro; then
    echo -e "${GREEN}✓${NC} XACRO ROS2包可用"
else
    echo -e "${RED}✗${NC} XACRO ROS2包不可用"
    echo "尝试手动安装..."
    sudo apt install -y ros-jazzy-xacro
fi

echo -e "\n${BLUE}4. 复制新的XACRO文件到工作空间${NC}"
if [ -f "arm6/urdf/arm6.urdf.xacro" ]; then
    cp arm6/urdf/arm6.urdf.xacro "$WORKSPACE_DIR/src/arm6/urdf/"
    echo -e "${GREEN}✓${NC} XACRO文件已复制"
else
    echo -e "${RED}✗${NC} 找不到XACRO文件"
fi

echo -e "\n${BLUE}5. 复制新的launch文件${NC}"
if [ -f "arm6/launch/display_xacro.launch.py" ]; then
    cp arm6/launch/display_xacro.launch.py "$WORKSPACE_DIR/src/arm6/launch/"
    echo -e "${GREEN}✓${NC} display_xacro.launch.py 已复制"
fi

if [ -f "arm6/launch/gazebo_sim.launch.py" ]; then
    cp arm6/launch/gazebo_sim.launch.py "$WORKSPACE_DIR/src/arm6/launch/"
    echo -e "${GREEN}✓${NC} gazebo_sim.launch.py 已复制"
fi

echo -e "\n${BLUE}6. 更新package.xml和CMakeLists.txt${NC}"
if [ -f "arm6/package.xml" ]; then
    cp arm6/package.xml "$WORKSPACE_DIR/src/arm6/"
    echo -e "${GREEN}✓${NC} package.xml 已更新"
fi

if [ -f "arm6/CMakeLists.txt" ]; then
    cp arm6/CMakeLists.txt "$WORKSPACE_DIR/src/arm6/"
    echo -e "${GREEN}✓${NC} CMakeLists.txt 已更新"
fi

echo -e "\n${BLUE}7. 测试XACRO文件${NC}"
cd "$WORKSPACE_DIR"
xacro_file="$WORKSPACE_DIR/src/arm6/urdf/arm6.urdf.xacro"
controllers_file="$WORKSPACE_DIR/src/arm6/config/arm_controllers.yaml"

if [ -f "$xacro_file" ]; then
    echo "测试XACRO处理..."
    if xacro "$xacro_file" controllers_file:="$controllers_file" > /tmp/test_urdf.xml; then
        echo -e "${GREEN}✓${NC} XACRO文件处理成功"
        echo "生成的URDF行数: $(wc -l < /tmp/test_urdf.xml)"
    else
        echo -e "${RED}✗${NC} XACRO文件处理失败"
    fi
else
    echo -e "${RED}✗${NC} XACRO文件不存在"
fi

echo -e "\n${BLUE}8. 重新构建包${NC}"
cd "$WORKSPACE_DIR"
source /opt/ros/jazzy/setup.bash

# 清理之前的构建
rm -rf build/arm6 install/arm6 log/

# 重新构建
if colcon build --packages-select arm6 --symlink-install; then
    echo -e "${GREEN}✓${NC} ARM6包重新构建成功"
    
    # 更新环境
    source install/setup.bash
    
    # 验证包
    if ros2 pkg list | grep -q arm6; then
        echo -e "${GREEN}✓${NC} arm6包可以被ROS2找到"
    else
        echo -e "${RED}✗${NC} arm6包无法被ROS2找到"
    fi
    
else
    echo -e "${RED}✗${NC} 构建失败"
    exit 1
fi

echo -e "\n${BLUE}9. 创建新的启动脚本${NC}"
cat > ~/start_arm6_xacro_rviz.sh << 'EOF'
#!/bin/bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
echo "启动ARM6 XACRO RViz2可视化..."
ros2 launch arm6 display_xacro.launch.py
EOF
chmod +x ~/start_arm6_xacro_rviz.sh

cat > ~/start_arm6_gazebo_sim.sh << 'EOF'
#!/bin/bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
echo "启动ARM6 Gazebo仿真..."
ros2 launch arm6 gazebo_sim.launch.py
EOF
chmod +x ~/start_arm6_gazebo_sim.sh

echo -e "${GREEN}✓${NC} 新的启动脚本已创建"

echo -e "\n${GREEN}=== 安装和构建完成！ ===${NC}"
echo -e "\n${BLUE}使用方法：${NC}"
echo "1. XACRO版本RViz2可视化:"
echo "   ~/start_arm6_xacro_rviz.sh"
echo ""
echo "2. Gazebo仿真:"
echo "   ~/start_arm6_gazebo_sim.sh"
echo ""
echo "3. 手动启动:"
echo "   cd ~/ros2_ws && source install/setup.bash"
echo "   ros2 launch arm6 display_xacro.launch.py"
echo "   ros2 launch arm6 gazebo_sim.launch.py"
echo ""
echo -e "${YELLOW}注意: 新的launch文件使用XACRO格式，支持参数化配置${NC}"
