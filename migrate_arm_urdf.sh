#!/bin/bash

# 迁移arm_urdf到arm6项目的脚本
# 作者: AI Assistant
# 日期: $(date)

echo "开始将arm_urdf适配到arm6项目..."

# 设置颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 设置路径
SIMULATION_DIR="/mnt/f/F Download/simulation"
ARM6_DIR="$SIMULATION_DIR/arm6"
ARM_URDF_DIR="$SIMULATION_DIR/arm_urdf"
TEST_WS="$SIMULATION_DIR/test_ws"

# 检查源目录是否存在
if [ ! -d "$ARM_URDF_DIR" ]; then
    echo -e "${RED}错误: arm_urdf目录不存在: $ARM_URDF_DIR${NC}"
    exit 1
fi

if [ ! -d "$ARM6_DIR" ]; then
    echo -e "${RED}错误: arm6目录不存在: $ARM6_DIR${NC}"
    exit 1
fi

echo -e "${BLUE}步骤1: 备份原有的arm6 URDF文件...${NC}"
# 创建备份目录
BACKUP_DIR="$ARM6_DIR/backup_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BACKUP_DIR"

# 备份原有文件
if [ -f "$ARM6_DIR/urdf/arm6.urdf" ]; then
    cp "$ARM6_DIR/urdf/arm6.urdf" "$BACKUP_DIR/"
    echo -e "${GREEN}已备份: arm6.urdf${NC}"
fi

if [ -f "$ARM6_DIR/urdf/arm6.urdf.xacro" ]; then
    cp "$ARM6_DIR/urdf/arm6.urdf.xacro" "$BACKUP_DIR/"
    echo -e "${GREEN}已备份: arm6.urdf.xacro${NC}"
fi

if [ -d "$ARM6_DIR/meshes" ]; then
    cp -r "$ARM6_DIR/meshes" "$BACKUP_DIR/"
    echo -e "${GREEN}已备份: meshes目录${NC}"
fi

echo -e "${BLUE}步骤2: 复制arm_urdf的文件到arm6...${NC}"

# 复制URDF文件
cp "$ARM_URDF_DIR/urdf/arm2.0.urdf" "$ARM6_DIR/urdf/arm6.urdf"
echo -e "${GREEN}已复制: arm2.0.urdf -> arm6.urdf${NC}"

# 复制meshes目录
if [ -d "$ARM_URDF_DIR/meshes" ]; then
    cp -r "$ARM_URDF_DIR/meshes"/* "$ARM6_DIR/meshes/"
    echo -e "${GREEN}已复制: meshes目录${NC}"
fi

# 复制config目录（如果存在）
if [ -d "$ARM_URDF_DIR/config" ]; then
    mkdir -p "$ARM6_DIR/config"
    cp -r "$ARM_URDF_DIR/config"/* "$ARM6_DIR/config/"
    echo -e "${GREEN}已复制: config目录${NC}"
fi

echo -e "${BLUE}步骤3: 修改URDF文件中的包名引用和关节限制...${NC}"

# 修改URDF文件中的包名从arm2.0改为arm6
sed -i 's/package:\/\/arm2\.0/package:\/\/arm6/g' "$ARM6_DIR/urdf/arm6.urdf"
echo -e "${GREEN}已更新: URDF文件中的包名引用${NC}"

# 修改关节限制，使关节可以正常运动
echo "修改关节限制以启用关节运动..."

# 为每个关节设置合理的运动范围
sed -i 's/lower="0"\s*upper="0"/lower="-3.14159" upper="3.14159"/g' "$ARM6_DIR/urdf/arm6.urdf"
sed -i 's/effort="0"/effort="100"/g' "$ARM6_DIR/urdf/arm6.urdf"
sed -i 's/velocity="0"/velocity="1.0"/g' "$ARM6_DIR/urdf/arm6.urdf"

echo -e "${GREEN}已更新: 关节限制参数${NC}"

echo -e "${BLUE}步骤4: 添加Gazebo插件到URDF文件...${NC}"

# 在URDF文件末尾添加Gazebo插件
echo "添加Gazebo控制插件..."

# 备份原URDF文件
cp "$ARM6_DIR/urdf/arm6.urdf" "$ARM6_DIR/urdf/arm6.urdf.backup"

# 创建临时文件来添加Gazebo插件
cat > "/tmp/gazebo_plugins.xml" << 'EOF'

  <!-- Gazebo插件配置 -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/arm6</robotNamespace>
    </plugin>
  </gazebo>

  <!-- 为每个链接添加材质属性 -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>
  
  <gazebo reference="Link1">
    <material>Gazebo/Orange</material>
  </gazebo>
  
  <gazebo reference="Link2">
    <material>Gazebo/Blue</material>
  </gazebo>
  
  <gazebo reference="Link3">
    <material>Gazebo/Orange</material>
  </gazebo>
  
  <gazebo reference="Link4">
    <material>Gazebo/Blue</material>
  </gazebo>
  
  <gazebo reference="Link5">
    <material>Gazebo/Orange</material>
  </gazebo>
  
  <gazebo reference="Link6">
    <material>Gazebo/Red</material>
  </gazebo>

EOF

# 在</robot>标签前插入Gazebo插件
sed -i '/<\/robot>/i\' "$ARM6_DIR/urdf/arm6.urdf"
sed -i '/<\/robot>/e cat /tmp/gazebo_plugins.xml' "$ARM6_DIR/urdf/arm6.urdf"

# 清理临时文件
rm -f "/tmp/gazebo_plugins.xml"

echo -e "${GREEN}已添加: Gazebo插件和材质${NC}"

echo -e "${BLUE}步骤5: 创建基于新URDF的XACRO文件...${NC}"

# 创建新的XACRO文件
cat > "$ARM6_DIR/urdf/arm6.urdf.xacro" << 'EOF'
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="arm6">
  
  <!-- 包含原始URDF内容 -->
  <xacro:include filename="$(find arm6)/urdf/arm6.urdf" />
  
  <!-- 可以在这里添加额外的XACRO宏定义 -->
  
</robot>
EOF

echo -e "${GREEN}已创建: 新的arm6.urdf.xacro文件${NC}"

echo -e "${BLUE}步骤6: 更新launch文件...${NC}"

# 更新display.launch文件
if [ -f "$ARM6_DIR/launch/display.launch" ]; then
    # 备份原launch文件
    cp "$ARM6_DIR/launch/display.launch" "$BACKUP_DIR/"
    
    # 创建新的launch文件
    cat > "$ARM6_DIR/launch/display.launch" << 'EOF'
<launch>
  <arg name="model" default="$(find arm6)/urdf/arm6.urdf.xacro"/>
  <arg name="gui" default="true" />
  <arg name="rvizconfig" default="$(find arm6)/rviz/urdf.rviz" />

  <param name="robot_description" command="$(find xacro)/xacro $(arg model)" />

  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <param name="use_gui" value="$(arg gui)"/>
  </node>

  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

  <node name="rviz" pkg="rviz" type="rviz" args="-d $(arg rvizconfig)" required="true" />
</launch>
EOF
    echo -e "${GREEN}已更新: display.launch文件${NC}"
fi

# 创建ROS2版本的launch文件
cat > "$ARM6_DIR/launch/display.launch.py" << 'EOF'
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    pkg_path = get_package_share_directory('arm6')
    
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'arm6.urdf.xacro'
    urdf = os.path.join(pkg_path, 'urdf', urdf_file_name)
    
    # 机器人描述
    robot_description_config = Command(['xacro ', urdf])
    
    # 参数声明
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # 节点定义
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': use_sim_time
        }]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_path, 'rviz', 'urdf.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])
EOF

echo -e "${GREEN}已创建: ROS2版本的display.launch.py文件${NC}"

echo -e "${BLUE}步骤7: 创建Gazebo仿真文件...${NC}"

# 执行创建Gazebo launch文件的脚本
if [ -f "$SIMULATION_DIR/create_gazebo_launch.sh" ]; then
    chmod +x "$SIMULATION_DIR/create_gazebo_launch.sh"
    bash "$SIMULATION_DIR/create_gazebo_launch.sh"
else
    echo -e "${YELLOW}警告: create_gazebo_launch.sh脚本不存在，跳过Gazebo文件创建${NC}"
fi

echo -e "${BLUE}步骤8: 更新package.xml文件...${NC}"

# 备份package.xml
cp "$ARM6_DIR/package.xml" "$BACKUP_DIR/"

# 更新package.xml以包含必要的依赖
cat > "$ARM6_DIR/package.xml" << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>arm6</name>
  <version>1.0.0</version>
  <description>ARM6 robot description package with updated URDF from arm_urdf</description>
  
  <maintainer email="user@example.com">User</maintainer>
  <license>Apache-2.0</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <depend>robot_state_publisher</depend>
  <depend>joint_state_publisher</depend>
  <depend>joint_state_publisher_gui</depend>
  <depend>xacro</depend>
  <depend>urdf</depend>
  <depend>rviz2</depend>
  
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

echo -e "${GREEN}已更新: package.xml文件${NC}"

echo -e "${BLUE}步骤9: 设置ROS2环境并测试...${NC}"

# 设置ROS2环境
source /opt/ros/jazzy/setup.bash

# 进入工作空间
cd "$TEST_WS"

# 复制更新后的arm6包到工作空间
if [ -d "src/arm6" ]; then
    rm -rf src/arm6
fi
cp -r "$ARM6_DIR" src/

echo -e "${BLUE}步骤10: 构建包...${NC}"

# 清理并重新构建
rm -rf build/ install/ log/
colcon build --packages-select arm6

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ 包构建成功！${NC}"
    
    # 设置环境
    source install/setup.bash
    
    echo -e "${BLUE}步骤11: 测试URDF文件...${NC}"
    
    # 测试XACRO处理
    echo "测试XACRO文件处理..."
    xacro src/arm6/urdf/arm6.urdf.xacro > /tmp/test_arm6.urdf
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ XACRO文件处理成功${NC}"
        
        # 检查生成的URDF文件
        echo "检查生成的URDF文件中的关节配置..."
        echo -e "${YELLOW}关节轴配置:${NC}"
        grep -A 2 "<axis" /tmp/test_arm6.urdf | grep "xyz=" || echo "未找到关节轴配置"
        
        # 清理临时文件
        rm -f /tmp/test_arm6.urdf
    else
        echo -e "${RED}✗ XACRO文件处理失败${NC}"
    fi
    
    echo -e "${GREEN}\n=== 迁移完成 ===${NC}"
    echo -e "${BLUE}备份文件位置: $BACKUP_DIR${NC}"
    echo -e "${BLUE}要启动机械臂可视化，请运行:${NC}"
    echo -e "${YELLOW}  ros2 launch arm6 display.launch.py${NC}"
    echo -e "${BLUE}或者在ROS1环境中:${NC}"
    echo -e "${YELLOW}  roslaunch arm6 display.launch${NC}"
    
else
    echo -e "${RED}✗ 包构建失败，请检查错误信息${NC}"
    exit 1
fi

echo -e "${GREEN}arm_urdf到arm6的迁移脚本执行完成！${NC}"