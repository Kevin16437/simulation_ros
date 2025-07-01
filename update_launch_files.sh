#!/bin/bash

# 更新launch文件修复脚本
echo "=== 更新ARM6 Launch文件 ==="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

WORKSPACE_DIR="$HOME/ros2_ws"
ARM6_DIR="$WORKSPACE_DIR/src/arm6"

echo -e "${BLUE}1. 检查工作空间${NC}"
if [ ! -d "$ARM6_DIR" ]; then
    echo -e "${RED}✗${NC} ARM6包不存在于 $ARM6_DIR"
    echo "请先运行部署脚本"
    exit 1
fi

echo -e "${GREEN}✓${NC} 找到ARM6包"

echo -e "\n${BLUE}2. 备份原始launch文件${NC}"
cd "$ARM6_DIR/launch"
cp display_ros2.launch.py display_ros2.launch.py.backup
cp gazebo_ros2.launch.py gazebo_ros2.launch.py.backup
echo -e "${GREEN}✓${NC} 备份完成"

echo -e "\n${BLUE}3. 更新display_ros2.launch.py${NC}"
cat > display_ros2.launch.py << 'EOF'
#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get the package directory
    pkg_path = get_package_share_directory('arm6')
    urdf_file = os.path.join(pkg_path, 'urdf', 'arm6.urdf')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Robot description
    robot_description_content = ParameterValue(Command(['cat ', urdf_file]), value_type=str)
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Joint state publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # RViz2
    rviz_config_file = os.path.join(pkg_path, 'config', 'arm6.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])
EOF

echo -e "${GREEN}✓${NC} display_ros2.launch.py 已更新"

echo -e "\n${BLUE}4. 更新gazebo_ros2.launch.py${NC}"
cat > gazebo_ros2.launch.py << 'EOF'
#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get the package directory
    pkg_path = get_package_share_directory('arm6')
    urdf_file = os.path.join(pkg_path, 'urdf', 'arm6.urdf')
    world_file = os.path.join(pkg_path, 'worlds', 'empty.world')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=world_file)
    
    # Robot description
    robot_description_content = ParameterValue(Command(['cat ', urdf_file]), value_type=str)
    
    # Gazebo launch (simplified for compatibility)
    try:
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('gazebo_ros'),
                '/launch/gazebo.launch.py'
            ]),
            launch_arguments={
                'world': world,
                'verbose': 'true'
            }.items()
        )
    except:
        # Fallback if gazebo_ros package is not available
        print("Warning: Gazebo launch not available, skipping Gazebo startup")
        gazebo = None
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'arm6',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Build launch description
    launch_nodes = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='Full path to world model file to load'
        ),
        robot_state_publisher,
    ]
    
    # Add Gazebo if available
    if gazebo is not None:
        launch_nodes.extend([gazebo, spawn_entity])
    
    return LaunchDescription(launch_nodes)
EOF

echo -e "${GREEN}✓${NC} gazebo_ros2.launch.py 已更新"

echo -e "\n${BLUE}5. 重新构建包${NC}"
cd "$WORKSPACE_DIR"
source /opt/ros/jazzy/setup.bash

# 重新构建
if colcon build --packages-select arm6 --symlink-install; then
    echo -e "${GREEN}✓${NC} ARM6包重新构建成功"
    
    # 更新环境
    source install/setup.bash
    
    echo -e "\n${BLUE}6. 更新启动脚本${NC}"
    cat > ~/start_arm6_rviz.sh << 'EOF'
#!/bin/bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
echo "启动ARM6 RViz2可视化..."
ros2 launch arm6 display_ros2.launch.py
EOF
    chmod +x ~/start_arm6_rviz.sh
    
    cat > ~/start_arm6_gazebo.sh << 'EOF'
#!/bin/bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
echo "启动ARM6 Gazebo仿真..."
ros2 launch arm6 gazebo_ros2.launch.py
EOF
    chmod +x ~/start_arm6_gazebo.sh
    
    echo -e "${GREEN}✓${NC} 启动脚本已更新"
    
else
    echo -e "${RED}✗${NC} 构建失败"
    exit 1
fi

echo -e "\n${GREEN}=== 更新完成！ ===${NC}"
echo -e "\n${BLUE}现在可以尝试启动：${NC}"
echo "~/start_arm6_rviz.sh"
echo ""
echo -e "${YELLOW}如果还有问题，请检查错误信息${NC}"
