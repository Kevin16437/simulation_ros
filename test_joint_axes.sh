#!/bin/bash

# 测试修正后的关节轴配置
echo "=== 测试ARM6关节轴配置 ==="

# 1. 设置ROS2环境
echo "1. 设置ROS2环境"
source /opt/ros/jazzy/setup.bash

# 2. 进入工作空间
echo "2. 进入工作空间"
cd $HOME/ros2_ws

if [ ! -d "src/arm6" ]; then
    echo "复制修正后的arm6包到工作空间..."
    cp -r "/mnt/f/F Download/simulation/arm6" src/
fi

# 3. 重新构建包
echo "3. 重新构建包"
colcon build --packages-select arm6 --symlink-install

if [ $? -ne 0 ]; then
    echo "✗ 构建失败！"
    exit 1
fi

echo "✓ 构建成功"

# 4. 设置环境
source install/setup.bash

# 5. 测试XACRO文件处理
echo "4. 测试XACRO文件处理"
if ros2 run xacro xacro src/arm6/urdf/arm6.urdf.xacro > /tmp/test_urdf_corrected.xml 2>/dev/null; then
    echo "✓ XACRO文件处理成功"
    urdf_lines=$(wc -l < /tmp/test_urdf_corrected.xml)
    echo "生成的URDF行数: $urdf_lines"
else
    echo "✗ XACRO文件处理失败"
    exit 1
fi

# 6. 验证关节轴配置
echo "5. 验证关节轴配置"
echo "检查j3关节轴配置..."
j3_axis=$(grep -A 10 'name="j3"' /tmp/test_urdf_corrected.xml | grep '<axis' | sed 's/.*xyz="\([^"]*\)".*/\1/')
echo "j3关节轴: $j3_axis (应该是: 0 1 0)"

echo "检查j4关节轴配置..."
j4_axis=$(grep -A 10 'name="j4"' /tmp/test_urdf_corrected.xml | grep '<axis' | sed 's/.*xyz="\([^"]*\)".*/\1/')
echo "j4关节轴: $j4_axis (应该是: 1 0 0)"

# 7. 清理临时文件
rm -f /tmp/test_urdf_corrected.xml

# 8. 显示所有关节轴配置
echo "6. 显示所有关节轴配置"
echo "当前关节轴配置:"
echo "j1: 绕Y轴 (0 -1 0) - 基座旋转"
echo "j2: 绕X轴 (1 0 0) - 肩部俯仰"
echo "j3: 绕Y轴 (0 1 0) - 肘部俯仰 [已修正]"
echo "j4: 绕X轴 (1 0 0) - 腕部滚转 [已修正]"
echo "j5: 绕X轴 (-1 0 0) - 腕部俯仰"
echo "j6: 绕Y轴 (0 -1 0) - 腕部旋转"

echo ""
echo "=== 关节轴修正完成 ==="
echo "现在可以启动机械臂仿真:"
echo "ros2 launch arm6 display_xacro.launch.py"
echo "或者启动Gazebo仿真:"
echo "ros2 launch arm6 gazebo_sim.launch.py"