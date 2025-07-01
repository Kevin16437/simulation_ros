#!/bin/bash

# 修复package.xml并重新构建ARM6项目
echo "=== 修复package.xml并重新构建ARM6项目 ==="

# 1. 设置ROS2环境
echo "1. 设置ROS2环境"
source /opt/ros/jazzy/setup.bash

# 2. 检查修复后的package.xml
echo "2. 检查package.xml文件"
if [ -f "arm6/package.xml" ]; then
    echo "✓ 主package.xml文件存在"
    # 检查是否还有重复的xacro依赖
    xacro_count=$(grep -c "<depend>xacro</depend>" arm6/package.xml)
    if [ "$xacro_count" -eq 1 ]; then
        echo "✓ xacro依赖项已修复（只有1个）"
    else
        echo "✗ xacro依赖项仍有问题（发现$xacro_count个）"
        exit 1
    fi
else
    echo "✗ package.xml文件不存在"
    exit 1
fi

# 3. 复制修复后的文件到工作空间
echo "3. 复制修复后的文件到工作空间"
if [ ! -d "$HOME/ros2_ws/src" ]; then
    echo "创建ROS2工作空间目录"
    mkdir -p $HOME/ros2_ws/src
fi

# 复制整个arm6包到工作空间
echo "复制arm6包到工作空间..."
cp -r arm6 $HOME/ros2_ws/src/
echo "✓ arm6包已复制到工作空间"

# 4. 进入工作空间并构建
echo "4. 构建ROS2包"
cd $HOME/ros2_ws

# 清理之前的构建
echo "清理之前的构建文件..."
rm -rf build/ install/ log/

# 重新构建
echo "开始构建..."
colcon build --packages-select arm6 --symlink-install

if [ $? -eq 0 ]; then
    echo "✓ 构建成功！"
    
    # 5. 设置环境并测试
    echo "5. 测试安装"
    source install/setup.bash
    
    # 测试XACRO文件处理
    echo "测试XACRO文件处理..."
    if ros2 run xacro xacro src/arm6/urdf/arm6.urdf.xacro > /tmp/test_urdf.xml 2>/dev/null; then
        echo "✓ XACRO文件处理成功"
        urdf_lines=$(wc -l < /tmp/test_urdf.xml)
        echo "生成的URDF行数: $urdf_lines"
        rm -f /tmp/test_urdf.xml
    else
        echo "✗ XACRO文件处理失败"
    fi
    
    # 测试launch文件
    echo "测试launch文件可用性..."
    if ros2 launch arm6 display_xacro.launch.py --help > /dev/null 2>&1; then
        echo "✓ display_xacro.launch.py 可用"
    else
        echo "✗ display_xacro.launch.py 不可用"
    fi
    
    echo ""
    echo "=== 修复完成 ==="
    echo "现在可以使用以下命令启动机器人："
    echo "cd $HOME/ros2_ws"
    echo "source install/setup.bash"
    echo "ros2 launch arm6 display_xacro.launch.py"
    
else
    echo "✗ 构建失败！"
    echo "请检查错误信息并修复问题。"
    exit 1
fi