#!/bin/bash

# ARM6项目调试脚本 - 在当前目录调试
# 此脚本用于在Windows环境下调试ROS2项目

echo "=== ARM6项目调试脚本 ==="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 获取当前目录
CURRENT_DIR=$(pwd)
echo -e "${BLUE}当前工作目录: $CURRENT_DIR${NC}"

# 1. 检查文件结构
echo -e "\n${YELLOW}1. 检查项目文件结构${NC}"

required_files=(
    "arm6/package.xml"
    "arm6/CMakeLists.txt"
    "arm6/urdf/arm6.urdf"
    "arm6/launch/display_ros2.launch.py"
    "arm6/launch/gazebo_ros2.launch.py"
    "arm6/config/arm_controllers.yaml"
    "arm6/config/arm6.rviz"
    "arm6/worlds/empty.world"
    "arm6/scripts/test_arm_movement.py"
)

for file in "${required_files[@]}"; do
    if [ -f "$file" ]; then
        echo -e "${GREEN}✓${NC} $file 存在"
    else
        echo -e "${RED}✗${NC} $file 不存在"
    fi
done

# 2. 检查STL网格文件
echo -e "\n${YELLOW}2. 检查STL网格文件${NC}"
stl_files=(
    "arm6/meshes/base_link.STL"
    "arm6/meshes/Link1.STL"
    "arm6/meshes/Link2.STL"
    "arm6/meshes/Link3.STL"
    "arm6/meshes/Link4.STL"
    "arm6/meshes/Link5.STL"
    "arm6/meshes/Link6.STL"
)

for file in "${stl_files[@]}"; do
    if [ -f "$file" ]; then
        echo -e "${GREEN}✓${NC} $file 存在"
    else
        echo -e "${RED}✗${NC} $file 不存在"
    fi
done

# 3. 检查Python脚本语法
echo -e "\n${YELLOW}3. 检查Python脚本语法${NC}"
python_files=(
    "arm6/launch/display_ros2.launch.py"
    "arm6/launch/gazebo_ros2.launch.py"
    "arm6/scripts/test_arm_movement.py"
)

for file in "${python_files[@]}"; do
    if [ -f "$file" ]; then
        if python3 -m py_compile "$file" 2>/dev/null; then
            echo -e "${GREEN}✓${NC} $file 语法正确"
        else
            echo -e "${RED}✗${NC} $file 语法错误"
        fi
    fi
done

# 4. 检查YAML配置文件
echo -e "\n${YELLOW}4. 检查YAML配置文件${NC}"
if [ -f "arm6/config/arm_controllers.yaml" ]; then
    if python3 -c "import yaml; yaml.safe_load(open('arm6/config/arm_controllers.yaml'))" 2>/dev/null; then
        echo -e "${GREEN}✓${NC} arm_controllers.yaml 格式正确"
    else
        echo -e "${RED}✗${NC} arm_controllers.yaml 格式错误"
    fi
fi

# 5. 检查URDF文件基本结构
echo -e "\n${YELLOW}5. 检查URDF文件基本结构${NC}"
if [ -f "arm6/urdf/arm6.urdf" ]; then
    # 检查是否包含必要的元素
    if grep -q "<robot" "arm6/urdf/arm6.urdf"; then
        echo -e "${GREEN}✓${NC} URDF包含robot元素"
    else
        echo -e "${RED}✗${NC} URDF缺少robot元素"
    fi
    
    if grep -q "<link" "arm6/urdf/arm6.urdf"; then
        echo -e "${GREEN}✓${NC} URDF包含link元素"
    else
        echo -e "${RED}✗${NC} URDF缺少link元素"
    fi
    
    if grep -q "<joint" "arm6/urdf/arm6.urdf"; then
        echo -e "${GREEN}✓${NC} URDF包含joint元素"
    else
        echo -e "${RED}✗${NC} URDF缺少joint元素"
    fi
    
    if grep -q "ros2_control" "arm6/urdf/arm6.urdf"; then
        echo -e "${GREEN}✓${NC} URDF包含ros2_control配置"
    else
        echo -e "${RED}✗${NC} URDF缺少ros2_control配置"
    fi
fi

# 6. 创建测试工作空间
echo -e "\n${YELLOW}6. 创建测试工作空间${NC}"
TEST_WS="$CURRENT_DIR/test_ws"

if [ ! -d "$TEST_WS" ]; then
    mkdir -p "$TEST_WS/src"
    echo -e "${GREEN}✓${NC} 创建测试工作空间: $TEST_WS"
else
    echo -e "${YELLOW}!${NC} 测试工作空间已存在: $TEST_WS"
fi

# 复制arm6包到测试工作空间
if [ -d "arm6" ]; then
    cp -r arm6 "$TEST_WS/src/"
    echo -e "${GREEN}✓${NC} 复制arm6包到测试工作空间"
fi

# 7. 生成使用说明
echo -e "\n${YELLOW}7. 生成使用说明${NC}"

cat > "$CURRENT_DIR/USAGE_INSTRUCTIONS.md" << 'EOF'
# ARM6机械臂项目使用说明

## 当前项目状态
项目已配置为ROS2兼容格式，包含以下主要组件：
- URDF机械臂模型（已修复关节限制）
- ROS2 launch文件
- Gazebo仿真配置
- ros2_control控制器配置

## 在WSL2中使用步骤

### 1. 准备WSL2环境
```bash
# 确保在WSL2 Ubuntu 24.04中
# 复制项目到WSL2
cp -r /mnt/f/F\ Download/simulation ~/arm6_project
cd ~/arm6_project
```

### 2. 安装ROS2环境
```bash
# 运行安装脚本
chmod +x setup_ros2_workspace.sh
./setup_ros2_workspace.sh
source ~/.bashrc
```

### 3. 设置工作空间
```bash
# 创建ROS2工作空间
mkdir -p ~/ros2_ws/src
cp -r arm6 ~/ros2_ws/src/
cd ~/ros2_ws
```

### 4. 构建项目
```bash
# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 构建包
colcon build --packages-select arm6
source install/setup.bash
```

### 5. 运行仿真
```bash
# RViz2可视化
ros2 launch arm6 display_ros2.launch.py

# Gazebo仿真（新终端）
ros2 launch arm6 gazebo_ros2.launch.py
```

## 故障排除

### 常见问题
1. **包找不到**: 确保source了正确的setup.bash
2. **Gazebo启动失败**: 安装gazebo包
3. **控制器加载失败**: 检查ros2_control相关包

### 调试命令
```bash
# 检查话题
ros2 topic list

# 检查节点
ros2 node list

# 检查参数
ros2 param list
```
EOF

echo -e "${GREEN}✓${NC} 生成使用说明: USAGE_INSTRUCTIONS.md"

# 8. 总结
echo -e "\n${BLUE}=== 调试总结 ===${NC}"
echo "项目文件已检查完毕。"
echo "测试工作空间已创建在: $TEST_WS"
echo "使用说明已生成: USAGE_INSTRUCTIONS.md"
echo ""
echo -e "${YELLOW}下一步操作：${NC}"
echo "1. 将项目复制到WSL2环境"
echo "2. 运行 setup_ros2_workspace.sh 安装ROS2"
echo "3. 按照 USAGE_INSTRUCTIONS.md 中的步骤操作"
echo ""
echo -e "${GREEN}项目调试完成！${NC}"
