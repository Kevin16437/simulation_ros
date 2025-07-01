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
