# ARM6机械臂ROS2仿真包

这是一个完整的6自由度机械臂ROS2仿真包，支持RViz2可视化和Gazebo物理仿真。

## 文件结构

```
arm6/
├── urdf/                    # URDF文件
│   ├── arm6.urdf           # 主URDF文件（已修复）
│   └── arm6_ros2_control.urdf.xacro  # 带ros2_control的版本
├── meshes/                  # STL网格文件
├── launch/                  # 启动文件
│   ├── display_ros2.launch.py      # RViz2显示
│   └── gazebo_ros2.launch.py       # Gazebo仿真
├── config/                  # 配置文件
│   ├── arm_controllers.yaml        # 控制器配置
│   └── arm6.rviz                   # RViz配置
├── worlds/                  # Gazebo世界文件
├── scripts/                 # Python脚本
└── package.xml             # 包描述文件
```

## 快速开始

### 1. 环境设置
```bash
# 运行自动安装脚本
chmod +x setup_ros2_workspace.sh
./setup_ros2_workspace.sh

# 重启终端或source环境
source ~/.bashrc
```

### 2. 复制包到工作空间
```bash
cp -r arm6 ~/ros2_ws/src/
cd ~/ros2_ws
```

### 3. 更新包配置
```bash
cd ~/ros2_ws/src/arm6
mv package_ros2.xml package.xml
mv CMakeLists_ros2.txt CMakeLists.txt
```

### 4. 构建包
```bash
cd ~/ros2_ws
colcon build --packages-select arm6
source install/setup.bash
```

### 5. 启动仿真
```bash
# 使用快速启动脚本
chmod +x quick_start.sh
./quick_start.sh

# 或者直接启动
ros2 launch arm6 display_ros2.launch.py      # RViz2可视化
ros2 launch arm6 gazebo_ros2.launch.py       # Gazebo仿真
```

## 主要修复内容

### URDF文件修复
- ✅ 修复了所有关节限制（从0改为合理值）
- ✅ 添加了Gazebo仿真插件
- ✅ 添加了transmission元素用于ros2_control
- ✅ 设置了材料属性

### ROS2兼容性
- ✅ 创建了ROS2格式的package.xml
- ✅ 更新了CMakeLists.txt
- ✅ 创建了Python格式的launch文件
- ✅ 添加了ros2_control配置

## 使用方法

### RViz2可视化
```bash
ros2 launch arm6 display_ros2.launch.py
```
这将启动RViz2并显示机械臂模型，您可以使用Joint State Publisher GUI控制关节。

### Gazebo仿真
```bash
ros2 launch arm6 gazebo_ros2.launch.py
```
这将启动Gazebo物理仿真环境。

### 控制机械臂
```bash
# 使用测试脚本
python3 ~/ros2_ws/src/arm6/scripts/test_arm_movement.py

# 或者手动发送轨迹命令
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/JointTrajectory ...
```

## 测试和验证

### 运行测试脚本
```bash
chmod +x test_arm6_setup.sh
./test_arm6_setup.sh
```

### 检查URDF语法
```bash
check_urdf ~/ros2_ws/src/arm6/urdf/arm6.urdf
```

### 查看关节状态
```bash
ros2 topic echo /joint_states
```

## 故障排除

### 常见问题

1. **包找不到**
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   ```

2. **Gazebo启动失败**
   ```bash
   sudo apt install gazebo
   ```

3. **控制器无法加载**
   - 检查控制器配置文件
   - 确保ros2_control包已安装

4. **网格文件找不到**
   - 确保STL文件在meshes目录中
   - 检查URDF中的文件路径

### 依赖包安装
```bash
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher-gui
```

## 进阶使用

### 自定义控制器
您可以编写自己的控制节点来控制机械臂：

```python
import rclpy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# 创建轨迹消息并发布到 /arm_controller/joint_trajectory
```

### MoveIt集成
可以集成MoveIt进行路径规划：
```bash
# 安装MoveIt
sudo apt install ros-humble-moveit
```

## 支持和贡献

如果您遇到问题或有改进建议，请：
1. 检查README和故障排除部分
2. 运行测试脚本诊断问题
3. 查看ROS2日志获取详细错误信息

## 许可证

BSD-3-Clause License
