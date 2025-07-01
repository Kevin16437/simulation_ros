# ARM_URDF 机器人描述包

## 概述

这是一个适配ROS2环境的机器人描述包，包含了6自由度机械臂的URDF模型文件、网格文件和启动文件。该包已从原始的ROS1 catkin包成功迁移到ROS2 ament_cmake包。

## 包结构

```
arm_urdf/
├── CMakeLists.txt          # ROS2 ament_cmake构建文件
├── package.xml             # ROS2包描述文件
├── README.md               # 本文件
├── config/                 # 配置文件目录
│   ├── arm_urdf.rviz      # RViz2配置文件
│   └── joint_names_arm2.0.yaml
├── launch/                 # 启动文件目录
│   ├── display.launch      # ROS1启动文件（保留）
│   ├── display_ros2.launch.py  # ROS2启动文件
│   └── gazebo.launch       # Gazebo启动文件（保留）
├── meshes/                 # 3D网格文件目录
│   ├── base_link.STL
│   ├── Link1.STL
│   ├── Link2.STL
│   ├── Link3.STL
│   ├── Link4.STL
│   ├── Link5.STL
│   └── Link6.STL
└── urdf/                   # URDF文件目录
    ├── arm2.0.csv
    └── arm2.0.urdf         # 主要的URDF描述文件
```

## 主要更改

### 从ROS1到ROS2的适配

1. **包描述文件更新**：
   - 更新`package.xml`为format="3"格式
   - 替换catkin依赖为ament_cmake依赖
   - 添加ROS2特定的依赖项（rviz2, xacro等）

2. **构建系统更新**：
   - 更新`CMakeLists.txt`使用ament_cmake
   - 更新安装目录结构

3. **URDF文件更新**：
   - 将所有包引用从`package://arm2.0`更新为`package://arm_urdf`
   - 保持原有的机器人结构和参数不变

4. **启动文件**：
   - 创建新的ROS2 Python启动文件`display_ros2.launch.py`
   - 保留原有的ROS1启动文件以便兼容

5. **配置文件**：
   - 创建适配ROS2的RViz配置文件

## 使用方法

### 构建包

```bash
# 在ROS2工作空间中
cd ~/ros2_ws
colcon build --packages-select arm_urdf
source install/setup.bash
```

### 启动机器人模型显示

```bash
# 使用ROS2启动文件
ros2 launch arm_urdf display_ros2.launch.py
```

这将启动：
- Robot State Publisher（发布机器人状态）
- Joint State Publisher GUI（关节状态控制界面）
- RViz2（3D可视化）

### 查看机器人模型

启动后，您可以在RViz2中看到机器人模型，并通过Joint State Publisher GUI控制各个关节的角度。

## 依赖项

- ROS2 (推荐Humble或更新版本)
- robot_state_publisher
- joint_state_publisher
- joint_state_publisher_gui
- xacro
- urdf
- rviz2

## 注意事项

1. 确保所有STL网格文件都在`meshes/`目录中
2. URDF文件中的所有包引用都已更新为`arm_urdf`
3. 如果需要在Gazebo中使用，可能需要额外的配置

## 故障排除

如果遇到包找不到的错误，请确保：
1. 包已正确构建
2. 工作空间已正确source
3. 所有依赖项都已安装

```bash
# 检查包是否可用
ros2 pkg list | grep arm_urdf

# 检查启动文件
ros2 launch arm_urdf --show-args display_ros2.launch.py
```