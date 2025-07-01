@echo off
chcp 65001 >nul
echo ========================================
echo ARM_URDF到ARM6项目迁移工具
echo ========================================
echo.

echo 正在启动WSL并执行迁移脚本...
echo.

REM 切换到脚本目录并执行
wsl -d Ubuntu-22.04 -e bash -c "cd '/mnt/f/F Download/simulation' && chmod +x migrate_arm_urdf.sh && ./migrate_arm_urdf.sh"

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ========================================
    echo ^✓ 迁移成功完成！
    echo ========================================
    echo.
    echo 现在可以启动机械臂可视化：
    echo.
    echo ROS2环境下：
    echo   wsl -d Ubuntu-22.04 -e bash -c "cd '/mnt/f/F Download/simulation/test_ws' ^&^& source /opt/ros/jazzy/setup.bash ^&^& source install/setup.bash ^&^& ros2 launch arm6 display.launch.py"
    echo.
    echo ROS1环境下：
    echo   wsl -d Ubuntu-22.04 -e bash -c "cd '/mnt/f/F Download/simulation/test_ws' ^&^& source /opt/ros/noetic/setup.bash ^&^& source devel/setup.bash ^&^& roslaunch arm6 display.launch"
    echo.
    echo Gazebo仿真：
    echo   wsl -d Ubuntu-22.04 -e bash -c "cd '/mnt/f/F Download/simulation/test_ws' ^&^& source /opt/ros/jazzy/setup.bash ^&^& source install/setup.bash ^&^& ros2 launch arm6 gazebo.launch.py"
    echo.
) else (
    echo.
    echo ========================================
    echo ^✗ 迁移过程中出现错误
    echo ========================================
    echo.
    echo 请检查以下可能的问题：
    echo 1. WSL是否正确安装和配置
    echo 2. Ubuntu-22.04是否已安装
    echo 3. ROS2 Jazzy是否正确安装
    echo 4. arm_urdf和arm6目录是否存在
    echo.
    echo 如需手动执行，请运行：
    echo   wsl -d Ubuntu-22.04
    echo   cd '/mnt/f/F Download/simulation'
    echo   chmod +x migrate_arm_urdf.sh
    echo   ./migrate_arm_urdf.sh
    echo.
)

echo 按任意键退出...
pause >nul