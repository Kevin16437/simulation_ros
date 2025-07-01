@echo off
echo === 测试ARM6关节轴配置 ===
echo.
echo 正在启动WSL并测试修正后的关节配置...
echo.

:: 确保脚本有执行权限并运行
wsl bash -c "cd '/mnt/f/F Download/simulation' && chmod +x test_joint_axes.sh && ./test_joint_axes.sh"

if %ERRORLEVEL% EQU 0 (
    echo.
    echo === 关节轴修正测试完成 ===
    echo.
    echo 修正内容:
    echo - j3关节: 从 ^(-1 0 0^) 修正为 ^(0 1 0^) - 绕Y轴旋转
    echo - j4关节: 从 ^(0 -1 0^) 修正为 ^(1 0 0^) - 绕X轴旋转
    echo.
    echo 现在可以在WSL中启动机械臂仿真:
    echo wsl
    echo cd ~/ros2_ws
    echo source install/setup.bash
    echo ros2 launch arm6 display_xacro.launch.py
) else (
    echo.
    echo === 测试过程中出现错误 ===
    echo 请检查上面的错误信息
)

pause