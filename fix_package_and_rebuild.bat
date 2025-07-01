@echo off
echo === 修复package.xml并重新构建ARM6项目 ===
echo.
echo 正在启动WSL并运行修复脚本...
echo.

:: 确保脚本有执行权限并运行
wsl bash -c "cd '/mnt/f/F Download/simulation' && chmod +x fix_package_and_rebuild.sh && ./fix_package_and_rebuild.sh"

if %ERRORLEVEL% EQU 0 (
    echo.
    echo === 修复成功完成 ===
    echo 现在可以在WSL中使用以下命令启动机器人：
    echo wsl
    echo cd ~/ros2_ws
    echo source install/setup.bash
    echo ros2 launch arm6 display_xacro.launch.py
) else (
    echo.
    echo === 修复过程中出现错误 ===
    echo 请检查上面的错误信息
)

pause