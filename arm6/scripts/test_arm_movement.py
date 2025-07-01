#!/usr/bin/env python3

"""
ARM6机械臂测试控制脚本
此脚本用于测试机械臂的基本运动功能
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import time

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # 创建发布器
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        
        # 关节名称
        self.joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        
        self.get_logger().info('ARM6控制器已启动')
        
    def send_joint_trajectory(self, positions, duration_sec=3.0):
        """发送关节轨迹命令"""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        
        msg.points = [point]
        
        self.publisher.publish(msg)
        self.get_logger().info(f'发送关节位置: {positions}')
        
    def move_to_home(self):
        """移动到初始位置"""
        home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.send_joint_trajectory(home_position)
        
    def demo_movement(self):
        """演示运动序列"""
        movements = [
            # 位置1：初始位置
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            # 位置2：第一关节旋转
            [1.57, 0.0, 0.0, 0.0, 0.0, 0.0],
            # 位置3：第二关节弯曲
            [1.57, -0.5, 0.0, 0.0, 0.0, 0.0],
            # 位置4：第三关节弯曲
            [1.57, -0.5, 0.5, 0.0, 0.0, 0.0],
            # 位置5：末端关节旋转
            [1.57, -0.5, 0.5, 0.0, 0.0, 1.57],
            # 返回初始位置
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ]
        
        for i, position in enumerate(movements):
            self.get_logger().info(f'执行动作 {i+1}/{len(movements)}')
            self.send_joint_trajectory(position, 4.0)
            time.sleep(5.0)  # 等待运动完成
            
    def sine_wave_movement(self):
        """正弦波运动测试"""
        self.get_logger().info('开始正弦波运动测试')
        
        for t in range(100):
            angle = math.sin(t * 0.1) * 0.5  # 幅度0.5弧度
            positions = [angle, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.send_joint_trajectory(positions, 0.5)
            time.sleep(0.5)

def main():
    rclpy.init()
    
    controller = ArmController()
    
    try:
        print("ARM6机械臂控制测试")
        print("请确保Gazebo仿真已启动并且控制器已加载")
        print("\n选择测试模式:")
        print("1. 移动到初始位置")
        print("2. 演示运动序列")
        print("3. 正弦波运动测试")
        print("4. 手动控制模式")
        
        choice = input("请输入选择 (1-4): ")
        
        if choice == '1':
            controller.move_to_home()
            print("移动到初始位置")
            
        elif choice == '2':
            controller.demo_movement()
            print("演示运动完成")
            
        elif choice == '3':
            controller.sine_wave_movement()
            print("正弦波运动测试完成")
            
        elif choice == '4':
            print("手动控制模式")
            print("输入6个关节角度，用空格分隔 (弧度)")
            print("例如: 0.5 -0.3 0.8 0.0 -0.5 1.0")
            print("输入 'q' 退出")
            
            while True:
                user_input = input("关节角度: ")
                if user_input.lower() == 'q':
                    break
                    
                try:
                    positions = [float(x) for x in user_input.split()]
                    if len(positions) != 6:
                        print("请输入6个角度值")
                        continue
                        
                    controller.send_joint_trajectory(positions)
                    print(f"发送位置: {positions}")
                    
                except ValueError:
                    print("输入格式错误，请输入数字")
                    
        else:
            print("无效选择")
            
    except KeyboardInterrupt:
        print("\n程序被用户中断")
        
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
