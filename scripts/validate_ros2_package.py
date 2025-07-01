#!/usr/bin/env python3

"""
ROS2包验证脚本
在没有ROS2环境的情况下验证包的基本结构和配置
"""

import os
import sys
import xml.etree.ElementTree as ET
import yaml
import json
from pathlib import Path

class Colors:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    NC = '\033[0m'  # No Color

class ROS2PackageValidator:
    def __init__(self, package_path):
        self.package_path = Path(package_path)
        self.errors = []
        self.warnings = []
        self.success = []
        
    def log_success(self, message):
        self.success.append(message)
        print(f"{Colors.GREEN}✓{Colors.NC} {message}")
        
    def log_warning(self, message):
        self.warnings.append(message)
        print(f"{Colors.YELLOW}!{Colors.NC} {message}")
        
    def log_error(self, message):
        self.errors.append(message)
        print(f"{Colors.RED}✗{Colors.NC} {message}")
        
    def validate_package_xml(self):
        """验证package.xml文件"""
        print(f"\n{Colors.BLUE}验证 package.xml{Colors.NC}")
        
        package_xml = self.package_path / "package.xml"
        if not package_xml.exists():
            self.log_error("package.xml 文件不存在")
            return False
            
        try:
            tree = ET.parse(package_xml)
            root = tree.getroot()
            
            # 检查基本元素
            if root.tag != "package":
                self.log_error("根元素不是 'package'")
                return False
                
            # 检查格式版本
            format_attr = root.get('format')
            if format_attr == '3':
                self.log_success("使用 package format 3 (ROS2)")
            elif format_attr == '2':
                self.log_warning("使用 package format 2 (可能是ROS1)")
            else:
                self.log_warning(f"未知的 package format: {format_attr}")
                
            # 检查必需元素
            required_elements = ['name', 'version', 'description', 'maintainer', 'license']
            for element in required_elements:
                if root.find(element) is not None:
                    self.log_success(f"包含 {element} 元素")
                else:
                    self.log_error(f"缺少 {element} 元素")
                    
            # 检查构建工具
            buildtool = root.find('buildtool_depend')
            if buildtool is not None and buildtool.text == 'ament_cmake':
                self.log_success("使用 ament_cmake (ROS2)")
            elif buildtool is not None and buildtool.text == 'catkin':
                self.log_warning("使用 catkin (ROS1)")
            else:
                self.log_warning("未指定构建工具")
                
            return True
            
        except ET.ParseError as e:
            self.log_error(f"XML解析错误: {e}")
            return False
            
    def validate_cmake_lists(self):
        """验证CMakeLists.txt文件"""
        print(f"\n{Colors.BLUE}验证 CMakeLists.txt{Colors.NC}")
        
        cmake_file = self.package_path / "CMakeLists.txt"
        if not cmake_file.exists():
            self.log_error("CMakeLists.txt 文件不存在")
            return False
            
        try:
            content = cmake_file.read_text()
            
            # 检查ROS2特征
            if "ament_cmake" in content:
                self.log_success("使用 ament_cmake (ROS2)")
            elif "catkin" in content:
                self.log_warning("使用 catkin (ROS1)")
            else:
                self.log_warning("未检测到ROS构建系统")
                
            # 检查必要的CMake命令
            required_commands = [
                "cmake_minimum_required",
                "project",
                "find_package"
            ]
            
            for cmd in required_commands:
                if cmd in content:
                    self.log_success(f"包含 {cmd} 命令")
                else:
                    self.log_error(f"缺少 {cmd} 命令")
                    
            return True
            
        except Exception as e:
            self.log_error(f"读取CMakeLists.txt失败: {e}")
            return False
            
    def validate_urdf(self):
        """验证URDF文件"""
        print(f"\n{Colors.BLUE}验证 URDF 文件{Colors.NC}")
        
        urdf_dir = self.package_path / "urdf"
        if not urdf_dir.exists():
            self.log_error("urdf 目录不存在")
            return False
            
        urdf_files = list(urdf_dir.glob("*.urdf"))
        if not urdf_files:
            self.log_error("未找到 .urdf 文件")
            return False
            
        for urdf_file in urdf_files:
            try:
                tree = ET.parse(urdf_file)
                root = tree.getroot()
                
                if root.tag != "robot":
                    self.log_error(f"{urdf_file.name}: 根元素不是 'robot'")
                    continue
                    
                # 检查基本元素
                links = root.findall("link")
                joints = root.findall("joint")
                
                self.log_success(f"{urdf_file.name}: 包含 {len(links)} 个链接")
                self.log_success(f"{urdf_file.name}: 包含 {len(joints)} 个关节")
                
                # 检查关节限制
                for joint in joints:
                    joint_name = joint.get('name', 'unnamed')
                    limit = joint.find('limit')
                    if limit is not None:
                        lower = limit.get('lower', '0')
                        upper = limit.get('upper', '0')
                        effort = limit.get('effort', '0')
                        velocity = limit.get('velocity', '0')
                        
                        if lower == '0' and upper == '0':
                            self.log_warning(f"关节 {joint_name} 的限制可能未正确设置")
                        else:
                            self.log_success(f"关节 {joint_name} 限制已设置")
                            
                # 检查ros2_control
                ros2_control = root.find("ros2_control")
                if ros2_control is not None:
                    self.log_success(f"{urdf_file.name}: 包含 ros2_control 配置")
                else:
                    self.log_warning(f"{urdf_file.name}: 缺少 ros2_control 配置")
                    
            except ET.ParseError as e:
                self.log_error(f"{urdf_file.name}: XML解析错误: {e}")
                
        return True
        
    def validate_launch_files(self):
        """验证launch文件"""
        print(f"\n{Colors.BLUE}验证 Launch 文件{Colors.NC}")
        
        launch_dir = self.package_path / "launch"
        if not launch_dir.exists():
            self.log_error("launch 目录不存在")
            return False
            
        # 检查ROS2 launch文件
        py_launch_files = list(launch_dir.glob("*.launch.py"))
        xml_launch_files = list(launch_dir.glob("*.launch"))
        
        if py_launch_files:
            self.log_success(f"找到 {len(py_launch_files)} 个 ROS2 Python launch 文件")
            for launch_file in py_launch_files:
                # 简单的语法检查
                try:
                    with open(launch_file, 'r') as f:
                        content = f.read()
                        if "LaunchDescription" in content:
                            self.log_success(f"{launch_file.name}: 包含 LaunchDescription")
                        else:
                            self.log_warning(f"{launch_file.name}: 可能不是有效的ROS2 launch文件")
                except Exception as e:
                    self.log_error(f"{launch_file.name}: 读取失败: {e}")
                    
        if xml_launch_files:
            self.log_warning(f"找到 {len(xml_launch_files)} 个 ROS1 XML launch 文件")
            
        return True
        
    def validate_config_files(self):
        """验证配置文件"""
        print(f"\n{Colors.BLUE}验证配置文件{Colors.NC}")
        
        config_dir = self.package_path / "config"
        if not config_dir.exists():
            self.log_warning("config 目录不存在")
            return True
            
        # 检查YAML文件
        yaml_files = list(config_dir.glob("*.yaml")) + list(config_dir.glob("*.yml"))
        
        for yaml_file in yaml_files:
            try:
                with open(yaml_file, 'r') as f:
                    yaml.safe_load(f)
                self.log_success(f"{yaml_file.name}: YAML格式正确")
            except yaml.YAMLError as e:
                self.log_error(f"{yaml_file.name}: YAML格式错误: {e}")
            except Exception as e:
                self.log_error(f"{yaml_file.name}: 读取失败: {e}")
                
        return True
        
    def validate_meshes(self):
        """验证网格文件"""
        print(f"\n{Colors.BLUE}验证网格文件{Colors.NC}")
        
        meshes_dir = self.package_path / "meshes"
        if not meshes_dir.exists():
            self.log_warning("meshes 目录不存在")
            return True
            
        stl_files = list(meshes_dir.glob("*.stl")) + list(meshes_dir.glob("*.STL"))
        dae_files = list(meshes_dir.glob("*.dae"))
        
        if stl_files:
            self.log_success(f"找到 {len(stl_files)} 个 STL 文件")
        if dae_files:
            self.log_success(f"找到 {len(dae_files)} 个 DAE 文件")
            
        if not stl_files and not dae_files:
            self.log_warning("未找到网格文件")
            
        return True
        
    def run_validation(self):
        """运行完整验证"""
        print(f"{Colors.BLUE}=== ROS2包验证开始 ==={Colors.NC}")
        print(f"验证包: {self.package_path}")
        
        if not self.package_path.exists():
            self.log_error(f"包路径不存在: {self.package_path}")
            return False
            
        # 运行所有验证
        self.validate_package_xml()
        self.validate_cmake_lists()
        self.validate_urdf()
        self.validate_launch_files()
        self.validate_config_files()
        self.validate_meshes()
        
        # 输出总结
        print(f"\n{Colors.BLUE}=== 验证总结 ==={Colors.NC}")
        print(f"{Colors.GREEN}成功: {len(self.success)}{Colors.NC}")
        print(f"{Colors.YELLOW}警告: {len(self.warnings)}{Colors.NC}")
        print(f"{Colors.RED}错误: {len(self.errors)}{Colors.NC}")
        
        if self.errors:
            print(f"\n{Colors.RED}需要修复的错误:{Colors.NC}")
            for error in self.errors:
                print(f"  - {error}")
                
        if self.warnings:
            print(f"\n{Colors.YELLOW}建议改进的警告:{Colors.NC}")
            for warning in self.warnings:
                print(f"  - {warning}")
                
        return len(self.errors) == 0

def main():
    if len(sys.argv) != 2:
        print("用法: python3 validate_ros2_package.py <package_path>")
        print("例如: python3 validate_ros2_package.py arm6")
        sys.exit(1)
        
    package_path = sys.argv[1]
    validator = ROS2PackageValidator(package_path)
    
    success = validator.run_validation()
    
    if success:
        print(f"\n{Colors.GREEN}✓ 包验证通过！{Colors.NC}")
        sys.exit(0)
    else:
        print(f"\n{Colors.RED}✗ 包验证失败，请修复错误后重试{Colors.NC}")
        sys.exit(1)

if __name__ == "__main__":
    main()
