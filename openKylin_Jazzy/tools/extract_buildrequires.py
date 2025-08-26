#!/usr/bin/env python3
"""
Script to extract build requirements for ROS Jazzy ros_core on Ubuntu 24.04
提取 ROS Jazzy ros_core 在 Ubuntu 24.04 上的构建依赖清单
"""

import os
import json
import subprocess
import re
from datetime import datetime
from pathlib import Path
import sys
from collections import defaultdict

class ROSBuildRequiresExtractor:
    def __init__(self, work_dir="verify_json"):
        self.work_dir = Path.home() / work_dir
        self.ros_packages = set()
        self.system_packages = set()
        self.all_dependencies = {}
        self.processed_packages = set()
        # 使用 defaultdict 来收集 required_by 信息
        self.dependencies_map = defaultdict(set)

    def setup_workspace(self):
        """创建工作目录结构"""
        print(f"[INFO] Setting up workspace in {self.work_dir}")
        self.work_dir.mkdir(exist_ok=True)
        (self.work_dir / "logs").mkdir(exist_ok=True)
        (self.work_dir / "cache").mkdir(exist_ok=True)

    def check_environment(self):
        """检查环境是否满足要求"""
        print("[INFO] Checking environment...")

        # 检查是否为Ubuntu 24.04
        try:
            with open('/etc/os-release', 'r') as f:
                content = f.read()
                if 'Ubuntu 24.04' not in content and 'Ubuntu 24.10' not in content:
                    print("[WARNING] This script is designed for Ubuntu 24.04/24.10")
                    response = input("Continue anyway? (y/n): ")
                    if response.lower() != 'y':
                        sys.exit(1)
        except:
            print("[WARNING] Cannot verify Ubuntu version")

        # 检查必要的命令是否存在
        required_commands = ['apt-cache', 'dpkg-query']
        for cmd in required_commands:
            if subprocess.run(['which', cmd], capture_output=True).returncode != 0:
                print(f"[ERROR] Required command '{cmd}' not found")
                sys.exit(1)

    def update_apt_cache(self):
        """更新apt缓存"""
        print("[INFO] Updating apt cache...")
        try:
            subprocess.run(['sudo', 'apt', 'update'], check=True)
        except subprocess.CalledProcessError:
            print("[WARNING] Failed to update apt cache, continuing with existing cache")

    def get_package_dependencies(self, package_name):
        """获取单个包的直接依赖"""
        if package_name in self.processed_packages:
            return []

        self.processed_packages.add(package_name)
        dependencies = []

        try:
            # 使用apt-cache获取依赖信息
            result = subprocess.run(
                ['apt-cache', 'depends', package_name],
                capture_output=True,
                text=True,
                check=True
            )

            # 解析输出
            for line in result.stdout.split('\n'):
                line = line.strip()
                if line.startswith('Depends:') or line.startswith('PreDepends:'):
                    # 提取包名（处理版本信息和或依赖）
                    match = re.search(r'(?:Depends:|PreDepends:)\s+<?([a-zA-Z0-9\-\+\.]+)', line)
                    if match:
                        dep_name = match.group(1)
                        dependencies.append(dep_name)

        except subprocess.CalledProcessError:
            print(f"[WARNING] Failed to get dependencies for {package_name}")

        return dependencies

    def recursive_get_dependencies(self, package_name, depth=0, max_depth=10, ros_parent=None):
        """递归获取所有依赖
        
        Args:
            package_name: 当前处理的包名
            depth: 递归深度
            max_depth: 最大递归深度
            ros_parent: 最近的 ROS 父包（用于记录 required_by）
        """
        if depth > max_depth:
            return

        if package_name in self.all_dependencies:
            # 如果已经处理过，只需要更新required_by
            if ros_parent and not package_name.startswith('ros-jazzy-'):
                self.dependencies_map[package_name].add(ros_parent)
            return

        print(f"[INFO] {'  ' * depth}Processing: {package_name}")

        # 获取直接依赖
        direct_deps = self.get_package_dependencies(package_name)
        self.all_dependencies[package_name] = direct_deps

        # 分类包
        if package_name.startswith('ros-jazzy-'):
            self.ros_packages.add(package_name)
            # 如果是 ROS 包，它将成为其依赖的系统包的 ros_parent
            current_ros_parent = package_name
        else:
            self.system_packages.add(package_name)
            # 记录被哪个 ROS 包依赖
            if ros_parent:
                self.dependencies_map[package_name].add(ros_parent)
            current_ros_parent = ros_parent  # 继续传递 ROS 父包

        # 递归处理依赖
        for dep in direct_deps:
            self.recursive_get_dependencies(dep, depth + 1, max_depth, ros_parent=current_ros_parent)

    def generate_buildrequire_entries(self):
        """生成构建依赖条目列表"""
        require_entries = []
        
        # 为每个系统包创建条目
        for pkg_name in sorted(self.system_packages):
            required_by = sorted(list(self.dependencies_map.get(pkg_name, [])))
            
            entry = {
                "require_pkg": pkg_name,
                "system_pkg": "unknown",
                "install_verify": "n",
                "search_require": "n",
                "miss": "unknown",
                "required_by": required_by
            }
            require_entries.append(entry)
        
        return require_entries

    def save_results(self, require_entries):
        """保存结果到文件"""
        # 保存为buildrequire_list_raw.json
        output_file = self.work_dir / "buildrequire_list_raw.json"

        # 按照要求的格式保存
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(require_entries, f, indent=2, ensure_ascii=False)

        print(f"\nResults saved to: {output_file}")

    def print_summary(self, require_entries):
        """打印摘要信息"""
        print("\n" + "=" * 60)
        print("DEPENDENCY EXTRACTION SUMMARY")
        print("=" * 60)
        print(f"Total ROS packages processed: {len(self.ros_packages)}")
        print(f"Total system dependencies found: {len(require_entries)}")
        print(f"Generated at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        
        # 显示被最多包依赖的系统包
        print("\nMost required dependencies:")
        sorted_deps = sorted(require_entries, key=lambda x: len(x['required_by']), reverse=True)
        for dep in sorted_deps[:10]:
            if dep['required_by']:
                print(f"  {dep['require_pkg']}: required by {len(dep['required_by'])} packages")
        
        print("=" * 60)

    def run(self):
        """执行主流程"""
        print("[INFO] Starting ROS Core build requirements extraction...")

        # 设置工作空间
        self.setup_workspace()

        # 检查环境
        self.check_environment()

        # 更新apt缓存
        self.update_apt_cache()

        # 开始递归获取依赖
        print("\n[INFO] Extracting dependencies for ros-jazzy-ros-core...")
        self.recursive_get_dependencies("ros-jazzy-ros-core")

        # 生成构建依赖条目
        require_entries = self.generate_buildrequire_entries()

        # 保存结果
        self.save_results(require_entries)

        # 打印摘要
        self.print_summary(require_entries)

        print(f"\n[SUCCESS] Dependency extraction completed!")
        print(f"[INFO] Next step: Run verification script to check package availability")
        print(f"[INFO] Results saved in: {self.work_dir}/buildrequire_list_raw.json")


if __name__ == "__main__":
    extractor = ROSBuildRequiresExtractor()
    try:
        extractor.run()
    except KeyboardInterrupt:
        print("\n[INFO] Process interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n[ERROR] An error occurred: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
