#!/usr/bin/env python3
"""
批量执行 apt search 命令并生成报告
适用于 openkylin2.0 系统
"""

import argparse
import subprocess
import os
import re
from datetime import datetime
import sys

def ensure_logs_dir():
    """确保 logs 目录存在"""
    if not os.path.exists('logs'):
        os.makedirs('logs')
        print("创建 logs 目录")

def parse_input_file(input_file):
    """解析输入文件，提取包名"""
    packages = []
    
    try:
        with open(input_file, 'r', encoding='utf-8') as f:
            content = f.read()
            
        # 使用正则表达式匹配包名
        pattern = r'\[PROGRESS\]\s+-\s+([^\s]+)\s+\(search_require:\s+y\)'
        matches = re.findall(pattern, content)
        
        for match in matches:
            packages.append(match)
            
        print(f"从输入文件中解析出 {len(packages)} 个包")
        return packages
        
    except Exception as e:
        print(f"解析输入文件时出错: {e}")
        sys.exit(1)

def execute_apt_search(package_name):
    """执行 apt search 命令并返回结果"""
    try:
        # 执行 apt search 命令
        cmd = ['apt', 'search', package_name]
        result = subprocess.run(cmd, 
                              capture_output=True, 
                              text=True, 
                              encoding='utf-8')
        
        # 获取输出
        output = result.stdout
        error = result.stderr
        
        # 如果有错误输出，也包含在结果中
        if error:
            output += f"\n错误信息:\n{error}"
            
        return output
        
    except Exception as e:
        return f"执行命令时出错: {e}"

def save_log(package_name, output):
    """保存单次搜索的日志"""
    ensure_logs_dir()
    
    # 使用当前时间作为文件名
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # 精确到毫秒
    filename = f"logs/{timestamp}_{package_name}.log"
    
    try:
        with open(filename, 'w', encoding='utf-8') as f:
            f.write(f"包名: {package_name}\n")
            f.write(f"搜索时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("="*50 + "\n")
            f.write(output)
            
        print(f"  日志已保存到: {filename}")
        
    except Exception as e:
        print(f"  保存日志时出错: {e}")

def generate_markdown_report(results):
    """生成 Markdown 格式的报告"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_filename = f"apt_search_report_{timestamp}.md"
    
    try:
        with open(report_filename, 'w', encoding='utf-8') as f:
            # 写入报告头部
            f.write("# APT Search 批量搜索报告\n\n")
            f.write(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            f.write(f"搜索包数量: {len(results)}\n\n")
            f.write("---\n\n")
            
            # 写入每个包的搜索结果
            for package_name, output in results:
                f.write(f"- searched_pkg: {package_name}\n")
                f.write("Results:\n")
                f.write("```\n")
                f.write(output.strip())
                f.write("\n```\n\n")
                
        print(f"\nMarkdown 报告已生成: {report_filename}")
        return report_filename
        
    except Exception as e:
        print(f"生成 Markdown 报告时出错: {e}")
        return None

def main():
    # 设置命令行参数
    parser = argparse.ArgumentParser(description='批量执行 apt search 并生成报告')
    parser.add_argument('--input', '-i', required=True, 
                       help='输入文件路径，包含要搜索的包列表')
    
    args = parser.parse_args()
    
    print(f"开始处理输入文件: {args.input}")
    print("="*60)
    
    # 解析输入文件
    packages = parse_input_file(args.input)
    
    if not packages:
        print("没有找到需要搜索的包")
        return
    
    # 存储所有结果
    results = []
    
    # 对每个包执行搜索
    print("\n开始执行 apt search:")
    print("-"*60)
    
    for i, package in enumerate(packages, 1):
        print(f"\n[{i}/{len(packages)}] 正在搜索: {package}")
        
        # 执行搜索
        output = execute_apt_search(package)
        
        # 保存日志
        save_log(package, output)
        
        # 输出到终端
        print(f"\n- searched_pkg: {package}")
        print("Results:")
        print("-"*40)
        print(output)
        print("-"*40)
        
        # 保存结果
        results.append((package, output))
    
    # 生成 Markdown 报告
    print("\n" + "="*60)
    report_file = generate_markdown_report(results)
    
    print("\n搜索完成！")
    print(f"- 搜索包数量: {len(packages)}")
    print(f"- 日志文件保存在: logs/")
    if report_file:
        print(f"- Markdown 报告: {report_file}")

if __name__ == "__main__":
    main()
