#!/bin/bash

OBS_API="https://build.tarsier-infra.isrc.ac.cn"
OBS_PROJECT="home:Sebastianhayashi:ROS-Humble-openEuler"
SRPM_DIR="$HOME/ros_humble_srpm"
LOG_FILE="import-fixed-$(date +%Y%m%d-%H%M%S).log"
FAILED_FILE="failed-imports-$(date +%Y%m%d-%H%M%S).txt"

# 确保在项目工作副本中
cd ~/home:Sebastianhayashi:ROS-Humble-openEuler || {
    echo "错误：无法进入项目目录"
    exit 1
}

# 统计
total=0
success=0
failed=0
skipped=0

# 获取所有 SRPM
srpms=("$SRPM_DIR"/*.src.rpm)

echo "开始导入并提交 SRPM 包" | tee "$LOG_FILE"
echo "时间: $(date)" | tee -a "$LOG_FILE"
echo "找到 ${#srpms[@]} 个 SRPM 文件" | tee -a "$LOG_FILE"
echo "================================" | tee -a "$LOG_FILE"

for srpm in "${srpms[@]}"; do
    ((total++))
    
    # 获取包名
    pkg_name=$(rpm -qp --queryformat '%{NAME}' "$srpm" 2>/dev/null)
    
    if [ -z "$pkg_name" ]; then
        echo "[$total/${#srpms[@]}] 错误：无法获取包名 - $srpm" | tee -a "$LOG_FILE"
        ((failed++))
        echo "$srpm" >> "$FAILED_FILE"
        continue
    fi
    
    # 检查包是否已存在并已提交
    if osc -A "$OBS_API" ls "$OBS_PROJECT" 2>/dev/null | grep -q "^${pkg_name}$"; then
        echo "[$total/${#srpms[@]}] 跳过已在服务器: $pkg_name" | tee -a "$LOG_FILE"
        ((skipped++))
        continue
    fi
    
    # 检查本地是否已有目录但未提交
    if [ -d "$pkg_name" ]; then
        echo "[$total/${#srpms[@]}] 发现本地目录，尝试提交: $pkg_name" | tee -a "$LOG_FILE"
        cd "$pkg_name"
        
        # 检查是否有文件需要添加
        if osc -A "$OBS_API" status 2>/dev/null | grep -q "^?"; then
            osc -A "$OBS_API" add * >/dev/null 2>&1
        fi
        
        # 尝试提交
        if osc -A "$OBS_API" ci -m "Import $pkg_name from SRPM" >/dev/null 2>&1; then
            ((success++))
            echo "  ✓ 成功提交本地包" | tee -a "$LOG_FILE"
        else
            ((failed++))
            echo "  ✗ 提交失败" | tee -a "$LOG_FILE"
            echo "$srpm" >> "$FAILED_FILE"
        fi
        cd ..
        continue
    fi
    
    echo "[$total/${#srpms[@]}] 导入: $pkg_name" | tee -a "$LOG_FILE"
    
    # 导入包（忽略返回值，因为即使"失败"也可能创建了目录）
    osc -A "$OBS_API" importsrcpkg "$srpm" >/dev/null 2>&1
    
    # 检查是否创建了目录
    if [ -d "$pkg_name" ]; then
        cd "$pkg_name"
        
        # 添加所有文件
        if osc -A "$OBS_API" add * >/dev/null 2>&1; then
            # 提交到服务器
            if osc -A "$OBS_API" ci -m "Import $pkg_name from SRPM" >/dev/null 2>&1; then
                ((success++))
                echo "  ✓ 成功导入并提交" | tee -a "$LOG_FILE"
            else
                ((failed++))
                echo "  ✗ 提交失败" | tee -a "$LOG_FILE"
                echo "$srpm" >> "$FAILED_FILE"
            fi
        else
            ((failed++))
            echo "  ✗ 添加文件失败" | tee -a "$LOG_FILE"
            echo "$srpm" >> "$FAILED_FILE"
        fi
        
        cd ..
    else
        ((failed++))
        echo "  ✗ 导入失败（未创建目录）" | tee -a "$LOG_FILE"
        echo "$srpm" >> "$FAILED_FILE"
    fi
    
    # 每10个包显示进度
    if (( total % 10 == 0 )); then
        echo "进度: $total/${#srpms[@]}, 成功: $success, 失败: $failed, 跳过: $skipped" | tee -a "$LOG_FILE"
        
        # 验证服务器上的包数量
        server_count=$(osc -A "$OBS_API" ls "$OBS_PROJECT" 2>/dev/null | wc -l)
        echo "服务器上的包数量: $server_count" | tee -a "$LOG_FILE"
    fi
done

echo "================================" | tee -a "$LOG_FILE"
echo "导入完成！" | tee -a "$LOG_FILE"
echo "总计: $total 个包" | tee -a "$LOG_FILE"
echo "成功: $success 个包" | tee -a "$LOG_FILE"
echo "失败: $failed 个包" | tee -a "$LOG_FILE"
echo "跳过: $skipped 个包" | tee -a "$LOG_FILE"

# 最终验证
echo "" | tee -a "$LOG_FILE"
echo "验证服务器上的包..." | tee -a "$LOG_FILE"
server_count=$(osc -A "$OBS_API" ls "$OBS_PROJECT" 2>/dev/null | wc -l)
echo "服务器上总共有 $server_count 个包" | tee -a "$LOG_FILE"

if [ $failed -gt 0 ]; then
    echo "" | tee -a "$LOG_FILE"
    echo "失败的包已记录在: $FAILED_FILE" | tee -a "$LOG_FILE"
fi
