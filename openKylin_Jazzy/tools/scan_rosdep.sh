#!/usr/bin/env bash
set -euo pipefail

# 用法： ./tools/scan_rosdep.sh [WORKSPACE]
# 功能：
#  1) 读取 scan_rosdep.sh 的缺包与候选
#  2) 用 rosdep keys/resolve 反查“哪个 rosdep 键映射到了缺失 APT 包”
#  3) 生成本地覆盖 YAML：对 noble 把这些键的包名替换为建议名（t64->非t64或邻近版本）
WORKSPACE="${1:-$(pwd)}"
OUTDIR_SCAN="${WORKSPACE}/.rosdep_scan"
KEYS_FILE="${OUTDIR_SCAN}/rosdep_keys.txt"
SUGG="${OUTDIR_SCAN}/apt_missing_suggestions.txt"
APT_MISSING_SYS="${OUTDIR_SCAN}/apt_missing_system.txt"
OVERLAY_DIR="${WORKSPACE}/rosdep-overlays"
OVERLAY_YAML="${OVERLAY_DIR}/ubuntu-noble-on-openkylin.yaml"

: "${ROS_OS_OVERRIDE:=ubuntu:24.04:noble}"
echo "[I] 使用 ROS_OS_OVERRIDE=${ROS_OS_OVERRIDE}"

if [[ ! -s "$APT_MISSING_SYS" || ! -s "$SUGG" ]]; then
  echo "[E] 未发现扫描产物。请先运行 ./tools/scan_rosdep.sh"
  exit 1
fi

mkdir -p "$OVERLAY_DIR"

echo "[1/4] 列出当前工作区涉及的 rosdep 键..."
rosdep keys --rosdistro jazzy --from-paths "${WORKSPACE}/src" --ignore-src \
  | sort -u > "$KEYS_FILE"

# 把建议表做成映射：missing_pkg -> suggested_pkg
declare -A MAP_SUGG
while read -r line; do
  [[ -z "${line:-}" ]] && continue
  left="$(echo "$line" | awk '{print $1}')"
  right="$(echo "$line" | awk '{print $3}')"
  [[ "$right" == "(no-suggestion)" ]] && continue
  MAP_SUGG["$left"]="$right"
done < "$SUGG"

echo "[2/4] 解析每个 rosdep 键在 noble 下的 APT 映射..."
TMPDIR="$(mktemp -d)"
trap 'rm -rf "$TMPDIR"' EXIT

OVERLAY_TMP="${TMPDIR}/overlay_body.yaml"
: > "$OVERLAY_TMP"

# 记录哪些键被覆盖了（去重用）
declare -A OV_KEYS

while read -r key; do
  [[ -z "${key:-}" ]] && continue
  # 解析此键的 apt 包（过滤注释行、空行）
  # 说明：不同 rosdep 版本输出格式略有差异，这里采取“剔注释后全收集非空行”的朴素法
  RESOLVED="$(ROS_OS_OVERRIDE="${ROS_OS_OVERRIDE}" rosdep resolve "$key" 2>/dev/null | sed '/^#/d;/^$/d')"
  [[ -z "${RESOLVED:-}" ]] && continue

  # 检测是否包含“缺失的系统库包”，并替换为建议名（若有）
  need_overlay=0
  new_list=()
  while read -r pkg; do
    [[ -z "${pkg:-}" ]] && continue
    if grep -qx "$pkg" "$APT_MISSING_SYS"; then
      if [[ -n "${MAP_SUGG[$pkg]:-}" ]]; then
        new_list+=("${MAP_SUGG[$pkg]}")
        need_overlay=1
      else
        # 没有建议就仍然放回原名（也可以选择跳过，这里保守处理）
        new_list+=("$pkg")
      fi
    else
      new_list+=("$pkg")
    fi
  done <<< "$RESOLVED"

  if [[ $need_overlay -eq 1 && -n "${new_list[*]:-}" && -z "${OV_KEYS[$key]:-}" ]]; then
    OV_KEYS["$key"]=1
    # 写入此键的覆盖，Ubuntu noble 使用 new_list
    {
      echo "${key}:"
      echo "  ubuntu:"
      echo "    noble: [$(printf "%s, " "${new_list[@]}" | sed 's/, $//')]"
    } >> "$OVERLAY_TMP"
  fi
done < "$KEYS_FILE"

echo "[3/4] 生成最终覆盖 YAML..."
{
  echo "# 本地覆盖：Ubuntu noble 规则在 openKylin 上的包名适配"
  echo "# 生成时间: $(date -u +"%F %T")"
  echo "# 提示：该文件仅覆盖“确实解析到缺失包”的 rosdep 键，其它键沿用上游。"
  echo
  cat "$OVERLAY_TMP"
} > "$OVERLAY_YAML"

echo "[4/4] 完成。覆盖文件： $OVERLAY_YAML"
echo
echo "== 接入步骤（一次性） =="
echo "sudo tee /etc/ros/rosdep/sources.list.d/99-openkylin-noble.list <<EOF"
echo "yaml file://${OVERLAY_YAML}"
echo "EOF"
echo "rosdep update"
echo
echo "== 之后使用 =="
echo "ROS_OS_OVERRIDE=${ROS_OS_OVERRIDE} rosdep install -y --rosdistro jazzy --from-paths src --ignore-src"
echo
echo "== 附送：自动生成的 skip-keys 候选（ROS 二进制包，持续从源码构建的话建议跳过） =="
if [[ -s "${OUTDIR_SCAN}/apt_missing_rosbins.txt" ]]; then
  echo "--skip-keys=\"$(tr '\n' ' ' < "${OUTDIR_SCAN}/apt_missing_rosbins.txt" | sed 's/ *$//')\""
else
  echo "(无)"
fi
