#!/usr/bin/env bash
# ============================================================================
# 把 ~ 下的四棵源目录增量同步进本仓库（本仓库是镜像拷贝，源目录不动）
# 用法:  ./sync_to_repo.sh            # 只同步
#        ./sync_to_repo.sh --commit  # 同步 + git add/commit（不 push）
# ============================================================================
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_ROOT="${SRC_ROOT:-$HOME}"

EXCLUDES=(
  --exclude='.git/'
  --exclude='__pycache__/'
  --exclude='*.pyc'
  --exclude='build/'
  --exclude='install/'
  --exclude='log/'
  --exclude='devel/'
  --exclude='managed_components/'
  --exclude='.pio/'
  --exclude='.pytest_cache/'
  --exclude='.mypy_cache/'
  --exclude='*.egg-info/'
  --exclude='*.pbstream'
  --exclude='*.pbf'
  --exclude='bags/'
  --exclude='leap_ros.tar'
)

echo ">>> 同步源目录: $SRC_ROOT"
rsync -a "${EXCLUDES[@]}" "$SRC_ROOT/xuegeros_ws/src"    "$REPO/xuegeros_ws/"
rsync -a "${EXCLUDES[@]}" "$SRC_ROOT/xuegeros_ws/maps"   "$REPO/xuegeros_ws/"
rsync -a "${EXCLUDES[@]}" "$SRC_ROOT/xuegeros_ws/models" "$REPO/xuegeros_ws/"
rsync -a "${EXCLUDES[@]}" "$SRC_ROOT/leap_demo"          "$REPO/"
rsync -a "${EXCLUDES[@]}" "$SRC_ROOT/semantic_slam_ws/src" "$REPO/semantic_slam_ws/"
rsync -a "${EXCLUDES[@]}" "$SRC_ROOT/YDLidar-SDK"        "$REPO/"

echo ">>> 同步完成"
git -C "$REPO" status --short | head -20
echo ">>> 变更条数: $(git -C "$REPO" status --short | wc -l)"

if [[ "${1:-}" == "--commit" ]]; then
  git -C "$REPO" add -A
  git -C "$REPO" commit -m "sync: 从本地工作区同步 $(date '+%Y-%m-%d %H:%M')" || echo "(无变更可提交)"
  echo ">>> 已提交（push 请手动执行: git -C \"$REPO\" push)"
fi
