#!/usr/bin/env bash
# --- CRLF 自愈：本文件若是 CRLF 行尾，先转成 LF 再重新执行自己 ---
if [ -z "${_LF_FIXED:-}" ] && grep -qU "$(printf '\r')" "$0" 2>/dev/null; then
  sed 's/\r$//' "$0" > "$0.lf.tmp" || true
  if [ -s "$0.lf.tmp" ]; then mv "$0.lf.tmp" "$0" || true; else rm -f "$0.lf.tmp"; fi
  _LF_FIXED=1 exec bash "$0" "$@"
fi
# ---------------------------------------------------------------------------
# start_gui.sh -- 起「ORB-SLAM3 可视化对照工具链」的简易网页界面
#
#   bash start_gui.sh                     # 起服务并自动开浏览器 (127.0.0.1:8770)
#   bash start_gui.sh --no-browser        # 服务器 / ssh 场景：自己开浏览器访问
#   bash start_gui.sh --port 9000
#   bash start_gui.sh --ds ~/dataset/slam_stereo_pairs_0_99_20260929
#   bash start_gui.sh --root ~/ORB_SLAM3
#
# 没有图形界面时（ssh）：本机做端口转发
#   ssh -L 8770:127.0.0.1:8770 用户名@服务器地址
# 然后在本地浏览器打开 http://127.0.0.1:8770
# ---------------------------------------------------------------------------
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${PY:-python3}"

if ! command -v "$PY" >/dev/null 2>&1; then
  echo "[x] 找不到 $PY（可以 PY=/usr/bin/python3 bash start_gui.sh 指定）"
  exit 1
fi
if [ ! -f "$HERE/gui_app.py" ]; then
  echo "[x] 缺 $HERE/gui_app.py"
  exit 1
fi

exec "$PY" "$HERE/gui_app.py" "$@"
