#!/usr/bin/env bash
# --- CRLF 自愈：本文件若是 CRLF 行尾，先转成 LF 再重新执行自己 ---
if [ -z "${_LF_FIXED:-}" ] && grep -qU "$(printf '\r')" "$0" 2>/dev/null; then
  sed 's/\r$//' "$0" > "$0.lf.tmp" || true
  if [ -s "$0.lf.tmp" ]; then mv "$0.lf.tmp" "$0" || true; else rm -f "$0.lf.tmp"; fi
  _LF_FIXED=1 exec bash "$0" "$@"
fi
# 薄封装：把 4Seasons 双目包转成 ORB-SLAM3 输入
#   bash make_orb_inputs.sh
#   DS_ROOT=/path/to/slam_stereo_pairs_0_99_20260929 bash make_orb_inputs.sh
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${PY:-python3}"
NAME="slam_stereo_pairs_0_99_20260929"

pick_root() {
  local c
  if [ -n "${DS_ROOT:-}" ]; then echo "$DS_ROOT"; return; fi
  for c in "$HOME/dataset/4Seasons/$NAME" "$HOME/dataset/$NAME" "$HOME/$NAME"; do
    if [ -d "$c/clean" ]; then echo "$c"; return; fi
  done
  echo "$HOME/dataset/$NAME"
}

DS_ROOT="$(pick_root)"
echo "[i] 数据集根目录: $DS_ROOT"
exec "$PY" "$HERE/make_orb_inputs.py" --ds-root "$DS_ROOT" "$@"