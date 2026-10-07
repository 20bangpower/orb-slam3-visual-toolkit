#!/usr/bin/env bash
# --- CRLF 自愈：本文件若是 CRLF 行尾，先转成 LF 再重新执行自己 ---
if [ -z "${_LF_FIXED:-}" ] && grep -qU "$(printf '\r')" "$0" 2>/dev/null; then
  sed 's/\r$//' "$0" > "$0.lf.tmp" || true
  if [ -s "$0.lf.tmp" ]; then mv "$0.lf.tmp" "$0" || true; else rm -f "$0.lf.tmp"; fi
  _LF_FIXED=1 exec bash "$0" "$@"
fi
# ---------------------------------------------------------------------------
# run_deviation.sh -- 出"正常 vs 攻击：位移偏差 + 建图对比"辅助分析图
#
#   bash run_deviation.sh                 # 用已有 SLAM 产物（run_compare.sh --slam 之后）
#   bash run_deviation.sh --groups fixed_pixel
#
# 数据来源：$RUNS/<组>/CameraTrajectory.txt、$RUNS/<组>/map_points.csv
# 产物：$OUT/fig4_4seasons_deviation.png / .pdf
#   (a) 位移偏差折线（黑=clean 基线）  (b) 近场建图点云 X-Z（蓝=clean，红=攻击）
#   (b) 主图显示全部点，空白角子图放大密集区（VIS_ZOOM_K 调缩放，0=不加子图）
# ---------------------------------------------------------------------------
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${ROOT:-$HOME/ORB_SLAM3}"
PY="${PY:-python3}"
RUNS="${RUNS:-$ROOT/runs/4seasons}"
OUT="${OUT:-$ROOT/outputs/fig4_4seasons/figures/fig4_4seasons_deviation}"
ZOOM_K="${VIS_ZOOM_K:-2.6}"   # 子图放大视窗半宽 = k x 中位半径（0=不加子图）

DS_NAME="slam_stereo_pairs_0_99_20260929"
if [ -z "${DS:-}" ]; then
  DS=""
  for c in "$HOME/dataset/4Seasons/$DS_NAME" "$HOME/dataset/$DS_NAME" "$HOME/$DS_NAME"; do
    if [ -d "$c/clean" ]; then DS="$c"; break; fi
  done
  if [ -z "$DS" ]; then DS="$HOME/dataset/$DS_NAME"; fi
fi

if [ ! -f "$RUNS/clean/CameraTrajectory.txt" ] && [ ! -f "$RUNS/clean/KeyFrameTrajectory.txt" ]; then
  echo "[x] 缺 $RUNS/clean/CameraTrajectory.txt（单目是 KeyFrameTrajectory.txt）"
  echo "    先跑 bash run_compare.sh --slam"
  exit 1
fi

echo "[i] 数据集: $DS"
echo "[i] 运行目录: $RUNS"
if [ ! -f "$RUNS/clean/map_points.csv" ]; then
  echo "[!] 缺 $RUNS/clean/map_points.csv —— 面板 (b) 建图点云会少掉 clean 的点。"
  echo "    先跑：  bash run_compare.sh --all"
fi
mkdir -p "$(dirname "$OUT")"
"$PY" "$HERE/plot_deviation.py" --ds "$DS" --runs "$RUNS" --out "$OUT" \
  --zoom-k "$ZOOM_K" "$@"

echo
echo "======== 完成 ========"
echo "成图: $OUT.png / $OUT.pdf"