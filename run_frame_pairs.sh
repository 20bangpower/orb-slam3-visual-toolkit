#!/usr/bin/env bash
# --- CRLF 自愈：本文件若是 CRLF 行尾，先转成 LF 再重新执行自己 ---
if [ -z "${_LF_FIXED:-}" ] && grep -qU "$(printf '\r')" "$0" 2>/dev/null; then
  sed 's/\r$//' "$0" > "$0.lf.tmp" || true
  if [ -s "$0.lf.tmp" ]; then mv "$0.lf.tmp" "$0" || true; else rm -f "$0.lf.tmp"; fi
  _LF_FIXED=1 exec bash "$0" "$@"
fi
# ---------------------------------------------------------------------------
# run_frame_pairs.sh -- 把数据集里【每个被干扰的帧】和原图成对导出，便于逐帧分析
#
#   bash run_frame_pairs.sh                     # 左目+右目，两个干扰组全部 92 帧
#   bash run_frame_pairs.sh --cams cam0         # 只看左目（快一半）
#   bash run_frame_pairs.sh --diff-col          # 对照表多一列"差异放大图"
#   bash run_frame_pairs.sh --groups fixed_pixel
#   bash run_frame_pairs.sh --max 10 --rows 5   # 先小批量试试
#
# 产物（默认写到 ~/ORB_SLAM3/outputs/fig4_4seasons/frame_pairs/）：
#   index.csv                          逐帧：改动像素数/占比/bbox/平均差
#   pairs/<组>/F###_<cam>.png          单帧对照图（左=clean 原图，右=干扰图，红框=改动区域）
#   pairs/<组>/F###_<cam>_modified.png 只有被改后的整图
#   frames/<组>/F###_<cam>_modified.png 被改后的整图（按组归档）
#   sheets/<组>_<cam>_sheetNN.png      一页 N 帧的对照表
#   sheets/<组>_<cam>_all.pdf          全部页 PDF
# ---------------------------------------------------------------------------
set -euo pipefail

ROOT="${ROOT:-$HOME/ORB_SLAM3}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${PY:-python3}"

DS_NAME="slam_stereo_pairs_0_99_20260929"
if [ -z "${DS:-}" ]; then
  DS=""
  for c in "$HOME/dataset/4Seasons/$DS_NAME" "$HOME/dataset/$DS_NAME" "$HOME/$DS_NAME"; do
    if [ -d "$c/clean" ]; then DS="$c"; break; fi
  done
  if [ -z "$DS" ]; then DS="$HOME/dataset/$DS_NAME"; fi
fi

OUT="${OUT:-$ROOT/outputs/fig4_4seasons/frame_pairs}"

if [ ! -d "$DS/clean" ]; then
  echo "[x] 找不到数据集: $DS"
  echo "    用 DS=/path/to/$DS_NAME bash $0 指定"
  exit 1
fi

mkdir -p "$OUT"
echo "[i] 数据集: $DS"
echo "[i] 输出到: $OUT"
"$PY" "$HERE/export_frame_pairs.py" --ds "$DS" --out "$OUT" "$@"

echo
echo "======== 完成 ========"
echo "逐帧统计 : $OUT/index.csv"
echo "单帧对照 : $OUT/pairs/<组>/F###_<cam>.png"
echo "对照表页 : $OUT/sheets/<组>_<cam>_sheetNN.png"
echo "整本 PDF : $OUT/sheets/<组>_<cam>_all.pdf"
if [ -d "$OUT/index.csv" ] 2>/dev/null; then :; fi
if [ -f "$OUT/index.csv" ]; then
  echo
  echo "[i] 按组汇总（改动像素数 中位/最小/最大）："
  "$PY" - "$OUT/index.csv" <<'PYEOF'
import csv, statistics, sys, collections
rows = list(csv.DictReader(open(sys.argv[1], encoding="utf-8")))
agg = collections.defaultdict(list)
for r in rows:
    agg[(r["group"], r["cam"])].append(int(r["changed_pixels"]))
for k in sorted(agg):
    v = agg[k]
    print("    %-12s %-5s 帧数=%-4d 中位=%-8d 最小=%-8d 最大=%d"
          % (k[0], k[1], len(v), statistics.median(v), min(v), max(v)))
PYEOF
fi