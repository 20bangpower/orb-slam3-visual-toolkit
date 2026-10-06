#!/usr/bin/env bash
# --- CRLF 自愈：本文件若是 CRLF 行尾，先转成 LF 再重新执行自己 ---
if [ -z "${_LF_FIXED:-}" ] && grep -qU "$(printf '\r')" "$0" 2>/dev/null; then
  sed 's/\r$//' "$0" > "$0.lf.tmp" || true
  if [ -s "$0.lf.tmp" ]; then mv "$0.lf.tmp" "$0" || true; else rm -f "$0.lf.tmp"; fi
  _LF_FIXED=1 exec bash "$0" "$@"
fi
# ---------------------------------------------------------------------------
# run_video.sh -- 逐帧 "(a)正常 vs (a')干扰" 对照 -> 视频（每帧停留 N 秒），便于分析
#
#   bash run_video.sh                     # 默认 = --fig4：clean 的 Fig.4 版式 | 干扰的 Fig.4 版式
#                                         #   （没快照会自动先跑 SLAM，不需要 X）
#   bash run_video.sh --slam              # 强制重跑一遍逐帧快照再出视频
#   bash run_video.sh --pair              # 备用：只用数据集图片做对照（不跑 SLAM）
#   bash run_video.sh --triple            # 一帧里放 3 个板块：
#                                         #    normal + fixed_pixel + world_plane（单个视频 triple.mp4）
#
# 其余参数原样透传给 make_fig4_video.py：
#   --groups fixed_pixel,world_plane   --cams cam0   --max 10   --hold 1.5
#   --all-frames   --diff-col   --width 1920   --height 0   --jobs 4   --gif
#
# 产物（默认 ~/ORB_SLAM3/outputs/fig4_4seasons/video/）：
#   <组>.mp4 / <组>.gif          每个干扰组一个视频
#   triple.mp4 / triple.gif    --triple 时：一帧 3 板块的单个视频
#   slides/triple/F###.png     --triple 时的单帧画面
#   slides/<组>/F###.png         单帧画面（可直接分发）
#   index_video.csv              帧号 / 时间戳 / 停留秒数
#
# fig4 模式的逐帧快照 (~/ORB_SLAM3/runs/4seasons/<组>/frames/F###_*) 由
#   VIS_FRAMES_DIR + VIS_NO_VIEWER=1 生成（无 X 也能跑）。
# ---------------------------------------------------------------------------
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${ROOT:-$HOME/ORB_SLAM3}"
PY="${PY:-python3}"
OUT="${OUT:-$ROOT/outputs/fig4_4seasons/video}"
RUNS="${RUNS:-$ROOT/runs/4seasons}"

DS_NAME="slam_stereo_pairs_0_99_20260929"
if [ -z "${DS:-}" ]; then
  DS=""
  for c in "$HOME/dataset/4Seasons/$DS_NAME" "$HOME/dataset/$DS_NAME" "$HOME/$DS_NAME"; do
    if [ -d "$c/clean" ]; then DS="$c"; break; fi
  done
  if [ -z "$DS" ]; then DS="$HOME/dataset/$DS_NAME"; fi
fi

MODE="${MODE:-fig4}"
DO_SLAM=0
WANT_GROUPS=""
PASS=()
SKIP=0
for a in "$@"; do
  if [ "$SKIP" = "1" ]; then SKIP=0; continue; fi
  case "$a" in
    --pair)     MODE=pair ;;
    --fig4)     MODE=fig4 ;;
    --slam)     DO_SLAM=1 ;;
    --groups)   SKIP=1 ;;                       # 值在下一轮被跳过
    --groups=*) WANT_GROUPS="${a#--groups=}" ;;
    --mode=*)   MODE="${a#--mode=}" ;;
    *)          PASS+=("$a") ;;
  esac
done
# 再扫一遍，抓空格写法的 --groups 值
prev=""
for a in "$@"; do
  if [ "$prev" = "--groups" ]; then WANT_GROUPS="$a"; fi
  prev="$a"
done
VID_GROUPS="${WANT_GROUPS:-${VID_GROUPS:-fixed_pixel,world_plane}}"

echo "======================================================"
echo " 逐帧对照视频"
echo "   模式   : $MODE"
echo "   ROOT   : $ROOT"
echo "   数据集 : $DS"
echo "   分组   : $VID_GROUPS"
echo "   输出   : $OUT"
echo "======================================================"

if [ ! -d "$DS/clean" ]; then
  echo "[x] 找不到数据集: $DS/clean"
  echo "    用 DS=/path/to/$DS_NAME bash $0 ... 指定"
  exit 1
fi

# ---------------------------------------------------------------- fig4 快照
need_slam() {
  if [ "$DO_SLAM" = "1" ]; then return 0; fi
  if [ ! -d "$RUNS/clean/frames" ]; then return 0; fi
  local g
  for g in ${VID_GROUPS//,/ }; do
    if [ ! -d "$RUNS/$g/frames" ]; then return 0; fi
  done
  return 1
}

run_slam_frames() {
  local g="$1"
  local yaml="$DS/orb_inputs/4seasons_orb3_stereo.yaml"
  local assoc="$DS/orb_inputs/${g}_stereo.txt"
  local bin="$ROOT/Examples/Stereo/stereo_tum_vi_vis"
  local rd="$RUNS/$g"

  [ -x "$bin" ] || { echo "[x] 没有 $bin —— 先跑 bash $HERE/patch_stereo.sh"; exit 1; }
  [ -f "$yaml" ] || { echo "[x] 没有 $yaml —— 先跑 bash $HERE/make_orb_inputs.sh"; exit 1; }
  [ -f "$assoc" ] || { echo "[x] 没有 $assoc —— 先跑 bash $HERE/make_orb_inputs.sh"; exit 1; }

  mkdir -p "$rd/frames"
  echo "──────── [$g] 跑 SLAM 逐帧快照 -> $rd/frames/ ────────"
  ( cd "$rd" && env VIS_OUT="$rd" VIS_NO_VIEWER=1 VIS_FRAMES_DIR="$rd/frames" \
      VIS_PACE_MS=0 "$bin" "$ROOT/Vocabulary/ORBvoc.txt" "$yaml" "$DS/$g" "$assoc" ) \
      > "$rd/frames_run.log" 2>&1 || true
  local n
  n="$(ls -1 "$rd/frames" 2>/dev/null | grep -c '_current_frame\.png$' || true)"
  echo "      [i] 快照 $n 帧（日志 $rd/frames_run.log）"
  if [ "$n" -eq 0 ]; then
    echo "      [!] 一帧都没导出。最常见原因：可执行文件还是旧的（不认识 VIS_FRAMES_DIR）。"
    echo "          先重编：  bash $HERE/patch_stereo.sh"
    echo "          日志尾部："
    tail -20 "$rd/frames_run.log" 2>/dev/null | sed 's/^/          /'
    return 1
  fi
  return 0
}

if [ "$MODE" = "fig4" ]; then
  if need_slam; then
    echo "[i] 需要逐帧快照 -> 先跑 SLAM（VIS_NO_VIEWER=1，不需要 X）"
    if ! run_slam_frames clean; then
      echo "[x] clean 组的逐帧快照没出来，先解决上面那个问题再跑。"
      exit 1
    fi
    for g in ${VID_GROUPS//,/ }; do
      if ! run_slam_frames "$g"; then
        echo "[x] $g 组的逐帧快照没出来，先解决上面那个问题再跑。"
        exit 1
      fi
    done
  else
    echo "[i] 已有逐帧快照，直接出视频（想重跑加 --slam）"
  fi
fi

# ------------------------------------------------------------------- 出视频
mkdir -p "$OUT"
CMD=("$PY" "$HERE/make_fig4_video.py" --mode "$MODE" --ds "$DS" --out "$OUT" --groups "$VID_GROUPS")
if [ "$MODE" = "fig4" ]; then
  CMD+=(--runs "$RUNS")
fi
CMD+=("${PASS[@]+"${PASS[@]}"}")

echo
echo "[i] $PY make_fig4_video.py --mode $MODE ..."
"${CMD[@]}"

echo
echo "======== 视频好了 ========"
echo "目录: $OUT"
ls -1sh "$OUT" 2>/dev/null | sed 's/^/  /'