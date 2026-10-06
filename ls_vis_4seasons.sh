#!/usr/bin/env bash
# --- CRLF 自愈：本文件若是 CRLF 行尾，先转成 LF 再重新执行自己 ---
if [ -z "${_LF_FIXED:-}" ] && grep -qU "$(printf '\r')" "$0" 2>/dev/null; then
  sed 's/\r$//' "$0" > "$0.lf.tmp" || true
  if [ -s "$0.lf.tmp" ]; then mv "$0.lf.tmp" "$0" || true; else rm -f "$0.lf.tmp"; fi
  _LF_FIXED=1 exec bash "$0" "$@"
fi
# ---------------------------------------------------------------------------
# ls_vis_4seasons.sh -- 4Seasons 这套流程的自检：逐条列路径 + 标注 [ok]/[缺]
#
#   bash ls_vis_4seasons.sh
#   ROOT=~/ORB_SLAM3 DS=~/dataset/slam_stereo_pairs_0_99_20260929 bash ls_vis_4seasons.sh
# ---------------------------------------------------------------------------
set -u
ROOT="${ROOT:-$HOME/ORB_SLAM3}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DS_NAME="slam_stereo_pairs_0_99_20260929"
if [ -z "${DS:-}" ]; then
  DS=""
  for c in "$HOME/dataset/4Seasons/$DS_NAME" "$HOME/dataset/$DS_NAME" "$HOME/$DS_NAME"; do
    if [ -d "$c/clean" ]; then DS="$c"; break; fi
  done
  if [ -z "$DS" ]; then DS="$HOME/dataset/$DS_NAME"; fi
fi
RUNS="${RUNS:-$ROOT/runs/4seasons}"
OUT="${OUT:-$ROOT/outputs/fig4_4seasons/figures}"

group() { echo; echo "════ $1"; }
one() {
  if [ -e "$1" ]; then
    printf '  [ok]  %-70s %s\n' "$1" "$(du -h "$1" 2>/dev/null | cut -f1)"
  else
    printf '  [缺]  %s\n' "$1"
  fi
}

echo "HOME = $HOME"
echo "ROOT = $ROOT"
echo "DS   = $DS"

group "1. 本工具链（$HERE）"
for f in install_dataset.sh make_orb_inputs.py make_orb_inputs.sh \
         vis_export.h mono_tum_vis.cc stereo_tum_vi_vis.cc \
         patch_mono.sh patch_stereo.sh patch_cmake.py build_mono_vis.py \
         run_compare.sh compose_compare.py pick_map_shot.py capture_pangolin.sh \
         dump_api.sh export_frame_pairs.py run_frame_pairs.sh \
         make_fig4_video.py run_video.sh \
         plot_deviation.py run_deviation.sh \
         README_4seasons.md; do one "$HERE/$f"; done

group "2. 数据集"
one "$DS"
one "$DS/README.md"
one "$DS/SHA256SUMS.csv"
for g in clean fixed_pixel world_plane; do
  one "$DS/$g/pairs.csv"
  one "$DS/$g/cam0"
  one "$DS/$g/cam1"
done
one "$DS/estimated_trajectories"
one "$DS/reference_figures"

group "3. ORB-SLAM3 输入（make_orb_inputs.py 之后）"
one "$DS/orb_inputs"
for g in clean fixed_pixel world_plane; do
  one "$DS/orb_inputs/${g}_mono.txt"
  one "$DS/orb_inputs/${g}_stereo.txt"
done
one "$DS/orb_inputs/4seasons_orb3_mono.yaml"
one "$DS/orb_inputs/4seasons_orb3_stereo.yaml"

group "4. 双目入口（patch_stereo.sh 之后）"
one "$ROOT/Examples/Stereo/stereo_tum_vi_vis.cc"
one "$ROOT/Examples/Stereo/vis_export.h"
one "$ROOT/Examples/Stereo/stereo_tum_vi_vis"
one "$ROOT/Examples/Monocular/mono_tum_vis"
one "$ROOT/Vocabulary/ORBvoc.txt"

group "5. 抓图工具（左图 = 真实 Pangolin 窗口截图）"
one "$ROOT/outputs/fig4_normal/capture_pangolin.sh"
one "$ROOT/outputs/fig4_normal/shot_fig4.sh"
one "$ROOT/outputs/fig4_normal/pangolin_panel2.py"
for t in Xvfb xwininfo import xdotool; do
  if command -v "$t" >/dev/null 2>&1; then
    printf '  [ok]  %-70s %s\n' "(命令) $t" "$(command -v "$t")"
  else
    printf '  [缺]  (命令) %s   -> sudo apt-get install -y xvfb x11-utils imagemagick xdotool\n' "$t"
  fi
done

group "6. 每组的导出（run_compare.sh --slam 之后）"
for g in clean fixed_pixel world_plane; do
  echo "  -- $g"
  for f in current_frame.png current_features.csv CameraTrajectory.txt \
           KeyFrameTrajectory.txt map_points.csv keyframes.csv map_view.png \
           capture.log slam_stdout.log; do
    one "$RUNS/$g/$f"
  done
  one "$RUNS/$g/pangolin_map.png"
  one "$RUNS/$g/pangolin_panel.png"
  one "$RUNS/$g/pangolin_panel.anchor.json"
  one "$RUNS/$g/shots"
done

group "7. 对照成图"
for f in fig4_4seasons_fixed_pixel.png fig4_4seasons_world_plane.png \
         fig4_4seasons_both.png fig4_4seasons_deviation.png; do one "$OUT/$f"; done

group "8. 逐帧对照导出（run_frame_pairs.sh 之后）"
FP="${FP:-$ROOT/outputs/fig4_4seasons/frame_pairs}"
one "$FP/index.csv"
for g in fixed_pixel world_plane; do
  for c in cam0 cam1; do
    one "$FP/pairs/$g/$c"
    one "$FP/sheets/${g}_${c}_all.pdf"
  done
done

group "9. 逐帧对照视频（run_video.sh 之后）"
VID="${VID:-$ROOT/outputs/fig4_4seasons/video}"
one "$VID/index_video.csv"
one "$VID/all.mp4"
for g in fixed_pixel world_plane; do
  one "$VID/$g.mp4"
  one "$VID/$g.gif"
  one "$VID/slides/$g"
done
if command -v ffmpeg >/dev/null 2>&1; then
  printf '  [ok]  %-70s %s\n' "(命令) ffmpeg" "$(command -v ffmpeg)"
else
  printf '  [缺]  (命令) ffmpeg  -> sudo apt-get install -y ffmpeg（没有就只出 GIF）\n'
fi

group "10. fig4 逐帧快照（run_video.sh --fig4 --slam 之后）"
for g in clean fixed_pixel world_plane; do one "$RUNS/$g/frames"; done

echo
echo "[i] 缺哪条就回对应步骤：install_dataset.sh -> make_orb_inputs.sh -> patch_stereo.sh -> run_compare.sh --all"