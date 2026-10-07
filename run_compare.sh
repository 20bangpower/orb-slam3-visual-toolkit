#!/usr/bin/env bash
# --- CRLF 自愈：本文件若是 CRLF 行尾，先转成 LF 再重新执行自己 ---
if [ -z "${_LF_FIXED:-}" ] && grep -qU "$(printf '\r')" "$0" 2>/dev/null; then
  sed 's/\r$//' "$0" > "$0.lf.tmp" || true
  if [ -s "$0.lf.tmp" ]; then mv "$0.lf.tmp" "$0" || true; else rm -f "$0.lf.tmp"; fi
  _LF_FIXED=1 exec bash "$0" "$@"
fi
# ---------------------------------------------------------------------------
# run_compare.sh -- 4Seasons 双目包：跑 SLAM + 画地图面板 + 拼【a / a' 对照图】
#
#   一张图 = 两个子图并排：
#      子图 (a)  = 正常输入 clean        [地图面板 | 数据帧]
#      子图 (a') = 同一批被补丁干扰后      [地图面板 | 数据帧]
#
#   bash run_compare.sh --all            # 跑 + 画面板 + 拼图（默认）
#   bash run_compare.sh --slam           # 只跑 SLAM
#   bash run_compare.sh --render         # 不跑 SLAM，只画左图面板（吃已有导出）
#   bash run_compare.sh --figures        # 只拼图
#   bash run_compare.sh --mono --all     # 退回单目（mono_tum_vis）
#   bash run_compare.sh --pangolin-shot --all   # 左图改用真实 Pangolin 窗口截图（含菜单栏）
#
# 目录约定（全部可用环境变量覆盖）：
#   ROOT = ~/ORB_SLAM3                                   SLAM 根目录
#   DS   = ~/dataset/slam_stereo_pairs_0_99_20260929     数据集根目录
#   词袋 = ROOT/Vocabulary/ORBvoc.txt
#   面板 = pangolin_panel2.py（优先脚本同目录，其次 ROOT/outputs/fig4_normal/）
#   抓图 = capture_pangolin.sh（脚本同目录或 ROOT/outputs/fig4_normal/，--pangolin-shot 才用）
#
# 产物：
#   $RUNS/<group>/                    每组导出 + 面板（pangolin_panel.png/.anchor.json）
#   $OUT/fig4_4seasons_fixed_pixel.png    (a) clean  vs  (a') fixed_pixel
#   $OUT/fig4_4seasons_world_plane.png    (a) clean  vs  (a') world_plane
#   $OUT/fig4_4seasons_both.png           (a)(a') / (b)(b') 两行叠一张
#
# 可调环境变量：GLYPH_EVERY GLYPH_DEPTH GLYPH_HALFW CAM_SCALE（蓝色三角大小/密度）
#               VIS_PACE_MS VIS_HOLD_SEC SECS INTERVAL SIZE
#               VIS_GROUPS FIGURES BIN VOC PANEL_PY CAP ANN_INSET
# ---------------------------------------------------------------------------
set -euo pipefail

ROOT="${ROOT:-$HOME/ORB_SLAM3}"
DS_NAME="slam_stereo_pairs_0_99_20260929"
if [ -z "${DS:-}" ]; then
  DS=""
  for c in "$HOME/dataset/4Seasons/$DS_NAME" "$HOME/dataset/$DS_NAME" "$HOME/$DS_NAME"; do
    if [ -d "$c/clean" ]; then DS="$c"; break; fi
  done
  if [ -z "$DS" ]; then DS="$HOME/dataset/$DS_NAME"; fi
fi
OUT="${OUT:-$ROOT/outputs/fig4_4seasons/figures}"
RUNS="${RUNS:-$ROOT/runs/4seasons}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${PY:-python3}"

# ---- 抓图脚本（--pangolin-shot 才用）----
CAP="${CAP:-}"
if [ -z "$CAP" ]; then
  for c in "$ROOT/outputs/fig4_normal/capture_pangolin.sh" \
           "$ROOT/outputs/fig4_normal/shot_fig4.sh" \
           "$ROOT/capture_pangolin.sh" "$ROOT/shot_fig4.sh" \
           "$HERE/capture_pangolin.sh"; do
    if [ -f "$c" ]; then CAP="$c"; break; fi
  done
fi

# ---- 参数解析 -------------------------------------------------------------
MONO=0
PANEL_SRC="${PANEL_SRC:-panel}"     # panel | shot
DO_SLAM=0; DO_RENDER=0; DO_FIGS=0
for a in "$@"; do
  case "$a" in
    --all)           DO_SLAM=1; DO_RENDER=1; DO_FIGS=1 ;;
    --slam)          DO_SLAM=1 ;;
    --render)        DO_RENDER=1 ;;
    --figures)       DO_FIGS=1 ;;
    --mono)          MONO=1 ;;
    --pangolin-shot) PANEL_SRC=shot ;;
    --panel)         PANEL_SRC=panel ;;
    -h|--help)       sed -n '2,32p' "$0"; exit 0 ;;
    *) echo "[x] 未知参数: $a"; exit 1 ;;
  esac
done
if [ "$((DO_SLAM + DO_RENDER + DO_FIGS))" -eq 0 ]; then
  DO_SLAM=1; DO_RENDER=1; DO_FIGS=1
fi

if [ "$MONO" = "1" ]; then
  BIN="${BIN:-$ROOT/Examples/Monocular/mono_tum_vis}"
  YAML_NAME="4seasons_orb3_mono.yaml"; ASSOC_SUF="mono"; KIND="单目"
else
  BIN="${BIN:-$ROOT/Examples/Stereo/stereo_tum_vi_vis}"
  YAML_NAME="4seasons_orb3_stereo.yaml"; ASSOC_SUF="stereo"; KIND="双目"
fi
VOC="${VOC:-$ROOT/Vocabulary/ORBvoc.txt}"
PANEL_PY="${PANEL_PY:-}"
if [ -z "$PANEL_PY" ]; then
  for c in "$ROOT/outputs/fig4_normal/pangolin_panel2.py" \
           "$ROOT/outputs/fig4_4seasons/pangolin_panel2.py" \
           "$HERE/pangolin_panel2.py"; do
    if [ -f "$c" ]; then PANEL_PY="$c"; break; fi
  done
  PANEL_PY="${PANEL_PY:-$ROOT/outputs/fig4_normal/pangolin_panel2.py}"
fi

# ---- 蓝色三角大小/密度（轨迹密就调小：GLYPH_DEPTH/HALFW 或 CAM_SCALE 调小，GLYPH_EVERY 调大）----
GLYPH_EVERY="${GLYPH_EVERY:-2}"
GLYPH_DEPTH="${GLYPH_DEPTH:-0.012}"
GLYPH_HALFW="${GLYPH_HALFW:-0.007}"
CAM_SCALE="${CAM_SCALE:-1.4}"

# ---- 抓图模式（--pangolin-shot）才需要的节奏 ----
if [ "$PANEL_SRC" = "shot" ]; then
  PACE="${VIS_PACE_MS:-200}"; HOLD="${VIS_HOLD_SEC:-30}"
  SECS="${SECS:-150}"; INTERVAL="${INTERVAL:-4}"; SIZE="${SIZE:-2400x2000x24}"
  ANN_INSET="${ANN_INSET:-0.14}"
else
  PACE="${VIS_PACE_MS:-0}"; HOLD="${VIS_HOLD_SEC:-0}"
  SECS="${SECS:-150}"; INTERVAL="${INTERVAL:-4}"; SIZE="${SIZE:-2400x2000x24}"
  ANN_INSET="${ANN_INSET:-0.035}"
fi

VIS_GROUPS="${VIS_GROUPS:-clean,fixed_pixel,world_plane}"
FIGURES="${FIGURES:-fixed_pixel,world_plane}"

# 只保留真实存在的组：对面少一组干扰数据也能正常出图
keep_groups() {   # $1 = 逗号分隔的组名 -> 回显还存在的
  local out="" g
  for g in ${1//,/ }; do
    if [ -d "$DS/$g" ] || [ -d "$RUNS/$g" ]; then out="${out:+$out,}$g"
    else echo "      [!] 数据集里没有 $g，跳过这一组" >&2; fi
  done
  printf '%s' "$out"
}
VIS_GROUPS="$(keep_groups "$VIS_GROUPS")"
FIGURES="$(keep_groups "$FIGURES")"
case ",$VIS_GROUPS," in
  *,clean,*) ;;
  *) echo "[x] 少了 clean 组（$DS/clean 或 $RUNS/clean 要有）"; exit 1 ;;
esac
GROUP_LIST="${VIS_GROUPS//,/ }"
FIG_LIST="${FIGURES//,/ }"

mkdir -p "$OUT" "$RUNS"

# ---------------------------------------------------------------- 文案/标注
cell_name() {
  case "$1" in
    clean)       echo "Normal input (clean)" ;;
    fixed_pixel) echo "Fixed-pixel overlay" ;;
    world_plane) echo "World-plane projection" ;;
    *)           echo "$1" ;;
  esac
}
ann_top() {  # 地图面板左上角蓝字
  case "$1" in
    clean) echo "${ANN_NORMAL:-Normal Scenario}" ;;
    *)     echo "${ANN_ATTACK:-Attack Scenario}" ;;
  esac
}
attack_word() {  # 子图标题里的干扰名
  case "$1" in
    fixed_pixel) echo "${W_FIXED:-Fixed-pixel overlay attack}" ;;
    world_plane) echo "${W_WORLD:-World-plane projection attack}" ;;
    clean)       echo "${ANN_NORMAL:-Normal Scenario (no attack)}" ;;
    *)           echo "$1" ;;
  esac
}
# 子图下方居中标题：$1 = 组名，$2 = 字母（a/b/c）
unit_label() {
  if [ "$1" = "clean" ]; then
    echo "($2) $(attack_word clean)"
  else
    echo "($2') $(attack_word "$1")"
  fi
}

# --------------------------------------------------------------------- 显示环境
RUNNER=()
if [ "$PANEL_SRC" = "panel" ] && [ -z "${DISPLAY:-}" ]; then
  if command -v xvfb-run >/dev/null 2>&1; then
    RUNNER=(xvfb-run -a -s "-screen 0 1600x1200x24 +extension GLX +render -noreset")
    echo "[i] 没有 DISPLAY，SLAM 用 xvfb-run 起虚拟屏（Viewer 正常）"
  else
    echo "[!] 没有 DISPLAY 也没 xvfb-run —— Viewer 会崩。"
    echo "    sudo apt-get install -y xvfb   或  DISPLAY=:0 bash $0 ..."
  fi
fi

check_deps_shot() {
  local miss="" t
  for t in Xvfb xwininfo import; do
    command -v "$t" >/dev/null 2>&1 || miss="$miss $t"
  done
  if [ -n "$miss" ]; then
    echo "[x] 抓真实 Pangolin 窗口截图缺这些工具:$miss"
    echo "    sudo apt-get install -y xvfb x11-utils imagemagick xdotool"
    return 1
  fi
  return 0
}

# --------------------------------------------------------------------- SLAM
run_slam() {
  local g="$1"
  local rd="$RUNS/$g"
  local yaml="$DS/orb_inputs/$YAML_NAME"
  local assoc="$DS/orb_inputs/${g}_${ASSOC_SUF}.txt"

  if [ ! -x "$BIN" ]; then
    echo "[x] 没有 $BIN"
    if [ "$MONO" = "1" ]; then echo "    先跑 bash $HERE/patch_mono.sh"
    else                          echo "    先跑 bash $HERE/patch_stereo.sh"; fi
    exit 1
  fi
  [ -f "$VOC" ] || { echo "[x] 没有词袋 $VOC"; exit 1; }
  [ -d "$DS/$g" ] || { echo "[x] 没有数据集目录 $DS/$g —— 先跑 install_dataset.sh"; exit 1; }

  local need_gen=0
  [ -f "$yaml" ]  || need_gen=1
  [ -f "$assoc" ] || need_gen=1
  if [ "$MONO" = "0" ] && [ -f "$yaml" ] && ! grep -q '^ThDepth' "$yaml"; then
    echo "[!] $YAML_NAME 缺 ThDepth（ORB-SLAM3 双目必需），重新生成"
    need_gen=1
  fi
  if [ "$need_gen" = "1" ]; then
    echo "[i] 生成 ORB-SLAM3 输入（关联文件 + yaml）-> $DS/orb_inputs"
    "$PY" "$HERE/make_orb_inputs.py" --ds-root "$DS"
  fi
  [ -f "$assoc" ] || { echo "[x] 还是缺关联文件 $assoc"; exit 1; }

  mkdir -p "$rd"
  echo "──────── [$g] 跑$KIND SLAM ────────"
  echo "      BIN  = $BIN"
  echo "      词组 = $yaml"

  if [ "$PANEL_SRC" = "shot" ]; then
    [ -f "$CAP" ] || { echo "[x] 找不到抓图脚本（capture_pangolin.sh / shot_fig4.sh）"; exit 1; }
    check_deps_shot || exit 1
    echo "      抓图 = $CAP (secs=$SECS interval=$INTERVAL size=$SIZE)"
    echo "      节奏 = VIS_PACE_MS=$PACE VIS_HOLD_SEC=$HOLD"
    local cmd
    cmd="cd '$rd' && VIS_OUT=. VIS_PACE_MS=$PACE VIS_HOLD_SEC=$HOLD '$BIN' '$VOC' '$yaml' '$DS/$g' '$assoc'"
    ( cd "$rd" && bash "$CAP" --cmd "$cmd" \
        --secs "$SECS" --interval "$INTERVAL" --out "$rd/shots" --size "$SIZE" ) \
        2>&1 | tee "$rd/capture.log" || true
    local nshots=0
    [ -d "$rd/shots" ] && nshots="$(ls -1 "$rd/shots" 2>/dev/null | grep -c '^w_.*\.png$' || true)"
    echo "      抓到窗口截图 $nshots 张 -> $rd/shots/"
  else
    ( cd "$rd" && ${RUNNER[@]+"${RUNNER[@]}"} \
        env VIS_OUT=. VIS_PACE_MS="$PACE" VIS_HOLD_SEC="$HOLD" \
        "$BIN" "$VOC" "$yaml" "$DS/$g" "$assoc" 2>&1 | tee "$rd/run.log" ) || true
  fi

  if [ -f "$rd/CameraTrajectory.txt" ]; then
    local n; n=$(wc -l < "$rd/CameraTrajectory.txt")
    echo "      CameraTrajectory.txt $n 行（输入 100 帧）"
    if [ "$n" -lt 60 ]; then echo "      [!] 只跟踪到 $n 帧，可能中途丢了；看 $rd/run.log"; fi
  elif [ -f "$rd/KeyFrameTrajectory.txt" ]; then
    echo "      KeyFrameTrajectory.txt $(wc -l < "$rd/KeyFrameTrajectory.txt") 行（单目没有逐帧轨迹）"
  else
    echo "      [!] 没生成任何轨迹文件，SLAM 可能没跑起来；看 $rd/run.log"
  fi
}

# ------------------------------------------------ 左图 A：pangolin_panel2.py 面板
render_panel() {
  local g="$1"
  local rd="$RUNS/$g"
  local traj="$rd/KeyFrameTrajectory.txt"

  echo "──────── [$g] 画地图面板（pangolin_panel2.py）────────"
  [ -f "$traj" ] || { echo "      [x] 缺 $traj（这组先跑 --slam）"; return 1; }
  if [ ! -f "$PANEL_PY" ]; then
    echo "      [x] 找不到 $PANEL_PY"
    echo "          用 PANEL_PY=/path/to/pangolin_panel2.py 指定"
    return 1
  fi
  [ -f "$rd/map_points.csv" ] || echo "      [!] 没有 map_points.csv，黑点会是空的"

  local ge="$GLYPH_EVERY" gd="$GLYPH_DEPTH" gh="$GLYPH_HALFW" cs="$CAM_SCALE"
  local log="$rd/pangolin_panel.log"
  local i=0
  while :; do
    i=$((i + 1))
    local args=(--traj "$traj" --menu none --ref-mode frustum --ref-fov 63
                --fit all --width 1200 --height 900
                --glyph-every "$ge" --glyph-depth "$gd" --glyph-halfw "$gh" --cam-scale "$cs"
                --out "$rd/pangolin_panel.png"
                --anchor-out "$rd/pangolin_panel.anchor.json")
    [ -f "$rd/map_points.csv" ] && args+=(--points "$rd/map_points.csv")
    [ -f "$rd/CameraTrajectory.txt" ] && args+=(--cam "$rd/CameraTrajectory.txt")

    if [ "$i" -eq 1 ]; then
      "$PY" "$PANEL_PY" "${args[@]}" > "$log" 2>&1 || { cat "$log"; return 1; }
    else
      "$PY" "$PANEL_PY" "${args[@]}" >> "$log" 2>&1 || { cat "$log"; return 1; }
    fi
    tail -3 "$log" | sed 's/^/      /'

    # pangolin_panel2.py 会在"关键帧太密"时提示 --glyph-every N；这里自动采纳再画一遍
    local sug
    sug="$(sed -n 's/.*建议加 --glyph-every \([0-9][0-9]*\).*/\1/p' "$log" | tail -1)"
    if [ "$i" -eq 1 ] && [ -n "$sug" ] && [ "$sug" -gt "$ge" ]; then
      echo "      [!] 关键帧太密 -> 自动重画：glyph-every $ge -> $sug，三角缩到 0.75x"
      ge="$sug"
      gd="$(awk -v v="$gd" 'BEGIN{printf "%.4f", v*0.75}')"
      gh="$(awk -v v="$gh" 'BEGIN{printf "%.4f", v*0.75}')"
      cs="$(awk -v v="$cs" 'BEGIN{printf "%.2f", (v*0.8 < 1.0 ? 1.0 : v*0.8)}')"
      continue
    fi
    break
  done

  cp -f "$rd/pangolin_panel.png" "$rd/pangolin_map.png"
  echo "      [i] 三角参数: glyph-every=$ge depth=$gd halfw=$gh cam-scale=$cs"
  echo "      [i] 面板 -> $rd/pangolin_panel.png  锚点 -> $rd/pangolin_panel.anchor.json"
}
# ---------------------------------------------------- 左图 B：真实 Pangolin 截图
pick_panel_shot() {
  local g="$1"
  local rd="$RUNS/$g"
  local shots="$rd/shots"

  echo "──────── [$g] 挑左图：真实 Pangolin 地图窗口截图 ────────"
  if [ ! -d "$shots" ] && [ ! -f "$rd/map_view.png" ]; then
    echo "      [x] 既没有 $shots 也没有 $rd/map_view.png（这组先跑 --slam）"
    return 1
  fi
  "$PY" "$HERE/pick_map_shot.py" --shots "$shots" \
      --mapview "$rd/map_view.png" --out "$rd/pangolin_map.png"
}

# ------------------------------------------------------------------- 拼图
panel_of() {  # $1 = group
  local rd="$RUNS/$1"
  if [ -f "$rd/pangolin_map.png" ]; then echo "$rd/pangolin_map.png"
  elif [ -f "$rd/pangolin_panel.png" ]; then echo "$rd/pangolin_panel.png"
  else echo "$rd/pangolin_map.png"; fi
}
anchor_of() {  # $1 = group（只有面板模式有 anchor.json）
  local f="$RUNS/$1/pangolin_panel.anchor.json"
  if [ -f "$f" ]; then echo "$f"; else echo ""; fi
}

row_spec() {   # $1 = attack group
  printf '%s,%s,%s|%s,%s,%s' \
    "$(cell_name clean)" "$(panel_of clean)" "$RUNS/clean/current_frame.png" \
    "$(cell_name "$1")" "$(panel_of "$1")" "$RUNS/$1/current_frame.png"
}

make_figures() {
  if [ -z "${FIG_LIST// /}" ]; then
    echo "[!] FIGURES 是空的，跳过拼图"; return 0
  fi

  local letters=(a b c d)
  local idx=0
  local rows=() labs=() adata=()
  for g in $FIG_LIST; do
    local L="${letters[$idx]}"
    rows+=(--row "$(row_spec "$g")")
    labs+=(--cell-label "$(unit_label clean "$L")" --cell-label "$(unit_label "$g" "$L")")
    adata+=(--cell-ann "$(ann_top clean)" --cell-ann "$(ann_top "$g")")
    adata+=(--cell-anchor-file "$(anchor_of clean)" --cell-anchor-file "$(anchor_of "$g")")
    idx=$((idx + 1))
  done

  # 单组：一张图 = (a) clean | (a') 干扰
  idx=0
  for g in $FIG_LIST; do
    local L="${letters[$idx]}"
    echo "──────── 拼图: ($L) clean  |  ($L') $g ────────"
    "$PY" "$HERE/compose_compare.py" \
      --row "$(row_spec "$g")" \
      --cell-label "$(unit_label clean "$L")" --cell-label "$(unit_label "$g" "$L")" \
      --cell-ann "$(ann_top clean)" --cell-ann "$(ann_top "$g")" \
      --cell-anchor-file "$(anchor_of clean)" --cell-anchor-file "$(anchor_of "$g")" \
      --ann-inset "$ANN_INSET" --no-cell-title --height 2.4 --fontsize 11 \
      --out "$OUT/fig4_4seasons_$g"
    idx=$((idx + 1))
  done

  # 多组：所有行叠一张（(a)(a') / (b)(b') ...）
  if [ "$idx" -ge 2 ]; then
    echo "──────── 拼图: 多行合并 ────────"
    "$PY" "$HERE/compose_compare.py" "${rows[@]}" "${labs[@]}" "${adata[@]}" \
      --ann-inset "$ANN_INSET" --no-cell-title --height 2.4 --fontsize 11 \
      --out "$OUT/fig4_4seasons_both"
  fi
}
# ------------------------------------------------------------------- 主流程
if [ "$DO_SLAM" = "1" ]; then
  for g in $GROUP_LIST; do run_slam "$g"; done
fi
if [ "$DO_RENDER" = "1" ]; then
  for g in $GROUP_LIST; do
    if [ "$PANEL_SRC" = "shot" ]; then
      pick_panel_shot "$g" || render_panel "$g" || true
    else
      render_panel "$g" || true
    fi
  done
fi
if [ "$DO_FIGS" = "1" ]; then
  make_figures
fi

echo
echo "======== 完成 ========"
echo "导出目录: $RUNS/{$(echo "$GROUP_LIST" | tr ' ' ',')}/"
echo "左图: $RUNS/<group>/pangolin_map.png（由 pangolin_panel2.py 画的全局地图面板）"
echo "成图目录: $OUT/"
if [ -d "$OUT" ]; then ls -1 "$OUT" | sed 's/^/  /'; fi