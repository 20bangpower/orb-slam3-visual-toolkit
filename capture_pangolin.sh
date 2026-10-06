#!/usr/bin/env bash
# --- CRLF 自愈：本文件若是 CRLF 行尾，先转成 LF 再重新执行自己 ---
if [ -z "${_LF_FIXED:-}" ] && grep -qU "$(printf '\r')" "$0" 2>/dev/null; then
  sed 's/\r$//' "$0" > "$0.lf.tmp" || true
  if [ -s "$0.lf.tmp" ]; then mv "$0.lf.tmp" "$0" || true; else rm -f "$0.lf.tmp"; fi
  _LF_FIXED=1 exec bash "$0" "$@"
fi
# ---------------------------------------------------------------------------
# shot_fig4.sh  --  抓 ORB-SLAM3 viewer 的真实窗口截图（复刻论文 Fig.4 左边的全局地图/轨迹图）
#
# 为什么这次"位置"不会再错：
#   脚本不假设窗口在屏幕的哪个角落，而是用 xwininfo 找出 ORB-SLAM3 的每个窗口的
#   *窗口 id*，再用 import -window <id> 直接抓那个窗口本身；只有抓不到 id 时才退回
#   整屏抓图 + 按该窗口的真实几何(x,y,w,h)去裁。所以窗口挪到哪儿都对。
#
# 用法 A（离屏跑，推荐）：
#   cd ~/ORB_SLAM3
#   bash shot_fig4.sh --secs 180 --cmd "./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml ~/dataset/TUM/rgbd_dataset_freiburg1_xyz Examples/RGB-D/associations/fr1_xyz.txt"
#
# 用法 B（桌面上 viewer 已经开着，抓现成的）：
#   DISPLAY=:0 bash shot_fig4.sh --attach --secs 30
#
# 用法 C（先确认脚本认不认得出窗口）：
#   DISPLAY=:0 bash shot_fig4.sh --list
#
# 产出（目录用 --out 改，默认 ./fig4shots）
#   root_NNN.png            整屏保底图
#   w_NNN_<窗口名>.png      按窗口抓的图；ORB-SLAM3: Map Viewer 那块 = Fig.4 左图
#   windows_NNN.txt         每步的 ORB-SLAM3 窗口 id / 名字 / 几何
#   index.txt               汇总
# ---------------------------------------------------------------------------
set -u

SECS=180; INTERVAL=4; OUT="fig4shots"; SIZE="2400x2000x24"
MODE=""; CMD=""; DISP=""

while [ $# -gt 0 ]; do
  case "$1" in
    --cmd)      MODE=run;    CMD="${2:-}"; shift 2 ;;
    --attach)   MODE=attach; shift ;;
    --list)     MODE=list;   shift ;;
    --secs)     SECS="$2";   shift 2 ;;
    --interval) INTERVAL="$2"; shift 2 ;;
    --out)      OUT="$2";    shift 2 ;;
    --size)     SIZE="$2";   shift 2 ;;
    --display)  DISP="$2";   shift 2 ;;
    -h|--help)  sed -n '2,26p' "$0"; exit 0 ;;
    *) echo "[x] 未知参数: $1"; exit 2 ;;
  esac
done
[ -n "$MODE" ] || { echo "[x] 用 --cmd \"<SLAM 命令>\" / --attach / --list，-h 看说明"; exit 2; }

PATH="$PATH:/usr/bin:/bin"
command -v xwininfo >/dev/null 2>&1 || { echo "[x] 缺 xwininfo:  sudo apt-get install -y x11-utils"; exit 1; }
GRAB=""
for c in import ffmpeg scrot; do command -v "$c" >/dev/null 2>&1 && { GRAB="$c"; break; }; done
[ -n "$GRAB" ] || { echo "[x] 缺抓屏工具:  sudo apt-get install -y imagemagick"; exit 1; }
HAVE_PIL=0; python3 -c 'import PIL' >/dev/null 2>&1 && HAVE_PIL=1

# ------------------------------- 工具函数 ----------------------------------
real_size() {   # $1=display -> WxHx24
  local s
  s="$(DISPLAY="$1" xdpyinfo 2>/dev/null | awk '/dimensions:/{print $2; exit}')"
  if [ -n "$s" ]; then echo "${s}x24"; else echo "$SIZE"; fi
}

list_windows() {   # $1=display -> "id<TAB>name"，只要 ORB-SLAM3 / Pangolin 相关窗口
  DISPLAY="$1" xwininfo -root -tree 2>/dev/null \
    | sed -n 's/^[[:space:]]*\(0x[0-9a-fA-F]\{1,\}\)[[:space:]]*"\([^"]*\)".*/\1\t\2/p' \
    | grep -iE 'orb|pangolin|slam' || true
}

geom_of() {   # $1=display $2=id -> "x,y,w,h"
  local o x y w h
  o="$(DISPLAY="$1" xwininfo -id "$2" 2>/dev/null || true)"
  x="$(printf '%s\n' "$o" | awk '/Absolute upper-left X/{print $4}')"
  y="$(printf '%s\n' "$o" | awk '/Absolute upper-left Y/{print $4}')"
  w="$(printf '%s\n' "$o" | awk '/^  Width:/{print $2}')"
  h="$(printf '%s\n' "$o" | awk '/^  Height:/{print $2}')"
  [ -n "$x" ] && [ -n "$w" ] && [ -n "$h" ] && echo "$x,$y,$w,$h"
}

grab_root() {   # $1=display $2=out
  case "$GRAB" in
    import) DISPLAY="$1" import -window root "$2" 2>/dev/null ;;
    ffmpeg) ffmpeg -loglevel error -y -f x11grab -video_size "$(real_size "$1")" -i "$1" -frames:v 1 "$2" 2>/dev/null ;;
    scrot)  DISPLAY="$1" scrot -o "$2" 2>/dev/null ;;
  esac
  [ -s "$2" ]
}

grab_win() {    # $1=display $2=id $3=out
  case "$GRAB" in
    import) DISPLAY="$1" import -window "$2" "$3" 2>/dev/null ;;
    ffmpeg) local g gx gy gw gh
            g="$(geom_of "$1" "$2")" || return 1
            IFS=, read -r gx gy gw gh <<< "$g"
            ffmpeg -loglevel error -y -f x11grab -video_size "${gw}x${gh}" -i "$1+$gx,$gy" -frames:v 1 "$3" 2>/dev/null ;;
    scrot)  return 1 ;;
  esac
  [ -s "$3" ]
}

crop_win() {    # $1=src $2=out $3="x,y,w,h"
  IFS=, read -r cx cy cw ch <<< "$3"
  if [ "$HAVE_PIL" = 1 ]; then
    python3 - "$1" "$2" "$cx" "$cy" "$cw" "$ch" <<'PY' 2>/dev/null
import sys
from PIL import Image
src, dst, x, y, w, h = sys.argv[1], sys.argv[2], *[int(v) for v in sys.argv[3:7]]
Image.open(src).crop((x, y, x + w, y + h)).save(dst)
PY
  else
    convert "$1" -crop "${cw}x${ch}+${cx}+${cy}" +repage "$2" 2>/dev/null
  fi
  [ -s "$2" ]
}

sanitize() { printf '%s' "$1" | sed 's/[^A-Za-z0-9._-]/_/g'; }

place_windows() {   # 把两个窗口挪开，免得互相遮挡（只在自建 Xvfb 里做；需要 xdotool）
  [ "$PLACED" = 1 ] && return 0
  if ! command -v xdotool >/dev/null 2>&1; then
    echo "[!] 没装 xdotool：两个窗口可能互相遮挡，抓到遮挡区会发黑/花屏。"
    echo "    装上更稳： sudo apt-get install -y xdotool   （脚本仍会按窗口 id 直抓）"
    PLACED=1
    return 0
  fi
  command -v xdotool >/dev/null 2>&1 || return 0
  local id
  id="$(printf '%s\n' "$1" | awk -F'\t' '$2=="ORB-SLAM3: Map Viewer"{print $1; exit}')"
  [ -n "$id" ] && DISPLAY="$2" xdotool windowmove "$id" 0 0 2>/dev/null
  id="$(printf '%s\n' "$1" | awk -F'\t' '$2=="ORB-SLAM3: Current Frame"{print $1; exit}')"
  [ -n "$id" ] && DISPLAY="$2" xdotool windowmove "$id" 0 1150 2>/dev/null
  PLACED=1
  echo "[i] 已把窗口摆开（map 左上角 / frame 下方），避免互相遮挡"
  return 0
}

# ------------------------------- --list ------------------------------------
if [ "$MODE" = list ]; then
  D="${DISP:-${DISPLAY:-:0}}"
  echo "[i] DISPLAY=$D  屏幕=$(real_size "$D")"
  echo "[i] 抓屏工具=$GRAB  python3+PIL=$([ "$HAVE_PIL" = 1 ] && echo 有 || echo 无)"
  echo "--- 所有窗口（顶层树，前 60 行）---"
  DISPLAY="$D" xwininfo -root -tree 2>/dev/null | sed -n '1,60p'
  echo "--- 本脚本会抓的窗口 ---"
  list_windows "$D"
  exit 0
fi

# ------------------------------- 启动 --------------------------------------
XVFB_PID=""; SPID=""
PLACED=0
cleanup() {
  if [ -n "$SPID" ]; then
    pkill -TERM -P "$SPID" 2>/dev/null
    kill -TERM "$SPID" 2>/dev/null
  fi
  [ -n "$XVFB_PID" ] && kill "$XVFB_PID" 2>/dev/null
  return 0
}
trap cleanup EXIT INT TERM

if [ "$MODE" = run ]; then
  command -v Xvfb >/dev/null 2>&1 || { echo "[x] 缺 Xvfb:  sudo apt-get install -y xvfb"; exit 1; }
  n=99; while [ -e "/tmp/.X${n}-lock" ]; do n=$((n+1)); done
  DISP=":$n"; SIZE="${SIZE:-2400x2000x24}"
  Xvfb "$DISP" -screen 0 "$SIZE" +extension GLX +render -noreset >/dev/null 2>&1 & XVFB_PID=$!
  sleep 2
  echo "[i] Xvfb $DISP ($SIZE) pid=$XVFB_PID"
  echo "[i] SLAM 启动: $CMD"
  ( DISPLAY="$DISP" bash -c "$CMD" ) > "slam_stdout.log" 2>&1 & SPID=$!
else
  [ -n "$DISP" ] || DISP="${DISPLAY:-:0}"
fi
echo "[i] DISPLAY=$DISP  屏幕=$(real_size "$DISP")  抓屏工具=$GRAB  python3+PIL=$([ "$HAVE_PIL" = 1 ] && echo 有 || echo 无)"

mkdir -p "$OUT"
: > "$OUT/index.txt"

# ------------------------------- 主循环 ------------------------------------
N=$(( SECS / INTERVAL + 1 ))
i=0
while [ "$i" -lt "$N" ]; do
  i=$((i+1)); tag="$(printf '%03d' "$i")"

  grab_root "$DISP" "$OUT/root_$tag.png" || echo "[!] 第 $i 步整屏抓图失败"

  winlist="$(list_windows "$DISP")"
  printf '%s\n' "$winlist" > "$OUT/windows_$tag.txt"

  if [ -z "$(printf '%s' "$winlist" | tr -d '[:space:]')" ]; then
    echo "[!] 第 $i 步：没找到 ORB-SLAM3 窗口（viewer 没开？词典还在加载？）"
    printf '%s: none\n' "$tag" >> "$OUT/index.txt"
  else
    place_windows "$winlist" "$DISP"
    printf '%s\n' "$winlist" | while IFS=$'\t' read -r wid wname; do
      [ -n "${wid:-}" ] || continue
      safe="$(sanitize "$wname")"
      dst="$OUT/w_${tag}_${safe}.png"
      if grab_win "$DISP" "$wid" "$dst"; then
        echo "[i] 第 $i 步抓到窗口 [$wname] -> $dst"
      else
        g="$(geom_of "$DISP" "$wid" || true)"
        if [ -n "$g" ] && crop_win "$OUT/root_$tag.png" "$dst" "$g"; then
          echo "[i] 第 $i 步 [$wname] 直抓失败，按几何 $g 从整屏裁 -> $dst"
        else
          echo "[!] 第 $i 步 [$wname] (id=$wid) 抓不到"
        fi
      fi
    done
    { printf '%s: ' "$tag"; printf '%s ' $winlist; echo; } >> "$OUT/index.txt"
  fi

  [ "$i" -lt "$N" ] && sleep "$INTERVAL"
done

echo "======== 完成 ========"
ls -l "$OUT" 2>/dev/null | sed -n '1,40p'
echo "[i] Fig.4 左图 = $OUT/w_*_ORB_SLAM3_Map_Viewer.png 里地图最全的那张"
echo "[i] 右图(帧特征) = $OUT/w_*_ORB_SLAM3_Current_Frame.png（如果 ORB-SLAM3 开了这个窗口）"
echo "[i] 打包: tar czf fig4shots.tgz $OUT"
[i] 提示：本脚本被 run_compare.sh 用 --cmd 调用，抓 Fig.4 左图（Pangolin Map Viewer 真实窗口截图）。
[i] 想让地图长得更全再抓，可在 SLAM 命令里加 VIS_PACE_MS=200 VIS_HOLD_SEC=30。