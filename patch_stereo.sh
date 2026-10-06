#!/usr/bin/env bash
# --- CRLF 自愈：本文件若是 CRLF 行尾，先转成 LF 再重新执行自己 ---
if [ -z "${_LF_FIXED:-}" ] && grep -qU "$(printf '\r')" "$0" 2>/dev/null; then
  sed 's/\r$//' "$0" > "$0.lf.tmp" || true
  if [ -s "$0.lf.tmp" ]; then mv "$0.lf.tmp" "$0" || true; else rm -f "$0.lf.tmp"; fi
  _LF_FIXED=1 exec bash "$0" "$@"
fi
# ---------------------------------------------------------------------------
# patch_stereo.sh -- 把双目入口 stereo_tum_vi_vis 装进 ORB-SLAM3 并编译
#
#   bash patch_stereo.sh               # 正常用法（双目，右图 = FrameDrawer::DrawFrame()，带状态条）
#   bash patch_stereo.sh --plain       # 右图改回自己画框（不依赖 DrawFrame，编译兜底用）
#   bash patch_stereo.sh --with-cmake  # 兜底：往 CMake 清单追加目标再 cmake+make（一般不用）
#
# 编译路线（和 patch_mono.sh 一样，避开了这个副本没有 add_executable 的问题）：
#   复用已经编好的 stereo_tum_vi 的 flags.make + link.txt，只把
#   stereo_tum_vi.cc.o 换成 stereo_tum_vi_vis.cc.o、输出名换成 stereo_tum_vi_vis，
#   直接调编译器。不跑 cmake，不动任何构建配置，不动官方 stereo_tum_vi.cc。
# ---------------------------------------------------------------------------
set -euo pipefail

WITH_CMAKE=0
DRAWFRAME=1
for a in "$@"; do
  case "$a" in
    --with-cmake) WITH_CMAKE=1 ;;
    --drawframe)  DRAWFRAME=1 ;;
    --plain|--no-drawframe) DRAWFRAME=0 ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "[x] 未知参数: $a"; exit 1 ;;
  esac
done

ROOT="${ROOT:-$HOME/ORB_SLAM3}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PY="${PY:-}"
if [ -z "$PY" ]; then
  if command -v python3 >/dev/null 2>&1; then PY=python3; else PY=python; fi
fi

TARGET="stereo_tum_vi_vis"
SRC_REL="Examples/Stereo/stereo_tum_vi_vis.cc"
DST_DIR="$ROOT/Examples/Stereo"
REFS="stereo_tum_vi,stereo_euroc,stereo_kitti,stereo_tum_vi_old,rgbd_tum,mono_tum_vi,mono_tum"

echo "[0/4] ROOT=$ROOT   PY=$PY   目标=$TARGET"
if [ ! -d "$ROOT" ]; then echo "[x] 没有 $ROOT（用 ROOT=/path/to/ORB_SLAM3 指定）"; exit 1; fi

echo "[1/4] 拷文件 -> $DST_DIR"
mkdir -p "$DST_DIR"
cp -f "$HERE/vis_export.h"          "$DST_DIR/vis_export.h"
cp -f "$HERE/stereo_tum_vi_vis.cc"  "$DST_DIR/stereo_tum_vi_vis.cc"

if [ "$DRAWFRAME" = "1" ]; then
  sed -i 's/^#define VIS_USE_DRAWFRAME 0$/#define VIS_USE_DRAWFRAME 1/' "$DST_DIR/vis_export.h"
  echo "      [i] 右图用 FrameDrawer::DrawFrame()（VIS_USE_DRAWFRAME=1）"
  echo "          自带 黑框=匹配到地图点 / 绿框=未匹配 + 底部 SLAM MODE|Maps..KFs..MPs..Matches.. 状态条"
  echo "          如果编译报“没有 DrawFrame 成员”，改用 bash patch_stereo.sh --plain 重跑。"
else
  echo "      [i] 右图用手绘特征框（黑=匹配到地图点，绿=未匹配），不带状态条"
fi
LOG="$HERE/build_stereo_vis.log"
BIN=""

echo
echo "[2/4] 路线 1：复用已有例子（stereo_tum_vi）的编译参数"
rc=0
"$PY" "$HERE/build_mono_vis.py" --root "$ROOT" \
  --target "$TARGET" --source "$SRC_REL" --refs "$REFS" > "$LOG" 2>&1 || rc=$?

if [ "$rc" -eq 0 ]; then
  BIN="$(sed -n 's/^BIN=//p' "$LOG" | tail -1)"
  tail -6 "$LOG" | sed 's/^/      /'
  echo "      [ok] 路线 1 成功"
elif [ "$rc" -eq 3 ]; then
  echo "      [!] 没找到可复用的构建参数（rc=3）。最后 12 行："
  tail -12 "$LOG" | sed 's/^/      /'
  echo
  echo "      先看看有哪些双目目标可以当参照："
  echo "        find $ROOT -name link.txt | grep -iE 'stereo|mono|rgbd' | head"
  echo
  if [ "$WITH_CMAKE" = "1" ]; then
    echo "[3/4] 路线 2（--with-cmake）：往 CMake 清单追加目标"
    echo "      [!] 会重跑 cmake，可能影响你现有 build/ 的目标，请自行确认"
    "$PY" "$HERE/patch_cmake.py" --root "$ROOT" \
      --source "$SRC_REL" --target "$TARGET" \
      --refs "stereo_tum_vi,rgbd_tum,mono_tum_vi,mono_tum,fig4_pangolin" || exit 1
    BUILD="$ROOT/build"
    if [ ! -f "$BUILD/CMakeCache.txt" ]; then
      cand="$(find "$ROOT" -maxdepth 4 -name CMakeCache.txt -print -quit 2>/dev/null || true)"
      if [ -n "$cand" ]; then BUILD="$(dirname "$cand")"; fi
    fi
    ( cd "$BUILD" && cmake .. > "$LOG" 2>&1 ) || { echo "[!] cmake 告警，最后 10 行："; tail -10 "$LOG" | sed 's/^/      /'; }
    if ! ( cd "$BUILD" && make -j"$(nproc)" "$TARGET" >> "$LOG" 2>&1 ); then
      echo "[x] 编译失败，最后 40 行（完整见 $LOG）："
      tail -40 "$LOG" | sed 's/^/      /'
      exit 1
    fi
    BIN="$(find "$BUILD" -maxdepth 4 -name "$TARGET" -type f -print -quit 2>/dev/null || true)"
  else
    echo "      [x] 路线 1 没拿到参数。请对照 build_stereo_vis.log 和上面 find 的输出。"
    echo "          （想强行走 CMake 那条路：bash patch_stereo.sh --with-cmake）"
    exit 1
  fi
else
  echo "      [x] 编译/链接报错（不是找不到参数），最后 30 行（完整见 $LOG）："
  tail -30 "$LOG" | sed 's/^/      /'
  {
    echo
    echo "==== 自动附上 dump_api.sh 的输出（你分支上的真实签名）===="
    ROOT="$ROOT" bash "$HERE/dump_api.sh" 2>&1 || true
  } >> "$LOG"
  echo
  echo "      已把 dump_api.sh 的签名信息追加进同一份日志，按日志提示对齐签名即可："
  echo "        cat $LOG"
  exit 1
fi

echo
echo "[4/4] 自检"
if [ -z "$BIN" ] || [ ! -x "$BIN" ]; then
  BIN="$(find "$ROOT" -maxdepth 5 -name "$TARGET" -type f -perm -u+x -print -quit 2>/dev/null || true)"
fi
if [ -z "$BIN" ] || [ ! -x "$BIN" ]; then
  echo "[x] 编译过了但没找到可执行文件，请查看日志 $LOG"; exit 1
fi
ls -l "$BIN"
if ldd "$BIN" 2>/dev/null | grep -q pango_; then
  ldd "$BIN" | grep pango_ | sed 's/^/      /'
else
  echo "[!] ldd 里没看到 pangolin —— Viewer 可能开不起来"
fi

echo
echo "======== 装好了 ========"
echo "BIN = $BIN"
echo "下一步: bash run_compare.sh --all"