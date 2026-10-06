#!/usr/bin/env bash
# --- CRLF 自愈：本文件若是 CRLF 行尾，先转成 LF 再重新执行自己 ---
if [ -z "${_LF_FIXED:-}" ] && grep -qU "$(printf '\r')" "$0" 2>/dev/null; then
  sed 's/\r$//' "$0" > "$0.lf.tmp" || true
  if [ -s "$0.lf.tmp" ]; then mv "$0.lf.tmp" "$0" || true; else rm -f "$0.lf.tmp"; fi
  _LF_FIXED=1 exec bash "$0" "$@"
fi
# ---------------------------------------------------------------------------
# install_dataset.sh -- 把 slam_stereo_pairs_0_99_20260929 装好并核验
#
#   bash install_dataset.sh                      # 已解压就只做校验+清单
#   ZIP=~/xxx.zip bash install_dataset.sh        # 有 zip 就顺便解压
#   DS_ROOT=/path/to/slam_stereo_pairs_0_99_20260929 bash install_dataset.sh
#
# 数据集根目录怎么定（按优先级）：
#   1) DS_ROOT 直接指定
#   2) $DEST/slam_stereo_pairs_0_99_20260929   （DEST 缺省 ~/dataset/4Seasons）
#   3) 自动找: ~/dataset/4Seasons/<NAME> | ~/dataset/<NAME> | ~/<NAME>
#
# 做三件事：解压（若需要）-> SHA256 全量校验 -> 生成三组的绝对路径清单
# ---------------------------------------------------------------------------
set -euo pipefail

NAME="slam_stereo_pairs_0_99_20260929"
ZIP="${ZIP:-$HOME/$NAME.zip}"

pick_root() {
  local c
  if [ -n "${DS_ROOT:-}" ]; then echo "$DS_ROOT"; return; fi
  if [ -n "${DEST:-}" ];   then echo "$DEST/$NAME"; return; fi
  for c in "$HOME/dataset/4Seasons/$NAME" "$HOME/dataset/$NAME" "$HOME/$NAME"; do
    if [ -d "$c/clean" ]; then echo "$c"; return; fi
  done
  echo "$HOME/dataset/$NAME"
}

ROOT="$(pick_root)"
echo "[i] 数据集根目录: $ROOT"

# ---------------------------------------------------------------- 1) 解压
if [ -d "$ROOT/clean" ]; then
  echo "[1/3] $ROOT 已经是解压好的包，跳过解压"
elif [ -f "$ZIP" ]; then
  echo "[1/3] 解压 $ZIP -> $(dirname "$ROOT")"
  mkdir -p "$(dirname "$ROOT")"
  unzip -q -o "$ZIP" -d "$(dirname "$ROOT")"
else
  echo "[x] $ROOT 不存在，也没找到压缩包 $ZIP"
  echo "    要么把 zip 放好: ZIP=/path/to/$NAME.zip bash $0"
  echo "    要么直接指到解压好的目录: DS_ROOT=/path/to/$NAME bash $0"
  exit 1
fi

if [ ! -d "$ROOT/clean" ]; then
  echo "[x] $ROOT 里没有 clean/ —— 确认一下是不是多套了一层目录"
  echo "    现在的内容："; ls -1 "$ROOT" | head -20 | sed 's/^/      /'
  exit 1
fi

# ------------------------------------------------------------ 2) 校验
echo
echo "[2/3] SHA256 全量校验（包自带 verify_files.py）"
python3 "$ROOT/verify_files.py"

# ------------------------------------------------------------ 3) 清单
echo
echo "[3/3] 生成绝对路径清单"
for g in clean fixed_pixel world_plane; do
  python3 "$ROOT/build_absolute_manifest.py" "$g"
done

echo
echo "======== 数据集就绪 ========"
echo "DS_ROOT = $ROOT"
for g in clean fixed_pixel world_plane; do
  n=$(ls "$ROOT/$g/cam0" 2>/dev/null | wc -l || true)
  m=$(ls "$ROOT/$g/cam1" 2>/dev/null | wc -l || true)
  printf '  %-12s cam0=%-4s cam1=%-4s\n' "$g" "$n" "$m"
done
echo
echo "[i] 图像 800x400、已去畸变+双目校正，不要再校正。"
echo "[i] 下一步: bash make_orb_inputs.sh"