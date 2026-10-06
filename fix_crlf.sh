#!/usr/bin/env bash
# --- CRLF 自愈：本文件若是 CRLF 行尾，先转成 LF 再重新执行自己 ---
if [ -z "${_LF_FIXED:-}" ] && grep -qU "$(printf '\r')" "$0" 2>/dev/null; then
  sed 's/\r$//' "$0" > "$0.lf.tmp" || true
  if [ -s "$0.lf.tmp" ]; then mv "$0.lf.tmp" "$0" || true; else rm -f "$0.lf.tmp"; fi
  _LF_FIXED=1 exec bash "$0" "$@"
fi
# 本目录下所有脚本统一成 LF 行尾（Windows 传过来的文件常见问题）
# 用法: bash fix_crlf.sh
cd "$(dirname "${BASH_SOURCE[0]}")" || exit 1
n=0
for f in *.sh *.py; do
  [ -f "$f" ] || continue
  if grep -q $'\r' "$f" 2>/dev/null; then
    sed -i 's/\r$//' "$f"
    echo "[i] 已转换 $f"
    n=$((n+1))
  fi
done
echo "[i] 共转换 $n 个文件"