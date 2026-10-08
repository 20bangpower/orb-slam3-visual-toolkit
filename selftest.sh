#!/usr/bin/env bash
# --- CRLF 自愈：本文件若是 CRLF 行尾，先转成 LF 再重新执行自己 ---
if [ -z "${_LF_FIXED:-}" ] && grep -qU "$(printf '\r')" "$0" 2>/dev/null; then
  sed 's/\r$//' "$0" > "$0.lf.tmp" || true
  if [ -s "$0.lf.tmp" ]; then mv "$0.lf.tmp" "$0" || true; else rm -f "$0.lf.tmp"; fi
  _LF_FIXED=1 exec bash "$0" "$@"
fi
# ---------------------------------------------------------------------------
# selftest.sh -- 自检：自己造假数据把整条出图链路跑一遍
#
#   bash selftest.sh                 # 全部用例（约半分钟）
#   bash selftest.sh --quick         # 最小规模先看通不通
#   bash selftest.sh --out /tmp/st   # 指定沙箱目录
#
# 不需要数据集、不需要跑 SLAM、不需要 X。跑完给出 通过数/总数，
# 详细报告在 <沙箱>/selftest_report.txt
# ---------------------------------------------------------------------------
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${PY:-python3}"

if ! command -v "$PY" >/dev/null 2>&1; then
  echo "[x] 找不到 $PY（可以 PY=/usr/bin/python3 bash selftest.sh 指定）"
  exit 1
fi
if [ ! -f "$HERE/selftest.py" ]; then
  echo "[x] 缺 $HERE/selftest.py"
  exit 1
fi

exec "$PY" "$HERE/selftest.py" "$@"