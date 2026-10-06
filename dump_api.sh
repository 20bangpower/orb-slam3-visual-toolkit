#!/usr/bin/env bash
# --- CRLF 自愈：本文件若是 CRLF 行尾，先转成 LF 再重新执行自己 ---
if [ -z "${_LF_FIXED:-}" ] && grep -qU "$(printf '\r')" "$0" 2>/dev/null; then
  sed 's/\r$//' "$0" > "$0.lf.tmp" || true
  if [ -s "$0.lf.tmp" ]; then mv "$0.lf.tmp" "$0" || true; else rm -f "$0.lf.tmp"; fi
  _LF_FIXED=1 exec bash "$0" "$@"
fi
# ---------------------------------------------------------------------------
# dump_api.sh -- 打印当前 ORB-SLAM3 里那几个 API 的真实签名
#
# 用途：mono_tum_vis.cc / vis_export.h 里有 5 处标了 "API 对齐点"，
# 如果签名与预期不一致，编译会报错。先跑这个脚本，
# 再按实际签名改 vis_export.h 一个文件即可。
# ---------------------------------------------------------------------------
set -uo pipefail
ROOT="${ROOT:-$HOME/ORB_SLAM3}"

echo "==== ROOT = $ROOT"
echo
echo "---- 1) System.h: getter 声明 ----"
grep -n "GetFrameDrawer\|GetAtlas\|GetViewer" "$ROOT/include/System.h" 2>/dev/null
echo
echo "---- 2) System.cc: getter 实现 ----"
grep -n -A4 "System::GetFrameDrawer\|System::GetAtlas\|System::GetViewer" "$ROOT/src/System.cc" 2>/dev/null
echo
echo "---- 3) FrameDrawer.h: 全部公有方法（看 DrawFrame / GetCurrentFeatures 签名）----"
grep -n "cv::Mat\|void \|std::vector\|bool " "$ROOT/include/FrameDrawer.h" 2>/dev/null
echo
echo "---- 4) Viewer.h: 存图相关 ----"
grep -n "RequestSaveMapImage\|mbSaveMapImg\|mMapImgFile" "$ROOT/include/Viewer.h" 2>/dev/null
echo
echo "---- 5) Viewer.cc: RequestSaveMapImage 实现（看要不要传文件名）----"
grep -n -A8 "Viewer::RequestSaveMapImage" "$ROOT/src/Viewer.cc" 2>/dev/null
echo
echo "---- 6) 蓝本：rgbd_tum.cc 里现有的导出块（照抄它就不会错）----"
grep -n -B4 -A16 "current_features.csv" "$ROOT/Examples/RGB-D/rgbd_tum.cc" 2>/dev/null
echo
echo "---- 7) 蓝本：rgbd_tum.cc 里的 GetCurrentFeatures 调用 ----"
grep -n -B3 -A8 "GetCurrentFeatures" "$ROOT/Examples/RGB-D/rgbd_tum.cc" "$ROOT/src/FrameDrawer.cc" 2>/dev/null
echo
echo "---- 8) Atlas.h: 取点/取关键帧 ----"
grep -n "GetAllMapPoints\|GetAllKeyFrames" "$ROOT/include/Atlas.h" 2>/dev/null
echo
echo "---- 9) MapPoint / KeyFrame: 取坐标 ----"
grep -n "GetWorldPos\|GetCameraCenter" "$ROOT/include/MapPoint.h" "$ROOT/include/KeyFrame.h" 2>/dev/null