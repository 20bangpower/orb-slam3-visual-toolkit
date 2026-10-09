// ---------------------------------------------------------------------------
// vis_export.h -- 把 ORB-SLAM3 运行过程中的可视化数据落盘
//
// 导出文件（都写在工作目录 CWD，或 VIS_OUT 指定的目录）：
//   current_frame.png     当前帧特征图（黑框=匹配到地图点，绿框=未匹配）
//   current_features.csv  u,v,used,vo
//   CameraTrajectory.txt  全部帧位姿，TUM 格式
//   KeyFrameTrajectory.txt 关键帧位姿，TUM 格式
//   map_points.csv        x,y,z（黑点来源）
//   keyframes.csv         x,y,z 关键帧相机中心
//   map_view.png          Pangolin 地图视图截图（需要 Viewer 开着）
//
// 逐帧快照（VIS_FRAMES_DIR 非空时才写，给"每一帧 a/a' 对照视频"用）：
//   F###_current_frame.png / F###_current_features.csv
//   F###_map_points.csv / F###_keyframes.csv
//   F###_kf_traj.txt / F###_cam_traj.txt   （到第 ### 帧为止的位姿快照）
//
// 常见的分支差异（本仓库所基于的 ORB-SLAM3 副本即如此）：
//   * MapPoint::GetWorldPos() / KeyFrame::GetCameraCenter() 返回类型随分支而异
//     （cv::Mat 或 Eigen::Vector3f），本文件自动识别，两种分支都能编译
//   * 当前帧那帧图改成自己用 cv::rectangle 画，不再依赖 FrameDrawer::DrawFrame()
//
// 另有 2 处需按实际分支核对（编译报错就改这两行）：
//   [API-A] FrameDrawer::GetCurrentFeatures(kps, vbMap, vbVO)
//   [API-B] Viewer::RequestSaveMapImage(path)
// 想看真实签名：bash dump_api.sh
// ---------------------------------------------------------------------------
#ifndef ORB_SLAM3_VIS_EXPORT_H
#define ORB_SLAM3_VIS_EXPORT_H

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <thread>
#include <string>
#include <vector>
#include <type_traits>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <System.h>
#include <FrameDrawer.h>
#include <Atlas.h>
#include <MapPoint.h>
#include <KeyFrame.h>
#include <Viewer.h>

// 置 1 则改用 FrameDrawer::DrawFrame() 出图（要求该函数存在）
#ifndef VIS_USE_DRAWFRAME
#define VIS_USE_DRAWFRAME 0
#endif

// 特征框的画法：外框半边长 / 线宽 / 中心小方块半边长
#ifndef VIS_BOX_HALF
#define VIS_BOX_HALF 8
#endif
#ifndef VIS_BOX_LW
#define VIS_BOX_LW 1
#endif
#ifndef VIS_DOT_HALF
#define VIS_DOT_HALF 2
#endif

namespace visexp {

// ---------------------------------------------------------------------------
// 0) 三维点通用解包：兼容 cv::Mat 与 Eigen::Vector3f 两种 ORB-SLAM3 分支
//    - 上游 / 多数分支：GetWorldPos() / GetCameraCenter() 返回 cv::Mat(3,1,CV_32F)
//    - 部分改过的分支：返回 Eigen::Vector3f
//    非模板重载接 cv::Mat，模板重载接 Eigen（用 enable_if 排除 cv::Mat，避免歧义）
// ---------------------------------------------------------------------------
namespace detail {

inline void unproject(const cv::Mat& m, float& x, float& y, float& z)
{
    x = y = z = 0.f;
    if(m.empty())
        return;
    cv::Mat d;
    m.convertTo(d, CV_32F);
    if(!d.isContinuous())
        d = d.clone();
    const float* p = d.ptr<float>(0);
    const size_t n = d.total();
    if(n >= 1) x = p[0];
    if(n >= 2) y = p[1];
    if(n >= 3) z = p[2];
}

template <typename Vec>
inline typename std::enable_if<
    !std::is_same<typename std::decay<Vec>::type, cv::Mat>::value, void>::type
unproject(const Vec& v, float& x, float& y, float& z)
{
    x = static_cast<float>(v(0));
    y = static_cast<float>(v(1));
    z = static_cast<float>(v(2));
}

}  // namespace detail

inline std::string ensureDir(const std::string& dir)
{
    return dir.empty() ? std::string(".") : dir;
}

// 读整数环境变量
inline int envInt(const char* name, int def)
{
    const char* e = getenv(name);
    return (e && e[0] != '\0') ? atoi(e) : def;
}

// VIS_PACE_MS：每帧之后睡一会儿。100 帧跑得太快（~2s），抓 Pangolin 窗口来不及。
inline void pace()
{
    const int ms = envInt("VIS_PACE_MS", 0);
    if(ms > 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// VIS_HOLD_SEC：跑完先别 Shutdown，保持窗口 N 秒，让抓图脚本有时间抓完最后一帧的地图。
inline void holdWindow()
{
    const int sec = envInt("VIS_HOLD_SEC", 0);
    if(sec > 0)
    {
        std::cout << "[visexp] 保持 Pangolin 窗口 " << sec
                  << " 秒（VIS_HOLD_SEC），方便抓窗口截图 ..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(sec));
    }
}

// ---------------------------------------------------------------------------
// 1) 当前帧特征图 + 特征表
//    自己画：黑框 = 匹配到地图点（vbMap），绿框 = 未匹配
// ---------------------------------------------------------------------------
inline bool saveCurrentFrame(ORB_SLAM3::System& SLAM,
                            const cv::Mat& imIn,
                            const std::string& pngPath,
                            const std::string& csvPath)
{
    ORB_SLAM3::FrameDrawer* pFD = SLAM.GetFrameDrawer();
    if(!pFD)
        return false;

    // ---- [API-A] 取当前帧特征 + vbMap / vbVO ----
    std::vector<cv::KeyPoint> vKps;
    std::vector<bool> vbMap, vbVO;
    pFD->GetCurrentFeatures(vKps, vbMap, vbVO);

    // ---- 特征表 ----
    if(!vKps.empty())
    {
        std::ofstream f(csvPath.c_str());
        if(f.is_open())
        {
            f << "u,v,used,vo\n";
            for(size_t i = 0; i < vKps.size(); i++)
            {
                const bool used = (i < vbMap.size()) ? vbMap[i] : false;
                const bool vo   = (i < vbVO.size())  ? vbVO[i]  : false;
                f << vKps[i].pt.x << "," << vKps[i].pt.y << ","
                  << (used ? 1 : 0) << "," << (vo ? 1 : 0) << "\n";
            }
        }
    }

    // ---- 当前帧图 ----
    cv::Mat im;
#if VIS_USE_DRAWFRAME
    im = pFD->DrawFrame();
#else
    if(!imIn.empty())
    {
        if(imIn.channels() == 1)
            cv::cvtColor(imIn, im, cv::COLOR_GRAY2BGR);
        else
            im = imIn.clone();

        for(size_t i = 0; i < vKps.size(); i++)
        {
            const bool used = (i < vbMap.size()) ? vbMap[i] : false;
            const cv::Scalar col = used ? cv::Scalar(0, 0, 0)        // 黑：匹配到地图点
                                       : cv::Scalar(0, 255, 0);      // 绿：未匹配
            const int cx = cvRound(vKps[i].pt.x);
            const int cy = cvRound(vKps[i].pt.y);
            cv::rectangle(im,
                          cv::Point(cx - VIS_BOX_HALF, cy - VIS_BOX_HALF),
                          cv::Point(cx + VIS_BOX_HALF, cy + VIS_BOX_HALF),
                          col, VIS_BOX_LW);
            cv::rectangle(im,
                          cv::Point(cx - VIS_DOT_HALF, cy - VIS_DOT_HALF),
                          cv::Point(cx + VIS_DOT_HALF, cy + VIS_DOT_HALF),
                          col, -1);
        }
    }
#endif

    if(!im.empty())
        cv::imwrite(pngPath, im);

    return !im.empty();
}

// ---------------------------------------------------------------------------
// 2) 轨迹（TUM 格式，直接给 pangolin_panel2.py 用）
// ---------------------------------------------------------------------------
inline void saveTrajectories(ORB_SLAM3::System& SLAM, const std::string& dir,
                             bool bMonocular = false)
{
    if(bMonocular)
    {
        // ORB-SLAM3 对单目会拒绝：SaveTrajectoryTUM cannot be used for monocular
        // 单目没有尺度、也没有逐帧位姿，只有关键帧位姿 -> 左图用 KeyFrameTrajectory.txt
        std::cout << "[visexp] 单目：跳过 CameraTrajectory.txt（无逐帧轨迹），"
                     "只写 KeyFrameTrajectory.txt" << std::endl;
    }
    else
    {
        SLAM.SaveTrajectoryTUM(dir + "/CameraTrajectory.txt");
    }
    SLAM.SaveKeyFrameTrajectoryTUM(dir + "/KeyFrameTrajectory.txt");
}

// ---------------------------------------------------------------------------
// 3) 地图点（黑点来源）
// ---------------------------------------------------------------------------
inline int saveMapPoints(ORB_SLAM3::System& SLAM, const std::string& path,
                         bool quiet = false)
{
    ORB_SLAM3::Atlas* pAtlas = SLAM.GetAtlas();
    if(!pAtlas)
        return 0;

    std::vector<ORB_SLAM3::MapPoint*> vpMPs = pAtlas->GetAllMapPoints();
    std::ofstream f(path.c_str());
    if(!f.is_open())
    {
        std::cerr << "[visexp] 打不开 " << path << std::endl;
        return 0;
    }
    f << "x,y,z\n";
    int n = 0;
    for(size_t i = 0; i < vpMPs.size(); i++)
    {
        if(!vpMPs[i] || vpMPs[i]->isBad())
            continue;
        // 兼容 cv::Mat / Eigen::Vector3f：统一解包成 float
        float mx = 0.f, my = 0.f, mz = 0.f;
        detail::unproject(vpMPs[i]->GetWorldPos(), mx, my, mz);
        f << mx << "," << my << "," << mz << "\n";
        n++;
    }
    if(!quiet)
        std::cout << "[visexp] map_points: " << n << " 点 -> " << path << std::endl;
    return n;
}

// ---------------------------------------------------------------------------
// 4) 关键帧相机中心
// ---------------------------------------------------------------------------
inline int saveKeyFrames(ORB_SLAM3::System& SLAM, const std::string& path,
                         bool quiet = false)
{
    ORB_SLAM3::Atlas* pAtlas = SLAM.GetAtlas();
    if(!pAtlas)
        return 0;

    std::vector<ORB_SLAM3::KeyFrame*> vpKFs = pAtlas->GetAllKeyFrames();
    std::ofstream f(path.c_str());
    if(!f.is_open())
        return 0;
    f << "x,y,z\n";
    int n = 0;
    for(size_t i = 0; i < vpKFs.size(); i++)
    {
        if(!vpKFs[i] || vpKFs[i]->isBad())
            continue;
        // 兼容 cv::Mat / Eigen::Vector3f：统一解包成 float
        float kx = 0.f, ky = 0.f, kz = 0.f;
        detail::unproject(vpKFs[i]->GetCameraCenter(), kx, ky, kz);
        f << kx << "," << ky << "," << kz << "\n";
        n++;
    }
    if(!quiet)
        std::cout << "[visexp] keyframes: " << n << " -> " << path << std::endl;
    return n;
}

// ---------------------------------------------------------------------------
// 5) 请 Viewer 存一张 Pangolin 地图视图（只在开 Viewer 时有效）
// ---------------------------------------------------------------------------
inline void requestMapShot(ORB_SLAM3::System& SLAM, const std::string& pngPath)
{
    ORB_SLAM3::Viewer* pV = SLAM.GetViewer();
    if(!pV)
    {
        std::cout << "[visexp] Viewer 没开（System 最后那个参数要为 true），跳过 map_view.png" << std::endl;
        return;
    }
    // ---- [API-B] ----
    pV->RequestSaveMapImage(pngPath);
}

// ---------------------------------------------------------------------------
// 6) 逐帧快照（VIS_FRAMES_DIR 非空时，每帧调一次）
//    给"每一帧 (a)正常 vs (a')干扰 对照视频"用：每帧一个完整的状态切片
// ---------------------------------------------------------------------------
inline std::string framesDir()
{
    const char* e = getenv("VIS_FRAMES_DIR");
    return (e && e[0] != '\0') ? std::string(e) : std::string();
}

inline std::string frameTag(int ni)
{
    char b[32];
    snprintf(b, sizeof(b), "F%03d", ni);
    return std::string(b);
}

// 逐帧快照：当前帧特征图 + 地图点 + 关键帧 + 到本帧为止的两条轨迹
inline bool saveFrameSnapshot(ORB_SLAM3::System& SLAM, const cv::Mat& imIn,
                              const std::string& dir, int ni,
                              bool bMonocular = false)
{
    if(dir.empty())
        return false;
    const std::string tag = frameTag(ni);

    saveCurrentFrame(SLAM, imIn, dir + "/" + tag + "_current_frame.png",
                     dir + "/" + tag + "_current_features.csv");
    saveMapPoints(SLAM, dir + "/" + tag + "_map_points.csv", true);
    saveKeyFrames(SLAM, dir + "/" + tag + "_keyframes.csv", true);

    // SaveXxxTUM 写的是"到目前为止"的全部位姿 -> 正好是第 ni 帧的快照。
    // 必须等地图里有关键帧之后再写（内部会取 vpKFs[0]，空地图会崩）。
    ORB_SLAM3::Atlas* pAtlas = SLAM.GetAtlas();
    if(pAtlas && !pAtlas->GetAllKeyFrames().empty())
    {
        // 万一某帧写轨迹抛异常，也不能把整趟跟踪带崩
        try
        {
            SLAM.SaveKeyFrameTrajectoryTUM(dir + "/" + tag + "_kf_traj.txt");
            if(!bMonocular)
                SLAM.SaveTrajectoryTUM(dir + "/" + tag + "_cam_traj.txt");
        }
        catch(const std::exception& e)
        {
            std::cerr << "[visexp] 逐帧轨迹 " << tag << " 写失败: "
                      << e.what() << std::endl;
        }
    }
    return true;
}
// ---------------------------------------------------------------------------
// 收尾：一次把该存的都存了
// ---------------------------------------------------------------------------
inline void saveAll(ORB_SLAM3::System& SLAM, const std::string& dir,
                    bool bMonocular = false)
{
    const std::string d = ensureDir(dir);
    saveTrajectories(SLAM, d, bMonocular);
    saveMapPoints(SLAM, d + "/map_points.csv");
    saveKeyFrames(SLAM, d + "/keyframes.csv");
    // 注意：requestMapShot() 必须在 SLAM.Shutdown() 之前调用，否则渲染线程已经停了，
    // 图存不出来。所以它不放在这里，见 mono_tum_vis.cc。
}

}  // namespace visexp

#endif  // ORB_SLAM3_VIS_EXPORT_H