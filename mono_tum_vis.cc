// ---------------------------------------------------------------------------
// mono_tum_vis.cc -- 单目入口，带 Fig.4 那套导出（换 4Seasons 数据集用这个）
//
// 和官方 Examples/Monocular/mono_tum.cc 的区别：
//   1) 关联文件支持绝对路径，也支持相对路径（相对 argv[3] 数据集根目录）
//   2) 每帧跟踪完调用 visexp::saveCurrentFrame()，落 current_frame.png /
//      current_features.csv（最后一帧覆盖前面的，和 RGB-D 版行为一致）
//   3) 结束前调用 visexp::saveAll()，落轨迹 / 地图点 / 关键帧 / map_view.png
//
// 编译安装：bash patch_mono.sh   （会自动拷文件 + 改 CMakeLists + make）
// 运行：
//   cd <运行目录>          # 导出文件就写在这里
//   <SLAM_ROOT>/Examples/Monocular/mono_tum_vis \
//       <SLAM_ROOT>/Vocabulary/ORBvoc.txt <...>/4seasons_orb3_mono.yaml \
//       <DS_ROOT>/clean <...>/clean_mono.txt
// ---------------------------------------------------------------------------
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <numeric>
#include <thread>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <System.h>
#include "vis_export.h"

using namespace std;

namespace {

bool isAbsolutePath(const string& p)
{
    if(p.empty())
        return false;
    if(p[0] == '/')
        return true;
    // Windows 风格盘符（虽然主要跑在 Ubuntu，留着不影响）
    return p.size() > 1 && p[1] == ':';
}

string joinPath(const string& a, const string& b)
{
    if(b.empty())
        return a;
    if(isAbsolutePath(b))
        return b;
    if(a.empty())
        return b;
    if(a[a.size() - 1] == '/')
        return a + b;
    return a + "/" + b;
}

// 关联文件每行: "<时间戳秒> <左图路径>"，用空格或 TAB 分隔
bool LoadImages(const string& strAssociationFilename,
                const string& strDatasetRoot,
                vector<string>& vstrImageFilenames,
                vector<double>& vTimestamps)
{
    ifstream fAssociation(strAssociationFilename.c_str());
    if(!fAssociation.is_open())
    {
        cerr << "[x] 打不开关联文件: " << strAssociationFilename << endl;
        return false;
    }
    string s;
    while(getline(fAssociation, s))
    {
        if(s.empty())
            continue;
        if(s[0] == '#')
            continue;
        stringstream ss(s);
        double t = 0.0;
        string sImg;
        if(!(ss >> t >> sImg))
            continue;
        vTimestamps.push_back(t);
        vstrImageFilenames.push_back(joinPath(strDatasetRoot, sImg));
    }
    fAssociation.close();
    cout << "[i] 关联文件 " << vstrImageFilenames.size() << " 帧: "
         << strAssociationFilename << endl;
    return !vstrImageFilenames.empty();
}

}  // namespace

int main(int argc, char** argv)
{
    if(argc < 4)
    {
        cerr << "Usage: ./mono_tum_vis path_to_vocabulary path_to_settings "
                "path_to_dataset_root [path_to_association]" << endl;
        cerr << "   关联文件缺省为 <dataset_root>/pairs_orb_mono.txt" << endl;
        return 1;
    }

    const string strVocFile      = argv[1];
    const string strSettingsFile = argv[2];
    const string strDatasetRoot  = argv[3];
    const string strAssocFile    = (argc >= 5) ? string(argv[4])
                                              : joinPath(strDatasetRoot, "pairs_orb_mono.txt");

    // 导出到哪：VIS_OUT 优先，否则用当前工作目录（和 RGB-D 版一致）
    string strOutDir = ".";
    if(const char* env = getenv("VIS_OUT"))
    {
        if(env[0] != '\0')
            strOutDir = env;
    }

    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    if(!LoadImages(strAssocFile, strDatasetRoot, vstrImageFilenames, vTimestamps))
        return 1;

    const int nImages = int(vstrImageFilenames.size());
    cout << "[i] 数据集: " << strDatasetRoot << endl;
    cout << "[i] 导出到: " << strOutDir << endl;

    // 逐帧快照目录（VIS_FRAMES_DIR）；留空 = 只存最后一帧（老行为）
    const string strFramesDir = visexp::framesDir();
    if(!strFramesDir.empty())
        cout << "[i] 逐帧快照 -> " << strFramesDir << "/F###_*（供对照视频用）" << endl;

    // Viewer 开关：VIS_NO_VIEWER=1 时关掉（不需要 X）
    bool bUseViewer = true;
    if(const char* e = getenv("VIS_NO_VIEWER"))
        if(e[0] == '1')
            bUseViewer = false;

    // 单目：最后一个参数 true 才有 Viewer（map_view.png 靠它）
    ORB_SLAM3::System SLAM(strVocFile, strSettingsFile,
                           ORB_SLAM3::System::MONOCULAR, bUseViewer);

    vector<float> vTimesTrack(nImages);
    int nSaved = 0;

    cout << "[i] 开始跟踪 " << nImages << " 帧 ..." << endl;
    for(int ni = 0; ni < nImages; ni++)
    {
        cv::Mat im = cv::imread(vstrImageFilenames[ni].c_str(), cv::IMREAD_UNCHANGED);
        if(im.empty())
        {
            cerr << "[!] 读不了图，跳过: " << vstrImageFilenames[ni] << endl;
            vTimesTrack[ni] = 0.0f;
            continue;
        }

        // 输入是灰度就直传；三通道按 RGB 读进来（yaml 里 Camera.RGB: 0 -> BGR）
        if(im.channels() == 4)
            cv::cvtColor(im, im, cv::COLOR_RGBA2BGR);

        const double tframe = vTimestamps[ni];

        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        SLAM.TrackMonocular(im, tframe);
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();

        vTimesTrack[ni] = chrono::duration_cast<chrono::duration<float> >(t2 - t1).count();

        // 每帧都存，最后一帧覆盖前面的 -> 最终就是"最后一帧"
        if(visexp::saveCurrentFrame(SLAM, im, strOutDir + "/current_frame.png",
                                    strOutDir + "/current_features.csv"))
            nSaved++;

        // 逐帧快照（VIS_FRAMES_DIR 非空时）：单目没有逐帧位姿，只写 KF 轨迹
        if(!strFramesDir.empty())
            visexp::saveFrameSnapshot(SLAM, im, strFramesDir, ni, true);

        if((ni + 1) % 10 == 0 || ni == nImages - 1)
            cout << "    " << (ni + 1) << "/" << nImages
                 << "  tracking=" << vTimesTrack[ni] << "s" << endl;
    }

    cout << "[i] 跟踪结束，存过 " << nSaved << " 张 current_frame.png" << endl;

    // 地图截图必须在 Shutdown 之前请求，并留时间给 Viewer 渲染线程落盘
    visexp::requestMapShot(SLAM, strOutDir + "/map_view.png");
    cout << "[i] 等 2 秒让 Viewer 落盘 map_view.png ..." << endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    SLAM.Shutdown();

    // 单目：只有关键帧轨迹，没有逐帧轨迹
    visexp::saveAll(SLAM, strOutDir, true);

    const float ttrack = accumulate(vTimesTrack.begin(), vTimesTrack.end(), 0.0f);
    cout << "[i] 总跟踪耗时 " << ttrack << "s（平均 "
         << (ttrack / nImages) << "s/帧）" << endl;
    cout << "[i] 产物：" << endl
         << "      " << strOutDir << "/current_frame.png" << endl
         << "      " << strOutDir << "/current_features.csv" << endl
         << "      " << strOutDir << "/KeyFrameTrajectory.txt" << endl
         << "      (单目没有逐帧轨迹，不写 CameraTrajectory.txt)" << endl

         << "      " << strOutDir << "/map_points.csv" << endl
         << "      " << strOutDir << "/keyframes.csv" << endl
         << "      " << strOutDir << "/map_view.png" << endl;

    return 0;
}
