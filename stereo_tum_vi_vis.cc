// ---------------------------------------------------------------------------
// stereo_tum_vi_vis.cc -- 双目入口（4Seasons 双目包用这个），带 Fig.4 那套导出
//
// 和官方 Examples/Stereo/stereo_tum_vi.cc 的区别：
//   1) 关联文件是 3 列 "<时间戳秒> <cam0/xx.png> <cam1/xx.png>"，支持相对 argv[3]
//      数据集根目录（也支持绝对路径）
//   2) 每帧 TrackStereo() 完调 visexp::saveCurrentFrame()，落 current_frame.png /
//      current_features.csv（用左目图叠加：黑框=匹配到地图点，绿框=未匹配）
//   3) 结束前：requestMapShot() -> 等 2s -> holdWindow()（保持 Pangolin 窗口 N 秒，
//      让 capture_pangolin.sh 有时间抓真实窗口截图）-> Shutdown() -> saveAll()
//   4) VIS_PACE_MS：每帧后睡一会儿，把 100 帧放慢，好让抓图脚本抓得到
//   5) VIS_FRAMES_DIR：非空时把每一帧的完整快照写进该目录（F###_*），
//      给"每一帧 (a)正常 vs (a')干扰 对照视频"用（make_fig4_video.py --mode fig4）
//
// 编译：bash patch_stereo.sh        （拷文件 + 复用 stereo_tum_vi 的编译参数）
// 运行：
//   cd <运行目录>          # 导出文件写在这里
//   <SLAM_ROOT>/Examples/Stereo/stereo_tum_vi_vis \
//       <SLAM_ROOT>/Vocabulary/ORBvoc.txt <DS>/orb_inputs/4seasons_orb3_stereo.yaml \
//       <DS>/clean <DS>/orb_inputs/clean_stereo.txt
// ---------------------------------------------------------------------------
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
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

// 关联文件每行: "<时间戳秒> <左图> <右图>"，空格或 TAB 分隔
bool LoadStereoImages(const string& strAssociationFilename,
                      const string& strDatasetRoot,
                      vector<string>& vstrLeft,
                      vector<string>& vstrRight,
                      vector<double>& vTimestamps)
{
    ifstream fAssociation(strAssociationFilename.c_str());
    if(!fAssociation.is_open())
    {
        cerr << "[x] 打不开关联文件: " << strAssociationFilename << endl;
        return false;
    }

    string s;
    int nBad = 0;
    while(getline(fAssociation, s))
    {
        if(s.empty() || s[0] == '#')
            continue;
        stringstream ss(s);
        double t = 0.0;
        string sLeft, sRight;
        if(!(ss >> t >> sLeft >> sRight))
        {
            nBad++;
            continue;
        }
        vTimestamps.push_back(t);
        vstrLeft.push_back(joinPath(strDatasetRoot, sLeft));
        vstrRight.push_back(joinPath(strDatasetRoot, sRight));
    }
    fAssociation.close();

    if(nBad > 0)
        cerr << "[!] 有 " << nBad << " 行不是 3 列（时间戳 左图 右图），已跳过" << endl;

    cout << "[i] 关联文件 " << vstrLeft.size() << " 对: " << strAssociationFilename << endl;
    return !vstrLeft.empty();
}

}  // namespace

int main(int argc, char** argv)
{
    if(argc < 4)
    {
        cerr << "Usage: ./stereo_tum_vi_vis path_to_vocabulary path_to_settings "
                "path_to_dataset_root [path_to_association]" << endl;
        cerr << "   关联文件缺省为 <dataset_root>/pairs_orb_stereo.txt" << endl;
        cerr << "   关联文件每行: <时间戳秒> <cam0/xx.png> <cam1/xx.png>" << endl;
        return 1;
    }

    const string strVocFile      = argv[1];
    const string strSettingsFile = argv[2];
    const string strDatasetRoot  = argv[3];
    const string strAssocFile    = (argc >= 5) ? string(argv[4])
                                              : joinPath(strDatasetRoot, "pairs_orb_stereo.txt");

    // 导出到哪：VIS_OUT 优先，否则用当前工作目录
    string strOutDir = ".";
    if(const char* env = getenv("VIS_OUT"))
    {
        if(env[0] != '\0')
            strOutDir = env;
    }

    vector<string> vstrLeft, vstrRight;
    vector<double> vTimestamps;
    if(!LoadStereoImages(strAssocFile, strDatasetRoot, vstrLeft, vstrRight, vTimestamps))
        return 1;

    if(vstrLeft.size() != vstrRight.size())
    {
        cerr << "[x] 左右目数量不一致: " << vstrLeft.size()
             << " vs " << vstrRight.size() << endl;
        return 1;
    }

    const int nImages = int(vstrLeft.size());
    cout << "[i] 数据集: " << strDatasetRoot << endl;
    cout << "[i] 导出到: " << strOutDir << endl;

    // 逐帧快照目录（VIS_FRAMES_DIR）；留空 = 只存最后一帧（老行为）
    const string strFramesDir = visexp::framesDir();
    if(!strFramesDir.empty())
        cout << "[i] 逐帧快照 -> " << strFramesDir << "/F###_*（供对照视频用）" << endl;

    // Viewer 开关：VIS_NO_VIEWER=1 时关掉（不需要 X，跑逐帧快照更快；
    // 代价是没有 map_view.png，但对照视频不需要它）
    bool bUseViewer = true;
    if(const char* e = getenv("VIS_NO_VIEWER"))
        if(e[0] == '1')
            bUseViewer = false;
    if(!bUseViewer)
        cout << "[i] VIS_NO_VIEWER=1 -> 不开 Pangolin Viewer（不需要 X）" << endl;

    // 双目：最后一个参数 true 才有 Viewer（map_view.png / 抓窗口截图都靠它）
    ORB_SLAM3::System SLAM(strVocFile, strSettingsFile,
                           ORB_SLAM3::System::STEREO, bUseViewer);

    vector<float> vTimesTrack(nImages);
    int nSaved = 0, nSkip = 0;

    cout << "[i] 开始双目跟踪 " << nImages << " 帧 ..." << endl;
    for(int ni = 0; ni < nImages; ni++)
    {
        cv::Mat imLeft  = cv::imread(vstrLeft[ni].c_str(),  cv::IMREAD_UNCHANGED);
        cv::Mat imRight = cv::imread(vstrRight[ni].c_str(), cv::IMREAD_UNCHANGED);
        if(imLeft.empty() || imRight.empty())
        {
            cerr << "[!] 读不了图，跳过: " << vstrLeft[ni]
                 << " | " << vstrRight[ni] << endl;
            nSkip++;
            vTimesTrack[ni] = 0.0f;
            continue;
        }

        if(imLeft.channels() == 4)
            cv::cvtColor(imLeft, imLeft, cv::COLOR_RGBA2BGR);
        if(imRight.channels() == 4)
            cv::cvtColor(imRight, imRight, cv::COLOR_RGBA2BGR);

        const double tframe = vTimestamps[ni];

        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        SLAM.TrackStereo(imLeft, imRight, tframe);
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();

        vTimesTrack[ni] = chrono::duration_cast<chrono::duration<float> >(t2 - t1).count();

        // 每帧都存，最后一帧覆盖前面的 -> 最终就是"最后一帧"
        if(visexp::saveCurrentFrame(SLAM, imLeft, strOutDir + "/current_frame.png",
                                    strOutDir + "/current_features.csv"))
            nSaved++;

        // 逐帧快照（VIS_FRAMES_DIR 非空时）：每一帧一个状态切片
        if(!strFramesDir.empty())
            visexp::saveFrameSnapshot(SLAM, imLeft, strFramesDir, ni);

        // 放慢：VIS_PACE_MS>0 时每帧后睡一下，让抓图脚本抓得到中间过程
        visexp::pace();

        if((ni + 1) % 10 == 0 || ni == nImages - 1)
            cout << "    " << (ni + 1) << "/" << nImages
                 << "  tracking=" << vTimesTrack[ni] << "s" << endl;
    }

    cout << "[i] 跟踪结束，存过 " << nSaved << " 张 current_frame.png";
    if(nSkip > 0)
        cout << "（跳过 " << nSkip << " 帧）";
    cout << endl;

    // 地图截图必须在 Shutdown 之前请求，并留时间给 Viewer 渲染线程落盘
    visexp::requestMapShot(SLAM, strOutDir + "/map_view.png");
    cout << "[i] 等 2 秒让 Viewer 落盘 map_view.png ..." << endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 再保持窗口一段时间，方便 capture_pangolin.sh 抓真实 Pangolin 窗口
    visexp::holdWindow();

    SLAM.Shutdown();

    // 双目有尺度、有逐帧位姿 -> 正常写 CameraTrajectory.txt
    visexp::saveAll(SLAM, strOutDir, false);

    const float ttrack = accumulate(vTimesTrack.begin(), vTimesTrack.end(), 0.0f);
    cout << "[i] 总跟踪耗时 " << ttrack << "s（平均 "
         << (ttrack / nImages) << "s/帧）" << endl;
    cout << "[i] 产物：" << endl
         << "      " << strOutDir << "/current_frame.png" << endl
         << "      " << strOutDir << "/current_features.csv" << endl
         << "      " << strOutDir << "/CameraTrajectory.txt" << endl
         << "      " << strOutDir << "/KeyFrameTrajectory.txt" << endl
         << "      " << strOutDir << "/map_points.csv" << endl
         << "      " << strOutDir << "/keyframes.csv" << endl
         << "      " << strOutDir << "/map_view.png" << endl;
    if(!strFramesDir.empty())
        cout << "      " << strFramesDir << "/F###_*（逐帧快照，共 "
             << nImages << " 帧）" << endl;

    return 0;
}