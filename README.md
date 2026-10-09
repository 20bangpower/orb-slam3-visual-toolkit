# orb_slam3 可视化工具链

把 ORB-SLAM3 的建图结果画成对照图的脚本。左边是重建出来的地图面板（地图点、关键帧轨迹、相机位置），
右边是当前帧，拼起来就是 (a) / (a') 的对照版式，用来比对正常数据与被改过的数据在建图和轨迹上的差别。
输出 PNG / PDF，也可以导出逐帧对照图和视频，版式参考论文 Fig.4。

## 用法

默认从 `~/ORB_SLAM3` 找 ORB-SLAM3，从 `~/dataset/slam_stereo_pairs_0_99_20260929` 找数据集，
脚本放在哪个目录都可以：

```bash
cd ~/ORB_SLAM3/outputs/fig4_4seasons

bash fix_crlf.sh           # 从 Windows 传过来的文件先修一下行尾，git clone 的一般不用
bash make_orb_inputs.sh    # 生成相机 yaml 和 TUM 关联文件
bash patch_stereo.sh       # 编译双目可视化例程 stereo_tum_vi_vis
bash run_compare.sh --all  # 跑 SLAM + 拼 Fig.4 对照图
bash start_gui.sh          # 网页界面：点按钮就能出对比图 / 对比视频
# Windows 上直接双击 orb-slam3-visual-toolkit.exe（机器要有 Python 3），起的是同一个界面

bash run_deviation.sh                # 位移偏差 + 建图点云对比
bash run_video.sh --triple --slam    # 逐帧视频（正常 + 两组干扰并排）
bash run_frame_pairs.sh              # 逐帧「原图 vs 被改图」对照图，不用跑 SLAM
bash selftest.sh           # 自检：不用数据集，自己造假数据把整条出图链路跑一遍
```

没有显示器的时候（服务器、ssh）在命令前加 `DISPLAY=`，或者用 `xvfb-run -a` 包一层。

不想敲命令就用网页界面：`bash start_gui.sh`（Windows 上双击 `orb-slam3-visual-toolkit.exe`）起服务，
浏览器自动打开 `http://127.0.0.1:8770`，页面上选数据集目录、点「输出对比图 / 输出对比视频」，
日志实时滚动，跑完直接预览。
只用 Python 标准库，不用装 flask；ssh 场景用 `bash start_gui.sh --no-browser` 配合 `ssh -L 8770:127.0.0.1:8770` 转发端口。

Windows 上直接双击 `orb-slam3-visual-toolkit.exe`（和 `gui_app.py` 放同一个目录）就起同一个界面，机器上要有 Python 3。
从 zip 解压出来如果被系统拦下：SmartScreen 弹窗点「更多信息 -> 仍要运行」，或者右键 exe -> 属性 -> 勾上「解除锁定」。
exe 自己只干两件事 —— 找 `gui_app.py`、找 Python 3，不依赖别的库；没装 Python 3 会提示去官网装（记得勾 Add python.exe to PATH）。
解压到带中文或空格的目录也能用。
Windows 上起界面只要 Python 3，但出图那几步要 bash，所以还是回 Linux/VM 里跑；页面会自己检测 bash 能不能用，不能用会在页眉直接标出来。

路径可以用环境变量改：`ROOT`（默认 `~/ORB_SLAM3`）、`DS`（默认 `~/dataset/slam_stereo_pairs_0_99_20260929`）、
`PY`（默认 `python3`），例如 `DS=/data/xxx ROOT=~/slam bash run_compare.sh --all`。

## 文件说明

| 文件 | 干什么 |
|---|---|
| `run_compare.sh` | 主脚本。跑 SLAM、导出每组结果、拼成 Fig.4 对照图。`--all` 一把梭，也可以 `--slam` / `--render` / `--figures` 分步跑 |
| `compose_compare.py` | 拼版用。把地图面板和当前帧排成 (a)/(a') 版式，加标题和 Current Position 标注 |
| `pangolin_panel2.py` | 画左边那块全局地图面板：地图点、关键帧轨迹、相机视锥，同时输出锚点 json |
| `pick_map_shot.py` | 用真实 Pangolin 窗口截图当面板时，从一堆截图里挑一张清楚的 |
| `capture_pangolin.sh` | 用 xvfb 批量抓真实 Pangolin 窗口截图，`--pangolin-shot` 时才用 |
| `patch_stereo.sh` | 把双目可视化例程加进 ORB-SLAM3，编出 `stereo_tum_vi_vis` |
| `patch_mono.sh` | 单目版本，备用（`run_compare.sh --mono`） |
| `stereo_tum_vi_vis.cc` | 双目例程源码：存当前帧、导轨迹和地图点 csv、开 Viewer |
| `mono_tum_vis.cc` | 单目例程源码，同上 |
| `vis_export.h` | 导出相关的代码，写 map_points.csv / keyframes.csv / 特征点 csv；兼容 GetWorldPos() 返回 cv::Mat 或 Eigen::Vector3f 两种 ORB-SLAM3 分支 |
| `patch_cmake.py` | 编译辅助，自动找 CMake 里的目标并复用它的编译参数 |
| `build_mono_vis.py` | 单目版的编译辅助 |
| `install_dataset.sh` | 数据集 SHA256 校验，顺便生成路径清单 |
| `make_orb_inputs.sh` / `make_orb_inputs.py` | 生成相机参数 yaml 和 TUM 关联文件 |
| `run_frame_pairs.sh` / `export_frame_pairs.py` | 逐帧导出「原图 vs 被改图」对照图、对照表和 PDF |
| `run_video.sh` / `make_fig4_video.py` | 逐帧视频：`--fig4` 论文版式、`--pair` 简版、`--triple` 三组并排 |
| `run_deviation.sh` / `plot_deviation.py` | 位移偏差折线 + 建图点云对比（两个干扰组时 2x2，只有一个时自动 1x2） |
| `make_demo_data.py` | 造一份假的演示数据，没有数据集也能先跑通版式 |
| `gui_app.py` | 网页界面后端：起本地服务、接按钮、跑脚本、把日志和产物回传页面 |
| `start_gui.sh` | 起网页界面（`bash start_gui.sh`） |
| `orb-slam3-visual-toolkit.exe` | Windows 双击即用的启动器：找到 `gui_app.py` 和 Python，起界面并打开浏览器 |
| `launcher.c` | 上面那个 exe 的源码，`gcc -O2 -s -o orb-slam3-visual-toolkit.exe launcher.c` 重新编译 |
| `ls_vis_4seasons.sh` | 自检脚本，逐条检查路径和源码改动在不在 |
| `selftest.sh` / `selftest.py` | 自检：不用数据集和 SLAM，自己造假数据把面板 / 拼图 / 偏差图 / 视频 / 网页界面整条链路跑一遍，最后打印通过数 |
| `dump_api.sh` | 编译报错时用，打印当前 ORB-SLAM3 里相关 API 的真实签名 |
| `fix_crlf.sh` | 修行尾，Windows 传过来的文件跑一下 |
| `preview/` | 几张已经跑出来的示例图，先看看成品长什么样 |
| `README_4seasons.md` | 详细说明：各种参数、产物路径、每种图怎么调 |

## 依赖

- Linux，已编译过的 ORB-SLAM3（默认 `~/ORB_SLAM3`）
- Python 3 + numpy、matplotlib、Pillow
- ffmpeg、xvfb（可选，没有 ffmpeg 就只出 GIF）
- 中文字体（可选，没装的话图里自动用英文）
- Windows 上只跑网页界面的话，装个 Python 3 就够（出图仍然在 Linux 上跑）

```bash
sudo apt-get install -y python3-numpy python3-matplotlib python3-pil ffmpeg xvfb
```

## 数据

- 完整是三组：`<数据集目录>/clean`、`fixed_pixel`、`world_plane`，每组 100 帧、左右目各一张，
  帧号和时间戳的对应表是 `<数据集目录>/comparison_pairs.csv`。
- 组数不写死。只有 `clean` + 一个干扰组时，缺的那组自动跳过、排版降级
  （对照图少一行、视频少一块、偏差图从 2x2 变 1x2），一样能出图。
- 索引 0~7 三组是一样的，只有 8 之后被改过，所以每组都要从第 0 帧开始单独跑一遍。
- 数据集没有 ground truth，偏差图以 clean 组的估计轨迹为基准，表示相对正常结果偏了多少。

## 声明

本仓库代码仅供学习与交流使用，请勿用于商业用途，转载请注明出处。

数据集版权归数据集作者所有，本仓库不包含数据集文件。