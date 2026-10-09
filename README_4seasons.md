# 4Seasons 双目包 -> Fig.4 版式对照图（a / a' 双子图）

一张输出图 = **两个子图并排**：

```
(a) Normal Scenario (no attack)          (a') Fixed-pixel overlay attack
   [ 地图面板 | 当前数据帧 ]                 [ 地图面板 | 当前数据帧 ]
```

- 左块 = `pangolin_panel2.py` 画的全局地图面板（白底、黑点=全部地图点、红点=当前帧视锥内的参考点、
  蓝色三角链=关键帧、绿线=关键帧连线、绿三角=当前相机），配 `--anchor-out` 的 json 画
  **品红 "Current Position" 框 + 标签**；左上角蓝字 `Normal Scenario` / `Attack Scenario`。
- 右块 = 当前数据帧：`FrameDrawer::DrawFrame()` 原样输出（**黑框=匹配到地图点，绿框=未匹配**，
  底部 `SLAM MODE | Maps.. KFs.. MPs.. Matches..` 状态条）。
- 每个子图下面一条居中标题：(a)/(a')、(b)/(b')。

## 一条命令

```bash
cd ~/ORB_SLAM3/outputs/fig4_4seasons
bash run_compare.sh --all
```

## 网页界面（不想敲命令就用这个）

```bash
cd ~/ORB_SLAM3/outputs/fig4_4seasons
bash start_gui.sh
```

Windows 上不想敲命令，就双击同一个目录里的 `orb-slam3-visual-toolkit.exe`（机器要有 Python 3），
它会自己找到 `gui_app.py` 并起服务、打开浏览器。
解压出来如果被系统拦下：SmartScreen 弹窗点「更多信息 -> 仍要运行」，或者右键 exe -> 属性 -> 勾上「解除锁定」。
解压到带中文或空格的目录也能用。

浏览器会自动打开 `http://127.0.0.1:8770`，页面上：

- **指定数据集路径** — 弹窗里像文件管理器一样点进数据集目录（带 `clean/` 的会标注「有 clean」）。
- **输出对比图** — 跑 `run_compare.sh --all`，出 Fig.4 版式对照图。
- **输出对比视频** — 跑 `run_video.sh --triple`，出逐帧对照视频。
- 另外还有 `位移偏差图` / `逐帧对照图` / `一键全部` / `停止`。
- 中间是实时日志，跑完在最下面直接预览图片、播放视频。

两个选项：`跳过 SLAM`（复用已有结果，只重新拼图）、`视频每帧停留 N 秒`。
服务器 / ssh 场景：

```bash
bash start_gui.sh --no-browser        # 服务端不开浏览器
# 本机再转发端口：
ssh -L 8770:127.0.0.1:8770 用户名@服务器地址
# 然后本地浏览器打开 http://127.0.0.1:8770
```

界面只用 Python 标准库（不需要 flask / tkinter），脚本照旧调用现有那几个 `.sh`，
组数还是自适应的 —— 只有两组数据一样能出图。

## 只有两组数据也能跑

三组（`clean` + `fixed_pixel` + `world_plane`）是完整形态，但脚本不写死组数：
数据集里只有 `clean` + 一个干扰组时，缺的那组自动跳过，排版跟着降级，一样能出图。

| 产物 | 三组（完整） | 只有两组 |
|---|---|---|
| 对照图 `run_compare.sh` | `fig4_4seasons_fixed_pixel` / `_world_plane`，再拼一张 `_both`（两行叠一起） | 只出存在那组的对照图，不再拼 `_both` |
| 逐帧视频 `run_video.sh --triple` | 一帧 3 个板块（正常 + 两组干扰），画布 2560 宽 | 一帧 2 个板块（正常 + 那一组干扰），画布 1920 宽 |
| 偏差图 `run_deviation.sh` | 2x2：上排两条偏差折线，下排两张建图对比 | 1x2：(a) 偏差折线 + (b) 建图对比 |
| 逐帧对照图 `run_frame_pairs.sh` | 两个干扰组各导一套逐帧对照页 | 只导存在那组 |

判断某一组在不在，看 `DS/<组>` 或 `RUNS/<组>` 目录有没有；跳过的组会打印 `[!] 数据集里没有 X，跳过这一组`。
`clean` 必须有，否则脚本直接报错退出。想手动限定处理哪几组：

```bash
VIS_GROUPS=clean,fixed_pixel bash run_compare.sh --all
bash run_video.sh --triple --groups fixed_pixel
bash run_deviation.sh --groups fixed_pixel
```

## 首次准备（四步）

```bash
cd ~/ORB_SLAM3/outputs/fig4_4seasons

# 1) 数据集：已解压就只做 SHA256 校验 + 生成绝对路径清单
bash install_dataset.sh

# 2) 生成 ORB-SLAM3 输入（关联文件 + yaml；单目/双目都生成）
bash make_orb_inputs.sh

# 3) 编译带导出的双目入口 -> ~/ORB_SLAM3/Examples/Stereo/stereo_tum_vi_vis
bash patch_stereo.sh
#    默认右图用 FrameDrawer::DrawFrame()（带状态条）。
#    万一你的分支没有 DrawFrame：bash patch_stereo.sh --plain 重编（改成自己画框）。
#    GetWorldPos()/GetCameraCenter() 返回 cv::Mat 还是 Eigen::Vector3f 都能编（自动识别，不用手改）。

# 4) 跑 SLAM + 画面板 + 拼图
bash run_compare.sh --all
```

自检：`bash ls_vis_4seasons.sh`（查路径和源码改动在不在）；
`bash selftest.sh` 会自己造一份假数据，把面板 / 拼图 / 偏差图 / 视频 / 网页界面整条链路跑一遍，
不需要数据集也不用跑 SLAM，跑完打印 x/y 通过。

只重跑某一环：

```bash
bash run_compare.sh --slam      # 只跑 SLAM
bash run_compare.sh --render    # 不跑 SLAM，只画左图面板（复用已有导出）
bash run_compare.sh --figures   # 只拼图
bash run_compare.sh --mono --all            # 退回单目（mono_tum_vis）
bash run_compare.sh --pangolin-shot --all   # 左图改用真实 Pangolin 窗口截图（含左侧菜单栏）
```

## 蓝色三角太大/太密怎么调

100 帧的轨迹很短、关键帧挤在一起，符号画大了会连成一条实心带。`run_compare.sh` 的默认值已经调小：

| 变量 | 默认 | 作用 |
|---|---|---|
| `GLYPH_EVERY` | 2 | 每 N 个关键帧画一个三角（太密就调大，如 3、4） |
| `GLYPH_DEPTH` | 0.012 | 三角长（占面板比例，越小越短） |
| `GLYPH_HALFW` | 0.007 | 三角半宽（占面板比例） |
| `CAM_SCALE` | 1.4 | 当前相机（绿/蓝大三角）放大倍数 |

例：还想再小一点

```bash
GLYPH_EVERY=3 GLYPH_DEPTH=0.008 GLYPH_HALFW=0.005 CAM_SCALE=1.1 \
  bash run_compare.sh --render --figures
```

`pangolin_panel2.py` 自己也会提示：关键帧中位间距 < 符号长时会打印
`[!] 关键帧很密...建议加 --glyph-every N`。

## 路径约定（全部可用环境变量覆盖）

| 变量 | 默认值 |
|---|---|
| `ROOT` | `~/ORB_SLAM3` |
| `DS` | `~/dataset/slam_stereo_pairs_0_99_20260929`（也自动找 `~/dataset/4Seasons/<NAME>`、`~/<NAME>`） |
| 词袋 | `$ROOT/Vocabulary/ORBvoc.txt` |
| 相机参数 | `$DS/orb_inputs/4seasons_orb3_stereo.yaml`（单目：`..._mono.yaml`） |
| 关联文件 | `$DS/orb_inputs/<group>_stereo.txt`（单目：`<group>_mono.txt`） |
| 面板脚本 | `$HERE/pangolin_panel2.py`；找不到再退回 `$ROOT/outputs/fig4_normal/pangolin_panel2.py`（都可用 `PANEL_PY=` 覆盖）|
| 抓图脚本 | `$ROOT/outputs/fig4_normal/capture_pangolin.sh`（`--pangolin-shot` 时才用） |
| 每组导出 | `$ROOT/runs/4seasons/<group>/` |
| 成图 | `$ROOT/outputs/fig4_4seasons/figures/` |

## 产物

```
$ROOT/runs/4seasons/<group>/
    pangolin_panel.png          左图（地图面板）
    pangolin_panel.anchor.json  当前相机锚点（给 compose 画 Current Position 框）
    pangolin_map.png            左图的固定名（= pangolin_panel.png 的副本）
    current_frame.png           右图（FrameDrawer::DrawFrame()）
    current_features.csv        u,v,used,vo
    CameraTrajectory.txt        逐帧位姿（双目有；单目没有）
    KeyFrameTrajectory.txt      关键帧位姿
    map_points.csv              地图点（黑点来源）
    keyframes.csv               关键帧相机中心
    map_view.png                SLAM 进程内 glReadPixels 存的地图视图
    run.log                     SLAM 的 stdout
    shots/                      --pangolin-shot 时的真实窗口截图

$ROOT/outputs/fig4_4seasons/figures/
    fig4_4seasons_fixed_pixel.png/.pdf   (a) clean  vs  (a') fixed_pixel
    fig4_4seasons_world_plane.png/.pdf   (a) clean  vs  (a') world_plane   # 仅当该组存在
    fig4_4seasons_both.png/.pdf          (a)(a') / (b)(b') 两行叠一张        # 仅两组都在时
```

> 只有两组数据（没有 `world_plane`）时，`_world_plane` / `_both` 不会生成，只有 `_fixed_pixel`。

## 逐帧导出：每张被改的原图 vs 原图（便于分析）

上面 `run_compare.sh` 出的是"整段轨迹"级别的一张对照图；如果要**逐帧**看"这张原图被贴了什么"，
用下面这个（不跑 SLAM，只读数据集，秒级完成）：

```bash
cd ~/ORB_SLAM3/outputs/fig4_4seasons
bash run_frame_pairs.sh                 # 两个干扰组 x 左右目 x 全部 92 个被改帧
bash run_frame_pairs.sh --cams cam0     # 只看左目（快一半）
bash run_frame_pairs.sh --diff-col      # 对照表多一列"差异放大图"
bash run_frame_pairs.sh --max 10        # 先小批量试
```

产物在 `~/ORB_SLAM3/outputs/fig4_4seasons/frame_pairs/`：

| 路径 | 内容 |
|---|---|
| `index.csv` | 逐帧统计：改动像素数 / 占比 / bbox / 平均差 |
| `pairs/<组>/F###_<cam>.png` | 单帧对照：左=clean 原图，右=干扰图，红框=改动区 |
| `pairs/<组>/F###_<cam>_modified.png` | 只有被改后的整图 |
| `frames/<组>/F###_<cam>_modified.png` | 被改后的整图（按组归档） |
| `sheets/<组>_<cam>_sheetNN.png` | 一页 N 帧的对照表（默认 8 帧/页） |
| `sheets/<组>_<cam>_all.pdf` | 全部页合成一本 PDF |

> 改动只在 **索引 8—99**（92 帧/组）；索引 0—7 三组逐字节相同，所以默认只导被改的帧，
> 想看全部加 `--all-frames`。数据量：92 帧 x 2 目 x 2 组 ≈ 368 张单帧图 + 约 24 页/组·目。
## 逐帧对照视频：每帧停留 1~2 s（便于逐帧分析）

`run_frame_pairs.sh` 出的是一张张对照表；要**像放幻灯片一样看**，用这个：

```bash
cd ~/ORB_SLAM3/outputs/fig4_4seasons

# 默认就是论文 Fig.4 版式：
#   左边一整块 (a) Normal Scenario     = [轨迹地图面板 | 当前帧]
#   右边一整块 (a') <攻击名> Scenario  = [轨迹地图面板 | 当前帧]
#   逐帧切换成视频，每帧停留 --hold 秒
bash run_video.sh

# 没有逐帧快照时会自动先跑一遍 SLAM（headless，不需要 X）；
# 想强制重跑快照：
bash run_video.sh --slam

# 备用：不跑 SLAM，只用数据集图片做对照
bash run_video.sh --pair
```

产物在 `~/ORB_SLAM3/outputs/fig4_4seasons/video/`：

| 路径 | 内容 |
|---|---|
| `<组>.mp4` | 每个干扰组一个视频，每帧停留 `--hold` 秒（默认 1.5） |
| `<组>.gif` | 没装 ffmpeg 时的兜底动图（`--gif` 可强制两个都出） |
| `slides/<组>/F###.png` | 标准化后的单帧画面（1920 宽，带页眉页脚），可直接分发 |
| `all.mp4` / `all.gif` | `--combined` 时：两个干扰组所有帧合成的一条视频 |
| `index_video.csv` | 帧号 / 时间戳 / 停留秒数 / 画面文件 |

常用旋钮（原样跟在后面）：

```bash
--groups fixed_pixel        # 只做一个干扰组
--cams cam0                 # 只看左目（fig4 模式只有左目）
--max 10                    # 先做 10 帧试试
--hold 2                    # 每帧停 2 秒
--frames 8,9,10             # 只做指定帧号
--all-frames                # 连没改动的帧也做（默认只做被改的 92 帧）
--combined                  # 每组一个视频之外，再把所有组合成一条 all.mp4
--triple                    # 一帧里放 3 个板块：normal + fixed_pixel + world_plane
--diff-col                  # pair 模式多一列"差异放大图"
--width 1920 --height 0     # 高度 0 = 自动裁到内容（画面最大）
--jobs 4                    # 并行渲染线程数
```

> **帧数是"全部"**：默认把 92 个被改帧全部做进来，一帧不落；跑完会打印
> `画面 N/92 张`，少于 92 会给出警告。想要索引 0—7 那 8 帧没改动的也放进去，加 `--all-frames`。

> - **每帧都是"原图 vs 被改图"的对照**：clean 在左、干扰图在右，红框标出改动区；
>   fig4 模式则还原论文那种 `(a) Normal Scenario | (a') 攻击` 双子图（左=地图面板，右=当前帧）。
> - **`--triple` = 三板块版**：同一帧里横着排 `(a) Normal | (a') fixed_pixel | (a'') world_plane`，
>   每个板块仍是 [地图面板 | 当前帧]，只出一条 `triple.mp4`（方便横向对照两种干扰）。
>   建议配 `--width 2560`（不指定时 triple 会自动用 2560），否则每个板块会偏小。
> - fig4 模式需要 SLAM 逐帧快照（`VIS_FRAMES_DIR`）：`run_video.sh --fig4 --slam` 会自动
>   重编前先跑 `patch_stereo.sh`，并用 `VIS_NO_VIEWER=1` 跑，**不需要 X 也能出快照**。
> - 视频长度 ≈ 92 帧 x hold。hold=1.5 s 时，每个干扰组的视频约 2 分 18 秒。
> - 装 ffmpeg：`sudo apt-get install -y ffmpeg`（没有 ffmpeg 就只出 GIF）。
## 位移偏差 + 建图对比（辅助分析图）

跑完 SLAM 之后，一张图看清"被攻击后轨迹偏了多少、地图建得怎么样"：

```bash
cd ~/ORB_SLAM3/outputs/fig4_4seasons
bash run_deviation.sh
```

产物：`figures/fig4_4seasons_deviation.png` / `.pdf`，**2x2 四个子图**：

| 子图 | 内容 |
|---|---|
| (a) | 位移偏差折线：固定像素贴图 vs 正常数据集 |
| (b) | 位移偏差折线：世界平面投影贴图 vs 正常数据集 |
| (c) | 建图点云对比：固定像素贴图 vs 正常数据集 |
| (d) | 建图点云对比：世界平面投影贴图 vs 正常数据集 |

(a)(b) 里**黑粗线 = 正常数据集 clean 基线**（偏差恒为 0），彩线 = 干扰数据集逐帧偏差（淡色填充），
并标注“最大 X m / 终点 X m”。

(c)(d) 是 X-Z 俯视的建图点云（等比正方形，X/Z 同尺度，**全部地图点**）：
**蓝点 = 正常数据集**构建的地图点，**红点 = 干扰数据集**构建的地图点；
空白角自动放一个**密集区放大子图**（取点最少的那个角，同样空时偏爱右下），
主图上用方框标出子图看的范围。子图视窗半宽 = `k x 中位半径`（默认 k=2.6），
中位半径对远点尾巴免疫，所以缩放倍率能自适应地图尺度；地图本身铺得很开
（放大不明显）时就不加子图，免得白占地方。
图上不出现任何“剔除/点数”文字，相关数字只在终端打印。
想自己调：`VIS_ZOOM_K=4 bash run_deviation.sh`（看得更远）/ `VIS_ZOOM_K=1.5`（放得更大）/ `0`（不要子图）。


终端会打印每组的**帧数 / 对齐帧数 / 未对齐帧数 / 轨迹长度 / 平均·最大·终点偏差 / 终点占轨迹百分比 / 地图点数**，
并在开头打印 `运行目录`。对齐帧数 = 与 clean 时间戳对上的帧数；它等于帧数就说明每一帧都真实参与了计算，
不存在“对不上就拿索引凑”的情况。

> ⚠ 本数据集**没有 ground truth**，图里以 clean 组的**估计轨迹**为基准，
> 偏差 = 攻击组估计位姿 − clean 组估计位姿（按时间戳同帧对齐）。
> 所以读数表示"相对正常估计偏移了多少"，不是绝对定位误差。

> 数据来源：(a)(b) 需要各组 `CameraTrajectory.txt`（跑过 `bash run_compare.sh --all`）；
> (c)(d) 需要各组 `map_points.csv`（同一次运行导出）。
## 可调旋钮

| 变量 | 默认 | 作用 |
|---|---|---|
| `GLYPH_EVERY` / `GLYPH_DEPTH` / `GLYPH_HALFW` / `CAM_SCALE` | 见上 | 蓝色三角大小/密度 |
| `VIS_GROUPS` | clean,fixed_pixel,world_plane | 跑哪些组 |
| `VIS_ZOOM_K` | 2.6 | (b) 子图放大视窗半宽 = k x 中位半径（k 越大看得越远，0 = 不加子图） |
| `FIGURES` | fixed_pixel,world_plane | 拼哪些对照图 |
| `ANN_NORMAL` / `ANN_ATTACK` | Normal Scenario / Attack Scenario | 面板左上角蓝字 |
| `W_FIXED` / `W_WORLD` | Fixed-pixel overlay attack / World-plane projection attack | 子图标题里的干扰名 |
| `ANN_INSET` | 0.035（`--pangolin-shot` 时 0.14） | 蓝字左缩进（让开菜单栏） |
| `PANEL_PY` | `$ROOT/outputs/fig4_normal/pangolin_panel2.py` | 面板脚本 |
| `BIN` | `$ROOT/Examples/Stereo/stereo_tum_vi_vis` | 可执行文件 |
| （`--pangolin-shot`）`VIS_PACE_MS`/`VIS_HOLD_SEC`/`SECS`/`INTERVAL`/`SIZE` | 200/30/150/4/2400x2000x24 | 抓真实窗口截图时的节奏 |

## 文件清单

| 文件 | 作用 |
|---|---|
| `install_dataset.sh` | 校验数据集（SHA256）+ 生成绝对路径清单 |
| `make_orb_inputs.py` / `.sh` | 生成 `<group>_mono.txt` / `<group>_stereo.txt` / 两个 yaml |
| `vis_export.h` | 导出逻辑：current_frame.png / current_features.csv / 轨迹 / map_points.csv / keyframes.csv / map_view.png；`pace()`/`holdWindow()` |
| `stereo_tum_vi_vis.cc` | **双目入口**（3 列关联 + `TrackStereo` + 上面那套导出） |
| `mono_tum_vis.cc` | 单目入口（`--mono` 时才用） |
| `patch_stereo.sh` | 拷文件 + 复用 `stereo_tum_vi` 的编译参数编出双目入口（`--plain` 可关掉 DrawFrame） |
| `patch_mono.sh` | 同上，单目版 |
| `build_mono_vis.py` | 复用已有目标的 `flags.make`/`link.txt` 直接编译，**不需要 CMake** |
| `compose_compare.py` | 拼版：一行放 N 个子图 `[地图|数据帧]`，子图标题 + 左上角标注 + Current Position 框 |
| `run_compare.sh` | 编排全程 |
| `pick_map_shot.py` | `--pangolin-shot` 时从 `shots/` 挑"地图最全"那张 |
| `capture_pangolin.sh` | `--pangolin-shot` 时抓真实 Pangolin 窗口截图 |
| `ls_vis_4seasons.sh` | 全链路自检 |
| `dump_api.sh` | 打印你分支上 `GetCurrentFeatures/RequestSaveMapImage` 的真实签名 |
| `export_frame_pairs.py` / `run_frame_pairs.sh` | 逐帧 clean vs 干扰 对照图 + 对照表页 + PDF（不跑 SLAM） |
| `make_fig4_video.py` / `run_video.sh` | 逐帧对照 -> 视频（mp4/gif），每帧停留 N 秒 |
| `plot_deviation.py` / `run_deviation.sh` | 位移偏差折线 + 建图对比两面板图（辅助分析） |
| `gui_app.py` | 网页界面后端：起本地服务、接按钮、跑脚本、把日志和产物回传页面 |
| `start_gui.sh` | 起网页界面（`bash start_gui.sh`） |
| `orb-slam3-visual-toolkit.exe` | Windows 双击启动器：找到 `gui_app.py` 和 Python 3，起界面并打开浏览器（源码 `launcher.c`） |
| `selftest.sh` / `selftest.py` | 自检：造假数据把面板 / 拼图 / 偏差图 / 视频 / 网页界面整条链路跑一遍 |

## 数据集要点（照抄包内 README，别踩坑）

- 三组：`clean`（正常）、`fixed_pixel`（索引 8—99 固定像素贴图）、`world_plane`（世界固定平面投影）。
- **索引 0—7 三组完全相同；每组必须从索引 0 独立初始化跑，不能从第 8 帧起跑。**
- `cam0` = 左目，`cam1` = 右目；800x400 PNG；相邻约 33 ms（100 帧 ≈ 3.3 s）。
- 图像**已经去畸变 + 双目校正**，`k1..k3/p1/p2` 必须保持 0，不要二次校正。
- 标定：`fx=fy=501.4757919305817`、`cx=421.7953735163109`、`cy=167.65799492501083`、
  `bf=150.69155075856955` → 基线 `b = bf/fx ≈ 0.3005 m`。
- 包里 12 条 TUM 轨迹**全是估计轨迹，没有 ground truth**。
- 不同批次的 clean 不能混用。

## 已知风险 / 注意

1. **双目优先**：数据集作者提示"先取左目当单目跑"，但这段是纯前向 + 只有 100 帧，单目容易漂；
   双目有 `bf` 和校正好的右目，更稳。要退回单目加 `--mono`。
2. **红点仍是近似**：`--ref-mode frustum` 取"当前关键帧视锥内的地图点"当参考点，
   不是真正的 `MapDrawer::mReferenceMapPoints`。要精确红点得让 SLAM 侧额外导出 `ref_map_points.csv`。
3. **Current Position 框**依赖 `pangolin_panel.anchor.json`；`--pangolin-shot` 模式下没有这个文件，
   所以那种模式下不会画框。
4. **100 帧的图会比较"薄"**：地图点和关键帧远少于 TUM fr1/xyz（792 帧）。
5. **没有 X 显示也没关系**：面板模式用 `xvfb-run` 起虚拟屏（Viewer 要开）；
   `--pangolin-shot` 模式由抓图脚本自己起带 GLX 的 Xvfb。