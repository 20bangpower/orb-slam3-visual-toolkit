#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gui_app.py -- 给「ORB-SLAM3 可视化对照工具链」套一个简易网页界面。

启动（在 ~/ORB_SLAM3/outputs/fig4_4seasons/ 下）：
    python3 gui_app.py                     # 起服务并自动打开浏览器 (127.0.0.1:8770)
    python3 gui_app.py --port 9000 --no-browser
    python3 gui_app.py --ds ~/dataset/slam_stereo_pairs_0_99_20260929
    python3 gui_app.py --root ~/ORB_SLAM3 --host 0.0.0.0    # 别的机器访问（ssh -L 转发更安全）

界面上的按钮：
    [指定数据集路径]   浏览服务器上的文件夹，选中数据集根目录
    [输出对比图]       bash run_compare.sh --all
    [输出对比视频]     bash run_video.sh --triple
    另外还有 [位移偏差图] / [逐帧对照图] / [一键全部] / [停止]

跑出来的图和视频会直接在页面下方预览，命令输出实时滚动。
只用 Python 标准库，不需要 flask、tkinter。
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

HERE = os.path.dirname(os.path.abspath(__file__))
DS_NAME = "slam_stereo_pairs_0_99_20260929"
ALL_GROUPS = ["clean", "fixed_pixel", "world_plane"]
GROUP_CN = {"clean": "正常数据集",
            "fixed_pixel": "固定像素贴图",
            "world_plane": "世界平面投影贴图"}
BASH = shutil.which("bash") or "/bin/bash"
IMG_EXT = (".png", ".jpg", ".jpeg", ".gif", ".svg")
VID_EXT = (".mp4", ".webm", ".gif")


def expand(p):
    return os.path.abspath(os.path.expanduser(str(p)))


def first_dir(cands):
    for c in cands:
        if c and os.path.isdir(os.path.expanduser(c)):
            return os.path.abspath(os.path.expanduser(c))
    return ""


def find_scripts(root):
    """脚本目录：gui_app.py 自己所在目录优先，其次 ROOT/outputs/fig4_4seasons。"""
    for c in (HERE, os.path.join(root, "outputs", "fig4_4seasons")):
        if os.path.isfile(os.path.join(c, "run_compare.sh")):
            return c
    return HERE


def list_files(path, exts, limit=60):
    out = []
    if not os.path.isdir(path):
        return out
    items = []
    for n in os.listdir(path):
        fp = os.path.join(path, n)
        if os.path.isfile(fp) and n.lower().endswith(exts):
            try:
                items.append((os.path.getmtime(fp), n, fp, os.path.getsize(fp)))
            except OSError:
                pass
    items.sort(reverse=True)
    for mtime, n, fp, size in items[:limit]:
        out.append({"name": n, "path": fp, "size": size, "mtime": int(mtime)})
    return out


class Config(object):
    def __init__(self, args):
        self.lock = threading.Lock()
        env = os.environ
        self.root = expand(args.root or env.get("ROOT") or "~/ORB_SLAM3")
        self.ds = expand(args.ds) if args.ds else first_dir(
            ["~/dataset/4Seasons/" + DS_NAME, "~/dataset/" + DS_NAME, "~/" + DS_NAME])
        self.py = args.py or env.get("PY") or sys.executable or "python3"
        self.hold = float(args.hold)
        self.skip_slam = False
        self.groups = [g for g in (args.groups or ["fixed_pixel", "world_plane"])
                       if g in ALL_GROUPS and g != "clean"]
        self.scripts = find_scripts(self.root)

    def paths(self):
        r = self.root
        return {"root": r, "ds": self.ds,
                "runs": os.path.join(r, "runs", "4seasons"),
                "figures": os.path.join(r, "outputs", "fig4_4seasons", "figures"),
                "videos": os.path.join(r, "outputs", "fig4_4seasons", "video"),
                "frame_pairs": os.path.join(r, "outputs", "fig4_4seasons", "frame_pairs"),
                "scripts": self.scripts}

    def update(self, d):
        with self.lock:
            if d.get("root"):
                self.root = expand(d["root"])
                self.scripts = find_scripts(self.root)
            if d.get("ds") is not None:
                self.ds = expand(d["ds"]) if str(d["ds"]).strip() else ""
            if d.get("py"):
                self.py = str(d["py"])
            if "hold" in d:
                try:
                    self.hold = float(d["hold"])
                except (TypeError, ValueError):
                    pass
            if "skip_slam" in d:
                self.skip_slam = bool(d["skip_slam"])
            if "groups" in d:
                self.groups = [g for g in (d["groups"] or [])
                               if g in ALL_GROUPS and g != "clean"]

    def state(self):
        p = self.paths()
        with self.lock:
            groups, skip, ds = list(self.groups), self.skip_slam, self.ds
        avail = {}
        for g in ALL_GROUPS:
            avail[g] = bool(ds and (os.path.isdir(os.path.join(ds, g))
                                    or os.path.isdir(os.path.join(p["runs"], g))))
        return {"root": self.root, "ds": ds, "py": self.py, "hold": self.hold,
                "skip_slam": skip, "groups": groups, "paths": p,
                "available": avail, "group_cn": GROUP_CN, "all_groups": ALL_GROUPS,
                "has_scripts": all(os.path.isfile(os.path.join(self.scripts, n))
                                   for n in ("run_compare.sh", "run_deviation.sh",
                                             "run_video.sh")),
                "has_ds_clean": bool(ds and os.path.isdir(os.path.join(ds, "clean")))}


class Job(object):
    def __init__(self, key, steps, env, cwd):
        self.key, self.steps, self.env, self.cwd = key, steps, env, cwd
        self.title = " + ".join(t for t, _ in steps)
        self.lock = threading.Lock()
        self.buf, self.done, self.rc, self.proc = [], False, None, None
        self.t0, self.t1 = time.time(), None

    def push(self, text):
        with self.lock:
            self.buf.append(text)
            if len(self.buf) > 40000:
                del self.buf[:10000]

    def snapshot(self, off):
        with self.lock:
            total = len(self.buf)
            try:
                off = max(0, min(int(off), total))
            except (TypeError, ValueError):
                off = 0
            return {"key": self.key, "title": self.title, "text": "".join(self.buf[off:]),
                    "offset": total, "done": self.done, "rc": self.rc,
                    "elapsed": round((self.t1 or time.time()) - self.t0, 1)}

    def kill(self):
        with self.lock:
            proc = self.proc
        if proc is not None and proc.poll() is None:
            try:
                proc.terminate()
            except OSError:
                pass
            for _ in range(10):
                if proc.poll() is not None:
                    break
                time.sleep(0.2)
            if proc.poll() is None:
                try:
                    proc.kill()
                except OSError:
                    pass


JOBS, JOBS_LOCK, CURRENT = {}, threading.Lock(), {"key": ""}


def build_steps(cfg, task, groups, skip_slam, hold):
    sc = cfg.scripts
    gl = ["--groups", ",".join(groups)] if groups else []
    steps = {
        "figures": ("输出对比图",
                    [BASH, os.path.join(sc, "run_compare.sh")]
                    + (["--render", "--figures"] if skip_slam else ["--all"])),
        "deviation": ("位移偏差图", [BASH, os.path.join(sc, "run_deviation.sh")] + gl),
        "video": ("输出对比视频",
                  [BASH, os.path.join(sc, "run_video.sh"), "--triple"]
                  + gl + ["--hold", str(hold)]),
        "frame_pairs": ("逐帧对照图",
                        [BASH, os.path.join(sc, "run_frame_pairs.sh")] + gl),
    }
    if task == "all":
        return [steps["figures"], steps["deviation"], steps["video"]]
    if task not in steps:
        raise ValueError("未知任务: " + str(task))
    return [steps[task]]


def run_job(job):
    for title, argv in job.steps:
        job.push("\n===== " + title + " =====\n[cmd] " + " ".join(argv) + "\n")
        try:
            proc = subprocess.Popen(argv, cwd=job.cwd, env=job.env,
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    bufsize=1, universal_newlines=True,
                                    encoding="utf-8", errors="replace")
        except OSError as exc:
            job.push("[x] 启动失败: " + str(exc) + "\n")
            job.rc = -1
            break
        job.proc = proc
        try:
            if proc.stdout is not None:
                for line in iter(proc.stdout.readline, ""):
                    job.push(line)
                proc.stdout.close()
            job.rc = proc.wait()
        except Exception as exc:                 # 读输出出意外也不能把任务卡死
            job.push("[x] 读取命令输出出错: " + str(exc) + "\n")
            try:
                proc.kill()
            except OSError:
                pass
            job.rc = proc.wait()
        job.push("[exit] " + str(job.rc) + "\n")
        if job.rc != 0:
            job.push("[!] 这一步没成功，后面的步骤先停下。\n")
            break
    job.proc, job.done, job.t1 = None, True, time.time()


def start_job(cfg, task, groups, skip_slam, hold):
    if not os.path.isfile(os.path.join(cfg.scripts, "run_compare.sh")):
        raise ValueError("找不到脚本目录（run_compare.sh），用 --root 指定 ORB_SLAM3")
    steps = build_steps(cfg, task, groups, skip_slam, hold)
    env = os.environ.copy()
    env["ROOT"], env["PY"], env["PYTHONUNBUFFERED"] = cfg.root, cfg.py, "1"
    if cfg.ds:
        env["DS"] = cfg.ds
    sel = ",".join(groups)
    env["VIS_GROUPS"] = "clean" + (("," + sel) if sel else "")
    env["VID_GROUPS"] = sel
    env["FIGURES"] = sel
    job = Job(task, steps, env, cfg.scripts)
    job.push("[i] 数据集 : " + (cfg.ds or "（未指定）") + "\n")
    job.push("[i] 根目录 : " + cfg.root + "\n")
    job.push("[i] 脚本   : " + cfg.scripts + "\n")
    with JOBS_LOCK:
        JOBS[task] = job
        CURRENT["key"] = task
    th = threading.Thread(target=run_job, args=(job,))
    th.daemon = True
    th.start()
    return job


PAGE = r"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ORB-SLAM3 可视化对照工具链</title>
<style>
:root{--bg:#f5f6f8;--card:#fff;--line:#e2e5ea;--txt:#1c2430;--mut:#6b7684;
--blue:#1f6feb;--green:#1a7f37;--red:#c0392b}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--txt);
font:14px/1.55 "Noto Sans CJK SC","WenQuanYi Zen Hei","Microsoft YaHei",sans-serif}
header{background:#12233b;color:#fff;padding:13px 20px;display:flex;align-items:center;gap:12px}
header h1{font-size:16px;margin:0;font-weight:600}
.wrap{max-width:1180px;margin:0 auto;padding:16px;display:grid;gap:14px}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:14px 16px}
.card h2{font-size:13px;margin:0 0 10px;color:var(--mut);font-weight:600;letter-spacing:.04em}
.row{display:flex;gap:8px;align-items:center;margin:7px 0;flex-wrap:wrap}
label.k{width:150px;color:var(--mut);flex:none}
input[type=text]{flex:1;min-width:200px;padding:7px 9px;border:1px solid var(--line);
border-radius:7px;font:inherit;background:#fbfcfe;color:var(--txt)}
input.num{flex:none;width:56px}
button{font:inherit;padding:7px 13px;border-radius:7px;border:1px solid var(--line);
background:#fff;cursor:pointer}
button:hover{background:#f0f3f7}
button.p{background:var(--blue);border-color:var(--blue);color:#fff;font-weight:600}
button.g{background:var(--green);border-color:var(--green);color:#fff;font-weight:600}
button.d{color:var(--red);border-color:#eccfcb}
button:disabled{opacity:.45;cursor:not-allowed}
.chk{display:inline-flex;gap:6px;align-items:center;margin-right:14px}
.tag{font-size:12px;padding:1px 7px;border-radius:99px;background:#eef2f7;color:var(--mut)}
.tag.miss{background:#fdecea;color:var(--red)}
.tag.ok{background:#e7f6ec;color:var(--green)}
.mut{color:var(--mut);font-size:12px}
pre.log{margin:0;background:#0e1726;color:#d7e3f4;padding:12px;border-radius:8px;
max-height:340px;overflow:auto;font:12px/1.5 Consolas,Menlo,monospace;white-space:pre-wrap}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(240px,1fr));gap:12px}
.item{border:1px solid var(--line);border-radius:9px;overflow:hidden;background:#fbfcfe}
.item img,.item video{width:100%;display:block;background:#fff}
.item .cap{padding:6px 9px;font-size:12px;color:var(--mut);display:flex;gap:8px;align-items:center}
.item .cap a{color:var(--blue);text-decoration:none}
.pdf{padding:26px;text-align:center;color:#8a94a3;background:#fff}
.bar{height:6px;background:#e8ecf2;border-radius:99px;overflow:hidden;margin-bottom:8px}
.bar>i{display:block;height:100%;width:35%;background:var(--blue);animation:sl 1.1s infinite}
@keyframes sl{0%{margin-left:-35%}100%{margin-left:100%}}
#modal{position:fixed;inset:0;background:rgba(12,20,33,.5);display:none;
align-items:center;justify-content:center}
#modal.on{display:flex}
.dlg{background:#fff;border-radius:10px;width:min(720px,92vw);max-height:80vh;
display:flex;flex-direction:column}
.dlg .hd{padding:12px 16px;border-bottom:1px solid var(--line);font-weight:600}
.dlg .bd{padding:10px 16px;overflow:auto}
.dlg .ft{padding:12px 16px;border-top:1px solid var(--line);display:flex;gap:8px;
justify-content:flex-end}
.dlist{display:grid;gap:4px;margin-top:6px}
.dlist button{text-align:left}
</style>
</head>
<body>
<header>
  <h1>ORB-SLAM3 可视化对照工具链</h1>
  <span class="tag" id="chip">加载中</span>
  <span style="flex:1"></span>
  <span class="mut" id="scripts" style="color:#9fb0c6"></span>
</header>
<div class="wrap">

  <div class="card">
    <h2>1 数据集与路径</h2>
    <div class="row">
      <label class="k">数据集目录</label>
      <input type="text" id="ds" placeholder="~/dataset/slam_stereo_pairs_0_99_20260929">
      <button class="p" id="pickDs">指定数据集路径</button>
    </div>
    <div class="row">
      <label class="k">ORB_SLAM3 根目录</label>
      <input type="text" id="root">
      <button id="pickRoot">浏览</button>
      <button id="apply">应用</button>
    </div>
    <div class="mut" id="paths"></div>
  </div>

  <div class="card">
    <h2>2 分组与选项</h2>
    <div class="row" id="groups"></div>
    <div class="row">
      <label class="chk"><input type="checkbox" id="skipSlam"> 跳过 SLAM（复用已有结果，只重新拼图）</label>
      <label class="chk">视频每帧停留 <input type="text" id="hold" class="num"> 秒</label>
    </div>
  </div>

  <div class="card">
    <h2>3 操作</h2>
    <div class="row">
      <button class="g" id="runFig">输出对比图</button>
      <button class="g" id="runVideo">输出对比视频</button>
      <button id="runDev">位移偏差图</button>
      <button id="runPairs">逐帧对照图</button>
      <button class="p" id="runAll">一键全部</button>
      <button class="d" id="stopBtn" disabled>停止</button>
    </div>
    <div class="mut" id="hint">第一次用先点「指定数据集路径」。</div>
  </div>

  <div class="card">
    <h2>4 运行日志</h2>
    <div class="bar" id="bar" style="display:none"><i></i></div>
    <pre class="log" id="log">（还没有开始）</pre>
    <div class="row"><button id="clearLog">清空</button><span class="mut" id="jobInfo"></span></div>
  </div>

  <div class="card">
    <h2>5 结果预览</h2>
    <div class="grid" id="grid"></div>
    <div class="mut" id="nores">还没有产物，先点上面的按钮。</div>
  </div>

</div>

<div id="modal"><div class="dlg">
  <div class="hd">选择文件夹</div>
  <div class="bd">
    <div class="row"><input type="text" id="curPath"><button id="goPath">转到</button></div>
    <div class="dlist" id="dlist"></div>
  </div>
  <div class="ft">
    <button id="cancelPick">取消</button>
    <button class="p" id="okPick">选这个目录</button>
  </div>
</div></div>

<script>
var S = null, jobKey = "", jobOff = 0, tm = null, curDir = "", pickMode = "ds";

function $(id) { return document.getElementById(id); }
function esc(s) {
  return String(s == null ? "" : s).replace(/[&<>"']/g, function (c) {
    return { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c];
  });
}
function api(p, o) {
  return fetch(p, o).then(function (r) {
    if (!r.ok) { return r.text().then(function (t) { throw new Error(t || r.statusText); }); }
    return r.json();
  });
}
function kb(n) {
  return n > 1048576 ? (n / 1048576).toFixed(1) + " MB"
                     : Math.max(1, Math.round(n / 1024)) + " KB";
}

function render() {
  $("ds").value = S.ds || "";
  $("root").value = S.root || "";
  $("hold").value = S.hold;
  $("skipSlam").checked = !!S.skip_slam;
  $("scripts").textContent = "脚本目录 " + S.paths.scripts;
  $("paths").innerHTML = "结果图 " + esc(S.paths.figures) + "<br>视频 " + esc(S.paths.videos);
  var h = "", i, g, ok;
  for (i = 0; i < S.all_groups.length; i++) {
    g = S.all_groups[i];
    ok = !!S.available[g];
    if (g === "clean") {
      h += '<label class="chk"><input type="checkbox" checked disabled> ' +
           esc(S.group_cn[g]) + ' <span class="tag' + (ok ? ' ok' : ' miss') + '">' +
           (ok ? '必备' : '缺失') + '</span></label>';
      continue;
    }
    h += '<label class="chk"><input type="checkbox" class="gp" value="' + esc(g) + '"' +
         (ok ? "" : " disabled") + ((ok && S.groups.indexOf(g) >= 0) ? " checked" : "") +
         '> ' + esc(S.group_cn[g]) + ' <span class="tag' + (ok ? ' ok' : ' miss') + '">' +
         (ok ? '可用' : '数据集里没有') + '</span></label>';
  }
  $("groups").innerHTML = h;
  var c = $("chip");
  if (!S.has_scripts) { c.textContent = "找不到脚本"; c.className = "tag miss"; }
  else if (!S.has_ds_clean) { c.textContent = "还没指定数据集"; c.className = "tag miss"; }
  else { c.textContent = "就绪"; c.className = "tag ok"; }
}

function refresh() {
  return api("/api/state").then(function (s) { S = s; render(); });
}

function pickGroups() {
  var xs = document.querySelectorAll("input.gp"), o = [], i;
  for (i = 0; i < xs.length; i++) {
    if (xs[i].checked && !xs[i].disabled) { o.push(xs[i].value); }
  }
  return o;
}

function card(it, kind) {
  var url = "/api/file?path=" + encodeURIComponent(it.path), inner;
  var low = it.name.toLowerCase();
  if (kind === "vid" && low.slice(-4) !== ".gif") {
    inner = '<video src="' + url + '" controls preload="metadata"></video>';
  }
  else if (low.slice(-4) === ".pdf") { inner = '<div class="pdf">PDF</div>'; }
  else { inner = '<a href="' + url + '" target="_blank"><img src="' + url + '"></a>'; }
  return '<div class="item">' + inner + '<div class="cap"><a href="' + url +
         '" target="_blank">' + esc(it.name) + '</a><span style="flex:1"></span><span>' +
         kb(it.size) + '</span></div></div>';
}

function loadProducts() {
  api("/api/products").then(function (d) {
    var h = "", i;
    for (i = 0; i < d.videos.length; i++) { h += card(d.videos[i], "vid"); }
    for (i = 0; i < d.figures.length; i++) { h += card(d.figures[i], "img"); }
    $("grid").innerHTML = h;
    $("nores").style.display = h ? "none" : "block";
  });
}

function run(task) {
  var body = { task: task, groups: pickGroups(), skip_slam: $("skipSlam").checked,
               hold: parseFloat($("hold").value) || 1.5,
               ds: $("ds").value, root: $("root").value };
  $("log").textContent = "";
  $("hint").textContent = "";
  jobOff = 0;
  api("/api/run", { method: "POST", headers: { "Content-Type": "application/json" },
                    body: JSON.stringify(body) })
    .then(function (r) {
      jobKey = r.job;
      $("stopBtn").disabled = false;
      $("hint").textContent = "已开始：" + r.title;
      tick();
    })
    .catch(function (e) { $("log").textContent = "启动失败：" + e.message; });
}

function tick() {
  if (tm) { clearTimeout(tm); tm = null; }
  if (!jobKey) { return; }
  api("/api/job?key=" + encodeURIComponent(jobKey) + "&off=" + jobOff)
    .then(function (j) {
      if (j.text) {
        var lg = $("log");
        lg.textContent += j.text;
        lg.scrollTop = lg.scrollHeight;
      }
      jobOff = j.offset;
      $("bar").style.display = j.done ? "none" : "block";
      $("jobInfo").textContent = "[" + j.title + "] " + (j.done
        ? ("结束 rc=" + j.rc + "，用时 " + j.elapsed + "s")
        : ("运行中 " + j.elapsed + "s"));
      if (j.done) {
        $("stopBtn").disabled = true;
        jobKey = "";
        loadProducts();
        refresh();
        return;
      }
      tm = setTimeout(tick, 900);
    })
    .catch(function () { tm = setTimeout(tick, 1500); });
}

function loadDir(p) {
  api("/api/ls?path=" + encodeURIComponent(p)).then(function (d) {
    curDir = d.path;
    $("curPath").value = d.path;
    var h = "", i;
    if (d.parent) { h += '<button class="ditem" data-p="' + esc(d.parent) + '">.. 上一层</button>'; }
    for (i = 0; i < d.dirs.length; i++) {
      h += '<button class="ditem" data-p="' + esc(d.dirs[i].path) + '">' +
           esc(d.dirs[i].name) +
           (d.dirs[i].has_clean ? ' <span class="tag ok">有 clean</span>' : '') +
           (d.dirs[i].has_dataset ? ' <span class="tag ok">数据集</span>' : '') + '</button>';
    }
    if (!h) { h = '<div class="mut">没有子文件夹</div>'; }
    $("dlist").innerHTML = h;
    var bs = document.querySelectorAll("button.ditem"), j;
    for (j = 0; j < bs.length; j++) {
      bs[j].onclick = function () { loadDir(this.getAttribute("data-p")); };
    }
  });
}

function openPick(mode, start) {
  pickMode = mode;
  $("modal").classList.add("on");
  loadDir(start || (S ? (mode === "root" ? S.root : S.ds) : "") || "/");
}

function saveCfg() {
  var body = { ds: $("ds").value, root: $("root").value,
               hold: parseFloat($("hold").value) || 1.5,
               skip_slam: $("skipSlam").checked, groups: pickGroups() };
  api("/api/config", { method: "POST", headers: { "Content-Type": "application/json" },
                       body: JSON.stringify(body) }).then(refresh);
}

function init() {
  $("pickDs").onclick = function () { openPick("ds", $("ds").value); };
  $("pickRoot").onclick = function () { openPick("root", $("root").value); };
  $("apply").onclick = saveCfg;
  $("cancelPick").onclick = function () { $("modal").classList.remove("on"); };
  $("goPath").onclick = function () { loadDir($("curPath").value); };
  $("okPick").onclick = function () {
    if (pickMode === "root") { $("root").value = curDir; }
    else { $("ds").value = curDir; }
    $("modal").classList.remove("on");
    saveCfg();
  };
  $("runFig").onclick = function () { run("figures"); };
  $("runVideo").onclick = function () { run("video"); };
  $("runDev").onclick = function () { run("deviation"); };
  $("runPairs").onclick = function () { run("frame_pairs"); };
  $("runAll").onclick = function () { run("all"); };
  $("stopBtn").onclick = function () { api("/api/stop", { method: "POST" }).catch(function () {}); };
  $("clearLog").onclick = function () { $("log").textContent = ""; };
  refresh().then(loadProducts);
}

init();
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    cfg = None
    server_version = "orbviz-gui"

    def log_message(self, *a):
        pass

    def _send(self, code, ctype, data):
        if isinstance(data, str):
            data = data.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _json(self, obj, code=200):
        self._send(code, "application/json; charset=utf-8",
                   json.dumps(obj, ensure_ascii=False))

    def _body(self):
        try:
            n = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            n = 0
        if n <= 0:
            return {}
        try:
            return json.loads(self.rfile.read(n).decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            return {}

    def _allowed(self, raw):
        p = expand(raw)
        for r in (self.cfg.root, self.cfg.ds, self.cfg.scripts, HERE):
            if r and (p == r or p.startswith(r.rstrip(os.sep) + os.sep)):
                return p
        return None

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        if u.path in ("/", "/index.html"):
            return self._send(200, "text/html; charset=utf-8", PAGE)
        if u.path == "/api/state":
            return self._json(self.cfg.state())
        if u.path == "/api/products":
            p = self.cfg.paths()
            return self._json({"figures": list_files(p["figures"], IMG_EXT + (".pdf",)),
                               "videos": list_files(p["videos"], VID_EXT)})
        if u.path == "/api/ls":
            return self._ls((q.get("path") or ["~"])[0])
        if u.path == "/api/job":
            with JOBS_LOCK:
                job = JOBS.get((q.get("key") or [""])[0])
            if job is None:
                return self._json({"text": "", "offset": 0, "done": True, "rc": None,
                                   "title": "", "key": "", "elapsed": 0})
            return self._json(job.snapshot((q.get("off") or ["0"])[0]))
        if u.path == "/api/file":
            return self._file((q.get("path") or [""])[0])
        return self._json({"error": "not found"}, 404)

    def _ls(self, raw):
        p = expand(raw) if raw else expand("~")
        if not os.path.isdir(p):
            up = os.path.dirname(p)
            p = up if os.path.isdir(up) else expand("~")
        dirs = []
        try:
            names = sorted(os.listdir(p), key=str.lower)
        except OSError:
            names = []
        for n in names:
            if n.startswith("."):
                continue
            fp = os.path.join(p, n)
            if not os.path.isdir(fp):
                continue
            dirs.append({"name": n, "path": fp,
                         "has_clean": os.path.isdir(os.path.join(fp, "clean")),
                         "has_dataset": os.path.isfile(
                             os.path.join(fp, "comparison_pairs.csv"))})
        stripped = p.rstrip(os.sep)
        parent = os.path.dirname(stripped) if os.path.dirname(stripped) else ""
        return self._json({"path": p, "parent": parent, "dirs": dirs[:400]})

    def _file(self, raw):
        p = self._allowed(raw) if raw else None
        if not p or not os.path.isfile(p):
            return self._send(404, "text/plain; charset=utf-8", "not found")
        ext = os.path.splitext(p)[1].lower()
        ctype = {".png": "image/png", ".jpg": "image/jpeg", ".jpeg": "image/jpeg",
                 ".gif": "image/gif", ".svg": "image/svg+xml", ".pdf": "application/pdf",
                 ".mp4": "video/mp4", ".webm": "video/webm",
                 ".csv": "text/csv; charset=utf-8", ".txt": "text/plain; charset=utf-8",
                 ".log": "text/plain; charset=utf-8"}.get(ext, "application/octet-stream")
        size = os.path.getsize(p)
        start, end, code = 0, max(0, size - 1), 200
        rng = self.headers.get("Range") or ""
        if rng.startswith("bytes=") and size > 0:
            try:
                a, _, b = rng[6:].partition("-")
                if a:
                    start = int(a)
                if b:
                    end = int(b)
                start = max(0, min(start, size - 1))
                end = max(start, min(end, size - 1))
                code = 206
            except ValueError:
                start, end, code = 0, size - 1, 200
        length = end - start + 1
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(length))
        self.send_header("Accept-Ranges", "bytes")
        if code == 206:
            self.send_header("Content-Range", "bytes %d-%d/%d" % (start, end, size))
        self.end_headers()
        try:
            with open(p, "rb") as fh:
                fh.seek(start)
                left = length
                while left > 0:
                    chunk = fh.read(min(262144, left))
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    left -= len(chunk)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self):
        u = urlparse(self.path)
        body = self._body()
        if u.path == "/api/config":
            self.cfg.update(body)
            return self._json(self.cfg.state())
        if u.path == "/api/run":
            return self._run(body)
        if u.path == "/api/stop":
            with JOBS_LOCK:
                job = JOBS.get(CURRENT.get("key") or "")
            if job is not None:
                job.kill()
            return self._json({"stopped": True})
        return self._json({"error": "not found"}, 404)

    def _run(self, body):
        task = str(body.get("task") or "")
        groups = [g for g in (body.get("groups") or []) if g in ALL_GROUPS and g != "clean"]
        data = {"groups": groups, "skip_slam": bool(body.get("skip_slam"))}
        if str(body.get("ds") or "").strip():
            data["ds"] = body["ds"]
        if body.get("root"):
            data["root"] = body.get("root")
        if body.get("hold") is not None:
            data["hold"] = body.get("hold")
        self.cfg.update(data)
        st = self.cfg.state()
        if not st["has_ds_clean"]:
            return self._json({"error": "数据集目录不对：找不到 clean/，先点「指定数据集路径」"}, 400)
        if not groups:
            groups = [g for g in ALL_GROUPS if g != "clean" and st["available"][g]]
        if task != "figures" and not groups:
            return self._json({"error": "数据集里没有可用的干扰组（fixed_pixel / world_plane）"}, 400)
        if not st["has_scripts"]:
            return self._json({"error": "找不到 run_compare.sh 等脚本，确认 gui_app.py 和它们在同一目录"}, 400)
        try:
            job = start_job(self.cfg, task, groups, self.cfg.skip_slam, self.cfg.hold)
        except (ValueError, OSError) as exc:
            return self._json({"error": str(exc)}, 400)
        return self._json({"job": job.key, "title": job.title})


def main():
    ap = argparse.ArgumentParser(description="ORB-SLAM3 可视化对照工具链 · 简易网页界面")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--root", default="", help="ORB_SLAM3 根目录（默认 ~/ORB_SLAM3）")
    ap.add_argument("--ds", default="", help="数据集根目录")
    ap.add_argument("--py", default="", help="python3 解释器")
    ap.add_argument("--hold", type=float, default=1.5, help="视频每帧停留秒数")
    ap.add_argument("--groups", default="", help="逗号分隔：fixed_pixel,world_plane")
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args()
    gs = [g.strip() for g in a.groups.split(",") if g.strip()]
    cfg = Config(argparse.Namespace(root=a.root, ds=a.ds, py=a.py, hold=a.hold, groups=gs))
    Handler.cfg = cfg
    st = cfg.state()
    url = "http://%s:%d/" % ("127.0.0.1" if a.host in ("0.0.0.0", "::") else a.host, a.port)
    print("=" * 60)
    print(" ORB-SLAM3 可视化对照工具链 · 网页界面")
    print("   地址       : " + url)
    print("   脚本目录   : " + st["paths"]["scripts"])
    print("   SLAM 根目录: " + st["root"])
    print("   数据集     : " + (st["ds"] or "（未指定，页面上点「指定数据集路径」）"))
    print("   Ctrl+C 退出")
    print("=" * 60)
    srv = ThreadingHTTPServer((a.host, a.port), Handler)
    if not a.no_browser:
        threading.Thread(target=lambda: (time.sleep(0.7), webbrowser.open(url)),
                         daemon=True).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n[i] 已退出")
    finally:
        srv.server_close()


if __name__ == "__main__":
    main()
