/* Windows 启动器源码：gcc -O2 -s -o orb-slam3-visual-toolkit.exe launcher.c */
#include <windows.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define MAXP 2048
#define MAXC 8192

static int is_file(const char *p) {
    DWORD a = GetFileAttributesA(p);
    return a != INVALID_FILE_ATTRIBUTES && !(a & FILE_ATTRIBUTE_DIRECTORY);
}

static int on_path(const char *name, char *out) {
    DWORD r = SearchPathA(NULL, name, ".exe", MAXP, out, NULL);
    return r > 0 && r < MAXP;
}

static int glob_first(const char *pattern, char *out) {
    WIN32_FIND_DATAA fd;
    HANDLE h = FindFirstFileA(pattern, &fd);
    if (h == INVALID_HANDLE_VALUE) return 0;
    FindClose(h);
    char dir[MAXP];
    strncpy(dir, pattern, MAXP - 1);
    dir[MAXP - 1] = 0;
    char *s = strrchr(dir, '\\');
    if (s) *(s + 1) = 0; else dir[0] = 0;
    snprintf(out, MAXP, "%s%s", dir, fd.cFileName);
    return is_file(out);
}

static int find_python(char *exe, char *prefix) {
    if (on_path("py", exe)) { strcpy(prefix, "-3 "); return 1; }
    if (on_path("python", exe)) { prefix[0] = 0; return 1; }
    if (on_path("python3", exe)) { prefix[0] = 0; return 1; }
    char pat[MAXP];
    const char *la = getenv("LOCALAPPDATA");
    if (la) {
        snprintf(pat, MAXP, "%s\\Programs\\Python\\Python3*\\python.exe", la);
        if (glob_first(pat, exe)) { prefix[0] = 0; return 1; }
    }
    if (glob_first("C:\\Python3*\\python.exe", exe)) { prefix[0] = 0; return 1; }
    return 0;
}

static void pause_hold(void) {
    printf("\n按回车键关闭窗口 ...");
    fflush(stdout);
    getchar();
}

int main(int argc, char **argv) {
    SetConsoleOutputCP(CP_UTF8);
    char exe[MAXP], dir[MAXP], script[MAXP], py[MAXP], prefix[8];
    char cmd[MAXC];

    GetModuleFileNameA(NULL, exe, MAXP);
    strncpy(dir, exe, MAXP - 1);
    dir[MAXP - 1] = 0;
    char *slash = strrchr(dir, '\\');
    if (slash) *slash = 0;

    const char *cands[3];
    char c0[MAXP], c1[MAXP], c2[MAXP];
    snprintf(c0, MAXP, "%s\\gui_app.py", dir);
    snprintf(c1, MAXP, "%s\\outputs\\fig4_4seasons\\gui_app.py", dir);
    snprintf(c2, MAXP, "%s\\..\\outputs\\fig4_4seasons\\gui_app.py", dir);
    cands[0] = c0; cands[1] = c1; cands[2] = c2;

    script[0] = 0;
    for (int i = 0; i < 3; i++) {
        if (is_file(cands[i])) { strncpy(script, cands[i], MAXP - 1); script[MAXP - 1] = 0; break; }
    }

    int port = 8770;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            port = atoi(argv[i + 1]);
        } else if (strncmp(argv[i], "--port=", 7) == 0) {
            port = atoi(argv[i] + 7);
        }
    }

    printf("============================================================\n");
    printf(" ORB-SLAM3 可视化对照工具链\n");
    printf("============================================================\n");

    if (!script[0]) {
        printf("[x] 没找到 gui_app.py。\n");
        printf("    这个 exe 要和 gui_app.py 放在一起（或者放在它上一级目录）。\n");
        printf("    当前目录：%s\n", dir);
        pause_hold();
        return 1;
    }
    printf("  脚本    : %s\n", script);

    if (!find_python(py, prefix)) {
        printf("[x] 没找到 Python。\n");
        printf("    装一个 Python 3 之后再双击本程序： https://www.python.org/downloads/\n");
        printf("    安装时记得勾选 Add python.exe to PATH。\n");
        pause_hold();
        return 1;
    }
    printf("  Python  : %s\n", py);
    printf("  界面地址: http://127.0.0.1:%d/\n", port);
    printf("  Ctrl+C 退出\n");
    printf("============================================================\n\n");
    fflush(stdout);

    snprintf(cmd, MAXC, "\"%s\" %s\"%s\"", py, prefix, script);
    for (int i = 1; i < argc; i++) {
        size_t used = strlen(cmd);
        if (used + strlen(argv[i]) + 4 >= MAXC) break;
        strcat(cmd, " \"");
        strcat(cmd, argv[i]);
        strcat(cmd, "\"");
    }

    STARTUPINFOA si;
    PROCESS_INFORMATION pi;
    ZeroMemory(&si, sizeof(si));
    si.cb = sizeof(si);
    ZeroMemory(&pi, sizeof(pi));

    if (!CreateProcessA(NULL, cmd, NULL, NULL, TRUE, 0, NULL, dir, &si, &pi)) {
        printf("[x] 启动失败，错误码 %lu\n", GetLastError());
        pause_hold();
        return 1;
    }
    WaitForSingleObject(pi.hProcess, INFINITE);
    DWORD rc = 1;
    GetExitCodeProcess(pi.hProcess, &rc);
    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);

    if (rc != 0) {
        printf("\n[x] 界面进程退出，返回码 %lu\n", rc);
        pause_hold();
    }
    return (int)rc;
}