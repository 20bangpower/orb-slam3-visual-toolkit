/* Windows 启动器源码：gcc -O2 -s -o orb-slam3-visual-toolkit.exe launcher.c
 *
 * 只干两件事：找到同目录（或上一级）的 gui_app.py、找到一个 Python 3，
 * 然后用它们起本机的网页界面。
 *
 * 全走宽字符 API，所以解压路径里有中文、空格也没问题。
 */
#include <windows.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <wchar.h>

#define MAXP 4096
#define MAXC 16384

static void say(const wchar_t *w) {          /* 宽字符 -> UTF-8 输出 */
    char buf[MAXP * 4];
    int n = WideCharToMultiByte(CP_UTF8, 0, w, -1, buf, (int)sizeof(buf), NULL, NULL);
    if (n > 0) fputs(buf, stdout);
}

static int is_file(const wchar_t *p) {
    DWORD a = GetFileAttributesW(p);
    return a != INVALID_FILE_ATTRIBUTES && !(a & FILE_ATTRIBUTE_DIRECTORY);
}

static int on_path(const wchar_t *name, wchar_t *out) {
    DWORD r = SearchPathW(NULL, name, L".exe", MAXP, out, NULL);
    return r > 0 && r < MAXP;
}

static int glob_first(const wchar_t *pattern, wchar_t *out) {
    WIN32_FIND_DATAW fd;
    HANDLE h = FindFirstFileW(pattern, &fd);
    if (h == INVALID_HANDLE_VALUE) return 0;
    FindClose(h);
    wchar_t dir[MAXP];
    wcsncpy(dir, pattern, MAXP - 1);
    dir[MAXP - 1] = 0;
    wchar_t *s = wcsrchr(dir, L'\\');
    if (s) *(s + 1) = 0; else dir[0] = 0;
    _snwprintf(out, MAXP, L"%s%s", dir, fd.cFileName);
    return is_file(out);
}

static int find_python(wchar_t *exe, wchar_t *prefix) {
    if (on_path(L"py", exe)) { wcscpy(prefix, L"-3 "); return 1; }
    if (on_path(L"python", exe)) { prefix[0] = 0; return 1; }
    if (on_path(L"python3", exe)) { prefix[0] = 0; return 1; }
    wchar_t pat[MAXP];
    const wchar_t *la = _wgetenv(L"LOCALAPPDATA");
    if (la) {
        _snwprintf(pat, MAXP, L"%s\\Programs\\Python\\Python3*\\python.exe", la);
        if (glob_first(pat, exe)) { prefix[0] = 0; return 1; }
    }
    if (glob_first(L"C:\\Python3*\\python.exe", exe)) { prefix[0] = 0; return 1; }
    return 0;
}

/* 命令行拆 token（认得双引号），只用来挑 --port 和原样转发 */
static int wsplit(wchar_t *s, wchar_t **out, int max) {
    int n = 0;
    while (*s && n < max) {
        while (*s == L' ' || *s == L'\t') s++;
        if (!*s) break;
        if (*s == L'"') {
            s++;
            out[n++] = s;
            while (*s && *s != L'"') s++;
            if (*s == L'"') *s++ = 0;
        } else {
            out[n++] = s;
            while (*s && *s != L' ' && *s != L'\t') s++;
            if (*s) *s++ = 0;
        }
    }
    return n;
}

static void pause_hold(void) {
    printf("\n按回车键关闭窗口 ...");
    fflush(stdout);
    getchar();
}

int main(void) {
    SetConsoleOutputCP(CP_UTF8);

    wchar_t exe[MAXP], dir[MAXP], script[MAXP], py[MAXP], prefix[8], cmd[MAXC];
    wchar_t cl[MAXC];
    wchar_t *args[64];

    wcsncpy(cl, GetCommandLineW(), MAXC - 1);
    cl[MAXC - 1] = 0;
    int nargs = wsplit(cl, args, 64);

    GetModuleFileNameW(NULL, exe, MAXP);
    wcsncpy(dir, exe, MAXP - 1);
    dir[MAXP - 1] = 0;
    wchar_t *slash = wcsrchr(dir, L'\\');
    if (slash) *slash = 0;

    const wchar_t *cands[3];
    wchar_t c0[MAXP], c1[MAXP], c2[MAXP];
    _snwprintf(c0, MAXP, L"%s\\gui_app.py", dir);
    _snwprintf(c1, MAXP, L"%s\\outputs\\fig4_4seasons\\gui_app.py", dir);
    _snwprintf(c2, MAXP, L"%s\\..\\outputs\\fig4_4seasons\\gui_app.py", dir);
    cands[0] = c0; cands[1] = c1; cands[2] = c2;

    script[0] = 0;
    for (int i = 0; i < 3; i++) {
        if (is_file(cands[i])) { wcsncpy(script, cands[i], MAXP - 1); script[MAXP - 1] = 0; break; }
    }

    int port = 8770;
    for (int i = 1; i < nargs; i++) {
        if (wcscmp(args[i], L"--port") == 0 && i + 1 < nargs) port = _wtoi(args[i + 1]);
        else if (wcsncmp(args[i], L"--port=", 7) == 0) port = _wtoi(args[i] + 7);
    }

    printf("============================================================\n");
    printf(" ORB-SLAM3 可视化对照工具链\n");
    printf("============================================================\n");

    if (!script[0]) {
        printf("[x] 没找到 gui_app.py。\n");
        printf("    这个 exe 要和 gui_app.py 放在一起（或者放在它上一级目录）。\n");
        say(L"    当前目录："); say(dir); say(L"\n");
        pause_hold();
        return 1;
    }
    say(L"  脚本    : "); say(script); say(L"\n");

    if (!find_python(py, prefix)) {
        printf("[x] 没找到 Python。\n");
        printf("    装一个 Python 3 之后再双击本程序： https://www.python.org/downloads/\n");
        printf("    安装时记得勾选 Add python.exe to PATH。\n");
        pause_hold();
        return 1;
    }
    say(L"  Python  : "); say(py); say(L"\n");
    printf("  界面地址: http://127.0.0.1:%d/\n", port);
    printf("  Ctrl+C 退出\n");
    printf("============================================================\n\n");
    fflush(stdout);

    _snwprintf(cmd, MAXC, L"\"%s\" %s\"%s\"", py, prefix, script);
    for (int i = 1; i < nargs; i++) {
        size_t used = wcslen(cmd);
        if (used + wcslen(args[i]) + 4 >= MAXC) break;
        wcscat(cmd, L" \"");
        wcscat(cmd, args[i]);
        wcscat(cmd, L"\"");
    }

    STARTUPINFOW si;
    PROCESS_INFORMATION pi;
    ZeroMemory(&si, sizeof(si));
    si.cb = sizeof(si);
    ZeroMemory(&pi, sizeof(pi));

    if (!CreateProcessW(NULL, cmd, NULL, NULL, TRUE, 0, NULL, dir, &si, &pi)) {
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
        printf("\n[i] 界面进程已退出（返回码 %lu）。上面若有提示，按提示处理。\n", rc);
        pause_hold();
    }
    return (int)rc;
}
