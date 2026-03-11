#define _CRT_SECURE_NO_WARNINGS

#include <locale.h>
#include <stdio.h>
#include <windows.h>

void system_enable_virtual_terminal() {
    SetConsoleOutputCP(65001);
    SetConsoleCP(65001);
    setlocale(LC_ALL, ".UTF8");

    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
    if (hOut == INVALID_HANDLE_VALUE) return;

    DWORD dwMode = 0;
    if (!GetConsoleMode(hOut, &dwMode)) return;
    dwMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
    SetConsoleMode(hOut, dwMode);
}

void ui_clear_screen_optimized() {
    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    if (hConsole == INVALID_HANDLE_VALUE) return;

    DWORD mode = 0;
    if (GetConsoleMode(hConsole, &mode) && (mode & ENABLE_VIRTUAL_TERMINAL_PROCESSING)) {
        fputs("\x1b[H\x1b[2J\x1b[3J", stdout);
        fflush(stdout);
        return;
    }

    CONSOLE_SCREEN_BUFFER_INFO csbi;
    if (!GetConsoleScreenBufferInfo(hConsole, &csbi)) return;

    DWORD cellCount = (DWORD)csbi.dwSize.X * (DWORD)csbi.dwSize.Y;
    DWORD count;
    COORD home = { 0, 0 };

    FillConsoleOutputCharacterA(hConsole, ' ', cellCount, home, &count);
    FillConsoleOutputAttribute(hConsole, csbi.wAttributes, cellCount, home, &count);
    SetConsoleCursorPosition(hConsole, home);
}

void ensure_console_width(int minCols) {
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    if (h == INVALID_HANDLE_VALUE) return;

    CONSOLE_SCREEN_BUFFER_INFO csbi;
    if (!GetConsoleScreenBufferInfo(h, &csbi)) return;

    COORD size = csbi.dwSize;
    if (size.X < minCols) {
        size.X = (SHORT)minCols;
        SetConsoleScreenBufferSize(h, size);
    }
}


void agv_prepare_console(void) {
    system_enable_virtual_terminal();
    ensure_console_width(180);
}

void ui_enter_alt_screen(void) {
    fputs("\x1b[?1049h\x1b[H\x1b[?25l", stdout);
    fflush(stdout);
}

void ui_leave_alt_screen(void) {
    fputs("\x1b[?1049l\x1b[?25h", stdout);
    fflush(stdout);
}
