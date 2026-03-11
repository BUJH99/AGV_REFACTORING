#define _CRT_SECURE_NO_WARNINGS

#include <chrono>
#include <clocale>
#include <conio.h>
#include <thread>
#include <windows.h>

#include "agv/internal/engine_internal.hpp"

void system_enable_virtual_terminal() {
    SetConsoleOutputCP(65001);
    SetConsoleCP(65001);
    std::setlocale(LC_ALL, ".UTF8");

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
        agv::internal::text::console_write("\x1b[H\x1b[2J\x1b[3J");
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
    agv::internal::text::console_write("\x1b[?1049h\x1b[H\x1b[?25l");
}

void ui_leave_alt_screen(void) {
    agv::internal::text::console_write("\x1b[?1049l\x1b[?25h");
}

void platform_sleep_for_ms(int ms) {
    if (ms <= 0) {
        return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int console_read_key_blocking() {
    return _getch();
}

std::optional<int> console_read_key_nonblocking() {
    if (!_kbhit()) {
        return std::nullopt;
    }
    return _getch();
}
