#define _CRT_SECURE_NO_WARNINGS

#include "agv/internal/engine_internal.hpp"

std::vector<std::string> collect_recent_logs(const Logger* logger) {
    std::vector<std::string> lines;
    if (!logger || logger->log_count <= 0) {
        return lines;
    }

    lines.reserve(static_cast<std::size_t>(logger->log_count));
    for (int offset = 0; offset < logger->log_count; ++offset) {
        const int index = (logger->log_head + offset) % LOG_BUFFER_LINES;
        lines.push_back(logger->logs[index]);
    }
    return lines;
}
