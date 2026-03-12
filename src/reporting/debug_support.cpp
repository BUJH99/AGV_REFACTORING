#define _CRT_SECURE_NO_WARNINGS

#include "agv/internal/engine_internal.hpp"

#include <algorithm>

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

std::vector<agv::core::StructuredLogEntry> collect_structured_logs(
    const Logger* logger,
    std::uint64_t since_seq,
    std::size_t max_entries) {
    std::vector<agv::core::StructuredLogEntry> logs;
    if (!logger || logger->structured_logs.empty() || max_entries == 0) {
        return logs;
    }

    logs.reserve(std::min<std::size_t>(max_entries, logger->structured_logs.size()));
    for (const agv::core::StructuredLogEntry& entry : logger->structured_logs) {
        if (entry.seq <= since_seq) {
            continue;
        }
        logs.push_back(entry);
        if (logs.size() >= max_entries) {
            break;
        }
    }

    return logs;
}
