#define _CRT_SECURE_NO_WARNINGS

#include "agv/internal/engine_internal.hpp"

#include <algorithm>
#include <cctype>

namespace {

using agv::core::StructuredLogEntry;

std::string strip_ansi(std::string_view text) {
    std::string cleaned;
    cleaned.reserve(text.size());

    for (std::size_t index = 0; index < text.size(); ++index) {
        const char ch = text[index];
        if (ch == '\x1b' && (index + 1) < text.size() && text[index + 1] == '[') {
            index += 2;
            while (index < text.size()) {
                const char code = text[index];
                if ((code >= '@' && code <= '~') || std::isalpha(static_cast<unsigned char>(code))) {
                    break;
                }
                ++index;
            }
            continue;
        }
        cleaned.push_back(ch);
    }

    return cleaned;
}

std::string uppercase(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
        [](unsigned char ch) { return static_cast<char>(std::toupper(ch)); });
    return value;
}

std::string lowercase(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
        [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return value;
}

std::string extract_prefix_token(std::string_view cleaned) {
    const std::size_t open = cleaned.find('[');
    const std::size_t close = (open == std::string_view::npos) ? std::string_view::npos : cleaned.find(']', open + 1);
    if (open == std::string_view::npos || close == std::string_view::npos || close <= open + 1) {
        return {};
    }
    return uppercase(std::string(cleaned.substr(open + 1, close - open - 1)));
}

std::optional<int> infer_agent_id_from_text(const Logger& logger, std::string_view cleaned) {
    if (!logger.owner || !logger.owner->agent_manager) {
        return std::nullopt;
    }

    const std::string marker = "Agent ";
    const std::size_t pos = cleaned.find(marker);
    if (pos == std::string_view::npos || (pos + marker.size()) >= cleaned.size()) {
        return std::nullopt;
    }

    const char symbol = cleaned[pos + marker.size()];
    for (int index = 0; index < MAX_AGENTS; ++index) {
        const Agent& agent = logger.owner->agent_manager->agents[index];
        if (agent.symbol == symbol) {
            return agent.id;
        }
    }

    return std::nullopt;
}

LoggerMessageMeta classify_legacy_log_line(const Logger& logger, std::string_view cleaned) {
    LoggerMessageMeta meta;
    meta.phaseIndex = (logger.context_phase_index >= 0) ? std::optional<int>(logger.context_phase_index) : std::nullopt;
    meta.agentId = infer_agent_id_from_text(logger, cleaned);

    const std::string token = extract_prefix_token(cleaned);
    const std::string lower = lowercase(std::string(cleaned));

    if (token == "WARN" || token == "AVOID") {
        meta.level = "Warn";
        meta.category = (lower.find("charge") != std::string::npos) ? "Charge" : "General";
        return meta;
    }
    if (token == "ERR" || token == "ERROR") {
        meta.level = "Error";
        meta.category = "General";
        return meta;
    }
    if (token == "CTRL" || token == "ALGO" || token == "MAP" || token == "CLEANUP") {
        meta.category = "Control";
        return meta;
    }
    if (token == "PHASE" || token == "EVENT") {
        meta.category = "Scenario";
        return meta;
    }
    if (token == "TASK" || token == "PARK" || token == "EXIT") {
        meta.category = "Dispatch";
        return meta;
    }
    if (token == "CHARGE") {
        meta.category = "Charge";
        return meta;
    }
    if (token == "WFG" || token == "CBS" || token == "WHCA" || token == "PLAN") {
        meta.category = "Planner";
        return meta;
    }
    if (token == "INFO") {
        if (lower.find("waiting") != std::string::npos || lower.find("holding goal") != std::string::npos) {
            meta.category = "Wait";
        } else if (lower.find("charging") != std::string::npos) {
            meta.category = "Charge";
        } else if (lower.find("returned home") != std::string::npos || lower.find("reached ") != std::string::npos) {
            meta.category = "Motion";
        }
        return meta;
    }

    return meta;
}

void append_plain_log(Logger& logger, std::string_view message) {
    const int idx = (logger.log_head + logger.log_count) % LOG_BUFFER_LINES;
    logger.logs[idx] = std::string(message);
    if (logger.logs[idx].size() >= LOG_BUFFER_WIDTH) {
        logger.logs[idx].resize(LOG_BUFFER_WIDTH - 1);
    }
    if (logger.log_count < LOG_BUFFER_LINES) {
        logger.log_count++;
    } else {
        logger.log_head = (logger.log_head + 1) % LOG_BUFFER_LINES;
    }
}

void append_structured_log(Logger& logger, LoggerMessageMeta meta, std::string_view message) {
    StructuredLogEntry entry;
    entry.seq = logger.next_structured_seq++;
    entry.step = logger.context_step;
    entry.frameId = logger.context_frame_id;
    entry.category = meta.category.empty() ? "General" : meta.category;
    entry.level = meta.level.empty() ? "Info" : meta.level;
    entry.text = strip_ansi(message);
    entry.agentId = meta.agentId;
    entry.phaseIndex = meta.phaseIndex.has_value()
        ? meta.phaseIndex
        : ((logger.context_phase_index >= 0) ? std::optional<int>(logger.context_phase_index) : std::nullopt);
    logger.structured_logs.push_back(std::move(entry));
    while (logger.structured_logs.size() > Logger::kStructuredLogHistoryLimit) {
        logger.structured_logs.pop_front();
    }
}

}  // namespace

Planner::Planner(std::unique_ptr<PlannerStrategy> strategy)
    : strategy_(std::move(strategy)) {}

Planner::Planner(const Planner& other) {
    if (other.strategy_) {
        strategy_ = other.strategy_->clone();
    }
}

Planner& Planner::operator=(const Planner& other) {
    if (this == &other) return *this;
    strategy_.reset();
    if (other.strategy_) {
        strategy_ = other.strategy_->clone();
    }
    return *this;
}

void Planner::reset(std::unique_ptr<PlannerStrategy> strategy) {
    strategy_ = std::move(strategy);
}

void Planner::planStep(const PlanningContext& context, AgentNodeSlots& next_positions) const {
    if (strategy_) {
        strategy_->planStep(context, next_positions);
    }
}

RendererFacade::RendererFacade(std::unique_ptr<RendererStrategy> strategy)
    : strategy_(std::move(strategy)) {}

RendererFacade::RendererFacade(const RendererFacade& other) {
    if (other.strategy_) {
        strategy_ = other.strategy_->clone();
    }
}

RendererFacade& RendererFacade::operator=(const RendererFacade& other) {
    if (this == &other) return *this;
    strategy_.reset();
    if (other.strategy_) {
        strategy_ = other.strategy_->clone();
    }
    return *this;
}

void RendererFacade::drawFrame(Simulation_* sim, bool is_paused) const {
    if (strategy_) {
        strategy_->drawFrame(sim, is_paused);
    }
}

void Logger::bindSimulation(Simulation_* simulation) {
    owner = simulation;
    context_step = owner ? owner->total_executed_steps : 0;
    context_frame_id = owner ? owner->render_model.frame_id : 0;
    context_phase_index = (owner && owner->scenario_manager) ? owner->scenario_manager->current_phase_index : -1;
}

void Logger::setContext(int step, std::uint64_t frame_id, int phase_index) {
    context_step = step;
    context_frame_id = frame_id;
    context_phase_index = phase_index;
}

void Logger::appendLine(std::string_view message) {
    append_plain_log(*this, message);
    append_structured_log(*this, classify_legacy_log_line(*this, strip_ansi(message)), message);
}

void Logger::appendStructuredLine(const LoggerMessageMeta& meta, std::string_view message) {
    append_plain_log(*this, message);
    append_structured_log(*this, meta, message);
}
