#define _CRT_SECURE_NO_WARNINGS

#include "agv/internal/engine_internal.hpp"

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

void Logger::appendLine(std::string_view message) {
    const int idx = (log_head + log_count) % LOG_BUFFER_LINES;
    logs[idx] = std::string(message);
    if (logs[idx].size() >= LOG_BUFFER_WIDTH) {
        logs[idx].resize(LOG_BUFFER_WIDTH - 1);
    }
    if (log_count < LOG_BUFFER_LINES) {
        log_count++;
    } else {
        log_head = (log_head + 1) % LOG_BUFFER_LINES;
    }
}
