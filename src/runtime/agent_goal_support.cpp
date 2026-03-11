#define _CRT_SECURE_NO_WARNINGS

#include "agent_goal_support.hpp"

#include <optional>
#include <span>

namespace {

constexpr int kReturnHomeHoldingGoalStuckSteps = 12;

enum class GoalTypeLocal {
    Parking,
    ParkedCar,
    Charge,
    HomeBase,
};

struct GoalAssignmentContextLocal final {
    Agent* agent{nullptr};
    GridMap* map{nullptr};
    AgentManager* agents{nullptr};
    Logger* logger{nullptr};
};

class TemporaryParkStateScopeLocal final {
public:
    explicit TemporaryParkStateScopeLocal(Node* node)
        : node_(node), restore_(node && node->is_parked) {
        if (restore_) {
            node_->is_parked = false;
        }
    }

    ~TemporaryParkStateScopeLocal() {
        if (restore_) {
            node_->is_parked = true;
        }
    }

private:
    Node* node_{nullptr};
    bool restore_{false};
};

class PathCostEvaluatorLocal final {
public:
    explicit PathCostEvaluatorLocal(const GoalAssignmentContextLocal& context)
        : context_(context) {}

    double compute(Node* goal) const {
        if (!context_.agent || !context_.agent->pos || !goal || !context_.map || !context_.agents) return INF;
        if (context_.agent->pos == goal) return 0.0;
        if (goal->is_obstacle) return INF;

        std::unique_ptr<Pathfinder> pathfinder = std::make_unique<Pathfinder>(context_.agent->pos, goal, context_.agent);
        if (!pathfinder) return INF;

        pathfinder->computeShortestPath(context_.map, context_.agents);
        const double cost = pathfinder->cells[context_.agent->pos->y][context_.agent->pos->x].g;
        return (cost >= INF * 0.5) ? INF : cost;
    }

private:
    GoalAssignmentContextLocal context_{};
};

struct GoalCandidateSetLocal final {
    std::span<Node*> nodes{};
    int require_parked{-1};
    bool respect_reservations{true};
    bool temporarily_unpark{false};
};

GoalCandidateSetLocal make_goal_candidate_set_local(
    GridMap* map,
    GoalTypeLocal type) {
    GoalCandidateSetLocal policy{};
    switch (type) {
    case GoalTypeLocal::Parking:
        policy.nodes = std::span<Node*>(map->goals, map->num_goals);
        policy.require_parked = 0;
        break;
    case GoalTypeLocal::ParkedCar:
        policy.nodes = std::span<Node*>(map->goals, map->num_goals);
        policy.require_parked = 1;
        policy.temporarily_unpark = true;
        break;
    case GoalTypeLocal::Charge:
        policy.nodes = std::span<Node*>(map->charge_stations, map->num_charge_stations);
        break;
    case GoalTypeLocal::HomeBase:
        break;
    }
    return policy;
}

bool goal_candidate_matches_local(
    const GoalCandidateSetLocal& candidates,
    const Agent* agent,
    const Node* node) {
    if (!agent || !node) return false;
    if (candidates.require_parked == 1 && !node->is_parked) return false;
    if (candidates.require_parked == 0 && node->is_parked) return false;
    if (candidates.respect_reservations &&
        node->reserved_by_agent != -1 &&
        node->reserved_by_agent != agent->id) {
        return false;
    }
    return true;
}

Node* select_best_candidate_local(
    const GoalCandidateSetLocal& candidates,
    Agent* agent,
    const PathCostEvaluatorLocal& evaluator,
    double* out_best_cost) {
    double best_cost = INF;
    Node* best_node = nullptr;

    for (Node* node : candidates.nodes) {
        if (!goal_candidate_matches_local(candidates, agent, node)) continue;

        TemporaryParkStateScopeLocal parked_scope(candidates.temporarily_unpark ? node : nullptr);
        const double cost = evaluator.compute(node);
        if (cost < best_cost) {
            best_cost = cost;
            best_node = node;
        }
    }

    if (out_best_cost) *out_best_cost = best_cost;
    return best_node;
}

std::string_view goal_selection_log_label_local(GoalTypeLocal type) {
    switch (type) {
    case GoalTypeLocal::Parking:
        return "parking goal";
    case GoalTypeLocal::ParkedCar:
        return "retrieval target";
    case GoalTypeLocal::Charge:
        return "charge station";
    case GoalTypeLocal::HomeBase:
    default:
        return {};
    }
}

bool agent_returns_home_local(AgentState state) {
    return state == AgentState::ReturningHomeEmpty ||
        state == AgentState::ReturningWithCar ||
        state == AgentState::ReturningHomeMaintenance;
}

std::optional<GoalTypeLocal> goal_type_for_state_local(AgentState state) {
    switch (state) {
    case AgentState::GoingToPark:
        return GoalTypeLocal::Parking;
    case AgentState::GoingToCollect:
        return GoalTypeLocal::ParkedCar;
    case AgentState::GoingToCharge:
        return GoalTypeLocal::Charge;
    case AgentState::ReturningHomeEmpty:
    case AgentState::ReturningWithCar:
    case AgentState::ReturningHomeMaintenance:
        return GoalTypeLocal::HomeBase;
    default:
        return std::nullopt;
    }
}

class GoalAssignmentPolicyLocal final {
public:
    explicit GoalAssignmentPolicyLocal(GoalAssignmentContextLocal context)
        : context_(context) {}

    Node* resolveGoalForCurrentState() const {
        if (!context_.agent) return nullptr;
        if (context_.agent->state == AgentState::ReturningHomeEmpty &&
            context_.agent->stuck_steps >= kReturnHomeHoldingGoalStuckSteps) {
            if (Node* holding_goal = selectTemporaryHoldingGoal()) {
                return holding_goal;
            }
        }
        const std::optional<GoalTypeLocal> goal_type = goal_type_for_state_local(context_.agent->state);
        if (!goal_type.has_value()) return nullptr;
        return selectBestGoal(*goal_type, nullptr);
    }

    void assignReservation(Node* new_goal) const {
        if (!context_.agent || !new_goal) return;
        if (context_.agent->goal && context_.agent->goal != new_goal) {
            context_.agent->goal->reserved_by_agent = -1;
        }
        context_.agent->goal = new_goal;
        context_.agent->goal->reserved_by_agent = context_.agent->id;
    }

    void handleMissingGoal() const {
        if (!context_.agent) return;
        if (agent_returns_home_local(context_.agent->state)) {
            if (!context_.agent->home_base) {
                context_.agent->state = AgentState::Idle;
                logger_log(context_.logger, "[%sWarn%s] Agent %c: no home position is configured. Switching to IDLE.",
                    C_B_RED, C_NRM, context_.agent->symbol);
            }
            return;
        }

        context_.agent->state = AgentState::Idle;
        logger_log(context_.logger, "[%sInfo%s] Agent %c: no valid goal found. Waiting.",
            C_YEL, C_NRM, context_.agent->symbol);
    }

    Node* selectBestChargeStation() const {
        double best_cost = INF;
        return selectBestGoal(GoalTypeLocal::Charge, &best_cost);
    }

private:
    Node* selectTemporaryHoldingGoal() const {
        const GoalCandidateSetLocal candidates = make_goal_candidate_set_local(context_.map, GoalTypeLocal::Parking);
        const PathCostEvaluatorLocal evaluator(context_);
        double best_cost = INF;
        Node* best = select_best_candidate_local(candidates, context_.agent, evaluator, &best_cost);
        if (best) {
            logger_log(context_.logger, "[%sPlan%s] Agent %c selected temporary holding goal (%d,%d) (cost %.1f)",
                C_CYN, C_NRM, context_.agent->symbol, best->x, best->y, best_cost);
        }
        return best;
    }

    Node* selectBestGoal(GoalTypeLocal type, double* out_cost) const {
        if (type == GoalTypeLocal::HomeBase) {
            const PathCostEvaluatorLocal evaluator(context_);
            if (out_cost) {
                *out_cost = context_.agent && context_.agent->pos && context_.agent->home_base
                    ? evaluator.compute(context_.agent->home_base)
                    : INF;
            }
            return context_.agent ? context_.agent->home_base : nullptr;
        }

        const GoalCandidateSetLocal candidates = make_goal_candidate_set_local(context_.map, type);
        const PathCostEvaluatorLocal evaluator(context_);
        Node* best = select_best_candidate_local(candidates, context_.agent, evaluator, out_cost);

        const std::string_view label = goal_selection_log_label_local(type);
        if (!label.empty() && best && out_cost) {
            logger_log(context_.logger, "[%sPlan%s] Agent %c selected %s (%d,%d) (cost %.1f)",
                C_CYN, C_NRM, context_.agent->symbol, label, best->x, best->y, *out_cost);
        }

        return best;
    }

    GoalAssignmentContextLocal context_{};
};

void release_current_goal_reservation_local(Agent* agent) {
    if (!agent || !agent->goal) return;
    agent->goal->reserved_by_agent = -1;
    agent->goal = nullptr;
}

void maybe_request_temporary_holding_goal_local(Agent* agent, Logger* logger) {
    if (!agent ||
        agent->state != AgentState::ReturningHomeEmpty ||
        agent->stuck_steps < kReturnHomeHoldingGoalStuckSteps ||
        !agent->goal ||
        agent->goal != agent->home_base) {
        return;
    }

    logger_log(logger, "[%sPlan%s] Agent %c is stuck while returning home. Reassigning to a temporary holding goal.",
        C_CYN, C_NRM, agent->symbol);
    release_current_goal_reservation_local(agent);
    agent->pf.reset();
}

void maybe_switch_to_charge_mode_local(Agent* agent, Logger* logger) {
    if (!agent) return;
    if (agent->state != AgentState::ReturningHomeEmpty ||
        agent->total_distance_traveled < DISTANCE_BEFORE_CHARGE) {
        return;
    }

    release_current_goal_reservation_local(agent);
    logger_log(logger, "[%sCharge%s] Agent %c exceeded the mileage threshold while returning home. Switching to charge mode.",
        C_B_YEL, C_NRM, agent->symbol);
    agent->state = AgentState::GoingToCharge;
}

bool agent_needs_goal_assignment_local(const Agent* agent) {
    return agent &&
        agent->goal == nullptr &&
        agent->state != AgentState::Idle &&
        agent->state != AgentState::Charging;
}

}  // namespace

Node* agent_runtime_select_best_charge_station(Agent* agent, GridMap* map, AgentManager* agents, Logger* logger) {
    return GoalAssignmentPolicyLocal(GoalAssignmentContextLocal{agent, map, agents, logger}).selectBestChargeStation();
}

void agent_runtime_assign_goal_if_needed(Agent* agent, GridMap* map, AgentManager* agents, Logger* logger) {
    if (!agent) return;

    if (!agent->pos) {
        agent->goal = nullptr;
        agent->state = AgentState::Idle;
        return;
    }

    maybe_switch_to_charge_mode_local(agent, logger);
    maybe_request_temporary_holding_goal_local(agent, logger);
    if (!agent_needs_goal_assignment_local(agent)) return;
    if (agent->state == AgentState::Idle || agent->state == AgentState::Charging || agent->goal) return;

    GoalAssignmentPolicyLocal policy(GoalAssignmentContextLocal{agent, map, agents, logger});
    Node* new_goal = policy.resolveGoalForCurrentState();
    if (new_goal) {
        policy.assignReservation(new_goal);
    } else {
        policy.handleMissingGoal();
    }
}
