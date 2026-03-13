#define _CRT_SECURE_NO_WARNINGS

#include <algorithm>
#include <string>
#include <string_view>
#include <ctime>

#include "agv/internal/engine_internal.hpp"

void grid_map_load_scenario(GridMap* map, AgentManager* am, int scenario_id);
Planner planner_from_pathalgo(PathAlgo algo);
void agent_begin_task_park(Agent* ag, ScenarioManager* sc, Logger* lg);
void agent_begin_task_exit(Agent* ag, ScenarioManager* sc, Logger* lg);

void agent_begin_task_park(Agent* agent, ScenarioManager* scenario, Logger* logger) {
    if (!agent || !scenario) return;
    agent->state = AgentState::GoingToPark;
    agent->metrics_task_active = true;
    agent->metrics_task_start_step = scenario->time_step;
    agent->metrics_distance_at_start = agent->total_distance_traveled;
    agent->metrics_turns_current = 0;
    if (logger) {
        logger_log_event(logger, "Dispatch", "Info", agent->id, scenario->current_phase_index,
            "Agent %c assigned to a parking task.", agent->symbol);
    }
}

void agent_begin_task_exit(Agent* agent, ScenarioManager* scenario, Logger* logger) {
    if (!agent || !scenario) return;
    agent->state = AgentState::GoingToCollect;
    agent->metrics_task_active = true;
    agent->metrics_task_start_step = scenario->time_step;
    agent->metrics_distance_at_start = agent->total_distance_traveled;
    agent->metrics_turns_current = 0;
    if (logger) {
        logger_log_event(logger, "Dispatch", "Info", agent->id, scenario->current_phase_index,
            "Agent %c assigned to a retrieval task.", agent->symbol);
    }
}

ScenarioManager::ScenarioManager() = default;

void ScenarioManager::clearTaskQueue() {
    task_queue.clear();
    task_count = 0;
}

void ScenarioManager::enqueueTask(TaskType type) {
    if (task_count >= MAX_TASKS) return;
    task_queue.push_back(TaskNode{ type, time_step });
    task_count++;
}

bool ScenarioManager::tryDequeueAssignableTask(int lot_full, int parked_count, TaskType* out_type) {
    for (auto it = task_queue.begin(); it != task_queue.end(); ++it) {
        bool can = false;
        if (lot_full) {
            if (it->type == TaskType::Exit && parked_count > 0) can = true;
        } else {
            if (it->type == TaskType::Park) can = true;
            else if (it->type == TaskType::Exit && parked_count > 0) can = true;
        }
        if (!can) continue;
        if (out_type) *out_type = it->type;
        task_queue.erase(it);
        task_count = static_cast<int>(task_queue.size());
        return true;
    }
    return false;
}

void ScenarioManager::applySpeedMultiplier(float speedMultiplier) {
    speed_multiplier = speedMultiplier;
    if (speedMultiplier <= 0.0f) {
        simulation_speed = 0;
        return;
    }
    simulation_speed = (int)(100.0f / speedMultiplier);
    if (simulation_speed < 0) simulation_speed = 0;
}

ScenarioManager::~ScenarioManager() {
    clearTaskQueue();
}

void simulation_set_speed_multiplier(Simulation* sim, double speed_multiplier) {
    if (!sim || !sim->scenario_manager) {
        return;
    }

    const float normalized = std::clamp(
        static_cast<float>(speed_multiplier),
        0.0f,
        MAX_SPEED_MULTIPLIER);
    sim->scenario_manager->applySpeedMultiplier(normalized);
}

namespace {

constexpr int kMinMapId = 1;
constexpr int kMaxMapId = 7;
std::string_view phase_type_name(PhaseType type) {
    return (type == PhaseType::Park) ? "park" : "exit";
}

void assign_dynamic_phase(DynamicPhase& phase, PhaseType type, int task_count) {
    phase.type = type;
    phase.task_count = std::max(1, task_count);
    phase.type_name = phase_type_name(type);
}

void reset_scenario_runtime_state(ScenarioManager& scenario) {
    scenario.clearTaskQueue();
    scenario.time_step = 0;
    scenario.num_phases = 0;
    scenario.current_phase_index = 0;
    scenario.tasks_completed_in_phase = 0;
    scenario.park_chance = 0;
    scenario.exit_chance = 0;
}

void apply_custom_config(ScenarioManager& scenario, const SimulationConfig& cfg) {
    scenario.num_phases = std::clamp(cfg.num_phases, 0, MAX_PHASES);
    for (int i = 0; i < scenario.num_phases; ++i) {
        const auto type = cfg.phases[i].type;
        assign_dynamic_phase(scenario.phases[i], type, cfg.phases[i].task_count);
    }
}

void apply_realtime_config(ScenarioManager& scenario, const SimulationConfig& cfg) {
    scenario.park_chance = cfg.realtime_park_chance;
    scenario.exit_chance = cfg.realtime_exit_chance;
}

int agv_clamp_map_id_local(int map_id) {
    return std::clamp(map_id, kMinMapId, kMaxMapId);
}

class TaskDispatchService final {
public:
    void update(Simulation* sim) const {
        if (!sim) return;
        ScenarioManager* scenario = sim->scenario_manager;
        AgentManager* agents = sim->agent_manager;
        GridMap* map = sim->map;
        Logger* logger = sim->logger;

        if (!advance_custom_phase_if_needed(scenario, logger)) return;
        generate_realtime_requests(sim, scenario, agents, map, logger);
        sim->workload_snapshot = agv_collect_agent_workload(agents);
        assign_idle_agents(sim, scenario, agents, map, logger);
    }

private:
    bool advance_custom_phase_if_needed(ScenarioManager* scenario, Logger* logger) const {
        if (scenario->mode != SimulationMode::Custom) return true;
        if (scenario->current_phase_index >= scenario->num_phases) return false;

        DynamicPhase* phase = &scenario->phases[scenario->current_phase_index];
        if (scenario->tasks_completed_in_phase < phase->task_count) return true;

        logger_log_event(logger, "Scenario", "Info", std::nullopt, scenario->current_phase_index,
            "Phase %d completed (%s %d).",
            scenario->current_phase_index + 1, phase->type_name.c_str(), phase->task_count);
        scenario->current_phase_index++;
        scenario->tasks_completed_in_phase = 0;
        if (scenario->current_phase_index < scenario->num_phases) {
            DynamicPhase* next_phase = &scenario->phases[scenario->current_phase_index];
            logger_log_event(logger, "Scenario", "Info", std::nullopt, scenario->current_phase_index,
                "Phase %d start: %s %d.",
                scenario->current_phase_index + 1, next_phase->type_name.c_str(), next_phase->task_count);
        }
        return false;
    }

    void generate_realtime_requests(
        Simulation* sim,
        ScenarioManager* scenario,
        AgentManager* agents,
        GridMap* map,
        Logger* logger) const {
        if (scenario->mode != SimulationMode::Realtime || scenario->time_step <= 0) return;

        const int roll_park = sim->nextRandomInt(500);
        const int roll_exit = sim->nextRandomInt(500);

        if (scenario->park_chance > 0 && roll_park < scenario->park_chance) {
            int before = scenario->task_count;
            if (agents->total_cars_parked < map->num_goals) {
                logger_log_event(logger, "Scenario", "Info", std::nullopt, std::nullopt,
                    "New parking request.");
                scenario->enqueueTask(TaskType::Park);
            }
            if (scenario->task_count > before) sim->requests_created_total++;
        }

        if (scenario->exit_chance > 0 && roll_exit < scenario->exit_chance) {
            int before = scenario->task_count;
            if (agents->total_cars_parked > 0) {
                logger_log_event(logger, "Scenario", "Info", std::nullopt, std::nullopt,
                    "New exit request.");
                scenario->enqueueTask(TaskType::Exit);
            }
            if (scenario->task_count > before) sim->requests_created_total++;
        }
    }

    void assign_idle_agents(
        Simulation* sim,
        ScenarioManager* scenario,
        AgentManager* agents,
        GridMap* map,
        Logger* logger) const {
        for (int i = 0; i < MAX_AGENTS; i++) {
            Agent* agent = &agents->agents[i];
            if (agent->state != AgentState::Idle || !agent->pos) continue;

            if (agent->total_distance_traveled >= DISTANCE_BEFORE_CHARGE) {
                if (agv_select_best_charge_station(agent, map, agents, logger)) {
                    agent->state = AgentState::GoingToCharge;
                } else {
                    logger_log_event(logger, "Charge", "Warn", agent->id, scenario->current_phase_index,
                        "Agent %c charge required but no station is available.", agent->symbol);
                }
                continue;
            }

            if (scenario->mode == SimulationMode::Custom) {
                if (scenario->current_phase_index >= scenario->num_phases) continue;
                DynamicPhase* phase = &scenario->phases[scenario->current_phase_index];
                if (phase->type == PhaseType::Park) {
                    if ((scenario->tasks_completed_in_phase + sim->workload_snapshot.active_park_agents) < phase->task_count &&
                        agents->total_cars_parked < map->num_goals) {
                        agent_begin_task_park(agent, scenario, logger);
                        sim->workload_snapshot.active_park_agents++;
                    }
                } else if ((scenario->tasks_completed_in_phase + sim->workload_snapshot.active_exit_agents) < phase->task_count &&
                    agents->total_cars_parked > 0) {
                    agent_begin_task_exit(agent, scenario, logger);
                    sim->workload_snapshot.active_exit_agents++;
                }
                continue;
            }

            if (scenario->mode == SimulationMode::Realtime && scenario->task_count > 0) {
                TaskType dequeued_type = TaskType::None;
                int lot_full = (agents->total_cars_parked >= map->num_goals);
                if (!scenario->tryDequeueAssignableTask(lot_full, agents->total_cars_parked, &dequeued_type)) continue;
                if (dequeued_type == TaskType::Park) agent_begin_task_park(agent, scenario, logger);
                else agent_begin_task_exit(agent, scenario, logger);
            }
        }
    }
};

const TaskDispatchService kTaskDispatchService{};

}  // namespace

SimulationConfig default_simulation_config() {
    SimulationConfig cfg{};
    cfg.seed = static_cast<unsigned int>(std::time(nullptr));
    cfg.map_id = 1;
    cfg.path_algo = PathAlgo::Default;
    cfg.mode = SimulationMode::Custom;
    cfg.speed_multiplier = 0.0f;
    cfg.num_phases = 1;
    cfg.phases[0].type = PhaseType::Park;
    cfg.phases[0].task_count = 1;
    cfg.suppress_stdout = false;
    return cfg;
}

bool apply_simulation_config(Simulation* sim, const SimulationConfig& cfg) {
    if (!sim) return false;

    sim->suppress_stdout = cfg.suppress_stdout;
    sim->render_state.suppress_flush = cfg.suppress_stdout;
    sim->reseedRandom(cfg.seed);

    sim->map_id = agv_clamp_map_id_local(cfg.map_id);
    grid_map_load_scenario(sim->map, sim->agent_manager, sim->map_id);

    sim->path_algo = cfg.path_algo;
    sim->planner = planner_from_pathalgo(sim->path_algo);
    sim->render_state.configureForAlgorithm(sim->path_algo);

    ScenarioManager& scenario = *sim->scenario_manager;
    reset_scenario_runtime_state(scenario);
    scenario.mode = cfg.mode;
    logger_log_event(sim->logger, "Control", "Info", std::nullopt, std::nullopt,
        "Scenario mode configured: %s.",
        scenario.mode == SimulationMode::Realtime ? "realtime" : "custom");

    if (scenario.mode == SimulationMode::Realtime) {
        apply_realtime_config(scenario, cfg);
    } else {
        apply_custom_config(scenario, cfg);
    }

    scenario.applySpeedMultiplier(cfg.speed_multiplier);
    logger_log_event(sim->logger, "Control", "Info", std::nullopt, std::nullopt,
        "Map #%d loaded.", sim->map_id);
    const char* algorithm_name = (sim->path_algo == PathAlgo::AStarSimple)
        ? "astar"
        : ((sim->path_algo == PathAlgo::DStarBasic) ? "dstar" : "default");
    logger_log_event(sim->logger, "Control", "Info", std::nullopt, std::nullopt,
        "Algorithm selected: %s", algorithm_name);
    sim->resetRuntimeStats();
    return true;
}

void agv_update_task_dispatch(Simulation* sim) {
    kTaskDispatchService.update(sim);
}
