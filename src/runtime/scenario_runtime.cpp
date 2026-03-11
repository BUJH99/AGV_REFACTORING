#define _CRT_SECURE_NO_WARNINGS

#include <algorithm>
#include <cctype>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <string>
#include <string_view>
#include <ctime>

#include <conio.h>
#include <windows.h>

#include "agv/internal/engine_internal.hpp"

#ifndef MAX_SPEED_MULTIPLIER
#define MAX_SPEED_MULTIPLIER 10000.0f
#endif

#ifndef MAX_TASKS
#define MAX_TASKS 50
#endif

#ifndef C_NRM
#define C_NRM "\x1b[0m"
#define C_RED "\x1b[31m"
#define C_GRN "\x1b[32m"
#define C_YEL "\x1b[33m"
#define C_CYN "\x1b[36m"
#define C_B_RED "\x1b[1;31m"
#define C_B_GRN "\x1b[1;32m"
#define C_B_YEL "\x1b[1;33m"
#define C_B_CYN "\x1b[1;36m"
#define C_B_WHT "\x1b[1;37m"
#endif

#ifndef DISTANCE_BEFORE_CHARGE
#define DISTANCE_BEFORE_CHARGE 300.0
#endif

void ui_clear_screen_optimized();
void grid_map_load_scenario(GridMap* map, AgentManager* am, int scenario_id);
Planner planner_from_pathalgo(PathAlgo algo);
void agent_begin_task_park(Agent* ag, ScenarioManager* sc, Logger* lg);
void agent_begin_task_exit(Agent* ag, ScenarioManager* sc, Logger* lg);

void agent_begin_task_park(Agent* agent, ScenarioManager* scenario, Logger* logger) {
    if (!agent || !scenario) return;
    agent->state = GOING_TO_PARK;
    agent->metrics_task_active = true;
    agent->metrics_task_start_step = scenario->time_step;
    agent->metrics_distance_at_start = agent->total_distance_traveled;
    agent->metrics_turns_current = 0;
    if (logger) logger_log(logger, "[%sTask%s] Agent %c assigned to a parking task.", C_CYN, C_NRM, agent->symbol);
}

void agent_begin_task_exit(Agent* agent, ScenarioManager* scenario, Logger* logger) {
    if (!agent || !scenario) return;
    agent->state = GOING_TO_COLLECT;
    agent->metrics_task_active = true;
    agent->metrics_task_start_step = scenario->time_step;
    agent->metrics_distance_at_start = agent->total_distance_traveled;
    agent->metrics_turns_current = 0;
    if (logger) logger_log(logger, "[%sTask%s] Agent %c assigned to a retrieval task.", C_CYN, C_NRM, agent->symbol);
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
            if (it->type == TASK_EXIT && parked_count > 0) can = true;
        } else {
            if (it->type == TASK_PARK) can = true;
            else if (it->type == TASK_EXIT && parked_count > 0) can = true;
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

namespace {

int simulation_setup_custom_scenario_local(Simulation* sim);
int simulation_setup_realtime_local(ScenarioManager* scenario);
int simulation_setup_speed_local(ScenarioManager* scenario);

void do_ms_pause(int ms) {
    Sleep(ms);
}

constexpr int kMinMapId = 1;
constexpr int kMaxMapId = 5;

const char* phase_type_name(PhaseType type) {
    return (type == PARK_PHASE) ? "park" : "exit";
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

void apply_selected_algorithm(Simulation* sim, int algorithm_choice) {
    sim->path_algo = (algorithm_choice == 2) ? PATHALGO_ASTAR_SIMPLE :
        (algorithm_choice == 3) ? PATHALGO_DSTAR_BASIC : PATHALGO_DEFAULT;
    logger_log(sim->logger, "[%sAlgo%s] Algorithm selected: %d", C_B_CYN, C_NRM, algorithm_choice);
    sim->render_state.configureForAlgorithm(sim->path_algo);
    sim->planner = planner_from_pathalgo(sim->path_algo);
}

bool run_mode_setup(Simulation* sim, char mode) {
    ScenarioManager* const scenario = sim->scenario_manager;
    switch (mode) {
    case 'a':
        scenario->mode = MODE_CUSTOM;
        return simulation_setup_custom_scenario_local(sim) &&
            simulation_setup_speed_local(scenario);
    case 'b':
        scenario->mode = MODE_REALTIME;
        return simulation_setup_realtime_local(scenario) &&
            simulation_setup_speed_local(scenario);
    case 'q':
        return false;
    default:
        return false;
    }
}

char get_single_char_local() {
    return _getch();
}

std::string_view trim_input(std::string_view input) {
    std::size_t begin = 0;
    while (begin < input.size() && std::isspace(static_cast<unsigned char>(input[begin]))) {
        ++begin;
    }

    std::size_t end = input.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(input[end - 1]))) {
        --end;
    }

    return input.substr(begin, end - begin);
}

bool try_parse_integer(std::string_view input, int& value) {
    const std::string trimmed(trim_input(input));
    if (trimmed.empty()) return false;

    try {
        std::size_t consumed = 0;
        const int parsed = std::stoi(trimmed, &consumed, 10);
        if (!trim_input(std::string_view(trimmed).substr(consumed)).empty()) {
            return false;
        }
        value = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

bool try_parse_float(std::string_view input, float& value) {
    const std::string trimmed(trim_input(input));
    if (trimmed.empty()) return false;

    try {
        std::size_t consumed = 0;
        const float parsed = std::stof(trimmed, &consumed);
        if (!trim_input(std::string_view(trimmed).substr(consumed)).empty()) {
            return false;
        }
        value = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

char get_char_input_local(const char* prompt, const char* valid) {
    char value;
    while (true) {
        agv::internal::text::console_write(prompt);
        value = (char)std::tolower(get_single_char_local());
        agv::internal::text::console_print("%c\n", value);
        if (std::strchr(valid, value)) return value;
        agv::internal::text::console_print(C_B_RED "\nInvalid input. Allowed values: (%s)\n" C_NRM, valid);
    }
}

int get_integer_input_local(const char* prompt, int min, int max) {
    std::string input;
    int value = 0;
    while (true) {
        agv::internal::text::console_write(prompt);
        if (std::getline(std::cin, input) &&
            try_parse_integer(input, value) &&
            value >= min && value <= max) {
            return value;
        }
        if (!std::cin.good()) {
            std::cin.clear();
        }
        agv::internal::text::console_print(C_B_RED "Invalid input. Enter an integer in the range %d~%d.\n" C_NRM, min, max);
    }
}

float get_float_input_local(const char* prompt, float min, float max) {
    std::string input;
    float value = 0.0f;
    while (true) {
        agv::internal::text::console_write(prompt);
        if (std::getline(std::cin, input) &&
            try_parse_float(input, value) &&
            value >= min && value <= max) {
            return value;
        }
        if (!std::cin.good()) {
            std::cin.clear();
        }
        agv::internal::text::console_print(C_B_RED "Invalid input. Enter a value in the range %.1f~%.1f.\n" C_NRM, min, max);
    }
}

int simulation_setup_custom_scenario_local(Simulation* sim) {
    ScenarioManager* scenario = sim->scenario_manager;

    agv::internal::text::console_print(C_B_WHT "--- Custom Scenario Setup ---\n" C_NRM);
    scenario->num_phases = get_integer_input_local(C_YEL "Enter phase count (1-20, 0=cancel): " C_NRM, 0, MAX_PHASES);
    if (scenario->num_phases == 0) return 0;

    int max_per_phase = (sim->map && sim->map->num_goals > 0) ? sim->map->num_goals : 100000;

    for (int i = 0; i < scenario->num_phases; i++) {
        agv::internal::text::console_print(C_B_CYN "\n--- Phase %d/%d ---\n" C_NRM, i + 1, scenario->num_phases);
        agv::internal::text::console_print("a. %sParking%s\n", C_YEL, C_NRM);
        agv::internal::text::console_print("b. %sRetrieval%s\n", C_CYN, C_NRM);
        char phase_kind = get_char_input_local("Select phase type: ", "ab");

        const std::string prompt = agv::internal::text::printf_like("Phase task count (1~%d): ", max_per_phase);
        const int task_count = get_integer_input_local(prompt.c_str(), 1, max_per_phase);
        assign_dynamic_phase(
            scenario->phases[i],
            (phase_kind == 'a') ? PARK_PHASE : EXIT_PHASE,
            task_count);

        agv::internal::text::console_print(C_GRN "Phase %d configured: %s x %d.\n" C_NRM,
            i + 1, scenario->phases[i].type_name.c_str(), scenario->phases[i].task_count);
    }

    agv::internal::text::console_print(C_B_GRN "\n--- Custom scenario configuration complete. ---\n" C_NRM);
    do_ms_pause(1500);
    return 1;
}

int simulation_setup_realtime_local(ScenarioManager* scenario) {
    agv::internal::text::console_print(C_B_WHT "--- Real-Time Scenario Setup ---\n" C_NRM);
    while (true) {
        scenario->park_chance = get_integer_input_local("\nParking request probability (0~100): ", 0, 100);
        scenario->exit_chance = get_integer_input_local("Retrieval request probability (0~100): ", 0, 100);
        if (scenario->park_chance + scenario->exit_chance <= 100) break;
        agv::internal::text::console_print(C_B_RED "The total probability must not exceed 100.\n" C_NRM);
    }
    agv::internal::text::console_print(C_B_GRN "\nReal-time configuration complete: parking=%d%%, retrieval=%d%%\n" C_NRM,
        scenario->park_chance, scenario->exit_chance);
    do_ms_pause(1500);
    return 1;
}

int simulation_setup_speed_local(ScenarioManager* scenario) {
    agv::internal::text::console_print(C_B_WHT "\n--- Simulation Speed Setup ---\n" C_NRM);

    scenario->speed_multiplier = get_float_input_local(
        "Enter speed multiplier (0.0=as fast as possible, up to 10000.0): ",
        0.0f, MAX_SPEED_MULTIPLIER);
    scenario->applySpeedMultiplier(scenario->speed_multiplier);

    agv::internal::text::console_print(C_B_GRN "\n--- %.1fx simulation speed configured. ---\n" C_NRM, scenario->speed_multiplier);
    do_ms_pause(1500);
    return 1;
}

int simulation_setup_map_local(Simulation* sim) {
    agv::internal::text::console_print(C_B_WHT "--- Select Map (1~5) ---\n" C_NRM);
    agv::internal::text::console_print("1. %sCompact parking lot%s (baseline)\n", C_B_GRN, C_NRM);
    agv::internal::text::console_print("2. %sMid-size lot with one retrieval target%s\n", C_YEL, C_NRM);
    agv::internal::text::console_print("3. %s8 AGVs + 900 requests%s (up to 16 AGVs, A~H)\n", C_YEL, C_NRM);
    agv::internal::text::console_print("4. %sDense lot with one retrieval target and four parking waves%s (up to 10 AGVs, A~J)\n", C_YEL, C_NRM);
    agv::internal::text::console_print("5. %sCharging-stress map%s (extra chargers, longer aisles, and heavy parking load)\n\n", C_YEL, C_NRM);
    int map_id = get_integer_input_local("Select map id (1~5): ", 1, 5);
    sim->map_id = map_id;
    grid_map_load_scenario(sim->map, sim->agent_manager, map_id);
    logger_log(sim->logger, "[%sMap%s] Map #%d loaded.", C_B_CYN, C_NRM, map_id);
    do_ms_pause(800);
    return 1;
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
        if (scenario->mode != MODE_CUSTOM) return true;
        if (scenario->current_phase_index >= scenario->num_phases) return false;

        DynamicPhase* phase = &scenario->phases[scenario->current_phase_index];
        if (scenario->tasks_completed_in_phase < phase->task_count) return true;

        logger_log(logger, "[%sPhase%s] %d completed (%s %d).", C_B_YEL, C_NRM,
            scenario->current_phase_index + 1, phase->type_name.c_str(), phase->task_count);
        scenario->current_phase_index++;
        scenario->tasks_completed_in_phase = 0;
        if (scenario->current_phase_index < scenario->num_phases) {
            DynamicPhase* next_phase = &scenario->phases[scenario->current_phase_index];
            logger_log(logger, "[%sPhase%s] %d start: %s %d.",
                C_B_YEL, C_NRM, scenario->current_phase_index + 1, next_phase->type_name.c_str(), next_phase->task_count);
            do_ms_pause(1500);
        }
        return false;
    }

    void generate_realtime_requests(
        Simulation* sim,
        ScenarioManager* scenario,
        AgentManager* agents,
        GridMap* map,
        Logger* logger) const {
        if (scenario->mode != MODE_REALTIME || scenario->time_step <= 0) return;

        int roll_park = std::rand() % 500;
        int roll_exit = std::rand() % 500;

        if (scenario->park_chance > 0 && roll_park < scenario->park_chance) {
            int before = scenario->task_count;
            if (agents->total_cars_parked < map->num_goals) {
                logger_log(logger, "[%sEvent%s] New parking request.", C_B_GRN, C_NRM);
                scenario->enqueueTask(TASK_PARK);
            }
            if (scenario->task_count > before) sim->requests_created_total++;
        }

        if (scenario->exit_chance > 0 && roll_exit < scenario->exit_chance) {
            int before = scenario->task_count;
            if (agents->total_cars_parked > 0) {
                logger_log(logger, "[%sEvent%s] New exit request.", C_B_YEL, C_NRM);
                scenario->enqueueTask(TASK_EXIT);
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
            if (agent->state != IDLE || !agent->pos) continue;

            if (agent->total_distance_traveled >= DISTANCE_BEFORE_CHARGE) {
                if (agv_select_best_charge_station(agent, map, agents, logger)) {
                    agent->state = GOING_TO_CHARGE;
                } else {
                    logger_log(logger, "[%sWarn%s] Agent %c charge required but no station is available.", C_YEL, C_NRM, agent->symbol);
                }
                continue;
            }

            if (scenario->mode == MODE_CUSTOM) {
                if (scenario->current_phase_index >= scenario->num_phases) continue;
                DynamicPhase* phase = &scenario->phases[scenario->current_phase_index];
                if (phase->type == PARK_PHASE) {
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

            if (scenario->mode == MODE_REALTIME && scenario->task_count > 0) {
                TaskType dequeued_type = TASK_NONE;
                int lot_full = (agents->total_cars_parked >= map->num_goals);
                if (!scenario->tryDequeueAssignableTask(lot_full, agents->total_cars_parked, &dequeued_type)) continue;
                if (dequeued_type == TASK_PARK) agent_begin_task_park(agent, scenario, logger);
                else agent_begin_task_exit(agent, scenario, logger);
            }
        }
    }
};

const TaskDispatchService kTaskDispatchService{};

}  // namespace

int simulation_setup(Simulation* sim) {
    ui_clear_screen_optimized();

    if (!simulation_setup_map_local(sim)) return 0;

    agv::internal::text::console_print(C_B_WHT "\n--- Select Path Planning Algorithm ---\n" C_NRM);
    agv::internal::text::console_print("1. %sDefault (WHCA* + D* Lite + WFG + CBS)%s\n", C_B_GRN, C_NRM);
    agv::internal::text::console_print("2. %sA* (single-agent)%s - recomputes the path from scratch each step\n", C_YEL, C_NRM);
    agv::internal::text::console_print("3. %sD* Lite (incremental)%s - reuses previous search when the map changes\n\n", C_YEL, C_NRM);
    apply_selected_algorithm(sim, get_integer_input_local("Select algorithm (1~3): ", 1, 3));

    agv::internal::text::console_print(C_B_WHT "\n--- Select Simulation Mode ---\n" C_NRM);
    agv::internal::text::console_print("a. %sCustom phased scenario%s\n", C_YEL, C_NRM);
    agv::internal::text::console_print("b. %sReal-time random scenario%s\n", C_CYN, C_NRM);
    agv::internal::text::console_print("q. %sQuit%s\n\n", C_RED, C_NRM);

    const int ok = run_mode_setup(sim, get_char_input_local("Select mode: ", "abq")) ? 1 : 0;
    if (ok) ui_clear_screen_optimized();
    return ok;
}

SimulationConfig default_simulation_config() {
    SimulationConfig cfg{};
    cfg.seed = static_cast<unsigned int>(std::time(nullptr));
    cfg.map_id = 1;
    cfg.path_algo = PATHALGO_DEFAULT;
    cfg.mode = MODE_CUSTOM;
    cfg.speed_multiplier = 0.0f;
    cfg.num_phases = 1;
    cfg.phases[0].type = PARK_PHASE;
    cfg.phases[0].task_count = 1;
    cfg.suppress_stdout = false;
    return cfg;
}

bool apply_simulation_config(Simulation* sim, const SimulationConfig& cfg) {
    if (!sim) return false;

    sim->suppress_stdout = cfg.suppress_stdout;
    sim->render_state.suppress_flush = cfg.suppress_stdout;
    sim->configured_seed = cfg.seed;
    std::srand(cfg.seed);

    sim->map_id = agv_clamp_map_id_local(cfg.map_id);
    grid_map_load_scenario(sim->map, sim->agent_manager, sim->map_id);

    sim->path_algo = cfg.path_algo;
    sim->planner = planner_from_pathalgo(sim->path_algo);
    sim->render_state.configureForAlgorithm(sim->path_algo);

    ScenarioManager& scenario = *sim->scenario_manager;
    reset_scenario_runtime_state(scenario);
    scenario.mode = cfg.mode;

    if (scenario.mode == MODE_REALTIME) {
        apply_realtime_config(scenario, cfg);
    } else {
        apply_custom_config(scenario, cfg);
    }

    scenario.applySpeedMultiplier(cfg.speed_multiplier);
    sim->resetRuntimeStats();
    return true;
}

void agv_update_task_dispatch(Simulation* sim) {
    kTaskDispatchService.update(sim);
}
