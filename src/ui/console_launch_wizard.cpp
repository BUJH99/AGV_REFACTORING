#include "agv/internal/console_launch_wizard.hpp"
#include "agv/internal/engine_internal.hpp"
#include "agv/internal/launch_ui_metadata.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include <windows.h>
#include <shlobj.h>

#include <nlohmann/json.hpp>

namespace agv::internal::console {

namespace {

using nlohmann::json;

constexpr int kProtocolVersion = 1;
constexpr int kMinMapId = 1;
constexpr int kMaxMapId = 7;

enum class WizardNav {
    Advance,
    Back,
    Cancel,
};

constexpr int kWizardStepCount = 7;

std::uint32_t wizard_seed_now() {
    return static_cast<std::uint32_t>(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
}

std::string trim_copy(std::string_view input) {
    std::size_t start = 0;
    while (start < input.size() && std::isspace(static_cast<unsigned char>(input[start])) != 0) {
        ++start;
    }

    std::size_t end = input.size();
    while (end > start && std::isspace(static_cast<unsigned char>(input[end - 1])) != 0) {
        --end;
    }

    return std::string(input.substr(start, end - start));
}

std::string lowercase_copy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

bool try_parse_int(std::string_view input, int& value) {
    const std::string trimmed = trim_copy(input);
    if (trimmed.empty()) {
        return false;
    }

    try {
        std::size_t consumed = 0;
        const int parsed = std::stoi(trimmed, &consumed, 10);
        if (!trim_copy(std::string_view(trimmed).substr(consumed)).empty()) {
            return false;
        }
        value = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

bool try_parse_uint32(std::string_view input, std::uint32_t& value) {
    const std::string trimmed = trim_copy(input);
    if (trimmed.empty()) {
        return false;
    }

    try {
        std::size_t consumed = 0;
        const auto parsed = std::stoull(trimmed, &consumed, 10);
        if (!trim_copy(std::string_view(trimmed).substr(consumed)).empty() ||
            parsed > std::numeric_limits<std::uint32_t>::max()) {
            return false;
        }
        value = static_cast<std::uint32_t>(parsed);
        return true;
    } catch (...) {
        return false;
    }
}

bool try_parse_double(std::string_view input, double& value) {
    const std::string trimmed = trim_copy(input);
    if (trimmed.empty()) {
        return false;
    }

    try {
        std::size_t consumed = 0;
        const double parsed = std::stod(trimmed, &consumed);
        if (!trim_copy(std::string_view(trimmed).substr(consumed)).empty()) {
            return false;
        }
        value = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

std::string phase_label(core::PhaseType type) {
    return (type == core::PhaseType::Exit) ? "Exit" : "Park";
}

json launch_config_to_json(const core::LaunchConfig& config) {
    json phases = json::array();
    for (const auto& phase : config.scenario.phases) {
        phases.push_back({
            {"type", phase.type == core::PhaseType::Exit ? "exit" : "park"},
            {"taskCount", phase.taskCount},
        });
    }

    return json{
        {"seed", config.seed},
        {"mapId", config.mapId},
        {"algorithm", config.algorithm == core::PathAlgo::AStarSimple
            ? "astar"
            : (config.algorithm == core::PathAlgo::DStarBasic ? "dstar" : "default")},
        {"scenario", {
            {"mode", config.scenario.mode == core::SimulationMode::Realtime ? "realtime" : "custom"},
            {"speedMultiplier", config.scenario.speedMultiplier},
            {"realtimeParkChance", config.scenario.realtimeParkChance},
            {"realtimeExitChance", config.scenario.realtimeExitChance},
            {"phases", std::move(phases)},
        }},
    };
}

std::optional<core::LaunchConfig> launch_config_from_json(const json& value) {
    if (!value.is_object()) {
        return std::nullopt;
    }

    core::LaunchConfig config;
    config.seed = value.value("seed", static_cast<std::uint32_t>(0));
    config.mapId = value.value("mapId", 1);

    const std::string algorithm = value.value("algorithm", std::string("default"));
    if (algorithm == "astar" || algorithm == "AStarSimple") {
        config.algorithm = core::PathAlgo::AStarSimple;
    } else if (algorithm == "dstar" || algorithm == "DStarBasic") {
        config.algorithm = core::PathAlgo::DStarBasic;
    } else {
        config.algorithm = core::PathAlgo::Default;
    }

    if (value.contains("scenario") && value.at("scenario").is_object()) {
        const json& scenario = value.at("scenario");
        const std::string mode = scenario.value("mode", std::string("custom"));
        config.scenario.mode = (mode == "realtime") ? core::SimulationMode::Realtime : core::SimulationMode::Custom;
        config.scenario.speedMultiplier = scenario.value("speedMultiplier", 0.0);
        config.scenario.realtimeParkChance = scenario.value("realtimeParkChance", 0);
        config.scenario.realtimeExitChance = scenario.value("realtimeExitChance", 0);
        config.scenario.phases.clear();
        if (scenario.contains("phases") && scenario.at("phases").is_array()) {
            for (const json& phase : scenario.at("phases")) {
                if (!phase.is_object()) {
                    continue;
                }
                core::PhaseConfig item;
                const std::string type = phase.value("type", std::string("park"));
                item.type = (type == "exit") ? core::PhaseType::Exit : core::PhaseType::Park;
                item.taskCount = phase.value("taskCount", 1);
                config.scenario.phases.push_back(item);
            }
        }
    }

    const core::ValidationResult validation = core::validateLaunchConfig(config);
    if (!validation.ok()) {
        return std::nullopt;
    }
    return validation.normalizedConfig;
}

std::string summary_text(const core::LaunchConfig& config) {
    std::ostringstream out;
    const auto& map = launch_ui::map_option(config.mapId);
    const auto& algorithm = launch_ui::algorithm_option(config.algorithm);
    const auto& mode = launch_ui::mode_option(config.scenario.mode);

    out << "Map: " << map.id << " - " << map.label << "\n";
    out << "Algorithm: " << algorithm.label << "\n";
    out << "Mode: " << mode.label << "\n";
    if (config.scenario.mode == core::SimulationMode::Realtime) {
        out << "Realtime chances: park=" << config.scenario.realtimeParkChance
            << "% exit=" << config.scenario.realtimeExitChance << "%\n";
    } else {
        out << "Phases:";
        if (config.scenario.phases.empty()) {
            out << " (empty)\n";
        } else {
            out << "\n";
            for (std::size_t index = 0; index < config.scenario.phases.size(); ++index) {
                out << "  " << (index + 1) << ". "
                    << phase_label(config.scenario.phases[index].type)
                    << " x " << config.scenario.phases[index].taskCount << "\n";
            }
        }
    }
    out << "Speed multiplier: " << config.scenario.speedMultiplier << "\n";
    out << "Seed: " << config.seed << "\n";
    return out.str();
}

void print_step_header(
    std::ostream& output,
    int step_number,
    std::string_view title,
    const core::LaunchConfig& draft) {
    output
        << "\n[" << step_number << "/" << kWizardStepCount << "] " << title << "\n"
        << "Current draft: " << launch_ui::summarize_launch_config(draft) << "\n"
        << "Tip: Enter keeps the current value. 'b' goes back. 'q' cancels.\n";
}

std::string read_line(std::istream& input, std::ostream& output, std::string_view prompt) {
    output << prompt;
    output.flush();

    std::string line;
    std::getline(input, line);
    return line;
}

WizardNav prompt_map(std::istream& input, std::ostream& output, core::LaunchConfig& draft) {
    print_step_header(output, 1, "Map Setup", draft);
    for (const auto& option : launch_ui::map_options()) {
        output << "  " << option.id << ". " << option.label
               << " (capacity " << option.capacity << ")\n"
               << "     " << option.description << "\n";
    }

    while (true) {
        const std::string raw = lowercase_copy(read_line(
            input, output, "Select map [current " + std::to_string(draft.mapId) + "]: "));
        if (raw.empty()) {
            return WizardNav::Advance;
        }
        if (raw == "q" || raw == "quit") {
            return WizardNav::Cancel;
        }
        if (raw == "b" || raw == "back") {
            return WizardNav::Back;
        }

        int map_id = 0;
        if (try_parse_int(raw, map_id) && map_id >= kMinMapId && map_id <= kMaxMapId) {
            draft.mapId = map_id;
            return WizardNav::Advance;
        }
        output << "Invalid map id. Choose 1 through 7.\n";
    }
}

WizardNav prompt_algorithm(std::istream& input, std::ostream& output, core::LaunchConfig& draft) {
    print_step_header(output, 2, "Algorithm Setup", draft);
    int index = 1;
    for (const auto& option : launch_ui::algorithm_options()) {
        output << "  " << index++ << ". " << option.label << "\n"
               << "     " << option.description << "\n";
    }

    while (true) {
        const std::string raw = lowercase_copy(read_line(
            input, output, "Select algorithm [current " +
                std::string(launch_ui::algorithm_option(draft.algorithm).label) + "]: "));
        if (raw.empty()) {
            return WizardNav::Advance;
        }
        if (raw == "q" || raw == "quit") {
            return WizardNav::Cancel;
        }
        if (raw == "b" || raw == "back") {
            return WizardNav::Back;
        }
        if (raw == "1" || raw == "default") {
            draft.algorithm = core::PathAlgo::Default;
            return WizardNav::Advance;
        }
        if (raw == "2" || raw == "astar") {
            draft.algorithm = core::PathAlgo::AStarSimple;
            return WizardNav::Advance;
        }
        if (raw == "3" || raw == "dstar") {
            draft.algorithm = core::PathAlgo::DStarBasic;
            return WizardNav::Advance;
        }
        output << "Invalid algorithm. Choose 1, 2, or 3.\n";
    }
}

WizardNav prompt_mode(std::istream& input, std::ostream& output, core::LaunchConfig& draft) {
    print_step_header(output, 3, "Mode Setup", draft);
    int index = 1;
    for (const auto& option : launch_ui::mode_options()) {
        output << "  " << index++ << ". " << option.label << "\n"
               << "     " << option.description << "\n";
    }

    while (true) {
        const std::string raw = lowercase_copy(read_line(
            input, output, "Select mode [current " +
                std::string(launch_ui::mode_option(draft.scenario.mode).label) + "]: "));
        if (raw.empty()) {
            return WizardNav::Advance;
        }
        if (raw == "q" || raw == "quit") {
            return WizardNav::Cancel;
        }
        if (raw == "b" || raw == "back") {
            return WizardNav::Back;
        }
        if (raw == "1" || raw == "custom") {
            draft.scenario.mode = core::SimulationMode::Custom;
            return WizardNav::Advance;
        }
        if (raw == "2" || raw == "realtime") {
            draft.scenario.mode = core::SimulationMode::Realtime;
            return WizardNav::Advance;
        }
        output << "Invalid mode. Choose 1 or 2.\n";
    }
}

WizardNav prompt_custom_scenario(std::istream& input, std::ostream& output, core::LaunchConfig& draft) {
    const auto backup = draft.scenario;
    const int capacity = launch_ui::map_option(draft.mapId).capacity;
    print_step_header(output, 4, "Custom Scenario Setup", draft);
    output << "This map can schedule up to " << capacity << " vehicle(s) per phase.\n";
    output << "Use 0 phases only if you want launch-time normalization back to Park x1.\n";

    while (true) {
        const std::string raw = lowercase_copy(read_line(
            input, output,
            "Phase count [current " + std::to_string(static_cast<int>(draft.scenario.phases.size())) + "]: "));
        if (raw.empty() && !draft.scenario.phases.empty()) {
            return WizardNav::Advance;
        }
        if (raw == "q" || raw == "quit") {
            draft.scenario = backup;
            return WizardNav::Cancel;
        }
        if (raw == "b" || raw == "back") {
            draft.scenario = backup;
            return WizardNav::Back;
        }

        int phase_count = 0;
        if (!try_parse_int(raw, phase_count) || phase_count < 0 || phase_count > 20) {
            output << "Invalid phase count. Choose 0 through 20.\n";
            continue;
        }

        std::vector<core::PhaseConfig> phases;
        phases.reserve(static_cast<std::size_t>(phase_count));

        for (int index = 0; index < phase_count; ++index) {
            const bool has_current_phase = index < static_cast<int>(backup.phases.size());
            while (true) {
                std::string type_prompt =
                    std::string("Phase ") + std::to_string(index + 1) +
                    " type ('park'/'exit')";
                if (has_current_phase) {
                    type_prompt += " [current " + lowercase_copy(phase_label(backup.phases[static_cast<std::size_t>(index)].type)) + "]";
                }
                type_prompt += ": ";
                const std::string type_raw = lowercase_copy(read_line(
                    input, output, type_prompt));
                if (type_raw.empty() && has_current_phase) {
                    core::PhaseConfig phase = backup.phases[static_cast<std::size_t>(index)];
                    while (true) {
                        output
                            << "Map #" << draft.mapId
                            << " capacity: up to " << capacity
                            << " vehicle(s) can be scheduled in a single "
                            << lowercase_copy(phase_label(phase.type)) << " phase.\n";
                        std::string count_prompt =
                            std::string("Phase ") + std::to_string(index + 1) +
                            " " + lowercase_copy(phase_label(phase.type)) + " count";
                        count_prompt += " [current " + std::to_string(phase.taskCount) + "]: ";
                        const std::string count_raw = lowercase_copy(read_line(input, output, count_prompt));
                        if (count_raw.empty()) {
                            phases.push_back(phase);
                            break;
                        }
                        if (count_raw == "q" || count_raw == "quit") {
                            draft.scenario = backup;
                            return WizardNav::Cancel;
                        }
                        if (count_raw == "b" || count_raw == "back") {
                            draft.scenario = backup;
                            return WizardNav::Back;
                        }

                        int task_count = 0;
                        if (try_parse_int(count_raw, task_count) && task_count >= 1 && task_count <= capacity) {
                            phase.taskCount = task_count;
                            phases.push_back(phase);
                            break;
                        }
                        output << "Invalid count. For map #" << draft.mapId
                               << ", enter a value in the range 1~" << capacity << ".\n";
                    }
                    break;
                }
                if (type_raw == "q" || type_raw == "quit") {
                    draft.scenario = backup;
                    return WizardNav::Cancel;
                }
                if (type_raw == "b" || type_raw == "back") {
                    draft.scenario = backup;
                    return WizardNav::Back;
                }
                if (type_raw != "park" && type_raw != "exit" && type_raw != "1" && type_raw != "2") {
                    output << "Invalid phase type. Use 'park' or 'exit'.\n";
                    continue;
                }

                core::PhaseConfig phase;
                phase.type = (type_raw == "exit" || type_raw == "2")
                    ? core::PhaseType::Exit
                    : core::PhaseType::Park;

                while (true) {
                    output
                        << "Map #" << draft.mapId
                        << " capacity: up to " << capacity
                        << " vehicle(s) can be scheduled in a single "
                        << lowercase_copy(phase_label(phase.type)) << " phase.\n";
                    std::string count_prompt =
                        std::string("Phase ") + std::to_string(index + 1) +
                        " " + lowercase_copy(phase_label(phase.type)) +
                        " count (1~" + std::to_string(capacity) + ")";
                    if (has_current_phase) {
                        count_prompt += " [current " + std::to_string(backup.phases[static_cast<std::size_t>(index)].taskCount) + "]";
                    }
                    count_prompt += ": ";
                    const std::string count_raw = lowercase_copy(read_line(input, output, count_prompt));
                    if (count_raw.empty() && has_current_phase) {
                        phase.taskCount = backup.phases[static_cast<std::size_t>(index)].taskCount;
                        phases.push_back(phase);
                        break;
                    }
                    if (count_raw == "q" || count_raw == "quit") {
                        draft.scenario = backup;
                        return WizardNav::Cancel;
                    }
                    if (count_raw == "b" || count_raw == "back") {
                        draft.scenario = backup;
                        return WizardNav::Back;
                    }

                    int task_count = 0;
                    if (try_parse_int(count_raw, task_count) &&
                        task_count >= 1 &&
                        task_count <= capacity) {
                        phase.taskCount = task_count;
                        phases.push_back(phase);
                        break;
                    }
                    output
                        << "Invalid count. For map #" << draft.mapId
                        << ", enter a value in the range 1~" << capacity << ".\n";
                }
                break;
            }
        }

        draft.scenario.mode = core::SimulationMode::Custom;
        draft.scenario.phases = std::move(phases);
        return WizardNav::Advance;
    }
}

WizardNav prompt_realtime_scenario(std::istream& input, std::ostream& output, core::LaunchConfig& draft) {
    const auto backup = draft.scenario;
    print_step_header(output, 4, "Realtime Scenario Setup", draft);
    output << "Enter keeps the current value for each field. Park + exit must stay at or below 100.\n";

    while (true) {
        const std::string park_raw = lowercase_copy(read_line(
            input, output,
            "Realtime park chance (0-100) [current " +
                std::to_string(draft.scenario.realtimeParkChance) + "]: "));
        if (park_raw == "q" || park_raw == "quit") {
            draft.scenario = backup;
            return WizardNav::Cancel;
        }
        if (park_raw == "b" || park_raw == "back") {
            draft.scenario = backup;
            return WizardNav::Back;
        }

        int park = draft.scenario.realtimeParkChance;
        if (!park_raw.empty() && (!try_parse_int(park_raw, park) || park < 0 || park > 100)) {
            output << "Invalid park chance. Choose 0 through 100.\n";
            continue;
        }

        const std::string exit_raw = lowercase_copy(read_line(
            input, output,
            "Realtime exit chance (0-100) [current " +
                std::to_string(draft.scenario.realtimeExitChance) + "]: "));
        if (exit_raw == "q" || exit_raw == "quit") {
            draft.scenario = backup;
            return WizardNav::Cancel;
        }
        if (exit_raw == "b" || exit_raw == "back") {
            draft.scenario = backup;
            return WizardNav::Back;
        }

        int exit = draft.scenario.realtimeExitChance;
        if (!exit_raw.empty() && (!try_parse_int(exit_raw, exit) || exit < 0 || exit > 100)) {
            output << "Invalid exit chance. Choose 0 through 100.\n";
            continue;
        }
        if ((park + exit) > 100) {
            output << "Park chance + exit chance must not exceed 100.\n";
            continue;
        }

        draft.scenario.mode = core::SimulationMode::Realtime;
        draft.scenario.realtimeParkChance = park;
        draft.scenario.realtimeExitChance = exit;
        return WizardNav::Advance;
    }
}

WizardNav prompt_scenario(std::istream& input, std::ostream& output, core::LaunchConfig& draft) {
    if (draft.scenario.mode == core::SimulationMode::Realtime) {
        return prompt_realtime_scenario(input, output, draft);
    }
    return prompt_custom_scenario(input, output, draft);
}

WizardNav prompt_speed(std::istream& input, std::ostream& output, core::LaunchConfig& draft) {
    print_step_header(output, 5, "Speed Setup", draft);
    output << "0.0 means full-speed execution with no deliberate sleep delay.\n";
    while (true) {
        const std::string raw = lowercase_copy(read_line(
            input, output,
            "Speed multiplier (0.0-10000.0) [current " +
                std::to_string(draft.scenario.speedMultiplier) + "]: "));
        if (raw.empty()) {
            return WizardNav::Advance;
        }
        if (raw == "q" || raw == "quit") {
            return WizardNav::Cancel;
        }
        if (raw == "b" || raw == "back") {
            return WizardNav::Back;
        }

        double multiplier = 0.0;
        if (try_parse_double(raw, multiplier) && multiplier >= 0.0 && multiplier <= 10000.0) {
            draft.scenario.speedMultiplier = multiplier;
            return WizardNav::Advance;
        }
        output << "Invalid speed multiplier. Choose 0.0 through 10000.0.\n";
    }
}

WizardNav prompt_seed(std::istream& input, std::ostream& output, core::LaunchConfig& draft) {
    print_step_header(output, 6, "Seed Setup", draft);
    output << "Same seed + same config reproduces the same random run.\n";
    while (true) {
        const std::string raw = lowercase_copy(read_line(
            input, output,
            "Seed (0-4294967295) [current " +
                std::to_string(draft.seed) + "]: "));
        if (raw.empty()) {
            return WizardNav::Advance;
        }
        if (raw == "q" || raw == "quit") {
            return WizardNav::Cancel;
        }
        if (raw == "b" || raw == "back") {
            return WizardNav::Back;
        }

        std::uint32_t seed = 0;
        if (try_parse_uint32(raw, seed)) {
            draft.seed = seed;
            return WizardNav::Advance;
        }
        output << "Invalid seed.\n";
    }
}

WizardNav prompt_summary(std::istream& input, std::ostream& output, core::LaunchConfig& draft) {
    const core::ValidationResult validation = core::validateLaunchConfig(draft);

    print_step_header(output, 7, "Summary", validation.normalizedConfig);
    output << summary_text(validation.normalizedConfig);
    for (const auto& warning : validation.warnings) {
        output << "Warning [" << warning.field << "]: " << warning.message << "\n";
    }
    for (const auto& error : validation.errors) {
        output << "Error [" << error.field << "]: " << error.message << "\n";
    }

    while (true) {
        const std::string raw = lowercase_copy(read_line(
            input, output, "Select action (Enter/start=run, 2/back, 3/cancel): "));
        if (raw.empty() || raw == "1" || raw == "start") {
            if (!validation.ok()) {
                output << "Cannot start until the validation errors are fixed.\n";
                continue;
            }
            draft = validation.normalizedConfig;
            return WizardNav::Advance;
        }
        if (raw == "2" || raw == "back" || raw == "b") {
            return WizardNav::Back;
        }
        if (raw == "3" || raw == "cancel" || raw == "q") {
            return WizardNav::Cancel;
        }
        output << "Invalid action.\n";
    }
}

bool run_guided_setup(std::istream& input, std::ostream& output, core::LaunchConfig& draft) {
    int step = 0;
    while (true) {
        WizardNav nav = WizardNav::Cancel;
        switch (step) {
            case 0:
                nav = prompt_map(input, output, draft);
                break;
            case 1:
                nav = prompt_algorithm(input, output, draft);
                break;
            case 2:
                nav = prompt_mode(input, output, draft);
                break;
            case 3:
                nav = prompt_scenario(input, output, draft);
                break;
            case 4:
                nav = prompt_speed(input, output, draft);
                break;
            case 5:
                nav = prompt_seed(input, output, draft);
                break;
            case 6:
                nav = prompt_summary(input, output, draft);
                break;
            default:
                return true;
        }

        if (nav == WizardNav::Cancel) {
            return false;
        }
        if (nav == WizardNav::Back) {
            if (step == 0) {
                output << "Already at the first step.\n";
            } else {
                --step;
            }
            continue;
        }
        ++step;
        if (step > 6) {
            return true;
        }
    }
}

}  // namespace

std::filesystem::path default_last_launch_path() {
    PWSTR raw_path = nullptr;
    std::filesystem::path root;
    if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_LocalAppData, KF_FLAG_DEFAULT, nullptr, &raw_path)) &&
        raw_path != nullptr) {
        root = std::filesystem::path(raw_path);
        CoTaskMemFree(raw_path);
    } else if (const char* env = std::getenv("LOCALAPPDATA")) {
        root = std::filesystem::path(env);
    } else {
        root = std::filesystem::temp_directory_path();
    }

    return root / "AGVRefactor" / "last_launch.json";
}

std::optional<core::LaunchConfig> load_last_launch_config(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input.is_open()) {
        return std::nullopt;
    }

    try {
        json payload = json::parse(input);
        if (!payload.is_object() || payload.value("version", 0) != kProtocolVersion || !payload.contains("launchConfig")) {
            return std::nullopt;
        }
        return launch_config_from_json(payload.at("launchConfig"));
    } catch (...) {
        return std::nullopt;
    }
}

bool save_last_launch_config(const std::filesystem::path& path, const core::LaunchConfig& config) {
    const std::filesystem::path directory = path.parent_path();
    std::error_code error;
    if (!directory.empty()) {
        std::filesystem::create_directories(directory, error);
        if (error) {
            return false;
        }
    }

    const std::filesystem::path temp_path = path.string() + ".tmp";
    std::ofstream output(temp_path, std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
        return false;
    }

    output << json{
        {"version", kProtocolVersion},
        {"launchConfig", launch_config_to_json(config)},
    }.dump(2);
    output.close();
    if (!output) {
        std::filesystem::remove(temp_path, error);
        return false;
    }

    std::filesystem::remove(path, error);
    error.clear();
    std::filesystem::rename(temp_path, path, error);
    if (error) {
        std::filesystem::remove(temp_path, error);
        return false;
    }

    return true;
}

bool run_console_launch_wizard(
    std::istream& input,
    std::ostream& output,
    const std::filesystem::path& last_launch_path,
    core::LaunchConfig& out_config) {
    const std::uint32_t seed = wizard_seed_now();
    const core::LaunchConfig recommended = launch_ui::recommended_launch_config(seed);
    const std::optional<core::LaunchConfig> last_used = load_last_launch_config(last_launch_path);

    while (true) {
        output
            << "\nAGV Launch\n"
            << "  1. Recommended\n"
            << "     " << launch_ui::summarize_launch_config(recommended) << "\n"
            << "     Balanced starter setup for quick validation runs.\n"
            << "  2. Last Used\n";
        if (last_used.has_value()) {
            output << "     " << launch_ui::summarize_launch_config(*last_used) << "\n";
            output << "     Reopen the most recent successful launch.\n";
        } else {
            output << "     unavailable - falls back to Recommended\n";
        }
        output
            << "  3. Guided Setup\n"
            << "     Walk through each setting with validation and current-value hints.\n"
            << "  4. Quit\n"
            << "     Exit without starting a session.\n"
            << "Tip: Press Enter to launch Recommended immediately.\n";
        const std::string raw = lowercase_copy(read_line(input, output, "Select option (1-4): "));
        if (raw.empty() || raw == "1" || raw == "recommended") {
            out_config = recommended;
            return true;
        }
        if (raw == "2" || raw == "last" || raw == "last used") {
            if (last_used.has_value()) {
                out_config = *last_used;
            } else {
                output << "Last used configuration is unavailable. Falling back to Recommended.\n";
                out_config = recommended;
            }
            return true;
        }
        if (raw == "3" || raw == "guided" || raw == "guided setup") {
            core::LaunchConfig draft = recommended;
            if (run_guided_setup(input, output, draft)) {
                out_config = draft;
                return true;
            }
            output << "Setup canceled. Returning to launch menu.\n";
            continue;
        }
        if (raw == "4" || raw == "q" || raw == "quit") {
            return false;
        }
        output << "Invalid option.\n";
    }
}

}  // namespace agv::internal::console
