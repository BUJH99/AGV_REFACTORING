#pragma once

#include "agv/internal/engine_internal.hpp"

Node* agent_runtime_select_best_charge_station(Agent* agent, GridMap* map, AgentManager* agents, Logger* logger);
void agent_runtime_assign_goal_if_needed(Agent* agent, GridMap* map, AgentManager* agents, Logger* logger);
