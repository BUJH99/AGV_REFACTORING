function clampTaskCount(value, maxCount) {
  const parsedValue = Number.parseInt(value ?? 1, 10);
  if (!Number.isFinite(parsedValue) || parsedValue < 1) {
    return 1;
  }
  return Math.min(parsedValue, Math.max(1, maxCount));
}

function normalizePhase(rawPhase, fallbackType = "park", maxCount = Number.MAX_SAFE_INTEGER) {
  const type = rawPhase?.type === "exit" ? "exit" : fallbackType;
  return {
    type,
    taskCount: clampTaskCount(rawPhase?.taskCount, maxCount),
  };
}

function normalizePhases(phases, maxCount = Number.MAX_SAFE_INTEGER) {
  const normalized = Array.isArray(phases)
    ? phases.map((phase) => normalizePhase(phase, "park", maxCount))
    : [];
  return normalized.length > 0
    ? normalized
    : [normalizePhase({ type: "park", taskCount: 1 }, "park", maxCount)];
}

function createDefaultCustomPhases(maxCount = Number.MAX_SAFE_INTEGER) {
  return normalizePhases(
    [
      { type: "park", taskCount: 8 },
      { type: "exit", taskCount: 4 },
    ],
    maxCount,
  );
}

const kLogRingSize = 8;
const kStatusRingSize = 8;
const kRunStepsPerTick = 1;
const kRunFrameDelayMs = 48;
const kChargeDistanceThreshold = 300.0;
const kImportantLogCategories = new Set(["Control", "Scenario", "Deadlock", "Charge"]);
const kAgentPalette = [
  "#2563eb",
  "#7c3aed",
  "#0f9b71",
  "#ef4444",
  "#f59e0b",
  "#14b8a6",
  "#e11d48",
  "#6366f1",
  "#0891b2",
  "#10b981",
  "#9333ea",
  "#f97316",
];

const hudKeys = [
  ["Map", (frame) => mapPillText(frame?.hud?.mapId)],
  ["Algorithm", (frame) => titleCase(frame?.hud?.algorithm ?? "-")],
  ["Mode", (frame) => titleCase(frame?.hud?.mode ?? "-")],
  ["Step", (frame) => String(frame?.hud?.step ?? "-")],
  ["Queued", (frame) => String(frame?.hud?.queuedTaskCount ?? "-")],
  ["In flight", (frame) => String(frame?.hud?.inFlightTaskCount ?? "-")],
  ["Parked", (frame) => String(frame?.hud?.parkedCars ?? "-")],
  ["Waiting", (frame) => String(frame?.hud?.waitingAgentCount ?? "-")],
];

const metricKeys = [
  ["Tasks completed", (metrics) => String(metrics?.tasksCompletedTotal ?? "-"), "primary"],
  ["Throughput", (metrics) => formatNumber(metrics?.throughput, 2), "primary"],
  ["Avg CPU ms", (metrics) => formatNumber(metrics?.avgCpuTimeMs, 2), "neutral"],
  ["Avg planning ms", (metrics) => formatNumber(metrics?.avgPlanningTimeMs, 2), "primary"],
  ["Deadlocks", (metrics) => String(metrics?.deadlockCount ?? "-"), "danger"],
  ["Outstanding", (metrics) => String(metrics?.outstandingTaskCount ?? "-"), "warning"],
  ["Wait edges / step", (metrics) => formatNumber(metrics?.plannerWaitEdgesPerStep, 2), "warning"],
  ["Conflict cycles", (metrics) => String(metrics?.plannerConflictCycleTotal ?? "-"), "warning"],
];

const state = {
  capabilities: null,
  mapOptions: [],
  backendPath: null,
  sessionId: null,
  captureLevel: "frame",
  scene: null,
  frame: null,
  metrics: null,
  paused: true,
  busy: false,
  running: false,
  logs: [],
  lastLogSeq: 0,
  statusLines: [],
  customPhases: createDefaultCustomPhases(),
  selectedAgentId: null,
  hoveredAgentId: null,
  showLayers: {
    semantic: true,
    agents: true,
    debug: true,
  },
  uiVisible: true,
  isSetupOpen: true,
  isMetricsOpen: false,
  isDebugExpanded: false,
  canvasView: null,
};

const elements = {
  appShell: document.getElementById("app-shell"),
  mapId: document.getElementById("map-id"),
  algorithm: document.getElementById("algorithm"),
  mode: document.getElementById("mode"),
  captureLevel: document.getElementById("capture-level"),
  speedMultiplier: document.getElementById("speed-multiplier"),
  seed: document.getElementById("seed"),
  realtimePark: document.getElementById("realtime-park"),
  realtimeExit: document.getElementById("realtime-exit"),
  mapCapacityHelp: document.getElementById("map-capacity-help"),
  mapInfoBanner: document.getElementById("map-info-banner"),
  phaseList: document.getElementById("phase-list"),
  phaseSummary: document.getElementById("phase-summary"),
  phaseCountBadge: document.getElementById("phase-count-badge"),
  scenarioCapacityBanner: document.getElementById("scenario-capacity-banner"),
  addParkPhaseButton: document.getElementById("add-park-phase"),
  addExitPhaseButton: document.getElementById("add-exit-phase"),
  resetPhasesButton: document.getElementById("reset-phases"),
  viewportEmpty: document.getElementById("viewport-empty"),
  viewportStartButton: document.getElementById("viewport-start-button"),
  viewportSetupButton: document.getElementById("viewport-setup-button"),
  launchSummaryEmpty: document.getElementById("launch-summary-empty"),
  launchSummaryDrawer: document.getElementById("launch-summary-drawer"),
  runControlHelp: document.getElementById("run-control-help"),
  sessionPill: document.getElementById("session-pill"),
  mapPill: document.getElementById("map-pill"),
  framePill: document.getElementById("frame-pill"),
  stepPill: document.getElementById("step-pill"),
  topStatGrid: document.getElementById("top-stat-grid"),
  liveIndicator: document.getElementById("live-indicator"),
  plannerSummaryGrid: document.getElementById("planner-summary-grid"),
  fleetSummary: document.getElementById("fleet-summary"),
  agentRosterList: document.getElementById("agent-roster-list"),
  telemetryContent: document.getElementById("telemetry-content"),
  modalSummaryGrid: document.getElementById("modal-summary-grid"),
  metricsGrid: document.getElementById("metrics-grid"),
  logsList: document.getElementById("logs-list"),
  statusList: document.getElementById("status-list"),
  debugCard: document.getElementById("debug-card"),
  debugOutput: document.getElementById("debug-output"),
  canvas: document.getElementById("scene-canvas"),
  routeHud: document.getElementById("route-hud"),
  routeAgentTitle: document.getElementById("route-agent-title"),
  routeGoal: document.getElementById("route-goal"),
  routeMeta: document.getElementById("route-meta"),
  routeWarning: document.getElementById("route-warning"),
  validateButton: document.getElementById("validate-button"),
  startButton: document.getElementById("start-button"),
  runButton: document.getElementById("run-button"),
  pauseButton: document.getElementById("pause-button"),
  stepButton: document.getElementById("step-button"),
  resetSessionButton: document.getElementById("reset-session-button"),
  debugButton: document.getElementById("debug-button"),
  debugToggleButton: document.getElementById("debug-toggle-button"),
  toggleUiButton: document.getElementById("toggle-ui-button"),
  openMetricsButton: document.getElementById("open-metrics-button"),
  closeMetricsButton: document.getElementById("close-metrics-button"),
  openSetupButton: document.getElementById("open-setup-button"),
  closeSetupButton: document.getElementById("close-setup-button"),
  setupBackdrop: document.getElementById("setup-backdrop"),
  setupDrawer: document.getElementById("setup-drawer"),
  metricsBackdrop: document.getElementById("metrics-backdrop"),
  metricsModal: document.getElementById("metrics-modal"),
  semanticLayerButton: document.getElementById("semantic-layer-button"),
  agentLayerButton: document.getElementById("agent-layer-button"),
  debugLayerButton: document.getElementById("debug-layer-button"),
};

function formatNumber(value, fractionDigits = 1) {
  if (value === null || value === undefined || Number.isNaN(value)) {
    return "-";
  }
  return Number(value).toFixed(fractionDigits);
}

function titleCase(value) {
  if (!value) {
    return "";
  }
  return String(value)
    .replace(/_/g, " ")
    .replace(/\b\w/g, (letter) => letter.toUpperCase());
}

function mapPillText(mapId) {
  if (mapId === null || mapId === undefined || mapId === "-") {
    return "Map -";
  }
  const option = state.mapOptions.find((entry) => Number(entry.id) === Number(mapId));
  return option ? `Map ${option.id}` : `Map ${mapId}`;
}

function agentStateShortLabel(value) {
  switch (value) {
    case "going_to_park":
      return "TO_PARK";
    case "returning_home_empty":
      return "HOME_EMPTY";
    case "going_to_collect":
      return "TO_PICK";
    case "returning_with_car":
      return "RETURN_CAR";
    case "going_to_charge":
      return "TO_CHARGE";
    case "charging":
      return "CHARGING";
    case "returning_home_maintenance":
      return "HOME_MAINT";
    case "idle":
    default:
      return "IDLE";
  }
}

function waitReasonShortLabel(value) {
  switch (value) {
    case "idle":
      return "IDLE";
    case "charging":
      return "CHG";
    case "goal_action":
      return "ACT";
    case "rotation":
      return "ROT";
    case "blocked_by_stationary":
      return "BLK";
    case "priority_yield":
      return "YLD";
    case "stuck":
      return "STK";
    case "oscillating":
      return "OSC";
    case "none":
    default:
      return "-";
  }
}

function waitReasonClass(value) {
  const short = waitReasonShortLabel(value).toLowerCase();
  if (short === "-") {
    return "none";
  }
  return short;
}

function coordText(coord) {
  if (!coord || coord.x === undefined || coord.y === undefined || coord.x < 0 || coord.y < 0) {
    return "-";
  }
  return `(${coord.x},${coord.y})`;
}

function taskSummary(agent) {
  if (!agent?.taskActive) {
    return "-";
  }
  return `${agent.taskAgeSteps}/${Number(agent.taskDistance || 0).toFixed(1)}/${agent.taskTurns}`;
}

function timerSummary(agent) {
  return `${agent?.chargeTimer || 0}/${agent?.actionTimer || 0}/${agent?.rotationWait || 0}`;
}

function chargeReservePercent(agent) {
  if (!agent) {
    return 0;
  }
  if ((agent.chargeTimer || 0) > 0 || agent.state === "charging") {
    return 100;
  }
  const traveled = Math.max(0, Number(agent.totalDistanceTraveled || 0));
  const remaining = Math.max(0, kChargeDistanceThreshold - traveled);
  return Math.round((remaining / kChargeDistanceThreshold) * 100);
}

function chargeReserveTone(agent) {
  if ((agent?.chargeTimer || 0) > 0 || agent?.state === "charging") {
    return "charging";
  }
  const percent = chargeReservePercent(agent);
  if (agent?.state === "going_to_charge" || percent <= 15) {
    return "danger";
  }
  if (percent <= 35) {
    return "warning";
  }
  return "healthy";
}

function chargeReserveMeta(agent) {
  const remainingDistance = Math.max(
    0,
    kChargeDistanceThreshold - Math.max(0, Number(agent?.totalDistanceTraveled || 0)),
  );
  if ((agent?.chargeTimer || 0) > 0 || agent?.state === "charging") {
    return `Charging now · ${agent?.chargeTimer || 0} steps left`;
  }
  if (agent?.state === "going_to_charge") {
    return `${remainingDistance.toFixed(0)} cells left until charge cutoff`;
  }
  return `${remainingDistance.toFixed(0)} cells left until charge cutoff`;
}

function shouldIncludeConsoleTail(entry) {
  if (!entry) {
    return false;
  }
  const level = String(entry.level || "").toLowerCase();
  if (level.includes("warn") || level.includes("error")) {
    return true;
  }
  return kImportantLogCategories.has(entry.category);
}

function currentAgents() {
  return Array.isArray(state.frame?.agents) ? state.frame.agents : [];
}

function currentMapOption() {
  const selectedMapId = Number(elements.mapId.value || 1);
  return state.mapOptions.find((option) => Number(option.id) === selectedMapId) || null;
}

function currentMapCapacity() {
  return Math.max(1, Number(currentMapOption()?.capacity || 1));
}

function currentMapDescription() {
  const option = currentMapOption();
  return option?.description || "Use this map to inspect planner behavior and AGV flow.";
}

function selectedAgent() {
  return currentAgents().find((agent) => agent.id === state.selectedAgentId) || null;
}

function hoveredAgent() {
  return currentAgents().find((agent) => agent.id === state.hoveredAgentId) || null;
}

function displayedAgent() {
  return hoveredAgent() || selectedAgent();
}

function activeAgentCount() {
  return currentAgents().filter((agent) => agent.isActive).length;
}

function waitingAgentCount() {
  return currentAgents().filter((agent) => agent.waitReason && agent.waitReason !== "none").length;
}

function stuckAgentCount() {
  return currentAgents().filter((agent) => agent.waitReason === "stuck").length;
}

function oscillatingAgentCount() {
  return currentAgents().filter((agent) => agent.waitReason === "oscillating").length;
}

function deterministicAgentColor(agent) {
  const seed = Number(agent?.id || 0) + String(agent?.symbol || "").charCodeAt(0);
  return kAgentPalette[Math.abs(seed) % kAgentPalette.length];
}

function hexToRgba(hex, alpha) {
  const normalized = String(hex || "").replace("#", "");
  if (normalized.length !== 6) {
    return `rgba(37, 99, 235, ${alpha})`;
  }
  const red = Number.parseInt(normalized.slice(0, 2), 16);
  const green = Number.parseInt(normalized.slice(2, 4), 16);
  const blue = Number.parseInt(normalized.slice(4, 6), 16);
  return `rgba(${red}, ${green}, ${blue}, ${alpha})`;
}

function createSummaryPill(text) {
  const pill = document.createElement("span");
  pill.className = "pill neutral";
  pill.textContent = text;
  return pill;
}

function createMiniMetric(label, value, tone = "neutral") {
  const card = document.createElement("div");
  card.className = "mini-stat";
  card.dataset.tone = tone;

  const metricLabel = document.createElement("div");
  metricLabel.className = "mini-stat-label";
  metricLabel.textContent = label;

  const metricValue = document.createElement("div");
  metricValue.className = "mini-stat-value";
  metricValue.textContent = value;

  card.append(metricLabel, metricValue);
  return card;
}

function createMetricTile(label, value, tone = "neutral") {
  const tile = document.createElement("div");
  tile.className = "metric-tile";
  tile.dataset.tone = tone;

  const tileLabel = document.createElement("div");
  tileLabel.className = "metric-label";
  tileLabel.textContent = label;

  const tileValue = document.createElement("div");
  tileValue.className = "metric-value";
  tileValue.textContent = value;

  tile.append(tileLabel, tileValue);
  return tile;
}

function createSummaryCard(label, value) {
  const card = document.createElement("div");
  card.className = "summary-card";

  const cardLabel = document.createElement("div");
  cardLabel.className = "summary-label";
  cardLabel.textContent = label;

  const cardValue = document.createElement("div");
  cardValue.className = "summary-value";
  cardValue.textContent = value;

  card.append(cardLabel, cardValue);
  return card;
}

function logLevelClass(level) {
  const normalized = String(level || "").toLowerCase();
  if (normalized.includes("error")) return "error";
  if (normalized.includes("warn")) return "warn";
  return "info";
}

function formatStepTag(step) {
  return `s${String(Math.max(0, Number(step || 0))).padStart(6, "0")}`;
}

function formatStructuredLogLine(entry) {
  const prefix = [formatStepTag(entry?.step), entry?.category || "General"];
  if (entry?.agentId !== null && entry?.agentId !== undefined) {
    prefix.push(`A${entry.agentId}`);
  }
  if (entry?.phaseIndex !== null && entry?.phaseIndex !== undefined) {
    prefix.push(`P${entry.phaseIndex}`);
  }
  return `[${prefix.join("][")}] ${entry?.text || ""}`;
}

function formatStructuredLogScope(entry) {
  const scope = [entry?.category || "General"];
  if (entry?.agentId !== null && entry?.agentId !== undefined) {
    scope.push(`A${entry.agentId}`);
  }
  if (entry?.phaseIndex !== null && entry?.phaseIndex !== undefined) {
    scope.push(`P${entry.phaseIndex}`);
  }
  return `[${scope.join("/")}]`;
}

function appendStatus(message) {
  state.statusLines.unshift({
    time: new Date().toLocaleTimeString(),
    text: message,
  });
  state.statusLines = state.statusLines.slice(0, kStatusRingSize);
  renderStatusFeed();
}

function populateSelect(select, values) {
  select.innerHTML = "";
  values.forEach((value) => {
    const option = document.createElement("option");
    option.value = String(value);
    option.textContent = titleCase(String(value));
    select.append(option);
  });
}

function populateMapSelect(mapOptions) {
  elements.mapId.innerHTML = "";
  mapOptions.forEach((option) => {
    const node = document.createElement("option");
    node.value = String(option.id);
    node.textContent = `${option.id} - ${option.label}`;
    elements.mapId.append(node);
  });
}

function modeValue() {
  return elements.mode.value || "custom";
}

function phaseStats() {
  return state.customPhases.reduce(
    (totals, phase) => {
      totals.phases += 1;
      totals[phase.type] += phase.taskCount;
      return totals;
    },
    { phases: 0, park: 0, exit: 0 },
  );
}

function updateMapCapacityUi() {
  const option = currentMapOption();
  const maxCount = currentMapCapacity();
  if (option) {
    elements.mapCapacityHelp.textContent =
      `Map ${option.id} · ${option.label} · per-phase limit 1-${maxCount} vehicles`;
    elements.mapInfoBanner.textContent = option.description
      ? `${option.description} Maximum ${maxCount} vehicles per parking or exit phase.`
      : `This map allows up to ${maxCount} vehicles in each parking or exit phase.`;
  } else {
    elements.mapCapacityHelp.textContent = `Per-phase limit 1-${maxCount} vehicles`;
    elements.mapInfoBanner.textContent = currentMapDescription();
  }

  elements.scenarioCapacityBanner.textContent =
    `This map can schedule up to ${maxCount} vehicle(s) in each parking or exit phase.`;
}

function renderPhaseSummary() {
  const stats = phaseStats();
  elements.phaseCountBadge.textContent = `${stats.phases} phase${stats.phases === 1 ? "" : "s"}`;
  elements.phaseSummary.innerHTML = "";
  [
    `Total phases ${stats.phases}`,
    `Parking ${stats.park}`,
    `Exit ${stats.exit}`,
  ].forEach((text) => {
    elements.phaseSummary.append(createSummaryPill(text));
  });
}

function renderLaunchSummaryInto(container) {
  if (!container) {
    return;
  }

  const option = currentMapOption();
  const capacity = currentMapCapacity();
  const summaryItems = [
    {
      label: "Map",
      value: option ? `Map ${option.id} · ${option.label}` : `Map ${elements.mapId.value || "1"}`,
    },
    {
      label: "Algorithm",
      value: titleCase(elements.algorithm.value || "default"),
    },
    {
      label: "Mode",
      value: modeValue() === "realtime" ? "Realtime request generation" : "Custom phase plan",
    },
    {
      label: "Capture",
      value: titleCase(elements.captureLevel.value || state.captureLevel),
    },
  ];

  if (modeValue() === "custom") {
    const stats = phaseStats();
    summaryItems.push({
      label: "Scenario",
      value: `${stats.phases} phases · parking ${stats.park} · exit ${stats.exit} · max ${capacity}/phase`,
    });
  } else {
    summaryItems.push({
      label: "Realtime mix",
      value: `Park ${elements.realtimePark.value || 0}% / Exit ${elements.realtimeExit.value || 0}%`,
    });
  }

  container.innerHTML = "";
  summaryItems.forEach((item) => {
    container.append(createSummaryCard(item.label, item.value));
  });
}

function renderLaunchSummaries() {
  renderLaunchSummaryInto(elements.launchSummaryEmpty);
  renderLaunchSummaryInto(elements.launchSummaryDrawer);
}

function syncModeVisibility() {
  const mode = modeValue();
  document.querySelectorAll(".mode-group").forEach((group) => {
    group.classList.toggle("hidden", group.dataset.mode !== mode);
  });
}

function syncCustomPhasesForMap(options = {}) {
  const maxCount = currentMapCapacity();
  const before = JSON.stringify(state.customPhases);
  state.customPhases = normalizePhases(state.customPhases, maxCount);
  const after = JSON.stringify(state.customPhases);
  updateMapCapacityUi();
  renderPhaseEditor();
  renderLaunchSummaries();
  if (options.announce && before !== after) {
    appendStatus(`Scenario counts were clamped to this map's per-phase limit of ${maxCount}.`);
  }
}

function updatePhase(index, updates) {
  state.customPhases[index] = normalizePhase(
    {
      ...state.customPhases[index],
      ...updates,
    },
    "park",
    currentMapCapacity(),
  );
  renderPhaseEditor();
  renderLaunchSummaries();
}

function addPhase(type) {
  state.customPhases.push(
    normalizePhase(
      {
        type,
        taskCount: type === "park" ? 8 : 4,
      },
      type,
      currentMapCapacity(),
    ),
  );
  renderPhaseEditor();
  renderLaunchSummaries();
}

function removePhase(index) {
  state.customPhases.splice(index, 1);
  state.customPhases = normalizePhases(state.customPhases, currentMapCapacity());
  renderPhaseEditor();
  renderLaunchSummaries();
}

function movePhase(index, direction) {
  const nextIndex = index + direction;
  if (nextIndex < 0 || nextIndex >= state.customPhases.length) {
    return;
  }

  const phases = [...state.customPhases];
  const [phase] = phases.splice(index, 1);
  phases.splice(nextIndex, 0, phase);
  state.customPhases = phases;
  renderPhaseEditor();
  renderLaunchSummaries();
}

function createMiniButton(label, title, disabled, onClick) {
  const button = document.createElement("button");
  button.type = "button";
  button.className = "ghost-button compact-button mini-button";
  button.textContent = label;
  button.title = title;
  button.disabled = disabled;
  button.addEventListener("mousedown", (event) => {
    event.preventDefault();
  });
  button.addEventListener("click", onClick);
  return button;
}

function renderPhaseEditor() {
  renderPhaseSummary();
  elements.phaseList.innerHTML = "";
  const maxCount = currentMapCapacity();

  state.customPhases.forEach((phase, index) => {
    const row = document.createElement("article");
    row.className = "phase-row";

    const head = document.createElement("div");
    head.className = "phase-row-head";

    const stepPill = document.createElement("span");
    stepPill.className = "phase-index";
    stepPill.textContent = `Phase ${index + 1}`;

    const controls = document.createElement("div");
    controls.className = "phase-controls";
    controls.append(
      createMiniButton("Up", "Move phase up", index === 0, () => movePhase(index, -1)),
      createMiniButton(
        "Down",
        "Move phase down",
        index === state.customPhases.length - 1,
        () => movePhase(index, 1),
      ),
      createMiniButton(
        "Remove",
        "Remove phase",
        state.customPhases.length === 1,
        () => removePhase(index),
      ),
    );

    head.append(stepPill, controls);

    const grid = document.createElement("div");
    grid.className = "phase-row-grid";

    const typeLabel = document.createElement("label");
    const typeText = document.createElement("span");
    typeText.textContent = "Phase type";
    const typeSelect = document.createElement("select");
    ["park", "exit"].forEach((type) => {
      const option = document.createElement("option");
      option.value = type;
      option.textContent = type === "park" ? "Parking" : "Exit";
      option.selected = phase.type === type;
      typeSelect.append(option);
    });
    typeSelect.addEventListener("change", (event) => {
      updatePhase(index, { type: event.target.value });
    });
    typeLabel.append(typeText, typeSelect);

    const countLabel = document.createElement("label");
    const countText = document.createElement("span");
    countText.textContent =
      phase.type === "park"
        ? `Cars to park (1-${maxCount})`
        : `Cars to exit (1-${maxCount})`;
    const countInput = document.createElement("input");
    countInput.type = "number";
    countInput.min = "1";
    countInput.max = String(maxCount);
    countInput.step = "1";
    countInput.inputMode = "numeric";
    countInput.value = String(phase.taskCount);
    const commitCount = () => {
      updatePhase(index, {
        taskCount: clampTaskCount(countInput.value, maxCount),
      });
    };
    countInput.addEventListener("change", commitCount);
    countInput.addEventListener("blur", commitCount);
    countInput.addEventListener("keydown", (event) => {
      if (event.key === "Enter") {
        event.preventDefault();
        commitCount();
        countInput.blur();
      }
    });

    const countRow = document.createElement("div");
    countRow.className = "phase-count-row";

    const maxButton = createMiniButton(
      "Max",
      `Fill to the map limit (${maxCount})`,
      phase.taskCount === maxCount,
      () => {
        countInput.value = String(maxCount);
        updatePhase(index, { taskCount: maxCount });
      },
    );

    countRow.append(countInput, maxButton);
    countLabel.append(countText, countRow);

    grid.append(typeLabel, countLabel);
    row.append(head, grid);
    elements.phaseList.append(row);
  });
}

function buildLaunchConfig() {
  const mode = modeValue();
  const launchConfig = {
    seed: Number(elements.seed.value || 0),
    mapId: Number(elements.mapId.value || 1),
    algorithm: elements.algorithm.value,
    scenario: {
      mode,
      speedMultiplier: Number(elements.speedMultiplier.value || 0),
    },
  };

  if (mode === "custom") {
    launchConfig.scenario.phases = normalizePhases(state.customPhases, currentMapCapacity());
  } else {
    launchConfig.scenario.realtimeParkChance = Number(elements.realtimePark.value || 0);
    launchConfig.scenario.realtimeExitChance = Number(elements.realtimeExit.value || 0);
  }

  return launchConfig;
}

function setUiVisible(visible) {
  state.uiVisible = Boolean(visible);
  elements.appShell.classList.toggle("ui-hidden", !state.uiVisible);
  elements.toggleUiButton.textContent = state.uiVisible ? "Hide UI" : "Show UI";
}

function setSetupOpen(open) {
  state.isSetupOpen = Boolean(open);
  elements.setupBackdrop.classList.toggle("hidden", !state.isSetupOpen);
  elements.setupDrawer.classList.toggle("open", state.isSetupOpen);
}

function setMetricsOpen(open) {
  state.isMetricsOpen = Boolean(open);
  elements.metricsBackdrop.classList.toggle("hidden", !state.isMetricsOpen);
  elements.metricsModal.classList.toggle("hidden", !state.isMetricsOpen);
}

function setDebugExpanded(open) {
  state.isDebugExpanded = Boolean(open);
  elements.debugOutput.classList.toggle("hidden", !state.isDebugExpanded);
  elements.debugToggleButton.textContent = state.isDebugExpanded ? "Hide" : "Show";
}

function normalizeSelection() {
  const validAgentIds = new Set(currentAgents().map((agent) => agent.id));
  if (!validAgentIds.has(state.selectedAgentId)) {
    state.selectedAgentId = null;
  }
  if (!validAgentIds.has(state.hoveredAgentId)) {
    state.hoveredAgentId = null;
  }
}

function setSessionState(sessionId, captureLevel) {
  state.sessionId = sessionId;
  if (captureLevel) {
    state.captureLevel = captureLevel;
  }

  const hasSession = Boolean(sessionId);
  elements.sessionPill.textContent = hasSession ? `Session ${sessionId}` : "No session";
  elements.startButton.textContent = hasSession ? "Reload Session" : "Load Session";
  elements.viewportStartButton.textContent = hasSession ? "Reload Session" : "Load Session";
  elements.viewportEmpty.classList.toggle("hidden", hasSession);
  syncInteractiveState();
}

function syncLayerButtons() {
  elements.semanticLayerButton.classList.toggle("active", state.showLayers.semantic);
  elements.agentLayerButton.classList.toggle("active", state.showLayers.agents);
  elements.debugLayerButton.classList.toggle("active", state.showLayers.debug);

  const effectiveCapture = elements.captureLevel.value || state.captureLevel;
  elements.debugLayerButton.disabled = effectiveCapture !== "debug" && state.captureLevel !== "debug";
}

function syncInteractiveState() {
  const hasSession = Boolean(state.sessionId);
  const effectiveCapture = elements.captureLevel.value || state.captureLevel;

  elements.validateButton.disabled = state.busy || state.running;
  elements.startButton.disabled = state.busy || state.running;
  elements.viewportStartButton.disabled = state.busy || state.running;
  elements.runButton.disabled = state.busy || state.running || !hasSession;
  elements.pauseButton.disabled = state.busy || !hasSession || (!state.running && state.paused);
  elements.stepButton.disabled = state.busy || state.running || !hasSession;
  elements.resetSessionButton.disabled = state.busy || state.running || !hasSession;
  elements.debugButton.disabled = state.busy || state.running || !hasSession;
  elements.debugLayerButton.disabled = effectiveCapture !== "debug" && state.captureLevel !== "debug";

  syncLayerButtons();

  if (!hasSession) {
    elements.runControlHelp.textContent =
      "Load a session first to render the map and enable the controls.";
    return;
  }

  if (state.running) {
    elements.runControlHelp.textContent =
      "Continuous run is active. Pause to inspect the map or reset back to setup.";
    return;
  }

  if (state.paused) {
    elements.runControlHelp.textContent =
      "Session loaded and paused. Use Single step for CLI-style stepping or Run for playback.";
    return;
  }

  elements.runControlHelp.textContent =
    "Session is live. Press Pause to freeze it or Reset to setup to discard it.";
}

function setBusy(isBusy) {
  state.busy = isBusy;
  syncInteractiveState();
}

function renderTopBar() {
  const agents = currentAgents();
  const frameId = state.frame?.frameId ?? "-";
  const step = state.frame?.hud?.step ?? "-";
  const waiting = waitingAgentCount();
  const stuck = stuckAgentCount();

  elements.mapPill.textContent = mapPillText(
    (state.frame?.hud?.mapId ?? elements.mapId.value) || "-",
  );
  elements.framePill.textContent = `Frame ${frameId}`;
  elements.stepPill.textContent = `Step ${step}`;

  elements.liveIndicator.className = "live-indicator";
  if (!state.sessionId) {
    elements.liveIndicator.classList.add("offline");
  } else if (state.running) {
    elements.liveIndicator.classList.add("running");
  } else {
    elements.liveIndicator.classList.add("idle");
  }

  elements.topStatGrid.innerHTML = "";
  [
    ["Active", String(activeAgentCount()), "active"],
    ["Waiting", String(waiting), waiting > 0 ? "warning" : "neutral"],
    ["Stuck", String(stuck), stuck > 0 ? "danger" : "neutral"],
    ["Oscillating", String(oscillatingAgentCount()), oscillatingAgentCount() > 0 ? "warning" : "neutral"],
  ].forEach(([label, value, tone]) => {
    elements.topStatGrid.append(createMiniMetric(label, value, tone));
  });

  elements.fleetSummary.textContent = `${agents.length} AGV · ${activeAgentCount()} active`;
}

function renderPlannerSummary() {
  const overlay = state.frame?.plannerOverlay;
  const cards = [
    ["Avg plan ms", formatNumber(state.metrics?.avgPlanningTimeMs, 2), "primary"],
    ["Avg CPU ms", formatNumber(state.metrics?.avgCpuTimeMs, 2), "neutral"],
    ["Throughput", formatNumber(state.metrics?.throughput, 2), "primary"],
    ["Deadlocks", String(state.metrics?.deadlockCount ?? "-"), Number(state.metrics?.deadlockCount || 0) > 0 ? "danger" : "neutral"],
    [
      "Wait edges",
      overlay?.available ? String(overlay.waitEdgeCount) : formatNumber(state.metrics?.plannerWaitEdgesPerStep, 2),
      Number(overlay?.waitEdgeCount || 0) > 0 ? "warning" : "neutral",
    ],
    [
      "CBS success",
      `${state.metrics?.plannerCbsSuccessCount ?? 0}/${state.metrics?.plannerCbsAttemptCount ?? 0}`,
      "warning",
    ],
  ];

  elements.plannerSummaryGrid.innerHTML = "";
  cards.forEach(([label, value, tone]) => {
    elements.plannerSummaryGrid.append(createMetricTile(label, value, tone));
  });
}

function renderModalSummary() {
  elements.modalSummaryGrid.innerHTML = "";
  hudKeys.forEach(([label, getter], index) => {
    const tone = index === 3 ? "primary" : index === 7 ? "warning" : "neutral";
    elements.modalSummaryGrid.append(createMetricTile(label, String(getter(state.frame)), tone));
  });
}

function renderMetrics() {
  elements.metricsGrid.innerHTML = "";
  metricKeys.forEach(([label, getter, tone]) => {
    elements.metricsGrid.append(createMetricTile(label, String(getter(state.metrics)), tone));
  });
}

function renderRoster() {
  const agents = currentAgents();
  elements.agentRosterList.innerHTML = "";

  if (agents.length === 0) {
    const empty = document.createElement("div");
    empty.className = "telemetry-empty";
    empty.innerHTML = "<h3>No agents yet</h3><p>Load a session to inspect each AGV in the roster.</p>";
    elements.agentRosterList.append(empty);
    return;
  }

  agents.forEach((agent) => {
    const button = document.createElement("button");
    button.type = "button";
    button.className = "roster-row";
    if (agent.id === state.selectedAgentId) {
      button.classList.add("selected");
    }
    if (agent.waitReason && agent.waitReason !== "none") {
      button.classList.add("waiting");
    }

    const color = deterministicAgentColor(agent);
    button.addEventListener("click", () => {
      state.selectedAgentId = agent.id;
      renderAll();
    });
    button.addEventListener("mouseenter", () => {
      state.hoveredAgentId = agent.id;
      renderRouteHud();
      renderScene();
    });
    button.addEventListener("mouseleave", () => {
      if (state.hoveredAgentId === agent.id) {
        state.hoveredAgentId = null;
        renderRouteHud();
        renderScene();
      }
    });

    const head = document.createElement("div");
    head.className = "roster-head";

    const primary = document.createElement("div");
    primary.className = "roster-primary";

    const swatch = document.createElement("div");
    swatch.className = "agent-swatch";
    swatch.style.background = `linear-gradient(135deg, ${color}, ${color}cc)`;
    swatch.textContent = agent.symbol || "?";

    const textGroup = document.createElement("div");
    const title = document.createElement("div");
    title.className = "roster-title";
    title.textContent = `Agent ${agent.symbol || "?"}`;

    const subtitle = document.createElement("div");
    subtitle.className = "roster-subtitle";
    subtitle.textContent = `${coordText(agent.position)} → ${coordText(agent.goal)}`;

    textGroup.append(title, subtitle);
    primary.append(swatch, textGroup);

    const wait = document.createElement("span");
    wait.className = `wait-badge ${waitReasonClass(agent.waitReason)}`;
    wait.textContent = waitReasonShortLabel(agent.waitReason);

    head.append(primary, wait);

    const body = document.createElement("div");
    body.className = "roster-body";

    const statusRow = document.createElement("div");
    statusRow.className = "roster-status-row";

    const stateText = document.createElement("div");
    stateText.className = "roster-meta";
    stateText.textContent = titleCase(agent.state || "idle");

    const batteryChip = document.createElement("span");
    batteryChip.className = `battery-chip ${chargeReserveTone(agent)}`;
    batteryChip.textContent = `${
      (agent.chargeTimer || 0) > 0 || agent.state === "charging" ? "CHG" : "BAT"
    } ${chargeReservePercent(agent)}%`;

    statusRow.append(stateText, batteryChip);

    const battery = document.createElement("div");
    battery.className = "roster-battery";

    const batteryMeta = document.createElement("div");
    batteryMeta.className = "roster-battery-meta";

    const batteryCopy = document.createElement("span");
    batteryCopy.textContent = chargeReserveMeta(agent);

    const distText = document.createElement("span");
    distText.textContent = `${Number(agent.totalDistanceTraveled || 0).toFixed(1)} cells traveled`;

    batteryMeta.append(batteryCopy, distText);

    const meter = document.createElement("div");
    meter.className = "battery-meter";
    const fill = document.createElement("div");
    fill.className = `battery-fill ${chargeReserveTone(agent)}`;
    fill.style.width = `${Math.max(8, chargeReservePercent(agent))}%`;
    meter.append(fill);

    battery.append(batteryMeta, meter);
    body.append(statusRow, battery);
    button.append(head, body);
    elements.agentRosterList.append(button);
  });
}

function createTelemetryField(label, value, wide = false) {
  const field = document.createElement("div");
  field.className = "telemetry-field";
  if (wide) {
    field.classList.add("wide");
  }

  const fieldLabel = document.createElement("div");
  fieldLabel.className = "telemetry-label";
  fieldLabel.textContent = label;

  const fieldValue = document.createElement("div");
  fieldValue.className = "telemetry-value";
  fieldValue.textContent = value;

  field.append(fieldLabel, fieldValue);
  return field;
}

function renderTelemetry() {
  const agent = selectedAgent();
  elements.telemetryContent.innerHTML = "";

  if (!agent) {
    const empty = document.createElement("div");
    empty.className = "telemetry-empty";
    empty.innerHTML =
      "<h3>No AGV selected</h3><p>Click an AGV on the map or in the roster to inspect its live telemetry.</p>";
    elements.telemetryContent.append(empty);
    return;
  }

  const color = deterministicAgentColor(agent);
  const shell = document.createElement("div");
  shell.className = "telemetry-shell";

  const head = document.createElement("div");
  head.className = "telemetry-head";

  const badge = document.createElement("div");
  badge.className = "telemetry-badge";
  badge.style.background = `linear-gradient(135deg, ${color}, ${color}cc)`;
  badge.textContent = agent.symbol || "?";

  const copy = document.createElement("div");
  copy.className = "telemetry-copy";

  const title = document.createElement("h3");
  title.textContent = `Agent ${agent.symbol || "?"}`;

  const subtitle = document.createElement("div");
  subtitle.className = "telemetry-subtitle";
  subtitle.textContent = `id ${agent.id} · ${agent.isActive ? "active" : "inactive"} · ${
    agent.movedLastStep ? "moved last step" : "held last step"
  }`;

  copy.append(title, subtitle);
  head.append(badge, copy);

  const grid = document.createElement("div");
  grid.className = "telemetry-grid";
  grid.append(
    createTelemetryField("State", agentStateShortLabel(agent.state)),
    createTelemetryField("Wait", waitReasonShortLabel(agent.waitReason)),
    createTelemetryField("Position", coordText(agent.position)),
    createTelemetryField("Goal", coordText(agent.goal)),
    createTelemetryField("Task age / d / t", taskSummary(agent), true),
    createTelemetryField("Timers", timerSummary(agent)),
    createTelemetryField("Stk / Osc", `${agent.stuckSteps || 0}/${agent.oscillationSteps || 0}`),
    createTelemetryField("Distance", Number(agent.totalDistanceTraveled || 0).toFixed(1)),
    createTelemetryField("Home", coordText(agent.home)),
  );

  shell.append(head, grid);
  elements.telemetryContent.append(shell);
}

function renderStatusFeed() {
  elements.statusList.innerHTML = "";

  if (state.statusLines.length === 0) {
    const empty = document.createElement("div");
    empty.className = "status-empty";
    empty.textContent = "Status messages from the shell and backend will appear here.";
    elements.statusList.append(empty);
    return;
  }

  state.statusLines.forEach((entry) => {
    const line = document.createElement("div");
    line.className = "status-line";

    const time = document.createElement("span");
    time.className = "status-time";
    time.textContent = `[${entry.time}]`;

    const text = document.createElement("span");
    text.textContent = entry.text;

    line.append(time, text);
    elements.statusList.append(line);
  });
}

function mergeLogs(entries, options = {}) {
  const incoming = Array.isArray(entries) ? entries : [];
  if (options.reset) {
    state.logs = [];
    state.lastLogSeq = 0;
  }

  incoming.forEach((entry) => {
    const seq = Number(entry?.seq || 0);
    if (seq <= state.lastLogSeq) {
      return;
    }
    state.lastLogSeq = seq;
    if (shouldIncludeConsoleTail(entry)) {
      state.logs.push(entry);
    }
  });

  if (state.logs.length > kLogRingSize) {
    state.logs = state.logs.slice(-kLogRingSize);
  }
}

function renderLogs() {
  elements.logsList.innerHTML = "";

  if (state.logs.length === 0) {
    const empty = document.createElement("div");
    empty.className = "tail-empty";
    empty.textContent =
      "Showing warnings, errors, and high-signal control/scenario/charge/deadlock events only.";
    elements.logsList.append(empty);
    return;
  }

  state.logs.slice(-kLogRingSize).forEach((entry) => {
    const line = document.createElement("div");
    line.className = `tail-line ${logLevelClass(entry.level)}`;

    const step = document.createElement("span");
    step.className = "tail-step";
    step.textContent = `[${formatStepTag(entry?.step)}]`;

    const level = document.createElement("span");
    level.className = "tail-level";
    level.textContent = `[${String(entry?.level || "info").toUpperCase()}]`;

    const scope = document.createElement("span");
    scope.className = "tail-scope";
    scope.textContent = formatStructuredLogScope(entry);

    const message = document.createElement("span");
    message.className = "tail-message";
    message.textContent = entry?.text || formatStructuredLogLine(entry);

    line.append(step, level, scope, message);
    elements.logsList.append(line);
  });
}

function renderRouteHud() {
  const agent = displayedAgent();

  if (!agent || !state.sessionId || !state.uiVisible) {
    elements.routeHud.classList.add("hidden");
    return;
  }

  elements.routeHud.classList.remove("hidden");
  elements.routeAgentTitle.textContent = `Agent ${agent.symbol || "?"}`;
  elements.routeGoal.textContent = `Goal ${coordText(agent.goal)} · Position ${coordText(agent.position)}`;
  elements.routeMeta.textContent = `${agentStateShortLabel(agent.state)} · task ${taskSummary(agent)} · wait ${waitReasonShortLabel(agent.waitReason)}`;

  const wait = waitReasonShortLabel(agent.waitReason);
  if (wait !== "-" && wait !== "IDLE" && wait !== "ACT") {
    elements.routeWarning.classList.remove("hidden");
    elements.routeWarning.textContent =
      wait === "OSC"
        ? "Oscillation detected. Switch to debug capture to inspect planner overlay."
        : "The selected AGV is currently waiting on a planner or movement constraint.";
  } else {
    elements.routeWarning.classList.add("hidden");
    elements.routeWarning.textContent = "";
  }
}

function renderDebugPanel() {
  elements.debugCard.classList.toggle("is-empty", !state.sessionId);
}

function effectiveDebugCaptureEnabled() {
  return (elements.captureLevel.value || state.captureLevel) === "debug" || state.captureLevel === "debug";
}

function baseTileAt(scene, x, y) {
  const tiles = String(scene?.baseTiles || "");
  const width = Number(scene?.width || 0);
  const index = y * width + x;
  return tiles[index] || ".";
}

function tileColor(tile) {
  if (tile === "+" || tile === "#" || tile === "X") return "#243656";
  if (tile === "~") return "#d5eef8";
  return "#f8fbff";
}

function goalStateMap() {
  const map = new Map();
  (state.frame?.goalStates || []).forEach((goal) => {
    map.set(`${goal.position.x},${goal.position.y}`, goal);
  });
  return map;
}

function viewportSize() {
  const rect = elements.canvas.getBoundingClientRect();
  return {
    width: Math.max(320, Math.floor(rect.width || 1280)),
    height: Math.max(320, Math.floor(rect.height || 760)),
  };
}

function computeViewportSafeArea(viewportWidth, viewportHeight) {
  const left = Math.min(430, Math.max(220, Math.floor(viewportWidth * 0.24)));
  const right = Math.min(430, Math.max(220, Math.floor(viewportWidth * 0.24)));
  const top = Math.min(190, Math.max(124, Math.floor(viewportHeight * 0.16)));
  const bottom = Math.min(270, Math.max(172, Math.floor(viewportHeight * 0.23)));
  return {
    left,
    right,
    top,
    bottom,
    availableWidth: Math.max(240, viewportWidth - left - right),
    availableHeight: Math.max(220, viewportHeight - top - bottom),
  };
}

function layoutViewportEmpty() {
  const { width, height } = viewportSize();
  const safe = computeViewportSafeArea(width, height);
  elements.viewportEmpty.style.left = `${safe.left + safe.availableWidth / 2}px`;
  elements.viewportEmpty.style.top = `${safe.top + safe.availableHeight / 2}px`;
  elements.viewportEmpty.style.width = `${Math.min(520, Math.max(340, safe.availableWidth * 0.58))}px`;
}

function computeCanvasView() {
  if (!state.scene) {
    return null;
  }

  const { width: viewportWidth, height: viewportHeight } = viewportSize();
  const width = Math.max(1, Number(state.scene.width || 1));
  const height = Math.max(1, Number(state.scene.height || 1));
  const safe = computeViewportSafeArea(viewportWidth, viewportHeight);
  const cellSize = Math.max(
    8,
    Math.min(safe.availableWidth / width, safe.availableHeight / height) * 0.84,
  );
  const worldWidth = width * cellSize;
  const worldHeight = height * cellSize;

  return {
    viewportWidth,
    viewportHeight,
    cellSize,
    worldWidth,
    worldHeight,
    originX: safe.left + (safe.availableWidth - worldWidth) / 2,
    originY: safe.top + (safe.availableHeight - worldHeight) / 2,
  };
}

function canvasPointFromEvent(event) {
  const rect = elements.canvas.getBoundingClientRect();
  return {
    x: event.clientX - rect.left,
    y: event.clientY - rect.top,
  };
}

function worldPointFromCanvasPoint(point, view = state.canvasView) {
  if (!view) {
    return null;
  }
  return {
    x: (point.x - view.originX) / view.cellSize,
    y: (point.y - view.originY) / view.cellSize,
  };
}

function agentAtCanvasPoint(point) {
  const world = worldPointFromCanvasPoint(point);
  if (!world) {
    return null;
  }
  const cellX = Math.floor(world.x);
  const cellY = Math.floor(world.y);
  return currentAgents().find(
    (agent) => agent.position?.x === cellX && agent.position?.y === cellY,
  ) || null;
}

function drawEmptyViewport(ctx, width, height) {
  const gradient = ctx.createLinearGradient(0, 0, width, height);
  gradient.addColorStop(0, "rgba(247, 250, 255, 1)");
  gradient.addColorStop(1, "rgba(231, 238, 251, 1)");
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, width, height);

  ctx.strokeStyle = "rgba(91, 111, 151, 0.08)";
  ctx.lineWidth = 1;
  for (let x = 0; x < width; x += 36) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, height);
    ctx.stroke();
  }
  for (let y = 0; y < height; y += 36) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }

  ctx.fillStyle = "rgba(37, 99, 235, 0.06)";
  [
    [120, 92, 180, 100],
    [420, 180, 140, 190],
    [680, 124, 240, 120],
    [1020, 220, 150, 180],
  ].forEach(([x, y, boxWidth, boxHeight]) => {
    ctx.fillRect(x, y, boxWidth, boxHeight);
  });

  ctx.fillStyle = "rgba(220, 63, 106, 0.08)";
  [
    [222, 490],
    [510, 520],
    [760, 468],
    [1040, 506],
  ].forEach(([x, y]) => {
    ctx.fillRect(x, y, 120, 56);
  });
}

function drawPlannerOverlay(ctx, view) {
  const overlay = state.frame?.plannerOverlay;
  if (!overlay?.available || !state.showLayers.debug || !effectiveDebugCaptureEnabled()) {
    return;
  }

  const focusAgentId = displayedAgent()?.id ?? null;
  const cellSize = view.cellSize;
  ctx.save();
  ctx.lineCap = "round";
  ctx.lineJoin = "round";

  (overlay.plannedPaths || []).forEach((path) => {
    if (!Array.isArray(path.cells) || path.cells.length < 2) {
      return;
    }
    const focused = focusAgentId !== null && path.agentId === focusAgentId;
    ctx.strokeStyle = focused ? "rgba(37, 99, 235, 0.92)" : "rgba(37, 99, 235, 0.28)";
    ctx.lineWidth = focused ? Math.max(3, cellSize * 0.22) : Math.max(1.5, cellSize * 0.1);
    ctx.beginPath();
    path.cells.forEach((cell, index) => {
      const centerX = cell.x * cellSize + cellSize / 2;
      const centerY = cell.y * cellSize + cellSize / 2;
      if (index === 0) {
        ctx.moveTo(centerX, centerY);
      } else {
        ctx.lineTo(centerX, centerY);
      }
    });
    ctx.stroke();
  });

  (overlay.waitEdges || []).forEach((edge) => {
    const fromX = edge.from.x * cellSize + cellSize / 2;
    const fromY = edge.from.y * cellSize + cellSize / 2;
    const toX = edge.to.x * cellSize + cellSize / 2;
    const toY = edge.to.y * cellSize + cellSize / 2;
    const focused =
      focusAgentId !== null &&
      (edge.fromAgentId === focusAgentId || edge.toAgentId === focusAgentId);

    ctx.strokeStyle = focused ? "rgba(220, 63, 106, 0.9)" : "rgba(217, 119, 6, 0.3)";
    ctx.lineWidth = focused ? Math.max(2, cellSize * 0.16) : Math.max(1, cellSize * 0.08);
    ctx.setLineDash(focused ? [] : [cellSize * 0.18, cellSize * 0.16]);
    ctx.beginPath();
    ctx.moveTo(fromX, fromY);
    ctx.lineTo(toX, toY);
    ctx.stroke();
  });

  ctx.restore();
}

function renderScene() {
  const canvas = elements.canvas;
  const ctx = canvas.getContext("2d");
  const { width: viewportWidth, height: viewportHeight } = viewportSize();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.floor(viewportWidth * dpr);
  canvas.height = Math.floor(viewportHeight * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, viewportWidth, viewportHeight);

  if (!state.scene) {
    state.canvasView = null;
    drawEmptyViewport(ctx, viewportWidth, viewportHeight);
    return;
  }

  const view = computeCanvasView();
  state.canvasView = view;

  const stageGradient = ctx.createLinearGradient(0, 0, viewportWidth, viewportHeight);
  stageGradient.addColorStop(0, "rgba(248, 251, 255, 1)");
  stageGradient.addColorStop(1, "rgba(228, 236, 250, 1)");
  ctx.fillStyle = stageGradient;
  ctx.fillRect(0, 0, viewportWidth, viewportHeight);

  ctx.save();
  ctx.translate(view.originX, view.originY);

  ctx.save();
  ctx.shadowColor = "rgba(33, 56, 105, 0.12)";
  ctx.shadowBlur = 28;
  ctx.shadowOffsetY = 14;
  ctx.fillStyle = "rgba(255, 255, 255, 0.92)";
  ctx.beginPath();
  ctx.roundRect(-20, -20, view.worldWidth + 40, view.worldHeight + 40, 26);
  ctx.fill();
  ctx.restore();

  ctx.strokeStyle = "rgba(201, 213, 236, 0.78)";
  ctx.lineWidth = 1.2;
  ctx.beginPath();
  ctx.roundRect(-20, -20, view.worldWidth + 40, view.worldHeight + 40, 26);
  ctx.stroke();

  const goals = goalStateMap();
  const homes = new Set(
    (state.scene.homeCells || []).map((home) => `${home.position.x},${home.position.y}`),
  );
  const chargers = new Set(
    (state.scene.chargerCells || []).map((charger) => `${charger.x},${charger.y}`),
  );

  for (let y = 0; y < state.scene.height; y += 1) {
    for (let x = 0; x < state.scene.width; x += 1) {
      const tile = baseTileAt(state.scene, x, y);
      const key = `${x},${y}`;

      ctx.fillStyle = tileColor(tile);
      ctx.fillRect(x * view.cellSize, y * view.cellSize, view.cellSize, view.cellSize);

      if (state.showLayers.semantic) {
        if (goals.has(key)) {
          const goal = goals.get(key);
          ctx.fillStyle = goal.isParked
            ? "rgba(15, 155, 113, 0.72)"
            : "rgba(240, 170, 58, 0.82)";
          ctx.beginPath();
          ctx.roundRect(
            x * view.cellSize + view.cellSize * 0.18,
            y * view.cellSize + view.cellSize * 0.18,
            view.cellSize * 0.64,
            view.cellSize * 0.64,
            view.cellSize * 0.14,
          );
          ctx.fill();
        } else if (homes.has(key)) {
          ctx.strokeStyle = "rgba(234, 100, 112, 0.92)";
          ctx.lineWidth = Math.max(1, view.cellSize * 0.1);
          ctx.beginPath();
          ctx.roundRect(
            x * view.cellSize + view.cellSize * 0.16,
            y * view.cellSize + view.cellSize * 0.16,
            view.cellSize * 0.68,
            view.cellSize * 0.68,
            view.cellSize * 0.14,
          );
          ctx.stroke();
        } else if (chargers.has(key)) {
          ctx.fillStyle = "rgba(32, 164, 201, 0.18)";
          ctx.beginPath();
          ctx.roundRect(
            x * view.cellSize + view.cellSize * 0.16,
            y * view.cellSize + view.cellSize * 0.16,
            view.cellSize * 0.68,
            view.cellSize * 0.68,
            view.cellSize * 0.16,
          );
          ctx.fill();
          ctx.fillStyle = "rgba(32, 164, 201, 0.94)";
          ctx.beginPath();
          ctx.arc(
            x * view.cellSize + view.cellSize / 2,
            y * view.cellSize + view.cellSize / 2,
            view.cellSize * 0.22,
            0,
            Math.PI * 2,
          );
          ctx.fill();
        }
      }

      ctx.strokeStyle = tile === "+" || tile === "#" || tile === "X"
        ? "rgba(255, 255, 255, 0.06)"
        : "rgba(91, 111, 151, 0.1)";
      ctx.lineWidth = 1;
      ctx.strokeRect(x * view.cellSize, y * view.cellSize, view.cellSize, view.cellSize);
    }
  }

  drawPlannerOverlay(ctx, view);

  if (state.showLayers.agents) {
    currentAgents().forEach((agent) => {
      const color = deterministicAgentColor(agent);
      const centerX = agent.position.x * view.cellSize + view.cellSize / 2;
      const centerY = agent.position.y * view.cellSize + view.cellSize / 2;
      const waiting = agent.waitReason && agent.waitReason !== "none";
      const selected = agent.id === state.selectedAgentId;
      const hovered = agent.id === state.hoveredAgentId;
      const chargeTone = chargeReserveTone(agent);

      if (selected || hovered || waiting) {
        ctx.strokeStyle = selected
          ? "rgba(37, 99, 235, 0.95)"
          : hovered
            ? "rgba(99, 102, 241, 0.9)"
            : "rgba(217, 119, 6, 0.82)";
        ctx.lineWidth = selected
          ? Math.max(3, view.cellSize * 0.2)
          : Math.max(2, view.cellSize * 0.14);
        ctx.beginPath();
        ctx.arc(centerX, centerY, view.cellSize * (selected ? 0.44 : 0.4), 0, Math.PI * 2);
        ctx.stroke();
      }

      ctx.save();
      ctx.shadowColor = hexToRgba(color, 0.35);
      ctx.shadowBlur = Math.max(12, view.cellSize * 0.52);
      ctx.shadowOffsetY = Math.max(2, view.cellSize * 0.12);
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.roundRect(
        centerX - view.cellSize * 0.28,
        centerY - view.cellSize * 0.28,
        view.cellSize * 0.56,
        view.cellSize * 0.56,
        view.cellSize * 0.16,
      );
      ctx.fill();
      ctx.restore();

      const gloss = ctx.createLinearGradient(
        centerX,
        centerY - view.cellSize * 0.28,
        centerX,
        centerY + view.cellSize * 0.28,
      );
      gloss.addColorStop(0, "rgba(255,255,255,0.34)");
      gloss.addColorStop(1, "rgba(255,255,255,0.02)");
      ctx.fillStyle = gloss;
      ctx.beginPath();
      ctx.roundRect(
        centerX - view.cellSize * 0.26,
        centerY - view.cellSize * 0.28,
        view.cellSize * 0.52,
        view.cellSize * 0.28,
        view.cellSize * 0.16,
      );
      ctx.fill();

      ctx.fillStyle = "#ffffff";
      ctx.font = `700 ${Math.max(10, Math.floor(view.cellSize * 0.34))}px "Segoe UI", sans-serif`;
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(agent.symbol || "A", centerX, centerY + 1);

      if (chargeTone !== "healthy") {
        ctx.fillStyle =
          chargeTone === "charging"
            ? "rgba(15, 155, 113, 0.96)"
            : chargeTone === "warning"
              ? "rgba(217, 119, 6, 0.96)"
              : "rgba(220, 63, 106, 0.96)";
        ctx.beginPath();
        ctx.arc(
          centerX + view.cellSize * 0.22,
          centerY - view.cellSize * 0.22,
          Math.max(3, view.cellSize * 0.11),
          0,
          Math.PI * 2,
        );
        ctx.fill();
      }
    });
  }

  ctx.restore();
}

function cloneJson(value) {
  return value ? JSON.parse(JSON.stringify(value)) : value;
}

function applyDelta(delta) {
  if (!state.frame || delta.requiresFullResync) {
    return;
  }

  const nextFrame = cloneJson(state.frame);
  nextFrame.frameId = delta.toFrameId;
  nextFrame.lastLogSeq = delta.lastLogSeq;
  if (delta.hudChanged) {
    nextFrame.hud = delta.hud;
  }
  if (delta.overlayChanged) {
    nextFrame.plannerOverlay = delta.plannerOverlay;
  }

  const agentsById = new Map((nextFrame.agents || []).map((agent) => [agent.id, agent]));
  (delta.agentUpdates || []).forEach((agent) => {
    agentsById.set(agent.id, agent);
  });
  nextFrame.agents = Array.from(agentsById.values()).sort((left, right) => left.id - right.id);

  const goalsByKey = new Map(
    (nextFrame.goalStates || []).map((goal) => [`${goal.position.x},${goal.position.y}`, goal]),
  );
  (delta.goalStateChanges || []).forEach((goal) => {
    goalsByKey.set(`${goal.position.x},${goal.position.y}`, goal);
  });
  nextFrame.goalStates = Array.from(goalsByKey.values());
  mergeLogs(delta.newLogs);
  nextFrame.logsTail = state.logs.slice(-kLogRingSize);
  state.frame = nextFrame;
  normalizeSelection();
}

function applyFrame(frame) {
  state.frame = frame;
  mergeLogs(frame.logsTail, { reset: frame.frameId === 0 });
  normalizeSelection();
}

function handleSync(sync) {
  if (!sync) {
    renderAll();
    return;
  }

  if (sync.kind === "frame") {
    applyFrame(sync.frame);
  } else if (sync.kind === "delta") {
    applyDelta(sync.delta);
  }
  renderAll();
}

function clearSessionView() {
  state.scene = null;
  state.frame = null;
  state.metrics = null;
  state.logs = [];
  state.lastLogSeq = 0;
  state.running = false;
  state.paused = true;
  state.selectedAgentId = null;
  state.hoveredAgentId = null;
  setSessionState(null, elements.captureLevel.value || state.captureLevel);
  renderAll();
}

function renderAll() {
  normalizeSelection();
  layoutViewportEmpty();
  elements.viewportEmpty.classList.toggle("hidden", Boolean(state.sessionId));
  renderLaunchSummaries();
  renderTopBar();
  renderPlannerSummary();
  renderRoster();
  renderTelemetry();
  renderModalSummary();
  renderMetrics();
  renderLogs();
  renderRouteHud();
  renderDebugPanel();
  renderScene();
  syncInteractiveState();
}

async function validateConfig() {
  const launchConfig = buildLaunchConfig();
  const response = await window.agvShell.validateConfig(launchConfig);
  const warnings = (response.warnings || []).map((issue) => issue.message);
  const errors = (response.errors || []).map((issue) => issue.message);

  if (errors.length > 0) {
    appendStatus(`Validation failed: ${errors.join(" | ")}`);
  } else if (warnings.length > 0) {
    appendStatus(`Validation warnings: ${warnings.join(" | ")}`);
  } else {
    appendStatus("Validation passed with no issues.");
  }
}

async function ensurePaused(paused) {
  if (!state.sessionId) {
    state.paused = Boolean(paused);
    syncInteractiveState();
    return { paused: state.paused };
  }

  const response = await window.agvShell.setPaused(Boolean(paused));
  state.paused = Boolean(response.paused);
  syncInteractiveState();
  return response;
}

async function startSession() {
  state.running = false;
  if (state.sessionId) {
    await window.agvShell.resetSession();
  }

  const launchConfig = buildLaunchConfig();
  const captureLevel = elements.captureLevel.value || "frame";
  const response = await window.agvShell.startSession({ launchConfig, captureLevel });

  state.scene = response.scene;
  state.metrics = response.metrics;
  state.logs = [];
  state.lastLogSeq = 0;
  state.running = false;
  state.paused = false;
  state.selectedAgentId = null;
  state.hoveredAgentId = null;
  applyFrame(response.frame);
  setSessionState(response.session.sessionId, response.captureLevel);
  await ensurePaused(true);
  renderAll();
  setSetupOpen(false);

  const warnings = (response.warnings || []).map((issue) => issue.message);
  appendStatus(`Session ${response.session.sessionId} loaded on map ${response.session.launchConfig.mapId}.`);
  appendStatus("Session starts paused. Use Run or Single step to advance.");
  if (warnings.length > 0) {
    appendStatus(`Normalized with warnings: ${warnings.join(" | ")}`);
  }
}

async function advance(payload, options = {}) {
  const response = await window.agvShell.advance({
    captureLevel: elements.captureLevel.value || state.captureLevel,
    ...payload,
  });
  state.metrics = response.metrics;
  state.paused = Boolean(response.advance.paused);
  setSessionState(state.sessionId, response.advance.captureLevel);
  handleSync(response.sync);

  if (!options.silent) {
    appendStatus(
      `Advance complete: ${response.advance.executedSteps} steps, frame ${response.advance.frameId}.`,
    );
  }
  return response;
}

function waitForPlaybackFrame(delayMs = kRunFrameDelayMs) {
  return new Promise((resolve) => {
    window.setTimeout(() => {
      window.requestAnimationFrame(() => resolve());
    }, delayMs);
  });
}

async function runContinuously() {
  if (!state.sessionId || state.running) {
    return;
  }

  state.running = true;
  state.paused = false;
  syncInteractiveState();
  appendStatus("Continuous run started.");

  try {
    await ensurePaused(false);
    while (state.running && state.sessionId) {
      const response = await advance({ steps: kRunStepsPerTick }, { silent: true });
      if (response.advance.complete) {
        appendStatus("Simulation reached completion.");
        break;
      }
      if (response.advance.executedSteps <= 0) {
        appendStatus("Run stopped because no additional steps were executed.");
        break;
      }
      await waitForPlaybackFrame();
    }
  } catch (error) {
    appendStatus(`Run failed: ${error.message}`);
  } finally {
    state.running = false;
    if (state.sessionId && !state.paused) {
      try {
        await ensurePaused(true);
      } catch (error) {
        appendStatus(`Pause sync failed: ${error.message}`);
      }
    }
    syncInteractiveState();
  }
}

async function pauseRun() {
  if (!state.sessionId || (!state.running && state.paused)) {
    return;
  }

  state.running = false;
  await ensurePaused(true);
  appendStatus("Run paused.");
}

async function stepOnce() {
  if (!state.sessionId || state.running) {
    return;
  }

  await ensurePaused(false);
  const response = await advance({ steps: 1 }, { silent: true });
  await ensurePaused(true);
  appendStatus(
    `Single step complete: ${response.advance.executedSteps} step(s), frame ${response.advance.frameId}.`,
  );
}

async function resetToSetup() {
  state.running = false;
  if (state.sessionId) {
    await window.agvShell.resetSession();
  }
  clearSessionView();
  elements.debugOutput.textContent = "No debug snapshot requested yet.";
  setDebugExpanded(false);
  setSetupOpen(true);
  appendStatus("Returned to setup. Adjust the launch settings and load a new session.");
}

async function loadDebugSnapshot() {
  const snapshot = await window.agvShell.getDebugSnapshot();
  elements.debugOutput.textContent = JSON.stringify(snapshot, null, 2);
  setDebugExpanded(true);
  appendStatus("Debug snapshot captured.");
}

async function runBusyAction(action, failurePrefix) {
  setBusy(true);
  try {
    await action();
  } catch (error) {
    appendStatus(`${failurePrefix} ${error.message}`);
  } finally {
    setBusy(false);
  }
}

function updateHoveredAgentFromEvent(event) {
  if (!state.scene) {
    return;
  }
  const point = canvasPointFromEvent(event);
  const hovered = agentAtCanvasPoint(point);
  const nextId = hovered?.id ?? null;
  if (nextId === state.hoveredAgentId) {
    return;
  }
  state.hoveredAgentId = nextId;
  renderRouteHud();
  renderScene();
}

async function bootstrap() {
  const response = await window.agvShell.bootstrap();
  state.capabilities = response.capabilities;
  state.backendPath = response.backendPath;
  state.mapOptions = Array.isArray(response.capabilities.mapOptions)
    ? response.capabilities.mapOptions
    : Array.from(
        {
          length:
            response.capabilities.mapIdRange.max - response.capabilities.mapIdRange.min + 1,
        },
        (_value, index) => {
          const mapId = response.capabilities.mapIdRange.min + index;
          return {
            id: mapId,
            label: `Map ${mapId}`,
            description: "",
            capacity: 1,
          };
        },
      );

  populateMapSelect(state.mapOptions);
  populateSelect(elements.algorithm, response.capabilities.algorithms || []);
  populateSelect(elements.mode, response.capabilities.modes || []);
  populateSelect(elements.captureLevel, response.capabilities.captureLevels || []);

  elements.mapId.value = String(state.mapOptions[0]?.id || 1);
  elements.algorithm.value = "default";
  elements.mode.value = "custom";
  elements.captureLevel.value = "frame";
  elements.seed.value = "7";

  state.customPhases = createDefaultCustomPhases(currentMapCapacity());
  syncModeVisibility();
  updateMapCapacityUi();
  renderPhaseEditor();
  renderLaunchSummaries();
  setUiVisible(true);
  setSetupOpen(true);
  setMetricsOpen(false);
  setDebugExpanded(false);
  renderStatusFeed();
  renderAll();

  appendStatus(`Backend discovered at ${state.backendPath}.`);
  appendStatus("Choose a scenario, then press Load Session to render the map.");
}

[
  elements.algorithm,
  elements.captureLevel,
  elements.speedMultiplier,
  elements.seed,
  elements.realtimePark,
  elements.realtimeExit,
].forEach((element) => {
  element.addEventListener("input", () => {
    renderLaunchSummaries();
    syncInteractiveState();
    renderScene();
  });
  element.addEventListener("change", () => {
    renderLaunchSummaries();
    syncInteractiveState();
    renderScene();
  });
});

elements.mapId.addEventListener("change", () => {
  syncCustomPhasesForMap({ announce: true });
});

elements.mode.addEventListener("change", () => {
  syncModeVisibility();
  renderLaunchSummaries();
});

elements.addParkPhaseButton.addEventListener("click", () => addPhase("park"));
elements.addExitPhaseButton.addEventListener("click", () => addPhase("exit"));
elements.resetPhasesButton.addEventListener("click", () => {
  state.customPhases = createDefaultCustomPhases(currentMapCapacity());
  renderPhaseEditor();
  renderLaunchSummaries();
  appendStatus("Scenario plan reset to the default custom template.");
});

elements.validateButton.addEventListener("click", async () => {
  await runBusyAction(validateConfig, "Validation error:");
});

async function loadSessionFromUi() {
  await runBusyAction(startSession, "Load failed:");
}

elements.startButton.addEventListener("click", loadSessionFromUi);
elements.viewportStartButton.addEventListener("click", loadSessionFromUi);

elements.viewportSetupButton.addEventListener("click", () => {
  setSetupOpen(true);
});

elements.runButton.addEventListener("click", () => {
  void runContinuously();
});

elements.pauseButton.addEventListener("click", () => {
  void pauseRun();
});

elements.stepButton.addEventListener("click", async () => {
  await runBusyAction(stepOnce, "Step failed:");
});

elements.resetSessionButton.addEventListener("click", async () => {
  await runBusyAction(resetToSetup, "Reset failed:");
});

elements.debugButton.addEventListener("click", async () => {
  await runBusyAction(loadDebugSnapshot, "Debug snapshot failed:");
});

elements.debugToggleButton.addEventListener("click", () => {
  setDebugExpanded(!state.isDebugExpanded);
});

elements.toggleUiButton.addEventListener("click", () => {
  setUiVisible(!state.uiVisible);
  renderRouteHud();
});

elements.openMetricsButton.addEventListener("click", () => {
  setMetricsOpen(true);
});

elements.closeMetricsButton.addEventListener("click", () => {
  setMetricsOpen(false);
});

elements.metricsBackdrop.addEventListener("click", () => {
  setMetricsOpen(false);
});

elements.openSetupButton.addEventListener("click", () => {
  setSetupOpen(true);
});

elements.closeSetupButton.addEventListener("click", () => {
  setSetupOpen(false);
});

elements.setupBackdrop.addEventListener("click", () => {
  setSetupOpen(false);
});

elements.semanticLayerButton.addEventListener("click", () => {
  state.showLayers.semantic = !state.showLayers.semantic;
  syncLayerButtons();
  renderScene();
});

elements.agentLayerButton.addEventListener("click", () => {
  state.showLayers.agents = !state.showLayers.agents;
  syncLayerButtons();
  renderScene();
});

elements.debugLayerButton.addEventListener("click", () => {
  if (elements.debugLayerButton.disabled) {
    appendStatus("Planner overlay is available only when the capture mode is set to debug.");
    return;
  }
  state.showLayers.debug = !state.showLayers.debug;
  syncLayerButtons();
  renderScene();
});
elements.canvas.addEventListener("mousemove", updateHoveredAgentFromEvent);
elements.canvas.addEventListener("click", (event) => {
  if (!state.scene) {
    return;
  }
  const point = canvasPointFromEvent(event);
  const agent = agentAtCanvasPoint(point);
  state.selectedAgentId = agent?.id ?? null;
  state.hoveredAgentId = agent?.id ?? null;
  renderAll();
});
elements.canvas.addEventListener("mouseleave", () => {
  if (state.hoveredAgentId !== null) {
    state.hoveredAgentId = null;
    renderRouteHud();
    renderScene();
  }
});

window.addEventListener("resize", () => {
  renderAll();
});

window.agvShell.onEvent((event) => {
  if (event.type === "backend-ready") {
    appendStatus(`Backend process started at ${event.backendPath}.`);
    return;
  }
  if (event.type === "backend-stderr") {
    appendStatus(`stderr: ${event.line}`);
    return;
  }
  if (event.type === "backend-exit") {
    appendStatus(`Backend exited with code ${event.code}.`);
  }
});

bootstrap().catch((error) => {
  appendStatus(`Bootstrap failed: ${error.message}`);
  elements.debugOutput.textContent = error.stack || error.message;
  setDebugExpanded(true);
});
