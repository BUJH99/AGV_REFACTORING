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
const kMetricHistoryLimit = 72;
const kChargeDistanceThreshold = 300.0;
const kPriorityReturningWithCar = 3;
const kPriorityGoingToCharge = 2;
const kPriorityMovingTask = 1;
const kPriorityDeadlockThreshold = 5;
const kPriorityStuckBoostMult = 10;
const kPriorityStuckBoostHard = 1000;
const kPriorityLayoutSpring = Object.freeze({
  type: "spring",
  stiffness: 520,
  damping: 34,
  mass: 0.78,
});
const kPriorityEntrySpring = Object.freeze({
  type: "spring",
  stiffness: 460,
  damping: 30,
  mass: 0.82,
});
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

const trendDeckSpecs = [
  {
    key: "throughput",
    label: "Throughput",
    tone: "primary",
    formatter: (value) => formatNumber(value, 2),
    deltaFormatter: (value) => formatSigned(value, 2),
    improvesWhen: "up",
    footnote: "completed tasks per recorded step",
  },
  {
    key: "avgPlanningTimeMs",
    label: "Avg plan ms",
    tone: "primary",
    formatter: (value) => `${formatNumber(value, 2)} ms`,
    deltaFormatter: (value) => `${formatSigned(value, 2)} ms`,
    improvesWhen: "down",
    footnote: "planner cost across the run",
  },
  {
    key: "outstandingTaskCount",
    label: "Outstanding",
    tone: "warning",
    formatter: (value) => formatInteger(value),
    deltaFormatter: (value) => formatSigned(value, 0),
    improvesWhen: "down",
    footnote: "queued plus in-flight backlog",
  },
  {
    key: "plannerWaitEdgesPerStep",
    label: "Wait edges",
    tone: "warning",
    formatter: (value) => formatNumber(value, 2),
    deltaFormatter: (value) => formatSigned(value, 2),
    improvesWhen: "down",
    footnote: "planner dependency edges per step",
  },
];

const metricSectionSpecs = [
  {
    title: "Operations",
    copy: "Session-level productivity, movement, and service latency.",
    items: [
      { label: "Tasks completed", key: "tasksCompletedTotal", tone: "primary", format: formatInteger },
      { label: "Tasks / AGV", key: "tasksPerAgent", format: (value) => formatNumber(value, 2) },
      { label: "Requests created", key: "requestsCreatedTotal", format: formatInteger },
      { label: "Avg request wait", key: "avgRequestWaitTicks", format: (value) => formatNumber(value, 1) },
      { label: "Avg movement / task", key: "avgMovementPerTask", format: (value) => formatNumber(value, 2) },
      { label: "Avg task distance", key: "avgTaskDistance", format: (value) => formatNumber(value, 2) },
      { label: "Avg task turns", key: "avgTaskTurns", format: (value) => formatNumber(value, 2) },
      { label: "Avg task steps", key: "avgTaskSteps", format: (value) => formatNumber(value, 2) },
      { label: "Steps / cell", key: "avgTaskStepsPerCell", format: (value) => formatNumber(value, 2) },
      { label: "Turns / 100 cells", key: "avgTaskTurnsPer100Cells", format: (value) => formatNumber(value, 2) },
      { label: "Task movement cover", key: "taskMovementCoverageRatio", format: (value) => formatPercent(value, 1) },
      { label: "Remaining parked", key: "remainingParkedVehicles", format: formatInteger },
    ],
  },
  {
    title: "CPU and Memory",
    copy: "Runtime cost, planning cost, and memory pressure already tracked by the backend.",
    items: [
      { label: "Avg CPU ms", key: "avgCpuTimeMs", tone: "primary", format: (value) => `${formatNumber(value, 2)} ms` },
      { label: "Max CPU ms", key: "maxStepCpuTimeMs", format: (value) => `${formatNumber(value, 2)} ms` },
      { label: "Total CPU ms", key: "totalCpuTimeMs", format: (value) => `${formatNumber(value, 1)} ms` },
      { label: "CPU tasks / sec", key: "tasksPerCpuSecond", format: (value) => formatNumber(value, 2) },
      { label: "Avg planning ms", key: "avgPlanningTimeMs", tone: "primary", format: (value) => `${formatNumber(value, 2)} ms` },
      { label: "Max planning ms", key: "maxPlanningTimeMs", format: (value) => `${formatNumber(value, 2)} ms` },
      { label: "Total planning ms", key: "totalPlanningTimeMs", format: (value) => `${formatNumber(value, 1)} ms` },
      { label: "Planning CPU share", key: "planningCpuShare", format: (value) => formatPercent(value, 1) },
      { label: "Planning ms / task", key: "avgPlanningTimePerTaskMs", format: (value) => `${formatNumber(value, 2)} ms` },
      { label: "Planning tasks / sec", key: "tasksPerPlanningSecond", format: (value) => formatNumber(value, 2) },
      { label: "Avg memory", key: "avgMemoryUsageKb", format: formatMemory },
      { label: "Peak memory", key: "memoryUsagePeakKb", format: formatMemory },
      { label: "Memory / AGV", key: "avgMemoryUsagePerAgentKb", format: formatMemory },
    ],
  },
  {
    title: "Planner Search",
    copy: "Search-space expansion, heap churn, and validity ratios from the planner backend.",
    items: [
      { label: "Nodes expanded", key: "algoNodesExpandedTotal", format: formatInteger },
      { label: "Generated nodes", key: "algoGeneratedNodesTotal", format: formatInteger },
      { label: "Valid expansions", key: "algoValidExpansionsTotal", format: formatInteger },
      { label: "Valid ratio", key: "validExpansionRatio", format: (value) => formatPercent(value, 1) },
      { label: "Avg nodes / step", key: "avgNodesExpandedPerStep", format: (value) => formatNumber(value, 2) },
      { label: "Avg nodes / task", key: "avgNodesExpandedPerTask", format: (value) => formatNumber(value, 2) },
      { label: "Nodes / planning ms", key: "nodesExpandedPerPlanningMs", format: (value) => formatNumber(value, 2) },
      { label: "Heap moves", key: "algoHeapMovesTotal", format: formatInteger },
      { label: "Heap / node", key: "heapMovesPerNodeExpanded", format: (value) => formatNumber(value, 2) },
      { label: "Movement cost", key: "totalMovementCost", format: (value) => formatNumber(value, 2) },
      { label: "Non-task movement", key: "nonTaskMovementCost", format: (value) => formatNumber(value, 2) },
      { label: "Non-task ratio", key: "nonTaskMovementRatio", format: (value) => formatPercent(value, 1) },
    ],
  },
  {
    title: "Queue and Backlog",
    copy: "Outstanding work, queue age, and how long the system has gone without a completion.",
    items: [
      { label: "Recorded steps", key: "recordedSteps", format: formatInteger },
      { label: "Queued tasks", key: "queuedTaskCount", tone: "warning", format: formatInteger },
      { label: "In flight tasks", key: "inFlightTaskCount", format: formatInteger },
      { label: "Outstanding tasks", key: "outstandingTaskCount", tone: "warning", format: formatInteger },
      { label: "Avg outstanding", key: "avgOutstandingTaskCount", format: (value) => formatNumber(value, 2) },
      { label: "Peak outstanding", key: "peakOutstandingTaskCount", format: formatInteger },
      { label: "Avg outstanding / AGV", key: "avgOutstandingTasksPerAgent", format: (value) => formatNumber(value, 2) },
      { label: "Peak outstanding / AGV", key: "peakOutstandingTasksPerAgent", format: (value) => formatNumber(value, 2) },
      { label: "Oldest queued age", key: "oldestQueuedRequestAge", format: formatInteger },
      { label: "Avg oldest age", key: "avgOldestQueuedRequestAge", format: (value) => formatNumber(value, 1) },
      { label: "Peak oldest age", key: "peakOldestQueuedRequestAge", format: formatInteger },
      { label: "Last completion step", key: "lastTaskCompletionStep", format: formatInteger },
      { label: "Steps since completion", key: "stepsSinceLastTaskCompletion", tone: "warning", format: formatInteger },
    ],
  },
  {
    title: "Stall and Conflict",
    copy: "Deadlocks, movement stalls, wait edges, cycles, and CBS outcomes.",
    items: [
      { label: "Deadlocks", key: "deadlockCount", tone: "danger", format: formatInteger },
      { label: "Steps with movement", key: "stepsWithMovement", format: formatInteger },
      { label: "Movement step ratio", key: "movementStepRatio", format: (value) => formatPercent(value, 1) },
      { label: "Stall steps", key: "stallStepCount", tone: "warning", format: formatInteger },
      { label: "Stall step ratio", key: "stallStepRatio", format: (value) => formatPercent(value, 1) },
      { label: "Max no-move streak", key: "maxNoMovementStreak", format: formatInteger },
      { label: "Wait edges sum", key: "plannerWaitEdgesSum", format: formatInteger },
      { label: "Wait edges / step", key: "plannerWaitEdgesPerStep", tone: "warning", format: (value) => formatNumber(value, 2) },
      { label: "Wait edges / conflict", key: "plannerWaitEdgesPerConflictStep", format: (value) => formatNumber(value, 2) },
      { label: "Wait-edge steps", key: "plannerWaitEdgeStepCount", format: formatInteger },
      { label: "Conflict cycles", key: "plannerConflictCycleTotal", tone: "warning", format: formatInteger },
      { label: "Cycle steps", key: "plannerCycleStepCount", format: formatInteger },
      { label: "Cycle step ratio", key: "plannerCycleStepRatio", format: (value) => formatPercent(value, 1) },
      { label: "CBS attempts", key: "plannerCbsAttemptCount", format: formatInteger },
      { label: "CBS success", key: "plannerCbsSuccessCount", format: formatInteger },
      { label: "CBS failures", key: "plannerCbsFailureCount", tone: "danger", format: formatInteger },
      { label: "CBS attempt rate", key: "plannerCbsAttemptRate", format: (value) => formatPercent(value, 1) },
      { label: "CBS success rate", key: "plannerCbsSuccessRate", format: (value) => formatPercent(value, 1) },
      { label: "CBS failure rate", key: "plannerCbsFailureRate", format: (value) => formatPercent(value, 1) },
    ],
  },
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
  metricHistory: [],
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
    map: true,
    semantic: true,
    agents: true,
    routeHud: true,
    plannedPaths: true,
    cbsPaths: true,
    conflictCells: true,
    conflictCounts: true,
    swapWaits: true,
  },
  uiVisible: true,
  uiSections: {
    topBar: true,
    leftStack: true,
    rightStack: true,
    mapTools: true,
    routeHud: true,
    bottomStrip: true,
    sceneLegend: true,
  },
  bottomPanels: {
    tail: true,
    diagnostics: true,
  },
  fleetView: "roster",
  rightStackView: "telemetry",
  priorityBoardMode: "detailed",
  isUiVisibilityPanelOpen: false,
  isSetupOpen: true,
  isMetricsOpen: false,
  isDebugExpanded: false,
  canvasView: null,
};

const elements = {
  appShell: document.getElementById("app-shell"),
  topBar: document.getElementById("top-bar"),
  mapTools: document.getElementById("map-tools"),
  leftStack: document.getElementById("left-stack"),
  rightStack: document.getElementById("right-stack"),
  sceneLegend: document.getElementById("scene-legend"),
  floatingDock: document.getElementById("floating-dock"),
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
  plannerTrendStrip: document.getElementById("planner-trend-strip"),
  fleetSummary: document.getElementById("fleet-summary"),
  fleetRosterTab: document.getElementById("fleet-roster-tab"),
  fleetPriorityTab: document.getElementById("fleet-priority-tab"),
  fleetRosterPane: document.getElementById("fleet-roster-pane"),
  fleetPriorityPane: document.getElementById("fleet-priority-pane"),
  priorityDetailedTab: document.getElementById("priority-detailed-tab"),
  priorityMinimalTab: document.getElementById("priority-minimal-tab"),
  agentRosterList: document.getElementById("agent-roster-list"),
  prioritySummaryStrip: document.getElementById("priority-summary-strip"),
  rightTelemetryTab: document.getElementById("right-telemetry-tab"),
  rightPlannerTab: document.getElementById("right-planner-tab"),
  rightDebugTab: document.getElementById("right-debug-tab"),
  rightTelemetryPane: document.getElementById("right-telemetry-pane"),
  rightPlannerPane: document.getElementById("right-planner-pane"),
  rightDebugPane: document.getElementById("right-debug-pane"),
  telemetryContent: document.getElementById("telemetry-content"),
  modalSummaryGrid: document.getElementById("modal-summary-grid"),
  metricsHeroGrid: document.getElementById("metrics-hero-grid"),
  metricsInfographicsGrid: document.getElementById("metrics-infographics-grid"),
  metricsDistributionGrid: document.getElementById("metrics-distribution-grid"),
  metricsFairnessList: document.getElementById("metrics-fairness-list"),
  metricsSections: document.getElementById("metrics-sections"),
  priorityBoard: document.getElementById("priority-board"),
  priorityBoardSummary: document.getElementById("priority-board-summary"),
  bottomStrip: document.getElementById("bottom-strip"),
  tailCard: document.getElementById("tail-card"),
  diagnosticsCard: document.getElementById("diagnostics-card"),
  logsList: document.getElementById("logs-list"),
  statusList: document.getElementById("status-list"),
  debugCard: document.getElementById("right-debug-pane"),
  debugOutput: document.getElementById("debug-output"),
  canvas: document.getElementById("scene-canvas"),
  routeHud: document.getElementById("route-hud"),
  routeAgentTitle: document.getElementById("route-agent-title"),
  routeGoal: document.getElementById("route-goal"),
  routeMeta: document.getElementById("route-meta"),
  routeWarning: document.getElementById("route-warning"),
  routeDebugPanel: document.getElementById("route-debug-panel"),
  routeDebugMeta: document.getElementById("route-debug-meta"),
  routeDebugBadges: document.getElementById("route-debug-badges"),
  routeDebugLegend: document.getElementById("route-debug-legend"),
  validateButton: document.getElementById("validate-button"),
  startButton: document.getElementById("start-button"),
  runButton: document.getElementById("run-button"),
  pauseButton: document.getElementById("pause-button"),
  stepButton: document.getElementById("step-button"),
  resetSessionButton: document.getElementById("reset-session-button"),
  debugButton: document.getElementById("debug-button"),
  debugToggleButton: document.getElementById("debug-toggle-button"),
  tailVisibilityButton: document.getElementById("tail-visibility-button"),
  diagnosticsVisibilityButton: document.getElementById("diagnostics-visibility-button"),
  toggleUiButton: document.getElementById("toggle-ui-button"),
  uiVisibilityButton: document.getElementById("ui-visibility-button"),
  uiVisibilityPanel: document.getElementById("ui-visibility-panel"),
  uiVisibilitySummary: document.getElementById("ui-visibility-summary"),
  uiVisibilityGrid: document.getElementById("ui-visibility-grid"),
  uiAllOnButton: document.getElementById("ui-all-on-button"),
  uiAllOffButton: document.getElementById("ui-all-off-button"),
  openMetricsButton: document.getElementById("open-metrics-button"),
  closeMetricsButton: document.getElementById("close-metrics-button"),
  openSetupButton: document.getElementById("open-setup-button"),
  closeSetupButton: document.getElementById("close-setup-button"),
  setupBackdrop: document.getElementById("setup-backdrop"),
  setupDrawer: document.getElementById("setup-drawer"),
  metricsBackdrop: document.getElementById("metrics-backdrop"),
  metricsModal: document.getElementById("metrics-modal"),
  mapLayerButton: document.getElementById("map-layer-button"),
  semanticLayerButton: document.getElementById("semantic-layer-button"),
  agentLayerButton: document.getElementById("agent-layer-button"),
  routeHudLayerButton: document.getElementById("route-hud-layer-button"),
  plannedLayerButton: document.getElementById("planned-layer-button"),
  cbsLayerButton: document.getElementById("cbs-layer-button"),
  conflictLayerButton: document.getElementById("conflict-layer-button"),
  conflictCountLayerButton: document.getElementById("conflict-count-layer-button"),
  swapLayerButton: document.getElementById("swap-layer-button"),
};

const rosterRows = new Map();
const priorityRows = new Map();
const kPlannerLayerKeys = ["plannedPaths", "cbsPaths", "conflictCells", "conflictCounts", "swapWaits"];
const kUiSectionSpecs = [
  { key: "topBar", element: "topBar", label: "Top Bar" },
  { key: "leftStack", element: "leftStack", label: "Fleet Panel" },
  { key: "rightStack", element: "rightStack", label: "Right Panel" },
  { key: "mapTools", element: "mapTools", label: "Map Tools" },
  { key: "routeHud", element: "routeHud", label: "Route HUD" },
  { key: "bottomStrip", element: "bottomStrip", label: "Bottom Panels" },
  { key: "sceneLegend", element: "sceneLegend", label: "Legend" },
];
const kBottomPanelSpecs = [
  {
    key: "tail",
    card: "tailCard",
    button: "tailVisibilityButton",
    content: "logsList",
    label: "Important Tail",
  },
  {
    key: "diagnostics",
    card: "diagnosticsCard",
    button: "diagnosticsVisibilityButton",
    content: "statusList",
    label: "Status Feed",
  },
];
const kLayerButtonSpecs = [
  { key: "map", element: "mapLayerButton", tone: "primary" },
  { key: "semantic", element: "semanticLayerButton", tone: "primary" },
  { key: "agents", element: "agentLayerButton", tone: "primary" },
  { key: "routeHud", element: "routeHudLayerButton", tone: "primary" },
  { key: "plannedPaths", element: "plannedLayerButton", tone: "primary", requiresDebug: true },
  { key: "cbsPaths", element: "cbsLayerButton", tone: "planner", requiresDebug: true },
  { key: "conflictCells", element: "conflictLayerButton", tone: "warning", requiresDebug: true },
  { key: "conflictCounts", element: "conflictCountLayerButton", tone: "warning", requiresDebug: true },
  { key: "swapWaits", element: "swapLayerButton", tone: "danger", requiresDebug: true },
];
const kShortcutEditableSelector = [
  "input",
  "select",
  "textarea",
  "button",
  "[contenteditable]",
  "[role='textbox']",
].join(", ");

function formatNumber(value, fractionDigits = 1) {
  if (value === null || value === undefined || Number.isNaN(value)) {
    return "-";
  }
  return Number(value).toFixed(fractionDigits);
}

function formatSigned(value, fractionDigits = 1) {
  if (value === null || value === undefined || Number.isNaN(value)) {
    return "-";
  }
  const numeric = Number(value);
  const sign = numeric > 0 ? "+" : "";
  return `${sign}${numeric.toFixed(fractionDigits)}`;
}

function formatInteger(value) {
  if (value === null || value === undefined || Number.isNaN(value)) {
    return "-";
  }
  return Math.round(Number(value)).toLocaleString();
}

function formatPercent(value, fractionDigits = 1) {
  if (value === null || value === undefined || Number.isNaN(value)) {
    return "-";
  }
  return `${(Number(value) * 100).toFixed(fractionDigits)}%`;
}

function formatMemory(value) {
  if (value === null || value === undefined || Number.isNaN(value)) {
    return "-";
  }
  const numeric = Number(value);
  if (Math.abs(numeric) >= 1024) {
    return `${(numeric / 1024).toFixed(1)} MB`;
  }
  return `${numeric.toFixed(0)} KB`;
}

function reducedMotionPreferred() {
  return Boolean(window.matchMedia?.("(prefers-reduced-motion: reduce)")?.matches);
}

function motionRuntime() {
  return window.motionRuntime?.animate ? window.motionRuntime : null;
}

function stopAnimations(target) {
  if (target?.getAnimations) {
    target.getAnimations().forEach((animation) => {
      try {
        animation.commitStyles?.();
      } catch (_error) {
        // Some animation implementations don't support commitStyles().
      }
      animation.cancel();
    });
  }
}

function animateElement(target, keyframes, options = {}) {
  const runtime = motionRuntime();
  if (!runtime || reducedMotionPreferred() || !target) {
    return null;
  }
  stopAnimations(target);
  return runtime.animate(target, keyframes, {
    duration: 0.24,
    easing: [0.22, 1, 0.36, 1],
    ...options,
  });
}

function captureChildPositions(container, selector) {
  const positions = new Map();
  if (!container) {
    return positions;
  }

  container.querySelectorAll(selector).forEach((node) => {
    const key = node.dataset.agentId || node.dataset.panelKey || node.id;
    if (!key) {
      return;
    }
    positions.set(key, node.getBoundingClientRect());
  });
  return positions;
}

function animateChildLayout(container, selector, previousPositions) {
  if (!container || reducedMotionPreferred()) {
    return;
  }

  const runtime = motionRuntime();
  if (!runtime) {
    return;
  }

  Array.from(container.querySelectorAll(selector)).forEach((node, index) => {
    const clearMotionState = () => {
      node.classList.remove("layout-moving", "layout-entering");
    };
    const key = node.dataset.agentId || node.dataset.panelKey || node.id;
    const previous = key ? previousPositions.get(key) : null;
    const next = node.getBoundingClientRect();
    const previousRank = Number(node.dataset.previousRank ?? node.dataset.rank ?? index);
    const nextRank = Number(node.dataset.rank ?? index);
    const rankShift = Math.max(1, Math.abs(previousRank - nextRank));

    if (previous) {
      const deltaX = previous.left - next.left;
      const deltaY = previous.top - next.top;
      const travel = Math.hypot(deltaX, deltaY);
      if (travel > 1) {
        node.classList.remove("layout-entering");
        node.classList.add("layout-moving");
        animateElement(
          node,
          {
            x: [deltaX, 0],
            y: [deltaY, 0],
            opacity: [0.94, 1],
            scale: [1.018 + Math.min(rankShift, 3) * 0.012, 1],
          },
          {
            ...kPriorityLayoutSpring,
            stiffness: Math.max(360, kPriorityLayoutSpring.stiffness - Math.min(rankShift * 28, 120)),
            damping: kPriorityLayoutSpring.damping + Math.min(rankShift * 2, 8),
            delay: Math.min(index * 0.008, 0.04),
            onComplete: clearMotionState,
          },
        );
      } else {
        clearMotionState();
      }
      return;
    }

    node.classList.remove("layout-moving");
    node.classList.add("layout-entering");
    animateElement(
      node,
      {
        opacity: [0, 1],
        y: [20, 0],
        scale: [0.97, 1],
      },
      {
        ...kPriorityEntrySpring,
        delay: index * 0.014,
        onComplete: clearMotionState,
      },
    );
  });
}

function toneColor(tone = "neutral") {
  switch (tone) {
    case "primary":
      return "#2563eb";
    case "warning":
      return "#d97706";
    case "danger":
      return "#dc3f6a";
    case "success":
      return "#0f9b71";
    case "active":
      return "#2563eb";
    case "neutral":
    default:
      return "#64748b";
  }
}

function titleCase(value) {
  if (!value) {
    return "";
  }
  return String(value)
    .replace(/_/g, " ")
    .replace(/\b\w/g, (letter) => letter.toUpperCase());
}

function priorityBaseWeight(agent) {
  switch (agent?.state) {
    case "returning_with_car":
      return kPriorityReturningWithCar;
    case "going_to_charge":
    case "returning_home_maintenance":
      return kPriorityGoingToCharge;
    case "going_to_park":
    case "going_to_collect":
      return kPriorityMovingTask;
    default:
      return 0;
  }
}

function priorityBaseLabel(agent) {
  switch (agent?.state) {
    case "returning_with_car":
      return "Return Car";
    case "going_to_charge":
      return "Charge Run";
    case "returning_home_maintenance":
      return "Maintenance";
    case "going_to_park":
      return "Park Task";
    case "going_to_collect":
      return "Collect Task";
    default:
      return "Idle";
  }
}

function priorityTone(agent) {
  const stuckSteps = Number(agent?.stuckSteps || 0);
  if (stuckSteps >= kPriorityDeadlockThreshold) {
    return "danger";
  }
  switch (priorityBaseWeight(agent)) {
    case kPriorityReturningWithCar:
      return "primary";
    case kPriorityGoingToCharge:
      return "warning";
    case kPriorityMovingTask:
      return "success";
    default:
      return "neutral";
  }
}

function priorityStuckBoost(agent) {
  const stuckSteps = Math.max(0, Number(agent?.stuckSteps || 0));
  return stuckSteps >= kPriorityDeadlockThreshold
    ? kPriorityStuckBoostHard
    : stuckSteps * kPriorityStuckBoostMult;
}

function prioritySnapshot(agent) {
  const baseWeight = priorityBaseWeight(agent);
  const stuckBoost = priorityStuckBoost(agent);
  const numericId = Number(agent?.id || 0);
  return {
    baseWeight,
    baseLabel: priorityBaseLabel(agent),
    stuckBoost,
    score: baseWeight * 100 + stuckBoost - numericId,
    tone: priorityTone(agent),
    escalated: stuckBoost >= kPriorityStuckBoostHard,
  };
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

function uiSectionVisible(key) {
  return state.uiSections[key] !== false;
}

function visibleUiSectionCount() {
  return kUiSectionSpecs.reduce(
    (count, spec) => count + (uiSectionVisible(spec.key) ? 1 : 0),
    0,
  );
}

function anyUiSectionsVisible() {
  return visibleUiSectionCount() > 0;
}

function allUiSectionsVisible() {
  return visibleUiSectionCount() === kUiSectionSpecs.length;
}

function visibleBottomPanelCount() {
  return kBottomPanelSpecs.reduce(
    (count, spec) => count + (state.bottomPanels[spec.key] ? 1 : 0),
    0,
  );
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

function currentOverlay() {
  return state.frame?.plannerOverlay?.available ? state.frame.plannerOverlay : null;
}

function plannerLayersVisible() {
  return kPlannerLayerKeys.some((key) => state.showLayers[key]);
}

function overlayIncludesAgent(agentIds, agentId) {
  return Array.isArray(agentIds) && agentIds.includes(agentId);
}

function overlayPathForAgent(paths, agentId) {
  return Array.isArray(paths)
    ? paths.find((path) => path.agentId === agentId) || null
    : null;
}

function isGridCoordValid(coord) {
  return Boolean(
    coord &&
      Number.isFinite(coord.x) &&
      Number.isFinite(coord.y) &&
      coord.x >= 0 &&
      coord.y >= 0,
  );
}

function agentById(agentId) {
  return currentAgents().find((agent) => agent.id === agentId) || null;
}

function agentSymbol(agent) {
  const rawSymbol = String(agent?.symbol || "").trim();
  return rawSymbol || "?";
}

function agentCallsign(agent) {
  return agentSymbol(agent);
}

function agentTag(agentId) {
  const agent = agentById(agentId);
  if (agent?.symbol) {
    return String(agent.symbol);
  }
  if (Number.isFinite(agentId) && agentId >= 0) {
    return String(agentId);
  }
  return "-";
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

function fairnessBreakdown() {
  return Array.isArray(state.metrics?.agentFairnessBreakdown)
    ? state.metrics.agentFairnessBreakdown
    : [];
}

function fairnessForAgent(agentId) {
  return fairnessBreakdown().find((item) => item.id === agentId) || null;
}

function maxFairnessValue(key) {
  return fairnessBreakdown().reduce(
    (maxValue, item) => Math.max(maxValue, Number(item?.[key] || 0)),
    0,
  );
}

function agentTasksCompleted(agent) {
  return Number(fairnessForAgent(agent?.id)?.tasksCompleted || 0);
}

function agentTaskShare(agent) {
  const total = Math.max(1, Number(state.metrics?.tasksCompletedTotal || 0));
  return agentTasksCompleted(agent) / total;
}

function agentTaskThroughput(agent) {
  const recordedSteps = Math.max(1, Number(state.metrics?.recordedSteps || state.frame?.hud?.step || 0));
  return (agentTasksCompleted(agent) / recordedSteps) * 100;
}

function agentTaskLoadTone(agent, maxTasksCompleted = maxFairnessValue("tasksCompleted")) {
  const relativeLoad = maxTasksCompleted > 0
    ? agentTasksCompleted(agent) / maxTasksCompleted
    : 0;
  if (relativeLoad >= 0.72) {
    return "primary";
  }
  if (relativeLoad >= 0.36) {
    return "warning";
  }
  return "neutral";
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

function createRouteDebugBadge(text, tone = "neutral") {
  const badge = document.createElement("span");
  badge.className = "route-debug-badge";
  badge.dataset.tone = tone;
  badge.textContent = text;
  return badge;
}

function createRouteLegendItem(kind, label, active = true) {
  const item = document.createElement("div");
  item.className = "route-debug-legend-item";
  item.classList.toggle("inactive", !active);

  const swatch = document.createElement("span");
  swatch.className = "route-legend-swatch";
  swatch.dataset.kind = kind;
  swatch.setAttribute("aria-hidden", "true");

  const text = document.createElement("span");
  text.textContent = label;

  item.append(swatch, text);
  return item;
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

function createEmptyState(title, message) {
  const empty = document.createElement("div");
  empty.className = "telemetry-empty";

  const heading = document.createElement("h3");
  heading.textContent = title;

  const copy = document.createElement("p");
  copy.textContent = message;

  empty.append(heading, copy);
  return empty;
}

function historyEntries(limit = kMetricHistoryLimit) {
  return state.metricHistory.slice(-limit);
}

function metricSeries(key, limit = kMetricHistoryLimit) {
  return historyEntries(limit).map((entry) => {
    const value = Number(entry?.[key]);
    return Number.isFinite(value) ? value : 0;
  });
}

function currentMetricEntry() {
  return historyEntries(1)[0] || null;
}

function previousMetricEntry() {
  const entries = historyEntries(2);
  return entries.length >= 2 ? entries[0] : null;
}

function updateMetricHistory(metrics) {
  state.metrics = metrics || null;
  if (!metrics) {
    state.metricHistory = [];
    return;
  }

  const snapshot = {
    ...metrics,
    frameId: Number(state.frame?.frameId ?? metrics.recordedSteps ?? state.metricHistory.length),
    step: Number(state.frame?.hud?.step ?? metrics.recordedSteps ?? 0),
    activeAgentsLive: activeAgentCount(),
    waitingAgentsLive: waitingAgentCount(),
    stuckAgentsLive: stuckAgentCount(),
    oscillatingAgentsLive: oscillatingAgentCount(),
  };

  const last = state.metricHistory[state.metricHistory.length - 1];
  if (last && last.frameId === snapshot.frameId && last.step === snapshot.step) {
    state.metricHistory[state.metricHistory.length - 1] = snapshot;
  } else {
    state.metricHistory.push(snapshot);
  }

  if (state.metricHistory.length > kMetricHistoryLimit) {
    state.metricHistory = state.metricHistory.slice(-kMetricHistoryLimit);
  }
}

function createSparkline(values, tone = "primary", options = {}) {
  const shell = document.createElement("div");
  shell.className = "sparkline-shell";
  const width = options.width || 220;
  const height = options.height || 58;
  const padding = 8;
  shell.style.minHeight = `${height}px`;

  const svg = document.createElementNS("http://www.w3.org/2000/svg", "svg");
  svg.setAttribute("viewBox", `0 0 ${width} ${height}`);
  svg.setAttribute("preserveAspectRatio", "none");

  const chartValues = values.length > 1 ? values : [values[0] ?? 0, values[0] ?? 0];
  const min = Math.min(...chartValues);
  const max = Math.max(...chartValues);
  const range = max - min || 1;
  const stepX = chartValues.length > 1 ? (width - padding * 2) / (chartValues.length - 1) : 0;
  const points = chartValues.map((value, index) => {
    const numeric = Number.isFinite(value) ? value : min;
    return {
      x: padding + index * stepX,
      y: height - padding - ((numeric - min) / range) * (height - padding * 2),
    };
  });

  [0.25, 0.5, 0.75].forEach((fraction) => {
    const line = document.createElementNS("http://www.w3.org/2000/svg", "line");
    line.setAttribute("x1", String(padding));
    line.setAttribute("x2", String(width - padding));
    line.setAttribute("y1", String(padding + (height - padding * 2) * fraction));
    line.setAttribute("y2", String(padding + (height - padding * 2) * fraction));
    line.setAttribute("stroke", "rgba(91, 111, 151, 0.14)");
    line.setAttribute("stroke-width", "1");
    svg.append(line);
  });

  const stroke = toneColor(tone);
  const area = document.createElementNS("http://www.w3.org/2000/svg", "path");
  area.setAttribute(
    "d",
    [
      `M ${points[0].x} ${height - padding}`,
      ...points.map((point) => `L ${point.x} ${point.y}`),
      `L ${points[points.length - 1].x} ${height - padding}`,
      "Z",
    ].join(" "),
  );
  area.setAttribute("fill", hexToRgba(stroke, 0.14));
  svg.append(area);

  const polyline = document.createElementNS("http://www.w3.org/2000/svg", "polyline");
  polyline.setAttribute(
    "points",
    points.map((point) => `${point.x},${point.y}`).join(" "),
  );
  polyline.setAttribute("fill", "none");
  polyline.setAttribute("stroke", stroke);
  polyline.setAttribute("stroke-width", String(options.strokeWidth || 3));
  polyline.setAttribute("stroke-linecap", "round");
  polyline.setAttribute("stroke-linejoin", "round");
  svg.append(polyline);

  const dot = document.createElementNS("http://www.w3.org/2000/svg", "circle");
  const lastPoint = points[points.length - 1];
  dot.setAttribute("cx", String(lastPoint.x));
  dot.setAttribute("cy", String(lastPoint.y));
  dot.setAttribute("r", String(options.dotRadius || 3.5));
  dot.setAttribute("fill", stroke);
  dot.setAttribute("stroke", "rgba(255, 255, 255, 0.95)");
  dot.setAttribute("stroke-width", "2");
  svg.append(dot);

  shell.append(svg);
  return shell;
}

function createTrendDelta(spec) {
  const currentEntry = currentMetricEntry();
  const previousEntry = previousMetricEntry();
  const currentValue = Number(currentEntry?.[spec.key]);
  const previousValue = Number(previousEntry?.[spec.key]);

  const delta = document.createElement("span");
  delta.className = "trend-delta flat";

  if (!Number.isFinite(currentValue) || !Number.isFinite(previousValue)) {
    delta.textContent = "warming up";
    return delta;
  }

  const diff = currentValue - previousValue;
  if (Math.abs(diff) < 0.0001) {
    delta.textContent = "flat";
    return delta;
  }

  const better = spec.improvesWhen === "down" ? diff < 0 : diff > 0;
  delta.className = `trend-delta ${better ? "up" : "down"}`;
  delta.textContent = `${spec.deltaFormatter(diff)} vs prev`;
  return delta;
}

function createTrendCard(spec, compact = false) {
  const card = document.createElement("div");
  card.className = compact ? "planner-trend-card" : "metric-trend-card";
  if (!compact) {
    card.dataset.tone = spec.tone;
  }

  const head = document.createElement("div");
  head.className = compact ? "planner-trend-head" : "metric-trend-head";

  const label = document.createElement("div");
  label.className = compact ? "planner-trend-label" : "metric-trend-label";
  label.textContent = spec.label;

  const value = document.createElement("div");
  value.className = compact ? "planner-trend-value" : "metric-trend-value";
  value.textContent = spec.formatter(state.metrics?.[spec.key]);

  head.append(label, createTrendDelta(spec));

  card.append(head, value);
  card.append(
    createSparkline(metricSeries(spec.key), spec.tone, {
      height: compact ? 52 : 82,
      strokeWidth: compact ? 2.6 : 3.2,
      dotRadius: compact ? 3 : 4,
    }),
  );

  if (!compact) {
    const foot = document.createElement("div");
    foot.className = "metric-trend-foot";
    foot.textContent = spec.footnote;
    card.append(foot);
  }

  return card;
}

function createProgressRow(label, valueText, fraction, tone = "primary") {
  const row = document.createElement("div");
  row.className = "progress-row";

  const head = document.createElement("div");
  head.className = "progress-head";

  const labelNode = document.createElement("div");
  labelNode.className = "progress-label";
  labelNode.textContent = label;

  const valueNode = document.createElement("div");
  valueNode.className = "progress-value";
  valueNode.textContent = valueText;

  const track = document.createElement("div");
  track.className = "progress-track";

  const fill = document.createElement("div");
  fill.className = `progress-fill ${tone}`;
  fill.style.width = `${Math.max(0, Math.min(100, Number(fraction || 0) * 100))}%`;
  track.append(fill);

  head.append(labelNode, valueNode);
  row.append(head, track);
  return row;
}

function createInfographicCard(title, copy, rows) {
  const card = document.createElement("div");
  card.className = "infographic-card";

  const heading = document.createElement("div");

  const titleNode = document.createElement("div");
  titleNode.className = "infographic-title";
  titleNode.textContent = title;

  const copyNode = document.createElement("div");
  copyNode.className = "infographic-copy";
  copyNode.textContent = copy;

  heading.append(titleNode, copyNode);

  const list = document.createElement("div");
  list.className = "progress-list";
  rows.forEach((row) => list.append(row));

  card.append(heading, list);
  return card;
}

function createDistributionStat(label, value) {
  const stat = document.createElement("div");
  stat.className = "distribution-stat";

  const statLabel = document.createElement("div");
  statLabel.className = "distribution-stat-label";
  statLabel.textContent = label;

  const statValue = document.createElement("div");
  statValue.className = "distribution-stat-value";
  statValue.textContent = value;

  stat.append(statLabel, statValue);
  return stat;
}

function createDistributionCard(title, copy, summary, formatter) {
  const card = document.createElement("div");
  card.className = "distribution-card";

  const heading = document.createElement("div");

  const titleNode = document.createElement("div");
  titleNode.className = "distribution-title";
  titleNode.textContent = title;

  const copyNode = document.createElement("div");
  copyNode.className = "distribution-copy";
  copyNode.textContent = copy;

  heading.append(titleNode, copyNode);

  const stats = document.createElement("div");
  stats.className = "distribution-summary";
  stats.append(
    createDistributionStat("Min", formatter(summary?.min)),
    createDistributionStat("Avg", formatter(summary?.avg)),
    createDistributionStat("Max", formatter(summary?.max)),
  );

  const band = document.createElement("div");
  band.className = "distribution-band";

  const fill = document.createElement("div");
  fill.className = "distribution-band-fill";
  const max = Math.max(0, Number(summary?.max || 0));
  const min = Math.max(0, Number(summary?.min || 0));
  const avg = Math.max(0, Number(summary?.avg || 0));
  const minFraction = max > 0 ? min / max : 0;
  fill.style.left = `${Math.max(0, Math.min(100, minFraction * 100))}%`;
  fill.style.width = `${Math.max(8, 100 - minFraction * 100)}%`;
  band.append(fill);

  const avgPin = document.createElement("div");
  avgPin.className = "distribution-pin";
  avgPin.style.left = `${Math.max(0, Math.min(100, max > 0 ? (avg / max) * 100 : 0))}%`;
  band.append(avgPin);

  const meta = document.createElement("div");
  meta.className = "distribution-meta";
  meta.append(
    Object.assign(document.createElement("span"), {
      textContent: `CV ${formatNumber(summary?.coefficientOfVariation, 2)}`,
    }),
    Object.assign(document.createElement("span"), {
      textContent: `Min/max ${formatNumber(summary?.minMaxRatio, 2)}`,
    }),
  );

  card.append(heading, stats, band, meta);
  return card;
}

function createFairnessRow(item, maxima) {
  const row = document.createElement("div");
  row.className = "fairness-row";

  const head = document.createElement("div");
  head.className = "fairness-head";

  const titleWrap = document.createElement("div");
  const title = document.createElement("div");
  title.className = "fairness-title";
  title.textContent = `Agent ${item.symbol || "?"}`;

  const subtitle = document.createElement("div");
  subtitle.className = "fairness-subtitle";
  subtitle.textContent = `id ${item.id} · tasks ${formatInteger(item.tasksCompleted)} · distance ${formatNumber(item.distanceCells, 1)} · idle ${formatInteger(item.idleSteps)}`;

  titleWrap.append(title, subtitle);

  const swatch = document.createElement("div");
  swatch.className = "agent-swatch";
  const color = deterministicAgentColor(item);
  swatch.style.background = `linear-gradient(135deg, ${color}, ${color}cc)`;
  swatch.textContent = item.symbol || "?";

  head.append(titleWrap, swatch);

  const meters = document.createElement("div");
  meters.className = "fairness-meters";

  [
    ["Tasks", item.tasksCompleted, maxima.tasksCompleted, "primary", formatInteger],
    ["Distance", item.distanceCells, maxima.distanceCells, "warning", (value) => formatNumber(value, 1)],
    ["Idle", item.idleSteps, maxima.idleSteps, "danger", formatInteger],
  ].forEach(([label, value, maxValue, tone, formatter]) => {
    const meterRow = document.createElement("div");
    meterRow.className = "fairness-meter-row";

    const meterLabel = document.createElement("div");
    meterLabel.className = "fairness-meter-label";
    meterLabel.textContent = label;

    const track = document.createElement("div");
    track.className = "fairness-meter-track";

    const fill = document.createElement("div");
    fill.className = `fairness-meter-fill ${tone}`;
    const percent = maxValue > 0 ? (Number(value || 0) / Number(maxValue)) * 100 : 0;
    fill.style.width = `${percent > 0 ? Math.max(6, percent) : 0}%`;
    track.append(fill);

    const meterValue = document.createElement("div");
    meterValue.className = "fairness-meter-value";
    meterValue.textContent = formatter(value);

    meterRow.append(meterLabel, track, meterValue);
    meters.append(meterRow);
  });

  row.append(head, meters);
  return row;
}

function createMetricSection(section) {
  const card = document.createElement("div");
  card.className = "metric-section";

  const title = document.createElement("div");
  title.className = "metric-section-title";
  title.textContent = section.title;

  const copy = document.createElement("div");
  copy.className = "metric-section-copy";
  copy.textContent = section.copy;

  const grid = document.createElement("div");
  grid.className = "metric-section-grid";
  section.items.forEach((item) => {
    grid.append(createMetricTile(item.label, item.format(state.metrics?.[item.key]), item.tone || "neutral"));
  });

  card.append(title, copy, grid);
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

function syncUiSectionClasses() {
  kUiSectionSpecs.forEach((spec) => {
    const element = elements[spec.element];
    if (!element) {
      return;
    }
    element.classList.toggle("is-hidden", !uiSectionVisible(spec.key));
  });

  state.uiVisible = anyUiSectionsVisible();
  elements.appShell.classList.toggle("ui-hidden", !state.uiVisible);
  elements.toggleUiButton.textContent = state.uiVisible ? "Hide UI" : "Show UI";
  elements.toggleUiButton.title = state.uiVisible
    ? "Hide every overlay panel"
    : "Restore every overlay panel";
}

function renderUiVisibilityPanel() {
  const enabledCount = visibleUiSectionCount();
  elements.uiVisibilitySummary.textContent =
    `${enabledCount}/${kUiSectionSpecs.length} on`;
  elements.uiVisibilityButton.setAttribute(
    "aria-expanded",
    String(state.isUiVisibilityPanelOpen),
  );

  elements.uiVisibilityGrid.innerHTML = "";
  kUiSectionSpecs.forEach((spec) => {
    const button = document.createElement("button");
    button.type = "button";
    button.className = "ui-control-chip";
    if (uiSectionVisible(spec.key)) {
      button.classList.add("active");
    }
    button.dataset.uiSection = spec.key;
    button.setAttribute("aria-pressed", String(uiSectionVisible(spec.key)));
    button.textContent = spec.label;
    button.addEventListener("click", () => {
      setUiSectionVisible(spec.key, !uiSectionVisible(spec.key));
    });
    elements.uiVisibilityGrid.append(button);
  });

  elements.uiAllOnButton.disabled = allUiSectionsVisible();
  elements.uiAllOffButton.disabled = !anyUiSectionsVisible();
}

function setUiVisibilityPanelOpen(open) {
  state.isUiVisibilityPanelOpen = Boolean(open);
  elements.uiVisibilityPanel.classList.toggle("hidden", !state.isUiVisibilityPanelOpen);
  renderUiVisibilityPanel();
  if (state.isUiVisibilityPanelOpen) {
    animateElement(
      elements.uiVisibilityPanel,
      {
        opacity: [0, 1],
        y: [18, 0],
        scale: [0.985, 1],
      },
      { duration: 0.22 },
    );
  }
}

function setAllUiSections(visible) {
  kUiSectionSpecs.forEach((spec) => {
    state.uiSections[spec.key] = Boolean(visible);
  });
  syncUiSectionClasses();
  renderUiVisibilityPanel();
  layoutViewportEmpty();
  renderScene();
  renderRouteHud();
}

function setUiSectionVisible(key, visible) {
  if (!Object.prototype.hasOwnProperty.call(state.uiSections, key)) {
    return;
  }
  state.uiSections[key] = Boolean(visible);
  syncUiSectionClasses();
  renderUiVisibilityPanel();
  layoutViewportEmpty();
  renderScene();
  renderRouteHud();
}

function setUiVisible(visible) {
  setAllUiSections(Boolean(visible));
}

function setFleetView(view, options = {}) {
  const nextView = view === "priority" ? "priority" : "roster";
  state.fleetView = nextView;

  const rosterActive = nextView === "roster";
  elements.fleetRosterTab.classList.toggle("active", rosterActive);
  elements.fleetPriorityTab.classList.toggle("active", !rosterActive);
  elements.fleetRosterTab.setAttribute("aria-selected", String(rosterActive));
  elements.fleetPriorityTab.setAttribute("aria-selected", String(!rosterActive));
  elements.fleetRosterPane.classList.toggle("hidden", !rosterActive);
  elements.fleetPriorityPane.classList.toggle("hidden", rosterActive);

  if (options.animate !== false) {
    const activePane = rosterActive ? elements.fleetRosterPane : elements.fleetPriorityPane;
    animateElement(
      activePane,
      {
        opacity: [0, 1],
        y: [16, 0],
      },
      { duration: 0.2 },
    );

    const rowSelector = rosterActive ? ".roster-row" : ".priority-row";
    activePane.querySelectorAll(rowSelector).forEach((row, index) => {
      animateElement(
        row,
        {
          opacity: [0, 1],
          y: [12, 0],
          scale: [0.99, 1],
        },
        {
          duration: 0.22,
          delay: index * 0.015,
        },
      );
    });
  }
}

function setRightStackView(view, options = {}) {
  const nextView = view === "planner" || view === "debug" ? view : "telemetry";
  state.rightStackView = nextView;

  const specs = [
    {
      name: "telemetry",
      tab: elements.rightTelemetryTab,
      pane: elements.rightTelemetryPane,
      rowSelector: ".telemetry-grid > *, .telemetry-empty",
    },
    {
      name: "planner",
      tab: elements.rightPlannerTab,
      pane: elements.rightPlannerPane,
      rowSelector: ".planner-summary-grid > *, .planner-trend-strip > *",
    },
    {
      name: "debug",
      tab: elements.rightDebugTab,
      pane: elements.rightDebugPane,
      rowSelector: ".debug-output, .header-actions > *",
    },
  ];

  specs.forEach((spec) => {
    const active = spec.name === nextView;
    spec.tab.classList.toggle("active", active);
    spec.tab.setAttribute("aria-selected", String(active));
    spec.pane.classList.toggle("hidden", !active);
  });

  if (options.animate === false) {
    return;
  }

  const activeSpec = specs.find((spec) => spec.name === nextView);
  if (!activeSpec) {
    return;
  }

  animateElement(
    activeSpec.pane,
    {
      opacity: [0, 1],
      y: [16, 0],
    },
    { duration: 0.2 },
  );

  activeSpec.pane.querySelectorAll(activeSpec.rowSelector).forEach((node, index) => {
    animateElement(
      node,
      {
        opacity: [0, 1],
        y: [10, 0],
      },
      {
        duration: 0.18,
        delay: index * 0.012,
      },
    );
  });
}

function setPriorityBoardMode(mode) {
  const nextMode = mode === "minimal" ? "minimal" : "detailed";
  const detailed = nextMode === "detailed";
  state.priorityBoardMode = nextMode;

  elements.priorityDetailedTab.classList.toggle("active", detailed);
  elements.priorityMinimalTab.classList.toggle("active", !detailed);
  elements.priorityDetailedTab.setAttribute("aria-selected", String(detailed));
  elements.priorityMinimalTab.setAttribute("aria-selected", String(!detailed));
  elements.priorityBoard.classList.toggle("minimal", !detailed);
  elements.prioritySummaryStrip.classList.toggle("hidden", !detailed);
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
  const debugEnabled = effectiveDebugCaptureEnabled();
  kLayerButtonSpecs.forEach((spec) => {
    const button = elements[spec.element];
    if (!button) {
      return;
    }
    button.classList.toggle("active", Boolean(state.showLayers[spec.key]));
    button.dataset.tone = spec.tone || "primary";
    button.disabled = Boolean(spec.requiresDebug && !debugEnabled);
  });
}

function syncInteractiveState() {
  const hasSession = Boolean(state.sessionId);

  elements.validateButton.disabled = state.busy || state.running;
  elements.startButton.disabled = state.busy || state.running;
  elements.viewportStartButton.disabled = state.busy || state.running;
  elements.runButton.disabled = state.busy || state.running || !hasSession;
  elements.pauseButton.disabled = state.busy || !hasSession || (!state.running && state.paused);
  elements.stepButton.disabled = state.busy || state.running || !hasSession;
  elements.resetSessionButton.disabled = state.busy || state.running || !hasSession;
  elements.debugButton.disabled = state.busy || state.running || !hasSession;

  syncLayerButtons();

  if (!hasSession) {
    elements.runControlHelp.textContent =
      "Load a session first to render the map and enable the controls.";
    return;
  }

  if (state.running) {
    elements.runControlHelp.textContent =
      "Continuous run is active. Press Space to pause, or let the session play until completion.";
    return;
  }

  if (state.paused) {
    elements.runControlHelp.textContent =
      "Session loaded and paused. Press Space to run, S to step, or R to reset to setup.";
    return;
  }

  elements.runControlHelp.textContent =
    "Session is live. Press Space to pause or R to reset to setup.";
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
  elements.plannerSummaryGrid.innerHTML = "";
  elements.plannerTrendStrip.innerHTML = "";

  if (!state.metrics) {
    elements.plannerSummaryGrid.append(createMetricTile("Status", "Load a session", "neutral"));
    return;
  }

  const overlay = state.frame?.plannerOverlay;
  const cards = [
    ["Avg plan ms", `${formatNumber(state.metrics?.avgPlanningTimeMs, 2)} ms`, "primary"],
    ["Max plan ms", `${formatNumber(state.metrics?.maxPlanningTimeMs, 2)} ms`, "primary"],
    ["Avg CPU ms", `${formatNumber(state.metrics?.avgCpuTimeMs, 2)} ms`, "neutral"],
    ["Throughput", formatNumber(state.metrics?.throughput, 2), "primary"],
    ["Outstanding", formatInteger(state.metrics?.outstandingTaskCount), "warning"],
    ["Deadlocks", String(state.metrics?.deadlockCount ?? "-"), Number(state.metrics?.deadlockCount || 0) > 0 ? "danger" : "neutral"],
    [
      "Wait edges",
      overlay?.available
        ? `${formatInteger(overlay.waitEdgeCount)} live`
        : formatNumber(state.metrics?.plannerWaitEdgesPerStep, 2),
      Number(overlay?.waitEdgeCount || state.metrics?.plannerWaitEdgesPerStep || 0) > 0
        ? "warning"
        : "neutral",
    ],
    [
      "CBS success",
      Number(state.metrics?.plannerCbsAttemptCount || 0) > 0
        ? formatPercent(state.metrics?.plannerCbsSuccessRate, 1)
        : "0/0",
      Number(state.metrics?.plannerCbsFailureCount || 0) > 0 ? "warning" : "neutral",
    ],
  ];

  cards.forEach(([label, value, tone]) => {
    elements.plannerSummaryGrid.append(createMetricTile(label, value, tone));
  });

  trendDeckSpecs.forEach((spec) => {
    elements.plannerTrendStrip.append(createTrendCard(spec, true));
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
  if (!state.metrics) {
    [
      elements.metricsHeroGrid,
      elements.metricsInfographicsGrid,
      elements.metricsDistributionGrid,
      elements.metricsFairnessList,
      elements.metricsSections,
    ].forEach((container) => {
      container.innerHTML = "";
      container.append(
        createEmptyState(
          "No live metrics yet",
          "Load a session and advance the simulation to populate this dashboard.",
        ),
      );
    });
    return;
  }

  const metrics = state.metrics;
  const agents = currentAgents();
  const totalAgents = Math.max(agents.length, Number(metrics.activeAgents || 0), 1);
  const outstandingPeak = Math.max(1, Number(metrics.peakOutstandingTaskCount || 0), Number(metrics.outstandingTaskCount || 0));
  const oldestPeak = Math.max(1, Number(metrics.peakOldestQueuedRequestAge || 0), Number(metrics.oldestQueuedRequestAge || 0));

  elements.metricsHeroGrid.innerHTML = "";
  trendDeckSpecs.forEach((spec) => {
    elements.metricsHeroGrid.append(createTrendCard(spec));
  });

  elements.metricsInfographicsGrid.innerHTML = "";
  elements.metricsInfographicsGrid.append(
    createInfographicCard("Queue pressure", "Backlog growth, in-flight work, and request age.", [
      createProgressRow("Queued", formatInteger(metrics.queuedTaskCount), Number(metrics.queuedTaskCount || 0) / outstandingPeak, "warning"),
      createProgressRow("In flight", formatInteger(metrics.inFlightTaskCount), Number(metrics.inFlightTaskCount || 0) / outstandingPeak, "primary"),
      createProgressRow("Outstanding", formatInteger(metrics.outstandingTaskCount), Number(metrics.outstandingTaskCount || 0) / outstandingPeak, "warning"),
      createProgressRow("Oldest age", formatInteger(metrics.oldestQueuedRequestAge), Number(metrics.oldestQueuedRequestAge || 0) / oldestPeak, "danger"),
    ]),
  );
  elements.metricsInfographicsGrid.append(
    createInfographicCard("Planner health", "Success ratios and execution quality from the backend planner.", [
      createProgressRow("Planning CPU", formatPercent(metrics.planningCpuShare, 1), metrics.planningCpuShare, "primary"),
      createProgressRow("Valid expansions", formatPercent(metrics.validExpansionRatio, 1), metrics.validExpansionRatio, "success"),
      createProgressRow(
        "CBS success",
        Number(metrics.plannerCbsAttemptCount || 0) > 0 ? formatPercent(metrics.plannerCbsSuccessRate, 1) : "0/0",
        Number(metrics.plannerCbsAttemptCount || 0) > 0 ? Number(metrics.plannerCbsSuccessRate || 0) : 0,
        "primary",
      ),
      createProgressRow("Movement steps", formatPercent(metrics.movementStepRatio, 1), metrics.movementStepRatio, "success"),
    ]),
  );
  elements.metricsInfographicsGrid.append(
    createInfographicCard("Fleet activity", "Real-time AGV distribution derived from the live frame.", [
      createProgressRow("Active", `${activeAgentCount()}/${totalAgents}`, activeAgentCount() / totalAgents, "primary"),
      createProgressRow("Waiting", `${waitingAgentCount()}/${totalAgents}`, waitingAgentCount() / totalAgents, "warning"),
      createProgressRow("Stuck", `${stuckAgentCount()}/${totalAgents}`, stuckAgentCount() / totalAgents, "danger"),
      createProgressRow("Oscillating", `${oscillatingAgentCount()}/${totalAgents}`, oscillatingAgentCount() / totalAgents, "warning"),
    ]),
  );

  elements.metricsDistributionGrid.innerHTML = "";
  elements.metricsDistributionGrid.append(
    createDistributionCard(
      "Tasks per AGV",
      "Fairness spread for completed task allocation.",
      metrics.tasksPerAgentSpread,
      (value) => formatNumber(value, 2),
    ),
    createDistributionCard(
      "Distance per AGV",
      "How evenly travel distance is being shared across the fleet.",
      metrics.distancePerAgentSpread,
      (value) => formatNumber(value, 1),
    ),
    createDistributionCard(
      "Idle steps per AGV",
      "Fleet idle-time dispersion highlights stranded or underused agents.",
      metrics.idleStepsPerAgentSpread,
      formatInteger,
    ),
  );

  elements.metricsFairnessList.innerHTML = "";
  const fairness = Array.isArray(metrics.agentFairnessBreakdown)
    ? [...metrics.agentFairnessBreakdown].sort((left, right) => left.id - right.id)
    : [];
  if (fairness.length === 0) {
    elements.metricsFairnessList.append(
      createEmptyState("No fleet breakdown", "Agent fairness data appears after the backend reports active agents."),
    );
  } else {
    const maxima = fairness.reduce(
      (acc, item) => ({
        tasksCompleted: Math.max(acc.tasksCompleted, Number(item.tasksCompleted || 0)),
        distanceCells: Math.max(acc.distanceCells, Number(item.distanceCells || 0)),
        idleSteps: Math.max(acc.idleSteps, Number(item.idleSteps || 0)),
      }),
      { tasksCompleted: 0, distanceCells: 0, idleSteps: 0 },
    );
    fairness.forEach((item) => {
      elements.metricsFairnessList.append(createFairnessRow(item, maxima));
    });
  }

  elements.metricsSections.innerHTML = "";
  metricSectionSpecs.forEach((section) => {
    elements.metricsSections.append(createMetricSection(section));
  });
}

function priorityWaitTone(waitReason) {
  switch (waitReason) {
    case "stuck":
    case "oscillating":
      return "danger";
    case "priority_yield":
    case "blocked_by_stationary":
    case "rotation":
      return "warning";
    case "goal_action":
    case "charging":
      return "success";
    default:
      return "neutral";
  }
}

function renderBottomPanels() {
  const bottomPanelsVisible = state.bottomPanels.tail || state.bottomPanels.diagnostics;

  elements.tailCard.classList.toggle("panel-collapsed", !bottomPanelsVisible);
  elements.diagnosticsCard.classList.toggle("panel-collapsed", !bottomPanelsVisible);

  kBottomPanelSpecs.forEach((spec) => {
    const button = elements[spec.button];
    const content = elements[spec.content];
    button.textContent = bottomPanelsVisible ? "Lower" : "Raise";
    button.setAttribute("aria-expanded", String(bottomPanelsVisible));
    button.title = bottomPanelsVisible ? "Lower bottom panels" : "Raise bottom panels";
    if (content) {
      content.setAttribute("aria-hidden", String(!bottomPanelsVisible));
    }
  });
}

function setBottomPanelVisible(key, visible) {
  const spec = kBottomPanelSpecs.find((item) => item.key === key);
  const nextVisible = Boolean(visible);
  if (!spec || ((state.bottomPanels.tail || state.bottomPanels.diagnostics) === nextVisible)) {
    return;
  }

  state.bottomPanels.tail = nextVisible;
  state.bottomPanels.diagnostics = nextVisible;
  renderBottomPanels();
  layoutViewportEmpty();
  renderScene();
}

function syncRightStackOffset() {
  if (!elements.rightStack || !elements.mapTools) {
    return;
  }

  if (!uiSectionVisible("rightStack")) {
    elements.rightStack.style.top = "";
    return;
  }

  const mapToolsVisible = uiSectionVisible("mapTools") && !elements.mapTools.classList.contains("is-hidden");
  if (!mapToolsVisible) {
    elements.rightStack.style.top = "";
    return;
  }

  const appRect = elements.appShell.getBoundingClientRect();
  const mapToolsRect = elements.mapTools.getBoundingClientRect();
  const clearSpacing = window.innerWidth <= 1120 ? 18 : 16;
  const baseTop = window.innerWidth <= 1400 ? 260 : 178;
  const nextTop = Math.max(baseTop, Math.round(mapToolsRect.bottom - appRect.top + clearSpacing));
  elements.rightStack.style.top = `${nextTop}px`;
}

function createPriorityRow(agentId) {
  const button = document.createElement("button");
  button.type = "button";
  button.className = "priority-row";
  button.dataset.agentId = String(agentId);
  button.addEventListener("mousedown", (event) => {
    event.preventDefault();
  });
  button.addEventListener("click", () => {
    const nextId = Number(button.dataset.agentId);
    state.selectedAgentId = Number.isFinite(nextId) ? nextId : null;
    renderAll();
  });
  button.addEventListener("mouseenter", () => {
    const nextId = Number(button.dataset.agentId);
    state.hoveredAgentId = Number.isFinite(nextId) ? nextId : null;
    renderRouteHud();
    renderScene();
  });
  button.addEventListener("mouseleave", () => {
    const nextId = Number(button.dataset.agentId);
    if (state.hoveredAgentId === nextId) {
      state.hoveredAgentId = null;
      renderRouteHud();
      renderScene();
    }
  });

  const head = document.createElement("div");
  head.className = "priority-row-head";

  const lead = document.createElement("div");
  lead.className = "priority-row-lead";

  const rank = document.createElement("span");
  rank.className = "priority-rank";

  const avatar = document.createElement("div");
  avatar.className = "priority-avatar";

  const copy = document.createElement("div");
  copy.className = "priority-copy";

  const title = document.createElement("div");
  title.className = "priority-title";

  const subtitle = document.createElement("div");
  subtitle.className = "priority-subtitle";

  copy.append(title, subtitle);
  lead.append(rank, avatar, copy);

  const score = document.createElement("span");
  score.className = "priority-score";
  head.append(lead, score);

  const chipRow = document.createElement("div");
  chipRow.className = "priority-chip-row";

  const baseChip = document.createElement("span");
  const boostChip = document.createElement("span");
  const waitChip = document.createElement("span");
  chipRow.append(baseChip, boostChip, waitChip);

  const meter = document.createElement("div");
  meter.className = "priority-meter";

  const fill = document.createElement("div");
  fill.className = "priority-meter-fill";
  meter.append(fill);

  const meta = document.createElement("div");
  meta.className = "priority-row-meta";

  const baseMeta = document.createElement("span");
  const liveMeta = document.createElement("span");
  meta.append(baseMeta, liveMeta);

  button.append(head, chipRow, meter, meta);

  return {
    button,
    rank,
    avatar,
    title,
    subtitle,
    score,
    baseChip,
    boostChip,
    waitChip,
    fill,
    baseMeta,
    liveMeta,
  };
}

function updatePriorityRow(row, agent, priority, rank, scoreFloor, scoreRange) {
  const minimalMode = state.priorityBoardMode === "minimal";
  const symbol = agentSymbol(agent);
  const previousRank = Number(row.button.dataset.rank ?? rank);
  const rankShift = previousRank - rank;
  row.button.dataset.agentId = String(agent.id);
  row.button.dataset.previousRank = String(previousRank);
  row.button.dataset.rank = String(rank);
  row.button.className = "priority-row";
  row.button.style.zIndex = String(Math.max(1, 200 - rank));
  row.button.classList.toggle("minimal", minimalMode);
  row.button.classList.toggle("promoted", rankShift > 0);
  row.button.classList.toggle("demoted", rankShift < 0);
  if (agent.id === state.selectedAgentId) {
    row.button.classList.add("selected");
  }
  if (rank === 0) {
    row.button.classList.add("leader");
  }
  row.button.title = minimalMode ? `${rank + 1}. ${symbol}` : "";
  row.button.setAttribute(
    "aria-label",
    `${rank === 0 ? "Leader" : `Priority ${rank + 1}`} ${symbol}, ${agentStateShortLabel(agent.state)}`,
  );

  const color = deterministicAgentColor(agent);
  const normalizedScore = Math.max(
    0.04,
    Math.min(1, (priority.score - scoreFloor) / Math.max(1, scoreRange)),
  );

  row.rank.className = "priority-rank";
  if (rank === 0) {
    row.rank.classList.add("leader");
  }
  row.rank.textContent = `#${rank + 1}`;
  row.avatar.style.background = `linear-gradient(135deg, ${color}, ${color}cc)`;
  row.avatar.textContent = symbol;
  row.title.textContent = rank === 0 ? "Leader" : `Agent ${symbol}`;
  row.subtitle.textContent = `${priority.baseLabel} · ${coordText(agent.position)} → ${coordText(agent.goal)}`;
  row.score.textContent = `PRI ${formatInteger(priority.score)}`;

  row.baseChip.className = `priority-chip ${priority.tone}`;
  row.baseChip.textContent = priority.baseLabel;

  row.boostChip.className = `priority-chip ${priority.escalated ? "danger" : "neutral"}`;
  row.boostChip.textContent = `Boost +${formatInteger(priority.stuckBoost)}`;

  row.waitChip.className = `priority-chip ${priorityWaitTone(agent.waitReason)}`;
  row.waitChip.textContent = `Wait ${waitReasonShortLabel(agent.waitReason)}`;

  row.fill.className = `priority-meter-fill ${priority.tone}`;
  row.fill.style.transform = `scaleX(${normalizedScore})`;

  row.baseMeta.textContent = `base ${formatInteger(priority.baseWeight * 100)} · tie-break -${formatInteger(agent.id)}`;
  row.liveMeta.textContent = `${agentStateShortLabel(agent.state)} · stk ${formatInteger(agent.stuckSteps || 0)} · osc ${formatInteger(agent.oscillationSteps || 0)}`;
}

function renderPrioritySummaryStrip(rankedAgents) {
  elements.prioritySummaryStrip.innerHTML = "";

  if (rankedAgents.length === 0) {
    return;
  }

  const leader = rankedAgents[0];
  const nextLeader = rankedAgents[1] || null;
  const criticalCount = rankedAgents.filter(
    ({ agent }) => Number(agent.stuckSteps || 0) >= kPriorityDeadlockThreshold,
  ).length;

  elements.prioritySummaryStrip.append(
    createMiniMetric("Leader", agentCallsign(leader.agent), "active"),
    createMiniMetric(
      "Gap",
      nextLeader
        ? formatInteger(leader.priority.score - nextLeader.priority.score)
        : formatInteger(leader.priority.score),
      nextLeader ? "warning" : "neutral",
    ),
    createMiniMetric(
      "Critical",
      formatInteger(criticalCount),
      criticalCount > 0 ? "danger" : "neutral",
    ),
  );
}

function renderPriorityBoard() {
  const rankedAgents = currentAgents()
    .map((agent) => ({
      agent,
      priority: prioritySnapshot(agent),
    }))
    .sort((left, right) => right.priority.score - left.priority.score);

  if (rankedAgents.length === 0) {
    priorityRows.forEach((row) => {
      row.button.remove();
    });
    priorityRows.clear();
    elements.priorityBoardSummary.textContent = "No agents";
    elements.prioritySummaryStrip.innerHTML = "";
    elements.priorityBoard.replaceChildren(
      createEmptyState(
        "Priority board offline",
        "Load a session to inspect how the planner currently orders AGVs.",
      ),
    );
    return;
  }

  elements.priorityBoardSummary.textContent =
    `${agentCallsign(rankedAgents[0].agent)} · ${formatInteger(rankedAgents.length)} tracked`;
  renderPrioritySummaryStrip(rankedAgents);

  Array.from(elements.priorityBoard.children).forEach((child) => {
    if (!child.classList.contains("priority-row")) {
      child.remove();
    }
  });

  const previous = captureChildPositions(elements.priorityBoard, ".priority-row");
  const activeIds = new Set();
  const scoreFloor = Math.min(
    0,
    rankedAgents.reduce((minimum, entry) => Math.min(minimum, entry.priority.score), 0),
  );
  const scoreCeiling = rankedAgents.reduce(
    (maximum, entry) => Math.max(maximum, entry.priority.score),
    scoreFloor,
  );
  const scoreRange = Math.max(1, scoreCeiling - scoreFloor);

  rankedAgents.forEach(({ agent, priority }, index) => {
    activeIds.add(agent.id);
    let row = priorityRows.get(agent.id);
    if (!row) {
      row = createPriorityRow(agent.id);
      priorityRows.set(agent.id, row);
    }

    updatePriorityRow(row, agent, priority, index, scoreFloor, scoreRange);

    const sibling = elements.priorityBoard.children[index] || null;
    if (row.button !== sibling) {
      elements.priorityBoard.insertBefore(row.button, sibling);
    }
  });

  const staleIds = [];
  priorityRows.forEach((row, agentId) => {
    if (activeIds.has(agentId)) {
      return;
    }
    row.button.remove();
    staleIds.push(agentId);
  });
  staleIds.forEach((agentId) => {
    priorityRows.delete(agentId);
  });

  if (state.fleetView === "priority" && uiSectionVisible("leftStack")) {
    animateChildLayout(elements.priorityBoard, ".priority-row", previous);
  }
}

function createRosterRow(agentId) {
  const button = document.createElement("button");
  button.type = "button";
  button.className = "roster-row";
  button.dataset.agentId = String(agentId);
  button.addEventListener("mousedown", (event) => {
    event.preventDefault();
  });
  button.addEventListener("click", () => {
    const nextId = Number(button.dataset.agentId);
    state.selectedAgentId = Number.isFinite(nextId) ? nextId : null;
    renderAll();
  });
  button.addEventListener("mouseenter", () => {
    const nextId = Number(button.dataset.agentId);
    state.hoveredAgentId = Number.isFinite(nextId) ? nextId : null;
    renderRouteHud();
    renderScene();
  });
  button.addEventListener("mouseleave", () => {
    const nextId = Number(button.dataset.agentId);
    if (state.hoveredAgentId === nextId) {
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

  const textGroup = document.createElement("div");

  const title = document.createElement("div");
  title.className = "roster-title";

  const subtitle = document.createElement("div");
  subtitle.className = "roster-subtitle";

  textGroup.append(title, subtitle);
  primary.append(swatch, textGroup);

  const wait = document.createElement("span");
  head.append(primary, wait);

  const body = document.createElement("div");
  body.className = "roster-body";

  const statusRow = document.createElement("div");
  statusRow.className = "roster-status-row";

  const stateText = document.createElement("div");
  stateText.className = "roster-meta";

  const signalGroup = document.createElement("div");
  signalGroup.className = "roster-signal-group";

  const taskChip = document.createElement("span");
  const batteryChip = document.createElement("span");

  signalGroup.append(taskChip, batteryChip);
  statusRow.append(stateText, signalGroup);

  const taskRail = document.createElement("div");
  taskRail.className = "roster-task-rail";

  const taskHead = document.createElement("div");
  taskHead.className = "roster-task-head";

  const taskLabel = document.createElement("span");
  taskLabel.className = "roster-task-label";
  taskLabel.textContent = "Task throughput";

  const taskValue = document.createElement("span");
  taskValue.className = "roster-task-value";

  taskHead.append(taskLabel, taskValue);

  const taskMeter = document.createElement("div");
  taskMeter.className = "roster-task-meter";

  const taskFill = document.createElement("div");
  taskMeter.append(taskFill);

  const taskMeta = document.createElement("div");
  taskMeta.className = "roster-task-meta";

  const shareText = document.createElement("span");
  const countText = document.createElement("span");

  taskMeta.append(shareText, countText);

  const battery = document.createElement("div");
  battery.className = "roster-battery";

  const batteryMeta = document.createElement("div");
  batteryMeta.className = "roster-battery-meta";

  const batteryCopy = document.createElement("span");
  const distText = document.createElement("span");

  batteryMeta.append(batteryCopy, distText);

  const meter = document.createElement("div");
  meter.className = "battery-meter";

  const fill = document.createElement("div");
  meter.append(fill);

  battery.append(batteryMeta, meter);
  taskRail.append(taskHead, taskMeter, taskMeta);
  body.append(statusRow, taskRail, battery);
  button.append(head, body);

  return {
    button,
    swatch,
    title,
    subtitle,
    wait,
    stateText,
    taskChip,
    batteryChip,
    taskValue,
    taskFill,
    shareText,
    countText,
    batteryCopy,
    distText,
    fill,
  };
}

function updateRosterRow(row, agent, maxTasksCompleted) {
  row.button.dataset.agentId = String(agent.id);
  row.button.className = "roster-row";
  if (agent.id === state.selectedAgentId) {
    row.button.classList.add("selected");
  }
  if (agent.waitReason && agent.waitReason !== "none") {
    row.button.classList.add("waiting");
  }

  const color = deterministicAgentColor(agent);
  const taskTone = agentTaskLoadTone(agent, maxTasksCompleted);

  row.swatch.style.background = `linear-gradient(135deg, ${color}, ${color}cc)`;
  row.swatch.textContent = agent.symbol || "?";
  row.title.textContent = `Agent ${agent.symbol || "?"}`;
  row.subtitle.textContent = `${coordText(agent.position)} → ${coordText(agent.goal)}`;
  row.wait.className = `wait-badge ${waitReasonClass(agent.waitReason)}`;
  row.wait.textContent = waitReasonShortLabel(agent.waitReason);
  row.stateText.textContent = titleCase(agent.state || "idle");
  row.taskChip.className = `task-chip ${taskTone}`;
  row.taskChip.textContent = `TASK ${formatInteger(agentTasksCompleted(agent))}`;
  row.batteryChip.className = `battery-chip ${chargeReserveTone(agent)}`;
  row.batteryChip.textContent = `${
    (agent.chargeTimer || 0) > 0 || agent.state === "charging" ? "CHG" : "BAT"
  } ${chargeReservePercent(agent)}%`;
  row.taskValue.textContent = `${formatNumber(agentTaskThroughput(agent), 2)} / 100 step`;
  row.taskFill.className = `roster-task-fill ${taskTone}`;
  row.taskFill.style.width = `${Math.max(
    0,
    Math.min(
      100,
      maxTasksCompleted > 0
        ? (agentTasksCompleted(agent) / maxTasksCompleted) * 100
        : 0,
    ),
  )}%`;
  row.shareText.textContent = `share ${formatPercent(agentTaskShare(agent), 0)}`;
  row.countText.textContent = `${formatInteger(agentTasksCompleted(agent))} completed`;
  row.batteryCopy.textContent = chargeReserveMeta(agent);
  row.distText.textContent = `${Number(agent.totalDistanceTraveled || 0).toFixed(1)} cells traveled`;
  row.fill.className = `battery-fill ${chargeReserveTone(agent)}`;
  row.fill.style.width = `${Math.max(8, chargeReservePercent(agent))}%`;
}

function renderRoster() {
  const agents = currentAgents();
  const maxTasksCompleted = maxFairnessValue("tasksCompleted");

  if (agents.length === 0) {
    rosterRows.forEach((row) => {
      row.button.remove();
    });
    rosterRows.clear();
    const empty = document.createElement("div");
    empty.className = "telemetry-empty";
    empty.innerHTML = "<h3>No agents yet</h3><p>Load a session to inspect each AGV in the roster.</p>";
    elements.agentRosterList.replaceChildren(empty);
    return;
  }

  Array.from(elements.agentRosterList.children).forEach((child) => {
    if (!child.classList.contains("roster-row")) {
      child.remove();
    }
  });

  const activeIds = new Set();
  agents.forEach((agent, index) => {
    activeIds.add(agent.id);
    let row = rosterRows.get(agent.id);
    if (!row) {
      row = createRosterRow(agent.id);
      rosterRows.set(agent.id, row);
    }

    updateRosterRow(row, agent, maxTasksCompleted);

    const sibling = elements.agentRosterList.children[index] || null;
    if (row.button !== sibling) {
      elements.agentRosterList.insertBefore(row.button, sibling);
    }
  });

  const staleIds = [];
  rosterRows.forEach((row, agentId) => {
    if (activeIds.has(agentId)) {
      return;
    }
    row.button.remove();
    staleIds.push(agentId);
  });
  staleIds.forEach((agentId) => {
    rosterRows.delete(agentId);
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

function renderRouteHudDebug(agent) {
  const overlay = currentOverlay();
  const showDebug = Boolean(
    agent &&
      overlay &&
      plannerLayersVisible() &&
      effectiveDebugCaptureEnabled(),
  );

  elements.routeDebugPanel.classList.toggle("hidden", !showDebug);
  if (!showDebug) {
    elements.routeDebugMeta.textContent = "Planner overlay inactive.";
    elements.routeDebugBadges.innerHTML = "";
    elements.routeDebugLegend.innerHTML = "";
    return;
  }

  const plannedPath = overlayPathForAgent(overlay.plannedPaths, agent.id);
  const cbsPath = overlayPathForAgent(overlay.cbsPaths, agent.id);
  const cbsPathCount = Array.isArray(overlay.cbsPaths) ? overlay.cbsPaths.length : 0;
  const cbsSummary = overlay.usedCbs
    ? cbsPathCount > 0
      ? ` · CBS ${formatInteger(cbsPathCount)} route${cbsPathCount === 1 ? "" : "s"}`
      : " · CBS live"
    : "";

  elements.routeDebugMeta.textContent =
    `${titleCase(overlay.algorithm || "default")} · horizon ${formatInteger(overlay.horizon)} · waits ${formatInteger(overlay.waitEdgeCount)}${cbsSummary}`;

  elements.routeDebugBadges.innerHTML = "";
  if (overlay.leaderAgentId >= 0) {
    elements.routeDebugBadges.append(
      createRouteDebugBadge(
        overlay.leaderAgentId === agent.id ? "Leader" : `Leader ${agentTag(overlay.leaderAgentId)}`,
        overlay.leaderAgentId === agent.id ? "success" : "neutral",
      ),
    );
  }
  if (overlayIncludesAgent(overlay.sccParticipantAgentIds, agent.id)) {
    elements.routeDebugBadges.append(createRouteDebugBadge("SCC cluster", "danger"));
  }
  if (overlayIncludesAgent(overlay.yieldAgentIds, agent.id)) {
    elements.routeDebugBadges.append(createRouteDebugBadge("Yielding", "warning"));
  }
  if (overlayIncludesAgent(overlay.pullOverAgentIds, agent.id)) {
    elements.routeDebugBadges.append(createRouteDebugBadge("Pull-over", "danger"));
  }
  if (plannedPath?.cells?.length >= 2) {
    elements.routeDebugBadges.append(
      createRouteDebugBadge(`Plan ${formatInteger(plannedPath.cells.length)} cells`, "primary"),
    );
  }
  if (cbsPath?.cells?.length >= 2) {
    elements.routeDebugBadges.append(
      createRouteDebugBadge(`CBS ${formatInteger(cbsPath.cells.length)} cells`, "success"),
    );
  } else if (overlay.usedCbs) {
    elements.routeDebugBadges.append(createRouteDebugBadge("CBS active", "success"));
  }
  if (elements.routeDebugBadges.childElementCount === 0) {
    elements.routeDebugBadges.append(createRouteDebugBadge("Monitoring only", "neutral"));
  }

  elements.routeDebugLegend.innerHTML = "";
  [
    ["planned", "Planned path", state.showLayers.plannedPaths],
    ["cbs", "CBS reroute", state.showLayers.cbsPaths],
    ["vertex", "Conflict cell", state.showLayers.conflictCells || state.showLayers.conflictCounts],
    ["swap", "Swap wait", state.showLayers.swapWaits],
  ].forEach(([kind, label, active]) => {
    elements.routeDebugLegend.append(createRouteLegendItem(kind, label, active));
  });
}

function renderRouteHud() {
  const agent = displayedAgent();

  if (
    !agent ||
    !state.sessionId ||
    !uiSectionVisible("routeHud") ||
    !state.showLayers.routeHud
  ) {
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

  renderRouteHudDebug(agent);
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
  if (!anyUiSectionsVisible()) {
    const insetX = Math.min(96, Math.max(32, Math.floor(viewportWidth * 0.06)));
    const insetY = Math.min(92, Math.max(28, Math.floor(viewportHeight * 0.05)));
    return {
      left: insetX,
      right: insetX,
      top: insetY,
      bottom: insetY,
      availableWidth: Math.max(240, viewportWidth - insetX * 2),
      availableHeight: Math.max(220, viewportHeight - insetY * 2),
    };
  }

  const left = uiSectionVisible("leftStack") || uiSectionVisible("routeHud")
    ? Math.min(430, Math.max(220, Math.floor(viewportWidth * 0.24)))
    : Math.min(92, Math.max(28, Math.floor(viewportWidth * 0.06)));
  const right = uiSectionVisible("rightStack") || uiSectionVisible("mapTools")
    ? Math.min(430, Math.max(220, Math.floor(viewportWidth * 0.24)))
    : Math.min(92, Math.max(28, Math.floor(viewportWidth * 0.06)));
  const top = uiSectionVisible("topBar") || uiSectionVisible("routeHud") || uiSectionVisible("mapTools")
    ? Math.min(190, Math.max(124, Math.floor(viewportHeight * 0.16)))
    : Math.min(84, Math.max(28, Math.floor(viewportHeight * 0.06)));

  let bottom = Math.min(72, Math.max(32, Math.floor(viewportHeight * 0.05)));
  if (uiSectionVisible("bottomStrip")) {
    const bottomPanelCount = visibleBottomPanelCount();
    bottom = bottomPanelCount === 0
      ? Math.min(126, Math.max(60, Math.floor(viewportHeight * 0.09)))
      : bottomPanelCount === 1
        ? Math.min(210, Math.max(118, Math.floor(viewportHeight * 0.17)))
        : Math.min(270, Math.max(172, Math.floor(viewportHeight * 0.23)));
  } else if (uiSectionVisible("sceneLegend")) {
    bottom = Math.min(126, Math.max(78, Math.floor(viewportHeight * 0.11)));
  }

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

function cellCenter(cell, cellSize) {
  return {
    x: cell.x * cellSize + cellSize / 2,
    y: cell.y * cellSize + cellSize / 2,
  };
}

function traceOverlayPath(ctx, cells, cellSize) {
  ctx.beginPath();
  cells.forEach((cell, index) => {
    const point = cellCenter(cell, cellSize);
    if (index === 0) {
      ctx.moveTo(point.x, point.y);
    } else {
      ctx.lineTo(point.x, point.y);
    }
  });
}

function drawOverlayPath(ctx, cells, cellSize, options = {}) {
  if (!Array.isArray(cells) || cells.length < 2) {
    return;
  }

  ctx.save();
  if (options.shadowColor) {
    ctx.shadowColor = options.shadowColor;
    ctx.shadowBlur = options.shadowBlur || 0;
  }

  if (options.underlayStyle && options.underlayWidth) {
    ctx.setLineDash([]);
    ctx.strokeStyle = options.underlayStyle;
    ctx.lineWidth = options.underlayWidth;
    traceOverlayPath(ctx, cells, cellSize);
    ctx.stroke();
  }

  ctx.setLineDash(options.dash || []);
  ctx.strokeStyle = options.strokeStyle || "rgba(37, 99, 235, 0.82)";
  ctx.lineWidth = options.lineWidth || Math.max(1.5, cellSize * 0.12);
  traceOverlayPath(ctx, cells, cellSize);
  ctx.stroke();

  const endPoint = cellCenter(cells[cells.length - 1], cellSize);
  if (options.endpointFill) {
    ctx.setLineDash([]);
    ctx.fillStyle = options.endpointFill;
    ctx.beginPath();
    ctx.arc(
      endPoint.x,
      endPoint.y,
      options.endpointRadius || Math.max(2.5, cellSize * 0.11),
      0,
      Math.PI * 2,
    );
    ctx.fill();
  }

  ctx.restore();
}

function collectVertexWaitMarkers(waitEdges, focusAgentId) {
  const markers = new Map();
  (waitEdges || []).forEach((edge) => {
    if (edge?.cause === "swap" || !isGridCoordValid(edge?.from)) {
      return;
    }

    const key = `${edge.from.x},${edge.from.y}`;
    const focused =
      focusAgentId !== null &&
      (edge.fromAgentId === focusAgentId || edge.toAgentId === focusAgentId);
    const marker = markers.get(key) || {
      cell: edge.from,
      count: 0,
      focused: false,
    };

    marker.count += 1;
    marker.focused = marker.focused || focused;
    markers.set(key, marker);
  });
  return Array.from(markers.values());
}

function drawVertexWaitMarker(ctx, marker, cellSize, options = {}) {
  const showRing = options.showRing !== false;
  const showCount = options.showCount !== false;
  const cellLeft = marker.cell.x * cellSize;
  const cellTop = marker.cell.y * cellSize;
  const center = cellCenter(marker.cell, cellSize);
  const ringRadius = Math.max(4, cellSize * (marker.focused ? 0.24 : 0.19));
  const countBoost = Math.min(marker.count - 1, 4) * Math.max(0.7, cellSize * 0.018);
  const outerRadius = ringRadius + countBoost;

  if (!showRing && !(showCount && marker.count > 1 && cellSize >= 12)) {
    return;
  }

  ctx.save();
  if (showRing) {
    ctx.fillStyle = marker.focused ? "rgba(220, 63, 106, 0.14)" : "rgba(217, 119, 6, 0.12)";
    ctx.beginPath();
    ctx.arc(center.x, center.y, outerRadius * 1.28, 0, Math.PI * 2);
    ctx.fill();

    ctx.strokeStyle = marker.focused ? "rgba(220, 63, 106, 0.92)" : "rgba(217, 119, 6, 0.84)";
    ctx.lineWidth = marker.focused ? Math.max(2, cellSize * 0.12) : Math.max(1.3, cellSize * 0.08);
    ctx.beginPath();
    ctx.arc(center.x, center.y, outerRadius, 0, Math.PI * 2);
    ctx.stroke();

    ctx.fillStyle = marker.focused ? "rgba(220, 63, 106, 0.98)" : "rgba(217, 119, 6, 0.96)";
    ctx.beginPath();
    ctx.arc(center.x, center.y, Math.max(2.4, cellSize * 0.085), 0, Math.PI * 2);
    ctx.fill();
  }

  if (showCount && marker.count > 1 && cellSize >= 12) {
    const badgeText = marker.count > 9 ? "9+" : String(marker.count);
    const fontSize = Math.max(8, Math.floor(cellSize * 0.23));
    const badgeHeight = Math.max(12, Math.round(cellSize * 0.28));
    const horizontalPadding = Math.max(4, Math.round(cellSize * 0.08));
    const badgeWidth = Math.max(
      badgeHeight,
      horizontalPadding * 2 + badgeText.length * Math.max(5, Math.round(fontSize * 0.58)),
    );
    const badgeX = cellLeft + cellSize - badgeWidth - Math.max(1, Math.round(cellSize * 0.08));
    const badgeY = cellTop + Math.max(1, Math.round(cellSize * 0.08));

    ctx.shadowColor = "rgba(255, 255, 255, 0.68)";
    ctx.shadowBlur = Math.max(4, cellSize * 0.12);
    ctx.fillStyle = marker.focused ? "rgba(255, 244, 247, 0.96)" : "rgba(255, 252, 244, 0.94)";
    ctx.beginPath();
    ctx.roundRect(badgeX, badgeY, badgeWidth, badgeHeight, badgeHeight / 2);
    ctx.fill();

    ctx.shadowBlur = 0;
    ctx.lineWidth = 1;
    ctx.strokeStyle = marker.focused ? "rgba(220, 63, 106, 0.34)" : "rgba(217, 119, 6, 0.28)";
    ctx.stroke();

    ctx.fillStyle = marker.focused ? "rgba(220, 63, 106, 0.94)" : "rgba(176, 98, 16, 0.96)";
    ctx.font = `800 ${fontSize}px "Segoe UI", sans-serif`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(badgeText, badgeX + badgeWidth / 2, badgeY + badgeHeight / 2 + 0.5);
  }

  ctx.restore();
}

function drawPlannerOverlay(ctx, view) {
  const overlay = currentOverlay();
  if (!overlay?.available || !plannerLayersVisible() || !effectiveDebugCaptureEnabled()) {
    return;
  }

  const focusAgentId = displayedAgent()?.id ?? null;
  const cellSize = view.cellSize;
  const cbsAgentIds = new Set((overlay.cbsPaths || []).map((path) => path.agentId));
  ctx.save();
  ctx.lineCap = "round";
  ctx.lineJoin = "round";

  if (state.showLayers.plannedPaths) {
    (overlay.plannedPaths || []).forEach((path) => {
      if (!Array.isArray(path.cells) || path.cells.length < 2) {
        return;
      }

      const focused = focusAgentId !== null && path.agentId === focusAgentId;
      const hasCbsPath = cbsAgentIds.has(path.agentId);
      drawOverlayPath(ctx, path.cells, cellSize, {
        underlayStyle: focused ? "rgba(255, 255, 255, 0.64)" : "rgba(255, 255, 255, 0.24)",
        underlayWidth: focused ? Math.max(4.2, cellSize * 0.33) : Math.max(2.4, cellSize * 0.18),
        strokeStyle: focused
          ? "rgba(37, 99, 235, 0.95)"
          : hasCbsPath
            ? "rgba(37, 99, 235, 0.16)"
            : "rgba(37, 99, 235, 0.34)",
        lineWidth: focused ? Math.max(2.6, cellSize * 0.2) : Math.max(1.5, cellSize * 0.1),
        shadowColor: focused ? "rgba(37, 99, 235, 0.22)" : "rgba(37, 99, 235, 0.08)",
        shadowBlur: focused ? cellSize * 0.78 : cellSize * 0.28,
      });
    });
  }

  if (state.showLayers.cbsPaths) {
    (overlay.cbsPaths || []).forEach((path) => {
      if (!Array.isArray(path.cells) || path.cells.length < 2) {
        return;
      }

      const focused = focusAgentId !== null && path.agentId === focusAgentId;
      drawOverlayPath(ctx, path.cells, cellSize, {
        underlayStyle: focused ? "rgba(255, 255, 255, 0.7)" : "rgba(255, 255, 255, 0.46)",
        underlayWidth: focused ? Math.max(4.8, cellSize * 0.38) : Math.max(3.2, cellSize * 0.25),
        strokeStyle: focused ? "rgba(15, 155, 113, 0.98)" : "rgba(15, 155, 113, 0.84)",
        lineWidth: focused ? Math.max(3.2, cellSize * 0.22) : Math.max(2.2, cellSize * 0.15),
        dash: [cellSize * 0.34, cellSize * 0.2],
        endpointFill: focused ? "rgba(15, 155, 113, 0.98)" : "rgba(15, 155, 113, 0.88)",
        endpointRadius: focused ? Math.max(3.2, cellSize * 0.13) : Math.max(2.4, cellSize * 0.1),
        shadowColor: focused ? "rgba(15, 155, 113, 0.32)" : "rgba(15, 155, 113, 0.14)",
        shadowBlur: focused ? cellSize * 0.92 : cellSize * 0.42,
      });
    });
  }

  if (state.showLayers.conflictCells || state.showLayers.conflictCounts) {
    collectVertexWaitMarkers(overlay.waitEdges, focusAgentId).forEach((marker) => {
      drawVertexWaitMarker(ctx, marker, cellSize, {
        showRing: state.showLayers.conflictCells,
        showCount: state.showLayers.conflictCounts,
      });
    });
  }

  if (state.showLayers.swapWaits) {
    (overlay.waitEdges || []).forEach((edge) => {
      if (edge?.cause !== "swap" || !isGridCoordValid(edge.from) || !isGridCoordValid(edge.to)) {
        return;
      }

      const fromPoint = cellCenter(edge.from, cellSize);
      const toPoint = cellCenter(edge.to, cellSize);
      const focused =
        focusAgentId !== null &&
        (edge.fromAgentId === focusAgentId || edge.toAgentId === focusAgentId);

      ctx.strokeStyle = focused ? "rgba(220, 63, 106, 0.9)" : "rgba(217, 119, 6, 0.42)";
      ctx.lineWidth = focused ? Math.max(2.2, cellSize * 0.16) : Math.max(1.2, cellSize * 0.09);
      ctx.setLineDash(focused ? [] : [cellSize * 0.18, cellSize * 0.16]);
      ctx.beginPath();
      ctx.moveTo(fromPoint.x, fromPoint.y);
      ctx.lineTo(toPoint.x, toPoint.y);
      ctx.stroke();

      ctx.setLineDash([]);
      ctx.fillStyle = focused ? "rgba(220, 63, 106, 0.94)" : "rgba(217, 119, 6, 0.88)";
      ctx.beginPath();
      ctx.arc(
        toPoint.x,
        toPoint.y,
        focused ? Math.max(2.8, cellSize * 0.1) : Math.max(2.2, cellSize * 0.08),
        0,
        Math.PI * 2,
      );
      ctx.fill();
    });
  }

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

      if (state.showLayers.map) {
        ctx.fillStyle = tileColor(tile);
        ctx.fillRect(x * view.cellSize, y * view.cellSize, view.cellSize, view.cellSize);
      }

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

      if (state.showLayers.map) {
        ctx.strokeStyle = tile === "+" || tile === "#" || tile === "X"
          ? "rgba(255, 255, 255, 0.06)"
          : "rgba(91, 111, 151, 0.1)";
        ctx.lineWidth = 1;
        ctx.strokeRect(x * view.cellSize, y * view.cellSize, view.cellSize, view.cellSize);
      }
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

function applySync(sync) {
  if (!sync) {
    return;
  }
  if (sync.kind === "frame") {
    applyFrame(sync.frame);
  } else if (sync.kind === "delta") {
    applyDelta(sync.delta);
  }
}

function handleSync(sync) {
  applySync(sync);
  renderAll();
}

function clearSessionView() {
  state.scene = null;
  state.frame = null;
  updateMetricHistory(null);
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
  syncUiSectionClasses();
  syncRightStackOffset();
  setFleetView(state.fleetView, { animate: false });
  setRightStackView(state.rightStackView, { animate: false });
  setPriorityBoardMode(state.priorityBoardMode);
  renderBottomPanels();
  layoutViewportEmpty();
  elements.viewportEmpty.classList.toggle("hidden", Boolean(state.sessionId));
  renderLaunchSummaries();
  renderTopBar();
  renderPlannerSummary();
  renderRoster();
  renderTelemetry();
  renderPriorityBoard();
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
  state.metricHistory = [];
  state.logs = [];
  state.lastLogSeq = 0;
  state.running = false;
  state.paused = false;
  state.selectedAgentId = null;
  state.hoveredAgentId = null;
  applyFrame(response.frame);
  updateMetricHistory(response.metrics);
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
  state.paused = Boolean(response.advance.paused);
  setSessionState(state.sessionId, response.advance.captureLevel);
  applySync(response.sync);
  updateMetricHistory(response.metrics);
  renderAll();

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

function interactiveShortcutTarget(target) {
  const candidate = target instanceof Element ? target : document.activeElement;
  if (!(candidate instanceof Element)) {
    return null;
  }
  return candidate.closest(kShortcutEditableSelector);
}

function shouldIgnoreShortcut(event) {
  return Boolean(
    event.defaultPrevented ||
      event.repeat ||
      event.altKey ||
      event.ctrlKey ||
      event.metaKey ||
      interactiveShortcutTarget(event.target),
  );
}

function triggerRun() {
  if (!state.sessionId || state.running || state.busy) {
    return;
  }
  void runContinuously();
}

function triggerPause() {
  if (!state.sessionId || state.busy || (!state.running && state.paused)) {
    return;
  }
  void pauseRun();
}

function triggerStep() {
  if (!state.sessionId || state.busy || state.running || !state.paused) {
    return;
  }
  void runBusyAction(stepOnce, "Step failed:");
}

function triggerReset() {
  if (!state.sessionId || state.busy || state.running) {
    return;
  }
  void runBusyAction(resetToSetup, "Reset failed:");
}

function handleGlobalShortcut(event) {
  if (shouldIgnoreShortcut(event) || !state.sessionId) {
    return;
  }

  switch (event.code) {
    case "Space":
      event.preventDefault();
      if (state.running || !state.paused) {
        triggerPause();
      } else {
        triggerRun();
      }
      break;
    case "KeyS":
      if (!state.paused || state.running) {
        return;
      }
      event.preventDefault();
      triggerStep();
      break;
    case "KeyR":
      event.preventDefault();
      triggerReset();
      break;
    default:
      break;
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
  triggerRun();
});

elements.pauseButton.addEventListener("click", () => {
  triggerPause();
});

elements.stepButton.addEventListener("click", () => {
  triggerStep();
});

elements.resetSessionButton.addEventListener("click", () => {
  triggerReset();
});

elements.debugButton.addEventListener("click", async () => {
  await runBusyAction(loadDebugSnapshot, "Debug snapshot failed:");
});

elements.debugToggleButton.addEventListener("click", () => {
  setDebugExpanded(!state.isDebugExpanded);
});

elements.tailVisibilityButton.addEventListener("click", () => {
  void setBottomPanelVisible("tail", !state.bottomPanels.tail);
});

elements.diagnosticsVisibilityButton.addEventListener("click", () => {
  void setBottomPanelVisible("diagnostics", !state.bottomPanels.diagnostics);
});

elements.fleetRosterTab.addEventListener("click", () => {
  setFleetView("roster");
});

elements.fleetPriorityTab.addEventListener("click", () => {
  setFleetView("priority");
});

elements.priorityDetailedTab.addEventListener("click", () => {
  setPriorityBoardMode("detailed");
  renderPriorityBoard();
});

elements.priorityMinimalTab.addEventListener("click", () => {
  setPriorityBoardMode("minimal");
  renderPriorityBoard();
});

elements.rightTelemetryTab.addEventListener("click", () => {
  setRightStackView("telemetry");
});

elements.rightPlannerTab.addEventListener("click", () => {
  setRightStackView("planner");
});

elements.rightDebugTab.addEventListener("click", () => {
  setRightStackView("debug");
});

elements.toggleUiButton.addEventListener("click", () => {
  setUiVisible(!anyUiSectionsVisible());
  renderAll();
});

elements.uiVisibilityButton.addEventListener("click", (event) => {
  event.stopPropagation();
  setUiVisibilityPanelOpen(!state.isUiVisibilityPanelOpen);
});

elements.uiVisibilityPanel.addEventListener("click", (event) => {
  event.stopPropagation();
});

elements.uiAllOnButton.addEventListener("click", () => {
  setAllUiSections(true);
});

elements.uiAllOffButton.addEventListener("click", () => {
  setAllUiSections(false);
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

kLayerButtonSpecs.forEach((spec) => {
  const button = elements[spec.element];
  if (!button) {
    return;
  }

  button.addEventListener("click", () => {
    if (button.disabled) {
      appendStatus("Planner layers are available only when the capture mode is set to debug.");
      return;
    }

    state.showLayers[spec.key] = !state.showLayers[spec.key];
    renderAll();
  });
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

document.addEventListener("keydown", (event) => {
  handleGlobalShortcut(event);
});

document.addEventListener("click", () => {
  if (state.isUiVisibilityPanelOpen) {
    setUiVisibilityPanelOpen(false);
  }
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

window.motionRuntimeReady?.then(() => {
  syncUiSectionClasses();
  renderUiVisibilityPanel();
  renderBottomPanels();
  renderPriorityBoard();
});

bootstrap().catch((error) => {
  appendStatus(`Bootstrap failed: ${error.message}`);
  elements.debugOutput.textContent = error.stack || error.message;
  setDebugExpanded(true);
});
