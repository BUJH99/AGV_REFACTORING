const { app, BrowserWindow, ipcMain } = require("electron");
const { spawn } = require("child_process");
const fs = require("fs");
const path = require("path");
const readline = require("readline");

const kProtocolVersion = 2;
const kDefaultCaptureLevel = "frame";
const kDefaultFrameOptions = Object.freeze({
  logsTail: true,
  maxLogEntries: 120,
  plannerOverlay: false,
});
const kReadmeDemoCaptureIntervalMs = 150;
const kReadmeDemoWindowBounds = Object.freeze({
  width: 1440,
  height: 900,
});

const repoRoot = path.resolve(__dirname, "..");
let isAppQuitInFlight = false;
const readmeDemoState = createReadmeDemoState();

if (readmeDemoState.enabled) {
  app.commandLine.appendSwitch("disable-background-timer-throttling");
  app.commandLine.appendSwitch("disable-renderer-backgrounding");
}

function isIgnorablePipeError(error) {
  const code = String(error?.code || "");
  return (
    code === "EPIPE" ||
    code === "ECONNRESET" ||
    code === "ERR_STREAM_DESTROYED" ||
    code === "ERR_STREAM_WRITE_AFTER_END"
  );
}

function resolveBackendPath() {
  const requested = process.env.AGV_RENDER_IPC_PATH;
  const candidates = requested
    ? [requested]
    : [
        path.join(repoRoot, "build-codex", "agv_render_ipc.exe"),
        path.join(repoRoot, "build-run", "agv_render_ipc.exe"),
        path.join(repoRoot, "build", "agv_render_ipc.exe"),
      ];

  for (const candidate of candidates) {
    if (candidate && fs.existsSync(candidate)) {
      return path.resolve(candidate);
    }
  }

  throw new Error(
    [
      "Unable to find agv_render_ipc.exe.",
      "Build the backend first with CMake or set AGV_RENDER_IPC_PATH.",
      `Checked: ${candidates.join(", ")}`,
    ].join(" "),
  );
}

function parsePositiveInteger(value, fallbackValue) {
  const parsed = Number.parseInt(String(value ?? ""), 10);
  if (!Number.isFinite(parsed) || parsed <= 0) {
    return fallbackValue;
  }
  return parsed;
}

function createReadmeDemoState() {
  const enabled = process.env.AGV_README_DEMO === "1";
  return {
    enabled,
    captureIntervalMs: parsePositiveInteger(
      process.env.AGV_README_DEMO_CAPTURE_INTERVAL_MS,
      kReadmeDemoCaptureIntervalMs,
    ),
    captureDir: path.resolve(
      process.env.AGV_README_DEMO_CAPTURE_DIR ||
        path.join(repoRoot, "electron", "output", "readme-demo"),
    ),
    manifestPath: path.resolve(
      process.env.AGV_README_DEMO_MANIFEST_PATH ||
        path.join(repoRoot, "electron", "output", "readme-demo", "manifest.json"),
    ),
    windowWidth: parsePositiveInteger(
      process.env.AGV_README_DEMO_WINDOW_WIDTH,
      kReadmeDemoWindowBounds.width,
    ),
    windowHeight: parsePositiveInteger(
      process.env.AGV_README_DEMO_WINDOW_HEIGHT,
      kReadmeDemoWindowBounds.height,
    ),
    frameCount: 0,
    frames: [],
    timer: null,
    capturePromise: Promise.resolve(),
    stopping: false,
    completed: false,
    failed: false,
    result: null,
  };
}

async function prepareReadmeDemoArtifacts() {
  await fs.promises.rm(readmeDemoState.captureDir, { recursive: true, force: true });
  await fs.promises.mkdir(readmeDemoState.captureDir, { recursive: true });
  await fs.promises.mkdir(path.dirname(readmeDemoState.manifestPath), { recursive: true });
  readmeDemoState.frameCount = 0;
  readmeDemoState.frames = [];
  readmeDemoState.timer = null;
  readmeDemoState.capturePromise = Promise.resolve();
  readmeDemoState.stopping = false;
  readmeDemoState.completed = false;
  readmeDemoState.failed = false;
  readmeDemoState.result = null;
}

async function captureReadmeDemoFrame(window, delayMs) {
  if (!readmeDemoState.enabled) {
    return;
  }
  if (window.isDestroyed() || window.webContents.isDestroyed()) {
    return;
  }

  const image = await window.webContents.capturePage();
  const size = image.getSize();
  const frameIndex = String(++readmeDemoState.frameCount).padStart(4, "0");
  const fileName = `frame_${frameIndex}.bgra`;
  const framePath = path.join(readmeDemoState.captureDir, fileName);
  await fs.promises.writeFile(framePath, image.toBitmap());
  readmeDemoState.frames.push({
    file: fileName,
    width: size.width,
    height: size.height,
    delayMs: Math.max(20, Number(delayMs || readmeDemoState.captureIntervalMs)),
  });
}

function scheduleReadmeDemoCapture(window) {
  if (!readmeDemoState.enabled || readmeDemoState.stopping) {
    return;
  }

  readmeDemoState.timer = setTimeout(() => {
    readmeDemoState.capturePromise = readmeDemoState.capturePromise
      .then(() => captureReadmeDemoFrame(window, readmeDemoState.captureIntervalMs))
      .catch((error) => {
        readmeDemoState.failed = true;
        readmeDemoState.result = { success: false, error: error.message };
        console.error("README demo capture failed:", error);
      })
      .finally(() => {
        scheduleReadmeDemoCapture(window);
      });
  }, readmeDemoState.captureIntervalMs);
}

async function startReadmeDemoCapture(window) {
  if (!readmeDemoState.enabled) {
    return;
  }

  await prepareReadmeDemoArtifacts();
  readmeDemoState.capturePromise = captureReadmeDemoFrame(
    window,
    readmeDemoState.captureIntervalMs,
  );
  await readmeDemoState.capturePromise;
  scheduleReadmeDemoCapture(window);
}

async function finalizeReadmeDemoCapture(window, payload = {}) {
  if (!readmeDemoState.enabled || readmeDemoState.completed) {
    return;
  }

  readmeDemoState.stopping = true;
  readmeDemoState.completed = true;
  readmeDemoState.result = payload;
  if (readmeDemoState.timer) {
    clearTimeout(readmeDemoState.timer);
    readmeDemoState.timer = null;
  }

  await readmeDemoState.capturePromise;
  if (!window.isDestroyed() && !window.webContents.isDestroyed()) {
    await captureReadmeDemoFrame(window, readmeDemoState.captureIntervalMs);
  }

  const manifest = {
    success: payload.success !== false && !readmeDemoState.failed,
    captureIntervalMs: readmeDemoState.captureIntervalMs,
    captureDir: readmeDemoState.captureDir,
    frames: readmeDemoState.frames,
    window: {
      width: readmeDemoState.windowWidth,
      height: readmeDemoState.windowHeight,
    },
    result: payload,
  };
  await fs.promises.writeFile(
    readmeDemoState.manifestPath,
    `${JSON.stringify(manifest, null, 2)}\n`,
    "utf8",
  );
}

class AgvBackendClient {
  constructor() {
    this.child = null;
    this.stdoutReader = null;
    this.stderrReader = null;
    this.pending = new Map();
    this.nextRequestId = 1;
    this.backendPath = null;
    this.eventSink = null;
    this.isShuttingDown = false;
  }

  setEventSink(callback) {
    this.eventSink = callback;
  }

  emit(event) {
    if (typeof this.eventSink === "function") {
      this.eventSink(event);
    }
  }

  childIsAlive() {
    return Boolean(
      this.child &&
        !this.child.killed &&
        this.child.exitCode === null &&
        this.child.signalCode === null,
    );
  }

  stdinIsWritable() {
    return Boolean(
      this.child?.stdin &&
        !this.child.stdin.destroyed &&
        !this.child.stdin.writableEnded &&
        !this.child.stdin.errored,
    );
  }

  clearChildReferences() {
    this.stdoutReader?.close();
    this.stderrReader?.close();
    this.child = null;
    this.stdoutReader = null;
    this.stderrReader = null;
  }

  async ensureStarted() {
    if (this.childIsAlive() && this.stdinIsWritable()) {
      return;
    }

    this.isShuttingDown = false;
    this.backendPath = resolveBackendPath();
    this.child = spawn(this.backendPath, [], {
      cwd: path.dirname(this.backendPath),
      stdio: ["pipe", "pipe", "pipe"],
      windowsHide: true,
    });

    this.stdoutReader = readline.createInterface({ input: this.child.stdout });
    this.stderrReader = readline.createInterface({ input: this.child.stderr });

    this.stdoutReader.on("line", (line) => this.handleStdout(line));
    this.stderrReader.on("line", (line) => {
      this.emit({ type: "backend-stderr", line });
    });
    this.child.stdin.on("error", (error) => {
      if (this.isShuttingDown && isIgnorablePipeError(error)) {
        return;
      }
      this.rejectAllPending(error);
      this.emit({ type: "backend-error", message: error.message });
    });

    this.child.on("error", (error) => {
      this.rejectAllPending(error);
      this.emit({ type: "backend-error", message: error.message });
    });

    this.child.on("exit", (code, signal) => {
      const error = new Error(
        `Backend exited with code ${code ?? "null"} and signal ${signal ?? "null"}.`,
      );
      this.rejectAllPending(error);
      this.clearChildReferences();
      this.emit({
        type: "backend-exit",
        code,
        signal,
      });
    });

    this.emit({
      type: "backend-ready",
      backendPath: this.backendPath,
    });
  }

  rejectAllPending(error) {
    for (const pending of this.pending.values()) {
      pending.reject(error);
    }
    this.pending.clear();
  }

  handleStdout(line) {
    const trimmed = String(line || "").trim();
    if (!trimmed) {
      return;
    }

    let message = null;
    try {
      message = JSON.parse(trimmed);
    } catch (error) {
      this.emit({
        type: "backend-parse-error",
        line: trimmed,
        message: error.message,
      });
      return;
    }

    if (message.type === "event") {
      this.emit(message);
      return;
    }

    if (message.type !== "response") {
      this.emit({
        type: "backend-unknown-message",
        payload: message,
      });
      return;
    }

    const key = String(message.requestId ?? "");
    const pending = this.pending.get(key);
    if (!pending) {
      this.emit({
        type: "backend-orphan-response",
        payload: message,
      });
      return;
    }

    this.pending.delete(key);
    pending.resolve(message);
  }

  async request(command, payload = {}) {
    await this.ensureStarted();
    if (!this.childIsAlive() || !this.stdinIsWritable()) {
      throw new Error("Backend connection is no longer writable.");
    }

    const requestId = `${command}-${this.nextRequestId++}`;
    const request = {
      protocolVersion: kProtocolVersion,
      requestId,
      command,
      ...payload,
    };

    return await new Promise((resolve, reject) => {
      this.pending.set(requestId, { resolve, reject });
      try {
        this.child.stdin.write(`${JSON.stringify(request)}\n`, (error) => {
          if (!error) {
            return;
          }
          this.pending.delete(requestId);
          reject(error);
        });
      } catch (error) {
        this.pending.delete(requestId);
        reject(error);
      }
    });
  }

  async shutdown() {
    if (!this.childIsAlive()) {
      return;
    }

    this.isShuttingDown = true;
    try {
      await this.request("shutdown");
    } catch (_error) {
      // Ignore shutdown races during app exit.
    }

    if (this.stdinIsWritable()) {
      this.child.stdin.end();
    }
  }
}

const backend = new AgvBackendClient();
const sessionState = {
  sessionId: null,
  lastFrameId: 0,
  lastLogSeq: 0,
  captureLevel: kDefaultCaptureLevel,
  paused: false,
};

function requireSession() {
  if (!sessionState.sessionId) {
    throw new Error("No active AGV session. Start a session first.");
  }
}

function resetSessionState() {
  sessionState.sessionId = null;
  sessionState.lastFrameId = 0;
  sessionState.lastLogSeq = 0;
  sessionState.captureLevel = kDefaultCaptureLevel;
  sessionState.paused = false;
}

function frameOptions() {
  return {
    ...kDefaultFrameOptions,
    plannerOverlay: sessionState.captureLevel === "debug",
  };
}

async function getFrameSnapshot() {
  const response = await backend.request("getFrame", {
    sessionId: sessionState.sessionId,
    options: frameOptions(),
  });
  sessionState.lastFrameId = response.frame.frameId;
  sessionState.lastLogSeq = response.frame.lastLogSeq;
  return response.frame;
}

async function syncAfterAdvance() {
  if (
    sessionState.captureLevel !== "frame" &&
    sessionState.captureLevel !== "debug"
  ) {
    return null;
  }

  if (sessionState.lastFrameId <= 0) {
    return {
      kind: "frame",
      frame: await getFrameSnapshot(),
    };
  }

  const deltaResponse = await backend.request("getDelta", {
    sessionId: sessionState.sessionId,
    sinceFrameId: sessionState.lastFrameId,
    options: frameOptions(),
  });
  const { delta } = deltaResponse;
  if (delta.requiresFullResync) {
    return {
      kind: "frame",
      frame: await getFrameSnapshot(),
    };
  }

  sessionState.lastFrameId = delta.toFrameId;
  sessionState.lastLogSeq = delta.lastLogSeq;
  return {
    kind: "delta",
    delta,
  };
}

async function getMetricsSnapshot() {
  const response = await backend.request("getMetrics", {
    sessionId: sessionState.sessionId,
  });
  return response.metrics;
}

function createWindow() {
  const windowOptions = readmeDemoState.enabled
    ? {
        width: readmeDemoState.windowWidth,
        height: readmeDemoState.windowHeight,
        resizable: false,
        minimizable: false,
        maximizable: false,
        fullscreenable: false,
        autoHideMenuBar: true,
      }
    : {
        width: 1520,
        height: 960,
        minWidth: 1180,
        minHeight: 760,
      };
  const window = new BrowserWindow({
    ...windowOptions,
    backgroundColor: "#e6ddcb",
    webPreferences: {
      preload: path.join(__dirname, "preload.js"),
      contextIsolation: true,
      nodeIntegration: false,
    },
  });

  backend.setEventSink((event) => {
    if (window.isDestroyed() || window.webContents.isDestroyed()) {
      return;
    }
    window.webContents.send("agv:event", event);
  });

  if (readmeDemoState.enabled) {
    window.webContents.once("did-finish-load", () => {
      void startReadmeDemoCapture(window);
    });
  }

  window.loadFile(path.join(__dirname, "index.html"), {
    query: readmeDemoState.enabled ? { readmeDemo: "1" } : {},
  });

  return window;
}

ipcMain.handle("agv:bootstrap", async () => {
  const response = await backend.request("getCapabilities");
  return {
    backendPath: backend.backendPath,
    capabilities: response.capabilities,
  };
});

ipcMain.handle("agv:validate-config", async (_event, launchConfig) => {
  const response = await backend.request("validateConfig", { launchConfig });
  return response;
});

ipcMain.handle("agv:start-session", async (_event, payload = {}) => {
  const captureLevel = payload.captureLevel || kDefaultCaptureLevel;
  const response = await backend.request("startSession", {
    launchConfig: payload.launchConfig,
    captureLevel,
  });

  sessionState.sessionId = response.session.sessionId;
  sessionState.captureLevel = response.captureLevel || captureLevel;
  sessionState.lastFrameId = response.session.frameId || 0;
  sessionState.lastLogSeq = 0;
  sessionState.paused = false;

  const sceneResponse = await backend.request("getStaticScene", {
    sessionId: sessionState.sessionId,
  });
  const frame = await getFrameSnapshot();
  const metrics = await getMetricsSnapshot();

  return {
    session: response.session,
    captureLevel: sessionState.captureLevel,
    warnings: response.warnings || [],
    scene: sceneResponse.scene,
    frame,
    metrics,
  };
});

ipcMain.handle("agv:advance", async (_event, payload = {}) => {
  requireSession();

  const requestedCaptureLevel = payload.captureLevel || sessionState.captureLevel;
  const response = await backend.request("advance", {
    sessionId: sessionState.sessionId,
    steps: payload.steps || 0,
    maxDurationMs: payload.maxDurationMs || 0,
    captureLevel: requestedCaptureLevel,
  });

  sessionState.captureLevel = response.captureLevel || requestedCaptureLevel;
  sessionState.paused = Boolean(response.paused);

  const sync = await syncAfterAdvance();
  const metrics = await getMetricsSnapshot();

  return {
    advance: response,
    sync,
    metrics,
  };
});

ipcMain.handle("agv:set-paused", async (_event, paused) => {
  requireSession();
  const response = await backend.request("setPaused", {
    sessionId: sessionState.sessionId,
    paused: Boolean(paused),
  });
  sessionState.paused = Boolean(response.paused);
  return response;
});

ipcMain.handle("agv:reset-session", async () => {
  resetSessionState();
  return { ok: true };
});

ipcMain.handle("agv:get-debug-snapshot", async () => {
  requireSession();
  const response = await backend.request("getDebugSnapshot", {
    sessionId: sessionState.sessionId,
  });
  return response.snapshot;
});

ipcMain.handle("agv:shutdown-session", async () => {
  resetSessionState();
  await backend.shutdown();
  return { ok: true };
});

ipcMain.handle("agv:complete-readme-demo", async (_event, payload = {}) => {
  if (!readmeDemoState.enabled) {
    return { ok: false, reason: "disabled" };
  }

  const window = BrowserWindow.getAllWindows()[0];
  if (!window || window.isDestroyed()) {
    return { ok: false, reason: "window-unavailable" };
  }

  await finalizeReadmeDemoCapture(window, payload);
  setTimeout(() => {
    if (payload.success === false) {
      app.exitCode = 1;
    }
    app.quit();
  }, 200);
  return { ok: true, frames: readmeDemoState.frames.length };
});

app.whenReady().then(() => {
  createWindow();

  app.on("activate", () => {
    if (BrowserWindow.getAllWindows().length === 0) {
      createWindow();
    }
  });
});

app.on("before-quit", async (event) => {
  if (isAppQuitInFlight || !backend.childIsAlive()) {
    return;
  }

  event.preventDefault();
  isAppQuitInFlight = true;
  try {
    await backend.shutdown();
  } finally {
    app.exit(0);
  }
});

app.on("window-all-closed", () => {
  if (process.platform !== "darwin") {
    app.quit();
  }
});
