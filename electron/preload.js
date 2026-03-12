const { contextBridge, ipcRenderer } = require("electron");

contextBridge.exposeInMainWorld("agvShell", {
  bootstrap: () => ipcRenderer.invoke("agv:bootstrap"),
  validateConfig: (launchConfig) =>
    ipcRenderer.invoke("agv:validate-config", launchConfig),
  startSession: (payload) => ipcRenderer.invoke("agv:start-session", payload),
  advance: (payload) => ipcRenderer.invoke("agv:advance", payload),
  setPaused: (paused) => ipcRenderer.invoke("agv:set-paused", paused),
  resetSession: () => ipcRenderer.invoke("agv:reset-session"),
  getDebugSnapshot: () => ipcRenderer.invoke("agv:get-debug-snapshot"),
  shutdown: () => ipcRenderer.invoke("agv:shutdown-session"),
  onEvent: (callback) => {
    const handler = (_event, payload) => callback(payload);
    ipcRenderer.on("agv:event", handler);
    return () => ipcRenderer.removeListener("agv:event", handler);
  },
});
