/* Presentation-only shell behavior. Simulation state and IPC remain in renderer.js. */
class WorkspaceUI {
  constructor({ state, elements, actions }) {
    this.state = state;
    this.elements = elements;
    this.actions = actions;
    this.returnFocus = null;
    this.toastTimer = null;
    this.nodes = Object.fromEntries([
      "workspace-header", "workspace-main", "workspace-footer", "simulation-nav",
      "analysis-nav", "workspace-map-button", "workspace-map-label",
      "workspace-map-description", "simulation-status", "layers-toggle-button",
      "events-toggle-button", "playback-rate", "setup-feedback", "workspace-toast",
    ].map((id) => [id, document.getElementById(id)]));
    this.bind();
  }

  sync() {
    const { state, elements, nodes } = this;
    const analysis = state.viewportMode === "metrics";
    for (const [id, active] of [["simulation-nav", !analysis], ["analysis-nav", analysis]]) {
      nodes[id].classList.toggle("active", active);
      if (active) nodes[id].setAttribute("aria-current", "page");
      else nodes[id].removeAttribute("aria-current");
    }
    nodes["workspace-main"].classList.toggle("without-fleet", !state.uiSections.leftStack);
    nodes["workspace-main"].classList.toggle("without-inspector", !state.uiSections.rightStack);

    const mapId = Number(state.frame?.hud?.mapId ?? elements.mapId.value);
    const map = state.mapOptions.find((item) => item.id === mapId);
    nodes["workspace-map-label"].textContent = `Map ${mapId || "—"}`;
    nodes["workspace-map-description"].textContent = map?.label || "Choose a map to begin";
    nodes["workspace-map-description"].title = map?.description || "";
    nodes["simulation-status"].textContent = !state.sessionId
      ? "No session" : state.running ? "Running" : state.paused ? "Paused" : "Ready";
    elements.runButton.classList.toggle("hidden", state.running);
    elements.pauseButton.classList.toggle("hidden", !state.running);
    elements.pauseButton.classList.add("primary-control");
    nodes["playback-rate"].disabled = state.busy;
    elements.openMetricsButton.disabled = !state.metrics;
    nodes["events-toggle-button"].disabled = analysis || state.viewportMode === "map";
    nodes["events-toggle-button"].setAttribute("aria-expanded", String(
      state.uiSections.bottomStrip && (state.bottomPanels.tail || state.bottomPanels.diagnostics),
    ));
    nodes["layers-toggle-button"].setAttribute("aria-expanded", String(state.isLayersOpen));
    elements.mapTools.classList.toggle("hidden", !state.isLayersOpen);
    elements.metricsOverviewPill.textContent = state.sessionId
      ? `Map ${mapId} · Session ${state.sessionId}` : "Load a session to view results";
  }

  setLayersOpen(open) {
    this.state.isLayersOpen = Boolean(open);
    this.elements.mapTools.classList.toggle("hidden", !open);
    this.nodes["layers-toggle-button"].setAttribute("aria-expanded", String(Boolean(open)));
  }

  setDialog(name, open) {
    const { state, elements, nodes } = this;
    const wasOpen = state.isSetupOpen || state.isMetricsOpen;
    if (open && !wasOpen) this.returnFocus = document.activeElement;
    if (name === "setup") {
      state.isSetupOpen = Boolean(open);
      if (open) state.isMetricsOpen = false;
    } else {
      state.isMetricsOpen = Boolean(open);
      if (open) state.isSetupOpen = false;
    }
    elements.setupBackdrop.classList.toggle("hidden", !state.isSetupOpen);
    elements.setupDrawer.classList.toggle("open", state.isSetupOpen);
    elements.setupDrawer.inert = !state.isSetupOpen;
    elements.setupDrawer.setAttribute("aria-hidden", String(!state.isSetupOpen));
    elements.metricsBackdrop.classList.toggle("hidden", !state.isMetricsOpen);
    elements.metricsModal.classList.toggle("hidden", !state.isMetricsOpen);
    elements.metricsModal.inert = !state.isMetricsOpen;
    const modalOpen = state.isSetupOpen || state.isMetricsOpen;
    for (const id of ["workspace-header", "workspace-main", "workspace-footer"]) {
      nodes[id].inert = modalOpen;
    }
    elements.uiVisibilityPanel.inert = modalOpen;
    if (open) {
      this.setLayersOpen(false);
      this.actions.closeSettings();
      const dialog = name === "setup" ? elements.setupDrawer : elements.metricsModal;
      if (!wasOpen) {
        const scrollArea = dialog.querySelector(".drawer-scroll, .metrics-modal-body");
        if (scrollArea) scrollArea.scrollTop = 0;
      }
      dialog.focus({ preventScroll: true });
      requestAnimationFrame(() => {
        const stillOpen = name === "setup" ? state.isSetupOpen : state.isMetricsOpen;
        if (stillOpen && !dialog.contains(document.activeElement)) {
          dialog.focus({ preventScroll: true });
        }
      });
    } else if (!modalOpen && wasOpen) {
      const target = this.returnFocus;
      if (target?.isConnected && target.getClientRects().length && !target.closest("[inert]")) {
        target.focus({ preventScroll: true });
      } else {
        nodes["simulation-nav"].focus({ preventScroll: true });
      }
      this.returnFocus = null;
    }
  }

  feedback(message, tone = "success") {
    const node = this.state.isSetupOpen ? this.nodes["setup-feedback"] : this.nodes["workspace-toast"];
    node.textContent = message;
    node.dataset.tone = tone;
    node.classList.remove("hidden");
    if (node === this.nodes["workspace-toast"]) {
      clearTimeout(this.toastTimer);
      this.toastTimer = setTimeout(() => node.classList.add("hidden"), 5000);
    }
  }

  bind() {
    const { state, elements, nodes, actions } = this;
    nodes["simulation-nav"].addEventListener("click", () => {
      actions.setViewportMode("live");
      actions.render();
    });
    nodes["analysis-nav"].addEventListener("click", () => {
      actions.setViewportMode("metrics");
      actions.render();
    });
    nodes["workspace-map-button"].addEventListener("click", () => {
      actions.openSetup();
      requestAnimationFrame(() => elements.mapId.focus({ preventScroll: true }));
    });
    nodes["layers-toggle-button"].addEventListener("click", (event) => {
      event.stopPropagation();
      this.setLayersOpen(!state.isLayersOpen);
    });
    elements.mapTools.addEventListener("click", (event) => event.stopPropagation());
    document.addEventListener("click", () => this.setLayersOpen(false));
    nodes["events-toggle-button"].addEventListener("click", () => {
      const open = !(state.uiSections.bottomStrip && (state.bottomPanels.tail || state.bottomPanels.diagnostics));
      state.uiSections.bottomStrip = true;
      actions.setEventsOpen(open);
      actions.render();
    });
    nodes["playback-rate"].addEventListener("change", (event) => {
      const rate = Number(event.target.value);
      if ([0.5, 1, 2, 4].includes(rate)) state.playbackRate = rate;
    });
    elements.launchForm.addEventListener("input", () => nodes["setup-feedback"].classList.add("hidden"));
    elements.launchForm.addEventListener("change", () => nodes["setup-feedback"].classList.add("hidden"));

    document.addEventListener("keydown", (event) => {
      if (event.key === "Escape") {
        if (state.isMetricsOpen) this.setDialog("metrics", false);
        else if (state.isSetupOpen) this.setDialog("setup", false);
        else if (state.isLayersOpen) {
          this.setLayersOpen(false);
          nodes["layers-toggle-button"].focus();
        } else if (state.isUiVisibilityPanelOpen) {
          actions.closeSettings();
          elements.uiVisibilityButton.focus();
        } else return;
        event.preventDefault();
        return;
      }
      if (event.key !== "Tab" || (!state.isSetupOpen && !state.isMetricsOpen)) return;
      const dialog = state.isSetupOpen ? elements.setupDrawer : elements.metricsModal;
      const focusable = [...dialog.querySelectorAll("button, input, select, textarea, summary, [tabindex='0']")]
        .filter((node) => !node.disabled && node.getClientRects().length && !node.closest("[inert]"));
      if (!focusable.length) { event.preventDefault(); return; }
      const first = focusable[0];
      const last = focusable[focusable.length - 1];
      if (event.shiftKey && (document.activeElement === first || document.activeElement === dialog)) {
        event.preventDefault(); last.focus();
      } else if (!event.shiftKey && (document.activeElement === last || document.activeElement === dialog)) {
        event.preventDefault(); first.focus();
      }
    });

    for (const list of document.querySelectorAll("[role='tablist']")) {
      list.addEventListener("keydown", (event) => {
        if (!["ArrowLeft", "ArrowRight", "Home", "End"].includes(event.key)) return;
        const tabs = [...list.querySelectorAll("[role='tab']")];
        const index = tabs.indexOf(document.activeElement);
        if (index < 0) return;
        event.preventDefault();
        const next = event.key === "Home" ? 0 : event.key === "End" ? tabs.length - 1
          : (index + (event.key === "ArrowRight" ? 1 : -1) + tabs.length) % tabs.length;
        tabs[next].focus();
        tabs[next].click();
      });
    }
  }
}
