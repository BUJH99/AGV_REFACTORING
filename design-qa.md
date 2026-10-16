# Precision Workspace — implementation QA

Date: 2026-10-16 (Asia/Seoul)

final result: passed

## Scope and visual references

The implementation follows the selected first mockup and the additional setup, analysis and inspection concepts created for this task. It updates the existing Electron application and keeps the C++ engine, IPC payloads and simulation contracts intact.

| Surface | Visual reference | Rendered evidence |
| --- | --- | --- |
| Simulation | [Selected mockup](agv-ui-mockup-precision-workspace.png) | [Actual application](docs/assets/design-qa/simulation-final.png) |
| Session setup | [Setup concept](agv-ui-setup-concept.png) | [Actual setup](docs/assets/design-qa/setup-final.png) |
| Analysis | [Analysis concept](agv-ui-analysis-concept.png) | [Actual analysis](docs/assets/design-qa/analysis-final.png) |
| Priority, planner, layers and events | [Inspection concept](agv-ui-inspection-concept.png) | [Actual inspection](docs/assets/design-qa/inspection-final.png) |

The reference images are 1586 × 992 pixels. Each was scaled proportionally into a 1440 × 900 comparison frame, with white padding where required. The application screenshots are 1440 × 900 CSS pixels, captured at CSS scale on a display with devicePixelRatio 1.5. Native window chrome is excluded.

The primary comparison uses Map 3, the Default algorithm, a Custom scenario, frame capture, AGV A selected, and a paused session at frame 58. Mockup metric values and agent positions are illustrative; the implementation displays live engine values. The setup comparison has advanced settings expanded. Inspection evidence shows the priority and planner panels, layers popover and events dock.

## Verification environment

- Actual Electron 30 application launched with its local `agv_render_ipc.exe` through Playwright's Electron integration. An in-app web browser cannot exercise this native window's preload/IPC connection, so native Electron automation was used.
- As-launched content size: 1506 × 897.
- Primary comparison viewport: 1440 × 900.
- Minimum supported native window: 1180 × 760 outer size, measured as 1166 × 698 content. Header/footer actions remain within the viewport, the full map fits, and the setup form scrolls above a fixed action footer.
- Screenshots were captured after settling or finishing transitions. Reduced-motion behavior was exercised separately.
- Console: no renderer errors, failed resources or application warnings in the final run.

## Visual comparison history

| Iteration | Finding | Fix and post-fix evidence |
| --- | --- | --- |
| Initial render | P2: fleet and inspector text was too small relative to the selected design. | Increased operational text to 13–14 px, strengthened selected-agent typography and raised muted text contrast. Reviewed simulation and focused inspector comparisons. |
| Initial setup | P2: repeated map descriptions and large phase rows pushed advanced settings and the launch summary below the first viewport. | Moved map descriptions into a disclosure, made phase controls compact, removed duplicate phase totals and reduced the summary to two useful rows. The normal two-phase setup, expanded advanced settings and summary now fit above the footer at 1440 × 900. |
| Initial analysis | P2: the operational rows were pushed below the first viewport and charts lacked labeled axes. | Added real step/value axes, matched chart order, tightened section spacing and used open metric rows. The four operational rows are visible at the primary viewport. |
| Keyboard pass | P2: a visibility transition could prevent initial focus from entering Setup. | Made opening visibility immediate, retained the closing transition, and added guarded focus handling. Initial focus, Tab/Shift+Tab trapping, Escape and return focus pass. |
| Runtime pass | P2: the previous unbundled ESM import could not resolve `framer-motion/dom`. | Loaded Motion's existing browser bundle and retained the renderer animation API. Final console is clean. |
| State restoration | P2: a panel hidden during a selection change could reopen with stale data. | Refresh visible regions when panel visibility changes, and preserve Settings toggle focus. The inspector reopens with the current selected AGV. |

No actionable P0, P1 or P2 findings remain in the reviewed desktop scope.

## Fidelity surfaces reviewed

- **Typography:** consistent Segoe UI/system font, deliberate control sizes, stronger primary values and plain operational labels such as Moving, At goal and Turning. Captions, numeric columns and expanded details were inspected separately.
- **Layout and spacing:** fixed header, fleet/map/inspector regions, persistent execution bar, a right setup drawer, and an optional event dock. The map is never covered by permanent side panels. The final setup form and minimum-size footer were inspected visually as well as measured.
- **Colors and surfaces:** white/pale neutral backgrounds, graphite text, cobalt primary/selected states, thin separators and modest corner radii. Decorative gradients, glass effects and canvas glow were removed. Muted text has approximately 4.99:1 contrast on white and 4.65:1 on the canvas background.
- **Map and asset quality:** the existing canvas renders actual map cells and agents. The inspector preview uses actual positions/goals and supplied planner coordinates; it does not synthesize missing routes. No mockup image is used as the working application UI.
- **Copy and content:** main navigation and execution controls follow the concepts. Technical data remains available in explicit details, planner/debug panels and the full metric explorer. The full explorer preserves 69 scalar values across five existing metric sections, plus trends and fairness data.
- **Interaction and accessibility:** visible focus, modal background inertness, focus trapping and return, tab arrow navigation, pressed states on toggles and selected rows, reduced motion, and non-repeating execution announcements were checked.

Full-view source/implementation pairs were inspected for every designed state:

- [Simulation comparison](docs/assets/design-qa/compare-simulation.png)
- [Setup comparison](docs/assets/design-qa/compare-setup.png)
- [Analysis comparison](docs/assets/design-qa/compare-analysis.png)
- [Inspection comparison](docs/assets/design-qa/compare-inspection.png)

The inspector and setup form were also reviewed at readable full-size/focused resolution. [Minimum-size evidence](docs/assets/design-qa/minimum-window.png) confirms the smaller desktop layout.

## Deliberate adaptations to real functionality

- The real 82 × 42 map keeps its aspect ratio and full extent instead of being cropped or stretched to the mockup's illustrative map shape. This leaves additional vertical space at some window sizes.
- Existing Fleet/Priority and AGV/Planner/Debug switches remain available even where the initial mockup omitted them. Detailed priority breakdowns are shown for the selected row; the complete ranked list remains scrollable.
- Coordinates, task counts, charge reserve, distances, planner metrics and button states come from the actual engine/session. There is no invented task ID, path length in meters or estimated arrival time. The main fleet column is labeled Done to accurately describe completed tasks.
- Normal frame capture shows position and destination in the preview. Planned routes and diagnostic overlays require Debug capture, as in the existing protocol.
- The explicit Reset and Events actions and the existing keyboard shortcuts remain available. Text labels replace decorative icon-only controls where the existing action benefits from a clear name.
- Longer forms, expanded telemetry and full diagnostic data scroll within their own regions. Mobile is outside this desktop application's supported window scope.

The above-the-fold copy review found only the functional adaptations listed above; no decorative marketing copy or unsupported product claims were added.

## Functional and code checks

- 27 GUI verification items passed, covering custom/realtime configuration, preset save/apply/delete, phase add/edit/reorder/remove/max/reset, validation errors and recovery, real session load/run/pause/step/reset, playback pacing, map/list selection, priority modes, planner/debug capture, all nine layer toggles, events, visibility/focus restore, analysis/full metrics, keyboard shortcuts, dialog focus, empty state, minimum window size and reduced motion.
- The real playback flow advanced 58 frames while announcing only execution-state changes, avoiding per-frame live-region repetition.
- 19 existing C++ tests passed: `LaunchWorkflowTest.*`, `RenderModelTest.*`, `DebugSupportTest.PublicDebugSnapshotCapturesLogsAgentsAndFrame`, and `SimulationEngineTest.DifferentAlgorithmsRemainExecutableThroughPublicApi`.
- JavaScript syntax checks passed for the renderer, workspace shell behavior, Motion adapter and main process.
- Whitespace checks passed with `cr-at-eol` enabled to honor the existing Windows line endings; actual trailing spaces were removed.

[GUI check record](docs/assets/design-qa/functional-checks.json) · [Console evidence](docs/assets/design-qa/console-health.json) · [Viewport evidence](docs/assets/design-qa/viewport-evidence.json)

Residual scope: the full C++ test suite, packaging/distribution and non-Windows platforms were not part of this GUI change. No remaining visual fixes are required for the reviewed states and supported desktop sizes.
