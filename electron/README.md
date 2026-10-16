# AGV Electron Shell

This directory contains a minimal Electron shell for the IPC-only architecture:

- Electron manages the GUI and the backend child process.
- The C++ core stays out-of-process in `agv_render_ipc.exe`.
- The renderer consumes only structured IPC payloads.

## Run

1. Build the C++ backend first.
   - `cmake -S . -B build-codex -G "MinGW Makefiles"`
   - `cmake --build build-codex -j4`
2. Install the desktop dependencies.
   - `cd electron`
   - `npm ci`
3. Start the shell.
   - `npm start`

If the backend executable is not under `build-codex/agv_render_ipc.exe`, set `AGV_RENDER_IPC_PATH` before launching Electron.

## Workspace

- **Simulation** keeps the fleet list, map and selected AGV inspector in separate regions. The fleet also has a priority view with detailed and minimal modes.
- **Analysis** shows step-based trends and operational status. **All metrics** opens the complete runtime, planner and fairness data.
- **Setup** contains map and algorithm selection, scenario presets, editable phases or realtime probabilities, and advanced settings. Validation feedback appears next to the launch actions.
- **Layers** controls the map overlays. Planned paths, CBS reroutes and conflict information require **Debug** capture in Setup.
- **Events** opens the engine log and activity dock. **Settings** controls panel visibility, and **Focus map** temporarily hides panels while preserving the previous visibility choices.
- The bottom bar controls running, pausing, stepping and resetting. **Speed** changes playback pacing; the separate engine speed parameter remains in Setup.

Click a fleet row or an AGV on the map to inspect it. Additional telemetry is under **More details**. The small map shows the actual position and goal; a planned route appears when the engine provides one.

In the simulation workspace, **Space** runs or pauses, **S** steps, **L** opens Setup, **M** opens all metrics, and **R** resets the session. The left and right arrows cycle the simulation, map and analysis views. **Escape** closes the active dialog or popover. Text input retains its normal keyboard behavior.
