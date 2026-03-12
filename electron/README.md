# AGV Electron Shell

This directory contains a minimal Electron shell for the IPC-only architecture:

- Electron manages the GUI and the backend child process.
- The C++ core stays out-of-process in `agv_render_ipc.exe`.
- The renderer consumes only structured IPC payloads.

## Run

1. Build the C++ backend first.
   - `cmake -S . -B build-codex -G "MinGW Makefiles"`
   - `cmake --build build-codex -j4`
2. Install Electron.
   - `cd electron`
   - `npm install`
3. Start the shell.
   - `npm start`

If the backend executable is not under `build-codex/agv_render_ipc.exe`, set `AGV_RENDER_IPC_PATH` before launching Electron.
