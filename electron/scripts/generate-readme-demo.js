"use strict";

const fs = require("fs");
const path = require("path");
const { spawn } = require("child_process");

const { createGifFromManifest } = require("./gif-encoder");

const repoRoot = path.resolve(__dirname, "..", "..");
const electronRoot = path.resolve(__dirname, "..");
const electronModuleRoot = path.join(electronRoot, "node_modules", "electron");
const electronPathFile = path.join(electronModuleRoot, "path.txt");
const captureRoot = path.join(electronRoot, "output", "readme-demo");
const manifestPath = path.join(captureRoot, "manifest.json");
const artifactPath = path.join(
  repoRoot,
  "docs",
  "assets",
  "readme",
  "agv-shell-demo.gif",
);
const targetWidth = 960;

function resolveElectronBinary() {
  if (!fs.existsSync(electronPathFile)) {
    throw new Error(`Electron path metadata was not found at ${electronPathFile}.`);
  }
  const executableName = fs.readFileSync(electronPathFile, "utf8").trim();
  const electronBinary = path.join(electronModuleRoot, "dist", executableName);
  if (!fs.existsSync(electronBinary)) {
    throw new Error(`Electron binary was not found at ${electronBinary}.`);
  }
  return electronBinary;
}

function ensureDirectory(filePath) {
  fs.mkdirSync(path.dirname(filePath), { recursive: true });
}

function runElectronReadmeDemo() {
  return new Promise((resolve, reject) => {
    const electronBinary = resolveElectronBinary();
    fs.rmSync(captureRoot, { recursive: true, force: true });
    ensureDirectory(manifestPath);
    ensureDirectory(artifactPath);

    const child = spawn(electronBinary, ["."], {
      cwd: electronRoot,
      env: {
        ...process.env,
        AGV_README_DEMO: "1",
        AGV_README_DEMO_CAPTURE_DIR: captureRoot,
        AGV_README_DEMO_MANIFEST_PATH: manifestPath,
        AGV_README_DEMO_CAPTURE_INTERVAL_MS: "150",
        AGV_README_DEMO_WINDOW_WIDTH: "1440",
        AGV_README_DEMO_WINDOW_HEIGHT: "900",
      },
      stdio: "inherit",
      windowsHide: false,
    });

    child.on("error", reject);
    child.on("exit", (code) => {
      if (code !== 0) {
        reject(new Error(`Electron README demo exited with code ${code}.`));
        return;
      }
      resolve();
    });
  });
}

function loadManifest() {
  if (!fs.existsSync(manifestPath)) {
    throw new Error(`README demo manifest was not generated at ${manifestPath}.`);
  }

  const manifest = JSON.parse(fs.readFileSync(manifestPath, "utf8"));
  if (manifest.success === false) {
    throw new Error(
      `Renderer reported README demo failure: ${manifest.result?.error || "unknown error"}.`,
    );
  }
  return manifest;
}

async function main() {
  await runElectronReadmeDemo();
  const manifest = loadManifest();
  const summary = createGifFromManifest(manifest, artifactPath, targetWidth);
  fs.rmSync(captureRoot, { recursive: true, force: true });
  console.log(
    `README demo GIF created at ${artifactPath} (${summary.width}x${summary.height}, ${summary.frameCount} frames).`,
  );
}

main().catch((error) => {
  console.error(error.message);
  process.exitCode = 1;
});
