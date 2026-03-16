"use strict";

const fs = require("fs");
const path = require("path");

const { GIFEncoder, quantize, applyPalette } = require("gifenc");

function resampleBgraFrameToRgba(bitmap, sourceWidth, sourceHeight, targetWidth) {
  const width = Math.max(1, Math.floor(targetWidth));
  const height = Math.max(1, Math.round((sourceHeight * width) / sourceWidth));
  const rgba = new Uint8Array(width * height * 4);

  for (let y = 0; y < height; y += 1) {
    const sourceY = Math.min(
      sourceHeight - 1,
      Math.floor(((y + 0.5) * sourceHeight) / height),
    );
    for (let x = 0; x < width; x += 1) {
      const sourceX = Math.min(
        sourceWidth - 1,
        Math.floor(((x + 0.5) * sourceWidth) / width),
      );
      const sourceIndex = (sourceY * sourceWidth + sourceX) * 4;
      const targetIndex = (y * width + x) * 4;
      rgba[targetIndex] = bitmap[sourceIndex + 2];
      rgba[targetIndex + 1] = bitmap[sourceIndex + 1];
      rgba[targetIndex + 2] = bitmap[sourceIndex];
      rgba[targetIndex + 3] = 255;
    }
  }

  return { width, height, rgba };
}

function createGifFromManifest(manifest, outputPath, targetWidth) {
  if (!Array.isArray(manifest.frames) || manifest.frames.length === 0) {
    throw new Error("No captured frames were recorded for the README demo.");
  }

  const gif = GIFEncoder();
  const rootDir = manifest.captureDir;
  let width = 0;
  let height = 0;
  let frameCount = 0;

  for (const frame of manifest.frames) {
    const framePath = path.join(rootDir, frame.file);
    const bitmap = fs.readFileSync(framePath);
    const resampled = resampleBgraFrameToRgba(
      bitmap,
      Number(frame.width),
      Number(frame.height),
      targetWidth,
    );

    width = resampled.width;
    height = resampled.height;

    const palette = quantize(resampled.rgba, 192, { format: "rgb565" });
    const index = applyPalette(resampled.rgba, palette, "rgb565");
    gif.writeFrame(index, width, height, {
      palette,
      delay: Number(frame.delayMs) || 150,
      repeat: frameCount === 0 ? 0 : undefined,
    });
    frameCount += 1;
  }

  gif.finish();
  fs.mkdirSync(path.dirname(outputPath), { recursive: true });
  fs.writeFileSync(outputPath, Buffer.from(gif.bytes()));

  return {
    width,
    height,
    frameCount,
  };
}

module.exports = {
  createGifFromManifest,
};
