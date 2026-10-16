"use strict";

const fs = require("fs");
const path = require("path");

const { GIFEncoder, quantize, applyPalette } = require("gifenc");

function resampleBgraFrameToRgba(bitmap, sourceWidth, sourceHeight, targetWidth) {
  const width = Math.max(1, Math.floor(targetWidth));
  const height = Math.max(1, Math.round((sourceHeight * width) / sourceWidth));
  const rgba = new Uint8Array(width * height * 4);

  // Preserve antialiased UI text when shrinking a captured desktop frame.
  for (let y = 0; y < height; y += 1) {
    const sourceY = Math.max(0, Math.min(sourceHeight - 1, ((y + 0.5) * sourceHeight) / height - 0.5));
    const top = Math.floor(sourceY);
    const bottom = Math.min(sourceHeight - 1, top + 1);
    const yWeight = sourceY - top;
    for (let x = 0; x < width; x += 1) {
      const sourceX = Math.max(0, Math.min(sourceWidth - 1, ((x + 0.5) * sourceWidth) / width - 0.5));
      const left = Math.floor(sourceX);
      const right = Math.min(sourceWidth - 1, left + 1);
      const xWeight = sourceX - left;
      const topLeft = (top * sourceWidth + left) * 4;
      const topRight = (top * sourceWidth + right) * 4;
      const bottomLeft = (bottom * sourceWidth + left) * 4;
      const bottomRight = (bottom * sourceWidth + right) * 4;
      const targetIndex = (y * width + x) * 4;
      for (let channel = 0; channel < 3; channel += 1) {
        const bgraChannel = 2 - channel;
        const topValue = bitmap[topLeft + bgraChannel] * (1 - xWeight) + bitmap[topRight + bgraChannel] * xWeight;
        const bottomValue = bitmap[bottomLeft + bgraChannel] * (1 - xWeight) + bitmap[bottomRight + bgraChannel] * xWeight;
        rgba[targetIndex + channel] = Math.round(topValue * (1 - yWeight) + bottomValue * yWeight);
      }
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
