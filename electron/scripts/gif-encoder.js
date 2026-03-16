"use strict";

const fs = require("fs");
const path = require("path");

const kWebSafeLevels = [0, 51, 102, 153, 204, 255];
const kGrayRamp = Array.from({ length: 40 }, (_value, index) =>
  Math.round((index * 255) / 39),
);

function buildPalette() {
  const rgb = [];
  for (const red of kWebSafeLevels) {
    for (const green of kWebSafeLevels) {
      for (const blue of kWebSafeLevels) {
        rgb.push(red, green, blue);
      }
    }
  }
  for (const gray of kGrayRamp) {
    rgb.push(gray, gray, gray);
  }
  return Buffer.from(rgb);
}

function closestPaletteIndex(red, green, blue) {
  const redBand = Math.max(
    0,
    Math.min(kWebSafeLevels.length - 1, Math.round(red / 51)),
  );
  const greenBand = Math.max(
    0,
    Math.min(kWebSafeLevels.length - 1, Math.round(green / 51)),
  );
  const blueBand = Math.max(
    0,
    Math.min(kWebSafeLevels.length - 1, Math.round(blue / 51)),
  );

  const webRed = kWebSafeLevels[redBand];
  const webGreen = kWebSafeLevels[greenBand];
  const webBlue = kWebSafeLevels[blueBand];
  const webDistance =
    (red - webRed) * (red - webRed) +
    (green - webGreen) * (green - webGreen) +
    (blue - webBlue) * (blue - webBlue);
  const webIndex = redBand * 36 + greenBand * 6 + blueBand;

  const luminance = Math.round((red + green + blue) / 3);
  const grayBand = Math.max(
    0,
    Math.min(kGrayRamp.length - 1, Math.round((luminance * 39) / 255)),
  );
  const grayValue = kGrayRamp[grayBand];
  const grayDistance =
    (red - grayValue) * (red - grayValue) +
    (green - grayValue) * (green - grayValue) +
    (blue - grayValue) * (blue - grayValue);

  return grayDistance < webDistance ? 216 + grayBand : webIndex;
}

function resampleFrameToIndices(bitmap, sourceWidth, sourceHeight, targetWidth) {
  const width = Math.max(1, Math.floor(targetWidth));
  const height = Math.max(1, Math.round((sourceHeight * width) / sourceWidth));
  const indices = Buffer.alloc(width * height);

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
      const blue = bitmap[sourceIndex];
      const green = bitmap[sourceIndex + 1];
      const red = bitmap[sourceIndex + 2];
      indices[y * width + x] = closestPaletteIndex(red, green, blue);
    }
  }

  return { width, height, indices };
}

function encodeLittleEndian(value) {
  const bytes = Buffer.alloc(2);
  bytes.writeUInt16LE(value, 0);
  return bytes;
}

function createBitWriter() {
  const bytes = [];
  let currentByte = 0;
  let bitOffset = 0;

  return {
    write(code, codeSize) {
      let value = code;
      let size = codeSize;
      while (size > 0) {
        currentByte |= (value & 1) << bitOffset;
        value >>= 1;
        bitOffset += 1;
        size -= 1;
        if (bitOffset === 8) {
          bytes.push(currentByte);
          currentByte = 0;
          bitOffset = 0;
        }
      }
    },
    finish() {
      if (bitOffset > 0) {
        bytes.push(currentByte);
      }
      return Buffer.from(bytes);
    },
  };
}

function lzwEncode(indices, minCodeSize) {
  const clearCode = 1 << minCodeSize;
  const endCode = clearCode + 1;
  const writer = createBitWriter();
  const dictionary = new Map();
  let codeSize = minCodeSize + 1;
  let nextCode = endCode + 1;

  writer.write(clearCode, codeSize);

  let prefix = indices[0];
  for (let position = 1; position < indices.length; position += 1) {
    const current = indices[position];
    const key = prefix * 256 + current;
    const existing = dictionary.get(key);
    if (existing !== undefined) {
      prefix = existing;
      continue;
    }

    writer.write(prefix, codeSize);
    if (nextCode < 4096) {
      dictionary.set(key, nextCode);
      nextCode += 1;
      if (nextCode === 1 << codeSize && codeSize < 12) {
        codeSize += 1;
      }
    } else {
      writer.write(clearCode, codeSize);
      dictionary.clear();
      codeSize = minCodeSize + 1;
      nextCode = endCode + 1;
    }
    prefix = current;
  }

  writer.write(prefix, codeSize);
  writer.write(endCode, codeSize);
  return writer.finish();
}

function toSubBlocks(data) {
  const chunks = [];
  for (let offset = 0; offset < data.length; offset += 255) {
    const chunk = data.subarray(offset, offset + 255);
    chunks.push(Buffer.from([chunk.length]));
    chunks.push(chunk);
  }
  chunks.push(Buffer.from([0]));
  return Buffer.concat(chunks);
}

function encodeFrame(indices, width, height, delayMs) {
  const minCodeSize = 8;
  const compressed = lzwEncode(indices, minCodeSize);
  const delayCentiseconds = Math.max(2, Math.round(delayMs / 10));

  return Buffer.concat([
    Buffer.from([
      0x21,
      0xf9,
      0x04,
      0x04,
      delayCentiseconds & 0xff,
      (delayCentiseconds >> 8) & 0xff,
      0x00,
      0x00,
      0x2c,
      0x00,
      0x00,
      0x00,
      0x00,
    ]),
    encodeLittleEndian(width),
    encodeLittleEndian(height),
    Buffer.from([0x00, minCodeSize]),
    toSubBlocks(compressed),
  ]);
}

function encodeGif({ frames, outputPath, width, height }) {
  const palette = buildPalette();
  const header = Buffer.concat([
    Buffer.from("GIF89a", "ascii"),
    encodeLittleEndian(width),
    encodeLittleEndian(height),
    Buffer.from([0xf7, 0x00, 0x00]),
    palette,
    Buffer.from([
      0x21,
      0xff,
      0x0b,
      0x4e,
      0x45,
      0x54,
      0x53,
      0x43,
      0x41,
      0x50,
      0x45,
      0x32,
      0x2e,
      0x30,
      0x03,
      0x01,
      0x00,
      0x00,
      0x00,
    ]),
  ]);

  const body = [header];
  for (const frame of frames) {
    body.push(encodeFrame(frame.indices, width, height, frame.delayMs));
  }
  body.push(Buffer.from([0x3b]));
  fs.writeFileSync(outputPath, Buffer.concat(body));
}

function createGifFromManifest(manifest, outputPath, targetWidth) {
  if (!Array.isArray(manifest.frames) || manifest.frames.length === 0) {
    throw new Error("No captured frames were recorded for the README demo.");
  }

  const rootDir = manifest.captureDir;
  const encodedFrames = [];
  let width = 0;
  let height = 0;

  for (const frame of manifest.frames) {
    const framePath = path.join(rootDir, frame.file);
    const bitmap = fs.readFileSync(framePath);
    const resampled = resampleFrameToIndices(
      bitmap,
      Number(frame.width),
      Number(frame.height),
      targetWidth,
    );
    width = resampled.width;
    height = resampled.height;
    encodedFrames.push({
      delayMs: Number(frame.delayMs) || 150,
      indices: resampled.indices,
    });
  }

  encodeGif({
    frames: encodedFrames,
    outputPath,
    width,
    height,
  });

  return {
    width,
    height,
    frameCount: encodedFrames.length,
  };
}

module.exports = {
  createGifFromManifest,
};
