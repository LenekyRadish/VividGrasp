import "./styles.css";
import tennisTextureUrl from "../tennis.jpg";
import baseballTextureUrl from "../baseball.jpg";

const CAMERA_SIZE = 600;
const CAMERA_Z = 1.0;
const CAMERA_FOV = 42;
const GROUND_HALF = CAMERA_Z * Math.tan(toRadians(CAMERA_FOV / 2));
const WORLD_MIN_X = -GROUND_HALF;
const WORLD_MAX_X = GROUND_HALF;
const WORLD_MIN_Y = -GROUND_HALF;
const WORLD_MAX_Y = GROUND_HALF;
const PIXELS_PER_METER = CAMERA_SIZE / (WORLD_MAX_X - WORLD_MIN_X);
const ARM_UPPER = 0.24;
const ARM_LOWER = 0.22;

const ARM = {
  upper: ARM_UPPER,
  lower: ARM_LOWER,
  home: solveIk(-0.14, -0.2),
};

const BALL_TYPES = {
  tennis: {
    label: "Tennis",
    radius: 0.0327,
    bin: "Bin 2",
    tray: { x: 0.32, y: 0.18 },
    fill: "#91d93e",
    stroke: "#467814",
  },
  baseball: {
    label: "Baseball",
    radius: 0.0365,
    bin: "Bin 1",
    tray: { x: 0.34, y: 0.0 },
    fill: "#f4f1e6",
    stroke: "#9d423d",
  },
  ping_pong: {
    label: "Ping-pong",
    radius: 0.02,
    bin: "Bin 3",
    tray: { x: 0.32, y: -0.18 },
    fill: "#ed762f",
    stroke: "#9a451c",
  },
};

const ORDER = ["tennis", "baseball", "ping_pong"];
const MASK_COLORS = {
  tennis: [140, 214, 54],
  baseball: [245, 242, 226],
  ping_pong: [237, 118, 47],
};

const dom = {
  camera: document.querySelector("#cameraCanvas"),
  masks: document.querySelector("#maskCanvas"),
  world: document.querySelector("#worldCanvas"),
  runButton: document.querySelector("#runButton"),
  resetButton: document.querySelector("#resetButton"),
  speedRange: document.querySelector("#speedRange"),
  speedValue: document.querySelector("#speedValue"),
  detectionCount: document.querySelector("#detectionCount"),
  runStatus: document.querySelector("#runStatus"),
  maskStatus: document.querySelector("#maskStatus"),
  sortCount: document.querySelector("#sortCount"),
  stepStatus: document.querySelector("#stepStatus"),
  detectionRows: document.querySelector("#detectionRows"),
  eventLog: document.querySelector("#eventLog"),
};

const cameraCtx = dom.camera.getContext("2d", { willReadFrequently: true });
const maskCtx = dom.masks.getContext("2d");
const worldCtx = dom.world.getContext("2d");
const textures = {
  tennis: loadImage(tennisTextureUrl),
  baseball: loadImage(baseballTextureUrl),
};

const state = {
  objects: createScene(),
  detections: {},
  masks: {},
  steps: [],
  logs: [],
  sorted: new Set(),
  held: null,
  running: false,
  speed: 1,
  stepIndex: 0,
  stepProgress: 0,
  currentAngles: ARM.home,
  lastFrame: 0,
};

dom.runButton.addEventListener("click", () => {
  if (state.running) {
    state.running = false;
    dom.runButton.textContent = "Resume Showcase";
    state.logs.unshift(makeLog("Paused", "Arm motion is paused at the current waypoint."));
    updateUi();
    return;
  }

  if (!state.steps.length || state.stepIndex >= state.steps.length) {
    startShowcase();
  } else {
    state.running = true;
    dom.runButton.textContent = "Pause";
    state.logs.unshift(makeLog("Resumed", "Continuing the current sorting path."));
    updateUi();
  }
});

dom.resetButton.addEventListener("click", () => {
  resetScene();
});

dom.speedRange.addEventListener("input", (event) => {
  state.speed = Number(event.target.value);
  dom.speedValue.value = `${state.speed.toFixed(2)}x`;
});

for (const image of Object.values(textures)) {
  image.addEventListener("load", () => {
    analyzeScene();
    drawWorld();
  });
}

resetScene();
requestAnimationFrame(tick);

function resetScene() {
  state.objects = createScene();
  state.detections = {};
  state.masks = {};
  state.steps = [];
  state.logs = [
    makeLog("Scene ready", "Three sports objects are inside the camera workspace."),
    makeLog("Vision pass", "HSV thresholds are scanning the synthetic camera frame."),
  ];
  state.sorted = new Set();
  state.held = null;
  state.running = false;
  state.stepIndex = 0;
  state.stepProgress = 0;
  state.currentAngles = ARM.home;
  dom.runButton.textContent = "Run Showcase";
  analyzeScene();
  updateUi();
  drawWorld();
}

function startShowcase() {
  analyzeScene();
  state.steps = buildMotionPlan();
  state.sorted = new Set();
  state.held = null;
  state.stepIndex = 0;
  state.stepProgress = 0;
  state.currentAngles = ARM.home;
  state.running = state.steps.length > 0;
  dom.runButton.textContent = state.running ? "Pause" : "Run Showcase";
  state.logs.unshift(makeLog("Motion plan", `${state.steps.length} waypoints queued from camera data.`));

  if (state.steps[0]) {
    state.logs.unshift(makeLog("Seeking object", state.steps[0].label));
  }

  updateUi();
}

function tick(timestamp) {
  const dt = state.lastFrame ? Math.min((timestamp - state.lastFrame) / 1000, 0.05) : 0;
  state.lastFrame = timestamp;

  if (state.running) {
    advanceMotion(dt * state.speed);
  }

  drawWorld();
  requestAnimationFrame(tick);
}

function advanceMotion(dt) {
  const step = state.steps[state.stepIndex];
  if (!step) {
    state.running = false;
    dom.runButton.textContent = "Run Showcase";
    updateUi();
    return;
  }

  state.stepProgress += dt / step.duration;

  if (state.stepProgress >= 1) {
    state.currentAngles = step.toAngles;

    if (step.phase === "pickup") {
      state.held = step.ball;
      state.objects[step.ball].state = "carried";
      state.logs.unshift(makeLog("Object attached", `${BALL_TYPES[step.ball].label} is held by the end effector.`));
    }

    if (step.phase === "drop") {
      state.objects[step.ball].world = { ...BALL_TYPES[step.ball].tray };
      state.objects[step.ball].state = "sorted";
      state.sorted.add(step.ball);
      state.held = null;
      state.logs.unshift(makeLog("Sorted", `${BALL_TYPES[step.ball].label} placed in ${BALL_TYPES[step.ball].bin}.`));
    }

    state.stepIndex += 1;
    state.stepProgress = 0;

    const nextStep = state.steps[state.stepIndex];
    if (nextStep) {
      state.logs.unshift(makeLog(nextStep.phase === "pickup" ? "Seeking object" : "Moving to tray", nextStep.label));
    } else {
      state.running = false;
      dom.runButton.textContent = "Run Showcase";
      state.logs.unshift(makeLog("Complete", "All detected objects have been moved to their trays."));
    }

    updateUi();
  }
}

function buildMotionPlan() {
  const steps = [];
  let angles = ARM.home;

  for (const key of ORDER) {
    const detection = state.detections[key];
    if (!detection) {
      continue;
    }

    const type = BALL_TYPES[key];
    const pickupAngles = solveIk(detection.world.x, detection.world.y);
    steps.push({
      ball: key,
      phase: "pickup",
      label: `${type.label} pickup at (${formatNumber(detection.world.x)}, ${formatNumber(detection.world.y)})`,
      fromAngles: angles,
      toAngles: pickupAngles,
      duration: 1.25,
      target: detection.world,
    });
    angles = pickupAngles;

    const dropAngles = solveIk(type.tray.x, type.tray.y);
    steps.push({
      ball: key,
      phase: "drop",
      label: `${type.label} drop into ${type.bin}`,
      fromAngles: angles,
      toAngles: dropAngles,
      duration: 1.45,
      target: type.tray,
    });
    angles = dropAngles;
  }

  return steps;
}

function analyzeScene() {
  drawCamera(false);
  const imageData = cameraCtx.getImageData(0, 0, CAMERA_SIZE, CAMERA_SIZE);
  const scan = scanHsvMasks(imageData);
  state.masks = scan.masks;
  state.detections = {};

  for (const key of ORDER) {
    const component = pickBestComponent(scan.components[key]);
    if (!component) {
      continue;
    }

    const world = convertCoordinates(component.cx, component.cy, BALL_TYPES[key].radius);
    state.detections[key] = {
      type: key,
      pixel: { x: component.cx, y: component.cy },
      world,
      area: component.area,
      radiusEstimate: component.radiusEstimate,
      averageHue: component.averageHue,
      averageSaturation: component.averageSaturation,
      averageValue: component.averageValue,
      bin: classifyBall(component.averageHue, component.averageSaturation, component.radiusEstimate),
      box: component.box,
    };
  }

  drawCamera(true);
  drawMasks();
  updateUi();
}

function scanHsvMasks(imageData) {
  const pixels = imageData.data;
  const count = CAMERA_SIZE * CAMERA_SIZE;
  const hue = new Float32Array(count);
  const saturation = new Float32Array(count);
  const value = new Float32Array(count);
  const masks = {
    tennis: new Uint8Array(count),
    baseball: new Uint8Array(count),
    ping_pong: new Uint8Array(count),
  };

  for (let i = 0; i < count; i += 1) {
    const p = i * 4;
    const hsv = rgbToHsv(pixels[p], pixels[p + 1], pixels[p + 2]);
    hue[i] = hsv.h;
    saturation[i] = hsv.s;
    value[i] = hsv.v;

    if (isTennis(hsv)) masks.tennis[i] = 1;
    if (isBaseball(hsv)) masks.baseball[i] = 1;
    if (isPingPong(hsv)) masks.ping_pong[i] = 1;
  }

  return {
    masks,
    components: {
      tennis: findComponents(masks.tennis, hue, saturation, value, 120),
      baseball: findComponents(masks.baseball, hue, saturation, value, 150),
      ping_pong: findComponents(masks.ping_pong, hue, saturation, value, 70),
    },
  };
}

function findComponents(mask, hue, saturation, value, minArea) {
  const visited = new Uint8Array(mask.length);
  const components = [];
  const queue = new Int32Array(mask.length);

  for (let index = 0; index < mask.length; index += 1) {
    if (!mask[index] || visited[index]) {
      continue;
    }

    let head = 0;
    let tail = 0;
    queue[tail] = index;
    tail += 1;
    visited[index] = 1;

    let area = 0;
    let sumX = 0;
    let sumY = 0;
    let sumHue = 0;
    let sumSaturation = 0;
    let sumValue = 0;
    let minX = CAMERA_SIZE;
    let minY = CAMERA_SIZE;
    let maxX = 0;
    let maxY = 0;

    const pushNeighbor = (next, valid) => {
      if (valid && mask[next] && !visited[next]) {
        visited[next] = 1;
        queue[tail] = next;
        tail += 1;
      }
    };

    while (head < tail) {
      const current = queue[head];
      head += 1;

      const x = current % CAMERA_SIZE;
      const y = Math.floor(current / CAMERA_SIZE);
      area += 1;
      sumX += x;
      sumY += y;
      sumHue += hue[current];
      sumSaturation += saturation[current];
      sumValue += value[current];
      minX = Math.min(minX, x);
      minY = Math.min(minY, y);
      maxX = Math.max(maxX, x);
      maxY = Math.max(maxY, y);

      pushNeighbor(current - 1, x > 0);
      pushNeighbor(current + 1, x < CAMERA_SIZE - 1);
      pushNeighbor(current - CAMERA_SIZE, y > 0);
      pushNeighbor(current + CAMERA_SIZE, y < CAMERA_SIZE - 1);
    }

    if (area >= minArea) {
      const boxWidth = maxX - minX + 1;
      const boxHeight = maxY - minY + 1;
      const fillRatio = area / Math.max(1, boxWidth * boxHeight);
      const radiusEstimate = Math.sqrt(area / Math.PI) / PIXELS_PER_METER;

      components.push({
        area,
        cx: sumX / area,
        cy: sumY / area,
        averageHue: sumHue / area,
        averageSaturation: sumSaturation / area,
        averageValue: sumValue / area,
        radiusEstimate,
        fillRatio,
        box: { minX, minY, maxX, maxY },
      });
    }
  }

  return components;
}

function pickBestComponent(components) {
  if (!components.length) {
    return null;
  }

  return [...components].sort((a, b) => {
    const scoreA = a.area * a.fillRatio;
    const scoreB = b.area * b.fillRatio;
    return scoreB - scoreA;
  })[0];
}

function isTennis({ h, s, v }) {
  return h >= 65 && h <= 170 && s > 0.28 && v > 0.35;
}

function isBaseball({ s, v }) {
  return s < 0.22 && v > 0.72;
}

function isPingPong({ h, s, v }) {
  return h >= 12 && h <= 46 && s > 0.35 && v > 0.35;
}

function classifyBall(hue, saturation, radius) {
  if (saturation < 0.2 && radius > 0.024) {
    return "Bin 1";
  }

  if (hue >= 65 && hue <= 170) {
    return "Bin 2";
  }

  if (radius <= 0.026 || (hue >= 12 && hue <= 52)) {
    return "Bin 3";
  }

  return "Bin 1";
}

function drawCamera(withOverlay) {
  const ctx = cameraCtx;
  ctx.clearRect(0, 0, CAMERA_SIZE, CAMERA_SIZE);

  const gradient = ctx.createLinearGradient(0, 0, CAMERA_SIZE, CAMERA_SIZE);
  gradient.addColorStop(0, "#252b2d");
  gradient.addColorStop(1, "#32393b");
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, CAMERA_SIZE, CAMERA_SIZE);

  drawCameraGrid(ctx);
  drawCameraTrays(ctx);

  for (const key of ORDER) {
    const object = state.objects[key];
    const px = worldToPixel(object.world.x, object.world.y);
    drawBallOnCamera(ctx, key, px.x, px.y, BALL_TYPES[key].radius * PIXELS_PER_METER);
  }

  if (!withOverlay) {
    return;
  }

  for (const detection of Object.values(state.detections)) {
    const type = BALL_TYPES[detection.type];
    const { minX, minY, maxX, maxY } = detection.box;
    ctx.save();
    ctx.strokeStyle = "#101416";
    ctx.lineWidth = 7;
    ctx.strokeRect(minX, minY, maxX - minX, maxY - minY);
    ctx.strokeStyle = type.fill;
    ctx.lineWidth = 3;
    ctx.strokeRect(minX, minY, maxX - minX, maxY - minY);
    ctx.fillStyle = "#de3f35";
    ctx.beginPath();
    ctx.arc(detection.pixel.x, detection.pixel.y, 6, 0, Math.PI * 2);
    ctx.fill();
    drawCanvasLabel(ctx, `${type.label} ${detection.bin}`, detection.pixel.x + 12, detection.pixel.y - 13);
    ctx.restore();
  }
}

function drawCameraGrid(ctx) {
  ctx.save();
  ctx.strokeStyle = "rgba(255, 255, 255, 0.08)";
  ctx.lineWidth = 1;
  for (let i = 0; i <= CAMERA_SIZE; i += 75) {
    ctx.beginPath();
    ctx.moveTo(i, 0);
    ctx.lineTo(i, CAMERA_SIZE);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(0, i);
    ctx.lineTo(CAMERA_SIZE, i);
    ctx.stroke();
  }
  ctx.strokeStyle = "rgba(255, 255, 255, 0.2)";
  ctx.strokeRect(42, 42, CAMERA_SIZE - 84, CAMERA_SIZE - 84);
  ctx.restore();
}

function drawCameraTrays(ctx) {
  for (const key of ORDER) {
    const type = BALL_TYPES[key];
    const center = worldToPixel(type.tray.x, type.tray.y);
    ctx.save();
    ctx.translate(center.x, center.y);
    ctx.fillStyle = "rgba(255, 255, 255, 0.1)";
    ctx.strokeStyle = "rgba(255, 255, 255, 0.34)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.roundRect(-42, -28, 84, 56, 8);
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  }
}

function drawBallOnCamera(ctx, key, x, y, radius) {
  const type = BALL_TYPES[key];
  ctx.save();
  ctx.beginPath();
  ctx.arc(x, y, radius, 0, Math.PI * 2);
  ctx.clip();

  if (key === "tennis" && textures.tennis.complete) {
    drawImageCover(ctx, textures.tennis, x - radius, y - radius, radius * 2, radius * 2);
    ctx.fillStyle = "rgba(145, 217, 62, 0.45)";
    ctx.fillRect(x - radius, y - radius, radius * 2, radius * 2);
  } else if (key === "baseball" && textures.baseball.complete) {
    drawImageCover(ctx, textures.baseball, x - radius, y - radius, radius * 2, radius * 2);
    ctx.fillStyle = "rgba(255, 255, 255, 0.25)";
    ctx.fillRect(x - radius, y - radius, radius * 2, radius * 2);
  } else {
    ctx.fillStyle = type.fill;
    ctx.fillRect(x - radius, y - radius, radius * 2, radius * 2);
  }

  ctx.restore();

  ctx.save();
  ctx.strokeStyle = type.stroke;
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.arc(x, y, radius, 0, Math.PI * 2);
  ctx.stroke();

  if (key === "ping_pong") {
    ctx.strokeStyle = "rgba(255,255,255,0.35)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(x - radius * 0.18, y - radius * 0.12, radius * 0.52, 0.15, Math.PI * 1.35);
    ctx.stroke();
  }

  ctx.restore();
}

function drawMasks() {
  const ctx = maskCtx;
  const width = dom.masks.width;
  const height = dom.masks.height;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#14191b";
  ctx.fillRect(0, 0, width, height);

  const paneWidth = width / ORDER.length;
  const imageHeight = height - 42;

  ORDER.forEach((key, paneIndex) => {
    const xOffset = Math.floor(paneIndex * paneWidth);
    const image = ctx.createImageData(paneWidth, imageHeight);
    const color = MASK_COLORS[key];
    const mask = state.masks[key];

    for (let y = 0; y < imageHeight; y += 1) {
      const sourceY = Math.floor((y / imageHeight) * CAMERA_SIZE);
      for (let x = 0; x < paneWidth; x += 1) {
        const sourceX = Math.floor((x / paneWidth) * CAMERA_SIZE);
        const sourceIndex = sourceY * CAMERA_SIZE + sourceX;
        const targetIndex = (y * paneWidth + x) * 4;
        const lit = mask && mask[sourceIndex];
        image.data[targetIndex] = lit ? color[0] : 28;
        image.data[targetIndex + 1] = lit ? color[1] : 33;
        image.data[targetIndex + 2] = lit ? color[2] : 35;
        image.data[targetIndex + 3] = 255;
      }
    }

    ctx.putImageData(image, xOffset, 0);
    ctx.strokeStyle = "rgba(255,255,255,0.16)";
    ctx.strokeRect(xOffset + 0.5, 0.5, paneWidth - 1, imageHeight - 1);
    drawCanvasLabel(ctx, `${BALL_TYPES[key].label} mask`, xOffset + 12, height - 17, "dark");
  });
}

function drawWorld() {
  const ctx = worldCtx;
  const width = dom.world.width;
  const height = dom.world.height;
  ctx.clearRect(0, 0, width, height);

  const gradient = ctx.createLinearGradient(0, 0, width, height);
  gradient.addColorStop(0, "#f5f7f7");
  gradient.addColorStop(1, "#e2e8ea");
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, width, height);

  drawWorldGrid(ctx);
  drawTrays(ctx);
  drawMotionPath(ctx);

  const angles = getRenderedAngles();
  const armPoints = getArmPoints(angles);

  for (const key of ORDER) {
    drawWorldBall(ctx, key, armPoints.hand);
  }

  drawArm(ctx, armPoints);
  drawWorldLabels(ctx);
}

function drawWorldGrid(ctx) {
  const width = dom.world.width;
  const height = dom.world.height;
  ctx.save();
  ctx.strokeStyle = "#ccd7dc";
  ctx.lineWidth = 1;
  for (let x = -0.4; x <= 0.401; x += 0.1) {
    const a = worldToScreen(x, -0.42);
    const b = worldToScreen(x, 0.42);
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.stroke();
  }
  for (let y = -0.4; y <= 0.401; y += 0.1) {
    const a = worldToScreen(-0.42, y);
    const b = worldToScreen(0.42, y);
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.stroke();
  }

  const min = worldToScreen(WORLD_MIN_X, WORLD_MIN_Y);
  const max = worldToScreen(WORLD_MAX_X, WORLD_MAX_Y);
  ctx.strokeStyle = "#7f8f96";
  ctx.lineWidth = 2;
  ctx.strokeRect(max.x, max.y, min.x - max.x, min.y - max.y);

  ctx.fillStyle = "rgba(23, 32, 38, 0.04)";
  ctx.beginPath();
  ctx.arc(width / 2, height / 2, 52, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
}

function drawTrays(ctx) {
  for (const key of ORDER) {
    const type = BALL_TYPES[key];
    const center = worldToScreen(type.tray.x, type.tray.y);
    ctx.save();
    ctx.translate(center.x, center.y);
    ctx.fillStyle = "#ffffff";
    ctx.strokeStyle = "#8fa0a7";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.roundRect(-44, -27, 88, 54, 8);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = type.fill;
    ctx.globalAlpha = 0.24;
    ctx.fillRect(-35, -18, 70, 36);
    ctx.globalAlpha = 1;
    ctx.fillStyle = "#172026";
    ctx.font = "700 12px Inter, sans-serif";
    ctx.textAlign = "center";
    ctx.fillText(type.bin, 0, 4);
    ctx.restore();
  }
}

function drawMotionPath(ctx) {
  if (!state.steps.length) {
    return;
  }

  ctx.save();
  ctx.lineWidth = 2;
  ctx.setLineDash([7, 7]);
  ctx.strokeStyle = "rgba(216, 102, 43, 0.65)";

  for (const step of state.steps) {
    const target = worldToScreen(step.target.x, step.target.y);
    const base = worldToScreen(0, 0);
    ctx.beginPath();
    ctx.moveTo(base.x, base.y);
    ctx.lineTo(target.x, target.y);
    ctx.stroke();
  }

  ctx.restore();
}

function drawWorldBall(ctx, key, handPosition) {
  const object = state.objects[key];
  const type = BALL_TYPES[key];
  let world = object.world;

  if (state.held === key) {
    world = handPosition;
  }

  const screen = worldToScreen(world.x, world.y);
  const radius = Math.max(11, type.radius * screenScale());

  ctx.save();
  ctx.shadowColor = "rgba(23, 32, 38, 0.24)";
  ctx.shadowBlur = 10;
  ctx.shadowOffsetY = 5;
  ctx.fillStyle = type.fill;
  ctx.strokeStyle = type.stroke;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(screen.x, screen.y, radius, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();

  if (key === "baseball") {
    ctx.strokeStyle = "#bd3f3a";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(screen.x - radius * 0.28, screen.y, radius * 0.65, -1.1, 1.1);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(screen.x + radius * 0.28, screen.y, radius * 0.65, Math.PI - 1.1, Math.PI + 1.1);
    ctx.stroke();
  }

  if (key === "tennis") {
    ctx.strokeStyle = "rgba(255,255,255,0.72)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(screen.x - radius * 0.25, screen.y, radius * 0.72, -1.0, 1.0);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(screen.x + radius * 0.25, screen.y, radius * 0.72, Math.PI - 1.0, Math.PI + 1.0);
    ctx.stroke();
  }

  ctx.restore();
}

function drawArm(ctx, points) {
  const base = worldToScreen(0, 0);
  const elbow = worldToScreen(points.elbow.x, points.elbow.y);
  const hand = worldToScreen(points.hand.x, points.hand.y);

  ctx.save();
  ctx.lineCap = "round";
  ctx.lineJoin = "round";

  ctx.strokeStyle = "#172026";
  ctx.lineWidth = 21;
  ctx.beginPath();
  ctx.moveTo(base.x, base.y);
  ctx.lineTo(elbow.x, elbow.y);
  ctx.lineTo(hand.x, hand.y);
  ctx.stroke();

  ctx.strokeStyle = "#d7a629";
  ctx.lineWidth = 13;
  ctx.beginPath();
  ctx.moveTo(base.x, base.y);
  ctx.lineTo(elbow.x, elbow.y);
  ctx.lineTo(hand.x, hand.y);
  ctx.stroke();

  drawJoint(ctx, base.x, base.y, 18, "#172026", "#ffffff");
  drawJoint(ctx, elbow.x, elbow.y, 14, "#f7fafb", "#172026");
  drawJoint(ctx, hand.x, hand.y, 12, "#d8662b", "#ffffff");
  ctx.restore();
}

function drawJoint(ctx, x, y, radius, fill, stroke) {
  ctx.fillStyle = fill;
  ctx.strokeStyle = stroke;
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.arc(x, y, radius, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
}

function drawWorldLabels(ctx) {
  ctx.save();
  ctx.font = "700 12px Inter, sans-serif";
  ctx.textAlign = "center";
  ctx.fillStyle = "#172026";

  for (const key of ORDER) {
    const type = BALL_TYPES[key];
    const tray = worldToScreen(type.tray.x, type.tray.y);
    ctx.fillText(type.label, tray.x, tray.y + 44);
  }

  const base = worldToScreen(0, 0);
  ctx.fillText("Base", base.x, base.y + 46);
  ctx.restore();
}

function getRenderedAngles() {
  const step = state.steps[state.stepIndex];
  if (!state.running || !step) {
    return state.currentAngles;
  }

  const eased = easeInOut(state.stepProgress);
  return {
    shoulder: lerp(step.fromAngles.shoulder, step.toAngles.shoulder, eased),
    elbow: lerp(step.fromAngles.elbow, step.toAngles.elbow, eased),
  };
}

function getArmPoints(angles) {
  const elbow = {
    x: ARM.upper * Math.cos(angles.shoulder),
    y: ARM.upper * Math.sin(angles.shoulder),
  };
  const hand = {
    x: elbow.x + ARM.lower * Math.cos(angles.shoulder + angles.elbow),
    y: elbow.y + ARM.lower * Math.sin(angles.shoulder + angles.elbow),
  };
  return { elbow, hand };
}

function updateUi() {
  const detections = Object.values(state.detections);
  dom.detectionCount.value = `${detections.length} object${detections.length === 1 ? "" : "s"}`;
  dom.sortCount.value = `${state.sorted.size} sorted`;
  dom.maskStatus.value = state.masks.tennis ? "Synced" : "Pending";

  const step = state.steps[state.stepIndex];
  dom.runStatus.value = state.running ? "Running" : state.stepIndex >= state.steps.length && state.steps.length ? "Complete" : "Ready";
  dom.stepStatus.value = step ? step.label : "Idle";

  dom.detectionRows.innerHTML = "";
  if (!detections.length) {
    const row = document.createElement("tr");
    row.innerHTML = `<td class="empty-row" colspan="4">No objects detected</td>`;
    dom.detectionRows.append(row);
  } else {
    for (const key of ORDER) {
      const detection = state.detections[key];
      if (!detection) {
        continue;
      }

      const type = BALL_TYPES[key];
      const row = document.createElement("tr");
      row.innerHTML = `
        <td><strong>${type.label}</strong><span>HSV ${Math.round(detection.averageHue)}deg, radius ${formatNumber(detection.radiusEstimate)}m</span></td>
        <td>${Math.round(detection.pixel.x)}, ${Math.round(detection.pixel.y)}</td>
        <td>${formatNumber(detection.world.x)}, ${formatNumber(detection.world.y)}, ${formatNumber(detection.world.z)}</td>
        <td><span class="bin-badge">${detection.bin}</span></td>
      `;
      dom.detectionRows.append(row);
    }
  }

  dom.eventLog.innerHTML = "";
  for (const item of state.logs.slice(0, 8)) {
    const li = document.createElement("li");
    li.innerHTML = `<strong>${item.title}</strong><span>${item.body}</span>`;
    dom.eventLog.append(li);
  }
}

function createScene() {
  const placed = [];
  const objects = {};

  for (const key of ORDER) {
    const type = BALL_TYPES[key];
    const world = randomSafePosition(type.radius, placed);
    placed.push({ ...world, radius: type.radius });
    objects[key] = {
      type: key,
      world,
      state: "waiting",
    };
  }

  return objects;
}

function randomSafePosition(radius, placed) {
  const area = GROUND_HALF - 0.08;

  for (let attempts = 0; attempts < 500; attempts += 1) {
    const x = randomBetween(-area, area);
    const y = randomBetween(-area, area);

    if (Math.hypot(x, y) < 0.18) {
      continue;
    }

    if (!positionIsClearOfTrays(x, y, radius)) {
      continue;
    }

    const overlap = placed.some((old) => Math.hypot(x - old.x, y - old.y) < radius + old.radius + 0.08);
    if (!overlap) {
      return { x, y };
    }
  }

  return { x: randomBetween(-0.25, 0.05), y: randomBetween(-0.28, 0.28) };
}

function positionIsClearOfTrays(x, y, radius) {
  return ORDER.every((key) => Math.hypot(x - BALL_TYPES[key].tray.x, y - BALL_TYPES[key].tray.y) > radius + 0.13);
}

function convertCoordinates(px, py, ballRadius) {
  const x = Math.max(0, Math.min(px, CAMERA_SIZE));
  const y = Math.max(0, Math.min(py, CAMERA_SIZE));
  const normX = x / CAMERA_SIZE;
  const normY = y / CAMERA_SIZE;
  return {
    x: WORLD_MIN_X + normX * (WORLD_MAX_X - WORLD_MIN_X),
    y: WORLD_MAX_Y - normY * (WORLD_MAX_Y - WORLD_MIN_Y),
    z: ballRadius,
  };
}

function worldToPixel(x, y) {
  return {
    x: ((x - WORLD_MIN_X) / (WORLD_MAX_X - WORLD_MIN_X)) * CAMERA_SIZE,
    y: ((WORLD_MAX_Y - y) / (WORLD_MAX_Y - WORLD_MIN_Y)) * CAMERA_SIZE,
  };
}

function worldToScreen(x, y) {
  const width = dom.world.width;
  const height = dom.world.height;
  const scale = screenScale();
  return {
    x: width / 2 + x * scale,
    y: height / 2 - y * scale,
  };
}

function screenScale() {
  return Math.min(dom.world.width, dom.world.height) * 0.98;
}

function solveIk(x, y) {
  const shoulderToTarget = Math.atan2(y, x);
  const distance = Math.min(Math.max(Math.hypot(x, y), 0.04), ARM_UPPER + ARM_LOWER - 0.01);
  const elbow = Math.acos(clamp((distance ** 2 - ARM_UPPER ** 2 - ARM_LOWER ** 2) / (2 * ARM_UPPER * ARM_LOWER), -1, 1));
  const shoulderOffset = Math.atan2(ARM_LOWER * Math.sin(elbow), ARM_UPPER + ARM_LOWER * Math.cos(elbow));
  return {
    shoulder: shoulderToTarget - shoulderOffset,
    elbow,
  };
}

function rgbToHsv(r, g, b) {
  const red = r / 255;
  const green = g / 255;
  const blue = b / 255;
  const max = Math.max(red, green, blue);
  const min = Math.min(red, green, blue);
  const delta = max - min;
  let h = 0;

  if (delta !== 0) {
    if (max === red) {
      h = 60 * (((green - blue) / delta) % 6);
    } else if (max === green) {
      h = 60 * ((blue - red) / delta + 2);
    } else {
      h = 60 * ((red - green) / delta + 4);
    }
  }

  if (h < 0) {
    h += 360;
  }

  return {
    h,
    s: max === 0 ? 0 : delta / max,
    v: max,
  };
}

function drawCanvasLabel(ctx, text, x, y, mode = "light") {
  ctx.save();
  ctx.font = "700 14px Inter, sans-serif";
  ctx.textBaseline = "middle";
  const width = Math.ceil(ctx.measureText(text).width) + 18;
  const labelX = Math.max(8, Math.min(x, ctx.canvas.width - width - 8));
  const labelY = Math.max(16, Math.min(y, ctx.canvas.height - 16));
  ctx.fillStyle = mode === "dark" ? "rgba(255,255,255,0.95)" : "rgba(16,20,22,0.88)";
  ctx.beginPath();
  ctx.roundRect(labelX, labelY - 15, width, 30, 7);
  ctx.fill();
  ctx.fillStyle = mode === "dark" ? "#172026" : "#ffffff";
  ctx.fillText(text, labelX + 9, labelY + 1);
  ctx.restore();
}

function drawImageCover(ctx, image, x, y, width, height) {
  const scale = Math.max(width / image.width, height / image.height);
  const drawWidth = image.width * scale;
  const drawHeight = image.height * scale;
  ctx.drawImage(image, x + (width - drawWidth) / 2, y + (height - drawHeight) / 2, drawWidth, drawHeight);
}

function loadImage(src) {
  const image = new Image();
  image.src = src;
  return image;
}

function makeLog(title, body) {
  return { title, body };
}

function toRadians(degrees) {
  return (degrees * Math.PI) / 180;
}

function randomBetween(min, max) {
  return min + Math.random() * (max - min);
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function lerp(start, end, amount) {
  return start + (end - start) * amount;
}

function easeInOut(t) {
  return (1 - Math.cos(Math.PI * clamp(t, 0, 1))) / 2;
}

function formatNumber(value) {
  return Number(value).toFixed(3);
}
