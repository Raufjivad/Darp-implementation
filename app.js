const OBSTACLE = -2;
const UNASSIGNED = -1;

const colors = [
  "#3f7cac",
  "#e56b6f",
  "#59a14f",
  "#f28e2b",
  "#8f63b8",
  "#4e9f9a",
  "#b07d2d",
  "#d05f9f",
];

const state = {
  rows: 12,
  cols: 16,
  robotCount: 3,
  robots: [],
  obstacles: new Set(),
  assignment: null,
  quotas: [],
  counts: [],
  mode: "obstacle",
};

const canvas = document.querySelector("#gridCanvas");
const ctx = canvas.getContext("2d");
const rowsInput = document.querySelector("#rowsInput");
const colsInput = document.querySelector("#colsInput");
const robotsInput = document.querySelector("#robotsInput");
const statusEl = document.querySelector("#status");
const mapJson = document.querySelector("#mapJson");

const keyOf = (row, col) => `${row},${col}`;
const parseKey = (key) => key.split(",").map(Number);

function initialize() {
  applyPreset("open");
  bindEvents();
  exportMap();
  draw();
}

function bindEvents() {
  document.querySelector("#resizeButton").addEventListener("click", () => {
    state.rows = clamp(parseInt(rowsInput.value, 10), 2, 60);
    state.cols = clamp(parseInt(colsInput.value, 10), 2, 80);
    state.robotCount = clamp(parseInt(robotsInput.value, 10), 1, 8);
    rowsInput.value = state.rows;
    colsInput.value = state.cols;
    robotsInput.value = state.robotCount;
    resetMap();
  });

  document.querySelector("#presetSelect").addEventListener("change", (event) => {
    applyPreset(event.target.value);
  });

  document.querySelector("#runButton").addEventListener("click", runDivision);
  document.querySelector("#clearButton").addEventListener("click", resetMap);
  document.querySelector("#exportButton").addEventListener("click", exportMap);
  document.querySelector("#importButton").addEventListener("click", importMap);

  document.querySelectorAll(".mode").forEach((button) => {
    button.addEventListener("click", () => {
      state.mode = button.dataset.mode;
      document.querySelectorAll(".mode").forEach((candidate) => {
        candidate.classList.toggle("active", candidate === button);
      });
    });
  });

  canvas.addEventListener("pointerdown", editCell);
  canvas.addEventListener("pointermove", (event) => {
    if (event.buttons === 1 && state.mode !== "robot") {
      editCell(event);
    }
  });
}

function resetMap() {
  state.robots = defaultRobots(state.rows, state.cols, state.robotCount);
  state.obstacles = new Set();
  state.assignment = null;
  state.quotas = [];
  state.counts = [];
  setStatus("Ready");
  exportMap();
  draw();
}

function applyPreset(name) {
  if (name === "rooms") {
    state.rows = 14;
    state.cols = 18;
    state.robotCount = 4;
  } else if (name === "corridor") {
    state.rows = 12;
    state.cols = 22;
    state.robotCount = 3;
  } else if (name === "maze") {
    state.rows = 16;
    state.cols = 22;
    state.robotCount = 4;
  }

  rowsInput.value = state.rows;
  colsInput.value = state.cols;
  robotsInput.value = state.robotCount;
  state.robots = defaultRobots(state.rows, state.cols, state.robotCount);
  state.obstacles = new Set();

  if (name === "rooms") {
    addWallCol(8, 1, 12, [[3, 8], [10, 8]]);
    addWallRow(6, 1, 16, [[6, 4], [6, 13]]);
  } else if (name === "corridor") {
    addWallRow(3, 0, 20, [[3, 2], [3, 18]]);
    addWallRow(8, 2, 21, [[8, 4], [8, 16]]);
    addWallCol(10, 3, 8, [[5, 10]]);
  } else if (name === "maze") {
    for (let col = 2; col < state.cols - 2; col += 4) {
      addWallCol(col, 1, state.rows - 2, [[(col / 2) % 2 === 0 ? 3 : 12, col]]);
    }
    addWallRow(7, 1, state.cols - 2, [[7, 5], [7, 12], [7, 19]]);
  }

  state.assignment = null;
  state.quotas = [];
  state.counts = [];
  setStatus("Ready");
  exportMap();
  draw();
}

function addWallRow(row, startCol, endCol, gaps) {
  const gapSet = new Set(gaps.map(([r, c]) => keyOf(r, c)));
  for (let col = startCol; col <= endCol; col += 1) {
    if (!gapSet.has(keyOf(row, col))) state.obstacles.add(keyOf(row, col));
  }
}

function addWallCol(col, startRow, endRow, gaps) {
  const gapSet = new Set(gaps.map(([r, c]) => keyOf(r, c)));
  for (let row = startRow; row <= endRow; row += 1) {
    if (!gapSet.has(keyOf(row, col))) state.obstacles.add(keyOf(row, col));
  }
}

function defaultRobots(rows, cols, count) {
  const candidates = [
    [0, 0],
    [0, cols - 1],
    [rows - 1, Math.floor(cols / 2)],
    [rows - 1, 0],
    [rows - 1, cols - 1],
    [Math.floor(rows / 2), 0],
    [Math.floor(rows / 2), cols - 1],
    [Math.floor(rows / 2), Math.floor(cols / 2)],
  ];
  return candidates.slice(0, count);
}

function editCell(event) {
  const cell = eventToCell(event);
  if (!cell) return;
  const [row, col] = cell;
  const key = keyOf(row, col);

  if (state.mode === "robot") {
    if (state.obstacles.has(key)) return;
    const existing = state.robots.findIndex(([r, c]) => r === row && c === col);
    if (existing >= 0) {
      state.robots.splice(existing, 1);
    } else if (state.robots.length < 8) {
      state.robots.push([row, col]);
    }
    state.robotCount = state.robots.length;
    robotsInput.value = state.robotCount;
  } else if (state.mode === "obstacle") {
    if (state.robots.some(([r, c]) => r === row && c === col)) return;
    state.obstacles.add(key);
  } else {
    state.obstacles.delete(key);
  }

  state.assignment = null;
  state.quotas = [];
  state.counts = [];
  exportMap();
  draw();
}

function eventToCell(event) {
  const rect = canvas.getBoundingClientRect();
  const scale = Math.min(canvas.width / state.cols, canvas.height / state.rows);
  const width = scale * state.cols;
  const height = scale * state.rows;
  const originX = (canvas.width - width) / 2;
  const originY = (canvas.height - height) / 2;
  const x = (event.clientX - rect.left) * (canvas.width / rect.width) - originX;
  const y = (event.clientY - rect.top) * (canvas.height / rect.height) - originY;
  const col = Math.floor(x / scale);
  const row = Math.floor(y / scale);
  if (row < 0 || row >= state.rows || col < 0 || col >= state.cols) return null;
  return [row, col];
}

function runDivision() {
  try {
    const result = divideAreas(state.rows, state.cols, state.robots, obstacleCells());
    state.assignment = result.assignment;
    state.quotas = result.quotas;
    state.counts = result.counts;
    setStatus("Division complete");
  } catch (error) {
    state.assignment = null;
    state.quotas = [];
    state.counts = [];
    setStatus(error.message, true);
  }
  exportMap();
  draw();
}

function divideAreas(rows, cols, robots, obstacles) {
  validateDimensions(rows, cols);
  const obstacleSet = new Set(obstacles.map(([r, c]) => keyOf(r, c)));
  validateInputs(rows, cols, robots, obstacleSet);

  const passable = rows * cols - obstacleSet.size;
  const quotas = equalQuotas(passable, robots.length);
  const assignment = Array.from({ length: rows }, () => Array(cols).fill(UNASSIGNED));

  obstacleSet.forEach((key) => {
    const [row, col] = parseKey(key);
    assignment[row][col] = OBSTACLE;
  });
  robots.forEach(([row, col], robotId) => {
    assignment[row][col] = robotId;
  });

  validateFreeSpaceConnected(rows, cols, obstacleSet, robots[0]);
  const distances = robots.map((start) => bfsDistances(rows, cols, obstacleSet, start));

  const counts = robots.map(() => 1);
  const frontier = [];
  robots.forEach((start, robotId) => {
    pushUnassignedNeighbors(frontier, rows, cols, assignment, distances[robotId], robotId, start);
  });

  let remaining = passable - robots.length;
  while (remaining > 0 && frontier.length > 0) {
    frontier.sort(compareFrontier);
    const [distance, row, col, robotId] = frontier.shift();
    if (assignment[row][col] !== UNASSIGNED) continue;
    if (counts[robotId] >= quotas[robotId]) continue;
    if (!Number.isFinite(distance)) continue;
    assignment[row][col] = robotId;
    counts[robotId] += 1;
    remaining -= 1;
    pushUnassignedNeighbors(frontier, rows, cols, assignment, distances[robotId], robotId, [row, col]);
  }

  if (remaining > 0) throw new Error("Could not build connected areas that satisfy all quotas");
  if (counts.some((count, index) => count !== quotas[index])) throw new Error("Quota mismatch");
  robots.forEach((robot, robotId) => {
    if (!robotAreaConnected(assignment, robotId, robot)) {
      throw new Error(`Robot ${robotId + 1} area is disconnected`);
    }
  });

  return { assignment, quotas, counts };
}

function compareFrontier(a, b) {
  return a[0] - b[0] || a[1] - b[1] || a[2] - b[2] || a[3] - b[3] || a[4] - b[4];
}

function equalQuotas(totalPassableCells, robotCount) {
  if (robotCount <= 0) throw new Error("Add at least one robot");
  if (totalPassableCells < robotCount) throw new Error("There must be at least one passable cell per robot");
  const base = Math.floor(totalPassableCells / robotCount);
  const extra = totalPassableCells % robotCount;
  return Array.from({ length: robotCount }, (_, robotId) => base + (robotId < extra ? 1 : 0));
}

function bfsDistances(rows, cols, obstacles, start) {
  const distances = Array.from({ length: rows }, () => Array(cols).fill(Infinity));
  distances[start[0]][start[1]] = 0;
  const queue = [start];
  for (let index = 0; index < queue.length; index += 1) {
    const [row, col] = queue[index];
    neighbors(rows, cols, row, col).forEach(([nextRow, nextCol]) => {
      if (obstacles.has(keyOf(nextRow, nextCol))) return;
      if (Number.isFinite(distances[nextRow][nextCol])) return;
      distances[nextRow][nextCol] = distances[row][col] + 1;
      queue.push([nextRow, nextCol]);
    });
  }
  return distances;
}

function pushUnassignedNeighbors(frontier, rows, cols, assignment, distances, robotId, cell) {
  neighbors(rows, cols, cell[0], cell[1]).forEach(([row, col]) => {
    if (assignment[row][col] === UNASSIGNED) {
      frontier.push([distances[row][col], row, col, robotId, frontier.length]);
    }
  });
}

function validateFreeSpaceConnected(rows, cols, obstacles, start) {
  const seen = reachableCells(rows, cols, obstacles, start);
  const passable = rows * cols - obstacles.size;
  if (seen.size !== passable) throw new Error("Passable grid cells must form one connected component");
}

function robotAreaConnected(assignment, robotId, start) {
  const cells = new Set();
  for (let row = 0; row < assignment.length; row += 1) {
    for (let col = 0; col < assignment[row].length; col += 1) {
      if (assignment[row][col] === robotId) cells.add(keyOf(row, col));
    }
  }
  if (cells.size === 0) return false;
  return sameSet(reachableCells(assignment.length, assignment[0].length, new Set(), start, cells), cells);
}

function reachableCells(rows, cols, obstacles, start, allowed = null) {
  const seen = new Set([keyOf(start[0], start[1])]);
  const queue = [start];
  for (let index = 0; index < queue.length; index += 1) {
    const [row, col] = queue[index];
    neighbors(rows, cols, row, col).forEach((nextCell) => {
      const key = keyOf(nextCell[0], nextCell[1]);
      if (seen.has(key) || obstacles.has(key)) return;
      if (allowed && !allowed.has(key)) return;
      seen.add(key);
      queue.push(nextCell);
    });
  }
  return seen;
}

function neighbors(rows, cols, row, col) {
  return [
    [row - 1, col],
    [row + 1, col],
    [row, col - 1],
    [row, col + 1],
  ].filter(([nextRow, nextCol]) => nextRow >= 0 && nextRow < rows && nextCol >= 0 && nextCol < cols);
}

function validateDimensions(rows, cols) {
  if (rows <= 0 || cols <= 0) throw new Error("Grid dimensions must be positive");
}

function validateInputs(rows, cols, robots, obstacles) {
  if (!robots.length) throw new Error("Add at least one robot");
  const seen = new Set();
  robots.forEach((robot) => {
    validateCell(rows, cols, robot, "Robot");
    const key = keyOf(robot[0], robot[1]);
    if (seen.has(key)) throw new Error(`Duplicate robot position: ${key}`);
    if (obstacles.has(key)) throw new Error(`Robot starts on an obstacle: ${key}`);
    seen.add(key);
  });
  obstacles.forEach((key) => validateCell(rows, cols, parseKey(key), "Obstacle"));
}

function validateCell(rows, cols, cell, name) {
  const [row, col] = cell;
  if (row < 0 || row >= rows || col < 0 || col >= cols) {
    throw new Error(`${name} cell outside grid: ${row},${col}`);
  }
}

function obstacleCells() {
  return Array.from(state.obstacles).map(parseKey);
}

function exportMap() {
  mapJson.value = JSON.stringify(
    {
      rows: state.rows,
      cols: state.cols,
      robots: state.robots,
      obstacles: obstacleCells(),
    },
    null,
    2,
  );
  updateStats();
}

function importMap() {
  try {
    const payload = JSON.parse(mapJson.value);
    state.rows = clamp(Number(payload.rows), 2, 60);
    state.cols = clamp(Number(payload.cols), 2, 80);
    state.robots = Array.isArray(payload.robots) ? payload.robots.map(normalizeCell) : [];
    state.obstacles = new Set((Array.isArray(payload.obstacles) ? payload.obstacles : []).map(normalizeCell).map(([r, c]) => keyOf(r, c)));
    state.robotCount = state.robots.length;
    rowsInput.value = state.rows;
    colsInput.value = state.cols;
    robotsInput.value = state.robotCount;
    state.assignment = null;
    state.quotas = [];
    state.counts = [];
    validateInputs(state.rows, state.cols, state.robots, state.obstacles);
    setStatus("Map imported");
    exportMap();
    draw();
  } catch (error) {
    setStatus(error.message, true);
  }
}

function normalizeCell(value) {
  if (!Array.isArray(value) || value.length !== 2) throw new Error("Cells must be [row, column]");
  return [Number(value[0]), Number(value[1])];
}

function draw() {
  const ratio = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.max(320, Math.floor(rect.width * ratio));
  canvas.height = Math.max(320, Math.floor(rect.height * ratio));
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  const width = canvas.width / ratio;
  const height = canvas.height / ratio;
  ctx.clearRect(0, 0, width, height);

  const scale = Math.floor(Math.min(width / state.cols, height / state.rows));
  const cell = Math.max(8, scale);
  const boardWidth = cell * state.cols;
  const boardHeight = cell * state.rows;
  const originX = Math.floor((width - boardWidth) / 2);
  const originY = Math.floor((height - boardHeight) / 2);

  for (let row = 0; row < state.rows; row += 1) {
    for (let col = 0; col < state.cols; col += 1) {
      drawCell(originX, originY, cell, row, col);
    }
  }

  drawRobots(originX, originY, cell);
  updateStats();
}

function drawCell(originX, originY, cell, row, col) {
  const x = originX + col * cell;
  const y = originY + row * cell;
  const obstacle = state.obstacles.has(keyOf(row, col));
  let fill = "#ffffff";

  if (state.assignment && state.assignment[row][col] >= 0) {
    fill = tint(colors[state.assignment[row][col] % colors.length], 0.24);
  }
  if (obstacle) fill = "#242a30";

  ctx.fillStyle = fill;
  ctx.fillRect(x, y, cell, cell);
  ctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue("--cell-line").trim();
  ctx.lineWidth = 1;
  ctx.strokeRect(x + 0.5, y + 0.5, cell, cell);
}

function drawRobots(originX, originY, cell) {
  state.robots.forEach(([row, col], robotId) => {
    const x = originX + col * cell + cell / 2;
    const y = originY + row * cell + cell / 2;
    const radius = Math.max(6, cell * 0.32);
    ctx.beginPath();
    ctx.fillStyle = colors[robotId % colors.length];
    ctx.arc(x, y, radius, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = "#fff";
    ctx.font = `700 ${Math.max(10, Math.floor(cell * 0.36))}px system-ui`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(String(robotId + 1), x, y);
  });
}

function updateStats() {
  document.querySelector("#passableStat").textContent = String(state.rows * state.cols - state.obstacles.size);
  document.querySelector("#obstacleStat").textContent = String(state.obstacles.size);
  document.querySelector("#robotStat").textContent = String(state.robots.length);
  const robotStats = document.querySelector("#robotStats");
  robotStats.innerHTML = "";
  state.robots.forEach((_, robotId) => {
    const row = document.createElement("div");
    row.className = "robot-row";
    row.innerHTML = `
      <span class="swatch" style="background:${colors[robotId % colors.length]}"></span>
      <strong>Robot ${robotId + 1}</strong>
      <span>${state.counts[robotId] ?? "-"} / ${state.quotas[robotId] ?? "-"}</span>
    `;
    robotStats.appendChild(row);
  });
}

function tint(hex, amount) {
  const parsed = hex.replace("#", "");
  const red = parseInt(parsed.slice(0, 2), 16);
  const green = parseInt(parsed.slice(2, 4), 16);
  const blue = parseInt(parsed.slice(4, 6), 16);
  const mix = (channel) => Math.round(channel + (255 - channel) * amount);
  return `rgb(${mix(red)}, ${mix(green)}, ${mix(blue)})`;
}

function sameSet(left, right) {
  if (left.size !== right.size) return false;
  for (const value of left) {
    if (!right.has(value)) return false;
  }
  return true;
}

function setStatus(message, error = false) {
  statusEl.textContent = message;
  statusEl.classList.toggle("error", error);
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, Number.isFinite(value) ? value : min));
}

window.addEventListener("resize", draw);
initialize();

