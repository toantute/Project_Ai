// =====================================================
// STATE MANAGEMENT
// =====================================================
const AppState = {
  mode: "single", // single | compare | triple
  editMode: "start",
  gridSize: 20,
  heuristicMode: "preset",
  customHeuristic: "Math.abs(x-goal_x)+Math.abs(y-goal_y)",
  hTable: {},
  animRunning: false,
  animPaused: false,
  speed: 5,
  panelCount: 1,
};

// Grid shared state
const Grid = {
  cells: [], // 2D array: 0=empty, 1=wall, N>1=weight
  startRow: 2,
  startCol: 2,
  goalRow: 17,
  goalCol: 17,
  size: 20,
};

// Per-algorithm state
const AlgoStates = [null, null, null];
const AlgoTimers = [null, null, null];
const AlgoTimes = [0, 0, 0];

// Canvas references
const canvases = [null, null, null];
const ctxs = [null, null, null];

// =====================================================
// NODE STRUCTURE
// =====================================================
class SearchNode {
  constructor(row, col, g = 0, h = 0, parent = null) {
    this.row = row;
    this.col = col;
    this.x = col;
    this.y = row;
    this.g = g; // cost from start
    this.h = h; // heuristic
    this.f = g + h; // total
    this.parent = parent;
  }
}

// =====================================================
// ALGORITHM LOGIC MODULE
// =====================================================
const Algorithms = {
  // Initialize algorithm state
  init(algoName, grid, startR, startC, goalR, goalC, heuristicFn) {
    const state = {
      name: algoName,
      grid,
      startR,
      startC,
      goalR,
      goalC,
      heuristicFn,
      open: [], // frontier
      closed: new Set(), // visited (as "r,c" strings)
      openSet: new Set(),
      parentMap: {},
      gMap: {},
      expandedCount: 0,
      pathCost: null,
      path: [],
      done: false,
      found: false,
      currentNode: null,
    };

    const startNode = new SearchNode(
      startR,
      startC,
      0,
      heuristicFn(startR, startC),
      null,
    );
    state.open.push(startNode);
    state.openSet.add(`${startR},${startC}`);
    state.gMap[`${startR},${startC}`] = 0;
    state.parentMap[`${startR},${startC}`] = null;
    return state;
  },

  // Single step
  step(state) {
    if (state.done || state.open.length === 0) {
      state.done = true;
      return null;
    }

    let node;
    switch (state.name) {
      case "bfs":
        node = this._bfsStep(state);
        break;
      case "dfs":
        node = this._dfsStep(state);
        break;
      case "ucs":
        node = this._ucsStep(state);
        break;
      case "gbfs":
        node = this._gbfsStep(state);
        break;
      case "astar":
        node = this._astarStep(state);
        break;
      default:
        node = this._astarStep(state);
    }

    if (!node) {
      state.done = true;
      return null;
    }

    state.currentNode = node;
    const key = `${node.row},${node.col}`;
    state.closed.add(key);
    state.openSet.delete(key);
    state.expandedCount++;

    // Check goal
    if (node.row === state.goalR && node.col === state.goalC) {
      state.done = true;
      state.found = true;
      state.path = this._reconstructPath(state, node);
      state.pathCost = node.g;
      return node;
    }

    // Expand neighbors
    const neighbors = this._getNeighbors(node, state);
    for (const nb of neighbors) {
      const nbKey = `${nb.row},${nb.col}`;
      if (state.closed.has(nbKey)) continue;

      const newG = node.g + this._moveCost(node, nb, state);
      const existing = state.gMap[nbKey];

      if (existing === undefined || newG < existing) {
        nb.g = newG;
        nb.h = state.heuristicFn(nb.row, nb.col);
        nb.f = nb.g + nb.h;
        nb.parent = node;
        state.gMap[nbKey] = newG;
        state.parentMap[nbKey] = node;

        if (!state.openSet.has(nbKey)) {
          state.open.push(nb);
          state.openSet.add(nbKey);
        }
      }
    }

    return node;
  },

  _bfsStep(state) {
    // FIFO
    return state.open.shift();
  },

  _dfsStep(state) {
    // LIFO
    return state.open.pop();
  },

  _ucsStep(state) {
    // Min g(n)
    let minIdx = 0;
    for (let i = 1; i < state.open.length; i++) {
      if (state.open[i].g < state.open[minIdx].g) minIdx = i;
    }
    return state.open.splice(minIdx, 1)[0];
  },

  _gbfsStep(state) {
    // Min h(n)
    let minIdx = 0;
    for (let i = 1; i < state.open.length; i++) {
      if (state.open[i].h < state.open[minIdx].h) minIdx = i;
    }
    return state.open.splice(minIdx, 1)[0];
  },

  _astarStep(state) {
    // Min f(n) = g + h
    let minIdx = 0;
    for (let i = 1; i < state.open.length; i++) {
      if (state.open[i].f < state.open[minIdx].f) minIdx = i;
    }
    return state.open.splice(minIdx, 1)[0];
  },

  _getNeighbors(node, state) {
    const dirs = [
      [-1, 0],
      [1, 0],
      [0, -1],
      [0, 1],
    ];
    const result = [];
    for (const [dr, dc] of dirs) {
      const r = node.row + dr,
        c = node.col + dc;
      if (r < 0 || r >= state.grid.size || c < 0 || c >= state.grid.size)
        continue;
      if (state.grid.cells[r][c] === 1) continue;
      result.push(new SearchNode(r, c));
    }
    return result;
  },

  _moveCost(from, to, state) {
    const w = state.grid.cells[to.row][to.col];
    return w > 1 ? w : 1;
  },

  _reconstructPath(state, goalNode) {
    const path = [];
    let key = `${goalNode.row},${goalNode.col}`;
    while (key !== null && state.parentMap[key] !== undefined) {
      const [r, c] = key.split(",").map(Number);
      path.unshift({ row: r, col: c });
      const parent = state.parentMap[key];
      key = parent ? `${parent.row},${parent.col}` : null;
    }
    return path;
  },
};

// =====================================================
// HEURISTIC MODULE
// =====================================================
const Heuristics = {
  get(goalR, goalC) {
    const mode = AppState.heuristicMode;
    if (mode === "preset") {
      return this.getPreset(
        document.getElementById("heuristicPreset").value,
        goalR,
        goalC,
      );
    } else if (mode === "custom") {
      return this.getCustom(AppState.customHeuristic, goalR, goalC);
    } else {
      return this.getTable(goalR, goalC);
    }
  },

  getPreset(type, goalR, goalC) {
    return (r, c) => {
      switch (type) {
        case "manhattan":
          return Math.abs(r - goalR) + Math.abs(c - goalC);
        case "euclidean":
          return Math.sqrt((r - goalR) ** 2 + (c - goalC) ** 2);
        case "chebyshev":
          return Math.max(Math.abs(r - goalR), Math.abs(c - goalC));
        case "zero":
          return 0;
        default:
          return Math.abs(r - goalR) + Math.abs(c - goalC);
      }
    };
  },

  getCustom(formula, goalR, goalC) {
    return (r, c) => {
      try {
        // Safe evaluation with limited scope
        const x = c,
          y = r,
          goal_x = goalC,
          goal_y = goalR;
        const safeFn = new Function(
          "x",
          "y",
          "goal_x",
          "goal_y",
          "Math",
          `"use strict"; return (${formula});`,
        );
        const val = safeFn(x, y, goal_x, goal_y, Math);
        return typeof val === "number" && isFinite(val) ? Math.max(0, val) : 0;
      } catch (e) {
        return 0;
      }
    };
  },

  getTable(goalR, goalC) {
    return (r, c) => {
      const key = `${r},${c}`;
      if (AppState.hTable[key] !== undefined) return AppState.hTable[key];
      return Math.abs(r - goalR) + Math.abs(c - goalC);
    };
  },
};

// =====================================================
// GRID INITIALIZATION
// =====================================================
function initGrid(size) {
  Grid.size = size;
  Grid.cells = Array.from({ length: size }, () => new Array(size).fill(0));
  Grid.startRow = Math.floor(size * 0.1);
  Grid.startCol = Math.floor(size * 0.1);
  Grid.goalRow = Math.floor(size * 0.85);
  Grid.goalCol = Math.floor(size * 0.85);
}

function resetGrid() {
  initGrid(AppState.gridSize);
  resetAll();
  drawAll();
}

function clearWalls() {
  for (let r = 0; r < Grid.size; r++)
    for (let c = 0; c < Grid.size; c++)
      if (Grid.cells[r][c] !== 0) Grid.cells[r][c] = 0;
  resetAll();
  drawAll();
}

function randomMaze() {
  clearWalls();
  const density = 0.3;
  for (let r = 0; r < Grid.size; r++) {
    for (let c = 0; c < Grid.size; c++) {
      if (
        (r === Grid.startRow && c === Grid.startCol) ||
        (r === Grid.goalRow && c === Grid.goalCol)
      )
        continue;
      if (Math.random() < density) Grid.cells[r][c] = 1;
    }
  }
  resetAll();
  drawAll();
}

// =====================================================
// RENDERING MODULE
// =====================================================
const CELL_PADDING = 1;

function setupCanvases() {
  for (let i = 0; i < 3; i++) {
    canvases[i] = document.getElementById(`canvas${i}`);
    ctxs[i] = canvases[i].getContext("2d");
    canvases[i].addEventListener("mousedown", (e) => onMouseDown(e, i));
    canvases[i].addEventListener("mousemove", (e) => onMouseMove(e, i));
    canvases[i].addEventListener("mouseup", () => (mouseDown = false));
    canvases[i].addEventListener("mouseleave", () => (mouseDown = false));
  }
}

function getCellSize(canvasIdx) {
  const wrapper = canvases[canvasIdx].parentElement;
  const wrapW = wrapper.clientWidth - 24;
  const wrapH = wrapper.clientHeight - 24;
  const cs = Math.floor(Math.min(wrapW, wrapH) / Grid.size);
  return Math.max(cs, 4);
}

function resizeCanvases() {
  for (let i = 0; i < 3; i++) {
    const cs = getCellSize(i);
    canvases[i].width = cs * Grid.size;
    canvases[i].height = cs * Grid.size;
  }
}

function getNodeColor(r, c, algoState) {
  if (r === Grid.startRow && c === Grid.startCol) return "#22c55e";
  if (r === Grid.goalRow && c === Grid.goalCol) return "#ef4444";
  if (Grid.cells[r][c] === 1) return "#1e293b";
  if (Grid.cells[r][c] > 1) {
    // Weight cell - orange tint scaled by weight
    const alpha = Math.min(0.8, Grid.cells[r][c] * 0.15);
    return `rgba(249,115,22,${alpha})`;
  }

  if (!algoState) return "#0f172a";

  const key = `${r},${c}`;

  // Path
  if (algoState.found) {
    for (const p of algoState.path) {
      if (p.row === r && p.col === c) return "#a855f7";
    }
  }

  // Current node
  if (
    algoState.currentNode &&
    algoState.currentNode.row === r &&
    algoState.currentNode.col === c
  )
    return "#60a5fa";

  if (algoState.closed.has(key)) return "#ca8a04";
  if (algoState.openSet.has(key)) return "#3b82f6";

  return "#0f172a";
}

function drawGrid(idx) {
  const canvas = canvases[idx];
  const ctx = ctxs[idx];
  const cs = getCellSize(idx);
  const state = AlgoStates[idx];

  ctx.clearRect(0, 0, canvas.width, canvas.height);

  for (let r = 0; r < Grid.size; r++) {
    for (let c = 0; c < Grid.size; c++) {
      const color = getNodeColor(r, c, state);
      ctx.fillStyle = color;
      ctx.fillRect(
        c * cs + CELL_PADDING,
        r * cs + CELL_PADDING,
        cs - CELL_PADDING * 2,
        cs - CELL_PADDING * 2,
      );

      // Draw weight number
      if (Grid.cells[r][c] > 1 && cs >= 14) {
        ctx.fillStyle = "rgba(249,115,22,0.9)";
        ctx.font = `bold ${Math.min(cs - 4, 11)}px JetBrains Mono`;
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(Grid.cells[r][c], c * cs + cs / 2, r * cs + cs / 2);
      }

      // Draw h value for informed search
      if (
        state &&
        cs >= 18 &&
        (state.name === "astar" || state.name === "gbfs")
      ) {
        if (state.openSet.has(`${r},${c}`) || state.closed.has(`${r},${c}`)) {
          const h = Math.round(state.heuristicFn(r, c));
          ctx.fillStyle = "rgba(255,255,255,0.5)";
          ctx.font = `${Math.min(cs - 8, 8)}px JetBrains Mono`;
          ctx.textAlign = "right";
          ctx.textBaseline = "bottom";
          ctx.fillText(h, c * cs + cs - 2, r * cs + cs - 1);
        }
      }
    }
  }

  // Draw grid lines
  ctx.strokeStyle = "rgba(255,255,255,0.04)";
  ctx.lineWidth = 0.5;
  for (let i = 0; i <= Grid.size; i++) {
    ctx.beginPath();
    ctx.moveTo(i * cs, 0);
    ctx.lineTo(i * cs, canvas.height);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(0, i * cs);
    ctx.lineTo(canvas.width, i * cs);
    ctx.stroke();
  }
}

function drawAll() {
  resizeCanvases();
  for (let i = 0; i < AppState.panelCount; i++) drawGrid(i);
}

// =====================================================
// MOUSE INTERACTION
// =====================================================
let mouseDown = false;
let lastEditCell = null;

function getCellFromEvent(e, idx) {
  const canvas = canvases[idx];
  const rect = canvas.getBoundingClientRect();
  const cs = getCellSize(idx);
  const x = e.clientX - rect.left;
  const y = e.clientY - rect.top;
  return { r: Math.floor(y / cs), c: Math.floor(x / cs) };
}

function onMouseDown(e, idx) {
  // Only allow editing on canvas 0 (master grid)
  if (idx !== 0) return;
  mouseDown = true;
  lastEditCell = null;
  const { r, c } = getCellFromEvent(e, 0);
  editCell(r, c);
}

function onMouseMove(e, idx) {
  if (!mouseDown || idx !== 0) return;
  const { r, c } = getCellFromEvent(e, 0);
  const key = `${r},${c}`;
  if (lastEditCell === key) return;
  lastEditCell = key;
  editCell(r, c);
}

function editCell(r, c) {
  if (r < 0 || r >= Grid.size || c < 0 || c >= Grid.size) return;
  const mode = AppState.editMode;

  if (mode === "start") {
    Grid.startRow = r;
    Grid.startCol = c;
    resetAll();
    drawAll();
  } else if (mode === "goal") {
    Grid.goalRow = r;
    Grid.goalCol = c;
    resetAll();
    drawAll();
  } else if (mode === "wall") {
    if (
      !(r === Grid.startRow && c === Grid.startCol) &&
      !(r === Grid.goalRow && c === Grid.goalCol)
    ) {
      Grid.cells[r][c] = 1;
      drawAll();
    }
  } else if (mode === "erase") {
    Grid.cells[r][c] = 0;
    drawAll();
  } else if (mode === "weight") {
    if (
      !(r === Grid.startRow && c === Grid.startCol) &&
      !(r === Grid.goalRow && c === Grid.goalCol) &&
      Grid.cells[r][c] !== 1
    ) {
      Grid.cells[r][c] =
        Grid.cells[r][c] >= 5 ? 0 : (Grid.cells[r][c] || 0) + 1;
      drawAll();
    }
  }
}

// =====================================================
// ANIMATION CONTROL
// =====================================================
function initAlgoState(idx) {
  const algoName = ["algo1", "algo2", "algo3"][idx];
  const algoEl = document.getElementById(algoName);
  const name = algoEl ? algoEl.value : "astar";
  const hFn = Heuristics.get(Grid.goalRow, Grid.goalCol);
  AlgoStates[idx] = Algorithms.init(
    name,
    Grid,
    Grid.startRow,
    Grid.startCol,
    Grid.goalRow,
    Grid.goalCol,
    hFn,
  );
  AlgoTimes[idx] = performance.now();
}

function startAll() {
  for (let i = 0; i < AppState.panelCount; i++) initAlgoState(i);
  AppState.animRunning = true;
  AppState.animPaused = false;
  updateControlBtns();
  for (let i = 0; i < AppState.panelCount; i++) scheduleStep(i);
}

function pauseAll() {
  AppState.animPaused = true;
  for (let i = 0; i < 3; i++) {
    if (AlgoTimers[i]) {
      clearTimeout(AlgoTimers[i]);
      AlgoTimers[i] = null;
    }
  }
  updateControlBtns();
}

function resumeAll() {
  AppState.animPaused = false;
  for (let i = 0; i < AppState.panelCount; i++) {
    if (AlgoStates[i] && !AlgoStates[i].done) scheduleStep(i);
  }
  updateControlBtns();
}

function stepAll() {
  if (!AlgoStates[0]) {
    for (let i = 0; i < AppState.panelCount; i++) initAlgoState(i);
  }
  for (let i = 0; i < AppState.panelCount; i++) {
    if (AlgoStates[i] && !AlgoStates[i].done) {
      Algorithms.step(AlgoStates[i]);
      updateStats(i);
    }
  }
  drawAll();
  updateCompare();
}

function resetAll() {
  AppState.animRunning = false;
  AppState.animPaused = false;
  for (let i = 0; i < 3; i++) {
    if (AlgoTimers[i]) {
      clearTimeout(AlgoTimers[i]);
      AlgoTimers[i] = null;
    }
    AlgoStates[i] = null;
    AlgoTimes[i] = 0;
    updateStatsReset(i);
  }
  updateControlBtns();
  drawAll();
  document.getElementById("winnerBadge").textContent = "—";
}

function scheduleStep(idx) {
  if (AppState.animPaused) return;
  const state = AlgoStates[idx];
  if (!state || state.done) {
    checkAllDone();
    return;
  }
  const delay = Math.max(10, 300 / AppState.speed);
  AlgoTimers[idx] = setTimeout(() => {
    if (AppState.animPaused) return;
    Algorithms.step(state);
    drawGrid(idx);
    updateStats(idx);
    updateCompare();
    scheduleStep(idx);
  }, delay);
}

function checkAllDone() {
  const allDone = Array.from(
    { length: AppState.panelCount },
    (_, i) => i,
  ).every((i) => !AlgoStates[i] || AlgoStates[i].done);
  if (allDone && AppState.animRunning) {
    AppState.animRunning = false;
    updateControlBtns();
    updateCompare();
    determineWinner();
  }
}

function determineWinner() {
  if (AppState.panelCount < 2) return;
  const s0 = AlgoStates[0],
    s1 = AlgoStates[1];
  if (!s0 || !s1) return;
  const names = [
    document.getElementById("algo1").value.toUpperCase(),
    document.getElementById("algo2").value.toUpperCase(),
  ];
  if (s0.found && s1.found) {
    if (s0.pathCost < s1.pathCost)
      document.getElementById("winnerBadge").textContent =
        `${names[0]} wins (cost)`;
    else if (s1.pathCost < s0.pathCost)
      document.getElementById("winnerBadge").textContent =
        `${names[1]} wins (cost)`;
    else if (s0.expandedCount <= s1.expandedCount)
      document.getElementById("winnerBadge").textContent =
        `${names[0]} wins (efficiency)`;
    else
      document.getElementById("winnerBadge").textContent =
        `${names[1]} wins (efficiency)`;
  } else if (s0.found)
    document.getElementById("winnerBadge").textContent =
      `${names[0]} found path`;
  else if (s1.found)
    document.getElementById("winnerBadge").textContent =
      `${names[1]} found path`;
  else document.getElementById("winnerBadge").textContent = "No path found";
}

function updateControlBtns() {
  document.getElementById("btnStart").disabled = AppState.animRunning;
  document.getElementById("btnPause").disabled =
    !AppState.animRunning || AppState.animPaused;
  document.getElementById("btnResume").disabled = !AppState.animPaused;
}

function updateStats(idx) {
  const s = AlgoStates[idx];
  if (!s) return;
  document.getElementById(`expanded${idx}`).textContent = s.expandedCount;
  document.getElementById(`cost${idx}`).textContent = s.found
    ? s.pathCost.toFixed(1)
    : s.done
      ? "✗"
      : "—";
  document.getElementById(`steps${idx}`).textContent = s.expandedCount;

  const elapsed = ((performance.now() - AlgoTimes[idx]) / 1000).toFixed(2);
  const statusEl = document.getElementById(`status${idx}`);
  if (s.done && s.found) {
    statusEl.textContent = `FOUND (${elapsed}s)`;
    statusEl.className = "status-msg status-done";
  } else if (s.done && !s.found) {
    statusEl.textContent = "NO PATH";
    statusEl.className = "status-msg status-no-path";
  } else {
    statusEl.textContent = "RUNNING";
    statusEl.className = "status-msg status-running";
  }
}

function updateStatsReset(idx) {
  document.getElementById(`expanded${idx}`).textContent = "0";
  document.getElementById(`cost${idx}`).textContent = "—";
  document.getElementById(`steps${idx}`).textContent = "0";
  const statusEl = document.getElementById(`status${idx}`);
  statusEl.textContent = "READY";
  statusEl.className = "status-msg status-none";
}

function updateCompare() {
  if (AppState.panelCount < 2) return;
  for (let i = 0; i < 2; i++) {
    const s = AlgoStates[i];
    if (!s) continue;
    document.getElementById(`cExpanded${i}`).textContent = s.expandedCount;
    document.getElementById(`cCost${i}`).textContent = s.found
      ? s.pathCost.toFixed(1)
      : s.done
        ? "✗"
        : "—";
    const elapsed = AlgoTimes[i]
      ? (performance.now() - AlgoTimes[i]).toFixed(0)
      : "—";
    document.getElementById(`cTime${i}`).textContent = s.done ? elapsed : "...";
    const isOptimal =
      s.name === "astar" || s.name === "ucs" || s.name === "bfs";
    document.getElementById(`cOptimal${i}`).textContent = s.found
      ? isOptimal
        ? "✓"
        : "~"
      : "—";
  }
}

// =====================================================
// UI CONTROLS
// =====================================================
function setMode(mode) {
  AppState.mode = mode;
  resetAll();

  // Update tab buttons
  document
    .querySelectorAll(".tab-btn")
    .forEach((b) => b.classList.remove("active"));
  event.target.classList.add("active");

  // Show/hide panels
  const p1 = document.getElementById("panel1");
  const p2 = document.getElementById("panel2");
  const a2row = document.getElementById("algo2row");
  const a3row = document.getElementById("algo3row");
  const compBar = document.getElementById("compBar");

  if (mode === "single") {
    p1.style.display = "none";
    p2.style.display = "none";
    a2row.style.display = "none";
    a3row.style.display = "none";
    compBar.style.display = "none";
    AppState.panelCount = 1;
  } else if (mode === "compare") {
    p1.style.display = "flex";
    p2.style.display = "none";
    a2row.style.display = "block";
    a3row.style.display = "none";
    compBar.style.display = "grid";
    AppState.panelCount = 2;
  } else {
    p1.style.display = "flex";
    p2.style.display = "flex";
    a2row.style.display = "block";
    a3row.style.display = "block";
    compBar.style.display = "none";
    AppState.panelCount = 3;
  }

  setTimeout(drawAll, 50);
}

function setEditMode(mode) {
  AppState.editMode = mode;
  document
    .querySelectorAll(".mode-btn")
    .forEach((b) => b.classList.remove("active"));
  document
    .getElementById("mode" + mode.charAt(0).toUpperCase() + mode.slice(1))
    ?.classList.add("active");
}

function onGridSizeChange(val) {
  AppState.gridSize = parseInt(val);
  document.getElementById("sizeVal").textContent = val;
  document.getElementById("sizeVal2").textContent = val;
  initGrid(AppState.gridSize);
  resetAll();
  drawAll();
}

function updateAlgoBadge(idx) {
  const names = {
    bfs: "BFS",
    dfs: "DFS",
    ucs: "UCS",
    gbfs: "GBFS",
    astar: "A*",
  };
  const classes = {
    bfs: "badge-bfs",
    dfs: "badge-dfs",
    ucs: "badge-ucs",
    gbfs: "badge-gbfs",
    astar: "badge-astar",
  };
  const algoEl = document.getElementById(["algo1", "algo2", "algo3"][idx]);
  if (!algoEl) return;
  const val = algoEl.value;
  const badge = document.getElementById(`badge${idx}`);
  badge.textContent = names[val];
  badge.className = "algo-badge " + classes[val];
}

function setHeuristicMode(mode, btn) {
  AppState.heuristicMode = mode;
  document
    .querySelectorAll(".htab")
    .forEach((b) => b.classList.remove("active"));
  btn.classList.add("active");
  document.getElementById("hPreset").style.display =
    mode === "preset" ? "block" : "none";
  document.getElementById("hCustom").style.display =
    mode === "custom" ? "block" : "none";
  document.getElementById("hTable").style.display =
    mode === "table" ? "block" : "none";
}

function testHeuristic() {
  const formula = document.getElementById("customHeuristic").value;
  AppState.customHeuristic = formula;
  try {
    const fn = Heuristics.getCustom(formula, 5, 5);
    const val = fn(0, 0);
    showToast(`✓ h(0,0) = ${val.toFixed(2)}`);
  } catch (e) {
    showToast(`✗ Error: ${e.message}`);
  }
}

function parseHTable() {
  const text = document.getElementById("hTableInput").value;
  const table = {};
  for (const line of text.split("\n")) {
    const m = line.trim().match(/^(\d+),(\d+)=(\d+\.?\d*)$/);
    if (m) table[`${m[1]},${m[2]}`] = parseFloat(m[3]);
  }
  AppState.hTable = table;
  showToast(`✓ Loaded ${Object.keys(table).length} heuristic values`);
}

document.getElementById("speedSlider").addEventListener("input", function () {
  AppState.speed = parseInt(this.value);
  document.getElementById("speedVal").textContent = this.value;
});

document
  .getElementById("customHeuristic")
  .addEventListener("change", function () {
    AppState.customHeuristic = this.value;
  });

// =====================================================
// CHART
// =====================================================
let chartInstance = null;

function showChart() {
  document.getElementById("chartOverlay").classList.add("show");
  setTimeout(renderChart, 100);
}

function closeChart(e) {
  if (!e || e.target === document.getElementById("chartOverlay")) {
    document.getElementById("chartOverlay").classList.remove("show");
  }
}

function renderChart() {
  const canvas = document.getElementById("chartCanvas");
  const ctx = canvas.getContext("2d");
  canvas.width = canvas.parentElement.clientWidth;
  canvas.height = 250;

  const labels = [];
  const data = [];
  const colors = ["#22d3ee", "#f87171", "#fbbf24"];

  for (let i = 0; i < AppState.panelCount; i++) {
    const s = AlgoStates[i];
    const algoEl = document.getElementById(["algo1", "algo2", "algo3"][i]);
    labels.push(algoEl ? algoEl.value.toUpperCase() : `Algo ${i + 1}`);
    data.push(s ? s.expandedCount : 0);
  }

  const maxVal = Math.max(...data, 1);
  const barW = 80,
    gap = 60;
  const totalW = (barW + gap) * labels.length;
  const startX = (canvas.width - totalW + gap) / 2;
  const maxH = 180;

  ctx.clearRect(0, 0, canvas.width, canvas.height);

  for (let i = 0; i < labels.length; i++) {
    const x = startX + i * (barW + gap);
    const h = (data[i] / maxVal) * maxH;
    const y = 220 - h;

    // Bar
    const grad = ctx.createLinearGradient(x, y, x, 220);
    grad.addColorStop(0, colors[i % colors.length]);
    grad.addColorStop(1, colors[i % colors.length] + "44");
    ctx.fillStyle = grad;
    ctx.beginPath();
    ctx.roundRect(x, y, barW, h, 6);
    ctx.fill();

    // Label
    ctx.fillStyle = "#e2e8f0";
    ctx.font = "bold 12px JetBrains Mono";
    ctx.textAlign = "center";
    ctx.fillText(labels[i], x + barW / 2, 240);

    // Value
    ctx.fillStyle = colors[i % colors.length];
    ctx.font = "bold 13px JetBrains Mono";
    ctx.fillText(data[i], x + barW / 2, y - 6);
  }

  // Y axis label
  ctx.fillStyle = "#64748b";
  ctx.font = "10px JetBrains Mono";
  ctx.textAlign = "left";
  ctx.fillText("Nodes expanded", 10, 15);
}

// =====================================================
// EXPORT
// =====================================================
function exportResults() {
  const results = [];
  for (let i = 0; i < AppState.panelCount; i++) {
    const s = AlgoStates[i];
    const algoEl = document.getElementById(["algo1", "algo2", "algo3"][i]);
    results.push({
      algorithm: algoEl ? algoEl.value : `algo${i + 1}`,
      expandedNodes: s ? s.expandedCount : 0,
      pathCost: s ? s.pathCost : null,
      pathFound: s ? s.found : false,
      pathLength: s ? s.path.length : 0,
      path: s ? s.path : [],
      heuristicMode: AppState.heuristicMode,
      gridSize: Grid.size,
      timestamp: new Date().toISOString(),
    });
  }

  const blob = new Blob(
    [JSON.stringify({ results, grid: Grid.cells }, null, 2)],
    { type: "application/json" },
  );
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = "search-results.json";
  a.click();
  URL.revokeObjectURL(url);
  showToast("📁 Results exported!");
}

// =====================================================
// UTILITIES
// =====================================================
function showToast(msg) {
  const t = document.getElementById("toast");
  t.textContent = msg;
  t.classList.add("show");
  setTimeout(() => t.classList.remove("show"), 2500);
}

// Resize observer
const ro = new ResizeObserver(() => drawAll());
ro.observe(document.getElementById("gridsArea"));

// =====================================================
// INITIALIZATION
// =====================================================
window.addEventListener("DOMContentLoaded", () => {
  setupCanvases();
  initGrid(AppState.gridSize);
  updateAlgoBadge(0);
  updateAlgoBadge(1);

  // Set default mode
  setMode("compare");
  // reactivate compare tab
  document.querySelectorAll(".tab-btn")[1].classList.add("active");
  document.querySelectorAll(".tab-btn")[0].classList.remove("active");

  drawAll();
});

window.addEventListener("resize", () => setTimeout(drawAll, 100));
