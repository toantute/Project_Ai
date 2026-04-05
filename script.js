// ═══════════════════════════════════════
//  CONSTANTS & STATE
// ═══════════════════════════════════════
const dx = [-1, 0, 1, 0];
const dy = [0, 1, 0, -1];

const TCOLORS = ["", "#BBF7D0", "#FEF08A", "#FED7AA", "#FECACA"];
const TNAMES = ["", "T1 · Grass", "T2 · Sand", "T3 · Rock", "T4 · Swamp"];

const ALGO_DESC = {
  BFS: "Duyệt theo từng lớp (FIFO). Đảm bảo đường ngắn nhất theo số bước trên đồ thị không có trọng số.",
  DFS: "Đi sâu theo một nhánh trước khi quay lại. Nhanh nhưng không tối ưu, có thể đi vòng xa.",
  UCS: "Mở rộng node có chi phí g thấp nhất (như Dijkstra). Tối ưu về chi phí đường đi.",
  DLS: "DFS với giới hạn độ sâu cứng. Hữu ích khi biết trước độ sâu lời giải.",
  IDS: "Lặp DLS với giới hạn tăng dần. Tối ưu + tiết kiệm bộ nhớ như BFS.",
  GBFS: "Tham lam: chỉ dùng heuristic h, bỏ qua g. Nhanh nhưng không đảm bảo tối ưu.",
  Astar:
    "Kết hợp g + h (f = g + h). Tối ưu và hiệu quả khi h là admissible heuristic.",
};

// Colors per algorithm for compare badges
const CMP_COLORS = {
  BFS: "#4f6ef7",
  DFS: "#7c3aed",
  UCS: "#059669",
  DLS: "#d97706",
  IDS: "#dc2626",
  GBFS: "#0891b2",
  Astar: "#be185d",
};

let M = [],
  m = 0,
  n = 0;
let sx = 0,
  sy = 0,
  tx = 0,
  ty = 0;
let w = [0, 1, 2, 3, 4];
let tcnt = 3;
let drawMode = "wall";
let isDown = false;
let steps = [],
  stepIdx = 0;
let timer = null,
  running = false;
let expandedCount = 0;

// ─── Compare Mode State ───────────────
let compareMode = false;
let cmpSelected = new Set(["BFS", "DFS", "Astar"]);
let cmpState = {}; // algo -> { steps, stepIdx, expandedCount, done, result }
let cmpTimer = null;
let cmpRunning = false;

// ═══════════════════════════════════════
//  HEAP (Min)
// ═══════════════════════════════════════
class Heap {
  constructor(cmp) {
    this.h = [];
    this.cmp = cmp;
  }
  push(x) {
    this.h.push(x);
    this._up(this.h.length - 1);
  }
  pop() {
    const t = this.h[0],
      l = this.h.pop();
    if (this.h.length) {
      this.h[0] = l;
      this._dn(0);
    }
    return t;
  }
  peek() {
    return this.h[0];
  }
  empty() {
    return !this.h.length;
  }
  _up(i) {
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (this.cmp(this.h[i], this.h[p]) < 0) {
        [this.h[i], this.h[p]] = [this.h[p], this.h[i]];
        i = p;
      } else break;
    }
  }
  _dn(i) {
    const n = this.h.length;
    while (1) {
      let s = i,
        l = 2 * i + 1,
        r = 2 * i + 2;
      if (l < n && this.cmp(this.h[l], this.h[s]) < 0) s = l;
      if (r < n && this.cmp(this.h[r], this.h[s]) < 0) s = r;
      if (s !== i) {
        [this.h[i], this.h[s]] = [this.h[s], this.h[i]];
        i = s;
      } else break;
    }
  }
}

// ═══════════════════════════════════════
//  UTILITIES
// ═══════════════════════════════════════
function ok(x, y) {
  return x >= 0 && y >= 0 && x < m && y < n;
}
function cell(x, y) {
  return document.getElementById(`c_${x}_${y}`);
}
function A2(v) {
  return Array.from({ length: m }, () => new Array(n).fill(v));
}
function wmin() {
  return Math.min(...w.slice(1, tcnt + 1));
}

function heuristic(x, y) {
  const ht = parseInt(document.getElementById("htype").value);
  const mul = parseFloat(document.getElementById("hmul").value) || 1;
  const ddx = Math.abs(x - tx),
    ddy = Math.abs(y - ty);
  if (ht === 1) return Math.sqrt(ddx * ddx + ddy * ddy) * wmin() * mul;
  return (ddx + ddy) * wmin() * mul;
}

function traceParent(par) {
  const path = [];
  let cx = tx,
    cy = ty;
  while (!(cx === sx && cy === sy)) {
    path.unshift([cx, cy]);
    [cx, cy] = par[cx][cy];
  }
  path.unshift([sx, sy]);
  return path;
}

// ═══════════════════════════════════════
//  ALGORITHMS
// ═══════════════════════════════════════
function algoBFS() {
  const st = [],
    vis = A2(false),
    par = A2(null);
  const q = [{ x: sx, y: sy, d: 0, g: 0 }];
  vis[sx][sy] = true;
  par[sx][sy] = [sx, sy];
  while (q.length) {
    const c = q.shift();
    const { x, y } = c;
    st.push({ t: "E", x, y, g: c.g, d: c.d });
    if (x === tx && y === ty) {
      st.push({ t: "F", path: traceParent(par), g: c.g });
      return st;
    }
    for (let i = 0; i < 4; i++) {
      const nx = x + dx[i],
        ny = y + dy[i];
      if (ok(nx, ny) && M[nx][ny] && !vis[nx][ny]) {
        vis[nx][ny] = true;
        par[nx][ny] = [x, y];
        q.push({ x: nx, y: ny, d: c.d + 1, g: c.g + w[M[nx][ny]] });
        st.push({ t: "Fr", x: nx, y: ny });
      }
    }
  }
  st.push({ t: "N" });
  return st;
}

function algoDFS() {
  const st = [],
    vis = A2(false),
    par = A2(null);
  const stack = [{ x: sx, y: sy, d: 0, g: 0 }];
  vis[sx][sy] = true;
  par[sx][sy] = [sx, sy];
  while (stack.length) {
    const c = stack.pop();
    const { x, y } = c;
    st.push({ t: "E", x, y, g: c.g, d: c.d });
    if (x === tx && y === ty) {
      st.push({ t: "F", path: traceParent(par), g: c.g });
      return st;
    }
    for (let i = 0; i < 4; i++) {
      const nx = x + dx[i],
        ny = y + dy[i];
      if (ok(nx, ny) && M[nx][ny] && !vis[nx][ny]) {
        vis[nx][ny] = true;
        par[nx][ny] = [x, y];
        stack.push({ x: nx, y: ny, d: c.d + 1, g: c.g + w[M[nx][ny]] });
        st.push({ t: "Fr", x: nx, y: ny });
      }
    }
  }
  st.push({ t: "N" });
  return st;
}

function algoUCS() {
  const st = [],
    dis = A2(Infinity),
    par = A2(null);
  const pq = new Heap((a, b) => a.f - b.f);
  dis[sx][sy] = 0;
  par[sx][sy] = [sx, sy];
  pq.push({ x: sx, y: sy, d: 0, g: 0, f: 0 });
  while (!pq.empty()) {
    const c = pq.pop();
    const { x, y } = c;
    if (c.g > dis[x][y]) continue;
    st.push({ t: "E", x, y, g: c.g, d: c.d });
    if (x === tx && y === ty) {
      st.push({ t: "F", path: traceParent(par), g: c.g });
      return st;
    }
    for (let i = 0; i < 4; i++) {
      const nx = x + dx[i],
        ny = y + dy[i];
      if (ok(nx, ny) && M[nx][ny]) {
        const ng = c.g + w[M[nx][ny]];
        if (ng < dis[nx][ny]) {
          dis[nx][ny] = ng;
          par[nx][ny] = [x, y];
          pq.push({ x: nx, y: ny, d: c.d + 1, g: ng, f: ng });
          st.push({ t: "Fr", x: nx, y: ny });
        }
      }
    }
  }
  st.push({ t: "N" });
  return st;
}

function algoDLS(lim) {
  const st = [],
    dmin = A2(Infinity),
    par = A2(null);
  let reachedLim = false;
  dmin[sx][sy] = 0;
  par[sx][sy] = [sx, sy];
  const stack = [{ x: sx, y: sy, d: 0, g: 0 }];
  while (stack.length) {
    const c = stack.pop();
    const { x, y } = c;
    if (c.d === lim) reachedLim = true;
    st.push({ t: "E", x, y, g: c.g, d: c.d });
    if (x === tx && y === ty) {
      st.push({ t: "F", path: traceParent(par), g: c.g });
      return { st, found: true, reachedLim };
    }
    for (let i = 0; i < 4; i++) {
      const nx = x + dx[i],
        ny = y + dy[i],
        nd = c.d + 1;
      if (ok(nx, ny) && M[nx][ny] && nd <= lim && nd < dmin[nx][ny]) {
        dmin[nx][ny] = nd;
        par[nx][ny] = [x, y];
        stack.push({ x: nx, y: ny, d: nd, g: c.g + w[M[nx][ny]] });
        st.push({ t: "Fr", x: nx, y: ny });
      }
    }
  }
  st.push({ t: "N" });
  return { st, found: false, reachedLim };
}

function algoIDS() {
  const all = [],
    cap = m * n;
  for (let l = 0; l <= cap; l++) {
    all.push({ t: "I", l });
    const { st, found, reachedLim } = algoDLS(l);
    all.push(...st);
    if (found) return all;
    if (!reachedLim) break;
  }
  if (all[all.length - 1]?.t === "N") all.pop();
  all.push({ t: "N" });
  return all;
}

function algoGBFS() {
  const st = [],
    vis = A2(false),
    par = A2(null);
  const pq = new Heap((a, b) => a.f - b.f);
  vis[sx][sy] = true;
  par[sx][sy] = [sx, sy];
  pq.push({ x: sx, y: sy, d: 0, g: 0, f: heuristic(sx, sy) });
  while (!pq.empty()) {
    const c = pq.pop();
    const { x, y } = c;
    st.push({ t: "E", x, y, g: c.g, d: c.d });
    if (x === tx && y === ty) {
      st.push({ t: "F", path: traceParent(par), g: c.g });
      return st;
    }
    for (let i = 0; i < 4; i++) {
      const nx = x + dx[i],
        ny = y + dy[i];
      if (ok(nx, ny) && M[nx][ny] && !vis[nx][ny]) {
        vis[nx][ny] = true;
        par[nx][ny] = [x, y];
        pq.push({
          x: nx,
          y: ny,
          d: c.d + 1,
          g: c.g + w[M[nx][ny]],
          f: heuristic(nx, ny),
        });
        st.push({ t: "Fr", x: nx, y: ny });
      }
    }
  }
  st.push({ t: "N" });
  return st;
}

function algoAstar() {
  const st = [],
    dis = A2(Infinity),
    par = A2(null);
  const pq = new Heap((a, b) => a.f - b.f);
  dis[sx][sy] = 0;
  par[sx][sy] = [sx, sy];
  pq.push({ x: sx, y: sy, d: 0, g: 0, f: heuristic(sx, sy) });
  while (!pq.empty()) {
    const c = pq.pop();
    const { x, y } = c;
    if (c.g > dis[x][y]) continue;
    st.push({ t: "E", x, y, g: c.g, d: c.d });
    if (x === tx && y === ty) {
      st.push({ t: "F", path: traceParent(par), g: c.g });
      return st;
    }
    for (let i = 0; i < 4; i++) {
      const nx = x + dx[i],
        ny = y + dy[i];
      if (ok(nx, ny) && M[nx][ny]) {
        const ng = c.g + w[M[nx][ny]];
        if (ng < dis[nx][ny]) {
          dis[nx][ny] = ng;
          par[nx][ny] = [x, y];
          pq.push({
            x: nx,
            y: ny,
            d: c.d + 1,
            g: ng,
            f: ng + heuristic(nx, ny),
          });
          st.push({ t: "Fr", x: nx, y: ny });
        }
      }
    }
  }
  st.push({ t: "N" });
  return st;
}

// Dispatch by name (used by both single-run and compare)
function genStepsFor(algo) {
  document.querySelectorAll("#terrain-ui .tc-row input").forEach((inp, i) => {
    w[i + 1] = Math.max(1, parseInt(inp.value) || 1);
  });
  if (algo === "BFS") return algoBFS();
  if (algo === "DFS") return algoDFS();
  if (algo === "UCS") return algoUCS();
  if (algo === "DLS")
    return algoDLS(parseInt(document.getElementById("dls-lim").value) || 10).st;
  if (algo === "IDS") return algoIDS();
  if (algo === "GBFS") return algoGBFS();
  if (algo === "Astar") return algoAstar();
  return [];
}

function genSteps() {
  return genStepsFor(document.getElementById("algo").value);
}

// ═══════════════════════════════════════
//  GRID BUILD & RENDER
// ═══════════════════════════════════════
function buildGrid() {
  clearVis();
  m = Math.max(
    3,
    Math.min(30, parseInt(document.getElementById("rows").value) || 12),
  );
  n = Math.max(
    3,
    Math.min(40, parseInt(document.getElementById("cols").value) || 16),
  );
  tcnt = Math.max(
    1,
    Math.min(4, parseInt(document.getElementById("tcnt").value) || 3),
  );
  M = Array.from({ length: m }, () => new Array(n).fill(1));
  sx = 0;
  sy = 0;
  tx = m - 1;
  ty = n - 1;
  updateTerrainUI();
  updateDrawModes();
  renderGrid();
  resetStats();
  if (compareMode) buildCompareView();
}

function updateTerrainUI() {
  tcnt = Math.max(
    1,
    Math.min(4, parseInt(document.getElementById("tcnt").value) || 3),
  );
  const c = document.getElementById("terrain-ui");
  c.innerHTML = "";
  for (let i = 1; i <= tcnt; i++) {
    const row = document.createElement("div");
    row.className = "tc-row";
    row.innerHTML = `
      <div class="tc-swatch" style="background:${TCOLORS[i]}"></div>
      <label>T${i}</label>
      <input type="number" value="${w[i] || i}" min="1" max="999" style="width:64px;margin:0"
        onchange="w[${i}]=Math.max(1,parseInt(this.value)||1)">`;
    c.appendChild(row);
  }
  updateDrawModes();
}

function updateDrawModes() {
  const c = document.getElementById("draw-modes");
  c.innerHTML = "";
  const modes = [
    { id: "wall", label: "⬛ Wall" },
    { id: "erase", label: "◻ Erase" },
    { id: "start", label: "🟢 Start" },
    { id: "end", label: "🔴 End" },
  ];
  for (let i = 1; i <= tcnt; i++)
    modes.push({ id: `t${i}`, label: `${TNAMES[i].split(" ")[0]}` });
  modes.forEach((mo) => {
    const b = document.createElement("div");
    b.className = "dm-btn" + (drawMode === mo.id ? " active" : "");
    b.textContent = mo.label;
    b.onclick = () => setDrawMode(mo.id);
    c.appendChild(b);
  });
}

function setDrawMode(id) {
  drawMode = id;
  updateDrawModes();
}

function renderGrid() {
  const el = document.getElementById("grid");
  const avW = Math.floor((window.innerWidth * 0.52 - 32) / n);
  const avH = Math.floor((window.innerHeight * 0.73 - 32) / m);
  const cs = Math.max(18, Math.min(48, Math.min(avW, avH)));
  el.style.gridTemplateColumns = `repeat(${n}, ${cs}px)`;
  el.innerHTML = "";
  for (let i = 0; i < m; i++) {
    for (let j = 0; j < n; j++) {
      const d = document.createElement("div");
      d.className = "cell";
      d.style.width = d.style.height = cs + "px";
      d.id = `c_${i}_${j}`;
      d.dataset.x = i;
      d.dataset.y = j;
      d.innerHTML = `<div class="cell-ov"></div><div class="cell-mk"></div>`;
      d.addEventListener("mousedown", (e) => {
        isDown = true;
        paintCell(i, j);
        e.preventDefault();
      });
      d.addEventListener("mousemove", () => {
        if (isDown) paintCell(i, j);
      });
      el.appendChild(d);
      refreshCell(i, j);
    }
  }
  document.addEventListener("mouseup", () => (isDown = false));
}

function refreshCell(i, j) {
  const d = document.getElementById(`c_${i}_${j}`);
  if (!d) return;
  d.className = "cell";
  const t = M[i][j];
  if (t === 0) d.classList.add("wall");
  else d.classList.add(`t${t}`);
  const mk = d.querySelector(".cell-mk");
  if (i === sx && j === sy) mk.textContent = "🟢";
  else if (i === tx && j === ty) mk.textContent = "🔴";
  else mk.textContent = "";
}

function paintCell(i, j) {
  if (running) return;
  const prevSx = sx,
    prevSy = sy,
    prevTx = tx,
    prevTy = ty;
  if (drawMode === "wall") {
    if ((i === sx && j === sy) || (i === tx && j === ty)) return;
    M[i][j] = 0;
  } else if (drawMode === "erase") {
    M[i][j] = 1;
  } else if (drawMode === "start") {
    sx = i;
    sy = j;
    if (M[i][j] === 0) M[i][j] = 1;
    refreshCell(prevSx, prevSy);
  } else if (drawMode === "end") {
    tx = i;
    ty = j;
    if (M[i][j] === 0) M[i][j] = 1;
    refreshCell(prevTx, prevTy);
  } else if (drawMode.startsWith("t")) {
    const t = parseInt(drawMode.slice(1));
    if ((i === sx && j === sy) || (i === tx && j === ty)) return;
    M[i][j] = t;
  }
  refreshCell(i, j);
}

function genRandom() {
  clearVis();
  for (let i = 0; i < m; i++) {
    for (let j = 0; j < n; j++) {
      if ((i === sx && j === sy) || (i === tx && j === ty)) continue;
      const r = Math.random();
      M[i][j] = r < 0.22 ? 0 : Math.ceil(Math.random() * tcnt);
    }
  }
  let cx = sx,
    cy = sy;
  while (cx !== tx || cy !== ty) {
    if (cx < tx && Math.random() > 0.4) {
      if (!M[cx + 1][cy]) M[cx + 1][cy] = 1;
      cx++;
    } else if (cy < ty && Math.random() > 0.4) {
      if (!M[cx][cy + 1]) M[cx][cy + 1] = 1;
      cy++;
    } else if (cx < tx) {
      if (!M[cx + 1][cy]) M[cx + 1][cy] = 1;
      cx++;
    } else {
      if (!M[cx][cy + 1]) M[cx][cy + 1] = 1;
      cy++;
    }
  }
  renderGrid();
  if (compareMode) buildCompareView();
}

// ═══════════════════════════════════════
//  SINGLE-MODE PLAYBACK
// ═══════════════════════════════════════
function startRun() {
  clearVis();
  steps = genSteps();
  stepIdx = 0;
  running = true;
  document.getElementById("btn-run").disabled = true;
  document.getElementById("btn-step").disabled = true;
  document.getElementById("btn-pause").disabled = false;
  const isIDS = document.getElementById("algo").value === "IDS";
  document.getElementById("s-depth-card").style.display = isIDS
    ? "block"
    : "none";
  animateLoop();
}

function animateLoop() {
  if (stepIdx >= steps.length) {
    running = false;
    document.getElementById("btn-run").disabled = false;
    document.getElementById("btn-step").disabled = false;
    document.getElementById("btn-pause").disabled = true;
    return;
  }
  processStep(steps[stepIdx++]);
  updateProgressBar();
  timer = setTimeout(animateLoop, getDelay());
}

function getDelay() {
  const s = parseInt(document.getElementById("speed").value);
  return [700, 400, 240, 160, 100, 65, 38, 20, 8, 1][s - 1];
}

function stepRun() {
  if (!steps.length) {
    steps = genSteps();
    stepIdx = 0;
    const isIDS = document.getElementById("algo").value === "IDS";
    document.getElementById("s-depth-card").style.display = isIDS
      ? "block"
      : "none";
  }
  if (stepIdx >= steps.length) return;
  processStep(steps[stepIdx++]);
  updateProgressBar();
}

function pauseRun() {
  if (timer) clearTimeout(timer);
  timer = null;
  running = false;
  const btn = document.getElementById("btn-run");
  btn.textContent = "▶ Resume";
  btn.onclick = resumeRun;
  btn.disabled = false;
  document.getElementById("btn-pause").disabled = true;
}

function resumeRun() {
  running = true;
  const btn = document.getElementById("btn-run");
  btn.textContent = "▶ Run";
  btn.onclick = startRun;
  btn.disabled = true;
  document.getElementById("btn-pause").disabled = false;
  animateLoop();
}

function doReset() {
  if (timer) clearTimeout(timer);
  timer = null;
  running = false;
  const btn = document.getElementById("btn-run");
  btn.textContent = "▶ Chạy";
  btn.onclick = startRun;
  clearVis();
  renderGrid();
}

function clearVis() {
  if (timer) clearTimeout(timer);
  timer = null;
  running = false;
  steps = [];
  stepIdx = 0;
  expandedCount = 0;
  document.querySelectorAll(".cell").forEach((c) => {
    c.classList.remove("vis", "front", "onpath", "popping", "pathpop");
    const ov = c.querySelector(".cell-ov");
    if (ov) ov.style.animation = "";
  });
  document.getElementById("log").innerHTML = "";
  resetStats();
  document.getElementById("btn-run").disabled = false;
  document.getElementById("btn-step").disabled = false;
  document.getElementById("btn-pause").disabled = true;
}

function clearAll() {
  clearVis();
  M = Array.from({ length: m }, () => new Array(n).fill(1));
  renderGrid();
  if (compareMode) buildCompareView();
}

function resetStats() {
  document.getElementById("s-exp").textContent = "—";
  document.getElementById("s-cost").textContent = "—";
  document.getElementById("s-len").textContent = "—";
  document.getElementById("pbar").style.width = "0%";
  document.getElementById("pbar-lbl").textContent = "Step 0 / 0";
}

// ═══════════════════════════════════════
//  STEP PROCESSOR (single mode)
// ═══════════════════════════════════════
function processStep(step) {
  if (step.t === "E") {
    expandedCount++;
    const c = cell(step.x, step.y);
    if (c) {
      c.classList.remove("front");
      c.classList.add("vis", "popping");
      setTimeout(() => c.classList.remove("popping"), 380);
    }
    document.getElementById("s-exp").textContent = expandedCount;
    addLog("expand", `▸ (${step.x},${step.y})  g=${step.g}  d=${step.d}`);
  } else if (step.t === "Fr") {
    const c = cell(step.x, step.y);
    if (c && !c.classList.contains("vis")) c.classList.add("front");
  } else if (step.t === "F") {
    step.path.forEach(([x, y], idx) => {
      setTimeout(() => {
        const c = cell(x, y);
        if (!c) return;
        c.classList.remove("front", "vis", "popping");
        c.classList.add("onpath", "pathpop");
        setTimeout(() => c.classList.remove("pathpop"), 300);
      }, idx * 42);
    });
    document.getElementById("s-cost").textContent = step.g;
    document.getElementById("s-len").textContent = step.path.length - 1;
    addLog(
      "found",
      `✓ Tìm thấy! Cost=${step.g}  Steps=${step.path.length - 1}`,
    );
  } else if (step.t === "N") {
    addLog("nofound", "✗ Không tìm thấy đường đi");
  } else if (step.t === "I") {
    document.getElementById("s-depth").textContent = step.l;
    addLog("info", `── IDS: depth limit = ${step.l}`);
  }
}

function addLog(cls, msg) {
  const log = document.getElementById("log");
  const e = document.createElement("div");
  e.className = `le ${cls}`;
  e.textContent = msg;
  log.appendChild(e);
  log.scrollTop = log.scrollHeight;
}

function updateProgressBar() {
  const p = steps.length ? (stepIdx / steps.length) * 100 : 0;
  document.getElementById("pbar").style.width = p + "%";
  document.getElementById("pbar-lbl").textContent =
    `Step ${stepIdx} / ${steps.length}`;
}

// ═══════════════════════════════════════
//  COMPARE MODE
// ═══════════════════════════════════════
function toggleCompareMode() {
  compareMode = !compareMode;

  const mainView = document.getElementById("main-view");
  const cmpView = document.getElementById("compare-view");
  const cmpSelSec = document.getElementById("cmp-sel-sec");
  const algoSec = document.getElementById("algo-sec");
  const singleStats = document.getElementById("single-stats");
  const singleProg = document.getElementById("single-progress");
  const cmpProgSec = document.getElementById("cmp-progress-sec");
  const btn = document.getElementById("btn-compare-toggle");

  if (compareMode) {
    // Stop any running single animation
    if (timer) {
      clearTimeout(timer);
      timer = null;
      running = false;
    }

    mainView.style.display = "none";
    cmpView.style.display = "flex";
    cmpSelSec.style.display = "block";
    algoSec.style.display = "none";
    singleStats.style.display = "none";
    singleProg.style.display = "none";
    cmpProgSec.style.display = "block";
    btn.textContent = "✕ Thoát so sánh";
    btn.classList.add("active");

    buildCmpAlgoChecks();
    buildCompareView();
  } else {
    stopCompare();

    mainView.style.display = "";
    cmpView.style.display = "none";
    cmpSelSec.style.display = "none";
    algoSec.style.display = "";
    singleStats.style.display = "";
    singleProg.style.display = "";
    cmpProgSec.style.display = "none";
    btn.textContent = "⊞ So sánh";
    btn.classList.remove("active");
  }
}

function buildCmpAlgoChecks() {
  const c = document.getElementById("cmp-algo-checks");
  c.innerHTML = "";
  const algos = ["BFS", "DFS", "UCS", "DLS", "IDS", "GBFS", "Astar"];
  algos.forEach((algo) => {
    const label = document.createElement("label");
    label.className = "cmp-check-label";
    label.innerHTML = `
      <input type="checkbox" value="${algo}" ${cmpSelected.has(algo) ? "checked" : ""}
        onchange="toggleCmpAlgo('${algo}', this.checked)">
      <span class="cmp-check-badge" style="background:${CMP_COLORS[algo]}">${algo}</span>`;
    c.appendChild(label);
  });
}

function toggleCmpAlgo(algo, checked) {
  if (checked) cmpSelected.add(algo);
  else cmpSelected.delete(algo);
  buildCompareView();
}

// ── Build compare card grid ──
function buildCompareView() {
  stopCompare();
  const container = document.getElementById("cmp-grids");
  container.innerHTML = "";
  document.getElementById("cmp-results-table").innerHTML = "";
  document.getElementById("cmp-progress-list").innerHTML = "";

  if (cmpSelected.size === 0) return;

  [...cmpSelected].forEach((algo) => {
    const color = CMP_COLORS[algo];
    const card = document.createElement("div");
    card.className = "cmp-card";
    card.id = `cmp-card-${algo}`;
    card.innerHTML = `
      <div class="cmp-card-header" style="border-top: 3px solid ${color}">
        <span class="cmp-algo-badge" style="background:${color}">${algo}</span>
        <span class="cmp-card-status" id="cmp-status-${algo}">Ready</span>
      </div>
      <div class="cmp-grid-wrap" id="cmp-gwrap-${algo}">
        <div id="cmp-grid-${algo}" class="cmp-mini-grid"></div>
      </div>
      <div class="cmp-card-stats">
        <div class="cmp-mini-stat">
          <div class="cmp-mini-stat-lbl">Expanded</div>
          <div class="cmp-mini-stat-val" id="cmp-exp-${algo}">—</div>
        </div>
        <div class="cmp-mini-stat">
          <div class="cmp-mini-stat-lbl">Cost</div>
          <div class="cmp-mini-stat-val" id="cmp-cost-${algo}">—</div>
        </div>
        <div class="cmp-mini-stat">
          <div class="cmp-mini-stat-lbl">Steps</div>
          <div class="cmp-mini-stat-val" id="cmp-len-${algo}">—</div>
        </div>
      </div>`;
    container.appendChild(card);
    renderMiniGrid(algo);
  });

  // Build progress bars in right panel
  const pl = document.getElementById("cmp-progress-list");
  pl.innerHTML = "";
  [...cmpSelected].forEach((algo) => {
    const div = document.createElement("div");
    div.style.marginBottom = "8px";
    div.innerHTML = `
      <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:3px">
        <span class="cmp-algo-badge sm" style="background:${CMP_COLORS[algo]}">${algo}</span>
        <span style="font-size:0.6rem;color:var(--muted);font-family:'JetBrains Mono',monospace" id="cmp-plbl-${algo}">0 / 0</span>
      </div>
      <div class="pbar-bg"><div class="pbar-fill" id="cmp-pbar-${algo}" style="width:0%;background:${CMP_COLORS[algo]}"></div></div>`;
    pl.appendChild(div);
  });
}

// ── Render a mini grid for one algorithm ──
function renderMiniGrid(algo) {
  const el = document.getElementById(`cmp-grid-${algo}`);
  if (!el) return;

  const cnt = cmpSelected.size;
  // Estimate card width
  const cols = cnt <= 2 ? 2 : cnt <= 4 ? 2 : 3;
  const centerW = window.innerWidth - 265 - 220 - 40; // minus both panels
  const cardW = Math.floor(centerW / cols) - 20;
  const cs = Math.max(
    5,
    Math.min(18, Math.min(Math.floor((cardW - 10) / n), Math.floor(160 / m))),
  );

  el.style.gridTemplateColumns = `repeat(${n}, ${cs}px)`;
  el.style.display = "inline-grid";
  el.style.gap = "1px";
  el.innerHTML = "";

  const tColors = ["", "#bbf7d0", "#fef08a", "#fed7aa", "#fecaca"];

  for (let i = 0; i < m; i++) {
    for (let j = 0; j < n; j++) {
      const d = document.createElement("div");
      const t = M[i][j];
      d.style.cssText = `width:${cs}px;height:${cs}px;position:relative;border-radius:1px;flex-shrink:0;`;
      d.style.background = t === 0 ? "var(--wall)" : tColors[t] || "#bbf7d0";
      d.dataset.terrain = t;
      d.id = `${algo}_c_${i}_${j}`;

      if (i === sx && j === sy) {
        d.innerHTML = `<span style="position:absolute;inset:0;display:flex;align-items:center;justify-content:center;font-size:${Math.max(6, cs - 1)}px;line-height:1">🟢</span>`;
      } else if (i === tx && j === ty) {
        d.innerHTML = `<span style="position:absolute;inset:0;display:flex;align-items:center;justify-content:center;font-size:${Math.max(6, cs - 1)}px;line-height:1">🔴</span>`;
      }
      el.appendChild(d);
    }
  }
}

function cmpCell(algo, x, y) {
  return document.getElementById(`${algo}_c_${x}_${y}`);
}

// ── Run comparison ──
function runComparison() {
  if (cmpSelected.size === 0) return;
  stopCompare();
  buildCompareView(); // reset all grids

  cmpState = {};
  [...cmpSelected].forEach((algo) => {
    cmpState[algo] = {
      steps: genStepsFor(algo),
      stepIdx: 0,
      expandedCount: 0,
      done: false,
      result: null,
    };
    const el = document.getElementById(`cmp-status-${algo}`);
    if (el) {
      el.textContent = "Running…";
      el.className = "cmp-card-status running";
    }
  });

  cmpRunning = true;
  document.getElementById("btn-cmp-run").disabled = true;
  document.getElementById("btn-cmp-stop").disabled = false;
  cmpAnimateLoop();
}

function cmpAnimateLoop() {
  const allDone = Object.values(cmpState).every(
    (s) => s.done || s.stepIdx >= s.steps.length,
  );
  if (allDone) {
    // Mark any not explicitly found as done
    Object.entries(cmpState).forEach(([algo, state]) => {
      if (!state.done) {
        state.done = true;
        if (!state.result)
          state.result = {
            found: false,
            cost: "—",
            steps: "—",
            expanded: state.expandedCount,
          };
      }
    });
    cmpRunning = false;
    document.getElementById("btn-cmp-run").disabled = false;
    document.getElementById("btn-cmp-stop").disabled = true;
    showCmpResults();
    return;
  }

  Object.entries(cmpState).forEach(([algo, state]) => {
    if (state.done) return;
    if (state.stepIdx >= state.steps.length) {
      state.done = true;
      return;
    }
    const step = state.steps[state.stepIdx++];
    processCompareStep(algo, state, step);

    // Update progress bar
    const total = state.steps.length;
    const pct = total ? (state.stepIdx / total) * 100 : 0;
    const pb = document.getElementById(`cmp-pbar-${algo}`);
    const pl = document.getElementById(`cmp-plbl-${algo}`);
    if (pb) pb.style.width = pct + "%";
    if (pl) pl.textContent = `${state.stepIdx} / ${total}`;
  });

  cmpTimer = setTimeout(cmpAnimateLoop, getDelay());
}

function processCompareStep(algo, state, step) {
  if (step.t === "E") {
    state.expandedCount++;
    const c = cmpCell(algo, step.x, step.y);
    if (
      c &&
      !(step.x === sx && step.y === sy) &&
      !(step.x === tx && step.y === ty)
    ) {
      c.style.background = "rgba(79,110,247,0.6)";
      c.style.outline = "1px solid rgba(79,110,247,0.75)";
      c.dataset.state = "vis";
    }
    const expEl = document.getElementById(`cmp-exp-${algo}`);
    if (expEl) expEl.textContent = state.expandedCount;
  } else if (step.t === "Fr") {
    const c = cmpCell(algo, step.x, step.y);
    if (
      c &&
      c.dataset.state !== "vis" &&
      !(step.x === sx && step.y === sy) &&
      !(step.x === tx && step.y === ty)
    ) {
      c.style.background = "rgba(124,58,237,0.5)";
      c.style.outline = "1px solid rgba(124,58,237,0.65)";
      c.dataset.state = "front";
    }
  } else if (step.t === "F") {
    step.path.forEach(([x, y], idx) => {
      setTimeout(() => {
        const c = cmpCell(algo, x, y);
        if (c && !(x === sx && y === sy) && !(x === tx && y === ty)) {
          c.style.background = "rgba(234,179,8,0.85)";
          c.style.outline = "1.5px solid rgba(202,138,4,0.95)";
          c.dataset.state = "path";
        }
      }, idx * 25);
    });
    const costEl = document.getElementById(`cmp-cost-${algo}`);
    const lenEl = document.getElementById(`cmp-len-${algo}`);
    if (costEl) costEl.textContent = step.g;
    if (lenEl) lenEl.textContent = step.path.length - 1;
    const statusEl = document.getElementById(`cmp-status-${algo}`);
    if (statusEl) {
      statusEl.textContent = "✓ Found";
      statusEl.className = "cmp-card-status found";
    }
    state.done = true;
    state.result = {
      found: true,
      cost: step.g,
      steps: step.path.length - 1,
      expanded: state.expandedCount,
    };
  } else if (step.t === "N") {
    const statusEl = document.getElementById(`cmp-status-${algo}`);
    if (statusEl) {
      statusEl.textContent = "✗ No path";
      statusEl.className = "cmp-card-status nofound";
    }
    state.done = true;
    state.result = {
      found: false,
      cost: "—",
      steps: "—",
      expanded: state.expandedCount,
    };
  } else if (step.t === "I") {
    // IDS depth marker — just skip in compare mode
  }
}

function stopCompare() {
  if (cmpTimer) {
    clearTimeout(cmpTimer);
    cmpTimer = null;
  }
  cmpRunning = false;
  document.getElementById("btn-cmp-run").disabled = false;
  document.getElementById("btn-cmp-stop").disabled = true;
}

function resetComparison() {
  stopCompare();
  buildCompareView();
  document.getElementById("cmp-progress-list").innerHTML = "";
  buildCmpAlgoChecks();
  buildCompareView();
}

// ── Results comparison table ──
function showCmpResults() {
  const tableDiv = document.getElementById("cmp-results-table");

  const results = Object.entries(cmpState)
    .filter(([, s]) => s.result)
    .map(([algo, s]) => ({ algo, ...s.result }));

  if (!results.length) return;

  const foundResults = results.filter((r) => r.found);
  const minCost = foundResults.length
    ? Math.min(...foundResults.map((r) => r.cost))
    : null;
  const minExp = foundResults.length
    ? Math.min(...foundResults.map((r) => r.expanded))
    : null;
  const minStep = foundResults.length
    ? Math.min(...foundResults.map((r) => r.steps))
    : null;

  // Sort: found first, then by cost asc
  results.sort((a, b) => {
    if (a.found && !b.found) return -1;
    if (!a.found && b.found) return 1;
    if (a.found && b.found) return a.cost - b.cost;
    return 0;
  });

  tableDiv.innerHTML = `
    <div class="cmp-table-wrap">
      <div class="cmp-table-title">
        <svg width="12" height="12" viewBox="0 0 12 12" fill="none">
          <rect x="1" y="7" width="2" height="4" rx=".5" fill="currentColor"/>
          <rect x="5" y="4" width="2" height="7" rx=".5" fill="currentColor"/>
          <rect x="9" y="1" width="2" height="10" rx=".5" fill="currentColor"/>
        </svg>
        Kết quả so sánh
      </div>
      <table class="cmp-table">
        <thead>
          <tr>
            <th>Thuật toán</th>
            <th>Kết quả</th>
            <th>Nodes mở rộng</th>
            <th>Chi phí (g)</th>
            <th>Số bước</th>
          </tr>
        </thead>
        <tbody>
          ${results
            .map((r, idx) => {
              const isBestCost = r.found && r.cost === minCost;
              const isBestExp = r.found && r.expanded === minExp;
              const isBestStep = r.found && r.steps === minStep;
              const rowClass = idx === 0 && r.found ? "cmp-winner-row" : "";
              return `
              <tr class="${rowClass}">
                <td>
                  <span class="cmp-algo-badge sm" style="background:${CMP_COLORS[r.algo]}">${r.algo}</span>
                  ${idx === 0 && r.found ? '<span class="cmp-crown">🏆</span>' : ""}
                </td>
                <td><span class="cmp-result-badge ${r.found ? "found" : "nofound"}">${r.found ? "✓ Tìm thấy" : "✗ Không có"}</span></td>
                <td class="${isBestExp ? "cmp-best-cell" : ""}">${r.expanded}${isBestExp ? ' <span class="cmp-star">★</span>' : ""}</td>
                <td class="${isBestCost ? "cmp-best-cell" : ""}">${r.cost}${isBestCost ? ' <span class="cmp-star">★</span>' : ""}</td>
                <td class="${isBestStep ? "cmp-best-cell" : ""}">${r.steps}${isBestStep ? ' <span class="cmp-star">★</span>' : ""}</td>
              </tr>`;
            })
            .join("")}
        </tbody>
      </table>
      <div class="cmp-table-note">★ = tốt nhất trong nhóm tìm thấy đường</div>
    </div>`;
}

// ═══════════════════════════════════════
//  UI UPDATES
// ═══════════════════════════════════════
function onAlgoChange() {
  const a = document.getElementById("algo").value;
  document.getElementById("dls-p").style.display =
    a === "DLS" ? "block" : "none";
  document.getElementById("h-p").style.display =
    a === "GBFS" || a === "Astar" ? "block" : "none";
  document.getElementById("algo-desc").textContent = ALGO_DESC[a] || "";
  document
    .querySelectorAll(".pill")
    .forEach((p) => p.classList.toggle("active", p.dataset.a === a));
}

// ═══════════════════════════════════════
//  INIT
// ═══════════════════════════════════════
window.addEventListener("DOMContentLoaded", () => {
  buildGrid();
  onAlgoChange();

  const mid = Math.floor(m / 2);
  for (let j = 2; j < n - 2; j++) {
    if (j !== Math.floor(n / 2)) M[mid][j] = 0;
  }
  for (let i = Math.floor(m / 2) + 1; i < m; i++) {
    for (let j = 0; j < n; j++) {
      if (M[i][j] !== 0 && !(i === tx && j === ty)) {
        M[i][j] = Math.ceil(Math.random() * Math.min(tcnt, 2));
      }
    }
  }
  renderGrid();
});
