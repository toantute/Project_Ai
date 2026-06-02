// ═══════════════════════════════════════
//  CONSTANTS & STATE
// ═══════════════════════════════════════
const dx = [-1, 0, 1, 0];
const dy = [0, 1, 0, -1];

const TCOLORS = ["", "#22863a", "#b5860d", "#6d3aaa", "#c0292b"];
const TNAMES = ["", "T1 · Cỏ", "T2 · Cát", "T3 · Đá", "T4 · Đầm lầy"];

const ALGO_DESC = {
  BFS: "Duyệt theo từng lớp (FIFO). Đảm bảo đường ngắn nhất theo số bước trên đồ thị không có trọng số.",
  DFS: "Đi sâu theo một nhánh trước khi quay lại. Nhanh nhưng không tối ưu, có thể đi vòng xa.",
  UCS: "Mở rộng node có chi phí g thấp nhất (như Dijkstra). Tối ưu về chi phí đường đi.",
  DLS: "DFS với giới hạn độ sâu cứng. Hữu ích khi biết trước độ sâu lời giải.",
  IDS: "Lặp DLS với giới hạn tăng dần. Tối ưu + tiết kiệm bộ nhớ như BFS.",
  GBFS: "Tham lam: chỉ dùng heuristic h, bỏ qua g. Nhanh nhưng không đảm bảo tối ưu.",
  Astar: "Kết hợp g + h (f = g + h). Tối ưu và hiệu quả khi h là admissible heuristic.",
};

// Colors per algorithm for compare badges
const CMP_COLORS = {
  BFS: "#00f0ff",
  DFS: "#8b5cf6",
  UCS: "#22c55e",
  DLS: "#f59e0b",
  IDS: "#ef4444",
  GBFS: "#0ea5e9",
  Astar: "#ec4899",
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
let isAnimatingPath = false;
let pathTimeouts = [];

// ─── Compare Mode State ───────────────
let compareMode = false;
let cmpRunners = [
  { id: "r1", algo: "BFS" },
  { id: "r2", algo: "DFS" },
  { id: "r3", algo: "Astar", ht: 0, mul: 1.0 }
];
let nextRunnerId = 4;
let cmpState = {}; // runnerId -> { steps, stepIdx, expandedCount, done, result, algo }
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

// ─── Manhattan heuristic + tie-breaking ─────────────────────────────
// f_tb = g + h*(1+p)
// p = wmin/(D*1000), epsilon rất nhỏ để ưu tiên node gần đích hơn
function heuristicTieBreak(x, y, mul ) {
  const ddx = Math.abs(x - tx);
  const ddy = Math.abs(y - ty);

  const wm = wmin();

  // p < minimum_cost_of_one_step / expected_maximum_path_length
  // Trên lưới m x n, độ dài đường đi tối đa là m * n.
  // Vì vậy p nên là 1 / (m * n)
  const p = 1.0 / (m * n);

  // Manhattan heuristic
  const h = (ddx + ddy) * wm;

  return h * (1 + p) * mul;
}

let activeLandmarks = [];
let landmarkDistsFrom = [];
let landmarkDistsTo = [];

function findClosestNonWall(r, c) {
  if (M[r][c] !== 0) return [r, c];
  const q = [[r, c]];
  const vis = A2(false);
  vis[r][c] = true;
  while (q.length) {
    const [cx, cy] = q.shift();
    if (M[cx][cy] !== 0) return [cx, cy];
    for (let i = 0; i < 4; i++) {
      const nx = cx + dx[i], ny = cy + dy[i];
      if (ok(nx, ny) && !vis[nx][ny]) {
        vis[nx][ny] = true;
        q.push([nx, ny]);
      }
    }
  }
  return [r, c];
}

function computeDistsFrom(lx, ly) {
  const dis = A2(Infinity);
  const pq = new Heap((a, b) => a.d - b.d);
  dis[lx][ly] = 0;
  pq.push({ x: lx, y: ly, d: 0 });
  while (!pq.empty()) {
    const c = pq.pop();
    if (c.d > dis[c.x][c.y]) continue;
    for (let i = 0; i < 4; i++) {
      const nx = c.x + dx[i], ny = c.y + dy[i];
      if (ok(nx, ny) && M[nx][ny] !== 0) {
        const nd = c.d + w[M[nx][ny]];
        if (nd < dis[nx][ny]) {
          dis[nx][ny] = nd;
          pq.push({ x: nx, y: ny, d: nd });
        }
      }
    }
  }
  return dis;
}

function computeDistsTo(lx, ly) {
  const dis = A2(Infinity);
  const pq = new Heap((a, b) => a.d - b.d);
  dis[lx][ly] = 0;
  pq.push({ x: lx, y: ly, d: 0 });
  while (!pq.empty()) {
    const c = pq.pop();
    if (c.d > dis[c.x][c.y]) continue;
    for (let i = 0; i < 4; i++) {
      const nx = c.x + dx[i], ny = c.y + dy[i];
      if (ok(nx, ny) && M[nx][ny] !== 0) {
        const nd = c.d + w[M[c.x][c.y]];
        if (nd < dis[nx][ny]) {
          dis[nx][ny] = nd;
          pq.push({ x: nx, y: ny, d: nd });
        }
      }
    }
  }
  return dis;
}

function initLandmarks() {
  activeLandmarks = [
    findClosestNonWall(0, 0),
    findClosestNonWall(0, n - 1),
    findClosestNonWall(m - 1, 0),
    findClosestNonWall(m - 1, n - 1)
  ];
  landmarkDistsFrom = activeLandmarks.map(([r, c]) => computeDistsFrom(r, c));
  landmarkDistsTo = activeLandmarks.map(([r, c]) => computeDistsTo(r, c));
}

function heuristic(x, y, ht, mul) {
  if (ht === undefined) ht = parseInt(document.getElementById("htype").value);
  if (mul === undefined)
    mul = parseFloat(document.getElementById("hmul").value) || 1;

  if (ht === 3) {
    if (!landmarkDistsFrom.length || !landmarkDistsTo.length) {
      const ddx = Math.abs(x - tx), ddy = Math.abs(y - ty);
      return (ddx + ddy) * wmin() * mul;
    }
    let maxH = 0;
    for (let i = 0; i < activeLandmarks.length; i++) {
      const d_v_L = landmarkDistsTo[i][x][y];
      const d_T_L = landmarkDistsTo[i][tx][ty];
      if (d_v_L !== Infinity && d_T_L !== Infinity) {
        maxH = Math.max(maxH, d_v_L - d_T_L);
      }
      const d_L_T = landmarkDistsFrom[i][tx][ty];
      const d_L_v = landmarkDistsFrom[i][x][y];
      if (d_L_T !== Infinity && d_L_v !== Infinity) {
        maxH = Math.max(maxH, d_L_T - d_L_v);
      }
    }
    return maxH * mul;
  }

  const ddx = Math.abs(x - tx),
    ddy = Math.abs(y - ty);
  // ht=2: Tie-breaking – dùng heuristicTieBreak với base Manhattan
  if (ht === 2) return heuristicTieBreak(x, y, mul);
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
        const ng = c.g + w[M[nx][ny]];
        const nd = c.d + 1;
        q.push({ x: nx, y: ny, d: nd, g: ng });
        st.push({ t: "Fr", x: nx, y: ny, g: ng, d: nd });
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
        const ng = c.g + w[M[nx][ny]];
        const nd = c.d + 1;
        stack.push({ x: nx, y: ny, d: nd, g: ng });
        st.push({ t: "Fr", x: nx, y: ny, g: ng, d: nd });
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
          const nd = c.d + 1;
          pq.push({ x: nx, y: ny, d: nd, g: ng, f: ng });
          st.push({ t: "Fr", x: nx, y: ny, g: ng, f: ng, d: nd });
        }
      }
    }
  }
  st.push({ t: "N" });
  return st;
}

function algoDLS(lim, ignoreFr = false) {
  const st = [],
    dmin = A2(Infinity),
    par = A2(null);
  let reachedLim = false;
  let ops = 0;
  dmin[sx][sy] = 0;
  par[sx][sy] = [sx, sy];
  const stack = [{ x: sx, y: sy, d: 0, g: 0 }];
  while (stack.length) {
    if (++ops > 500000) {
      st.push({ t: "Err" });
      return { st, found: false, reachedLim: false, error: true };
    }
    const c = stack.pop();
    const { x, y } = c;
    if (c.d === lim) reachedLim = true;
    st.push({ t: "E", x, y, g: c.g, d: c.d });
    if (x === tx && y === ty) {
      st.push({ t: "F", path: traceParent(par), g: c.g });
      return { st, found: true, reachedLim };
    }

    // Tối ưu hoá đường đi: xếp các node kề theo khoảng cách Manhattan tới đích.
    // Đẩy node xa đích vào trước, node gần đích vào sau, để Stack sẽ POP node gần đích ra xử lý ĐẦU TIÊN.
    const nbs = [];
    for (let i = 0; i < 4; i++) {
      nbs.push({ nx: x + dx[i], ny: y + dy[i] });
    }
    nbs.sort((a, b) => {
      const da = Math.abs(a.nx - tx) + Math.abs(a.ny - ty);
      const db = Math.abs(b.nx - tx) + Math.abs(b.ny - ty);
      return db - da;
    });

    for (let i = 0; i < 4; i++) {
      const { nx, ny } = nbs[i];
      const nd = c.d + 1;
      if (ok(nx, ny) && M[nx][ny] && nd <= lim && nd < dmin[nx][ny]) {
        dmin[nx][ny] = nd;
        par[nx][ny] = [x, y];
        const ng = c.g + w[M[nx][ny]];
        stack.push({ x: nx, y: ny, d: nd, g: ng });
        if (!ignoreFr) st.push({ t: "Fr", x: nx, y: ny, g: ng, d: nd });
      }
    }
  }
  st.push({ t: "N" });
  return { st, found: false, reachedLim };
}

function algoIDS(cfgCap) {
  const all = [];
  const cap = cfgCap !== undefined ? cfgCap : 9999;

  for (let l = 1; l <= cap; l++) {
    all.push({ t: "I", l });
    all.push({ t: "C" });
    const { st, found, error } = algoDLS(l, false);

    // Remove trailing N or Err before appending steps
    if (st.length > 0) {
      const last = st[st.length - 1];
      if (last.t === "N" || last.t === "Err") st.pop();
    }

    for (let i = 0; i < st.length; i++) all.push(st[i]);

    if (found) return all;
    if (error) {
      all.push({ t: "Err" });
      return all;
    }
  }

  all.push({ t: "N" });
  return all;
}

function algoGBFS(ht, mul) {
  let activeHt = ht;
  if (activeHt === undefined) activeHt = parseInt(document.getElementById("htype").value);
  let activeMul = mul;
  if (activeMul === undefined) activeMul = parseFloat(document.getElementById("hmul").value) || 1;
  if (activeHt === 3) initLandmarks();

  const st = [],
    vis = A2(false),
    par = A2(null);
  const pq = new Heap((a, b) => {
    if (a.h !== b.h) return a.h - b.h;
    return a.rand - b.rand;
  });
  
  vis[sx][sy] = true;
  par[sx][sy] = [sx, sy];
  
  const startH = heuristic(sx, sy, activeHt, activeMul);
  const startRand = Math.random();
  pq.push({ x: sx, y: sy, d: 0, g: 0, h: startH, rand: startRand });
  
  while (!pq.empty()) {
    const c = pq.pop();
    const { x, y } = c;
    
    st.push({ t: "E", x, y, g: c.g, d: c.d, h: c.h, f: c.h, rand: c.rand });
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
        const ng = c.g + w[M[nx][ny]];
        const nd = c.d + 1;
        const nh = heuristic(nx, ny, activeHt, activeMul);
        const nrand = Math.random();
        pq.push({ x: nx, y: ny, d: nd, g: ng, h: nh, rand: nrand });
        st.push({ t: "Fr", x: nx, y: ny, g: ng, h: nh, f: nh, d: nd, rand: nrand });
      }
    }
  }
  st.push({ t: "N" });
  return st;
}

function algoAstar(ht, mul) {
  let activeHt = ht;
  if (activeHt === undefined) activeHt = parseInt(document.getElementById("htype").value);
  let activeMul = mul;
  if (activeMul === undefined) activeMul = parseFloat(document.getElementById("hmul").value) || 1;
  if (activeHt === 3) initLandmarks();

  const st = [],
    dis = A2(Infinity),
    par = A2(null);
  const pq = new Heap((a, b) => {
    if (a.f !== b.f) return a.f - b.f;
    if (a.h !== b.h) return a.h - b.h;
    return a.rand - b.rand;
  });
  dis[sx][sy] = 0;
  par[sx][sy] = [sx, sy];
  
  const startH = heuristic(sx, sy, activeHt, activeMul);
  const startRand = Math.random();
  pq.push({ x: sx, y: sy, d: 0, g: 0, f: startH, h: startH, rand: startRand });
  
  while (!pq.empty()) {
    const c = pq.pop();
    const { x, y } = c;
    if (c.g > dis[x][y]) continue;
    const _eh = c.f - c.g;
    st.push({ t: "E", x, y, g: c.g, d: c.d, h: c.h !== undefined ? c.h : (_eh >= 0 ? _eh : undefined), f: c.f, rand: c.rand });
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
          const nd = c.d + 1;
          const nh = heuristic(nx, ny, activeHt, activeMul);
          const nf = ng + nh;
          const nrand = Math.random();
          pq.push({ x: nx, y: ny, d: nd, g: ng, f: nf, h: nh, rand: nrand });
          st.push({ t: "Fr", x: nx, y: ny, g: ng, h: nh, f: nf, d: nd, rand: nrand });
        }
      }
    }
  }
  st.push({ t: "N" });
  return st;
}



// Dispatch by name (used by both single-run and compare)
function genStepsFor(algo, cfg) {
  document.querySelectorAll("#terrain-ui .tc-row input").forEach((inp, i) => {
    w[i + 1] = Math.max(1, parseInt(inp.value) || 1);
  });

  const autoEl = document.getElementById("dls-auto");
  const dLimitVal =
    autoEl && autoEl.checked
      ? 9999
      : parseInt(document.getElementById("dls-lim").value) || 10;
  const dLimit = cfg && cfg.dls !== undefined ? cfg.dls : dLimitVal;
  const hType = cfg ? cfg.ht : undefined;
  const hMul = cfg ? cfg.mul : undefined;

  if (algo === "BFS") return algoBFS();
  if (algo === "DFS") return algoDFS();
  if (algo === "UCS") return algoUCS();
  if (algo === "DLS") return algoDLS(dLimit).st;
  if (algo === "IDS") return algoIDS(dLimit);
  if (algo === "GBFS") return algoGBFS(hType, hMul);
  if (algo === "Astar") return algoAstar(hType, hMul);
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
  const icons = ["🌿","🏜️","🪨","🌊"];
  for (let i = 1; i <= tcnt; i++) {
    const row = document.createElement("div");
    row.className = "tc-row";
    row.innerHTML = `
      <div class="tc-swatch" style="background:${TCOLORS[i]}; display: flex; align-items: center; justify-content: center; font-size: 12px; text-shadow: 0 1px 2px rgba(0,0,0,0.3);">${icons[i-1]}</div>
      <label style="width: auto; flex: 1;">T${i} · ${TNAMES[i].replace(/T\d · /, "")}</label>
      <input type="number" value="${w[i] || i}" min="1" max="999" style="width:64px;margin:0;flex:none;"
        onchange="w[${i}]=Math.max(1,parseInt(this.value)||1)">`;
    c.appendChild(row);
  }
  updateDrawModes();
}

function updateDrawModes() {
  const c = document.getElementById("draw-modes");
  c.innerHTML = "";
  const modes = [
    { id: "wall",  icon: "⬛", label: "Tường" },
    { id: "erase", icon: "◻",  label: "Xoá" },
    { id: "start", icon: "🏠", label: "Xuất phát" },
    { id: "end",   icon: "🏆", label: "Đích đến" },
  ];
  for (let i = 1; i <= tcnt; i++) {
    const icons = ["🌿","🏜️","🪨","🌊"];
    modes.push({ id: `t${i}`, icon: icons[i-1] || "◆", label: TNAMES[i].replace(/T\d · /, "") });
  }
  modes.forEach((mo) => {
    const b = document.createElement("div");
    b.className = "dm-btn" + (drawMode === mo.id ? " active" : "");
    b.innerHTML = `<span class="dm-icon">${mo.icon}</span>${mo.label}`;
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
  const LBL_SIZE = 22;
  const avW = Math.floor((window.innerWidth * 0.52 - 32 - LBL_SIZE) / n);
  const avH = Math.floor((window.innerHeight * 0.73 - 32 - LBL_SIZE) / m);
  const cs = Math.max(18, Math.min(48, Math.min(avW, avH)));
  
  el.style.gridTemplateColumns = `${LBL_SIZE}px repeat(${n}, ${cs}px)`;
  el.innerHTML = "";

  // Top-left corner
  const corner = document.createElement("div");
  corner.className = "grid-lbl";
  corner.style.height = LBL_SIZE + "px";
  el.appendChild(corner);

  // Column headers
  for (let j = 0; j < n; j++) {
    const lbl = document.createElement("div");
    lbl.className = "grid-lbl";
    lbl.textContent = j;
    lbl.style.height = LBL_SIZE + "px";
    el.appendChild(lbl);
  }

  for (let i = 0; i < m; i++) {
    // Row header
    const rlbl = document.createElement("div");
    rlbl.className = "grid-lbl";
    rlbl.textContent = i;
    rlbl.style.height = cs + "px";
    el.appendChild(rlbl);

    for (let j = 0; j < n; j++) {
      const d = document.createElement("div");
      d.className = "cell";
      d.style.width = d.style.height = cs + "px";
      d.id = `c_${i}_${j}`;
      d.dataset.x = i;
      d.dataset.y = j;
      d.innerHTML = `<div class="cell-ov"></div><div class="cell-mk"></div><div class="cell-info"></div>`;
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

  el.style.position = "relative";
  const char = document.createElement("img");
  char.id = "char-runner";
  char.src = "img/dungim (2).png";
  char.style.position = "absolute";
  char.style.display = "none";
  char.style.zIndex = "50";
  char.style.pointerEvents = "none";
  char.style.transition = "transform 0.2s cubic-bezier(0.34, 1.56, 0.64, 1)";
  el.appendChild(char);
}

let charMoveTimer = null;
let charLastX = -1;

function updateCharRunner(x, y, instant = false, forceDuration = null) {
  if (compareMode) return;
  const char = document.getElementById("char-runner");
  const c = document.getElementById(`c_${x}_${y}`);
  if (!char || !c) return;
  
  char.style.display = "block";
  
  const cx = c.offsetLeft;
  const cy = c.offsetTop;
  const cw = c.offsetWidth;
  const ch = c.offsetHeight;
  
  char.style.width = cw + "px";
  char.style.height = ch + "px";

  if (charMoveTimer) {
    clearTimeout(charMoveTimer);
    charMoveTimer = null;
  }

  let scaleX = char.dataset.scaleX ? parseFloat(char.dataset.scaleX) : 0.7;
  
  if (charLastX === -1) {
    char.style.transition = "none";
    char.style.transform = `translate(${cx}px, ${cy}px) scaleX(0.7) scaleY(0.7)`;
    charLastX = cx;
    char.dataset.scaleX = 0.7;
    if (!char.src.includes("dungim%20(2).png") && !char.src.includes("dungim (2).png")) char.src = "img/dungim (2).png";
    return;
  }

  if (cx < charLastX) {
    scaleX = -0.7;
  } else if (cx > charLastX) {
    scaleX = 0.7;
  }
  charLastX = cx;
  char.dataset.scaleX = scaleX;

  let duration = forceDuration !== null ? forceDuration : 300;
  let standDelay = duration + 50;
  
  if (instant) {
    char.style.transition = "none";
    if (!char.src.includes("dungim%20(2).png") && !char.src.includes("dungim (2).png")) char.src = "img/dungim (2).png";
  } else {
    char.style.transition = `transform ${duration}ms linear`;
    if (!char.src.includes("diboquaphai")) char.src = "img/diboquaphai.png";
    
    charMoveTimer = setTimeout(() => {
      if (!char.src.includes("dungim%20(2).png") && !char.src.includes("dungim (2).png")) char.src = "img/dungim (2).png";
      charMoveTimer = null;
    }, standDelay);
  }
  
  char.style.objectFit = "contain";
  char.style.transform = `translate(${cx}px, ${cy}px) scaleX(${scaleX}) scaleY(0.7)`;
}

function refreshCell(i, j) {
  const d = document.getElementById(`c_${i}_${j}`);
  if (!d) return;
  d.className = "cell";
  const t = M[i][j];
  if (t === 0) d.classList.add("wall");
  else d.classList.add(`t${t}`);
  const mk = d.querySelector(".cell-mk");
  if (i === sx && j === sy) mk.textContent = "🏠";
  else if (i === tx && j === ty) mk.textContent = "🏆";
  else mk.textContent = "";
}

function paintCell(i, j) {
  if (running || cmpRunning) return;
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
    if (compareMode) refreshCompareCellAll(prevSx, prevSy);
  } else if (drawMode === "end") {
    tx = i;
    ty = j;
    if (M[i][j] === 0) M[i][j] = 1;
    refreshCell(prevTx, prevTy);
    if (compareMode) refreshCompareCellAll(prevTx, prevTy);
  } else if (drawMode.startsWith("t")) {
    const t = parseInt(drawMode.slice(1));
    if ((i === sx && j === sy) || (i === tx && j === ty)) return;
    M[i][j] = t;
  }
  refreshCell(i, j);
  if (compareMode) refreshCompareCellAll(i, j);
}

function refreshCompareCellAll(x, y) {
  if (!cmpRunners) return;
  cmpRunners.forEach(runner => {
    const d = cmpCell(runner.id, x, y);
    if (!d) return;
    const tColors = ["", "var(--t1)", "var(--t2)", "var(--t3)", "var(--t4)"];
    const t = M[x][y];
    d.style.background = t === 0 ? "var(--wall)" : tColors[t] || "var(--t1)";
    d.dataset.terrain = t;
    d.style.outline = "";
    d.style.boxShadow = "";
    delete d.dataset.state;

    let mkHtml = "";
    if (x === sx && y === sy) {
      mkHtml = `<span style="position:absolute;inset:0;display:flex;align-items:center;justify-content:center;font-size:clamp(7px, 1.2vw, 16px);line-height:1;margin:auto;z-index:10">🏠</span>`;
    } else if (x === tx && y === ty) {
      mkHtml = `<span style="position:absolute;inset:0;display:flex;align-items:center;justify-content:center;font-size:clamp(7px, 1.2vw, 16px);line-height:1;margin:auto;z-index:10">🏆</span>`;
    }
    d.innerHTML = `<div class="cell-info" style="display:none"></div>${mkHtml}`;
  });
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
  dsInit();
  animateLoop();
}

function animateLoop() {
  if (stepIdx >= steps.length) {
    running = false;
    if (!isAnimatingPath) {
      document.getElementById("btn-run").disabled = false;
      document.getElementById("btn-step").disabled = false;
      const btnStepBack = document.getElementById("btn-step-back");
      if (btnStepBack) btnStepBack.disabled = false;
      document.getElementById("btn-pause").disabled = true;
    }
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
    dsInit();
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
  btn.textContent = "▶ Tiếp tục";
  btn.onclick = resumeRun;
  btn.disabled = false;
  document.getElementById("btn-pause").disabled = true;
}

function resumeRun() {
  running = true;
  const btn = document.getElementById("btn-run");
  btn.textContent = "▶ Chạy";
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
  
  if (typeof pathTimeouts !== 'undefined') {
    pathTimeouts.forEach(t => clearTimeout(t));
    pathTimeouts = [];
    isAnimatingPath = false;
  }
  
  steps = [];
  stepIdx = 0;
  expandedCount = 0;
  // Reset DS visualizer
  dsAlgo = null; dsType = null; dsItems = []; dsPopPending = false;
  dsPendingSortId = null;
  if (dsSortTimer) { clearTimeout(dsSortTimer); dsSortTimer = null; }
  const dsSec = document.getElementById('ds-vis-sec');
  if (dsSec && !compareMode) dsSec.style.display = 'none';
  const gridEl = document.getElementById("grid");
  if (gridEl) gridEl.classList.remove("path-found");
  document.querySelectorAll(".cell").forEach((c) => {
    c.classList.remove("vis", "front", "onpath", "popping", "pathpop");
    const ov = c.querySelector(".cell-ov");
    if (ov) ov.style.animation = "";
    const info = c.querySelector(".cell-info");
    if (info) {
      info.textContent = "";
      info.style.display = "none";
    }
  });
  
  const char = document.getElementById("char-runner");
  if (char) char.style.display = "none";

  resetStats();
  document.getElementById("btn-run").disabled = false;
  document.getElementById("btn-step").disabled = false;
  const btnStepBack = document.getElementById("btn-step-back");
  if (btnStepBack) btnStepBack.disabled = false;
  document.getElementById("btn-pause").disabled = true;
}

function clearVisOnly() {
  const gridEl = document.getElementById("grid");
  if (gridEl) gridEl.classList.remove("path-found");
  document.querySelectorAll(".cell").forEach((c) => {
    c.classList.remove("vis", "front", "onpath", "popping", "pathpop");
    const ov = c.querySelector(".cell-ov");
    if (ov) ov.style.animation = "";
    const info = c.querySelector(".cell-info");
    if (info) {
      info.textContent = "";
      info.style.display = "none";
    }
  });
  expandedCount = 0;
  document.getElementById("s-exp").textContent = "—";
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
// Returns whether the cell-info labels should be shown
function showLabels() {
  const el = document.getElementById("chk-labels");
  return el ? el.checked : false;
}

function showCmpLabels() {
  const el = document.getElementById("chk-labels-cmp");
  return el ? el.checked : false;
}

function toggleCmpLabels() {
  const show = showCmpLabels();
  document.querySelectorAll(".cmp-mini-grid .cell-info").forEach((info) => {
    if (info.innerHTML) {
      info.style.display = show ? "flex" : "none";
    }
  });
}

function toggleSingleLabels() {
  const show = showLabels();
  document.querySelectorAll("#grid .cell-info").forEach((info) => {
    if (info.innerHTML) {
      info.style.display = show ? "flex" : "none";
    }
  });
}

function generateCellInfoHTML(step, algo) {
  if (!algo) algo = document.getElementById("algo").value;
  const parts = [];

  // Determine which values to show based on the current algorithm
  if (algo === "BFS" || algo === "DFS" || algo === "DLS" || algo === "IDS") {
    if (step.d !== undefined)
      parts.push(`<span class="ci-d">d:${step.d}</span>`);
  } else if (algo === "UCS") {
    if (step.g !== undefined)
      parts.push(`<span class="ci-g">g:${+step.g.toFixed(1)}</span>`);
  } else if (algo === "GBFS") {
    if (step.h !== undefined)
      parts.push(`<span class="ci-h">h:${+step.h.toFixed(1)}</span>`);
  } else if (algo === "Astar" || algo === "AstarTB") {
    if (step.f !== undefined)
      parts.push(`<span class="ci-f">f:${+step.f.toFixed(1)}</span>`);
    if (step.g !== undefined)
      parts.push(`<span class="ci-g">g:${+step.g.toFixed(1)}</span>`);
    if (step.h !== undefined)
      parts.push(`<span class="ci-h">h:${+step.h.toFixed(1)}</span>`);
  }

  if (parts.length === 0) return null;
  return parts.join("");
}

function setCellInfo(c, step, algo) {
  const info = c.querySelector(".cell-info");
  if (!info) return;
  const html = generateCellInfoHTML(step, algo);
  if (!html) {
    info.style.display = "none";
    info.innerHTML = "";
    return;
  }
  info.innerHTML = html;
  const shouldShow = algo ? showCmpLabels() : showLabels();
  info.style.display = shouldShow ? "flex" : "none";
}

// ═══════════════════════════════════════
//  DATA STRUCTURE VISUALIZER
// ═══════════════════════════════════════
const DS_MAX = 7;      // max items shown before "+N"
let dsItems = [];      // [{id, x, y, g, h, f, d}]
let dsNextId = 0;
let dsAlgo = null;
let dsType = null;     // 'queue' | 'stack' | 'pqueue'
let dsPopPending = false; // prevent re-render during leave animation
let dsPendingSortId = null;
let dsSortTimer = null;

function dsInit() {
  if (compareMode) return;
  dsItems = [];
  dsNextId = 0;
  dsPopPending = false;
  dsPendingSortId = null;
  if (dsSortTimer) { clearTimeout(dsSortTimer); dsSortTimer = null; }
  dsAlgo = document.getElementById('algo').value;

  if (dsAlgo === 'BFS') dsType = 'queue';
  else if (['DFS','DLS','IDS'].includes(dsAlgo)) dsType = 'stack';
  else dsType = 'pqueue';

  const sec = document.getElementById('ds-vis-sec');
  const lbl = document.getElementById('ds-vis-label');
  if (!sec) return;
  sec.style.display = '';

  if (dsType === 'queue') {
    lbl.textContent = 'Queue – FIFO (BFS)';
  } else if (dsType === 'stack') {
    lbl.textContent = `Stack – LIFO (${dsAlgo})`;
  } else {
    const nm = { UCS:'g', GBFS:'h', Astar:'f' };
    lbl.textContent = `Priority Queue – min-${nm[dsAlgo]||'?'} (${dsAlgo === 'Astar' ? 'A★' : dsAlgo})`;
  }

  dsRender(null);
}

function dsSort(a, b) {
  if (dsAlgo === 'UCS')   return (a.g ?? 0) - (b.g ?? 0);
  if (dsAlgo === 'GBFS')  return (a.h ?? 0) - (b.h ?? 0);
  if (dsAlgo === 'Astar') {
    const fa = a.f ?? 0, fb = b.f ?? 0;
    if (fa !== fb) return fa - fb;
    const ha = a.h ?? 0, hb = b.h ?? 0;
    if (ha !== hb) return ha - hb;
    return (a.rand ?? 0) - (b.rand ?? 0);
  }
  return 0;
}

function dsGetValHTML(item) {
  const fmt = v => (v !== undefined && v !== null) ? +v.toFixed(1) : '?';
  if (dsAlgo === 'BFS' || dsAlgo === 'DFS' || dsAlgo === 'DLS' || dsAlgo === 'IDS')
    return `<span class="ds-val-d">d:${item.d ?? '?'}</span>`;
  if (dsAlgo === 'UCS')   return `<span class="ds-val-g">g:${fmt(item.g)}</span>`;
  if (dsAlgo === 'GBFS')  return `<span class="ds-val-h">h:${fmt(item.h)}</span>`;
  if (dsAlgo === 'Astar') {
    return `
      <div style="font-size: 0.45rem; line-height: 1.1; margin-bottom: 1px;">
        <span class="ds-val-g">g:${fmt(item.g)}</span> 
        <span class="ds-val-h">h:${fmt(item.h)}</span>
      </div>
      <span class="ds-val-f">f:${fmt(item.f)}</span>
    `;
  }
  return '';
}

function spawnFlyingNode(fromRect, toRect, innerHtml, isPush) {
  const ghost = document.createElement('div');
  const duration = isPush ? 600 : 300;
  ghost.className = 'ds-item';
  ghost.style.position = 'fixed';
  ghost.style.zIndex = 9999;
  ghost.style.margin = '0';
  ghost.style.left = fromRect.left + 'px';
  ghost.style.top = fromRect.top + 'px';
  ghost.style.width = fromRect.width + 'px';
  ghost.style.height = fromRect.height + 'px';
  ghost.style.transition = `all ${duration}ms cubic-bezier(0.25, 1, 0.5, 1)`;
  ghost.style.pointerEvents = 'none';
  ghost.innerHTML = innerHtml;
  
  if (isPush) {
    ghost.style.background = 'var(--front-bg)';
    ghost.style.color = '#fff';
    ghost.style.opacity = '0.9';
  } else {
    ghost.style.background = '#f472b6';
    ghost.style.color = '#fff';
    ghost.style.boxShadow = '0 0 15px rgba(244,114,182,0.6)';
    ghost.style.opacity = '1';
  }

  document.body.appendChild(ghost);
  ghost.getBoundingClientRect(); // force reflow

  ghost.style.left = toRect.left + 'px';
  ghost.style.top = toRect.top + 'px';
  ghost.style.width = toRect.width + 'px';
  ghost.style.height = toRect.height + 'px';
  ghost.style.opacity = '0.1';
  ghost.style.transform = isPush ? 'scale(0.5)' : 'scale(1.5)';

  setTimeout(() => ghost.remove(), duration);
}

function dsPushItem(step) {
  if (!dsAlgo || compareMode) return;
  const item = {
    id: dsNextId++,
    x: step.x, y: step.y,
    g: step.g, h: step.h, f: step.f, d: step.d, rand: step.rand
  };

  if (dsSortTimer) {
    clearTimeout(dsSortTimer);
    dsSortTimer = null;
    dsPendingSortId = null;
  }

  if (dsType === 'pqueue') {
    // Replace existing entry for same cell (relaxation / update)
    const eIdx = dsItems.findIndex(i => i.x === item.x && i.y === item.y);
    if (eIdx !== -1) {
      item.id = dsItems[eIdx].id; // keep same id to avoid re-enter anim
      dsItems[eIdx] = item;
    } else {
      dsItems.push(item);
    }
    dsItems.sort(dsSort);
    dsPendingSortId = item.id;
  } else if (dsType === 'queue') {
    dsItems.push(item);   // enqueue to back
  } else {
    dsItems.unshift(item); // push to top (front of array)
  }

  dsRender(item.id);

  // Flying animation
  const cellEl = document.getElementById(`c_${step.x}_${step.y}`);
  const track = document.getElementById('ds-track');
  if (cellEl && track) {
    const fR = cellEl.getBoundingClientRect();
    let targetRect;
    const newEl = document.getElementById(`ds-item-${item.id}`);
    
    if (newEl) {
      targetRect = newEl.getBoundingClientRect();
    } else {
      const tR = track.getBoundingClientRect();
      targetRect = {
        left: tR.left + tR.width / 2 - 20,
        top: tR.top + tR.height / 2 - 20,
        width: 40,
        height: 40
      };
    }
    
    spawnFlyingNode(fR, targetRect, `<div class="ds-coord">(${step.x},${step.y})</div>`, true);
  }

  if (dsType === 'pqueue') {
    dsSortTimer = setTimeout(() => {
      dsPendingSortId = null;
      dsRender(null, { useFlip: true });
    }, 600);
  }
}

function dsPopItem(step) {
  if (!dsAlgo || compareMode) return;
  const idx = dsItems.findIndex(i => i.x === step.x && i.y === step.y);
  if (idx === -1) { dsRender(null); return; }

  // Animate the leaving DOM element before removing from array
  const track = document.getElementById('ds-track');
  if (track && idx < DS_MAX) {
    const itemEls = track.querySelectorAll('.ds-item');
    const el = itemEls[idx];
    if (el) {
      const fR = el.getBoundingClientRect();
      const cellEl = document.getElementById(`c_${step.x}_${step.y}`);
      const tR = cellEl ? cellEl.getBoundingClientRect() : fR;
      spawnFlyingNode(fR, tR, el.innerHTML, false);

      el.classList.remove('ds-next', 'ds-enter-l', 'ds-enter-r', 'ds-enter-d');
      el.classList.add('ds-leaving');
      dsPopPending = true;
      dsItems.splice(idx, 1);
      setTimeout(() => {
        dsPopPending = false;
        dsRender(null);
      }, 300);
      return;
    }
  }
  dsItems.splice(idx, 1);
  dsRender(null);
}

function dsClearItems() {
  if (!dsAlgo || compareMode) return;
  dsItems = [];
  dsPopPending = false;
  dsRender(null);
}

function dsRender(newItemId, opts = {}) {
  if (dsPopPending) return; // wait for leave animation to finish
  const track = document.getElementById('ds-track');
  const badge = document.getElementById('ds-size-badge');
  if (!track) return;
  if (badge) badge.textContent = dsItems.length;

  if (dsItems.length === 0) {
    track.innerHTML = '<span class="ds-empty">empty</span>';
    return;
  }

  const oldRects = new Map();
  if (opts.useFlip) {
    track.querySelectorAll('.ds-item').forEach(el => {
      const idStr = el.id.replace('ds-item-', '');
      if (idStr) oldRects.set(parseInt(idStr), el.getBoundingClientRect());
    });
  }

  let visible = dsItems.slice(0, DS_MAX);
  let overflow = dsItems.length - DS_MAX;

  if (dsPendingSortId != null) {
    const pId = dsPendingSortId;
    const pIdx = visible.findIndex(i => i.id === pId);
    if (pIdx !== -1) {
      const [pItem] = visible.splice(pIdx, 1);
      visible.push(pItem);
    } else {
      const realIdx = dsItems.findIndex(i => i.id === pId);
      if (realIdx !== -1) {
        visible.push(dsItems[realIdx]);
        if (visible.length > DS_MAX) {
          visible.splice(DS_MAX - 1, 1);
        }
      }
    }
  }

  track.innerHTML = '';

  visible.forEach((item, idx) => {
    const isNew  = (item.id === newItemId);
    const isNext = (idx === 0);

    // Enter animation direction
    let enterCls = '';
    if (isNew) {
      if (dsType === 'queue')  enterCls = ' ds-enter-r'; // new items at back → enter from right
      else if (dsType === 'stack') enterCls = ' ds-enter-l'; // new top → enter from left
      else                     enterCls = ' ds-enter-d'; // sorted insert → scale up
    }

    const el = document.createElement('div');
    el.className = 'ds-item' + (isNext ? ' ds-next' : '') + enterCls;
    el.id = `ds-item-${item.id}`;

    const valHTML = dsGetValHTML(item);
    el.innerHTML  = `<div class="ds-coord">(${item.x},${item.y})</div>` +
                    (valHTML ? `<div class="ds-val">${valHTML}</div>` : '');
    track.appendChild(el);

    // Separator arrow
    if (idx < visible.length - 1 || overflow > 0) {
      const sep = document.createElement('span');
      sep.className = 'ds-sep';
      // Stack: arrows point right (top is left), Queue/PQ: left
      sep.textContent = dsType === 'stack' ? '›' : '›';
      track.appendChild(sep);
    }
  });

  if (overflow > 0) {
    const more = document.createElement('div');
    more.className = 'ds-more';
    more.textContent = `+${overflow}`;
    track.appendChild(more);
  }

  if (opts.useFlip) {
    const newEls = Array.from(track.querySelectorAll('.ds-item'));
    newEls.forEach(el => {
      const idStr = el.id.replace('ds-item-', '');
      const oldRect = oldRects.get(parseInt(idStr));
      if (oldRect) {
        const newRect = el.getBoundingClientRect();
        const dx = oldRect.left - newRect.left;
        const dy = oldRect.top - newRect.top;
        if (dx !== 0 || dy !== 0) {
          el.style.transition = 'none';
          el.style.transform = `translate(${dx}px, ${dy}px)`;
          el.style.zIndex = '100';
          
          requestAnimationFrame(() => {
            requestAnimationFrame(() => {
              el.style.transition = 'transform 300ms cubic-bezier(0.34, 1.56, 0.64, 1)';
              el.style.transform = 'none';
              setTimeout(() => { el.style.zIndex = ''; }, 300);
            });
          });
        }
      }
    });
  }
}

// ═══════════════════════════════════════
function showExpandAnimation(domCell, step) {
  // Ring ripple
  const ring = document.createElement('div');
  ring.className = 'cell-expand-ring';
  domCell.appendChild(ring);
  setTimeout(() => ring.remove(), 520);
}

function processStep(step) {
  if (step.t === "E") {
    expandedCount++;
    const c = cell(step.x, step.y);
    if (c) {
      c.classList.remove("front");
      c.classList.add("vis", "popping");
      setTimeout(() => c.classList.remove("popping"), 380);
      // Clear cell-info when a node gets expanded (it's no longer frontier)
      const info = c.querySelector(".cell-info");
      if (info) {
        info.textContent = "";
        info.style.display = "none";
      }
      // Show expand animation + why popup
      showExpandAnimation(c, step);
      // Pop from DS visualizer (item was at front/top/min)
      dsPopItem(step);
    }
    document.getElementById("s-exp").textContent = expandedCount;
    const tbTag = step.tb ? ' <span style="color:#f97316;font-size:0.8em;font-weight:700">[TB]</span>' : "";
  } else if (step.t === "Fr") {
    const c = cell(step.x, step.y);
    if (c && !c.classList.contains("vis")) {
      c.classList.add("front");
      setCellInfo(c, step);
    }
    // Push neighbor to DS visualizer
    dsPushItem(step);
  } else if (step.t === "F") {
    const gridEl = document.getElementById("grid");
    if (gridEl) gridEl.classList.add("path-found");
    isAnimatingPath = true;
    document.getElementById("btn-run").disabled = true;
    document.getElementById("btn-step").disabled = true;
    const btnStepBack = document.getElementById("btn-step-back");
    if (btnStepBack) btnStepBack.disabled = true;
    document.getElementById("btn-pause").disabled = true;

    updateCharRunner(sx, sy, true);
    step.path.forEach(([x, y], idx) => {
      const t = setTimeout(() => {
        const c = cell(x, y);
        if (!c) return;
        c.classList.remove("front", "vis", "popping");
        c.classList.add("onpath", "pathpop");
        setTimeout(() => c.classList.remove("pathpop"), 100);
        
        updateCharRunner(x, y, false, 100);
        
        if (idx === step.path.length - 1) {
          isAnimatingPath = false;
          document.getElementById("btn-run").disabled = false;
          document.getElementById("btn-step").disabled = false;
          if (btnStepBack) btnStepBack.disabled = false;
        }
      }, idx * 100);
      pathTimeouts.push(t);
    });
    document.getElementById("s-cost").textContent = step.g;
    document.getElementById("s-len").textContent = step.path.length - 1;
  } else if (step.t === "N") {
  } else if (step.t === "Err") {
  } else if (step.t === "I") {
    document.getElementById("s-depth").textContent = step.l;
  } else if (step.t === "C") {
    clearVisOnly();
    dsClearItems(); // IDS restarts → clear DS
  }
}

function updateProgressBar() {
  const p = steps.length ? (stepIdx / steps.length) * 100 : 0;
  document.getElementById("pbar").style.width = p + "%";
  document.getElementById("pbar-lbl").textContent =
    `Bước ${stepIdx} / ${steps.length}`;
}

function fastForwardTo(targetIdx) {
  let expCount = 0;
  let sCost = "—";
  let sLen = "—";
  let sDepth = "0";

  const cellState = Array.from({ length: m }, () => Array.from({ length: n }, () => ({ cls: null, info: null })));

  let simDsItems = [];
  let simDsNextId = 0;
  let lastEx = -1, lastEy = -1;

  for (let i = 0; i < targetIdx; i++) {
    const step = steps[i];
    if (step.t === "E") {
      expCount++;
      cellState[step.x][step.y].cls = "vis";
      cellState[step.x][step.y].info = null;
      lastEx = step.x;
      lastEy = step.y;

      if (dsAlgo && !compareMode) {
        const idx = simDsItems.findIndex(i => i.x === step.x && i.y === step.y);
        if (idx !== -1) simDsItems.splice(idx, 1);
      }
    } else if (step.t === "Fr") {
      if (cellState[step.x][step.y].cls !== "vis") {
        cellState[step.x][step.y].cls = "front";
        cellState[step.x][step.y].info = generateCellInfoHTML(step);
      }

      if (dsAlgo && !compareMode) {
        const item = {
          id: simDsNextId++,
          x: step.x, y: step.y,
          g: step.g, h: step.h, f: step.f, d: step.d, rand: step.rand
        };
        if (dsType === 'pqueue') {
          const eIdx = simDsItems.findIndex(i => i.x === item.x && i.y === item.y);
          if (eIdx !== -1) {
            item.id = simDsItems[eIdx].id;
            simDsItems[eIdx] = item;
          } else {
            simDsItems.push(item);
          }
          simDsItems.sort(dsSort);
        } else if (dsType === 'queue') {
          simDsItems.push(item);
        } else {
          simDsItems.unshift(item);
        }
      }
    } else if (step.t === "F") {
      step.path.forEach(([x, y]) => {
        cellState[x][y].cls = "onpath";
      });
      sCost = step.g;
      sLen = step.path.length - 1;
    } else if (step.t === "N") {
    } else if (step.t === "Err") {
    } else if (step.t === "I") {
      sDepth = step.l;
    } else if (step.t === "C") {
      for (let r = 0; r < m; r++) {
        for (let c = 0; c < n; c++) {
          cellState[r][c].cls = null;
          cellState[r][c].info = null;
        }
      }
      expCount = 0;
      sCost = "—";
      sLen = "—";

      if (dsAlgo && !compareMode) {
        simDsItems = [];
        simDsNextId = 0;
      }
    }
  }

  clearVisOnly();

  for (let r = 0; r < m; r++) {
    for (let c = 0; c < n; c++) {
      const st = cellState[r][c];
      if (st.cls || st.info) {
        const domC = cell(r, c);
        if (domC) {
          if (st.cls) domC.classList.add(st.cls);
          if (st.info) {
            const infoEl = domC.querySelector(".cell-info");
            if (infoEl) {
              infoEl.innerHTML = st.info;
              infoEl.style.display = showLabels() ? "flex" : "none";
            }
          }
        }
      }
    }
  }

  expandedCount = expCount;
  document.getElementById("s-exp").textContent = expandedCount === 0 ? "—" : expandedCount;
  document.getElementById("s-cost").textContent = sCost;
  document.getElementById("s-len").textContent = sLen;
  document.getElementById("s-depth").textContent = sDepth;

  const gridEl = document.getElementById("grid");
  if (gridEl) {
    if (sCost !== "—") gridEl.classList.add("path-found");
    else gridEl.classList.remove("path-found");
  }

  if (dsAlgo && !compareMode) {
    dsItems = simDsItems;
    dsNextId = simDsNextId;
    dsPopPending = false;
    dsPendingSortId = null;
    if (dsSortTimer) { clearTimeout(dsSortTimer); dsSortTimer = null; }
    dsRender(null);
  }

  if (sLen !== "—") {
    updateCharRunner(tx, ty, true);
  } else {
    const char = document.getElementById("char-runner");
    if (char) char.style.display = "none";
  }

  stepIdx = targetIdx;
  updateProgressBar();
}

function stepBack() {
  if (!steps || steps.length === 0) return;
  if (stepIdx <= 0) return;
  if (running) pauseRun();

  const stepToUndo = steps[stepIdx - 1];

  let undoFr_fromRect = null;
  let undoFr_html = null;

  if (dsAlgo && !compareMode && stepToUndo.t === "Fr") {
    const item = dsItems.find(i => i.x === stepToUndo.x && i.y === stepToUndo.y);
    if (item) {
      const el = document.getElementById(`ds-item-${item.id}`);
      if (el) {
        undoFr_fromRect = el.getBoundingClientRect();
        undoFr_html = el.innerHTML;
      }
    }
  }

  fastForwardTo(stepIdx - 1);

  if (dsAlgo && !compareMode) {
    const cellEl = document.getElementById(`c_${stepToUndo.x}_${stepToUndo.y}`);
    if (cellEl) {
      if (stepToUndo.t === "E") {
        const fR = cellEl.getBoundingClientRect();
        let tR;
        const item = dsItems.find(i => i.x === stepToUndo.x && i.y === stepToUndo.y);
        if (item) {
          const newEl = document.getElementById(`ds-item-${item.id}`);
          if (newEl) tR = newEl.getBoundingClientRect();
        }
        if (!tR) {
          const track = document.getElementById('ds-track');
          if (track) tR = track.getBoundingClientRect();
        }
        if (tR) {
          spawnFlyingNode(fR, tR, `<div class="ds-coord">(${stepToUndo.x},${stepToUndo.y})</div>`, true);
        }
      } else if (stepToUndo.t === "Fr" && undoFr_fromRect) {
        const tR = cellEl.getBoundingClientRect();
        spawnFlyingNode(undoFr_fromRect, tR, undoFr_html, false);
      }
    }
  }
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
  const algoPills = document.getElementById("algo-pills");

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
    if (algoPills) algoPills.style.display = "none";
    // Hide DS visualizer in compare mode
    const dsSec = document.getElementById('ds-vis-sec');
    if (dsSec) dsSec.style.display = 'none';
    btn.textContent = "✕ Thoát so sánh";
    btn.classList.add("active");

    buildCmpAlgoChecks();
    buildCompareView();
  } else {
    stopCompare();

    mainView.style.display = "flex";
    cmpView.style.display = "none";
    cmpSelSec.style.display = "none";
    algoSec.style.display = "";
    singleStats.style.display = "";
    singleProg.style.display = "";
    cmpProgSec.style.display = "none";
    if (algoPills) algoPills.style.display = "flex";
    btn.textContent = "⊞ So sánh";
    btn.classList.remove("active");
  }
}

function buildCmpAlgoChecks() {
  const c = document.getElementById("cmp-algo-checks");
  if (!c) return;
  c.innerHTML = "";

  const listContainer = document.createElement("div");
  listContainer.className = "cmp-runners-list";

  cmpRunners.forEach((runner) => {
    const item = document.createElement("div");
    item.className = "cmp-runner-item";

    // Dropdown options
    const algos = ["BFS", "DFS", "UCS", "DLS", "IDS", "GBFS", "Astar"];
    const options = algos.map(a =>
      `<option value="${a}" ${runner.algo === a ? "selected" : ""}>${a === "Astar" ? "A★" : a}</option>`
    ).join("");

    // Deletion button (only if > 2 runners)
    const deleteBtn = cmpRunners.length > 2
      ? `<button class="btn-delete-runner" onclick="deleteRunner('${runner.id}')" title="Xóa cấu hình">✕</button>`
      : "";

    // Color badge style
    const color = CMP_COLORS[runner.algo];

    item.innerHTML = `
      <span class="cmp-runner-badge" style="background:${color}" id="cmp-badge-${runner.id}"></span>
      <select class="cmp-runner-select" onchange="changeRunnerAlgo('${runner.id}', this.value)">
        ${options}
      </select>
      ${deleteBtn}
    `;
    listContainer.appendChild(item);
  });

  c.appendChild(listContainer);

  // Add runner button (only if < 7 runners)
  if (cmpRunners.length < 7) {
    const addBtn = document.createElement("button");
    addBtn.className = "btn btn-sec btn-add-runner";
    addBtn.style.width = "100%";
    addBtn.style.marginTop = "8px";
    addBtn.innerHTML = "＋ Thêm cấu hình";
    addBtn.onclick = addRunner;
    c.appendChild(addBtn);
  }
}

function addRunner() {
  if (cmpRunners.length >= 7) return;
  const id = `r${nextRunnerId++}`;
  cmpRunners.push({
    id: id,
    algo: "Astar",
    ht: 0,
    mul: 1.0,
    dls: 15,
    autoDls: false
  });
  buildCmpAlgoChecks();
  buildCompareView();
}

function deleteRunner(id) {
  if (cmpRunners.length <= 2) return;
  cmpRunners = cmpRunners.filter(r => r.id !== id);
  buildCmpAlgoChecks();
  buildCompareView();
}

function changeRunnerAlgo(id, newAlgo) {
  const runner = cmpRunners.find(r => r.id === id);
  if (!runner) return;

  runner.algo = newAlgo;
  if (newAlgo === "GBFS" || newAlgo === "Astar") {
    if (runner.ht === undefined) runner.ht = 0;
    if (runner.mul === undefined) runner.mul = 1.0;
  } else if (newAlgo === "DLS" || newAlgo === "IDS") {
    if (runner.dls === undefined) runner.dls = 15;
    if (newAlgo === "IDS") {
      runner.autoDls = true;
    } else if (runner.autoDls === undefined) {
      runner.autoDls = false;
    }
  }

  buildCmpAlgoChecks();
  buildCompareView();
}

function saveRunnerParams(id) {
  const runner = cmpRunners.find(r => r.id === id);
  if (!runner) return;

  if (runner.algo === "DLS" || runner.algo === "IDS") {
    const dlsEl = document.getElementById(`cmp-dls-${id}`);
    const autoEl = document.getElementById(`cmp-auto-${id}`);
    if (autoEl) {
      runner.autoDls = autoEl.checked;
      if (dlsEl) dlsEl.disabled = autoEl.checked;
    }
    if (dlsEl) {
      runner.dls = parseInt(dlsEl.value) || 15;
    }
  } else if (runner.algo === "GBFS" || runner.algo === "Astar") {
    const htEl = document.getElementById(`cmp-ht-${id}`);
    const hmulEl = document.getElementById(`cmp-hmul-${id}`);
    if (htEl) runner.ht = parseInt(htEl.value) || 0;
    if (hmulEl) runner.mul = parseFloat(hmulEl.value) || 1.0;
  }
}

function getRunnerDisplayName(runner) {
  const algo = runner.algo;
  if (algo === "BFS" || algo === "DFS" || algo === "UCS") {
    return algo;
  }
  if (algo === "DLS" || algo === "IDS") {
    const name = algo === "IDS" ? "IDS" : "DLS";
    return runner.autoDls ? `${name} - Tới đích` : `${name} - Độ sâu ${runner.dls || 15}`;
  }
  if (algo === "GBFS" || algo === "Astar") {
    const name = algo === "Astar" ? "A★" : "GBFS";
    const htype = runner.ht || 0;
    let hName = "Manhattan";
    if (htype === 1) hName = "Euclidean";
    else if (htype === 2) hName = "Manhattan (Tie-Breaking)";
    else if (htype === 3) hName = "Landmarks";

    const mul = runner.mul !== undefined ? runner.mul : 1.0;
    const mulStr = mul !== 1.0 ? ` × ${mul}` : "";
    return `${name} - ${hName}${mulStr}`;
  }
  return algo;
}

// ── Build compare card grid ──
function buildCompareView() {
  stopCompare();
  const container = document.getElementById("cmp-grids");
  container.innerHTML = "";
  document.getElementById("cmp-results-table").innerHTML = "";
  document.getElementById("cmp-progress-list").innerHTML = "";

  if (cmpRunners.length === 0) return;

  cmpRunners.forEach((runner) => {
    const algo = runner.algo;
    const color = CMP_COLORS[algo];
    const card = document.createElement("div");
    card.className = "cmp-card";
    card.id = `cmp-card-${runner.id}`;
    let extraParams = "";
    if (algo === "DLS" || algo === "IDS") {
      extraParams = `<div class="cmp-card-params">
        <label>Độ sâu:</label>
        <input type="number" id="cmp-dls-${runner.id}" value="${runner.dls || 15}" min="1" style="width: 50px" ${runner.autoDls ? "disabled" : ""} onchange="saveRunnerParams('${runner.id}')" />
        <label style="display:flex;align-items:center;gap:3px;cursor:pointer">
          <input type="checkbox" id="cmp-auto-${runner.id}" ${runner.autoDls ? "checked" : ""} onchange="saveRunnerParams('${runner.id}')" /> Max
        </label>
      </div>`;
    } else if (algo === "GBFS" || algo === "Astar") {
      extraParams = `<div class="cmp-card-params">
        <label>Heuristic:</label>
        <select id="cmp-ht-${runner.id}" onchange="saveRunnerParams('${runner.id}')">
          <option value="0" ${runner.ht === 0 ? "selected" : ""}>Manhattan</option>
          <option value="1" ${runner.ht === 1 ? "selected" : ""}>Euclidean</option>
          <option value="2" ${runner.ht === 2 ? "selected" : ""}>Manhattan (Tie-Breaking)</option>
          <option value="3" ${runner.ht === 3 ? "selected" : ""}>Landmarks (ALT)</option>
        </select>
        <label>Mul:</label>
        <input type="number" id="cmp-hmul-${runner.id}" value="${runner.mul !== undefined ? runner.mul : 1}" step="0.1" style="width: 50px" onchange="saveRunnerParams('${runner.id}')" />
      </div>`;
    }

    let lblText = 'Queue (FIFO)';
    if (algo === 'DFS' || algo === 'DLS' || algo === 'IDS') lblText = 'Stack (LIFO)';
    else if (algo === 'UCS' || algo === 'GBFS' || algo === 'Astar') {
      const nm = { UCS:'g', GBFS:'h', Astar:'f' };
      lblText = `PQ (min-${nm[algo]})`;
    }

    card.innerHTML = `
      <div class="cmp-card-header" style="border-top: 3px solid ${color}">
        <span class="cmp-algo-badge" style="background:${color}">${algo === "Astar" ? "A★" : algo}</span>
        <span class="cmp-card-status" id="cmp-status-${runner.id}">Ready</span>
      </div>
      ${extraParams}
      <div class="cmp-grid-wrap" id="cmp-gwrap-${runner.id}">
        <div id="cmp-grid-${runner.id}" class="cmp-mini-grid"></div>
      </div>
      <div class="cmp-ds-vis" id="cmp-ds-vis-${runner.id}" style="display: none; padding: 4px; margin: 4px 12px; border: 1px solid var(--border); border-radius: 8px; background: rgba(255, 255, 255, 0.5);">
         <div style="font-size: 0.6rem; font-weight: 600; margin-bottom: 2px; display: flex; justify-content: space-between;">
            <span id="cmp-ds-lbl-${runner.id}">${lblText}</span>
            <span id="cmp-ds-sz-${runner.id}" style="background: var(--primary); color: #fff; border-radius: 10px; padding: 0 4px; font-size: 0.55rem;">0</span>
         </div>
         <div class="ds-track-wrap" style="min-height: 24px; padding: 2px; margin: 0;">
            <div class="ds-track" id="cmp-ds-track-${runner.id}" style="min-height: 24px; padding: 2px;">
               <span class="ds-empty" style="font-size: 0.55rem">...</span>
            </div>
         </div>
      </div>
      <div class="cmp-card-stats">
        <div class="cmp-mini-stat">
           <div class="cmp-mini-stat-lbl">Expanded</div>
           <div class="cmp-mini-stat-val" id="cmp-exp-${runner.id}">—</div>
        </div>
        <div class="cmp-mini-stat">
           <div class="cmp-mini-stat-lbl">Cost</div>
           <div class="cmp-mini-stat-val" id="cmp-cost-${runner.id}">—</div>
        </div>
        <div class="cmp-mini-stat">
           <div class="cmp-mini-stat-lbl">Steps</div>
           <div class="cmp-mini-stat-val" id="cmp-len-${runner.id}">—</div>
        </div>
      </div>`;
    container.appendChild(card);
    renderMiniGrid(runner);
  });

  // Build progress bars in right panel
  const pl = document.getElementById("cmp-progress-list");
  pl.innerHTML = "";
  cmpRunners.forEach((runner) => {
    const algo = runner.algo;
    const displayName = getRunnerDisplayName(runner);
    const div = document.createElement("div");
    div.style.marginBottom = "8px";
    div.innerHTML = `
      <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:3px">
        <span class="cmp-algo-badge sm" style="background:${CMP_COLORS[algo]}">${algo === "Astar" ? "A★" : algo}</span>
        <span class="cmp-progress-name-text" style="font-size:0.6rem;font-weight:600;margin-left:4px;color:var(--text-main);flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap">${displayName}</span>
        <span style="font-size:0.6rem;color:var(--muted);font-family:'JetBrains Mono',monospace;margin-left:4px" id="cmp-plbl-${runner.id}">0 / 0</span>
      </div>
      <div class="pbar-bg"><div class="pbar-fill" id="cmp-pbar-${runner.id}" style="width:0%;background:${CMP_COLORS[algo]}"></div></div>`;
    pl.appendChild(div);
  });
}

// ── Render a mini grid for one algorithm ──
function renderMiniGrid(runner) {
  const el = document.getElementById(`cmp-grid-${runner.id}`);
  if (!el) return;

  el.style.gridTemplateColumns = `14px repeat(${n}, 1fr)`;
  el.style.display = "grid";
  el.style.gap = "1px";
  el.style.width = "100%";
  el.style.maxWidth = "300px";
  el.innerHTML = "";

  const tColors = ["", "var(--t1)", "var(--t2)", "var(--t3)", "var(--t4)"];

  // Top-left corner
  const corner = document.createElement("div");
  corner.style.height = "14px";
  el.appendChild(corner);

  // Column headers
  for (let j = 0; j < n; j++) {
    const lbl = document.createElement("div");
    lbl.textContent = j;
    lbl.style.display = "flex";
    lbl.style.alignItems = "center";
    lbl.style.justifyContent = "center";
    lbl.style.fontSize = "8px";
    lbl.style.color = "var(--muted)";
    lbl.style.height = "14px";
    el.appendChild(lbl);
  }

  for (let i = 0; i < m; i++) {
    // Row header
    const rlbl = document.createElement("div");
    rlbl.textContent = i;
    rlbl.style.display = "flex";
    rlbl.style.alignItems = "center";
    rlbl.style.justifyContent = "center";
    rlbl.style.fontSize = "8px";
    rlbl.style.color = "var(--muted)";
    el.appendChild(rlbl);

    for (let j = 0; j < n; j++) {
      const d = document.createElement("div");
      d.className = "cmp-cell";
      const t = M[i][j];
      d.style.cssText = `position:relative;border-radius:1px;width:100%;height:100%;aspect-ratio:1/1;`;
      d.style.background = t === 0 ? "var(--wall)" : tColors[t] || "var(--t1)";
      d.dataset.terrain = t;
      d.id = `${runner.id}_c_${i}_${j}`;

      let mkHtml = "";
      if (i === sx && j === sy) {
        mkHtml = `<span style="position:absolute;inset:0;display:flex;align-items:center;justify-content:center;font-size:clamp(7px, 1.2vw, 16px);line-height:1;margin:auto;z-index:10">🏠</span>`;
      } else if (i === tx && j === ty) {
        mkHtml = `<span style="position:absolute;inset:0;display:flex;align-items:center;justify-content:center;font-size:clamp(7px, 1.2vw, 16px);line-height:1;margin:auto;z-index:10">🏆</span>`;
      }
      d.innerHTML = `<div class="cell-info"></div>${mkHtml}`;
      d.addEventListener("mousedown", (e) => {
        isDown = true;
        paintCell(i, j);
        e.preventDefault();
      });
      d.addEventListener("mousemove", () => {
        if (isDown) paintCell(i, j);
      });
      el.appendChild(d);
    }
  }
}

function cmpCell(runnerId, x, y) {
  return document.getElementById(`${runnerId}_c_${x}_${y}`);
}

// ── Run comparison ──
let cmpGlobalStep = 0;

function initCompareState() {
  if (cmpRunners.length === 0) return false;

  if (cmpTimer) {
    clearTimeout(cmpTimer);
    cmpTimer = null;
  }
  cmpRunning = false;

  const cfgMap = {};
  cmpRunners.forEach((runner) => {
    let cfg = {};
    if (runner.algo === "DLS" || runner.algo === "IDS") {
      const dlsEl = document.getElementById(`cmp-dls-${runner.id}`);
      const autoEl = document.getElementById(`cmp-auto-${runner.id}`);
      if (autoEl && autoEl.checked) cfg.dls = 9999;
      else if (dlsEl) cfg.dls = parseInt(dlsEl.value) || 15;
    } else if (runner.algo === "GBFS" || runner.algo === "Astar") {
      const htEl = document.getElementById(`cmp-ht-${runner.id}`);
      const hmulEl = document.getElementById(`cmp-hmul-${runner.id}`);
      if (htEl) cfg.ht = parseInt(htEl.value) || 0;
      if (hmulEl) cfg.mul = parseFloat(hmulEl.value) || 1;
    }
    cfgMap[runner.id] = cfg;
  });

  buildCompareView();

  cmpState = {};
  cmpRunners.forEach((runner) => {
    cmpState[runner.id] = {
      algo: runner.algo,
      steps: genStepsFor(runner.algo, cfgMap[runner.id]),
      stepIdx: 0,
      expandedCount: 0,
      done: false,
      result: null,
      dsItems: [],
      dsNextId: 0,
    };
    cmpDsRender(runner.id);
    const el = document.getElementById(`cmp-status-${runner.id}`);
    if (el) {
      el.textContent = "Ready";
      el.className = "cmp-card-status";
    }
  });

  document.getElementById("cmp-results-table").innerHTML = "";
  cmpGlobalStep = 0;
  return true;
}

function runComparison() {
  if (!cmpState || Object.keys(cmpState).length === 0 || cmpGlobalStep === 0) {
    if (!initCompareState()) return;
  } else if (cmpGlobalStep > 0 && !cmpRunning) {
    if (!initCompareState()) return;
  }

  cmpRunners.forEach((runner) => {
    const el = document.getElementById(`cmp-status-${runner.id}`);
    if (el && !cmpState[runner.id].done) {
      el.textContent = "Running…";
      el.className = "cmp-card-status running";
    }
    const dsVis = document.getElementById(`cmp-ds-vis-${runner.id}`);
    if (dsVis) dsVis.style.display = "block";
  });

  cmpRunning = true;
  document.getElementById("btn-cmp-run").disabled = true;
  document.getElementById("btn-cmp-step").disabled = true;
  document.getElementById("btn-cmp-step-back").disabled = true;
  document.getElementById("btn-cmp-pause").disabled = false;
  cmpAnimateLoop();
}

function pauseCompare() {
  if (cmpTimer) {
    clearTimeout(cmpTimer);
    cmpTimer = null;
  }
  cmpRunning = false;
  const btn = document.getElementById("btn-cmp-run");
  btn.textContent = "▶ Tiếp tục";
  btn.onclick = resumeCompare;
  btn.disabled = false;

  document.getElementById("btn-cmp-step").disabled = false;
  document.getElementById("btn-cmp-step-back").disabled = false;
  document.getElementById("btn-cmp-pause").disabled = true;
}

// Helper to resume comparison
function resumeCompare() {
  cmpRunning = true;
  const btn = document.getElementById("btn-cmp-run");
  btn.textContent = "▶ Chạy so sánh";
  btn.onclick = runComparison;
  btn.disabled = true;

  document.getElementById("btn-cmp-step").disabled = true;
  document.getElementById("btn-cmp-step-back").disabled = true;
  document.getElementById("btn-cmp-pause").disabled = false;

  cmpRunners.forEach((runner) => {
    const el = document.getElementById(`cmp-status-${runner.id}`);
    if (el && !cmpState[runner.id].done) {
      el.textContent = "Running…";
      el.className = "cmp-card-status running";
    }
    const dsVis = document.getElementById(`cmp-ds-vis-${runner.id}`);
    if (dsVis) dsVis.style.display = "block";
  });

  cmpAnimateLoop();
}

function clearVisCompareAllAction() {
  if (cmpTimer) {
    clearTimeout(cmpTimer);
    cmpTimer = null;
  }
  cmpRunning = false;

  cmpRunners.forEach((runner) => {
    clearCompareVis(runner.id);
    const expEl = document.getElementById(`cmp-exp-${runner.id}`);
    const costEl = document.getElementById(`cmp-cost-${runner.id}`);
    const lenEl = document.getElementById(`cmp-len-${runner.id}`);
    const statusEl = document.getElementById(`cmp-status-${runner.id}`);
    if (expEl) expEl.textContent = "—";
    if (costEl) costEl.textContent = "—";
    if (lenEl) lenEl.textContent = "—";
    if (statusEl) {
      statusEl.textContent = "Ready";
      statusEl.className = "cmp-card-status";
    }

    if (cmpState && cmpState[runner.id]) {
      cmpState[runner.id].stepIdx = 0;
      cmpState[runner.id].expandedCount = 0;
      cmpState[runner.id].done = false;
      cmpState[runner.id].result = null;
      cmpState[runner.id].dsItems = [];
      cmpState[runner.id].dsNextId = 0;
      cmpDsRender(runner.id);
    }
    const dsVis = document.getElementById(`cmp-ds-vis-${runner.id}`);
    if (dsVis) dsVis.style.display = "none";

    const pb = document.getElementById(`cmp-pbar-${runner.id}`);
    const pl = document.getElementById(`cmp-plbl-${runner.id}`);
    if (pb) pb.style.width = "0%";
    if (pl && cmpState && cmpState[runner.id]) pl.textContent = `0 / ${cmpState[runner.id].steps.length}`;
  });

  cmpGlobalStep = 0;
  document.getElementById("cmp-results-table").innerHTML = "";

  const btn = document.getElementById("btn-cmp-run");
  btn.textContent = "▶ Chạy so sánh";
  btn.onclick = runComparison;
  btn.disabled = false;

  document.getElementById("btn-cmp-step").disabled = false;
  document.getElementById("btn-cmp-step-back").disabled = false;
  document.getElementById("btn-cmp-pause").disabled = true;
}

function cmpAnimateLoop() {
  const allDone = Object.values(cmpState).every(
    (s) => s.done || s.stepIdx >= s.steps.length,
  );
  if (allDone) {
    Object.entries(cmpState).forEach(([runnerId, state]) => {
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
    const btn = document.getElementById("btn-cmp-run");
    btn.textContent = "▶ Chạy so sánh";
    btn.onclick = runComparison;
    btn.disabled = false;
    document.getElementById("btn-cmp-step").disabled = false;
    document.getElementById("btn-cmp-step-back").disabled = false;
    document.getElementById("btn-cmp-pause").disabled = true;
    showCmpResults();
    return;
  }

  Object.entries(cmpState).forEach(([runnerId, state]) => {
    if (state.done) return;
    if (state.stepIdx >= state.steps.length) {
      state.done = true;
      return;
    }
    const step = state.steps[state.stepIdx++];
    processCompareStep(runnerId, state, step);

    const total = state.steps.length;
    const pct = total ? (state.stepIdx / total) * 100 : 0;
    const pb = document.getElementById(`cmp-pbar-${runnerId}`);
    const pl = document.getElementById(`cmp-plbl-${runnerId}`);
    if (pb) pb.style.width = pct + "%";
    if (pl) pl.textContent = `${state.stepIdx} / ${total}`;
  });

  cmpGlobalStep++;
  cmpTimer = setTimeout(cmpAnimateLoop, getDelay());
}

function stepCompare() {
  if (!cmpState || Object.keys(cmpState).length === 0) {
    if (!initCompareState()) return;
  }

  const allDone = Object.values(cmpState).every(
    (s) => s.done || s.stepIdx >= s.steps.length,
  );

  if (allDone) return;

  Object.entries(cmpState).forEach(([runnerId, state]) => {
    if (state.done) return;
    if (state.stepIdx >= state.steps.length) {
      state.done = true;
      return;
    }
    const step = state.steps[state.stepIdx++];
    processCompareStep(runnerId, state, step);

    const total = state.steps.length;
    const pct = total ? (state.stepIdx / total) * 100 : 0;
    const pb = document.getElementById(`cmp-pbar-${runnerId}`);
    const pl = document.getElementById(`cmp-plbl-${runnerId}`);
    if (pb) pb.style.width = pct + "%";
    if (pl) pl.textContent = `${state.stepIdx} / ${total}`;
  });

  cmpRunners.forEach((runner) => {
    const dsVis = document.getElementById(`cmp-ds-vis-${runner.id}`);
    if (dsVis) dsVis.style.display = "block";
  });

  cmpGlobalStep++;

  const allDoneNow = Object.values(cmpState).every(
    (s) => s.done || s.stepIdx >= s.steps.length,
  );
  if (allDoneNow) {
    Object.entries(cmpState).forEach(([runnerId, state]) => {
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
    showCmpResults();
  }
}

function fastForwardCompareTo(targetGlobalStep) {
  cmpRunners.forEach((runner) => {
    clearCompareVis(runner.id);
  });
  document.getElementById("cmp-results-table").innerHTML = "";

  let allDone = true;

  cmpRunners.forEach((runner) => {
    const runnerId = runner.id;
    const state = cmpState[runnerId];
    if (!state) return;
    const steps = state.steps;
    const targetIdx = Math.min(targetGlobalStep, steps.length);

    let expCount = 0;
    let sCost = "—";
    let sLen = "—";
    let isFound = false;
    let isNoPath = false;
    let isErr = false;

    let simDsItems = [];
    let simDsNextId = 0;

    const cellState = Array.from({ length: m }, () => Array.from({ length: n }, () => ({ state: null, info: null })));

    for (let i = 0; i < targetIdx; i++) {
      const step = steps[i];
      if (step.t === "E") {
        expCount++;
        cellState[step.x][step.y].state = "vis";
        cellState[step.x][step.y].info = null;
        const dsIdx = simDsItems.findIndex(item => item.x === step.x && item.y === step.y);
        if (dsIdx !== -1) simDsItems.splice(dsIdx, 1);
      } else if (step.t === "Fr") {
        if (cellState[step.x][step.y].state !== "vis") {
          cellState[step.x][step.y].state = "front";
          cellState[step.x][step.y].info = generateCellInfoHTML(step, state.algo);
        }
        const item = {
          id: simDsNextId++,
          x: step.x, y: step.y,
          g: step.g, h: step.h, f: step.f, d: step.d, rand: step.rand
        };
        const algo = state.algo;
        let dsType = 'pqueue';
        if (algo === 'BFS') dsType = 'queue';
        else if (['DFS','DLS','IDS'].includes(algo)) dsType = 'stack';

        if (dsType === 'pqueue') {
          const eIdx = simDsItems.findIndex(i => i.x === item.x && i.y === item.y);
          if (eIdx !== -1) simDsItems[eIdx] = item;
          else simDsItems.push(item);
          simDsItems.sort((a, b) => {
            if (algo === 'UCS') return (a.g ?? 0) - (b.g ?? 0);
            if (algo === 'GBFS') return (a.h ?? 0) - (b.h ?? 0);
            if (algo === 'Astar') {
              const fa = a.f ?? 0, fb = b.f ?? 0;
              if (fa !== fb) return fa - fb;
              const ha = a.h ?? 0, hb = b.h ?? 0;
              if (ha !== hb) return ha - hb;
              return (a.rand ?? 0) - (b.rand ?? 0);
            }
            return 0;
          });
        } else if (dsType === 'queue') {
          simDsItems.push(item);
        } else {
          simDsItems.unshift(item);
        }
      } else if (step.t === "F") {
        step.path.forEach(([x, y]) => {
          cellState[x][y].state = "path";
          cellState[x][y].info = null;
        });
        sCost = step.g;
        sLen = step.path.length - 1;
        isFound = true;
      } else if (step.t === "N") {
        isNoPath = true;
      } else if (step.t === "Err") {
        isErr = true;
      } else if (step.t === "C") {
        for (let r = 0; r < m; r++) {
          for (let c = 0; c < n; c++) {
            cellState[r][c].state = null;
            cellState[r][c].info = null;
          }
        }
        expCount = 0;
        sCost = "—";
        sLen = "—";
      }
    }

    for (let r = 0; r < m; r++) {
      for (let c = 0; c < n; c++) {
        const st = cellState[r][c];
        if (st.state || st.info) {
          const domC = cmpCell(runnerId, r, c);
          if (domC) {
            if (st.state) {
              domC.dataset.state = st.state;
              if (st.state === "vis" && !(r === sx && c === sy) && !(r === tx && c === ty)) {
                domC.style.background = "var(--vis-bg)";
                domC.style.outline = "1px solid var(--vis-border)";
                domC.style.boxShadow = "";
              } else if (st.state === "front" && !(r === sx && c === sy) && !(r === tx && c === ty)) {
                domC.style.background = "var(--front-bg)";
                domC.style.outline = "1px solid var(--front-border)";
                domC.style.boxShadow = "";
              } else if (st.state === "path") {
                const tColors = ["", "var(--t1)", "var(--t2)", "var(--t3)", "var(--t4)"];
                const t = M[r][c];
                const baseColor = t === 0 ? "var(--wall)" : (tColors[t] || "var(--t1)");
                domC.style.background = `linear-gradient(rgba(255,255,255,0.45), rgba(255,255,255,0.45)), ${baseColor}`;
                domC.style.outline = "2.5px solid var(--path-border)";
                domC.style.boxShadow = "0 0 8px rgba(255,255,255,0.6), inset 0 0 3px rgba(255,255,255,0.25)";
              }
            }
            if (st.info) {
              const infoEl = domC.querySelector(".cell-info");
              if (infoEl) {
                infoEl.innerHTML = st.info;
                infoEl.style.display = showCmpLabels() ? "flex" : "none";
              }
            }
          }
        }
      }
    }

    const expEl = document.getElementById(`cmp-exp-${runnerId}`);
    if (expEl) expEl.textContent = expCount;
    const costEl = document.getElementById(`cmp-cost-${runnerId}`);
    if (costEl) costEl.textContent = sCost;
    const lenEl = document.getElementById(`cmp-len-${runnerId}`);
    if (lenEl) lenEl.textContent = sLen;

    const statusEl = document.getElementById(`cmp-status-${runnerId}`);
    if (statusEl) {
      if (isFound) {
        statusEl.textContent = "✓ Found";
        statusEl.className = "cmp-card-status found";
      } else if (isNoPath) {
        statusEl.textContent = "✗ No path";
        statusEl.className = "cmp-card-status nofound";
      } else if (isErr) {
        statusEl.textContent = "⚠ Overloaded";
        statusEl.className = "cmp-card-status nofound";
      } else if (targetIdx >= steps.length) {
        statusEl.textContent = "Done";
        statusEl.className = "cmp-card-status";
      } else {
        statusEl.textContent = "Running…";
        statusEl.className = "cmp-card-status running";
      }
    }

    state.stepIdx = targetIdx;
    state.expandedCount = expCount;
    state.done = (targetIdx >= steps.length) || isFound || isNoPath || isErr;
    state.dsItems = simDsItems;
    state.dsNextId = simDsNextId;
    
    const gridEl = document.getElementById(`cmp-grid-${runnerId}`);
    if (gridEl) {
      if (isFound) gridEl.classList.add("path-found");
      else gridEl.classList.remove("path-found");
    }

    cmpDsRender(runnerId);
    if (state.done) {
      state.result = {
        found: isFound,
        cost: sCost,
        steps: sLen,
        expanded: isErr ? "OOM" : expCount,
      };
    } else {
      state.result = null;
    }

    if (!state.done) allDone = false;

    const total = steps.length;
    const pct = total ? (targetIdx / total) * 100 : 0;
    const pb = document.getElementById(`cmp-pbar-${runnerId}`);
    const pl = document.getElementById(`cmp-plbl-${runnerId}`);
    if (pb) pb.style.width = pct + "%";
    if (pl) pl.textContent = `${targetIdx} / ${total}`;
  });

  cmpGlobalStep = targetGlobalStep;

  if (allDone && targetGlobalStep > 0) {
    showCmpResults();
  }
}

function stepBackCompare() {
  if (!cmpState || Object.keys(cmpState).length === 0) return;
  if (cmpGlobalStep <= 0) return;
  if (cmpRunning) pauseCompare();

  let undoInfo = {};
  cmpRunners.forEach((runner) => {
    const state = cmpState[runner.id];
    if (state) {
      const undoIdx = cmpGlobalStep - 1;
      if (undoIdx >= 0 && undoIdx < state.steps.length) {
        const stepToUndo = state.steps[undoIdx];
        if (stepToUndo.t === "Fr") {
          const item = state.dsItems.find(i => i.x === stepToUndo.x && i.y === stepToUndo.y);
          if (item) {
            const el = document.getElementById(`cmp-ds-item-${runner.id}-${item.id}`);
            if (el) {
              undoInfo[runner.id] = {
                fromRect: el.getBoundingClientRect(),
                html: el.innerHTML
              };
            }
          }
        }
      }
    }
  });

  fastForwardCompareTo(cmpGlobalStep - 1);

  cmpRunners.forEach((runner) => {
    const state = cmpState[runner.id];
    if (!state) return;
    const undoIdx = cmpGlobalStep;
    if (undoIdx < state.steps.length) {
      const stepToUndo = state.steps[undoIdx];
      const cellEl = cmpCell(runner.id, stepToUndo.x, stepToUndo.y);
      if (cellEl) {
        if (stepToUndo.t === "E") {
          const fR = cellEl.getBoundingClientRect();
          let tR;
          const item = state.dsItems.find(i => i.x === stepToUndo.x && i.y === stepToUndo.y);
          if (item) {
            const newEl = document.getElementById(`cmp-ds-item-${runner.id}-${item.id}`);
            if (newEl) tR = newEl.getBoundingClientRect();
          }
          if (!tR) {
            const track = document.getElementById(`cmp-ds-track-${runner.id}`);
            if (track) tR = track.getBoundingClientRect();
          }
          if (tR) {
            spawnFlyingNode(fR, tR, `<div style="font-size:0.5rem;font-weight:bold;">(${stepToUndo.x},${stepToUndo.y})</div>`, true);
          }
        } else if (stepToUndo.t === "Fr" && undoInfo[runner.id]) {
          const tR = cellEl.getBoundingClientRect();
          spawnFlyingNode(undoInfo[runner.id].fromRect, tR, undoInfo[runner.id].html, false);
        }
      }
    }
  });
}

function processCompareStep(runnerId, state, step) {
  const algo = state.algo;
  if (step.t === "E") {
    state.expandedCount++;
    const c = cmpCell(runnerId, step.x, step.y);
    if (
      c &&
      !(step.x === sx && step.y === sy) &&
      !(step.x === tx && step.y === ty)
    ) {
      c.style.background = "var(--vis-bg)";
      c.style.outline = "1px solid var(--vis-border)";
      c.dataset.state = "vis";
      // Clear cell-info on expand
      const info = c.querySelector(".cell-info");
      if (info) {
        info.innerHTML = "";
        info.style.display = "none";
      }
    }
    const expEl = document.getElementById(`cmp-exp-${runnerId}`);
    if (expEl) expEl.textContent = state.expandedCount;
    cmpDsPopItem(runnerId, step);
  } else if (step.t === "Fr") {
    const c = cmpCell(runnerId, step.x, step.y);
    if (
      c &&
      c.dataset.state !== "vis" &&
      !(step.x === sx && step.y === sy) &&
      !(step.x === tx && step.y === ty)
    ) {
      c.style.background = "var(--front-bg)";
      c.style.outline = "1px solid var(--front-border)";
      c.dataset.state = "front";
      // Set cell info
      setCellInfo(c, step, algo);
    }
    cmpDsPushItem(runnerId, step);
  } else if (step.t === "F") {
    const gridEl = document.getElementById(`cmp-grid-${runnerId}`);
    if (gridEl) gridEl.classList.add("path-found");
    step.path.forEach(([x, y], idx) => {
      setTimeout(() => {
        const c = cmpCell(runnerId, x, y);
        if (c) {
          const tColors = ["", "var(--t1)", "var(--t2)", "var(--t3)", "var(--t4)"];
          const t = M[x][y];
          const baseColor = t === 0 ? "var(--wall)" : (tColors[t] || "var(--t1)");
          c.style.background = `linear-gradient(rgba(255,255,255,0.45), rgba(255,255,255,0.45)), ${baseColor}`;
          c.style.outline = "2.5px solid var(--path-border)";
          c.style.boxShadow = "0 0 8px rgba(255,255,255,0.6), inset 0 0 3px rgba(255,255,255,0.25)";
          c.dataset.state = "path";
        }
      }, idx * 25);
    });
    const costEl = document.getElementById(`cmp-cost-${runnerId}`);
    const lenEl = document.getElementById(`cmp-len-${runnerId}`);
    if (costEl) costEl.textContent = step.g;
    if (lenEl) lenEl.textContent = step.path.length - 1;
    const statusEl = document.getElementById(`cmp-status-${runnerId}`);
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
    const statusEl = document.getElementById(`cmp-status-${runnerId}`);
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
  } else if (step.t === "Err") {
    const statusEl = document.getElementById(`cmp-status-${runnerId}`);
    if (statusEl) {
      statusEl.textContent = "⚠ Overloaded";
      statusEl.className = "cmp-card-status nofound";
    }
    state.done = true;
    state.result = {
      found: false,
      cost: "—",
      steps: "—",
      expanded: "OOM",
    };
  } else if (step.t === "I") {
    // IDS depth marker — just skip in compare mode
  } else if (step.t === "C") {
    clearCompareVis(runnerId);
  }
}

function clearCompareVis(runnerId) {
  const gridEl = document.getElementById(`cmp-grid-${runnerId}`);
  if (gridEl) gridEl.classList.remove("path-found");
  const tColors = ["", "var(--t1)", "var(--t2)", "var(--t3)", "var(--t4)"];
  for (let i = 0; i < m; i++) {
    for (let j = 0; j < n; j++) {
      const c = cmpCell(runnerId, i, j);
      if (c) {
        if (c.dataset.state) {
          delete c.dataset.state;
          const t = M[i][j];
          c.style.boxShadow = "";
          if (i === sx && j === sy) {
            c.style.background = tColors[t] || "var(--t1)";
            c.style.outline = "";
          } else if (i === tx && j === ty) {
            c.style.background = tColors[t] || "var(--t1)";
            c.style.outline = "";
          } else {
            c.style.background =
              t === 0 ? "var(--wall)" : tColors[t] || "var(--t1)";
            c.style.outline = "";
          }
        }
        // Always clear cell-info text and display
        const info = c.querySelector(".cell-info");
        if (info) {
          info.innerHTML = "";
          info.style.display = "none";
        }
      }
    }
  }
}

function stopCompare() {
  if (cmpTimer) {
    clearTimeout(cmpTimer);
    cmpTimer = null;
  }
  cmpRunning = false;
  const btn = document.getElementById("btn-cmp-run");
  if (btn) {
    btn.textContent = "▶ Chạy so sánh";
    btn.onclick = runComparison;
    btn.disabled = false;
  }
  const btnPause = document.getElementById("btn-cmp-pause");
  if (btnPause) btnPause.disabled = true;
}

function resetComparison() {
  stopCompare();
  cmpState = {};
  cmpGlobalStep = 0;
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
    .map(([runnerId, s]) => {
      const runner = cmpRunners.find((r) => r.id === runnerId) || { id: runnerId, algo: s.algo };
      return { runner, ...s.result };
    });

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

  // Sort: found first, then by cost asc, then by steps asc, then by expanded asc
  results.sort((a, b) => {
    if (a.found && !b.found) return -1;
    if (!a.found && b.found) return 1;
    if (a.found && b.found) {
      if (a.cost !== b.cost) return a.cost - b.cost;
      if (a.steps !== b.steps) return a.steps - b.steps;
      return a.expanded - b.expanded;
    }
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
            <th>Số nút đã duyệt</th>
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
        const displayName = getRunnerDisplayName(r.runner);
        return `
              <tr class="${rowClass}">
                <td>
                  <span class="cmp-algo-badge sm" style="background:${CMP_COLORS[r.runner.algo]}">${r.runner.algo === "Astar" ? "A★" : r.runner.algo}</span>
                  <span style="font-size:0.75rem;font-weight:600;color:var(--text-main);margin-left:4px">${displayName}</span>
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
    a === "DLS" || a === "IDS" ? "block" : "none";
  document.getElementById("h-p").style.display =
    a === "GBFS" || a === "Astar" ? "block" : "none";
  document.getElementById("algo-desc").textContent = ALGO_DESC[a] || "";
  document
    .querySelectorAll(".pill")
    .forEach((p) => p.classList.toggle("active", p.dataset.a === a));

  if (a === "IDS") {
    const dlsAuto = document.getElementById("dls-auto");
    const dlsLim = document.getElementById("dls-lim");
    if (dlsAuto) dlsAuto.checked = true;
    if (dlsLim) dlsLim.disabled = true;
  }
}

// Helper để pill header đồng bộ với dropdown và gọi onAlgoChange
function selectAlgo(name) {
  const sel = document.getElementById("algo");
  if (sel) { sel.value = name; }
  onAlgoChange();
}

// ═══════════════════════════════════════
//  COMPARE MODE DS VISUALIZER
// ═══════════════════════════════════════
function cmpDsPushItem(runnerId, step) {
  const state = cmpState[runnerId];
  if (!state || state.done) return;
  const algo = state.algo;
  
  let dsType = 'pqueue';
  if (algo === 'BFS') dsType = 'queue';
  else if (['DFS','DLS','IDS'].includes(algo)) dsType = 'stack';

  const item = {
    id: state.dsNextId++,
    x: step.x, y: step.y,
    g: step.g, h: step.h, f: step.f, d: step.d, rand: step.rand
  };

  if (state.dsSortTimer) {
    clearTimeout(state.dsSortTimer);
    state.dsSortTimer = null;
    state.dsPendingSortId = null;
  }

  if (dsType === 'pqueue') {
    const eIdx = state.dsItems.findIndex(i => i.x === item.x && i.y === item.y);
    if (eIdx !== -1) {
      item.id = state.dsItems[eIdx].id;
      state.dsItems[eIdx] = item;
    } else {
      state.dsItems.push(item);
    }
    
    state.dsItems.sort((a, b) => {
      if (algo === 'UCS')   return (a.g ?? 0) - (b.g ?? 0);
      if (algo === 'GBFS')  return (a.h ?? 0) - (b.h ?? 0);
      if (algo === 'Astar') {
        const fa = a.f ?? 0, fb = b.f ?? 0;
        if (fa !== fb) return fa - fb;
        const ha = a.h ?? 0, hb = b.h ?? 0;
        if (ha !== hb) return ha - hb;
        return (a.rand ?? 0) - (b.rand ?? 0);
      }
      return 0;
    });
    state.dsPendingSortId = item.id;
  } else if (dsType === 'queue') {
    state.dsItems.push(item);
  } else {
    state.dsItems.unshift(item);
  }

  cmpDsRender(runnerId, item.id);

  // Flying animation
  const cellEl = document.getElementById(`${runnerId}_c_${step.x}_${step.y}`);
  const track = document.getElementById(`cmp-ds-track-${runnerId}`);
  if (cellEl && track) {
    const fR = cellEl.getBoundingClientRect();
    let targetRect;
    const newEl = document.getElementById(`cmp-ds-item-${runnerId}-${item.id}`);
    
    if (newEl) {
      targetRect = newEl.getBoundingClientRect();
    } else {
      const tR = track.getBoundingClientRect();
      targetRect = {
        left: tR.left + tR.width / 2 - 12,
        top: tR.top + tR.height / 2 - 12,
        width: 24,
        height: 24
      };
    }
    
    spawnFlyingNode(fR, targetRect, `<div style="font-size:0.5rem;font-weight:bold;">(${step.x},${step.y})</div>`, true);
  }

  if (dsType === 'pqueue') {
    state.dsSortTimer = setTimeout(() => {
      state.dsPendingSortId = null;
      cmpDsRender(runnerId, null, { useFlip: true });
    }, 600);
  }
}

function cmpDsPopItem(runnerId, step) {
  const state = cmpState[runnerId];
  if (!state || state.done) return;
  const idx = state.dsItems.findIndex(i => i.x === step.x && i.y === step.y);
  if (idx === -1) { cmpDsRender(runnerId, null); return; }

  const track = document.getElementById(`cmp-ds-track-${runnerId}`);
  const MAX = 4;
  if (track && idx < MAX) {
    const itemEls = track.querySelectorAll('.ds-item');
    const el = itemEls[idx];
    if (el) {
      const fR = el.getBoundingClientRect();
      const cellEl = document.getElementById(`${runnerId}_c_${step.x}_${step.y}`);
      const tR = cellEl ? cellEl.getBoundingClientRect() : fR;
      spawnFlyingNode(fR, tR, el.innerHTML, false);

      el.classList.remove('ds-next', 'ds-enter-l', 'ds-enter-r', 'ds-enter-d');
      el.classList.add('ds-leaving');
      state.dsPopPending = true;
      state.dsItems.splice(idx, 1);
      setTimeout(() => {
        state.dsPopPending = false;
        cmpDsRender(runnerId, null);
      }, 300);
      return;
    }
  }
  
  state.dsItems.splice(idx, 1);
  cmpDsRender(runnerId, null);
}

function cmpDsRender(runnerId, newItemId = null, opts = {}) {
  const state = cmpState[runnerId];
  if (!state || state.dsPopPending) return;
  
  const track = document.getElementById(`cmp-ds-track-${runnerId}`);
  const badge = document.getElementById(`cmp-ds-sz-${runnerId}`);
  const lbl = document.getElementById(`cmp-ds-lbl-${runnerId}`);
  if (!track) return;
  
  if (badge) badge.textContent = state.dsItems.length;

  const algo = state.algo;
  let dsType = 'pqueue';
  if (algo === 'BFS') dsType = 'queue';
  else if (['DFS','DLS','IDS'].includes(algo)) dsType = 'stack';

  if (lbl) {
    if (dsType === 'queue') {
      lbl.textContent = 'Queue (FIFO)';
    } else if (dsType === 'stack') {
      lbl.textContent = 'Stack (LIFO)';
    } else {
      const nm = { UCS:'g', GBFS:'h', Astar:'f' };
      lbl.textContent = `PQ (min-${nm[algo]||'?'})`;
    }
  }

  if (state.dsItems.length === 0) {
    track.innerHTML = '<span class="ds-empty" style="font-size: 0.55rem">empty</span>';
    return;
  }

  const oldRects = new Map();
  if (opts.useFlip) {
    track.querySelectorAll('.ds-item').forEach(el => {
      const match = el.id.match(/cmp-ds-item-.*?-(\d+)/);
      if (match) oldRects.set(parseInt(match[1]), el.getBoundingClientRect());
    });
  }

  const MAX = 4;
  let visible = state.dsItems.slice(0, MAX);
  let overflow = state.dsItems.length - MAX;

  if (state.dsPendingSortId != null) {
    const pId = state.dsPendingSortId;
    const pIdx = visible.findIndex(i => i.id === pId);
    if (pIdx !== -1) {
      const [pItem] = visible.splice(pIdx, 1);
      visible.push(pItem);
    } else {
      const realIdx = state.dsItems.findIndex(i => i.id === pId);
      if (realIdx !== -1) {
        visible.push(state.dsItems[realIdx]);
        if (visible.length > MAX) visible.splice(MAX - 1, 1);
      }
    }
  }

  track.innerHTML = '';
  visible.forEach((item, idx) => {
    const isNew = (item.id === newItemId);
    const isNext = (idx === 0);

    let enterCls = '';
    if (isNew) {
      if (dsType === 'queue') enterCls = ' ds-enter-r';
      else if (dsType === 'stack') enterCls = ' ds-enter-l';
      else enterCls = ' ds-enter-d';
    }

    const el = document.createElement('div');
    el.className = 'ds-item' + (isNext ? ' ds-next' : '') + enterCls;
    el.id = `cmp-ds-item-${runnerId}-${item.id}`;
    el.style.minWidth = '24px';
    el.style.padding = '2px 4px';
    el.style.fontSize = '0.55rem';
    el.style.margin = '0 2px';

    const fmt = v => (v !== undefined && v !== null) ? +v.toFixed(1) : '?';
    let valStr = '';
    if (algo === 'BFS' || algo === 'DFS' || algo === 'DLS' || algo === 'IDS') valStr = `d:${item.d ?? '?'}`;
    else if (algo === 'UCS') valStr = `g:${fmt(item.g)}`;
    else if (algo === 'GBFS') valStr = `h:${fmt(item.h)}`;
    else if (algo === 'Astar') valStr = `f:${fmt(item.f)}`;

    el.innerHTML = `<div style="color:var(--text-main);font-weight:600">(${item.x},${item.y})</div>
                    ${valStr ? `<div style="color:var(--muted);font-size:0.45rem">${valStr}</div>` : ''}`;
    track.appendChild(el);

    if (idx < visible.length - 1 || overflow > 0) {
      const sep = document.createElement('span');
      sep.className = 'ds-sep';
      sep.style.margin = '0 1px';
      sep.textContent = '›';
      track.appendChild(sep);
    }
  });

  if (overflow > 0) {
    const more = document.createElement('div');
    more.className = 'ds-more';
    more.style.fontSize = '0.5rem';
    more.textContent = `+${overflow}`;
    track.appendChild(more);
  }

  if (opts.useFlip) {
    const newEls = Array.from(track.querySelectorAll('.ds-item'));
    newEls.forEach(el => {
      const match = el.id.match(/cmp-ds-item-.*?-(\d+)/);
      if (match) {
        const oldRect = oldRects.get(parseInt(match[1]));
        if (oldRect) {
          const newRect = el.getBoundingClientRect();
          const dx = oldRect.left - newRect.left;
          const dy = oldRect.top - newRect.top;
          if (dx !== 0 || dy !== 0) {
            el.style.transition = 'none';
            el.style.transform = `translate(${dx}px, ${dy}px)`;
            el.style.zIndex = '100';
            requestAnimationFrame(() => {
              requestAnimationFrame(() => {
                el.style.transition = 'transform 300ms cubic-bezier(0.34, 1.56, 0.64, 1)';
                el.style.transform = 'none';
                setTimeout(() => { el.style.zIndex = ''; }, 300);
              });
            });
          }
        }
      }
    });
  }
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
