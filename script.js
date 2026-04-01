 // ═══════════════════════════════════════════════════
      //  STATE
      // ═══════════════════════════════════════════════════
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

      const TCOLORS = ["", "#D0EAD4", "#FBF2C0", "#FFE0B2", "#FDDCDC"];
      const TNAMES = ["", "T1 · Grass", "T2 · Sand", "T3 · Rock", "T4 · Swamp"];

      const dx = [-1, 0, 1, 0],
        dy = [0, 1, 0, -1];

      const ALGO_DESC = {
        BFS: "Explores all nodes layer by layer. Guarantees shortest path (by steps) on unweighted graphs.",
        DFS: "Dives deep along one branch before backtracking. Fast but not optimal.",
        UCS: "Expands lowest-cost node first (Dijkstra). Guarantees optimal cost path.",
        DLS: "DFS with a hard depth limit. Useful when solution depth is known.",
        IDS: "Repeatedly runs DLS with increasing depth limits. Optimal + memory-efficient.",
        GBFS: "Greedy: follows heuristic only. Fast but not always optimal.",
        Astar:
          "Combines cost (g) + heuristic (h). Optimal and efficient with admissible h.",
      };

      // ═══════════════════════════════════════════════════
      //  GRID BUILD
      // ═══════════════════════════════════════════════════
      function buildGrid() {
        m = parseInt(document.getElementById("rows").value) || 10;
        n = parseInt(document.getElementById("cols").value) || 12;
        tcnt = parseInt(document.getElementById("tcnt").value) || 2;
        tcnt = Math.max(1, Math.min(4, tcnt));
        M = Array.from({ length: m }, () => new Array(n).fill(1));
        sx = 0;
        sy = 0;
        tx = m - 1;
        ty = n - 1;
        updateTerrainUI();
        updateDrawModes();
        renderGrid();
        clearVis();
      }

      function updateTerrainUI() {
        tcnt = Math.max(
          1,
          Math.min(4, parseInt(document.getElementById("tcnt").value) || 2),
        );
        const c = document.getElementById("terrain-ui");
        c.innerHTML = "";
        for (let i = 1; i <= tcnt; i++) {
          const row = document.createElement("div");
          row.className = "tc-row";
          row.innerHTML = `
      <div class="tc-swatch" style="background:${TCOLORS[i]}"></div>
      <label>T${i}</label>
      <input type="number" value="${w[i] || i}" min="1" max="999"
        style="width:60px;margin:0"
        onchange="w[${i}]=Math.max(1,parseInt(this.value)||1)">
    `;
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
          modes.push({ id: `t${i}`, label: `T${i}`, color: TCOLORS[i] });
        modes.forEach((mo) => {
          const b = document.createElement("div");
          b.className = "dm-btn" + (drawMode === mo.id ? " active" : "");
          b.textContent = mo.label;
          if (mo.color) b.style.borderLeftColor = mo.color;
          b.onclick = () => setMode(mo.id);
          c.appendChild(b);
        });
      }

      function setMode(id) {
        drawMode = id;
        updateDrawModes();
      }

      function renderGrid() {
        const el = document.getElementById("grid");
        const avW = Math.floor((window.innerWidth * 0.5 - 32) / n);
        const avH = Math.floor((window.innerHeight * 0.72 - 32) / m);
        const cs = Math.max(16, Math.min(44, Math.min(avW, avH)));
        el.style.gridTemplateColumns = `repeat(${n},${cs}px)`;
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
              paint(i, j);
              e.preventDefault();
            });
            d.addEventListener("mousemove", () => {
              if (isDown) paint(i, j);
            });
            el.appendChild(d);
            paintCell(i, j);
          }
        }
        document.addEventListener("mouseup", () => (isDown = false));
      }

      function paintCell(i, j) {
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

      function paint(i, j) {
        if (running) return;
        const old_sx = sx,
          old_sy = sy,
          old_tx = tx,
          old_ty = ty;
        if (drawMode === "wall") {
          if ((i === sx && j === sy) || (i === tx && j === ty)) return;
          M[i][j] = 0;
        } else if (drawMode === "erase") {
          M[i][j] = 1;
        } else if (drawMode === "start") {
          sx = i;
          sy = j;
          if (M[i][j] === 0) M[i][j] = 1;
          paintCell(old_sx, old_sy);
        } else if (drawMode === "end") {
          tx = i;
          ty = j;
          if (M[i][j] === 0) M[i][j] = 1;
          paintCell(old_tx, old_ty);
        } else if (drawMode.startsWith("t")) {
          const t = parseInt(drawMode.slice(1));
          if ((i === sx && j === sy) || (i === tx && j === ty)) return;
          M[i][j] = t;
        }
        paintCell(i, j);
      }

      // ═══════════════════════════════════════════════════
      //  RANDOM MAZE
      // ═══════════════════════════════════════════════════
      function genRandom() {
        clearVis();
        for (let i = 0; i < m; i++) {
          for (let j = 0; j < n; j++) {
            if ((i === sx && j === sy) || (i === tx && j === ty)) continue;
            const r = Math.random();
            if (r < 0.22) M[i][j] = 0;
            else {
              M[i][j] = Math.ceil(Math.random() * tcnt);
            }
          }
        }
        // ensure connectivity from sx,sy: carve a guaranteed path
        let cx = sx,
          cy = sy;
        while (cx !== tx || cy !== ty) {
          if (cx < tx && Math.random() > 0.4) {
            M[cx + 1][cy] = M[cx + 1][cy] || 1;
            cx++;
          } else if (cy < ty && Math.random() > 0.4) {
            M[cx][cy + 1] = M[cx][cy + 1] || 1;
            cy++;
          } else if (cx < tx) {
            M[cx + 1][cy] = M[cx + 1][cy] || 1;
            cx++;
          } else {
            M[cx][cy + 1] = M[cx][cy + 1] || 1;
            cy++;
          }
        }
        renderGrid();
      }

      // ═══════════════════════════════════════════════════
      //  MIN-HEAP
      // ═══════════════════════════════════════════════════
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

      // ═══════════════════════════════════════════════════
      //  HEURISTIC
      // ═══════════════════════════════════════════════════
      function H(x, y) {
        const ht = parseInt(document.getElementById("htype").value);
        const mul = parseFloat(document.getElementById("hmul").value) || 1;
        const wmin = Math.min(...w.slice(1, tcnt + 1));
        const ddx = Math.abs(x - tx),
          ddy = Math.abs(y - ty);
        if (ht === 1) return Math.sqrt(ddx * ddx + ddy * ddy) * wmin * mul;
        return (ddx + ddy) * wmin * mul;
      }

      function traceP(par) {
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

      // ═══════════════════════════════════════════════════
      //  ALGORITHMS  → return steps[]
      // ═══════════════════════════════════════════════════
      function algoBFS() {
        const st = [];
        const vis = A2(false),
          par = A2(null);
        const q = [{ x: sx, y: sy, d: 0, g: 0 }];
        vis[sx][sy] = true;
        par[sx][sy] = [sx, sy];
        while (q.length) {
          const c = q.shift();
          const { x, y } = c;
          st.push({ t: "E", x, y, g: c.g, d: c.d });
          if (x === tx && y === ty) {
            st.push({ t: "F", path: traceP(par), g: c.g });
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
        const st = [];
        const vis = A2(false),
          par = A2(null);
        const stack = [{ x: sx, y: sy, d: 0, g: 0 }];
        vis[sx][sy] = true;
        par[sx][sy] = [sx, sy];
        while (stack.length) {
          const c = stack.pop();
          const { x, y } = c;
          st.push({ t: "E", x, y, g: c.g, d: c.d });
          if (x === tx && y === ty) {
            st.push({ t: "F", path: traceP(par), g: c.g });
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
        const st = [];
        const dis = A2(Infinity),
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
            st.push({ t: "F", path: traceP(par), g: c.g });
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
        const st = [];
        const dmin = A2(Infinity),
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
            st.push({ t: "F", path: traceP(par), g: c.g });
            return { st, found: true, reachedLim };
          }
          for (let i = 0; i < 4; i++) {
            const nx = x + dx[i],
              ny = y + dy[i];
            const nd = c.d + 1;
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
        const all = [];
        const cap = m * n;
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
        const st = [];
        const vis = A2(false),
          par = A2(null);
        const pq = new Heap((a, b) => a.f - b.f);
        vis[sx][sy] = true;
        par[sx][sy] = [sx, sy];
        pq.push({ x: sx, y: sy, d: 0, g: 0, f: H(sx, sy) });
        while (!pq.empty()) {
          const c = pq.pop();
          const { x, y } = c;
          st.push({ t: "E", x, y, g: c.g, d: c.d });
          if (x === tx && y === ty) {
            st.push({ t: "F", path: traceP(par), g: c.g });
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
                f: H(nx, ny),
              });
              st.push({ t: "Fr", x: nx, y: ny });
            }
          }
        }
        st.push({ t: "N" });
        return st;
      }

      function algoAstar() {
        const st = [];
        const dis = A2(Infinity),
          par = A2(null);
        const pq = new Heap((a, b) => a.f - b.f);
        dis[sx][sy] = 0;
        par[sx][sy] = [sx, sy];
        pq.push({ x: sx, y: sy, d: 0, g: 0, f: H(sx, sy) });
        while (!pq.empty()) {
          const c = pq.pop();
          const { x, y } = c;
          if (c.g > dis[x][y]) continue;
          st.push({ t: "E", x, y, g: c.g, d: c.d });
          if (x === tx && y === ty) {
            st.push({ t: "F", path: traceP(par), g: c.g });
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
                pq.push({ x: nx, y: ny, d: c.d + 1, g: ng, f: ng + H(nx, ny) });
                st.push({ t: "Fr", x: nx, y: ny });
              }
            }
          }
        }
        st.push({ t: "N" });
        return st;
      }

      function genSteps() {
        // refresh terrain costs from UI
        document
          .querySelectorAll("#terrain-ui .tc-row input")
          .forEach((inp, i) => {
            w[i + 1] = Math.max(1, parseInt(inp.value) || 1);
          });
        const a = document.getElementById("algo").value;
        if (a === "BFS") return algoBFS();
        if (a === "DFS") return algoDFS();
        if (a === "UCS") return algoUCS();
        if (a === "DLS")
          return algoDLS(
            parseInt(document.getElementById("dls-lim").value) || 10,
          ).st;
        if (a === "IDS") return algoIDS();
        if (a === "GBFS") return algoGBFS();
        if (a === "Astar") return algoAstar();
      }

      // ═══════════════════════════════════════════════════
      //  VISUALIZATION CONTROLS
      // ═══════════════════════════════════════════════════
      function startRun() {
        clearVis();
        steps = genSteps();
        stepIdx = 0;
        running = true;
        document.getElementById("btn-run").disabled = true;
        document.getElementById("btn-step").disabled = true;
        document.getElementById("btn-pause").disabled = false;
        document.getElementById("s-depth-card").style.display =
          document.getElementById("algo").value === "IDS" ? "block" : "none";
        animate();
      }

      function animate() {
        if (stepIdx >= steps.length) {
          running = false;
          document.getElementById("btn-run").disabled = false;
          document.getElementById("btn-step").disabled = false;
          document.getElementById("btn-pause").disabled = true;
          return;
        }
        doStep(steps[stepIdx++]);
        updatePBar();
        timer = setTimeout(animate, getDelay());
      }

      function getDelay() {
        const s = parseInt(document.getElementById("speed").value);
        return [600, 350, 220, 150, 100, 65, 40, 22, 10, 2][s - 1];
      }

      function stepRun() {
        if (!steps.length) {
          steps = genSteps();
          stepIdx = 0;
          document.getElementById("s-depth-card").style.display =
            document.getElementById("algo").value === "IDS" ? "block" : "none";
        }
        if (stepIdx >= steps.length) return;
        doStep(steps[stepIdx++]);
        updatePBar();
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
        animate();
      }

      function doReset() {
        if (timer) clearTimeout(timer);
        timer = null;
        running = false;
        const btn = document.getElementById("btn-run");
        btn.textContent = "▶ Run";
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
          c.querySelector(".cell-ov").style.background = "";
        });
        document.getElementById("log").innerHTML = "";
        document.getElementById("s-exp").textContent = "—";
        document.getElementById("s-cost").textContent = "—";
        document.getElementById("s-len").textContent = "—";
        document.getElementById("pbar").style.width = "0%";
        document.getElementById("pbar-lbl").textContent = "Step 0 / 0";
        document.getElementById("btn-run").disabled = false;
        document.getElementById("btn-step").disabled = false;
        document.getElementById("btn-pause").disabled = true;
      }

      function clearAll() {
        clearVis();
        M = Array.from({ length: m }, () => new Array(n).fill(1));
        renderGrid();
      }

      // ═══════════════════════════════════════════════════
      //  STEP PROCESSOR
      // ═══════════════════════════════════════════════════
      function doStep(step) {
        if (step.t === "E") {
          expandedCount++;
          const c = cell(step.x, step.y);
          if (c) {
            c.classList.remove("front");
            c.classList.add("vis", "popping");
            setTimeout(() => c.classList.remove("popping"), 350);
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
              setTimeout(() => c.classList.remove("pathpop"), 280);
            }, idx * 38);
          });
          document.getElementById("s-cost").textContent = step.g;
          document.getElementById("s-len").textContent = step.path.length - 1;
          addLog(
            "found",
            `✓ Path found! Cost=${step.g}  Steps=${step.path.length - 1}`,
          );
        } else if (step.t === "N") {
          addLog("nofound", "✗ No path found");
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

      function updatePBar() {
        const p = steps.length ? (stepIdx / steps.length) * 100 : 0;
        document.getElementById("pbar").style.width = p + "%";
        document.getElementById("pbar-lbl").textContent =
          `Step ${stepIdx} / ${steps.length}`;
      }

      // ═══════════════════════════════════════════════════
      //  UI HELPERS
      // ═══════════════════════════════════════════════════
      function onAlgoChange() {
        const a = document.getElementById("algo").value;
        document.getElementById("dls-p").style.display =
          a === "DLS" ? "block" : "none";
        document.getElementById("h-p").style.display =
          a === "GBFS" || a === "Astar" ? "block" : "none";
        document.getElementById("algo-desc").textContent = ALGO_DESC[a] || "";
        // update header pill
        document.querySelectorAll(".pill").forEach((p) => {
          p.classList.toggle("active-algo", p.dataset.a === a);
        });
      }

      function cell(x, y) {
        return document.getElementById(`c_${x}_${y}`);
      }
      function ok(x, y) {
        return x >= 0 && y >= 0 && x < m && y < n;
      }
      function A2(v) {
        return Array.from({ length: m }, () => new Array(n).fill(v));
      }

      // ═══════════════════════════════════════════════════
      //  INIT
      // ═══════════════════════════════════════════════════
      buildGrid();
      onAlgoChange();
      // draw a small default demo pattern
      (function demo() {
        // a few walls in the middle
        const mid = Math.floor(m / 2);
        for (let j = 2; j < n - 2; j++)
          if (j !== Math.floor(n / 2)) M[mid][j] = 0;
        // some terrain variety
        for (let i = 0; i < m; i++)
          for (let j = 0; j < n; j++) {
            if (
              M[i][j] !== 0 &&
              !((i === sx && j === sy) || (i === tx && j === ty))
            ) {
              if (i > m / 2)
                M[i][j] = Math.ceil(Math.random() * Math.min(tcnt, 2));
            }
          }
        renderGrid();
      })();
