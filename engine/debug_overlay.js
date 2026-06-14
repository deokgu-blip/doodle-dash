// engine/debug_overlay.js
// LEFT-TOP REAL-TIME DEBUG OVERLAY (display only — ZERO gameplay/physics/render-behaviour
// change). Shows the user (iPhone 17 Pro "random, different-place-each-run stutter" report)
// WHAT is actually happening, with windows long enough to CATCH a brief spike that a 1s
// MAX misses (the spike passes through the 1s window before the eye/finger reports it):
//
//   • FPS        — ACHIEVED render FPS. We cap render at 60fps (game.startLoop's RENDER_DT
//                  gate), so the IDEAL is 60; a render over its 16.67ms budget drops the
//                  achieved FPS below 60. Averaged over the most recent ~0.5s window.
//   • frame ms   — render-to-render gap avg / MAX over the most recent ~1s window. The 1s
//                  MAX resets fast, so a brief spike that lands between two reads is lost —
//                  hence worst5s + jank below.
//   • worst5s    — the worst (largest) render-gap ms over the most recent 5s window. A brief
//                  stutter stays in this number for ~5s, long enough for the eye to register
//                  it AND for a screenshot to capture it (the 1s MAX would already be gone).
//   • jank       — COUNT of frames over 25ms in the most recent 10s window. Even after a
//                  spike fully passes out of the gap windows, "it janked N times in 10s"
//                  proves the random hitch happened (and how often) — the key evidence the
//                  60/17 max overlay was hiding.
//   • heap       — performance.memory.usedJSHeapSize in MB + a trend arrow (▲ rising / ▼
//                  falling / = flat) between reads. A steady sawtooth-rise → a GC sweep is
//                  coming (the GC pause is a prime random-hitch suspect). 'n/a' where the
//                  API is absent (non-Chromium / iOS Safari).
//   • raf        — the ACTUAL requestAnimationFrame callback interval (ms), smoothed. On a
//                  120Hz panel this is ~8.3, on 60Hz ~16.7. Compared against the render gap
//                  (~16.7 due to the 60fps cap) it exposes a CAP↔REFRESH beat mismatch
//                  (judder): if raf≈8.3 but renders fire on an uneven 2,2,3,2… frame cadence
//                  the eye sees micro-stutter even at "60fps".
//   • calls/tris — three renderer.info.render.calls / .triangles that frame.
//   • mem        — three renderer.info.memory.geometries / .textures (a creeping count = a leak).
//
// PERF (the overlay must NOT itself cause the jank it measures):
//   • frame() and raf() are called every render / every rAF and only push ONE number into a
//     fixed pre-sized ring buffer (no allocation, no string build, no DOM touch) — O(1).
//   • The text string + the single DOM write happen on a THROTTLE (~4×/s), so the overlay
//     never dirties layout per frame. heap is read only at that throttle (also not per frame).
//   • Ring buffers are pre-sized for the LONGEST window (10s @ 144fps); nothing is allocated
//     per frame and the single-pass recompute fills every window in one walk.

const FPS_WINDOW_MS = 500;     // achieved-FPS average window (~0.5s)
const GAP_WINDOW_MS = 1000;    // frame-ms avg/max window (~1s)
const WORST_WINDOW_MS = 5000;  // worst-frame (max gap) window (~5s) — holds a brief spike
const JANK_WINDOW_MS = 10000;  // jank-count window (~10s)
const JANK_MS = 25;            // a frame longer than this counts as a jank
const DRAW_INTERVAL_MS = 250;  // DOM text refresh throttle (~4×/s)
// ring capacity must cover the LONGEST window at the highest refresh: 10s × 144fps = 1440.
// (At the 60fps render cap renders are ~half that; rAF samples can hit 144 — size for it.)
const RING = 1536;             // ring-buffer capacity (≥ 10s of 144fps samples)
const RAF_EMA = 0.1;           // smoothing factor for the raf interval EMA (low-pass)

export class DebugOverlay {
  /**
   * @param {HTMLElement|null} el         the fixed top-left DOM element (null in headless)
   * @param {() => (object|null)} infoFn  returns the live three renderer.info (or null)
   */
  constructor(el, infoFn) {
    this.el = el || null;
    this._infoFn = typeof infoFn === 'function' ? infoFn : () => null;

    // ring buffer of render-gap samples: parallel arrays {gapMs, t} — fixed size, no GC.
    this._gap = new Float64Array(RING);
    this._tAt = new Float64Array(RING);
    this._head = 0;        // next write index
    this._count = 0;       // valid samples (≤ RING)

    this._lastRenderT = 0; // performance.now() of the previous frame() call
    this._lastDrawT = 0;   // last time we wrote the DOM text
    this._frames = 0;      // total frames observed (sanity)

    // RAF interval (display refresh) — fed every rAF via raf(), independent of the render
    // cap. EMA-smoothed so the displayed ms is steady (no per-frame jitter readout). O(1).
    this._lastRafT = 0;
    this._rafEma = 0;      // smoothed rAF callback interval (ms); 0 until first measured

    // heap trend: last read used-heap (MB) so we can show ▲/▼/= without storing history.
    this._lastHeapMB = null;

    // latest computed stats (also readable headless via stats()).
    this._stats = {
      fps: 0, avgMs: 0, maxMs: 0, worst5s: 0, jank: 0,
      heapMB: null, heapTrend: '=', rafMs: 0,
      calls: 0, tris: 0, geo: 0, tex: 0,
    };
  }

  /** Wall-clock now. Overridable so headless tests can feed a deterministic clock. */
  _now() {
    return (typeof performance !== 'undefined' && performance.now) ? performance.now() : Date.now();
  }

  /** Call EXACTLY ONCE per real render (right after renderer.render()). O(1): records the
   * real render-to-render gap into the ring and throttle-writes the text. `nowMs` may be
   * supplied (RAF timestamp / test clock); falls back to performance.now(). */
  frame(nowMs) {
    const t = (nowMs == null) ? this._now() : nowMs;
    this._frames++;
    if (this._lastRenderT) {
      let gap = t - this._lastRenderT;
      if (gap < 0) gap = 0;
      // record the sample (overwrite oldest — no allocation).
      this._gap[this._head] = gap;
      this._tAt[this._head] = t;
      this._head = (this._head + 1) % RING;
      if (this._count < RING) this._count++;
    }
    this._lastRenderT = t;

    // THROTTLE the expensive part (string build + DOM write + heap read) to ~4×/s.
    if (t - this._lastDrawT >= DRAW_INTERVAL_MS) {
      this._lastDrawT = t;
      this._recompute(t);
      this._write();
    }
  }

  /** Call EXACTLY ONCE per requestAnimationFrame callback (BEFORE the render cap gate), so
   * we measure the TRUE display-refresh interval regardless of how often we render. O(1):
   * just EMA-update the smoothed interval. `nowMs` is the rAF timestamp (or test clock). */
  raf(nowMs) {
    const t = (nowMs == null) ? this._now() : nowMs;
    if (this._lastRafT) {
      let dt = t - this._lastRafT;
      if (dt < 0) dt = 0;
      if (dt > 100) dt = 100; // ignore tab-switch stalls (not a refresh interval)
      // EMA low-pass: steady readout, no per-frame jitter. Seed on the first sample.
      this._rafEma = this._rafEma ? (this._rafEma + (dt - this._rafEma) * RAF_EMA) : dt;
    }
    this._lastRafT = t;
  }

  /** Recompute all windows in ONE pass over the ring (no allocation):
   *   • avg/max gap (1s), achieved FPS (0.5s), worst gap (5s), jank count (>25ms, 10s).
   * Then pull the live three info, the smoothed rAF interval, and the heap (+trend). */
  _recompute(t) {
    let sumAll = 0, nAll = 0, maxAll = 0;   // GAP_WINDOW_MS → avg/max ms
    let nFps = 0;                            // FPS_WINDOW_MS → achieved fps
    let worst = 0;                           // WORST_WINDOW_MS → worst gap ms
    let jank = 0;                            // JANK_WINDOW_MS → count of >JANK_MS frames
    const cutGap = t - GAP_WINDOW_MS;
    const cutFps = t - FPS_WINDOW_MS;
    const cutWorst = t - WORST_WINDOW_MS;
    const cutJank = t - JANK_WINDOW_MS;      // the LONGEST window — bounds the walk
    // walk the valid samples newest→oldest (no array copy / sort). Stop at the longest
    // (jank) window edge; each sample then contributes to whichever shorter windows cover it.
    for (let k = 0; k < this._count; k++) {
      let i = this._head - 1 - k;
      if (i < 0) i += RING;
      const at = this._tAt[i];
      if (at < cutJank) break;               // older than every window ⇒ stop
      const g = this._gap[i];
      if (at >= cutJank && g > JANK_MS) jank++;
      if (at >= cutWorst && g > worst) worst = g;
      if (at >= cutGap) { sumAll += g; nAll++; if (g > maxAll) maxAll = g; }
      if (at >= cutFps) nFps++;
    }
    const s = this._stats;
    s.avgMs = nAll ? (sumAll / nAll) : 0;
    s.maxMs = maxAll;
    s.worst5s = worst;
    s.jank = jank;
    // achieved FPS = renders in the last FPS window / window-seconds.
    s.fps = nFps ? (nFps / (FPS_WINDOW_MS / 1000)) : 0;
    // rAF interval (display refresh), smoothed.
    s.rafMs = this._rafEma || 0;

    // HEAP (Chromium only) + trend vs the previous read. Read here (throttled), not per frame.
    const mem = (typeof performance !== 'undefined') ? performance.memory : null;
    if (mem && typeof mem.usedJSHeapSize === 'number') {
      const mb = mem.usedJSHeapSize / (1024 * 1024);
      if (this._lastHeapMB == null) s.heapTrend = '=';
      else if (mb > this._lastHeapMB + 0.05) s.heapTrend = '▲';      // ▲ rising
      else if (mb < this._lastHeapMB - 0.05) s.heapTrend = '▼';      // ▼ falling (a GC sweep)
      else s.heapTrend = '=';
      this._lastHeapMB = mb;
      s.heapMB = mb;
    } else {
      s.heapMB = null;
      s.heapTrend = '=';
    }

    const info = this._infoFn();
    if (info) {
      const r = info.render || {};
      const m = info.memory || {};
      s.calls = r.calls || 0;
      s.tris = r.triangles || 0;
      s.geo = m.geometries || 0;
      s.tex = m.textures || 0;
    }
  }

  /** Write the throttled text to the DOM (single textContent set — one layout dirty). */
  _write() {
    if (!this.el) return;
    const s = this._stats;
    const fps = s.fps.toFixed(0).padStart(2, ' ');
    const avg = s.avgMs.toFixed(1).padStart(4, ' ');
    const max = s.maxMs.toFixed(1).padStart(5, ' ');
    const w5 = s.worst5s.toFixed(1).padStart(5, ' ');
    const raf = s.rafMs.toFixed(1).padStart(4, ' ');
    const heap = (s.heapMB == null) ? 'n/a' : (s.heapMB.toFixed(1) + ' ' + s.heapTrend);
    this.el.textContent =
      `FPS ${fps}/60   raf ${raf}\n` +
      `ms  ${avg} avg  ${max} max\n` +
      `worst5s ${w5}  jank ${s.jank}\n` +
      `heap ${heap}\n` +
      `draw ${s.calls}  tris ${s.tris}\n` +
      `mem geo ${s.geo}  tex ${s.tex}`;
  }

  /** Headless/test accessor — the latest computed stats (numbers). */
  stats() { return { ...this._stats, frames: this._frames }; }

  /** Reset the timing baseline (e.g. after a long stall/tab-switch) so the first gap
   * after the reset isn't counted as a spike. Optional — not called in the live loop. */
  resetTiming() { this._lastRenderT = 0; this._lastRafT = 0; }
}
