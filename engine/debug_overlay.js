// engine/debug_overlay.js
// LEFT-TOP REAL-TIME DEBUG OVERLAY (display only — ZERO gameplay/physics/render-behaviour
// change). Shows the user (iPhone 17 Pro "still stutters" report) WHAT is actually
// happening on screen each render:
//
//   • FPS        — ACHIEVED render FPS. We cap render at 60fps (see game.startLoop's
//                  RENDER_DT gate), so the IDEAL here is 60; whenever a render takes
//                  longer than its 16.67ms budget the achieved FPS drops below 60 and
//                  THAT is a visible hitch. Measured from the REAL performance.now()
//                  delta between consecutive renders (NOT sim/timeScale time), averaged
//                  over the most recent ~0.5s window.
//   • frame ms   — render-to-render gap, avg / MAX over the most recent ~1s window. The
//                  MAX is the key number: a single 33ms+ gap = the moment the eye sees a
//                  jank. The user can watch which course region spikes the max.
//   • calls/tris — three renderer.info.render.calls / .triangles (draw-call + geometry
//                  load that frame).
//   • mem        — three renderer.info.memory.geometries / .textures (live GPU object
//                  count — a creeping number across restarts = a leak).
//
// PERF (the overlay must NOT itself cause the jank it measures):
//   • frame() is called EVERY render and only pushes ONE number into a fixed ring buffer
//     (no allocation, no string build, no DOM touch) — O(1).
//   • The text string + the single DOM write happen on a THROTTLE (~4×/s), so the overlay
//     never dirties layout per frame.
//   • The ring buffer is pre-sized; nothing is allocated per frame.

const FPS_WINDOW_MS = 500;    // achieved-FPS average window (~0.5s)
const GAP_WINDOW_MS = 1000;   // frame-ms avg/max window (~1s)
const DRAW_INTERVAL_MS = 250; // DOM text refresh throttle (~4×/s)
const RING = 256;             // ring-buffer capacity (≥ 1s of 144fps render gaps)

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

    // latest computed stats (also readable headless via stats())
    this._stats = { fps: 0, avgMs: 0, maxMs: 0, calls: 0, tris: 0, geo: 0, tex: 0 };
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

    // THROTTLE the expensive part (string build + DOM write) to ~4×/s.
    if (t - this._lastDrawT >= DRAW_INTERVAL_MS) {
      this._lastDrawT = t;
      this._recompute(t);
      this._write();
    }
  }

  /** Recompute avg/max gap (over the GAP window) + achieved FPS (over the FPS window) +
   * pull the live three info. No allocation. */
  _recompute(t) {
    let sumAll = 0, nAll = 0, maxAll = 0;   // GAP_WINDOW_MS window → avg/max ms
    let nFps = 0;                            // FPS_WINDOW_MS window → achieved fps
    const cutGap = t - GAP_WINDOW_MS;
    const cutFps = t - FPS_WINDOW_MS;
    // walk the valid samples (no array copy / sort).
    for (let k = 0; k < this._count; k++) {
      // index from newest backward
      let i = this._head - 1 - k;
      if (i < 0) i += RING;
      const at = this._tAt[i];
      if (at < cutGap) break;               // older than the longest window ⇒ stop
      const g = this._gap[i];
      sumAll += g; nAll++;
      if (g > maxAll) maxAll = g;
      if (at >= cutFps) nFps++;
    }
    const s = this._stats;
    s.avgMs = nAll ? (sumAll / nAll) : 0;
    s.maxMs = maxAll;
    // achieved FPS = frames rendered in the last FPS window / window-seconds. Each sample
    // is a render gap that ENDED within the window, so nFps renders happened in ~window.
    s.fps = nFps ? (nFps / (FPS_WINDOW_MS / 1000)) : 0;

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
    // pad/format compactly; monospace so columns line up. Build ONE string.
    const fps = s.fps.toFixed(0).padStart(2, ' ');
    const avg = s.avgMs.toFixed(1).padStart(4, ' ');
    const max = s.maxMs.toFixed(1).padStart(5, ' ');
    this.el.textContent =
      `FPS ${fps}/60\n` +
      `ms  ${avg} avg  ${max} max\n` +
      `draw ${s.calls}  tris ${s.tris}\n` +
      `mem geo ${s.geo}  tex ${s.tex}`;
  }

  /** Headless/test accessor — the latest computed stats (numbers). */
  stats() { return { ...this._stats, frames: this._frames }; }

  /** Reset the timing baseline (e.g. after a long stall/tab-switch) so the first gap
   * after the reset isn't counted as a spike. Optional — not called in the live loop. */
  resetTiming() { this._lastRenderT = 0; }
}
