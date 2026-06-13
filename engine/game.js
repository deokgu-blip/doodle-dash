// engine/game.js
// Ties Physics + Renderer + GameState + input + HUD into one playable game.
// Also exposes the deterministic headless debug API (window.__DC) — L24.

import { Physics, presetStroke } from './physics.js';
import { Renderer } from './renderer.js';
import { validateTrack } from './track_schema.js';

const COUNTDOWN_MS = 2600; // 3 -> 2 -> 1 -> GO

export class Game {
  /**
   * @param {Object} opts
   * @param {HTMLCanvasElement} opts.canvas       three.js render canvas
   * @param {HTMLCanvasElement} opts.drawCanvas   draw-box overlay (2D input)
   * @param {Object}            [opts.hud]        HUD DOM refs (optional in headless)
   * @param {boolean}           [opts.headless]   if true, no rAF loop / no render
   */
  constructor(opts) {
    this.canvas = opts.canvas;
    this.drawCanvas = opts.drawCanvas || null;
    this.hud = opts.hud || {};
    this.headless = !!opts.headless;

    this.physics = new Physics();
    this.renderer = this.headless ? null : new Renderer(this.canvas);

    this.state = {
      phase: 'ready',     // ready | countdown | running | win | lose
      trackId: '',
      bodyX: 0,
      progress: 0,
      timeMs: 0,
      countdownMs: 0,
      legDrawn: false,
      best: this._loadBest(),
    };
    this.track = null;
    this._lastStroke = null;
    this._raf = null;
    this._lastT = 0;
  }

  // ── track loading (data only) ──
  async loadTrack(trackOrId) {
    let data = trackOrId;
    if (typeof trackOrId === 'string') {
      const res = await fetch(`tracks/${trackOrId}.json`);
      data = await res.json();
    }
    this.track = validateTrack(data);
    this.state.trackId = this.track.id;
    this.physics.buildTrack(this.track);
    if (this.renderer) {
      this.renderer.buildTrack(this.physics, this.track);
    }
    // restore a previously drawn stroke (or default) so the body is ready
    if (this._lastStroke) {
      this.physics.setLegStroke(this._lastStroke);
      if (this.renderer) this.renderer.rebuildLegs(this.physics);
    }
    this.state.legDrawn = this.physics.legDrawn;
    this._enterPhase('countdown');
    return this.track;
  }

  // ── leg input ──
  setLegStroke(points, spec) {
    this._lastStroke = points;
    this.physics.setLegStroke(points, spec);
    if (this.renderer) this.renderer.rebuildLegs(this.physics);
    this.state.legDrawn = this.physics.legDrawn;
  }
  setLegPreset(name) {
    this.setLegStroke(presetStroke(name));
  }

  restart() {
    this.physics.buildTrack(this.track);
    if (this.renderer) this.renderer.buildTrack(this.physics, this.track);
    if (this._lastStroke) {
      this.physics.setLegStroke(this._lastStroke);
      if (this.renderer) this.renderer.rebuildLegs(this.physics);
    }
    this.state.legDrawn = this.physics.legDrawn;
    this.state.timeMs = 0;
    this._enterPhase('countdown');
  }

  // ── phase machine ──
  _enterPhase(phase) {
    this.state.phase = phase;
    if (phase === 'countdown') this.state.countdownMs = COUNTDOWN_MS;
    this._renderHud();
  }

  // ── fixed-step sim (deterministic) ──
  /** Advance the simulation by ms worth of fixed steps. Used by loop & headless. */
  step(ms) {
    const dt = this.physics.FIXED_DT;
    let acc = ms;
    while (acc >= dt) {
      this._tick(dt);
      acc -= dt;
    }
  }

  _tick(dt) {
    const s = this.state;
    if (s.phase === 'countdown') {
      s.countdownMs -= dt;
      // hold the body during countdown (legs locked)
      this.physics.update(dt, false);
      if (s.countdownMs <= 0) this._enterPhase('running');
    } else if (s.phase === 'running') {
      s.timeMs += dt;
      this.physics.update(dt, true);
      s.bodyX = this.physics.bodyX;
      s.progress = this.physics.progress;
      if (this.physics.exploded) {
        // explosion = sim failure; treat as lose (shouldn't happen)
        this._enterPhase('lose');
      } else if (this.physics.bodyX >= this.physics.finishX) {
        s.progress = 1;
        this._recordBest();
        this._enterPhase('win');
      }
    } else {
      // ready/win/lose: keep physics settled but no progress
      this.physics.update(dt, false);
    }
  }

  // ── render loop (browser only) ──
  startLoop() {
    if (this.headless) return;
    const loop = (t) => {
      this._raf = requestAnimationFrame(loop);
      if (!this._lastT) this._lastT = t;
      let frame = t - this._lastT;
      this._lastT = t;
      if (frame > 100) frame = 100; // clamp tab-switch spikes
      this.step(frame);
      this.renderer.sync(this.physics);
      this.renderer.updateCamera(this.physics);
      this.renderer.render();
      this._renderHud();
    };
    this._raf = requestAnimationFrame(loop);
  }
  stopLoop() { if (this._raf) cancelAnimationFrame(this._raf); this._raf = null; }

  // ── HUD ──
  _renderHud() {
    const h = this.hud, s = this.state;
    if (!h || !h.progressFill) return;
    h.progressFill.style.width = `${Math.round(s.progress * 100)}%`;
    if (h.youMarker) h.youMarker.style.left = `${Math.round(s.progress * 100)}%`;

    if (h.countdown) {
      if (s.phase === 'countdown') {
        const n = Math.ceil(s.countdownMs / 700);
        h.countdown.textContent = n >= 4 ? '3' : (n <= 0 ? 'GO!' : String(n));
        h.countdown.style.display = 'block';
      } else {
        h.countdown.style.display = 'none';
      }
    }
    if (h.overlay) {
      if (s.phase === 'win' || s.phase === 'lose') {
        h.overlay.style.display = 'flex';
        if (h.overlayTitle) h.overlayTitle.textContent = s.phase === 'win' ? 'terrific!' : 'try again';
      } else {
        h.overlay.style.display = 'none';
      }
    }
  }

  // ── best time persistence ──
  _loadBest() {
    try { return JSON.parse(localStorage.getItem('dc_best') || '{}'); }
    catch { return {}; }
  }
  _recordBest() {
    const id = this.state.trackId, t = this.state.timeMs;
    if (!this.state.best[id] || t < this.state.best[id]) {
      this.state.best[id] = t;
      try { localStorage.setItem('dc_best', JSON.stringify(this.state.best)); } catch {}
    }
  }

  // ── headless debug API (L24) ──
  getState() {
    return {
      phase: this.state.phase,
      bodyX: this.physics.bodyX,
      progress: this.physics.progress,
      finishX: this.physics.finishX,
      startX: this.physics.startX,
      exploded: this.physics.exploded,
      timeMs: this.state.timeMs,
      legDrawn: this.physics.legDrawn,
    };
  }

  /** Force phase to running immediately (skip countdown) — headless helper. */
  forceStart() { this._enterPhase('running'); }
}
