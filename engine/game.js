// engine/game.js
// Ties Physics + Renderer + GameState + input + HUD into one playable game.
// Also exposes the deterministic headless debug API (window.__DC) — L24.

import { Physics, presetStroke } from './physics.js';
import { Renderer } from './renderer.js';
import { validateTrack, RIVAL_DEFAULTS } from './track_schema.js';

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
    // RIVAL: a SECOND, fully independent walker on a parallel lane (the computer
    // opponent). Same class, own track build, own leg preset + pace. Player and
    // rival never share state — they are two clean Physics instances.
    this.rival = new Physics();
    this.rivalSpec = null;       // resolved RivalSpec for the current track (or null)
    this.renderer = this.headless ? null : new Renderer(this.canvas);

    this.state = {
      phase: 'ready',     // ready | countdown | running | win | lose
      trackId: '',
      bodyX: 0,
      progress: 0,
      rivalProgress: 0,   // the opponent's progress along its lane (0..1)
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
    // RIVAL: resolve spec (track.rival or defaults if a race is requested) and
    // build the opponent walker on its own copy of the track + give it its leg.
    this._setupRival();
    if (this.renderer) {
      this.renderer.buildTrack(this.physics, this.track, this.rivalSpec);
      if (this.rivalSpec) this.renderer.rebuildRivalLegs(this.rival);
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

  /** Build the rival walker for the current track (no-op if no rival). The rival
   * gets its OWN buildTrack (independent surface model) + a fixed designed leg
   * preset + a pace multiplier so it is "competitive" but tunable from data. */
  _setupRival() {
    const spec = this.track.rival
      ? { ...RIVAL_DEFAULTS, ...this.track.rival }
      : null;
    this.rivalSpec = spec;
    if (!spec) { this.state.rivalProgress = 0; return; }
    this.rival.buildTrack(this.track);
    this.rival.paceFactor = spec.pace;
    this.rival.setLegStroke(presetStroke(spec.legPreset));
    this.state.rivalProgress = this.rival.progress;
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
    this._setupRival();
    if (this.renderer) {
      this.renderer.buildTrack(this.physics, this.track, this.rivalSpec);
      if (this.rivalSpec) this.renderer.rebuildRivalLegs(this.rival);
    }
    if (this._lastStroke) {
      this.physics.setLegStroke(this._lastStroke);
      if (this.renderer) this.renderer.rebuildLegs(this.physics);
    }
    this.state.legDrawn = this.physics.legDrawn;
    this.state.timeMs = 0;
    this.state.progress = 0;
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
    // PERSIST the sub-tick remainder across frames. The old `let acc = ms` threw
    // away any leftover < dt every call, so on a high-refresh display (e.g. a
    // 120Hz phone: ~8.3ms/frame, which is < dt 16.67ms) almost every frame ran
    // ZERO ticks and the sim crawled far slower than real time ("slow motion"),
    // no matter how high baseSpeed was. Carrying the remainder makes the sim
    // advance at true real time on ANY refresh rate.
    this._acc = (this._acc || 0) + ms;
    if (this._acc > 250) this._acc = 250; // spiral-of-death guard (long stalls)
    while (this._acc >= dt) {
      this._tick(dt);
      this._acc -= dt;
    }
  }

  _tick(dt) {
    const s = this.state;
    const hasRival = !!this.rivalSpec;
    if (s.phase === 'countdown') {
      s.countdownMs -= dt;
      // hold both bodies during countdown (legs locked, no advance)
      this.physics.update(dt, false);
      if (hasRival) this.rival.update(dt, false);
      if (s.countdownMs <= 0) this._enterPhase('running');
    } else if (s.phase === 'running') {
      s.timeMs += dt;
      // RACE: both walkers advance every step. The rival is autonomous (its leg
      // is already set), the player is driven by the drawn leg.
      this.physics.update(dt, true);
      if (hasRival) this.rival.update(dt, true);
      s.bodyX = this.physics.bodyX;
      s.progress = this.physics.progress;
      if (hasRival) s.rivalProgress = this.rival.progress;

      const playerDone = this.physics.bodyX >= this.physics.finishX;
      const rivalDone = hasRival && this.rival.bodyX >= this.rival.finishX;

      if (this.physics.exploded) {
        // explosion = sim failure; treat as lose (shouldn't happen)
        this._enterPhase('lose');
      } else if (rivalDone && !playerDone) {
        // the opponent crossed the line first ⇒ immediate LOSE.
        s.rivalProgress = 1;
        this._enterPhase('lose');
      } else if (playerDone) {
        // player finished — WIN if ahead of (or tied with) the rival, else LOSE.
        s.progress = 1;
        if (rivalDone && this.rival.bodyX > this.physics.bodyX) {
          this._enterPhase('lose');
        } else {
          this._recordBest();
          this._enterPhase('win');
        }
      }
    } else {
      // ready/win/lose: keep both settled but no progress
      this.physics.update(dt, false);
      if (hasRival) this.rival.update(dt, false);
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
      if (this.rivalSpec) this.renderer.syncRival(this.rival);
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
    // RIVAL marker on the same bar (only when there is an opponent).
    if (h.rivalMarker) {
      if (this.rivalSpec) {
        h.rivalMarker.style.display = 'block';
        h.rivalMarker.style.left = `${Math.round(s.rivalProgress * 100)}%`;
      } else {
        h.rivalMarker.style.display = 'none';
      }
    }

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
      // RIVAL / RACE state
      hasRival: !!this.rivalSpec,
      rivalProgress: this.rivalSpec ? this.rival.progress : 0,
      rivalBodyX: this.rivalSpec ? this.rival.bodyX : null,
      rivalExploded: this.rivalSpec ? this.rival.exploded : false,
    };
  }

  /** Force phase to running immediately (skip countdown) — headless helper. */
  forceStart() { this._enterPhase('running'); }

  /** Headless helper: disable the rival (used by the isolated player-walker
   * sub-tests that build their own custom track directly on `game.physics` and
   * must not be interrupted by a stale rival crossing its old finish line). */
  clearRival() { this.rivalSpec = null; this.state.rivalProgress = 0; }

  /** Headless helper: (re)enable a rival from a spec on the CURRENT track build,
   * so a race can be driven without a renderer. */
  enableRival(spec) {
    this.rivalSpec = spec ? { ...spec } : null;
    if (!this.rivalSpec) { this.state.rivalProgress = 0; return; }
    this.rival.buildTrack(this.track);
    this.rival.paceFactor = this.rivalSpec.pace;
    this.rival.setLegStroke(presetStroke(this.rivalSpec.legPreset));
    this.state.rivalProgress = this.rival.progress;
  }
}
