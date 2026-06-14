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
      phase: 'ready',     // ready | idle | countdown | running | win | lose
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
    // ── SLOW-MOTION (bullet-time) while drawing (§A) ──
    // The core loop is "redraw the leg as the path changes". To make that feel good
    // and FAIR, the WHOLE game (player + rival) slows to 10% speed the instant the
    // user starts a stroke and snaps back to full speed when the stroke is applied.
    // It is a pure TIME multiplier on step() — deterministic, fixed-timestep-safe.
    this._timeScale = 1.0;
    this.SLOWMO_SCALE = 0.1;   // 90% slower while drawing (bullet-time)
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
    // REFERENCE START: enter the PRE-RACE IDLE FLOAT — the cube (and rival) hover above
    // the track and bob gently. NO countdown yet (it starts when the player begins their
    // first stroke), NO forward. The "DRAW A LEG" hint shows.
    this._enterPhase('idle');
    return this.track;
  }

  /** Build the rival walker for the current track (no-op if no rival). The rival
   * gets its OWN buildTrack (independent surface model) + a fixed designed leg
   * preset + a pace multiplier so it is "competitive" but tunable from data.
   *
   * ADAPTIVE RIVAL (leg-swap-by-x): because the course now alternates WALL/GAP/stairs
   * (need a LONG leg) with TUNNEL (need a SHORT leg) — mutually exclusive — NO single
   * leg can finish. So the rival (BOLT) must REDRAW its leg per zone, exactly like the
   * player. We build an x-scheduled list of (untilX → preset) from the track segments:
   * a SHORT preset just before each tunnel, a LONG preset before each wall/gap/stairs.
   * The rival then swaps presets as it crosses each threshold (mid-run, continue branch
   * — no teleport), so it completes the course and the race stays a real contest. */
  _setupRival() {
    const spec = this.track.rival
      ? { ...RIVAL_DEFAULTS, ...this.track.rival }
      : null;
    this.rivalSpec = spec;
    if (!spec) { this.state.rivalProgress = 0; return; }
    this.rival.buildTrack(this.track);
    this.rival.paceFactor = spec.pace;
    this._rivalSchedule = this._buildRivalSchedule();
    this._rivalLegIdx = -1;
    this._applyRivalLegForX(this.rival.bodyX, true); // fresh first leg
    this.state.rivalProgress = this.rival.progress;
  }

  /** Build the rival's leg-swap-by-x schedule from the segment model. Each entry is
   * { x, preset }: AT (and after) x the rival uses `preset`, until the next entry. We
   * place a SHORT leg ('short') a little BEFORE each tunnel mouth and a LONG leg
   * ('long') a little before each wall/gap/stairs gate (and at the very start). The
   * presets are chosen so 'short'.reach (~0.73) passes every tunnel (clearance>=0.85)
   * and 'long'.reach (~1.7) clears every wall/gap/stairs. The schedule is derived from
   * the DATA (segment kinds + their x ranges), never hardcoded x values. */
  _buildRivalSchedule() {
    const segs = this.rival._segs || [];
    if (!segs.length) return [{ x: -Infinity, preset: 'long' }];
    // collect gate windows with their kind. tunnels need SHORT; wall/gap/stairs need LONG.
    const gates = [];
    const seen = new Set();
    for (const s of segs) {
      const key = s.kind + ':' + Math.round(s.x0 * 100);
      if (s.kind === 'tunnel') gates.push({ x: s.x0, need: 'short' });
      else if (s.kind === 'wall') gates.push({ x: s.x0, need: 'long' });
      else if (s.kind === 'gap') gates.push({ x: s.x0, need: 'long' });
      else if (s.kind === 'stairs' && !seen.has('stairsrun')) {
        // first tread of a stairs run only (treads abut) — gate once per run.
        gates.push({ x: s.x0, need: 'long' });
      }
    }
    // also gate each stairs RUN once (group abutting treads) — replace the naive add.
    // Rebuild cleanly: scan and group stairs.
    gates.length = 0;
    let prevStairX1 = -Infinity;
    for (const s of segs) {
      if (s.kind === 'tunnel') gates.push({ x: s.x0, need: 'short' });
      else if (s.kind === 'wall' || s.kind === 'gap') gates.push({ x: s.x0, need: 'long' });
      else if (s.kind === 'stairs') {
        if (s.x0 > prevStairX1 + 0.05) gates.push({ x: s.x0, need: 'long' });
        prevStairX1 = s.x1;
      }
    }
    gates.sort((a, b) => a.x - b.x);
    // build the schedule: switch to the gate's required preset SWITCH_LEAD before its x
    // (so the rival has the right leg AT the gate). Start with a LONG leg (the start zone
    // before the first gate typically wants to reach a wall/gap). Collapse consecutive
    // equal presets.
    const SWITCH_LEAD = 2.5; // world u before the gate to have swapped (clears countdown jitter)
    const presetFor = (need) => (need === 'short' ? 'short' : 'long');
    const out = [{ x: -Infinity, preset: 'long' }];
    for (const gate of gates) {
      const p = presetFor(gate.need);
      const last = out[out.length - 1];
      if (last.preset === p) continue;          // already on the right leg
      out.push({ x: gate.x - SWITCH_LEAD, preset: p });
    }
    return out;
  }

  /** Apply the scheduled rival leg for forward position x (swaps when crossing the
   * next threshold). `fresh` forces a fresh placement (start). Uses the continue
   * branch otherwise so a mid-run swap never teleports the rival. */
  _applyRivalLegForX(x, fresh = false) {
    if (!this._rivalSchedule || !this._rivalSchedule.length) return;
    // find the latest schedule entry whose x <= current x.
    let idx = 0;
    for (let i = 0; i < this._rivalSchedule.length; i++) {
      if (this._rivalSchedule[i].x <= x) idx = i; else break;
    }
    if (idx === this._rivalLegIdx && !fresh) return; // no change
    this._rivalLegIdx = idx;
    const preset = this._rivalSchedule[idx].preset;
    this.rival.setLegStroke(presetStroke(preset), fresh ? { fresh: true } : {});
    if (this.renderer) this.renderer.rebuildRivalLegs(this.rival);
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

  // ── SLOW-MOTION signals from the draw input (§A) ──
  /** The user STARTED a stroke.
   *   • From the PRE-RACE IDLE FLOAT (phase 'idle'): this is the FIRST draw → kick off
   *     the 3-2-1 COUNTDOWN (the cube keeps floating + bobbing while the player draws
   *     their first leg; the race starts at GO). This is the reference start: the
   *     countdown is tied to the player's first stroke, NOT to track entry.
   *   • Mid-run (phase 'running'): enter bullet-time (whole game slows to 10%) so the
   *     redraw feels good — the cube keeps creeping forward while you redraw.
   * forceStart()/headless paths skip 'idle', so this never fires there (tests start
   * immediately in 'running'). */
  beginDraw() {
    if (this.state.phase === 'idle') {
      // first stroke ⇒ start the countdown (still floating; no slow-mo needed at v=0).
      this._enterPhase('countdown');
      return;
    }
    this._timeScale = this.SLOWMO_SCALE;
  }
  /** The stroke ENDED (a new leg was applied, or the gesture cancelled) → full speed. */
  endDraw() { this._timeScale = 1.0; }
  get timeScale() { return this._timeScale; }

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
    // back to the PRE-RACE IDLE FLOAT (same as a fresh load) — float + bob, no countdown
    // until the player draws again.
    this._enterPhase('idle');
  }

  // ── phase machine ──
  // START FLOW (reference): idle-float → (player's first draw) → 3s countdown (still
  // floating, player draws their leg) → GO → running (cube eases from float to ground).
  //   • idle      : PRE-RACE float + bob ON, no countdown, no forward.
  //   • countdown : float still ON (the player draws while the 3-2-1 ticks), no forward.
  //   • running   : float OFF — both walkers ease to the grounded pose and the race runs.
  // The float toggle is set on BOTH walkers (player + rival) so they hover together.
  _enterPhase(phase) {
    this.state.phase = phase;
    const float = (phase === 'idle' || phase === 'countdown');
    if (this.physics.setIdleFloat) this.physics.setIdleFloat(float);
    if (this.rivalSpec && this.rival.setIdleFloat) this.rival.setIdleFloat(float);
    if (phase === 'countdown') this.state.countdownMs = COUNTDOWN_MS;
    this._renderHud();
  }

  // ── fixed-step sim (deterministic) ──
  /** Advance the simulation by ms worth of fixed steps. Used by loop & headless.
   *
   * SUB-STEP dt ADAPTS TO SLOW-MOTION (smoothness fix). The loop hands us ALREADY
   * time-scaled ms (frame·timeScale). With a constant 16.67ms tick the accumulator
   * only reached one tick every ~10 frames during 0.1× bullet-time (per-frame intake
   * ≈ 1.67ms), so the cube sat still for ~10 frames then JUMPED — the "stuttery slow-
   * mo" the user reported. We instead shrink the fixed tick to `FIXED_DT · timeScale`
   * while slowed, so the accumulator drains ~1 tiny tick PER FRAME: 0.1× as far, but
   * CONTINUOUS (a smooth crawl, not a 10-frame strobe). The walker is procedural /
   * kinematic — every per-step quantity (Δx=v·dt, tilt ease 1−e^(−lerp·dt), gait ω,
   * the airborne gravity arc) is proportional to dt, so a smaller dt is identical
   * behaviour, just finer-grained. At full speed (timeScale==1) dt stays FIXED_DT
   * exactly ⇒ ZERO behaviour change and refresh-rate independence is preserved.
   *
   * The remainder is STILL carried across frames in `_acc` (dropping it leaks time),
   * and that carry stays correct when dt changes mid-stream: `_acc` holds leftover
   * ms, the new dt simply re-quantizes it — so the slow→full snap on endDraw never
   * loses or duplicates simulated time. */
  step(ms) {
    const ts = this._timeScale || 1;
    const base = this.physics.FIXED_DT;
    // effective fixed tick:
    //   • NORMAL speed (ts==1): exactly FIXED_DT ⇒ ZERO behaviour change + the
    //     accumulator carry keeps the sim at real time on ANY refresh rate.
    //   • SLOW-MO (ts<1): we want ~1 tick PER FRAME so motion is continuous (not a
    //     10-frame freeze→jump strobe). The per-frame intake is `ms` (≈ frame·ts), so
    //     a tick of FIXED_DT·ts matches a 60Hz frame's intake — but on a 120/144Hz
    //     display each frame's intake is SMALLER, so that tick would still need 2+
    //     frames to fire (≈50% frozen frames). Tracking the actual intake `ms` (with
    //     a small floor for numeric sanity, capped at the 60Hz slow tick so we never
    //     take a coarse step) makes it ~1 tick/frame at EVERY refresh rate.
    const dt = ts >= 1
      ? base
      : Math.min(base * ts, Math.max(ms, 0.25));
    this._acc = (this._acc || 0) + ms;
    if (this._acc > 250) this._acc = 250; // spiral-of-death guard (long stalls)
    // spiral guard #2: cap ticks PER CALL. The intake per frame is ms (≈ frame·ts),
    // so at the matched dt this is ~1 tick/frame; the cap only bites on a catch-up
    // burst (e.g. tab-switch). 240 ticks/frame is generous headroom either way.
    let n = 0;
    while (this._acc >= dt && n < 240) {
      this._tick(dt);
      this._acc -= dt;
      n++;
    }
  }

  _tick(dt) {
    const s = this.state;
    const hasRival = !!this.rivalSpec;
    if (s.phase === 'idle') {
      // PRE-RACE: float + bob both bodies (walker.update handles the idle-float pose).
      // No countdown, no forward — we wait for the player's first stroke (beginDraw).
      this.physics.update(dt, false);
      if (hasRival) this.rival.update(dt, false);
    } else if (s.phase === 'countdown') {
      s.countdownMs -= dt;
      // hold both bodies during countdown — they keep FLOATING + bobbing (idle-float is
      // still on) while the player draws their first leg; no advance. At GO ⇒ running.
      this.physics.update(dt, false);
      if (hasRival) this.rival.update(dt, false);
      if (s.countdownMs <= 0) this._enterPhase('running');
    } else if (s.phase === 'running') {
      s.timeMs += dt;
      // RACE: both walkers advance every step. The rival is autonomous; it SWAPS its
      // leg per the x-schedule (long↔short) as it crosses each gate so it can clear both
      // the long-required (wall/gap/stairs) and short-required (tunnel) gimmicks. The
      // player is driven by the user's drawn leg.
      if (hasRival) this._applyRivalLegForX(this.rival.bodyX);
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
      // SLOW-MOTION: scale REAL frame time by the current timeScale (0.1 while the
      // user is drawing, 1.0 otherwise). The fixed-step accumulator in step() carries
      // the sub-tick remainder, so 0.1× simply advances ~10% of the ticks per frame —
      // both player AND rival slow together (it is applied to the whole step).
      this.step(frame * this._timeScale);
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

  /** Force phase to running immediately (skip the idle-float + countdown) — headless
   * helper used by the verifier/diag. Goes straight to 'running' so EVERY existing test
   * gate (no-pen / no-slip / redraw / race / tunnel …) still starts the race instantly.
   * _enterPhase('running') clears the idle-float on both walkers, so the cube is on the
   * ground from frame 0 (no float→ground settle in the headless path). */
  forceStart() {
    if (this.physics.setIdleFloat) this.physics.setIdleFloat(false);
    if (this.rivalSpec && this.rival.setIdleFloat) this.rival.setIdleFloat(false);
    this._enterPhase('running');
  }

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
    this._rivalSchedule = this._buildRivalSchedule();
    this._rivalLegIdx = -1;
    this._applyRivalLegForX(this.rival.bodyX, true);
    this.state.rivalProgress = this.rival.progress;
  }
}
