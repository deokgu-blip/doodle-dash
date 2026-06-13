// engine/walker.js
// PROCEDURAL / KINEMATIC locomotion for Draw Climber (replaces the Matter.js
// solver). The original game is NOT a rigid-body simulation — it is a designed
// "looks-like-physics" animation. We model it that way: a deterministic walker
// whose forward speed, climbing and foot motion are DESIGNED functions, so we
// get zero penetration / zero slip / monotone length→speed BY CONSTRUCTION.
//
// ── What is KEPT (validated look & input — do not change) ──
//   * The leg is the user's DRAWN STROKE rendered as a thin smooth pen LINE
//     (renderer.js _buildStrokeRibbon). It anchors at the cube CENTRE by the
//     stroke's START point (chain[0] == pin == {0,0}) and extends one way; two
//     legs straddle the cube in z (side = ±1) and spin 180° out of phase.
//   * Input: input.js → normalized stroke → setLegStroke → legReach/shape. No
//     stroke ⇒ no legs ⇒ no motion (input-driven).
//   * window.__DC headless API, fixed timestep determinism, single-file build.
//   * The data contract: engine LOADS tracks/*.json only (track_schema.js).
//
// ── What is DROPPED ──
//   * Matter.Engine.update solver, revolute pins, motors, friction, restitution,
//     substeps, anti-launch clamps, ratchets, grip-assist — all gone. The Matter
//     library is still imported by index.html for legacy tooling but UNUSED here.
//
// ── The model (physics y is +DOWN, Matter convention, so the renderer that maps
//    render-y = -physics-y is untouched). "Up" = smaller (more negative) y. ──
//
//   1. TERRAIN = HEIGHT FUNCTION. buildTrack() turns TrackData segments into
//      (a) floor slabs the renderer draws (same look as before) and (b) a list
//      of segment metas with a surfaceY(x) sampler + per-segment kind/step data.
//   2. STATE. x (forward), the drawn leg's reach (length) + shape, leg phase θ.
//   3. EACH FIXED STEP:
//        v = baseSpeed * legSpeedFactor(reach, shape) * terrainFactor(local)
//        x += v*dt
//        - legSpeedFactor: longer leg ⇒ bigger stride ⇒ faster (monotone curve).
//        - terrainFactor: flat=1, uphill slower, downhill faster, stairs per the
//          climb rule.
//      CLIMB RULE (designed, not emergent): a step of height h ahead is climbable
//      iff reach >= climbThreshold(h). If not, the walker STOPS at the step (v=0,
//      blocked). Long leg ⇒ tall steps OK; short leg ⇒ blocked.
//      BODY HEIGHT y = surfaceY(x) + bodyClearance (above surface) → the foot can
//      NEVER go below the surface (structural 0 penetration). On stairs y rises
//      smoothly (interpolated, no teleport). A small phase-locked vertical BOB
//      adds walking juice.
//   4. LEG ROTATION (no-slip by construction). ω = v / effectiveRadius, so the
//      contact foot's world speed is ~0 (no slip). The foot's lowest point is
//      clamped to sit ON the surface (never below). Two legs 180° apart. Faster
//      ⇒ spins faster (walking cadence).
//
// All tunable "feel" knobs live in TUNE below (designer-adjustable).

import { SEGMENT_DEFAULTS } from './track_schema.js';

// Player cube edge (world units) — unchanged from the old build so the renderer
// (PHYS_CONST.CUBE_SIZE) and the look are identical.
const CUBE_SIZE = 0.9;
// Axle at the cube geometric CENTRE (legs pin here and spin about it).
const AXLE_X = 0.0;
const AXLE_Y = 0.0;

// Floor slab thickness below the surface (render only — for the ribbon depth).
const FLOOR_THICK = 4.0;

// ── Leg (drawn-stroke) geometry — identical mapping to the old build so the
//    rendered line and the length→reach relation are unchanged. ──
const LEG_WORLD_SCALE = 1.0;     // normalized [-1,1] → world units (length preserved)
const LEG_LINE_RADIUS = 0.13;    // rendered line half-thickness == physics radius
const LEG_CIRCLE_SPACING = LEG_LINE_RADIUS * 1.15; // resample spacing along stroke
const LEG_REACH_MIN = 0.6;       // clamp: below this the leg can't reach the ground
const LEG_REACH_MAX = 1.7;       // clamp: above this it would be a giant lever
const LEG_MAX_CIRCLES = 40;      // chain sample cap (shape-faithful, bounded)

// ── DESIGNER FEEL KNOBS (the "hand feel") ──
// Tuned for a grounded, planted walk: not too fast/slow, no penetration, no
// slip, no stop-go jerk. 'natural' is ultimately the user's call.
const TUNE = {
  baseSpeed: 6.0,        // world u/s at the reference leg on flat ground
  refReach: 1.0,         // reach that maps to legSpeedFactor == 1
  // legSpeedFactor(reach): longer ⇒ bigger stride ⇒ faster. A gentle monotone
  // power curve, clamped. At reach=MIN ≈ 0.66, at reach=MAX ≈ 1.43.
  speedReachPow: 0.62,   // exponent of (reach/refReach)
  speedFactorMin: 0.55,
  speedFactorMax: 1.55,
  // terrain factors
  uphillSlow: 0.55,      // multiplier on a ramp going up (per unit slope, blended)
  downhillFast: 1.25,    // multiplier on a ramp going down
  stairClimbSlow: 0.7,   // climbing stairs is a bit slower than flat
  // CLIMB RULE: a step of height h is climbable iff reach >= climbBase + climbK*h
  // i.e. taller steps demand a longer leg. Solve h_max(reach) = (reach-base)/K.
  // Calibrated so the SHORTEST leg (reach 0.6) clears a LOW step (~0.25) but is
  // BLOCKED at the T01 step (0.45) and a HIGH step (0.9); the LONGEST leg (1.7)
  // clears them all. maxClimb(0.6)=0.30, maxClimb(1.05)=0.75, maxClimb(1.7)=1.40.
  climbBase: 0.30,       // a reach of 0.30+ clears an infinitesimal lip
  climbK: 1.0,           // each +1 unit of step height needs +1.0 reach
  // body — the cube centre floats so the foot tip just GRAZES the surface at the
  // bottom of its circular sweep and is ABOVE everywhere else (structural 0
  // penetration). clearance = (max chain distance from axle) + lineRadius + bob
  // headroom; the bob then oscillates DOWNWARD within that headroom so the lowest
  // instant still keeps the foot on (never below) the surface.
  graze: 0.0,             // allowed graze depth at bottom of sweep (0 ⇒ exact touch)
  surfaceLerp: 16.0,      // 1/s — how fast body-y eases to the target surface (smooth stair step-up)
  bobAmp: 0.05,           // vertical bob amplitude (juice), world units (downward-only)
  // cadence: ω = v / effectiveRadius. effectiveRadius == the CONTACT foot's lever
  // arm (reach + lineRadius) so the planted foot's world speed is v − ω·r == 0
  // (no slip, BY CONSTRUCTION). Longer reach ⇒ larger radius ⇒ lower ω: a long
  // leg makes long slow strides, a short leg quick small ones (Draw Climber feel).
  effRadiusMin: 0.32,
  // idle ω when v≈0 but a leg exists (so the foot doesn't sit dead) — tiny.
  idleOmega: 0.0,
};

export class Physics {
  constructor() {
    this.FIXED_DT = 1000 / 60;       // ms (kept name for game.js)
    this.SUBSTEPS = 1;               // kinematic — no substepping needed
    this.SUB_DT = this.FIXED_DT;     // ms (verifier reads SUB_DT for ω scaling)

    // ── public state the renderer & game read ──
    this.cube = null;                // { position:{x,y}, angle, velocity:{x,y} }
    this.legs = [];                  // [{ body, side, chain, pinLocal, lineRadius, ... }]
    this.floorBodies = [];           // render slabs ({ bounds, position, _dcTopY, label })
    this.legDrawn = false;
    this.startX = 0;
    this.finishX = 1;
    this._exploded = false;

    // ── kinematic state ──
    this._x = 0;                     // forward position (world)
    this._bodyY = 0;                 // cube centre y (physics, +down) — smoothed
    this._theta = 0;                 // master leg phase (rad)
    this._reach = 0;                 // current leg reach (world units)
    this._shape = null;              // shape descriptor (chain etc.)
    this._chain = null;              // axle-local chain (for both legs' visual)
    this._legPhaseOffset = Math.PI;  // second leg is 180° out of phase
    this._blocked = false;           // true when stopped at an unclimbable step
    this._vx = 0;                    // last realized forward speed (u/s)
    this._vTip = 0;                  // last foot tip linear speed (u/s)
    this._omega = 0;                 // last leg angular speed (rad/s)

    // gate used by the verifier's leg-driven assertion (motor-off ⇒ no motion).
    this.motorEnabled = true;
    // legacy fields some tuning scripts read (harmless no-ops now)
    this.motorSpeed = 0;
    this._fixedSpeed = 0;

    // segment height model
    this._segs = [];                 // [{ x0,x1, kind, surfFn(x), topY0, topY1, ... }]
    this._maxSurfaceTopY = 0;
  }

  reset() {
    this.cube = null;
    this.legs = [];
    this.floorBodies = [];
    this.legDrawn = false;
    this._exploded = false;
    this._segs = [];
    this._blocked = false;
    this._vx = 0;
    this._vTip = 0;
    this._theta = 0;
  }

  // ── TERRAIN: build floor slabs (for the renderer) + a height model. ──
  // Engine LOADS data only; nothing is hardcoded here beyond render cosmetics.
  buildTrack(track) {
    this.reset();
    this.startX = track.startX;
    this.finishX = track.finishX;

    let cursorX = track.startX - 3;  // start a little before the cube
    const groundY = 0;               // surface y; smaller y = higher (physics +down)
    const thick = FLOOR_THICK;
    let surfaceY = groundY;

    const addSlab = (cx, topY, len, slabH) => {
      const half = len / 2, halfH = slabH / 2;
      const b = {
        label: 'floor',
        position: { x: cx, y: topY + halfH },
        bounds: { min: { x: cx - half, y: topY }, max: { x: cx + half, y: topY + slabH } },
        _dcTopY: topY,
      };
      this.floorBodies.push(b);
      return b;
    };
    // record a flat surface segment over [x0,x1] at constant topY
    const addFlatSeg = (x0, x1, topY, kind) => {
      this._segs.push({ x0, x1, kind: kind || 'flat', topYa: topY, topYb: topY,
        surfFn: () => topY });
      if (topY < this._maxSurfaceTopY) this._maxSurfaceTopY = topY;
    };

    for (const seg of track.segments) {
      const len = seg.length;
      if (seg.kind === 'flat') {
        addSlab(cursorX + len / 2, surfaceY, len, thick);
        addFlatSeg(cursorX, cursorX + len, surfaceY, 'flat');
        cursorX += len;
      } else if (seg.kind === 'stairs') {
        const steps = seg.steps;
        const stepLen = len / steps;
        const stepH = seg.height / steps;   // per-step rise (designed climb unit)
        for (let i = 0; i < steps; i++) {
          surfaceY -= stepH;                // each step rises (y up = negative)
          // CAPTURE the tread top in a per-step CONST — `surfaceY` is a mutating
          // loop variable, so a closure over it would read the FINAL value for
          // every tread (all treads would collapse to the same height). treadY is
          // frozen per tread so each step sits at its own height (true staircase).
          const treadY = surfaceY;
          const slabH = thick + stepH * (i + 1);
          const cx = cursorX + stepLen / 2;
          addSlab(cx, treadY, stepLen, slabH);
          // each tread is a flat segment; its LEFT edge (x0) is the RISER the
          // walker must mount, carrying stepH for the climb rule.
          const x0 = cursorX, x1 = cursorX + stepLen;
          this._segs.push({ x0, x1, kind: 'stairs', topYa: treadY, topYb: treadY,
            stepH, surfFn: () => treadY });
          if (treadY < this._maxSurfaceTopY) this._maxSurfaceTopY = treadY;
          cursorX += stepLen;
        }
      } else if (seg.kind === 'ramp') {
        const dy = -(seg.height ?? 0);      // up = negative y
        const x0 = cursorX, x1 = cursorX + len;
        const topY0 = surfaceY, topY1 = surfaceY + dy;
        // render slab: a thick rect rotated to the ramp angle (visual only). We
        // approximate the ribbon with an axis box top at the mid height — the
        // height model below carries the true sloped surface.
        const midTop = (topY0 + topY1) / 2;
        addSlab(cursorX + len / 2, midTop, len, thick + Math.abs(dy));
        const slope = dy / len;
        this._segs.push({ x0, x1, kind: 'ramp', topYa: topY0, topYb: topY1, slope,
          surfFn: (px) => topY0 + slope * (px - x0) });
        if (topY1 < this._maxSurfaceTopY) this._maxSurfaceTopY = topY1;
        surfaceY += dy;
        cursorX += len;
      } else if (seg.kind === 'gap') {
        // no floor: model as a segment with no surface (surfFn null).
        const x0 = cursorX, x1 = cursorX + len;
        this._segs.push({ x0, x1, kind: 'gap', topYa: null, topYb: null, surfFn: () => null });
        cursorX += len;
      } else if (seg.kind === 'wall') {
        const h = seg.height ?? 1;
        // a vertical wall is an unclimbable riser of height h at cursorX.
        const b = {
          label: 'floor',
          position: { x: cursorX, y: surfaceY - h / 2 },
          bounds: { min: { x: cursorX - 0.3, y: surfaceY - h }, max: { x: cursorX + 0.3, y: surfaceY } },
          _dcTopY: surfaceY - h,
        };
        this.floorBodies.push(b);
        this._segs.push({ x0: cursorX - 0.3, x1: cursorX + 0.3, kind: 'wall',
          topYa: surfaceY, topYb: surfaceY, stepH: h, surfFn: () => surfaceY });
      }
    }

    this.surfaceY = surfaceY;

    // create the cube (positioned in setLegStroke once we know the reach)
    this.cube = {
      position: { x: track.startX, y: groundY - CUBE_SIZE / 2 - 0.05 },
      angle: 0,
      velocity: { x: 0, y: 0 },
      label: 'cube',
    };
    this._x = track.startX;
    this._bodyY = this.cube.position.y;
  }

  /**
   * Expected track surface y (physics, +down) at any x. Scans the segment model;
   * returns the HIGHEST (most negative) surface covering px, or null over a gap.
   */
  surfaceYAt(px) {
    let best = null;
    for (const s of this._segs) {
      if (px < s.x0 || px > s.x1) continue;
      const y = s.surfFn(px);
      if (y == null) continue;
      if (best == null || y < best) best = y;
    }
    return best;
  }

  /** Local segment under px (for terrain / climb decisions). */
  _segAt(px) {
    let found = null;
    for (const s of this._segs) {
      if (px < s.x0 || px > s.x1) continue;
      // prefer the highest surface (matches surfaceYAt)
      const y = s.surfFn(px);
      if (y == null) { if (!found) found = s; continue; }
      if (!found || (found.surfFn(px) == null) || y < found.surfFn(px)) found = s;
    }
    return found;
  }

  // ── LEG INPUT: build the drawn-stroke chain + set reach/shape. ──
  setLegStroke(points, spec = {}) {
    if (!this.cube) return;
    const scale = (spec.scale ?? 1.0) * LEG_WORLD_SCALE;

    this.legs = [];
    if (!points || points.length < 2) { this.legDrawn = false; return; }

    // 1. map normalized stroke → world (length preserved, no re-fit)
    const stroke = points.map((p) => ({ x: p.x * scale, y: p.y * scale }));

    // 2. resample to an evenly-spaced chain (bounded, shape-faithful)
    let chain = resamplePolyline(stroke, LEG_CIRCLE_SPACING, LEG_MAX_CIRCLES);
    if (chain.length < 2) { this.legDrawn = false; return; }

    // 3. anchor at the stroke START (chain[0]) → re-centre to box origin {0,0}.
    const anchor = { x: chain[0].x, y: chain[0].y };
    chain = chain.map((c) => ({ x: c.x - anchor.x, y: c.y - anchor.y }));

    // 4. reach = farthest sample from the anchor (drawn length), clamped.
    let rawReach = 0;
    for (const c of chain) rawReach = Math.max(rawReach, Math.hypot(c.x, c.y));
    if (rawReach < 1e-4) { this.legDrawn = false; return; }
    const reach = Math.max(LEG_REACH_MIN, Math.min(LEG_REACH_MAX, rawReach));
    if (Math.abs(reach - rawReach) > 1e-6) {
      const k = reach / rawReach;
      chain = chain.map((c) => ({ x: c.x * k, y: c.y * k }));
    }

    this._reach = reach;
    this._chain = chain;

    // 5. place the cube so the foot tip just touches the start-segment surface.
    //    The cube centre floats a designed clearance above the surface; the leg
    //    reaches DOWN to the ground (the reference look).
    const startSurfaceY = this.surfaceYAt(this.startX);
    const surf = (startSurfaceY == null) ? 0 : startSurfaceY;
    const clearance = this._bodyClearance(reach);
    const cubeY = surf - clearance;     // above the surface (physics +down)
    this.cube.position.x = this.startX;
    this.cube.position.y = cubeY;
    this.cube.velocity.x = 0; this.cube.velocity.y = 0;
    this.cube.angle = 0;
    this._x = this.startX;
    this._bodyY = cubeY;
    this._theta = 0;
    this._blocked = false;
    this._vx = 0;
    this._vTip = 0;

    // 6. build the two leg objects the renderer reads (visual + state only —
    //    NO Matter bodies). Both share the same chain (axle-local); side gives the
    //    z straddle; angle is set each step from the phase.
    const defs = [
      { side: -1, phaseOffset: 0 },
      { side: +1, phaseOffset: this._legPhaseOffset },
    ];
    for (const def of defs) {
      const legAngle = this._theta + def.phaseOffset;
      const body = {
        position: { x: this.cube.position.x + AXLE_X, y: this.cube.position.y + AXLE_Y },
        angle: legAngle,
        angularVelocity: 0,
        velocity: { x: 0, y: 0 },
        // a minimal "parts" array so the verifier's foot scan works: parts[0] is
        // a proxy, parts[1..] trace the chain in WORLD space (kept in sync each step).
        parts: this._buildParts(this.cube.position.x + AXLE_X, this.cube.position.y + AXLE_Y, legAngle, chain),
      };
      this.legs.push({
        body, side: def.side, phaseOffset: def.phaseOffset,
        radius: reach, chain, lineRadius: LEG_LINE_RADIUS, pinLocal: { x: 0, y: 0 },
      });
    }
    this.legDrawn = this.legs.length > 0;

    // legacy reads
    this.motorSpeed = 0;
    this._fixedSpeed = 0;
  }

  /** cube-centre clearance above the surface. The foot tip sweeps a circle of
   * radius (reach + lineRadius) about the centre, so floating the centre exactly
   * that far up makes the foot just GRAZE the surface at the bottom of the sweep
   * and stay ABOVE everywhere else (structural 0 penetration). We add bobAmp of
   * headroom because the bob oscillates the centre downward within it. */
  _bodyClearance(reach) {
    return reach + LEG_LINE_RADIUS + TUNE.bobAmp - TUNE.graze;
  }

  /** Build WORLD-space "parts" tracing the chain rotated by `angle` about the
   * axle (axleX,axleY). parts[0] is a proxy at the axle; parts[1..] are the chain
   * samples. The verifier's no-slip / foot scan reads parts[1..].position. */
  _buildParts(axleX, axleY, angle, chain) {
    const ca = Math.cos(angle), sa = Math.sin(angle);
    const parts = [{ position: { x: axleX, y: axleY } }]; // proxy
    for (const c of chain) {
      parts.push({ position: { x: axleX + c.x * ca - c.y * sa, y: axleY + c.x * sa + c.y * ca } });
    }
    return parts;
  }

  // ── DESIGNED speed / climb mappings ──

  /** Monotone length→speed: longer leg ⇒ faster (clamped). */
  legSpeedFactor(reach) {
    const f = Math.pow(reach / TUNE.refReach, TUNE.speedReachPow);
    return Math.max(TUNE.speedFactorMin, Math.min(TUNE.speedFactorMax, f));
  }

  /** Max step height a given reach can climb (designed rule). */
  maxClimbHeight(reach) {
    return Math.max(0, (reach - TUNE.climbBase) / TUNE.climbK);
  }

  /** Is a step of height h climbable by `reach`? */
  canClimb(reach, h) {
    return reach >= TUNE.climbBase + TUNE.climbK * h;
  }

  /** Effective rolling radius for ω = v/r. This is the CONTACT foot's lever arm
   * (== the body clearance == the distance from the axle/centre down to the foot
   * at the bottom of the sweep). Using exactly this makes the planted foot's world
   * velocity v − ω·r == 0 (no slip, structural). Long leg → long slow stride. */
  _effRadius(reach) {
    return Math.max(TUNE.effRadiusMin, reach + LEG_LINE_RADIUS);
  }

  // ── MAIN STEP ──
  update(dtMs, running) {
    if (!this.cube) return;
    const dt = dtMs / 1000; // seconds
    const drive = running && this.legDrawn && this.motorEnabled;

    let v = 0;
    if (drive) {
      const reach = this._reach;
      // 1. base * length factor
      v = TUNE.baseSpeed * this.legSpeedFactor(reach);

      // 2. terrain factor + CLIMB RULE (look a touch ahead so we stop AT the step)
      const lookX = this._x + CUBE_SIZE * 0.5;
      const seg = this._segAt(this._x);
      const aheadSeg = this._segAt(lookX);
      let terrain = 1;
      this._blocked = false;

      // climb rule: if a stairs/wall step lies just ahead, gate on reach.
      const riser = this._nextRiser(this._x, lookX + 0.5);
      if (riser && !this.canClimb(reach, riser.h)) {
        // blocked: stop just before the riser.
        v = 0;
        this._blocked = true;
        if (this._x > riser.x - CUBE_SIZE * 0.5) {
          this._x = riser.x - CUBE_SIZE * 0.5;
        }
      } else if (seg) {
        if (seg.kind === 'ramp') {
          // slope > 0 means descending (topY increases downhill since y is +down…
          // careful: y +down so going UP is slope<0). Use sign of slope.
          terrain = seg.slope < 0 ? lerpSlow(seg.slope, TUNE.uphillSlow) : lerpFast(seg.slope, TUNE.downhillFast);
        } else if (seg.kind === 'stairs' || (riser && this.canClimb(reach, riser.h))) {
          terrain = TUNE.stairClimbSlow;
        } else if (seg.kind === 'gap') {
          // over a gap there is no floor; the walker can't push — treat as blocked.
          v = 0; this._blocked = true;
        }
      }
      v *= terrain;

      // 3. advance
      const adv = v * dt;
      const nextX = this._x + adv;
      if (nextX < this.finishX + 1) this._x = nextX;
    }
    this._vx = v;

    // 4. body y rides the surface (smoothly), foot never goes below it. We track
    //    the HIGHEST surface across the body footprint (centre ± half-cube), so as
    //    soon as the LEADING edge meets a step the body starts rising — it never
    //    lags into the riser. Easing DOWN is smooth; rising UP snaps to the target
    //    (clamp) so the foot can never penetrate the step it just mounted.
    const half = CUBE_SIZE * 0.5 + LEG_LINE_RADIUS;
    let topSurf = null; // most-negative (highest) surface under the footprint
    for (let sx = this._x - half; sx <= this._x + half + 1e-6; sx += half) {
      const s = this.surfaceYAt(sx);
      if (s != null && (topSurf == null || s < topSurf)) topSurf = s;
    }
    if (topSurf != null) {
      const targetY = topSurf - this._bodyClearance(this._reach);
      if (targetY < this._bodyY) {
        // surface rose (step up) — snap up so the foot never dips into the step.
        this._bodyY = targetY;
      } else {
        // surface fell (step down / descent) — ease down smoothly (juice).
        const a = 1 - Math.exp(-TUNE.surfaceLerp * dt);
        this._bodyY += (targetY - this._bodyY) * a;
      }
    }
    // bob: small phase-locked walking oscillation (juice). DOWNWARD-ONLY within
    // the bob headroom built into _bodyClearance, so the foot never dips below
    // the surface ((1-cos)/2 ∈ [0,1] ⇒ bob ∈ [0,bobAmp], physics +down).
    const bob = (drive && v > 1e-3)
      ? ((1 - Math.cos(this._theta * 2)) * 0.5) * TUNE.bobAmp : 0;

    this.cube.position.x = this._x;
    this.cube.position.y = this._bodyY + bob;
    this.cube.velocity.x = v;
    this.cube.angle = 0;

    // 5. leg phase advances by ω = v / effectiveRadius (no-slip rolling). When
    //    v≈0 the legs stop too (planted). Faster ⇒ spins faster (cadence).
    let omega = 0;
    if (drive) {
      const r = this._effRadius(this._reach);
      omega = (v > 1e-6) ? (v / r) : TUNE.idleOmega;
      this._theta += omega * dt;
      this._vTip = omega * r; // == v (no-slip)
    } else {
      this._vTip = 0;
    }
    this._omega = omega; // rad/s — recorded on each leg.body in _syncLegs

    // 6. sync the two leg visuals + their world parts. Foot lowest point is
    //    clamped to sit ON the surface (never below) — structural 0 penetration.
    this._syncLegs();

    // safety: NaN guard (kinematic can't explode, but assert anyway)
    if (!Number.isFinite(this._x) || !Number.isFinite(this._bodyY)) this._exploded = true;
  }

  /** Find the next upward riser (stairs/wall step) whose base lies in (fromX, toX].
   * Returns { x, h } or null. A riser is the boundary where the surface JUMPS UP
   * (topY decreases) between adjacent segments. */
  _nextRiser(fromX, toX) {
    let best = null;
    for (const s of this._segs) {
      if (s.kind === 'stairs' && s.stepH > 0) {
        // the riser sits at the LEFT edge of a stairs tread (x0).
        if (s.x0 > fromX && s.x0 <= toX) {
          if (!best || s.x0 < best.x) best = { x: s.x0, h: s.stepH };
        }
      } else if (s.kind === 'wall' && s.stepH > 0) {
        const rx = (s.x0 + s.x1) / 2;
        if (rx > fromX && rx <= toX) {
          if (!best || rx < best.x) best = { x: rx, h: s.stepH };
        }
      }
    }
    return best;
  }

  /** Recompute both legs' world transform + parts from the current cube + phase.
   * With the cube floating clearance = reach+lineRadius+bob above the surface the
   * foot can NEVER dip below it (structural), so this is a pure transform rebuild
   * plus a tiny defensive UP-ONLY clamp (never lowers, never disturbs the smooth
   * descent ease) that catches any sub-epsilon round-off. */
  _syncLegs() {
    if (!this.legs.length) return;
    const axleX = this.cube.position.x + AXLE_X;
    const axleY = this.cube.position.y + AXLE_Y;
    // Matter convention the verifier reads: body.angularVelocity is the per-SUB_DT
    // delta-angle (it divides by SUB_DT/1000 to recover rad/s). We are kinematic,
    // so record ω·(SUB_DT/1000) to keep that reconstruction exact.
    const angVelMatter = (this._omega || 0) * (this.SUB_DT / 1000);
    for (const l of this.legs) {
      const angle = this._theta + l.phaseOffset;
      l.body.position.x = axleX;
      l.body.position.y = axleY;
      l.body.angle = angle;
      l.body.angularVelocity = angVelMatter;
      l.body.parts = this._buildParts(axleX, axleY, angle, l.chain);
    }
    // defensive up-only clamp against the local surface under the foot.
    const surf = this.surfaceYAt(this.cube.position.x);
    if (surf == null) return;
    let lowY = -Infinity;
    for (const l of this.legs) {
      for (let i = 1; i < l.body.parts.length; i++) {
        const py = l.body.parts[i].position.y + l.lineRadius;
        if (py > lowY) lowY = py;
      }
    }
    const below = lowY - surf; // >0 ⇒ penetrating (should be ~0 by construction)
    if (below > 1e-6) {
      this.cube.position.y -= below;
      const nAxleY = this.cube.position.y + AXLE_Y;
      for (const m of this.legs) {
        m.body.position.y = nAxleY;
        m.body.parts = this._buildParts(axleX, nAxleY, this._theta + m.phaseOffset, m.chain);
      }
    }
  }

  // ── getters game.js / renderer.js read ──
  get bodyX() { return this.cube ? this.cube.position.x : this.startX; }
  get bodyY() { return this.cube ? this.cube.position.y : 0; }
  get progress() {
    const t = (this.bodyX - this.startX) / (this.finishX - this.startX);
    return Math.max(0, Math.min(1, t));
  }
  get exploded() { return this._exploded; }

  // ── verification helpers (kept API-compatible) ──

  /** pivot diagnostics — axle == cube centre, plus float-above-surface. */
  pivotInfo() {
    if (!this.cube) return null;
    const cubeCenterY = this.cube.position.y;
    const axleY = this.cube.position.y + AXLE_Y;
    const surfaceY = this.surfaceYAt(this.cube.position.x);
    return {
      axleY, cubeCenterY, gap: Math.abs(axleY - cubeCenterY),
      reach: this.legs[0] ? this.legs[0].radius : null,
      aboveSurface: surfaceY != null ? (surfaceY - cubeCenterY) : null,
    };
  }

  /** Geometry metrics of leg[0] in its LOCAL (axle-at-origin) frame. */
  legMetrics() {
    const l = this.legs[0];
    if (!l) return null;
    const ch = l.chain, r = l.lineRadius;
    let reach = 0, strokeLen = 0;
    let minx = Infinity, miny = Infinity, maxx = -Infinity, maxy = -Infinity;
    for (let i = 0; i < ch.length; i++) {
      const c = ch[i];
      reach = Math.max(reach, Math.hypot(c.x, c.y));
      minx = Math.min(minx, c.x); maxx = Math.max(maxx, c.x);
      miny = Math.min(miny, c.y); maxy = Math.max(maxy, c.y);
      if (i > 0) strokeLen += Math.hypot(c.x - ch[i - 1].x, c.y - ch[i - 1].y);
    }
    const bboxArea = Math.max(0, maxx - minx) * Math.max(0, maxy - miny);
    const lineArea = strokeLen * 2 * r + Math.PI * r * r;
    const hullFillArea = convexHullArea(ch.concat([{ x: 0, y: 0 }]));
    return { reach, strokeLen, lineRadius: r, bboxArea, lineArea, hullFillArea, parts: ch.length };
  }

  /** anchor diagnostics — pinned by stroke START (chain[0]), one-sided limb. */
  legAnchorInfo() {
    const l = this.legs[0];
    if (!l || !l.body || !l.body.parts) return null;
    const ch = l.chain;
    const axleWorld = { x: this.cube.position.x + AXLE_X, y: this.cube.position.y + AXLE_Y };
    const startPart = l.body.parts.length > 1 ? l.body.parts[1] : l.body.parts[0];
    const axleToStartWorld = Math.hypot(startPart.position.x - axleWorld.x, startPart.position.y - axleWorld.y);
    let reach = 0, cxs = 0, cys = 0;
    for (const c of ch) { reach = Math.max(reach, Math.hypot(c.x, c.y)); cxs += c.x; cys += c.y; }
    const cx = cxs / ch.length, cy = cys / ch.length;
    const centroidToAxle = Math.hypot(cx, cy);
    const centroidFrac = reach > 1e-6 ? centroidToAxle / reach : 0;
    return { axleToStartWorld, reach, centroidToAxle, centroidFrac };
  }

  // expose the designed mappings for the verifier
  get tune() { return TUNE; }
}

// ── helpers ──
function clamp01(v) { return Math.max(0, Math.min(1, v)); }

// terrain blend helpers: slope is dy/len in physics y (+down). Going UP ⇒ slope<0.
function lerpSlow(slope, slowFactor) {
  // steeper up (more negative slope) ⇒ closer to slowFactor; flat ⇒ 1.
  const s = Math.min(1, Math.abs(slope) * 2);
  return 1 + (slowFactor - 1) * s;
}
function lerpFast(slope, fastFactor) {
  const s = Math.min(1, Math.abs(slope) * 2);
  return 1 + (fastFactor - 1) * s;
}

function convexHullArea(pts) {
  if (pts.length < 3) return 0;
  const P = pts.slice().sort((a, b) => a.x - b.x || a.y - b.y);
  const cross = (o, a, b) => (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
  const lower = [];
  for (const p of P) { while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], p) <= 0) lower.pop(); lower.push(p); }
  const upper = [];
  for (let i = P.length - 1; i >= 0; i--) { const p = P[i]; while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], p) <= 0) upper.pop(); upper.push(p); }
  const hull = lower.slice(0, -1).concat(upper.slice(0, -1));
  let area = 0;
  for (let i = 0; i < hull.length; i++) { const a = hull[i], b = hull[(i + 1) % hull.length]; area += a.x * b.y - b.x * a.y; }
  return Math.abs(area) / 2;
}

/** Resample a polyline into evenly-spaced points (bounded by maxPts). */
function resamplePolyline(pts, spacing, maxPts) {
  if (pts.length < 2) return pts.slice();
  let total = 0;
  for (let i = 1; i < pts.length; i++) total += Math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
  if (total < 1e-6) return [pts[0]];
  const eff = Math.max(spacing, total / (maxPts - 1));
  const out = [{ x: pts[0].x, y: pts[0].y }];
  let acc = 0, next = eff;
  for (let i = 1; i < pts.length; i++) {
    let ax = pts[i - 1].x, ay = pts[i - 1].y;
    const bx = pts[i].x, by = pts[i].y;
    let segLen = Math.hypot(bx - ax, by - ay);
    while (segLen > 1e-9 && acc + segLen >= next) {
      const t = (next - acc) / segLen;
      const nx = ax + (bx - ax) * t, ny = ay + (by - ay) * t;
      out.push({ x: nx, y: ny });
      const consumed = next - acc;
      ax = nx; ay = ny; segLen -= consumed; acc = next; next += eff;
    }
    acc += segLen;
  }
  const last = pts[pts.length - 1];
  const tail = out[out.length - 1];
  if (Math.hypot(last.x - tail.x, last.y - tail.y) > eff * 0.25) out.push({ x: last.x, y: last.y });
  return out;
}

/** Built-in leg presets (box [-1,1]^2) — unchanged from the old build so the
 * verifier's named strokes (wheel/short/long/L/ring/limb/...) keep meaning. */
export function presetStroke(name) {
  if (name === 'wheel') {
    const pts = []; const N = 7;
    for (let i = 0; i < N; i++) { const a = (i / N) * Math.PI * 2 + Math.PI / 2; pts.push({ x: Math.cos(a) * 0.85, y: Math.sin(a) * 0.85 }); }
    return pts;
  }
  if (name === 'stick') return [{ x: -0.9, y: 0 }, { x: 0.9, y: 0 }];
  if (name === 'hook') return [{ x: -0.2, y: -0.8 }, { x: -0.2, y: 0.4 }, { x: 0.5, y: 0.7 }, { x: 0.85, y: 0.2 }];
  if (name === 'limb') return [{ x: 0.0, y: 0.0 }, { x: 0.0, y: 0.55 }, { x: 0.45, y: 0.95 }];
  if (name === 'limb_long') return [{ x: 0.0, y: -0.2 }, { x: 0.1, y: 0.5 }, { x: 0.55, y: 0.95 }, { x: 0.95, y: 0.8 }];
  if (name === 'limb_short') return [{ x: 0.0, y: 0.0 }, { x: 0.18, y: 0.28 }, { x: 0.42, y: 0.36 }];
  if (name === 'short') return [{ x: -0.32, y: 0.18 }, { x: 0.32, y: -0.18 }];
  if (name === 'long') return [{ x: -0.95, y: 0.55 }, { x: 0.95, y: -0.55 }];
  if (name === 'L') return [{ x: -0.2, y: -0.9 }, { x: -0.2, y: 0.5 }, { x: 0.9, y: 0.5 }];
  if (name === 'arc') {
    const pts = []; const N = 14;
    for (let i = 0; i <= N; i++) { const a = Math.PI * (0.15 + (i / N) * 0.7); pts.push({ x: Math.cos(a) * 0.9, y: Math.sin(a) * 0.9 }); }
    return pts;
  }
  if (name === 'short_bar') return [{ x: 0.0, y: 0.0 }, { x: 0.0, y: 0.45 }];
  if (name === 'long_bar') return [{ x: 0.0, y: 0.0 }, { x: 0.05, y: 1.0 }];
  if (name === 'blob') return [{ x: 0.0, y: 0.0 }, { x: 0.15, y: 0.1 }, { x: 0.05, y: 0.25 }, { x: 0.22, y: 0.2 }, { x: 0.12, y: 0.32 }, { x: 0.28, y: 0.3 }];
  if (name === 'arc_big') {
    const pts = []; const N = 16;
    for (let i = 0; i <= N; i++) { const a = (i / N) * Math.PI * 1.1; pts.push({ x: (1 - Math.cos(a)) * 0.55, y: Math.sin(a) * 0.85 }); }
    return pts;
  }
  if (name === 'zigzag') return [{ x: 0.0, y: 0.0 }, { x: 0.25, y: 0.2 }, { x: -0.1, y: 0.4 }, { x: 0.35, y: 0.55 }, { x: 0.0, y: 0.75 }, { x: 0.4, y: 0.9 }];
  if (name === 'ring') {
    const pts = []; const N = 22;
    for (let i = 0; i <= N; i++) { const a = (i / N) * Math.PI * 1.9; pts.push({ x: Math.cos(a) * 0.9, y: Math.sin(a) * 0.9 }); }
    return pts;
  }
  return [{ x: -0.9, y: 0 }, { x: 0.9, y: 0 }];
}

export const PHYS_CONST = { CUBE_SIZE, AXLE_X, AXLE_Y };
