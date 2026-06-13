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
  // BODY TILT (reference look): the cube leans to match the LOCAL surface tangent
  // — nose up on an ascent, nose down on a descent, level on the flat. We measure
  // the slope by sampling the surface a small dx either side of the body and take
  // atan of the rise/run, then EASE cube.angle toward it (no snap). On stairs we
  // tilt to the staircase's OVERALL diagonal (not the saw-tooth of each tread).
  tiltDx: 0.55,          // half-width (world u) of the slope-probe around the body
  // The dynamic T01 chains STEEP up→down ramps that are only ~4u long, so the lean
  // must swing ~1.1rad (nose-up → nose-down) across a short descent. A too-slow ease
  // never reaches the descent target before the next seam (the body stays ~level on
  // short descents). We raise the responsiveness so the lean tracks short ramps while
  // staying under the (I) per-frame-snap cap (0.03 rad/frame).
  tiltLerp: 9.0,         // 1/s — how fast cube.angle eases to the target tilt
  tiltSlewMax: 0.029,    // rad/frame cap on cube.angle change (slew limiter ⇒ no per-frame lean snap at segment seams; just under the (I) DANG_MAX 0.03 gate)
  tiltMax: 0.65,         // rad — clamp so a near-vertical step can't flip the body (§E: lowered 0.85→0.65, the reference lean is gentler)
  tiltGain: 1.0,         // scale on the measured tangent angle (1 = exact match)
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
  surfaceLerp: 9.0,       // 1/s — how fast the BASE body-y (terrain follow, bob-free) eases to the target surface (smooth stair step-up; lower ⇒ gentler, no per-frame snap)
  surfaceSlewMax: 0.04,   // world u/frame cap on the BASE body-y change (slew limiter ⇒ structurally no per-frame snap; > the true rise-rate so it never blocks the climb)
  // ── GEOMETRIC WALKING BOB (reference look) ──
  // The body height is NOT a constant clearance + sine. It is DERIVED from the
  // legs' real contact geometry: each leg (a rigid rotated chain) has a current
  // "support depth" = the vertical drop from the cube centre to its DEEPEST
  // (ground-side) chain point + lineRadius. A leg pointing straight DOWN has its
  // farthest sample directly below ⇒ depth ≈ reach+r (MAX) ⇒ body floats HIGH; a
  // tilted leg's lowest point is shallower ⇒ body DROPS. support = max(depthL,
  // depthR) (the deeper leg carries the body). The two legs are 180° out of phase,
  // so support oscillates → the body rises when a leg plants vertically and dips
  // between plants: a real alternating walking bob, by construction.
  // We let the body float at `surfaceY - support`, smoothing only the LOW-FREQUENCY
  // terrain trend (a bob-free BASE y) and adding the (target − base) bob on top, so
  // the cube visibly bobs IN-FRAME while the camera (which follows the base) stays
  // smooth. bobGain scales the visible amplitude (1 = the raw geometric bob).
  bobGain: 1.0,           // scale on the geometric bob amplitude (visible "juice")
  bobMax: 0.5,            // world-u clamp on the per-frame bob excursion (anti-motion-sickness)
  // cadence: ω = v / effectiveRadius. effectiveRadius == the CONTACT foot's lever
  // arm (reach + lineRadius) so the planted foot's world speed is v − ω·r == 0
  // (no slip, BY CONSTRUCTION). Longer reach ⇒ larger radius ⇒ lower ω: a long
  // leg makes long slow strides, a short leg quick small ones (Draw Climber feel).
  effRadiusMin: 0.32,
  // idle ω when v≈0 but a leg exists (so the foot doesn't sit dead) — tiny.
  idleOmega: 0.0,

  // ── AIRBORNE / CREST JUMP (designed, physical) ──
  // The reference walker, when it hits a CONVEX crest (flat/up → sudden steep
  // down, a hilltop) WITH SPEED, LAUNCHES off the edge and flies a ballistic arc,
  // landing further down the descent — it does NOT glue to the surface. We model
  // this as a designed state: while GROUNDED the body rides surfaceY; at a crest
  // where the surface's downward acceleration (v²·curvature) exceeds gravity, the
  // foot can no longer push on the ground → we switch to AIRBORNE and integrate a
  // true parabola (x keeps its forward v, y under gravity g) until the arc meets
  // the surface again (LAND → GROUNDED). Faster / sharper crest ⇒ farther flight;
  // gentle slopes or low speed never trigger (no false jumps on the flat).
  gravity: 26.0,         // world u/s² downward (physics +down) — arc fall rate
  crestLookDx: 0.7,      // half-window (world u) to finite-difference the surface slope around the body for crest detection
  crestAheadDx: 0.7,     // how far AHEAD (world u) to measure the post-crest slope (the descent the body would have to follow)
  // LAUNCH condition: the surface curls DOWN by Δslope over the crest window. The
  // ground's downward acceleration if the body STAYED on it is a_surf ≈ v²·dSlope/dx.
  // When a_surf > gravity·launchMargin the foot would have to be PULLED down faster
  // than free-fall (impossible — no glue) ⇒ the body leaves the surface. margin>1
  // gives a little hysteresis so only a real, speed-backed crest launches (no jitter).
  launchMargin: 1.0,     // a_surf must exceed gravity·this to launch (≥1 ⇒ true ballistic departure)
  minLaunchSpeed: 3.5,   // u/s — below this horizontal speed the body never launches (slow walks stay grounded)
  minDropSlope: 0.18,    // the post-crest descent slope (physics +down, downhill>0) must exceed this — gentle dips don't launch
  airTiltLerp: 6.0,      // 1/s — in the air the body eases its lean toward the FLIGHT-PATH angle (atan(vy/vx)) for a natural arc pose
  landMergeLerp: 14.0,   // 1/s — on LAND, how fast the body re-settles onto the ground support (a soft touchdown, no snap)
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
    this._bodyY = 0;                 // cube centre y (physics, +down) — FULL height (base + geometric bob)
    this._bodyBaseY = 0;             // bob-FREE terrain-follow body-y (camera tracks this ⇒ screen smooth while cube bobs in-frame)
    this._bob = 0;                   // current geometric bob excursion (physics +down; >0 ⇒ body dipped below base)
    this._angle = 0;                 // cube tilt (rad) — eased toward surface tangent
    this._theta = 0;                 // master leg phase (rad)
    this._reach = 0;                 // current leg reach (world units)
    this._shape = null;              // shape descriptor (chain etc.)
    this._chain = null;              // axle-local chain (for both legs' visual)
    this._legPhaseOffset = Math.PI;  // second leg is 180° out of phase
    this._blocked = false;           // true when stopped at an unclimbable step
    this._blockedByRiser = false;    // §C: blocked specifically by a riser (climb) — legs keep trying
    this._trying = false;            // §C: true while struggling in place (legs churn, x≈0)
    this._vx = 0;                    // last realized forward speed (u/s)
    this._vTip = 0;                  // last foot tip linear speed (u/s)
    this._omega = 0;                 // last leg angular speed (rad/s)

    // ── AIRBORNE state (crest jump) ──
    this._air = false;               // true while in a ballistic flight (off the ground)
    this._vy = 0;                    // vertical velocity of the GROUND-CONTACT level (physics +down; up = negative)
    this._footBaseY = 0;             // y of the foot-contact level (surfaceY-equivalent the body floats on); grounded == surfaceY, airborne == ballistic
    this._prevFootBaseY = 0;         // previous frame's foot-contact level (to estimate vertical velocity at launch)
    this._airFrames = 0;             // frames spent airborne in the current flight (diagnostic)
    this._landMerge = 0;             // 0..1 touchdown re-settle blend (1 right after landing → eases to 0)

    // gate used by the verifier's leg-driven assertion (motor-off ⇒ no motion).
    this.motorEnabled = true;
    // RIVAL/RACE: a forward-speed multiplier (1 = the leg's natural pace). The
    // computer opponent scales its walker's pace via this so it can be tuned
    // "competitive". ω is derived from the REALIZED v (post-pace), so no-slip /
    // no-penetration hold structurally at ANY pace. Player keeps paceFactor=1.
    this.paceFactor = 1.0;
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
    this._blockedByRiser = false;
    this._trying = false;
    this._vx = 0;
    this._vTip = 0;
    this._theta = 0;
    this._angle = 0;
    this._air = false;
    this._vy = 0;
    this._airFrames = 0;
    this._landMerge = 0;
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
    // a RAMP slab: an oriented box whose TOP face is the sloped surface. We pass the
    // two end heights so the renderer can rotate the slab to the ramp angle (a
    // tilted slab, not a flat box at mid-height). The render uses physics y (+down).
    const addRampSlab = (x0, x1, topY0, topY1, slabH) => {
      const cx = (x0 + x1) / 2;
      const len = x1 - x0;
      const span = Math.hypot(len, (topY1 - topY0)); // along-slope length
      const b = {
        label: 'floor', kind: 'ramp',
        position: { x: cx, y: (topY0 + topY1) / 2 + slabH / 2 },
        bounds: { min: { x: x0, y: Math.min(topY0, topY1) },
                  max: { x: x1, y: Math.max(topY0, topY1) + slabH } },
        _dcTopY: Math.min(topY0, topY1),
        _dcRamp: { x0, x1, topY0, topY1, len, span, slabH },
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
        // render slab: a TILTED slab whose TOP face IS the sloped surface (matches
        // the reference's smooth hills) — NOT a flat box at mid-height. The height
        // model below carries the same true sloped surface for the walker.
        addRampSlab(x0, x1, topY0, topY1, thick);
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
    this._bodyBaseY = this.cube.position.y;
    this._bob = 0;
    this._air = false;
    this._vy = 0;
    this._airFrames = 0;
    this._landMerge = 0;
    const startSurf0 = this.surfaceYAt(track.startX);
    this._footBaseY = (startSurf0 == null) ? groundY : startSurf0;
    this._prevFootBaseY = this._footBaseY;
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

  /** Local surface SLOPE (d(physY)/dx, physics +down ⇒ downhill > 0) at px. Prefers
   * the segment's ANALYTIC slope (ramps are linear) to avoid a finite-difference
   * spike at a seam; falls back to a small central difference over a gap/unknown. */
  surfaceSlopeAt(px) {
    const s = this._segAt(px);
    if (s && s.kind === 'ramp') return s.slope;        // downhill > 0 (y +down)
    if (s && (s.kind === 'flat' || s.kind === 'wall' || s.kind === 'stairs')) return 0;
    const dx = 0.35;
    const yR = this.surfaceYAt(px + dx), yL = this.surfaceYAt(px - dx);
    if (yR == null || yL == null) return 0;
    return (yR - yL) / (2 * dx);
  }

  /** Surface height used to FLOAT THE BODY (not the physical surface). Over stairs
   * we return the run's smooth DIAGONAL (lerp first→last tread) instead of the
   * stepped saw-tooth, so the body glides up the staircase with no per-frame snap
   * (reference look). The diagonal is ALWAYS at or ABOVE the stepped treads (a
   * staircase's hypotenuse sits above its steps), so floating to it keeps the foot
   * ON/above every tread — zero penetration is preserved. Elsewhere == surfaceYAt. */
  bodySurfaceYAt(px) {
    const seg = this._segAt(px);
    if (seg && seg.kind === 'stairs') {
      // find the contiguous stair RUN containing px (abutting stair treads).
      let x0 = seg.x0, x1 = seg.x1, ya = seg.surfFn(seg.x0), yb = seg.surfFn(seg.x1);
      for (const s of this._segs) {
        if (s.kind !== 'stairs') continue;
        if (s.x1 > x0 - 1e-6 && s.x0 < x1 + 1e-6) {
          if (s.x0 < x0) { x0 = s.x0; ya = s.surfFn(s.x0); }
          if (s.x1 > x1) { x1 = s.x1; yb = s.surfFn(s.x1); }
        }
      }
      const run = Math.max(1e-3, x1 - x0);
      // Ride the staircase HYPOTENUSE — the line through each tread's top-OUTER
      // corner — which sits at or ABOVE every tread top. Floating the body to this
      // line keeps the foot on/above every tread (zero penetration) while the body
      // glides up a single smooth diagonal (no per-step snap). For an ASCENT (yb<ya)
      // bias the START end up by a full step (the hypotenuse leads the first tread);
      // for a DESCENT bias the END end. We bias both ends up by one step which is
      // safe (always ≥ the treads) and symmetric.
      const stepRise = seg.stepH || 0;
      const yTop0 = ya - stepRise, yTop1 = yb - stepRise;
      const t = clamp01((px - x0) / run);
      return yTop0 + (yTop1 - yTop0) * t;
    }
    return this.surfaceYAt(px);
  }

  /** Target BODY TILT (cube.angle, physics convention) at forward position px.
   *
   * The cube leans to match the LOCAL surface TANGENT — nose up on an ascent,
   * down on a descent, level on the flat (the reference look). We measure the
   * tangent by finite-differencing the surface a small dx either side of the
   * body. On STAIRS we tilt to the staircase's OVERALL diagonal (the run between
   * the first and last tread of the run), NOT the per-tread saw-tooth, so the
   * body climbs the steps along a single smooth diagonal instead of jittering.
   *
   * Sign: physics y is +down and the renderer draws cube.rotation.z = -cube.angle
   * with render-y = -phys-y. Setting cube.angle = atan(dPhysY/dx) makes the screen
   * rotation = atan(dRenderY/dx) (the visible surface tangent): uphill ⇒ CCW nose-up,
   * downhill ⇒ CW nose-down, flat ⇒ 0. (Derived & checked in the verifier (H).)
   */
  _targetTilt(px) {
    const seg = this._segAt(px);
    // On a stair run, lean to the whole run's diagonal (first→last tread).
    if (seg && seg.kind === 'stairs') {
      let x0 = seg.x0, x1 = seg.x1, ya = seg.surfFn(seg.x0), yb = seg.surfFn(seg.x1);
      for (const s of this._segs) {
        if (s.kind !== 'stairs') continue;
        // contiguous stair treads share the same run if they abut.
        if (s.x1 > x0 - 1e-6 && s.x0 < x1 + 1e-6) {
          if (s.x0 < x0) { x0 = s.x0; ya = s.surfFn(s.x0); }
          if (s.x1 > x1) { x1 = s.x1; yb = s.surfFn(s.x1); }
        }
      }
      const run = Math.max(1e-3, x1 - x0);
      const slope = (yb - ya) / run;          // physics slope (+down): up ⇒ negative
      return clampMag(Math.atan(slope) * TUNE.tiltGain, TUNE.tiltMax);
    }
    // Ramp: use the segment's ANALYTIC slope (constant along a ramp). This avoids a
    // finite-difference spike when the probe window straddles a sharp seam/riser at
    // a segment boundary (e.g. a ramp→stairs riser) which would briefly over-tilt
    // the body. A flat segment has slope 0 ⇒ level.
    if (seg && seg.kind === 'ramp') {
      return clampMag(Math.atan(seg.slope) * TUNE.tiltGain, TUNE.tiltMax);
    }
    if (seg && (seg.kind === 'flat' || seg.kind === 'wall')) return 0;
    // gap / unknown: finite-difference (and hold the current lean over a gap).
    const dx = TUNE.tiltDx;
    const yR = this.surfaceYAt(px + dx);
    const yL = this.surfaceYAt(px - dx);
    if (yR == null || yL == null) return this._angle;   // over a gap — hold current lean
    const slope = (yR - yL) / (2 * dx);                 // physics slope (+down)
    return clampMag(Math.atan(slope) * TUNE.tiltGain, TUNE.tiltMax);
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
  // CONTINUE-ON-REDRAW (§A): the core fun loop is "the path changes → redraw the
  // leg → keep going". So a redraw must NOT teleport back to startX. We CONTINUE
  // from the CURRENT forward position whenever a leg already exists (i.e. the user
  // is re-drawing mid-run). Only a FRESH placement — the very first leg of a track
  // (no legs yet) — anchors at startX. Callers can force a fresh placement with
  // spec.fresh (used by buildTrack/restart paths). The new leg's reach changes the
  // float height, so we re-FLOAT vertically at the current x (the cube hovers to the
  // new leg's support depth) while x / progress / phase are PRESERVED.
  setLegStroke(points, spec = {}) {
    if (!this.cube) return;
    const scale = (spec.scale ?? 1.0) * LEG_WORLD_SCALE;
    // continue if legs already exist (mid-run redraw) UNLESS a fresh start is forced.
    const hadLegs = this.legs.length > 0 && this.legDrawn;
    const fresh = spec.fresh === true || !hadLegs;

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

    // 5. place / re-float the cube.
    //    • FRESH (first leg of a track): anchor at startX, level, phase reset — the
    //      foot tip just grazes the start-segment surface (the original behaviour).
    //    • CONTINUE (mid-run redraw): KEEP x / progress / spin phase / tilt. Only
    //      re-FLOAT the vertical height at the CURRENT x for the NEW leg's support
    //      depth, so the cube hovers to the new leg without dropping/teleporting and
    //      the run carries on from where it was. This is the "redraw to adapt, keep
    //      going" core loop — NO restart, NO progress loss.
    if (fresh) {
      const startSurfaceY = this.surfaceYAt(this.startX);
      const surf = (startSurfaceY == null) ? 0 : startSurfaceY;
      // Float the cube centre so the DEEPER leg's lowest contact point just grazes
      // the surface (geometric support, NOT a constant clearance). The bob is the
      // by-product of this depth changing as the legs spin. At θ=0/tilt=0 this is the
      // creation pose's support depth.
      const support0 = this._supportDepth(0, 0);
      const cubeY = surf - support0;     // above the surface (physics +down)
      this.cube.position.x = this.startX;
      this.cube.position.y = cubeY;
      this.cube.velocity.x = 0; this.cube.velocity.y = 0;
      this.cube.angle = 0;
      this._x = this.startX;
      this._bodyY = cubeY;
      this._bodyBaseY = cubeY;
      this._bob = 0;
      this._angle = 0;
      this._theta = 0;
      this._blocked = false;
      this._vx = 0;
      this._vTip = 0;
      this._air = false;
      this._vy = 0;
      this._airFrames = 0;
      this._landMerge = 0;
      this._footBaseY = surf;
      this._prevFootBaseY = surf;
    } else if (this._air) {
      // CONTINUE while AIRBORNE: a mid-flight redraw must NOT teleport the body to the
      // ground (that would be a fake landing). Keep the flight going — x / θ / tilt /
      // _vy / _footBaseY (the ballistic level) all PRESERVED. Only the leg shape (reach)
      // changed; the arc continues and the next update() step keeps integrating gravity.
      this.cube.velocity.x = 0; this.cube.velocity.y = 0;
      this.cube.angle = this._angle;
      this._blocked = false;
      this._vx = 0;
      this._vTip = 0;
      // _x, _theta, _angle, _air, _vy, _footBaseY, _bodyY, _bodyBaseY all PRESERVED.
    } else {
      // CONTINUE (GROUNDED): re-float vertically at the current x for the new leg's
      // reach. x, _theta, _angle, progress are all preserved (carry on running). The
      // foot grazes the surface continuously (no float), so we place the cube so the
      // CURRENT support touches the surface.
      const curX = this._x;
      const surfNow = this.surfaceYAt(curX);
      const surf = (surfNow == null) ? this.cube.position.y : surfNow;
      const supportNow = this._supportDepth(this._theta, this._angle);
      const cubeY = surf - supportNow;
      this.cube.position.x = curX;        // x UNCHANGED — continue from here
      this.cube.position.y = cubeY;
      this.cube.velocity.x = 0; this.cube.velocity.y = 0;
      this.cube.angle = this._angle;
      this._bodyY = cubeY;
      this._bodyBaseY = cubeY;
      this._bob = 0;
      this._blocked = false;
      this._vx = 0;
      this._vTip = 0;
      this._footBaseY = surf;
      this._prevFootBaseY = surf;
      // _x, _theta, _angle intentionally PRESERVED.
    }

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

  /** Vertical support DEPTH of one leg at world angle `legAngle` — the drop from
   * the cube centre (pivot) DOWN to the leg's DEEPEST (ground-side) chain point,
   * plus lineRadius. Physics y is +down, so for a local chain point (cx,cy) the
   * vertical drop after rotating by legAngle is cx·sin(θ) + cy·cos(θ); the deepest
   * point maximizes it. A leg pointing straight DOWN (its farthest sample directly
   * below the pivot) gives depth ≈ reach + r (MAX); a tilted leg's lowest point is
   * shallower ⇒ smaller depth ⇒ the body drops. This is the geometric walking bob:
   * with the two legs 180° apart, support = max(depthL,depthR) oscillates so the
   * body rises on a vertical plant and dips between plants (reference look). */
  _legSupportDepth(legAngle) {
    const ch = this._chain;
    if (!ch || !ch.length) return 0;
    const ca = Math.cos(legAngle), sa = Math.sin(legAngle);
    let deepest = 0;
    for (let i = 0; i < ch.length; i++) {
      const drop = ch[i].x * sa + ch[i].y * ca; // +down component below the pivot
      if (drop > deepest) deepest = drop;
    }
    return deepest + LEG_LINE_RADIUS;
  }

  /** Support depth that actually carries the body = the DEEPER of the two legs
   * (they are 180° apart, so they alternate carrying it). `theta` is the master
   * spin phase; `tilt` is the body lean (both legs inherit it, exactly as in
   * _syncLegs). Returns the max over the two legs. Used both at body placement and
   * each step to FLOAT the body so the deepest contact just grazes the surface
   * (structural 0 penetration) — the bob falls straight out of this geometry. */
  _supportDepth(theta, tilt) {
    let support = 0;
    const offs = [0, this._legPhaseOffset];
    for (const off of offs) {
      const d = this._legSupportDepth(theta + off + (tilt || 0));
      if (d > support) support = d;
    }
    return support;
  }

  /** EXACT GROUNDED PLACEMENT. Given the cube at forward x with the legs rotated by
   * (theta + tilt), find the cube-centre y so the DEEPEST foot point just GRAZES the
   * surface directly beneath it (gap ≈ 0 — no float, no penetration), on flat OR a
   * slope. We rotate every chain sample of both legs into world (axle-local, +tilt),
   * then for each the surface under its world x is surfaceYAt(axleX+rx). The required
   * cube y is the one that makes  min over samples of (surfaceUnder − (centreY+ry+r))
   * == 0, i.e. centreY = min over samples of (surfaceUnder − ry − r). Returns that
   * centreY (physics +down). Falls back to surfaceY−support if the surface is null.
   * This replaces the vertical-drop `support`+`slopeLift` approximation that floated
   * the foot on slopes (the "floating" the user reported). */
  _groundedCubeY(axleX, theta, tilt) {
    const ch = this._chain;
    if (!ch || !ch.length) {
      const s = this.surfaceYAt(axleX);
      return (s == null) ? this._bodyY : s - (this._reach + LEG_LINE_RADIUS);
    }
    const offs = [0, this._legPhaseOffset];
    let bestY = null; // the LOWEST (most-negative, highest body) centreY that still grazes
    for (const off of offs) {
      const a = theta + off + (tilt || 0);
      const ca = Math.cos(a), sa = Math.sin(a);
      for (let i = 0; i < ch.length; i++) {
        const rx = ch[i].x * ca - ch[i].y * sa;   // world offset from axle (x)
        const ry = ch[i].x * sa + ch[i].y * ca;   // world offset from axle (y, +down)
        const surfUnder = this.surfaceYAt(axleX + rx);
        if (surfUnder == null) continue;
        // centreY that puts THIS sample's bottom (ry + lineRadius) exactly on the
        // surface under it: centreY = surfUnder − ry − lineRadius. The body must sit
        // at the MIN such centreY (highest body) so NO sample dips below its surface.
        const cY = surfUnder - ry - LEG_LINE_RADIUS;
        if (bestY == null || cY < bestY) bestY = cY;
      }
    }
    if (bestY == null) {
      const s = this.surfaceYAt(axleX);
      return (s == null) ? this._bodyY : s - (this._reach + LEG_LINE_RADIUS);
    }
    return bestY;
  }

  /** CREST LAUNCH TEST (called only when GROUNDED). At a CONVEX crest the surface
   * curls DOWN: the downhill slope just AHEAD is steeper than just BEHIND. If the
   * body kept following that surface its downward acceleration would be
   *     a_surf ≈ v² · d(slope)/dx        (curvature × forward-speed²)
   * which is the centripetal demand of riding a convex hump. A foot can only PUSH,
   * not PULL — so once a_surf exceeds gravity the foot leaves the ground and the
   * body flies a parabola (the reference's "speed off a hilltop" launch). We set
   * the launch vertical velocity to the body's CURRENT along-surface vertical speed
   * (v·slopeBehind, physics +down) so the arc tangentially continues the pre-crest
   * motion (smooth departure, no pop). Returns true if it switched to AIRBORNE.
   * Gated by minLaunchSpeed and minDropSlope so the FLAT (slope≈0, curvature 0) and
   * slow walks NEVER launch (no false jumps). */
  _maybeLaunch(v) {
    if (v < TUNE.minLaunchSpeed) return false;        // too slow → stay grounded
    const x = this._x;
    const slopeBehind = this.surfaceSlopeAt(x - TUNE.crestLookDx); // up<0, down>0
    const slopeAhead = this.surfaceSlopeAt(x + TUNE.crestAheadDx);
    const dSlope = slopeAhead - slopeBehind;          // convex crest ⇒ +（curls down）
    if (dSlope <= 0) return false;                     // concave / flat / climbing — no launch
    if (slopeAhead < TUNE.minDropSlope) return false;  // the descent ahead is too gentle
    const dxSpan = TUNE.crestLookDx + TUNE.crestAheadDx;
    const curvature = dSlope / dxSpan;                 // d(slope)/dx
    const aSurf = v * v * curvature;                   // downward accel demanded to stay glued
    if (aSurf <= TUNE.gravity * TUNE.launchMargin) return false; // gravity can hold it → grounded
    // LAUNCH. Vertical velocity = current along-surface vertical speed (tangential).
    // On a hump the body is usually near-flat/rising at the crest, so vy0 is small
    // or slightly upward; gravity then arcs it down onto the descent. Clamp so a
    // freak slope can't fling it absurdly.
    this._air = true;
    this._airFrames = 0;
    this._vy = clampMag(v * slopeBehind, v); // physics +down; rising crest ⇒ negative (up)
    this._landMerge = 0;
    return true;
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
      this._blockedByRiser = false;  // §C: a CLIMB block (vs a gap) ⇒ legs keep trying

      // climb rule: if a stairs/wall step lies just ahead, gate on reach.
      const riser = this._nextRiser(this._x, lookX + 0.5);
      if (riser && !this.canClimb(reach, riser.h)) {
        // blocked: stop just before the riser.
        v = 0;
        this._blocked = true;
        this._blockedByRiser = true; // a step we can't clear — struggle against it
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

      // RIVAL pace scaling: applied to the realized speed so ω = v/r still gives
      // exact no-slip and the foot still grazes (never penetrates) the surface.
      v *= this.paceFactor;

      // 3. advance
      const adv = v * dt;
      const nextX = this._x + adv;
      if (nextX < this.finishX + 1) this._x = nextX;
    }
    this._vx = v;

    // ── ORDER NOTE: the geometric walking bob below reads the legs' CURRENT world
    //    angles (master phase θ + body tilt), so we advance θ and ease the tilt
    //    FIRST, then derive the body height from the resulting contact geometry. ──

    // (a) BODY TILT: ease cube.angle toward the local surface tangent (reference
    //     look — nose up on ascents, down on descents, level on flats / stairs along
    //     their diagonal). Slew-limited so a discrete target jump at a segment seam
    //     (flat→ramp, ramp crest, stair run) can never snap. The legs inherit this
    //     tilt (anchored at the cube centre) so the whole walker leans together.
    {
      let tgt, lerp;
      if (this._air) {
        // AIRBORNE: lean to the FLIGHT-PATH angle (nose follows the arc) — rising at
        // launch (nose up), pitching down through the apex onto the descent. physics
        // y +down, so atan(vy/vx): vy<0 (up) ⇒ nose-up (negative), vy>0 ⇒ nose-down.
        const vxNow = Math.max(1e-3, this._vx);
        tgt = clampMag(Math.atan2(this._vy, vxNow), TUNE.tiltMax);
        lerp = TUNE.airTiltLerp;
      } else {
        tgt = this._targetTilt(this._x);
        lerp = TUNE.tiltLerp;
      }
      const a = 1 - Math.exp(-lerp * dt);
      let step = (tgt - this._angle) * a;
      // keep the per-frame slew cap on the GROUND (anti-snap at seams). In the air the
      // arc pose may need to swing a touch faster; allow ~2× the cap so the nose can
      // pitch over the apex (still bounded — no snap).
      step = clampMag(step, this._air ? TUNE.tiltSlewMax * 2 : TUNE.tiltSlewMax);
      this._angle += step;
    }

    // (b) LEG PHASE advances by ω. There are TWO regimes:
    //
    //   • WALKING (v>0): the planted (deeper) foot must stay STATIONARY on the
    //     ground as the body rocks forward over it (a walk, not a rigid wheel). The
    //     planted foot's WORLD horizontal velocity is vx − ω·ry_contact, where
    //     ry_contact is that foot's CURRENT vertical lever below the pivot. Setting
    //     ω = vSurf / ry_contact makes it ≈ 0 (no-slip, BY CONSTRUCTION) at EVERY
    //     phase — including while the geometric bob rocks the body (the contact
    //     lever shrinks off the vertical plant, so ω speeds up to compensate, just
    //     like a real foot staying put while the hip swings over it). Using the
    //     fixed reach+r (the straight-down lever) instead — as the old roller did —
    //     no longer holds once the bob lets the body ride a TILTED contact, which is
    //     exactly the slip regression we are fixing here.
    //   • BLOCKED but ABLE TO REACH (§C "trying"): v=0 yet the leg is long enough to
    //     poke past the body (reach > body radius). The legs keep churning at the
    //     natural WALKING CADENCE (ω the leg would have on the flat) so the walker
    //     visibly STRUGGLES against the step instead of freezing; the foot just
    //     slips (net x ≈ 0 — NO fake forward push). When the motor is off, ω=0.
    let omega = 0;
    const cosA = Math.max(0.2, Math.cos(this._angle || 0)); // along-surface factor
    if (drive && v > 1e-6) {
      const vSurf = v / cosA;
      // ry_contact = the deeper (carrying) leg's CURRENT vertical lever (== support
      // depth at the live phase+tilt). Floor it so a near-horizontal pose can't blow
      // ω up; this is the same quantity the body floats on (so foot & body agree).
      const ryContact = Math.max(TUNE.effRadiusMin, this._supportDepth(this._theta, this._angle));
      omega = vSurf / ryContact;
      this._theta += omega * dt;
      this._vTip = omega * ryContact; // == v_surface (planted foot stationary)
    } else if (drive && this._blockedByRiser && this._reach > CUBE_SIZE * 0.5) {
      // §C: struggle in place. Spin at the cadence the leg would have if walking on
      // the flat (vNatural/effR) so the churn looks like a real walking effort. The
      // body x does NOT advance (v stays 0) — the foot slips on the riser.
      const vNat = TUNE.baseSpeed * this.legSpeedFactor(this._reach) * this.paceFactor;
      const ryContact = Math.max(TUNE.effRadiusMin, this._supportDepth(this._theta, this._angle));
      omega = vNat / ryContact;
      this._theta += omega * dt;
      this._vTip = 0; // slipping — no net foot progress
      this._trying = true;
    } else {
      this._vTip = 0;
      this._trying = false;
    }
    this._omega = omega; // rad/s — recorded on each leg.body in _syncLegs

    // (c) BODY HEIGHT — TWO STATES: GROUNDED (the foot DEFINITELY touches the
    //     ground, no float) and AIRBORNE (a ballistic arc launched off a crest).
    //
    //     `_footBaseY` is the LEVEL the support foot rests on:
    //       • GROUNDED: the actual surfaceY under the footprint (foot grazes it).
    //       • AIRBORNE: a parabola integrated under gravity (the foot is in the air).
    //     The cube centre is placed at `_footBaseY − support − slopeLift` so the
    //     DEEPEST leg's lowest point sits EXACTLY on `_footBaseY` (grounded ⇒ on the
    //     surface, gap≈0 every frame — no more floating headroom). The bob is then
    //     INTRINSIC: as `support` oscillates with the spinning legs the cube centre
    //     itself rises (vertical plant) and dips (tilted leg) — the real walking bob.
    //     The CAMERA tracks `_bodyBaseY`, a low-passed (smoothed) version so the
    //     screen never jolts while the cube bobs / flies in-frame.
    const reachR = this._reach + LEG_LINE_RADIUS;
    // EXACT grounded cube centre: the y that makes the DEEPEST foot point graze the
    // surface beneath it (flat OR slope) — gap ≈ 0, no float, no penetration. This is
    // the #1 fix (replaces the vertical-drop `support`+`slopeLift` that floated the
    // foot on slopes). The bob is intrinsic: as the legs spin, the grazing y rises on
    // a vertical plant and dips on a tilted leg (a real alternating walking bob).
    const groundedY = (drive || this.legDrawn)
      ? this._groundedCubeY(this._x, this._theta, this._angle)
      : (this._footBaseY - reachR);

    // surface under the body (its own x) — the level the contact foot rests on. Used
    // for the airborne clearance and the camera trend.
    let groundSurf = this.surfaceYAt(this._x);
    if (groundSurf == null) groundSurf = this._footBaseY; // over a gap: keep last level

    this._prevFootBaseY = this._footBaseY;

    if (!this._air) {
      // ── GROUNDED ──
      // CREST DETECTION: if the body keeps following the surface, its downward
      // acceleration would be a_surf ≈ v²·d(slope)/dx (curvature × speed²). At a
      // CONVEX crest (slope drops: flat/up → steep down) a_surf can exceed gravity —
      // then the foot can no longer be held down (no glue) and the body LAUNCHES.
      const launched = this._maybeLaunch(v);
      if (!launched) {
        // ride the surface: foot grazes it EVERY frame (no float). The grazing y is
        // the bob (support oscillation). Track the contact level + its vertical speed.
        this._footBaseY = groundSurf;
        this._vy = (this._footBaseY - this._prevFootBaseY) / Math.max(1e-4, dt);
        this._bodyY = groundedY;
        // soft touchdown re-settle (decays after a landing) — eases any residual.
        if (this._landMerge > 1e-3) this._landMerge *= Math.exp(-TUNE.landMergeLerp * dt);
        else this._landMerge = 0;
      }
    }
    if (this._air) {
      // ── AIRBORNE — integrate the ballistic arc of the CONTACT LEVEL. ──
      this._vy += TUNE.gravity * dt;            // gravity pulls DOWN (physics +down)
      this._footBaseY += this._vy * dt;          // the level the body would land on
      this._airFrames++;
      // LAND when the descending arc meets the surface DIRECTLY UNDER THE BODY (its
      // own x), NOT the lead-window min (which still includes the higher ground we
      // just launched off — that would re-ground us on the launch frame). On a
      // downhill crest the surface ahead drops away below the arc, so footBaseY (the
      // contact level) stays ABOVE it (clearance) until the parabola descends onto
      // the descent: touchdown then. Skip the very first airborne frame so a launch
      // from a flat lip (vy0≈0) can't instantly re-land before the arc lifts clear.
      const landSurf = this.surfaceYAt(this._x);
      const canLand = this._airFrames > 1 && this._vy > 0 && landSurf != null;
      if (canLand && this._footBaseY >= landSurf - 1e-4) {
        // TOUCHDOWN → GROUNDED. Snap the contact level onto the surface and re-settle.
        this._footBaseY = landSurf;
        this._air = false;
        this._vy = 0;
        this._landMerge = 1; // mark a fresh landing (cosmetic ease in the renderer/tilt)
      }
      // cube centre rides the arc: it is the grounded pose RAISED by the clearance of
      // the ballistic contact level above the ground under the body (the legs still
      // spin in the air, so the bob continues — it just isn't touching anything).
      const clearance = groundSurf - this._footBaseY; // ≥0 ⇒ contact level above ground
      this._bodyY = groundedY - Math.max(0, clearance);
    }

    // CAMERA BASE: low-pass the cube height so the SCREEN is smooth (no per-foot
    // bob jolt, no jump snap). The camera follows the GROUND trend — NOT the
    // airborne arc — so while the cube flies in-frame the screen keeps gliding along
    // the terrain (the reference look: the world scrolls smoothly, the cube hops).
    // Target = the grounded vertical-plant height over the ground under the body.
    {
      const baseTargetY = groundSurf - reachR;
      const a = 1 - Math.exp(-TUNE.surfaceLerp * dt);
      let step = (baseTargetY - this._bodyBaseY) * a;
      step = clampMag(step, TUNE.surfaceSlewMax); // gentle slew on the camera → no snap
      this._bodyBaseY += step;
    }
    // BOB (the walking juice, reported for the verifier (J)): the GEOMETRIC dip of the
    // body below a vertical-plant pose = reachR − support. It is HIGH (body dipped)
    // when the carrying leg is TILTED (shallow support) and ≈0 on a vertical plant —
    // a clean function of leg phase, so it correlates with verticality and is free of
    // camera-base lag (which the old cube−base definition picked up at landings).
    const supportNow = (drive || this.legDrawn) ? this._supportDepth(this._theta, this._angle) : reachR;
    this._bob = Math.max(0, reachR - supportNow);

    this.cube.position.x = this._x;
    this.cube.position.y = this._bodyY;
    this.cube.velocity.x = v;
    this.cube.velocity.y = this._air ? this._vy : 0;
    this.cube.angle = this._angle;

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
    // legs inherit the body tilt: world leg angle = spin phase + body lean. The tilt
    // eases slowly (slew-capped) so it adds only a small slow rotation; no-slip is
    // derived from the spin ω = v/r (the dominant term). The foot still sweeps a
    // circle of the same radius about the centre, so the no-penetration math holds.
    const angVelMatter = (this._omega || 0) * (this.SUB_DT / 1000);
    const tilt = this._angle || 0;
    for (const l of this.legs) {
      const angle = this._theta + l.phaseOffset + tilt;
      l.body.position.x = axleX;
      l.body.position.y = axleY;
      l.body.angle = angle;
      l.body.angularVelocity = angVelMatter;
      l.body.parts = this._buildParts(axleX, axleY, angle, l.chain);
    }
    // defensive UP-ONLY clamp against the surface UNDER EACH FOOT POINT (its own x —
    // NOT the surface at the body centre, which is wrong on a slope and would falsely
    // LIFT the body when a leg sample trails over lower ground, re-introducing the
    // "floating on slopes" gap). While AIRBORNE the body is meant to be above the
    // surface (positive clearance), so the clamp is skipped entirely.
    if (this._air) return;
    let below = 0; // worst penetration of any foot point below ITS local surface
    for (const l of this.legs) {
      for (let i = 1; i < l.body.parts.length; i++) {
        const p = l.body.parts[i];
        const su = this.surfaceYAt(p.position.x);
        if (su == null) continue;
        const d = (p.position.y + l.lineRadius) - su; // >0 ⇒ this point is below its surface
        if (d > below) below = d;
      }
    }
    if (below > 1e-6) {
      // up-only correction: lift the body so the deepest penetrating point sits on its
      // surface. _groundedCubeY already makes this ≈0 by construction; this only mops
      // up sub-epsilon round-off (or a one-frame seam transient). Tiny, bounded.
      this.cube.position.y -= below;
      this._bodyY = this.cube.position.y;
      const nAxleY = this.cube.position.y + AXLE_Y;
      for (const m of this.legs) {
        m.body.position.y = nAxleY;
        m.body.parts = this._buildParts(axleX, nAxleY, this._theta + m.phaseOffset + tilt, m.chain);
      }
    }
  }

  // ── getters game.js / renderer.js read ──
  get bodyX() { return this.cube ? this.cube.position.x : this.startX; }
  get bodyY() { return this.cube ? this.cube.position.y : 0; }
  /** BOB-FREE body height (physics +down). This is the low-frequency terrain-follow
   * base WITHOUT the walking bob added — the CAMERA tracks this so the screen stays
   * smooth while the cube visibly bobs IN-FRAME (the bob is cube.position.y − this).
   * Falls back to bodyY before the first step. */
  get bodyCamY() { return this.cube ? (Number.isFinite(this._bodyBaseY) ? this._bodyBaseY : this.cube.position.y) : 0; }
  /** Current geometric bob excursion (render-positive UP amount the cube sits BELOW
   * the bob-free base; ≥ 0). Exposed for the verifier's (J) bob-amplitude report. */
  get bob() { return this._bob || 0; }
  get trying() { return !!this._trying; }
  /** True while the walker is in a ballistic flight (off the ground at a crest). */
  get airborne() { return !!this._air; }
  /** Vertical velocity (physics +down; up = negative) — non-zero only while airborne. */
  get vy() { return this._vy || 0; }
  /** Clearance of the support foot's lowest point ABOVE the surface under the body
   * (world units; >0 ⇒ in the air, ≈0 ⇒ grounded/grazing). For (O)/(P) diagnostics. */
  footClearance() {
    if (!this.legs.length || !this.cube) return null;
    // closest approach of ANY foot point to the surface UNDER THAT point (its own x).
    // >0 ⇒ the foot is above the surface everywhere (clearance, e.g. airborne);
    // ≈0 ⇒ grazing; <0 ⇒ a point is below its local surface (penetration).
    let closest = Infinity;
    for (const l of this.legs) {
      for (let i = 1; i < l.body.parts.length; i++) {
        const p = l.body.parts[i];
        const su = this.surfaceYAt(p.position.x);
        if (su == null) continue;
        const gap = su - (p.position.y + l.lineRadius);
        if (gap < closest) closest = gap;
      }
    }
    return closest === Infinity ? null : closest;
  }
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
function clampMag(v, m) { return v > m ? m : (v < -m ? -m : v); }

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
  // §E: a LONGER demo/verify limb so the drawn leg reads as a real long stride
  // (more pronounced two-leg gait). Farthest sample ≈1.6 (clamps under LEG_REACH_MAX 1.7).
  if (name === 'limb_long') return [{ x: 0.0, y: -0.25 }, { x: 0.12, y: 0.55 }, { x: 0.62, y: 1.05 }, { x: 1.15, y: 0.95 }];
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
