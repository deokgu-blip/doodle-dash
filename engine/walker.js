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

// GAIT-LOFT: above this loft height (world u) the support foot has cleared the
// surface in the current stride, so `airborne` is reported true (the verifier then
// treats these frames as the float phase of a stride — legs still roll). Small so a
// real hop is flagged but the grazing plant phase (loft≈0) stays "grounded".
const LOFT_AIR_EPS = 0.10;

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
  // TERRAIN-FACTOR LOW-PASS: rate k (1/s) at which the applied terrain multiplier eases
  // toward the instantaneous slope target. τ = 1/k ≈ 0.5s. Fast slope reversals (bumps,
  // ~12 sign flips/s) average to ≈1 (a steady pace over a net-flat sine field); a long
  // sustained ramp (slope held > a few hundred ms) still reaches its uphillSlow /
  // downhillFast target, so hills are still slow-up / fast-down. ω uses the smoothed v so
  // no-slip is preserved; only the SPEED is eased, never the height / contact / physics.
  terrainLerp: 2.0,      // 1/s — terrain-factor low-pass rate (τ ≈ 0.5s)
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
  // ── TUNNEL RULE (the INVERSE of the climb/wall rule) ──
  // A LOW CEILING segment blocks a leg whose REACH is too long: the rotating leg
  // (radius ≈ reach about the axle) sweeps UP and strikes the ceiling. So a tunnel
  // passes iff reach <= tunnelMaxReach(clearance). The WALL rule needs reach >= ~Rw
  // (0.30 + wallHeight); we keep wall heights so Rw > the tunnel clearance band, so
  // NO single reach passes BOTH a wall and a tunnel (mutually-exclusive gimmicks —
  // the user MUST redraw long↔short). tunnelClearanceDefault is the fallback ceiling
  // when a segment omits `clearance`. tunnelEnterGap = how far ahead of the ceiling a
  // too-long leg is stopped (so the leg visibly hits the ceiling mouth, struggling).
  tunnelClearanceDefault: 0.95, // max passable reach when a tunnel omits `clearance`
  tunnelEnterGap: 0.30,         // world-u — stop a too-long leg this far before the ceiling start
  tunnelCeilLift: 0.06,         // world-u — render the ceiling this far ABOVE (reach+r) of the max passable leg so a passing leg never punches it
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

  // ── GAIT-LOFT (a RUN, not a forced flight) — spec rewrite ──
  // The walker is ALWAYS running: the legs keep rolling and the body lands once per
  // stride. On a STEEPER downhill (× speed) a single stride naturally (a) reaches
  // farther, (b) lofts a little HIGHER and stays off the ground a little LONGER, and
  // (c) so the legs roll a LITTLE LESS per unit distance (bigger stride = fewer
  // rotations). There is NO discrete "launch off a crest" — the loft is a smooth,
  // gait-phase-locked HOP added on top of the surface-follow body height, whose
  // amplitude/duration grow MONOTONELY with downhill-steepness × speed and is CAPPED
  // so the cube never flies high. The previous discrete forced launch + frozen-leg
  // ballistic state is GONE; "airborne" now just means the current stride's hop has
  // lifted the foot clear of the surface (legs still roll throughout).
  //
  //   loft(θ) = loftAmp(slope,v) · hop(θ)        (added ABOVE the grounded pose)
  // where hop(θ) ∈ [0,1] is a smooth gait pulse that is 0 at each foot-plant (a leg
  // straight down) and peaks BETWEEN plants (the float phase of a bounding run); and
  //   loftAmp(slope,v) = loftMax · clamp01(steepNorm) · clamp01(speedNorm)
  // so flat / uphill / slow ⇒ ~0 loft (grounded walk), steep+fast ⇒ a controlled hop.
  loftMax: 1.20,         // world-u HARD CAP on the per-stride hop height (the old wheel flew 6.35u — this keeps it grounded-feeling)
  loftSlopeRef: 0.85,    // downhill slope (physics +down, downhill>0) at which the steepness factor saturates to 1 (gentler ⇒ proportionally less)
  loftSlopeMin: 0.12,    // below this downhill slope the loft is 0 (gentle dips / flats never hop)
  loftSpeedRef: 6.5,     // forward speed (u/s) at which the speed factor saturates to 1 (slower ⇒ proportionally less; a slow walk barely hops)
  loftSpeedMin: 2.5,     // below this speed the loft is 0 (slow walks stay grounded)
  loftReachRef: 1.3,     // a longer reach (bigger natural stride) lofts a touch more; this reach maps the reach-factor to 1 (shorter ⇒ a little less, but NOT suppressed to 0 — every leg runs)
  // STRIDE STRETCH: during the float (hop>0) part of a stride the legs roll SLOWER so
  // a steep stride covers more ground per rotation (spec #2/#3 "roll a little"). At a
  // foot-plant (hop=0) ω == v/r EXACTLY (no-slip preserved); mid-float ω is divided by
  // (1 + strideStretch·hop). The stretch scales with the SAME steep×speed factor as the
  // loft, so flat ground keeps the normal cadence.
  strideStretch: 1.6,    // max extra rolling-radius factor at full loft (1.6 ⇒ ~2.6× radius mid-float ⇒ ~0.38× cadence there)
  loftLerp: 16.0,        // 1/s — how fast the live loft eases toward its phase target (smooth, no pop at slope onset)
  airTiltLerp: 7.0,      // 1/s — body eases its lean a touch faster during the hovering float (nose follows the gentle arc)
  landMergeLerp: 14.0,   // 1/s — residual touchdown re-settle ease (kept for cosmetic continuity)
  // reach factor floor: even the shortest leg still runs and lofts a little (never 0).
  loftReachFloor: 0.55,  // min reach-factor (so short legs hop less, but are not frozen-grounded)

  // ── PRE-RACE IDLE FLOAT (reference start look) ──
  // Before the race starts (the player has not yet started drawing a leg, OR is drawing
  // the very first leg during the 3-2-1 countdown) the cube does NOT sit on the track.
  // It HOVERS a little ABOVE the surface and bobs gently up/down on a slow sine — a
  // "ready, floating" pose. No forward, no rotation, no penetration. The float is driven
  // by a phase clock (engine-time) so the renderer sees the body rise/fall every frame.
  idleFloatLift: 0.85,   // world-u — how far ABOVE the surface the cube CENTRE's float anchor sits (cube bottom = anchor + CUBE_SIZE/2, always above surface)
  idleBobAmp: 0.22,      // world-u — peak-to-base amplitude of the idle bob (gentle "둥실")
  idleBobHz: 0.55,       // Hz — idle bob frequency (slow, calm)
  idleLerp: 8.0,         // 1/s — how fast the body eases into / out of the float pose (smooth float→ground settle, no snap)
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
    this.ceilingBodies = [];         // TUNNEL low-ceiling render bars ({ x0,x1, ceilingY, floorY, clearance })
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
    this._blockedByTunnel = false;   // blocked specifically by a low ceiling (too-long leg) — legs keep trying
    this._trying = false;            // §C: true while struggling in place (legs churn, x≈0)
    this._vx = 0;                    // last realized forward speed (u/s)
    this._vTip = 0;                  // last foot tip linear speed (u/s)
    this._omega = 0;                 // last leg angular speed (rad/s)
    // ── TERRAIN-FACTOR LOW-PASS (anti-bumps-ripple) ──
    // The instantaneous terrain multiplier (uphillSlow ↔ downhillFast) flips many times a
    // second on `bumps` (each ~½u sub-ramp reverses slope sign), which made v output ±2-4×
    // ripple → the cube lurched forward in stutters on hills (a SPEED ripple, NOT a frame
    // drop). We LOW-PASS the factor with a ~0.5s time-constant: fast slope reversals
    // (bumps) average to ≈1 (a steady pace, since a sine hill is net-flat), while a long
    // SUSTAINED ramp (low frequency) still drives the factor all the way to its target
    // (uphill slow / downhill fast preserved). ω is derived from the SMOOTHED v, so
    // no-slip holds; body height/contact uses the real surfaceY (unsmoothed).
    this._terrainF = 1;              // eased terrain multiplier applied to v

    // ── GAIT-LOFT state (a run, not a forced flight) ──
    this._air = false;               // true while the current stride's hop has lifted the foot clear of the surface (legs STILL roll)
    this._vy = 0;                    // vertical velocity of the body (physics +down; up = negative) — derived from the loft change
    this._footBaseY = 0;             // y of the foot-contact level (== surfaceY under the body; the loft rides ABOVE this)
    this._prevFootBaseY = 0;         // previous frame's foot-contact level (vertical velocity estimate)
    this._airFrames = 0;             // frames spent in the loft float of the current stride (diagnostic)
    this._landMerge = 0;             // residual touchdown re-settle blend (cosmetic continuity)
    this._loft = 0;                  // live per-frame loft height above the grounded pose (world u, eased toward the phase target)
    this._loftAmpLive = 0;           // eased loft AMPLITUDE for the current steep×speed (so stride-stretch & loft share one value)
    this._prevLoft = 0;              // previous frame's loft (for the body vertical velocity)

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
    this._segX0 = null;              // Float64Array of segment x0 (ascending) — for O(log n) lookup
    this._segHint = 0;               // last-resolved segment index (current-segment pointer cache)
    this._maxSurfaceTopY = 0;

    // ── PRE-RACE IDLE FLOAT (reference start look) ──
    // When `_idleFloat` is on, update() ignores the locomotion body-height path and
    // instead HOVERS the cube above the surface with a gentle sine bob (no forward,
    // no spin, no penetration). Turned on by the game while phase ∈ {idle, countdown}
    // and off at GO (then the body eases back to the grounded pose). The phase clock
    // advances with engine dt so the bob is deterministic + refresh-rate independent.
    this._idleFloat = false;
    this._idlePhase = 0;             // rad — idle bob sine phase (engine-time driven)

    // ── PER-FRAME ALLOCATION SCRATCH (GC-pressure fix) ──
    // The steady-running hot path (_supportDepth / _groundedCubeY / _nextRiser /
    // _nextTunnel) used to allocate a fresh 2-element `offs` array and a `{x,...}`
    // result object EVERY call (×1-3 per frame, ×player+rival, ×fixed sub-steps).
    // At 120Hz that is thousands of throwaway objects/s feeding the young-gen GC —
    // a likely source of the random mid-run hitch (a GC sweep landing on a frame).
    // We REUSE these instance scratch objects: the two `offs` loops are now 2-case
    // UNROLLED (no array), and the riser/tunnel scans WRITE INTO a reusable object
    // (callers read the fields the same step ⇒ no aliasing). Byte-identical results.
    this._riserHit = { found: false, x: 0, h: 0 };
    this._tunnelHit = { found: false, x: 0, clearance: 0 };
  }

  reset() {
    this.cube = null;
    this.legs = [];
    this.floorBodies = [];
    this.ceilingBodies = [];
    this.legDrawn = false;
    this._exploded = false;
    this._segs = [];
    this._segX0 = null;
    this._segHint = 0;
    this._blocked = false;
    this._blockedByRiser = false;
    this._blockedByTunnel = false;
    this._trying = false;
    this._vx = 0;
    this._vTip = 0;
    this._terrainF = 1;
    this._theta = 0;
    this._angle = 0;
    this._air = false;
    this._vy = 0;
    this._airFrames = 0;
    this._landMerge = 0;
    this._loft = 0;
    this._loftAmpLive = 0;
    this._prevLoft = 0;
    this._idleFloat = false;
    this._idlePhase = 0;
  }

  /** Pre-race idle float toggle (reference start look). While ON the cube hovers
   * above the surface and bobs gently (no forward, no spin); update() handles it. */
  setIdleFloat(on) {
    this._idleFloat = !!on;
    if (!on) return;
    // entering the float: reset the bob phase so the start pose is the float midline.
    this._idlePhase = 0;
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
        // GAP = a V-TRENCH WITH A FLOOR (no soft-lock). The bottom drops by `depth`
        // below the lip and rises back to the same level over `width`. It is built as
        // TWO ramps (a steep descent into the V, a steep ascent out) so:
        //   • the ENTRY LIP is a convex crest: a fast LONG leg LAUNCHES off it and
        //     glides across the trench, landing on/near the far rim (skips the climb).
        //   • a SHORT/slow leg can't launch ⇒ it walks DOWN the descent into the
        //     bottom, then SLOWLY climbs the ascent out (uphill = slow) — it always
        //     escapes (there is a floor everywhere), it is just much slower across.
        // The two ramps share the existing ramp surface/slope/crest machinery, so
        // there is no special-case physics and no penetration (analytic slope float).
        const w = (seg.width != null ? seg.width : len);
        const depth = seg.depth;
        const halfW = w / 2;
        // descent ramp: drop `depth` over halfW (steep).
        const dx0 = cursorX, dx1 = cursorX + halfW;
        const dTopY0 = surfaceY, dTopY1 = surfaceY + depth; // +down ⇒ deeper
        addRampSlab(dx0, dx1, dTopY0, dTopY1, thick);
        const dSlope = depth / halfW;
        this._segs.push({ x0: dx0, x1: dx1, kind: 'ramp', gap: true, topYa: dTopY0, topYb: dTopY1,
          slope: dSlope, surfFn: (px) => dTopY0 + dSlope * (px - dx0) });
        if (dTopY1 < this._maxSurfaceTopY) this._maxSurfaceTopY = dTopY1;
        // ascent ramp: climb `depth` back over halfW.
        const ax0 = cursorX + halfW, ax1 = cursorX + w;
        const aTopY0 = surfaceY + depth, aTopY1 = surfaceY;
        addRampSlab(ax0, ax1, aTopY0, aTopY1, thick);
        const aSlope = -depth / halfW;
        this._segs.push({ x0: ax0, x1: ax1, kind: 'ramp', gap: true, topYa: aTopY0, topYb: aTopY1,
          slope: aSlope, surfFn: (px) => aTopY0 + aSlope * (px - ax0) });
        cursorX += w;
        // surfaceY returns to its original level (the V is symmetric).
      } else if (seg.kind === 'wall') {
        const h = seg.height ?? 1;
        // WALL = a tall STEP-UP to a higher plateau, gated by leg length. The riser is
        // a SHORT, STEEP (but still walkable, analytic) ascent ramp carrying stepH=h so
        // the climb rule applies: a SHORT leg is BLOCKED at its base (can't climb h), a
        // LONG leg climbs it and continues on the raised plateau. We keep kind:'wall' so
        // _nextRiser/canClimb gate it exactly like a stairs riser (struggle-in-place for
        // short legs), and the surface stays raised by h afterwards (a real wall/ledge).
        const riseLen = Math.min(1.2, Math.max(0.6, h * 0.5)); // short, steep face
        const wx0 = cursorX, wx1 = cursorX + riseLen;
        const wTopY0 = surfaceY, wTopY1 = surfaceY - h; // up = negative y
        addRampSlab(wx0, wx1, wTopY0, wTopY1, thick);
        const wSlope = -h / riseLen;
        this._segs.push({ x0: wx0, x1: wx1, kind: 'wall', stepH: h, topYa: wTopY0, topYb: wTopY1,
          slope: wSlope, surfFn: (px) => wTopY0 + wSlope * (px - wx0) });
        if (wTopY1 < this._maxSurfaceTopY) this._maxSurfaceTopY = wTopY1;
        surfaceY -= h;
        cursorX += riseLen;
      } else if (seg.kind === 'bumps') {
        // BUMPS = a continuous wavy surface: a sine of half-amplitude `amp` over `freq`
        // full periods across `length`. We sample it into many short LINEAR sub-ramps
        // so the existing ramp slope/tilt/crest machinery rides it with NO penetration
        // (each sub-segment is analytic) and the body TILT tracks each rise/fall. The
        // surface returns to the entry level at the end (whole sine periods), so the
        // track stays continuous. Bumps are gentle enough not to launch on their own
        // (small amp) — they read as rolling terrain the body bobs/tilts over.
        const amp = seg.amp, freq = seg.freq;
        const nSub = Math.max(8, Math.ceil(freq * 8)); // ~8 samples per period
        const baseY = surfaceY;
        const wave = (t) => baseY - amp * Math.sin(2 * Math.PI * freq * t); // up = -amp at sin>0
        for (let k = 0; k < nSub; k++) {
          const t0 = k / nSub, t1 = (k + 1) / nSub;
          const sx0 = cursorX + len * t0, sx1 = cursorX + len * t1;
          const y0 = wave(t0), y1 = wave(t1);
          addRampSlab(sx0, sx1, y0, y1, thick);
          const sl = (y1 - y0) / (sx1 - sx0);
          this._segs.push({ x0: sx0, x1: sx1, kind: 'bumps', topYa: y0, topYb: y1,
            slope: sl, surfFn: (px) => y0 + sl * (px - sx0) });
          if (y0 < this._maxSurfaceTopY) this._maxSurfaceTopY = y0;
          if (y1 < this._maxSurfaceTopY) this._maxSurfaceTopY = y1;
        }
        cursorX += len;
        // surfaceY unchanged (whole periods ⇒ ends at baseY).
      } else if (seg.kind === 'tunnel') {
        // TUNNEL = a flat FLOOR stretch with a LOW CEILING above it (the inverse of a
        // WALL). The floor is a normal flat surface (the cube walks it). The ceiling is
        // a low bar gated by leg reach: a leg passes iff reach <= clearance. A too-long
        // leg's rotating sweep (radius ≈ reach about the axle) would strike the ceiling,
        // so the walker is BLOCKED at the tunnel MOUTH (struggle-in-place, no advance) —
        // the user must redraw a SHORTER leg. A short leg passes under with headroom.
        const clearance = (seg.clearance != null) ? seg.clearance : SEGMENT_DEFAULTS.tunnelClearance;
        const tx0 = cursorX, tx1 = cursorX + len;
        // CAPTURE the floor level in a per-segment CONST. `surfaceY` is a mutating loop
        // variable; a closure over it would read the FINAL surface (after later ramps),
        // so the tunnel floor would wrongly sample the end-of-track height (same closure
        // bug the stairs `treadY` const fixes). Freeze it here.
        const floorY = surfaceY;
        // floor render slab (same look as a flat).
        addSlab(tx0 + len / 2, floorY, len, thick);
        // CEILING world-y (physics +down ⇒ smaller y is higher). The leg pivots about
        // the cube CENTRE (axle) and sweeps a FULL circle of radius ≈ reach+lineRadius,
        // so its TOPMOST point rises reach+r ABOVE the axle. The axle itself floats
        // ≈ reach+r above the floor (vertical-plant support). So a leg of reach R reaches
        // ≈ 2·(R+r) above the floor at the top of its sweep. We anchor the ceiling just
        // ABOVE the LONGEST passable leg (reach==clearance): 2·(clearance+r) + a small
        // lift, so that leg clears it with headroom and ANY longer leg strikes it. Render
        // + gate share this ceilingY (WYSIWYG: the bar you see is the bar you hit).
        const ceilGapAboveFloor = 2 * (clearance + LEG_LINE_RADIUS) + TUNE.tunnelCeilLift;
        const ceilingY = floorY - ceilGapAboveFloor; // up = negative y
        this._segs.push({ x0: tx0, x1: tx1, kind: 'tunnel', topYa: floorY, topYb: floorY,
          clearance, ceilingY, surfFn: () => floorY });
        if (floorY < this._maxSurfaceTopY) this._maxSurfaceTopY = floorY;
        // record the ceiling as a render body (a low bar the renderer draws as an obstacle).
        this.ceilingBodies.push({ x0: tx0, x1: tx1, ceilingY, floorY, clearance });
        cursorX += len;
        // surfaceY unchanged (flat floor through the tunnel).
      }
    }

    this.surfaceY = surfaceY;

    // ── BUILD THE O(log n) LOOKUP INDEX. ──
    // _segs is appended in cursor (x) order, so x0 is non-decreasing with NO overlaps
    // between consecutive segments (verified for every track kind). We cache the x0
    // boundaries so surfaceYAt/_segAt/surfaceSlopeAt can binary-search to the covering
    // segment instead of linear-scanning all of _segs every call. The "highest surface
    // wins" semantics are preserved exactly by checking the matched segment plus its
    // immediate seam neighbours (the only place two segments can share an x). Results
    // are byte-identical to the old full scan; only the iteration count drops.
    const xs = new Float64Array(this._segs.length);
    for (let i = 0; i < this._segs.length; i++) xs[i] = this._segs[i].x0;
    this._segX0 = xs;
    this._segHint = 0;

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
    this._loft = 0;
    this._loftAmpLive = 0;
    this._prevLoft = 0;
    const startSurf0 = this.surfaceYAt(track.startX);
    this._footBaseY = (startSurf0 == null) ? groundY : startSurf0;
    this._prevFootBaseY = this._footBaseY;
    // a rebuild clears any prior idle-float (the game re-arms it via setIdleFloat for
    // the interactive start; headless/forceStart paths build + start immediately).
    this._idleFloat = false;
    this._idlePhase = 0;
  }

  /**
   * Find the index of the LAST segment whose x0 <= px, using a hint (current-segment
   * pointer) and binary search on the ascending _segX0. Returns -1 if px is before the
   * first segment. The covering segment for px is at this index OR a neighbour that
   * shares a seam (px == prev.x1 == this.x0); callers check the small neighbour window.
   * This replaces the full O(n) linear scan of _segs in surfaceYAt/_segAt with O(log n)
   * (O(1) on the common monotone-forward path via the hint). */
  _segIdxAt(px) {
    const xs = this._segX0;
    if (!xs || xs.length === 0) return -1;
    const n = xs.length;
    // hint fast-path: the body advances monotonically, so the covering segment is
    // usually the hinted one or its neighbour (O(1)).
    let h = this._segHint;
    if (h < 0) h = 0; else if (h >= n) h = n - 1;
    if (xs[h] <= px && (h + 1 >= n || xs[h + 1] > px)) return h;
    if (h + 1 < n && xs[h + 1] <= px && (h + 2 >= n || xs[h + 2] > px)) { this._segHint = h + 1; return h + 1; }
    if (h > 0 && xs[h - 1] <= px && xs[h] > px) { this._segHint = h - 1; return h - 1; }
    // binary search: rightmost index with xs[i] <= px.
    let lo = 0, hi = n - 1, res = -1;
    while (lo <= hi) {
      const mid = (lo + hi) >> 1;
      if (xs[mid] <= px) { res = mid; lo = mid + 1; } else hi = mid - 1;
    }
    if (res >= 0) this._segHint = res;
    return res;
  }

  /**
   * Expected track surface y (physics, +down) at any x. Returns the HIGHEST (most
   * negative) surface covering px, or null over a gap. O(log n) via _segIdxAt; the
   * "highest covering surface" semantics are preserved by also testing the seam
   * neighbours of the matched segment (the only place two segments can share an x).
   */
  surfaceYAt(px) {
    const segs = this._segs;
    const i = this._segIdxAt(px);
    if (i < 0) return null;
    let best = null;
    // check the matched segment and one neighbour on each side (seam coverage). With no
    // overlaps elsewhere, this window contains every segment that can cover px.
    for (let j = i - 1; j <= i + 1; j++) {
      if (j < 0 || j >= segs.length) continue;
      const s = segs[j];
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
    // ramp / wall (steep riser face) / bumps (wavy surface) all carry an analytic slope.
    if (s && (s.kind === 'ramp' || s.kind === 'wall' || s.kind === 'bumps') && typeof s.slope === 'number') return s.slope; // downhill > 0 (y +down)
    if (s && (s.kind === 'flat' || s.kind === 'stairs')) return 0;
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
    // ramp / bumps (wavy surface): lean to the analytic local slope. The body tilt
    // tracks each rise/fall of the bump field (nose up the front of a bump, down its
    // back), capped by tiltMax. A wall's steep riser face also leans (capped) so the
    // body climbs the ledge nose-up rather than staying flat against a vertical face.
    if (seg && (seg.kind === 'ramp' || seg.kind === 'bumps' || seg.kind === 'wall') && typeof seg.slope === 'number') {
      return clampMag(Math.atan(seg.slope) * TUNE.tiltGain, TUNE.tiltMax);
    }
    if (seg && seg.kind === 'flat') return 0;
    // gap / unknown: finite-difference (and hold the current lean over a gap).
    const dx = TUNE.tiltDx;
    const yR = this.surfaceYAt(px + dx);
    const yL = this.surfaceYAt(px - dx);
    if (yR == null || yL == null) return this._angle;   // over a gap — hold current lean
    const slope = (yR - yL) / (2 * dx);                 // physics slope (+down)
    return clampMag(Math.atan(slope) * TUNE.tiltGain, TUNE.tiltMax);
  }

  /** TUNNEL ceiling y (physics +down) at px, or null if no ceiling there. A passing
   * (short) leg's topmost point must stay BELOW (>=) this (no ceiling penetration). */
  ceilingYAt(px) {
    for (const s of this._segs) {
      if (s.kind !== 'tunnel') continue;
      if (px < s.x0 || px > s.x1) continue;
      return s.ceilingY;
    }
    return null;
  }

  /** Smallest clearance of any leg point BELOW the tunnel ceiling above it (world u).
   * >0 ⇒ the leg stays under the ceiling (clearance), <0 ⇒ a point punched through the
   * ceiling (penetration). null if not under any ceiling. For the TUNNEL no-penetration
   * verification (a passing short leg must never strike the ceiling). */
  ceilingClearance() {
    if (!this.legs.length || !this.cube) return null;
    let closest = Infinity;
    for (const l of this.legs) {
      for (let i = 1; i < l.body.parts.length; i++) {
        const p = l.body.parts[i];
        const cy = this.ceilingYAt(p.position.x);
        if (cy == null) continue;
        // ceiling is ABOVE (smaller y); the leg point's TOP is p.y − lineRadius. clearance
        // below the ceiling = (top of leg point) − ceilingY  (>0 ⇒ leg is below ceiling).
        const gap = (p.position.y - l.lineRadius) - cy;
        if (gap < closest) closest = gap;
      }
    }
    return closest === Infinity ? null : closest;
  }

  /** Local segment under px (for terrain / climb decisions). O(log n) via _segIdxAt;
   * preserves the "prefer the highest surface" semantics by testing the matched
   * segment plus its seam neighbours (the only place two segments can share an x). */
  _segAt(px) {
    const segs = this._segs;
    const i = this._segIdxAt(px);
    if (i < 0) return null;
    let found = null, foundY = null;
    for (let j = i - 1; j <= i + 1; j++) {
      if (j < 0 || j >= segs.length) continue;
      const s = segs[j];
      if (px < s.x0 || px > s.x1) continue;
      const y = s.surfFn(px);
      if (y == null) { if (!found) found = s; continue; }
      if (!found || foundY == null || y < foundY) { found = s; foundY = y; }
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
      this._terrainF = 1;
      this._air = false;
      this._vy = 0;
      this._airFrames = 0;
      this._landMerge = 0;
      this._loft = 0;
      this._loftAmpLive = 0;
      this._footBaseY = surf;
      this._prevFootBaseY = surf;
    } else if (this._air) {
      // CONTINUE while in the LOFT float of a stride: a mid-stride redraw must NOT
      // teleport the body to the ground (that would be a fake landing snap). The loft
      // is recomputed every step from phase × slope × v, so we just PRESERVE x / θ /
      // tilt / the live loft; the next update() keeps the run continuous.
      this.cube.velocity.x = 0; this.cube.velocity.y = 0;
      this.cube.angle = this._angle;
      this._blocked = false;
      this._vx = 0;
      this._vTip = 0;
      // _x, _theta, _angle, _air, _loft, _footBaseY, _bodyY, _bodyBaseY all PRESERVED.
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
   * samples. The verifier's no-slip / foot scan reads parts[1..].position.
   *
   * PERF (GC churn): _syncLegs runs every fixed sub-step (×2 legs, ×player+rival,
   * and AGAIN inside the up-only surface clamp), and the old version allocated a
   * fresh array + (chain.length+1) `{position:{x,y}}` objects EACH call — at 120Hz
   * that is thousands of throwaway objects per second feeding the GC. The leg bodies
   * are PLAIN procedural proxies (NOT real Matter bodies in the world), and `parts`
   * is only READ (position.x/y) — never structurally retained — so we REUSE the
   * existing parts array + its position objects IN-PLACE when the chain length is
   * unchanged (it only changes on a redraw, which rebuilds the leg). Same numeric
   * values, zero per-tick allocation ⇒ behaviour-identical, GC-quiet. A `target`
   * (the leg's current body.parts) is reused if its shape matches; otherwise a fresh
   * array is built (first placement / after a redraw). */
  _buildParts(axleX, axleY, angle, chain, target) {
    const ca = Math.cos(angle), sa = Math.sin(angle);
    const n = chain.length + 1; // proxy + one per chain sample
    let parts = target;
    if (!parts || parts.length !== n) {
      // fresh build (first time, or chain length changed after a redraw).
      parts = new Array(n);
      parts[0] = { position: { x: axleX, y: axleY } };
      for (let i = 1; i < n; i++) parts[i] = { position: { x: 0, y: 0 } };
    } else {
      parts[0].position.x = axleX; parts[0].position.y = axleY; // proxy
    }
    for (let i = 0; i < chain.length; i++) {
      const c = chain[i];
      const p = parts[i + 1].position;
      p.x = axleX + c.x * ca - c.y * sa;
      p.y = axleY + c.x * sa + c.y * ca;
    }
    return parts;
  }

  // ── DESIGNED speed / climb mappings ──

  /** Monotone length→speed: longer leg ⇒ faster (clamped). */
  legSpeedFactor(reach) {
    const f = Math.pow(reach / TUNE.refReach, TUNE.speedReachPow);
    return Math.max(TUNE.speedFactorMin, Math.min(TUNE.speedFactorMax, f));
  }

  /** Normalized reach t ∈ [0,1] over the legal reach band (MIN..MAX). */
  reachNorm(reach) {
    const t = (reach - LEG_REACH_MIN) / (LEG_REACH_MAX - LEG_REACH_MIN);
    return Math.max(0, Math.min(1, t));
  }

  /** Monotone reach factor jf ∈ [0,1] from reach (spec #2): a LONG leg (reach→MAX)
   * gives jf→1 (bigger natural stride ⇒ lofts a touch more/farther), a SHORT leg
   * (reach→MIN) gives jf→0 (lofts less). Kept as a diagnostic the verifier reports;
   * the actual loft amplitude uses reachF in _loftAmp (floored so short legs still run). */
  jumpFactor(reach) {
    return this.reachNorm(reach);
  }

  /** Max step height a given reach can climb (designed rule). */
  maxClimbHeight(reach) {
    return Math.max(0, (reach - TUNE.climbBase) / TUNE.climbK);
  }

  /** Is a step of height h climbable by `reach`? */
  canClimb(reach, h) {
    return reach >= TUNE.climbBase + TUNE.climbK * h;
  }

  /** TUNNEL gate (the INVERSE of canClimb): a low ceiling of `clearance` passes a leg
   * iff its REACH is short enough (the rotating leg of radius ≈ reach would otherwise
   * strike the ceiling). reach <= clearance ⇒ pass; longer ⇒ BLOCKED at the mouth. */
  canPassTunnel(reach, clearance) {
    const c = (clearance != null) ? clearance : TUNE.tunnelClearanceDefault;
    return reach <= c;
  }

  /** Find the next TUNNEL whose mouth (x0) lies in (fromX, toX]. Returns { x, clearance }
   * or null. The mouth is where a too-long leg is stopped (struggle-in-place). */
  _nextTunnel(fromX, toX) {
    // REUSE a scratch result object (no per-frame {x,clearance} allocation). The
    // caller (update) reads .x/.clearance THIS step before any other _nextTunnel
    // call, so a single reused object is safe. `found` distinguishes "no tunnel" from
    // a stale scratch. Byte-identical fields to the old freshly-allocated object.
    let bestX = Infinity, bestClr = 0, any = false;
    for (const s of this._segs) {
      if (s.kind !== 'tunnel') continue;
      if (s.x0 > fromX && s.x0 <= toX) {
        if (!any || s.x0 < bestX) { bestX = s.x0; bestClr = s.clearance; any = true; }
      }
    }
    if (!any) { this._tunnelHit.found = false; return null; }
    const h = this._tunnelHit;
    h.found = true; h.x = bestX; h.clearance = bestClr;
    return h;
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
    // 2-CASE UNROLL (no `offs` array allocation — the two legs are 180° apart, i.e.
    // phase offsets 0 and _legPhaseOffset). Byte-identical to the old max-over-[0,off].
    const t = theta + (tilt || 0);
    const d0 = this._legSupportDepth(t);
    const d1 = this._legSupportDepth(t + this._legPhaseOffset);
    return d0 > d1 ? d0 : d1;
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
    // 2-CASE UNROLL (no `offs` array): scan the chain at BOTH leg angles (offsets 0
    // and _legPhaseOffset), tracking the LOWEST (most-negative, highest body) centreY
    // that still grazes. Byte-identical to the old min-over-[0,off]×chain.
    const t = theta + (tilt || 0);
    const best0 = this._legGroundedCubeY(axleX, t, ch);
    const best1 = this._legGroundedCubeY(axleX, t + this._legPhaseOffset, ch);
    let bestY;
    if (best0 == null) bestY = best1;
    else if (best1 == null) bestY = best0;
    else bestY = best0 < best1 ? best0 : best1;
    if (bestY == null) {
      const s = this.surfaceYAt(axleX);
      return (s == null) ? this._bodyY : s - (this._reach + LEG_LINE_RADIUS);
    }
    return bestY;
  }

  /** ONE leg's grounded-cube-Y candidate: the MIN centreY over its chain samples
   * (rotated by world angle `a` about the axle) that still grazes the surface under
   * each sample. Returns null if no sample is over a surface. Split out of
   * _groundedCubeY so the two legs are scanned WITHOUT allocating an `offs` array. */
  _legGroundedCubeY(axleX, a, ch) {
    const ca = Math.cos(a), sa = Math.sin(a);
    let bestY = null;
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
    return bestY;
  }

  /** GAIT-LOFT amplitude (world-u peak hop height) for the CURRENT local downhill
   * steepness × forward speed. This replaces the discrete crest LAUNCH: there is no
   * trigger and no ballistic state — the body just hops a LITTLE more, the steeper &
   * faster the descent. Returns 0 on flat / uphill / slow ground (a grounded walk),
   * rising MONOTONELY toward loftMax (a hard cap so the cube never flies high).
   *   amp = loftMax · steepF · speedF · reachF
   * steepF: downhill slope mapped 0 (≤loftSlopeMin) → 1 (≥loftSlopeRef).
   * speedF: forward v mapped 0 (≤loftSpeedMin) → 1 (≥loftSpeedRef).
   * reachF: a longer reach (bigger natural stride) lofts a touch more, floored so a
   *         short leg still runs (never frozen-grounded). */
  _loftAmp(v) {
    // local downhill slope just AHEAD (the descent the body is running into). Using a
    // small forward look makes the loft RAMP UP smoothly as the cube enters a steep
    // pitch (no per-frame pop), and decay as it flattens out.
    const slopeHere = this.surfaceSlopeAt(this._x);
    const slopeAhead = this.surfaceSlopeAt(this._x + 0.6);
    const slope = Math.max(slopeHere, slopeAhead); // physics +down: downhill > 0
    if (slope <= TUNE.loftSlopeMin) return 0;
    const steepF = clamp01((slope - TUNE.loftSlopeMin) / (TUNE.loftSlopeRef - TUNE.loftSlopeMin));
    if (v <= TUNE.loftSpeedMin) return 0;
    const speedF = clamp01((v - TUNE.loftSpeedMin) / (TUNE.loftSpeedRef - TUNE.loftSpeedMin));
    const reachF = Math.max(TUNE.loftReachFloor, Math.min(1, this._reach / TUNE.loftReachRef));
    return TUNE.loftMax * steepF * speedF * reachF;
  }

  /** GAIT HOP PULSE ∈ [0,1] at the master phase θ. With the two legs 180° apart, a
   * foot plants twice per π of θ (each leg points ~straight-down once per half-turn).
   * The hop is 0 AT a plant (body on the ground, foot in contact) and peaks BETWEEN
   * plants (the float phase). cos(2θ) is +1 at the plants (θ=0,π…) and −1 between, so
   * 0.5·(1−cos(2θ)) ∈ [0,1] is a smooth pulse that is 0 at every plant and 1 mid-air.
   * Smooth ⇒ continuous loft, no pop; the body lands once per stride (a real run). */
  _hopPulse(theta) {
    return 0.5 * (1 - Math.cos(2 * theta));
  }

  // ── MAIN STEP ──
  update(dtMs, running) {
    if (!this.cube) return;
    const dt = dtMs / 1000; // seconds

    // ── PRE-RACE IDLE FLOAT (reference start look) ──
    // Before the race begins the cube does NOT rest on the track: it HOVERS above the
    // surface and bobs gently on a slow sine. No forward motion, no leg spin, no
    // penetration. We compute a float-anchor a fixed lift ABOVE the surface under the
    // body, add a sine bob, and EASE the body toward it (so the very first frame and the
    // GO transition are smooth — no snap). The legs (if any are drawn during the
    // countdown) ride with the body but DO NOT roll (θ frozen) — the run starts at GO.
    if (this._idleFloat) {
      this._idlePhase += 2 * Math.PI * TUNE.idleBobHz * dt;
      // ORIENT THE LEGS HORIZONTAL while floating: rotate the leg phase so the drawn
      // chain's FARTHEST point points sideways (≈0°), not down — so neither leg tip dips
      // through the track under the hovering body. (The two legs are 180° apart, so one
      // points fwd-horizontal and the other back-horizontal: a tidy "arms out" ready
      // pose.) The run's spin starts at GO (θ then advances from this idle orientation).
      if (this._chain && this._chain.length) {
        let fx = 0, fy = 0, fd = -1;
        for (const c of this._chain) { const d = c.x * c.x + c.y * c.y; if (d > fd) { fd = d; fx = c.x; fy = c.y; } }
        this._theta = -Math.atan2(fy, fx); // farthest sample → angle 0 (horizontal fwd)
      } else {
        this._theta = 0;
      }
      const surf0 = this.surfaceYAt(this._x);
      const surf = (surf0 == null) ? this._footBaseY : surf0;
      // physics +down: subtract to float ABOVE the surface; bob oscillates around it.
      const bob = TUNE.idleBobAmp * Math.sin(this._idlePhase);
      const targetY = surf - TUNE.idleFloatLift - bob;
      const a = 1 - Math.exp(-TUNE.idleLerp * dt);
      this._bodyY += (targetY - this._bodyY) * a;
      // camera base eases to the float MIDLINE (bob-free) so the screen sits still while
      // the cube bobs in-frame.
      const baseTarget = surf - TUNE.idleFloatLift;
      this._bodyBaseY += (baseTarget - this._bodyBaseY) * a;
      this._angle = 0;
      this._bob = Math.abs(bob);
      this._vx = 0; this._vTip = 0; this._omega = 0; this._vy = 0;
      this._air = false; this._airFrames = 0; this._loft = 0;
      this._footBaseY = surf; this._prevFootBaseY = surf;
      this.cube.position.x = this._x;
      this.cube.position.y = this._bodyY;
      this.cube.velocity.x = 0; this.cube.velocity.y = 0;
      this.cube.angle = 0;
      // keep the (frozen) legs glued under the floating body so they hover too.
      this._syncLegs();
      return;
    }

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
      this._blockedByTunnel = false; // a LOW-CEILING block (too-long leg) ⇒ legs keep trying

      // climb rule: if a stairs/wall step lies just ahead, gate on reach (LONG leg needed).
      const riser = this._nextRiser(this._x, lookX + 0.5);
      // tunnel rule (INVERSE): if a low-ceiling mouth lies just ahead, gate on reach
      // (SHORT leg needed). A too-long leg's rotating sweep strikes the ceiling, so we
      // stop it at the mouth (struggle-in-place), the user must redraw shorter.
      const tunnel = this._nextTunnel(this._x, lookX + 0.5);
      const tunnelBlocks = tunnel && !this.canPassTunnel(reach, tunnel.clearance);
      // whichever gate's stop-point comes FIRST along x wins (so a wall just before a
      // tunnel, or vice-versa, blocks at the nearer obstacle).
      const riserStopX = (riser && !this.canClimb(reach, riser.h)) ? (riser.x - CUBE_SIZE * 0.5) : Infinity;
      const tunnelStopX = tunnelBlocks ? (tunnel.x - CUBE_SIZE * 0.5 - TUNE.tunnelEnterGap) : Infinity;
      if (tunnelBlocks && tunnelStopX <= riserStopX) {
        // BLOCKED by the low ceiling: stop just before the tunnel mouth. The leg is too
        // long — it churns in place (struggle) and makes NO net forward progress until a
        // SHORTER leg is drawn (canPassTunnel). No artificial advance, not a soft-lock.
        v = 0;
        this._blocked = true;
        this._blockedByTunnel = true;
        if (this._x > tunnelStopX) this._x = tunnelStopX;
      } else if (riser && !this.canClimb(reach, riser.h)) {
        // blocked: stop just before the riser.
        v = 0;
        this._blocked = true;
        this._blockedByRiser = true; // a step we can't clear — struggle against it
        if (this._x > riser.x - CUBE_SIZE * 0.5) {
          this._x = riser.x - CUBE_SIZE * 0.5;
        }
      } else if (seg) {
        if (seg.kind === 'ramp' || seg.kind === 'bumps') {
          // slope > 0 means descending (topY increases downhill since y is +down…
          // careful: y +down so going UP is slope<0). Use sign of slope. The GAP's
          // descent/ascent ramps and the BUMPS sub-ramps ride this same blend, so a
          // short leg crawls UP the V-trench exit / bump fronts (uphill = slow) while
          // a long leg either launches over (gap) or rolls across faster.
          terrain = seg.slope < 0 ? lerpSlow(seg.slope, TUNE.uphillSlow) : lerpFast(seg.slope, TUNE.downhillFast);
        } else if (seg.kind === 'wall') {
          // climbing the steep wall face (a climbable long leg) — slow, like stairs.
          terrain = TUNE.stairClimbSlow;
        } else if (seg.kind === 'stairs' || (riser && this.canClimb(reach, riser.h))) {
          terrain = TUNE.stairClimbSlow;
        }
      }
      // LOW-PASS the terrain multiplier (anti-bumps-ripple). `terrain` is the INSTANTANEOUS
      // slope target; we ease `_terrainF` toward it with a ~0.5s time-constant so the fast
      // sign-flips of a `bumps` field (each sub-ramp reverses uphill↔downhill ~12×/s)
      // average to ≈1 (a steady pace — a sine hill is net-flat), while a SUSTAINED ramp
      // (slope held) still pulls `_terrainF` to its uphillSlow / downhillFast target. We
      // apply the SMOOTHED factor, never the raw one. ω below is derived from this realized
      // v, so no-slip is preserved; the body height / contact use the real (unsmoothed)
      // surfaceY, so no-penetration / grounding are untouched. (On a blocked frame v was
      // set to 0 above; 0·_terrainF stays 0, and `terrain`=1 there eases _terrainF back to
      // neutral so the resume pace is correct.)
      {
        const a = 1 - Math.exp(-TUNE.terrainLerp * dt);
        this._terrainF += (terrain - this._terrainF) * a;
      }
      v *= this._terrainF;

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
      // The body keeps leaning to the SURFACE tangent (nose-down on a descent) even
      // during the per-stride loft float — a bounding run keeps its down-slope posture,
      // it does NOT pitch around the small hop arc (that read as the old fake flight).
      // A touch faster ease during a loft so the nose tracks the descent it is hopping
      // down — but the per-frame slew stays under the (I) snap cap in EVERY regime (no
      // pop at the loft↔ground transitions).
      const tgt = this._targetTilt(this._x);
      const lerp = this._air ? TUNE.airTiltLerp : TUNE.tiltLerp;
      const a = 1 - Math.exp(-lerp * dt);
      let step = (tgt - this._angle) * a;
      step = clampMag(step, TUNE.tiltSlewMax);
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
    // GAIT-LOFT amplitude for the current downhill steepness × speed (0 on flat/slow),
    // eased so it ramps in/out smoothly (no pop at a slope onset). ONE value drives
    // BOTH the body loft (height) and the stride-stretch (cadence) so they stay in sync.
    {
      const ampTarget = drive ? this._loftAmp(v) : 0;
      const a = 1 - Math.exp(-TUNE.loftLerp * dt);
      this._loftAmpLive += (ampTarget - this._loftAmpLive) * a;
      if (this._loftAmpLive < 1e-4) this._loftAmpLive = 0;
    }
    // hop pulse ∈ [0,1] at the live phase: 0 at a foot-plant, 1 mid-float.
    const hop = this._hopPulse(this._theta);
    // normalized loft intensity ∈ [0,1] (amp scaled by the cap) — drives stride-stretch.
    const loftIntensity = clamp01(this._loftAmpLive / Math.max(1e-4, TUNE.loftMax));
    if (drive && v > 1e-6) {
      const vSurf = v / cosA;
      // ry_contact = the deeper (carrying) leg's CURRENT vertical lever (== support
      // depth at the live phase+tilt). Floor it so a near-horizontal pose can't blow
      // ω up; this is the same quantity the body floats on (so foot & body agree).
      const ryContact = Math.max(TUNE.effRadiusMin, this._supportDepth(this._theta, this._angle));
      // STRIDE STRETCH: during the float (hop>0) part of a STEEP stride the legs roll
      // SLOWER (the rolling radius is inflated) so the stride covers more ground per
      // rotation — "roll a little" / bigger boards. At a foot-plant (hop=0) the factor
      // is 1, so ω == v/r EXACTLY → no-slip is preserved on contact (the only phase the
      // foot is actually on the ground). Mid-float the foot is clear of the surface, so
      // the slower roll never slips. The stretch scales with the loft intensity so flat
      // ground keeps the normal cadence everywhere.
      const stretch = 1 + TUNE.strideStretch * loftIntensity * hop;
      omega = vSurf / (ryContact * stretch);
      this._theta += omega * dt;
      this._vTip = omega * ryContact; // == v_surface at a plant (planted foot stationary)
      this._trying = false;
    } else if (drive && (this._blockedByRiser || this._blockedByTunnel) && this._reach > CUBE_SIZE * 0.5) {
      // §C: struggle in place (riser climb OR low-ceiling tunnel). Spin at the cadence
      // the leg would have if walking on the flat (vNatural/effR) so the churn looks like
      // a real walking effort. The body x does NOT advance (v stays 0) — the foot slips
      // against the wall (riser) or the leg keeps spinning into the low ceiling (tunnel).
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

    // ── GAIT-LOFT BODY HEIGHT (a run, not a forced flight) ──
    // The contact level is ALWAYS the surface under the body (the foot lands once per
    // stride — there is no ballistic departure). On top of the grounded pose we add a
    // smooth, gait-phase-locked HOP: loft = ampLive · hop(θ). It is 0 at each foot-plant
    // (body grazes the surface, foot in contact) and lifts BETWEEN plants, the steeper &
    // faster the descent (ampLive), capped at loftMax. So the cube runs along the ground
    // and floats a LITTLE in each stride — the float grows with the slope, never flies.
    this._footBaseY = groundSurf;
    const loftTargetH = this._loftAmpLive * hop; // world-u above the grounded pose
    // ease the live loft toward the phase target (already smooth, this just removes any
    // residual step at a redraw / slope onset — no pop).
    {
      const a = 1 - Math.exp(-TUNE.loftLerp * dt);
      this._loft += (loftTargetH - this._loft) * a;
      if (this._loft < 1e-4) this._loft = 0;
    }
    // body sits at the grounded pose RAISED by the loft (physics +down ⇒ subtract).
    this._bodyY = groundedY - this._loft;
    // vertical velocity of the body (for cube.velocity / render) — the loft's rate.
    this._vy = -(this._loft - (this._prevLoft || 0)) / Math.max(1e-4, dt);
    this._prevLoft = this._loft;
    // "airborne" === the loft has lifted the foot clear of the surface in this stride.
    // Legs STILL roll throughout (ω>0); this flag only tells the verifier the foot is
    // legitimately off the ground (so it skips the contact gap / slip / penetration
    // checks for these frames — they are the float phase of a stride, not a violation).
    const wasAir = this._air;
    this._air = this._loft > LOFT_AIR_EPS;
    if (this._air) this._airFrames++; else this._airFrames = 0;
    if (wasAir && !this._air) this._landMerge = 1; // touchdown this frame (cosmetic)
    if (this._landMerge > 1e-3) this._landMerge *= Math.exp(-TUNE.landMergeLerp * dt);
    else this._landMerge = 0;

    // CAMERA BASE: low-pass the cube height so the SCREEN is smooth (no per-foot
    // bob jolt, no loft hop jolt). The camera follows the GROUND trend — NOT the
    // per-stride loft — so while the cube hops in-frame the screen keeps gliding along
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
    this.cube.velocity.y = this._vy; // loft rate (0 when grounded/flat)
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
    // REUSE a scratch result object (no per-frame {x,h} allocation). The caller
    // (update) reads .x/.h THIS step before any other _nextRiser call, so a single
    // reused object is safe. Byte-identical fields to the old freshly-allocated one.
    let bestX = Infinity, bestH = 0, any = false;
    for (const s of this._segs) {
      if (s.kind === 'stairs' && s.stepH > 0) {
        // the riser sits at the LEFT edge of a stairs tread (x0).
        if (s.x0 > fromX && s.x0 <= toX) {
          if (!any || s.x0 < bestX) { bestX = s.x0; bestH = s.stepH; any = true; }
        }
      } else if (s.kind === 'wall' && s.stepH > 0) {
        const rx = (s.x0 + s.x1) / 2;
        if (rx > fromX && rx <= toX) {
          if (!any || rx < bestX) { bestX = rx; bestH = s.stepH; any = true; }
        }
      }
    }
    if (!any) { this._riserHit.found = false; return null; }
    const h = this._riserHit;
    h.found = true; h.x = bestX; h.h = bestH;
    return h;
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
      // reuse the existing parts array/objects in place (no per-tick allocation).
      l.body.parts = this._buildParts(axleX, axleY, angle, l.chain, l.body.parts);
    }
    // defensive UP-ONLY clamp against the surface UNDER EACH FOOT POINT (its own x —
    // NOT the surface at the body centre, which is wrong on a slope and would falsely
    // LIFT the body when a leg sample trails over lower ground, re-introducing the
    // "floating on slopes" gap). While AIRBORNE the body is meant to be above the
    // surface (positive clearance), so the clamp is skipped entirely. During the PRE-RACE
    // IDLE FLOAT the body is DELIBERATELY hovering above the surface (a drawn first leg
    // points down past the surface), so the clamp must NOT yank it down to ground.
    if (this._air || this._idleFloat) return;
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
        m.body.parts = this._buildParts(axleX, nAxleY, this._theta + m.phaseOffset + tilt, m.chain, m.body.parts);
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
  /** True while blocked specifically by a low ceiling (a too-long leg at a tunnel mouth). */
  get blockedByTunnel() { return !!this._blockedByTunnel; }
  /** True while blocked specifically by a riser (a too-short leg at a wall/stairs step). */
  get blockedByRiser() { return !!this._blockedByRiser; }
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
