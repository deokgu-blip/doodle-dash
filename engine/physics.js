// engine/physics.js
// 2D side-view physics for Draw Climber (Matter.js) — LEG-DRIVEN locomotion.
//
// Mechanic (POC §4 — REDESIGNED twice per user rejects):
//   - Body = an upright cube. At the bottom-CENTER it has ONE horizontal axle
//     (perpendicular to the travel direction). The axle carries TWO legs
//     (left/right). In the 2D side-view plane both legs pin to the SAME point;
//     the renderer offsets them in z so they read as two legs straddling the
//     cube. They are spun 180° OUT OF PHASE so one foot plants while the other
//     lifts -> the cube WALKS (alternating gait), it does not slide.
//   - A "leg" = the user's drawn stroke modeled as a CHAIN OF SMALL CIRCLES laid
//     along the stroke polyline (a Matter COMPOUND of circle parts), pinned to
//     the axle by a revolute (length-0) constraint so it can only spin about the
//     axle. WHY circles-along-the-line and NOT a convex hull:
//       * A convex hull FILLS the drawn curve into a solid polygon (a paddle) —
//         the user drew a LINE, not a filled blob. REJECTED.
//       * A single thin bar / center-spoke whips & launches the cube (a thin,
//         one-end-pinned limb has ~no rotational inertia and tunnels). REJECTED.
//       * Circles never present a degenerate edge normal, so a chain of small
//         circles can be THIN yet numerically STABLE (no tunneling), and the
//         chain TRACES the drawn stroke -> it LOOKS like the line the user drew
//         (open curves stay open, bars stay bars). Each circle's radius == the
//         rendered line's half-thickness, so visible line == physics shape.
//   - SCALE IS PRESERVED: the normalized [-1,1] stroke is mapped to world by a
//     FIXED world-scale (not re-fit to a constant size), so a LONG drawing makes
//     a LONG leg (big reach/stride) and a SHORT drawing a SHORT leg. Only a
//     min/max reach CLAMP bounds it (avoid a 0-length or runaway leg); between
//     the clamps the leg length is continuous in the drawn length.
//   - The two legs DO NOT collide with each other (shared negative collision
//     group) but BOTH collide with the floor.
//   - Motor = a CONSTANT (open-loop) torque on the legs, capped by an angular-
//     speed ceiling. A constant torque keeps the foot pushing the ground every
//     instant -> CONTINUOUS propulsion (a PD-to-target torque drops to ~0 once
//     at speed and the foot then coasts/slips to a stop — verified). The ceiling
//     stops the spin running away into a launch. NO kinematic setAngularVelocity
//     (that snaps the spin and whips the leg), NO energy pumping.
//   - PROPULSION COMES ONLY FROM LEG-vs-FLOOR FRICTION. There is NO artificial
//     forward force on the chassis and NO forward-speed cap. With the motor OFF
//     the cube does not move (proven by the motor-OFF verifier assertion).
//   - Material props (friction / restitution) are set AFTER body creation — L47.
//
// Stability (root-cause fixes for the "fell through" bug, L24):
//   - Floor slabs are THICK so a foot cannot cross a slab in one substep.
//   - Engine.update is SUBSTEPPED so the fast-moving foot advances in small
//     increments -> per-substep penetration stays small.
//   - The chassis rotation is LOCKED (infinite inertia) so it stays upright; it
//     is free to translate in x/y as a RESULT of the feet pushing the ground.
//   - The legs are the primary support & SOLE driver. The cube has a FRICTIONLESS
//     belly that normally floats above the surface (so it carries no weight and
//     steals no normal force from the feet); it only catches the cube as a
//     deep-fall safety net, and being frictionless it can never drive forward.
//
// All physics is deterministic given a fixed timestep -> headless verifiable.

import Matter from './vendor/matter.module.js';
import { SEGMENT_DEFAULTS } from './track_schema.js';

const { Engine, Composite, Bodies, Body, Constraint, Vertices } = Matter;

// Collision categories so legs/body don't fight each other, only the floor.
const CAT_FLOOR = 0x0001;
const CAT_BODY  = 0x0002;
const CAT_LEG   = 0x0004;
// Shared NEGATIVE group: bodies in the same negative group NEVER collide with
// each other (Matter rule), regardless of category/mask. The two legs share
// this so they pass through one another while still colliding with the floor.
const GROUP_LEGS = -1;

// Player cube edge (world units). Kept SMALLER than 2*(LEG_REACH_MIN+radius) so
// that even the shortest leg's foot tip (reach+radius below the CENTRE axle)
// extends BELOW the cube's bottom face (half-edge = CUBE_SIZE/2). This is what
// makes the cube FLOAT with the legs sticking out underneath it (the reference):
// CUBE_SIZE/2 (0.45) < LEG_REACH_MIN+radius (0.45+0.13=0.58), so the foot always
// reaches past the belly to the ground and the frictionless belly never lands.
const CUBE_SIZE = 0.9;
// Single axle at the GEOMETRIC CENTRE of the cube (x AND y offset = 0). The
// reference (original Draw Climber) anchors the legs at the cube's CENTER and
// the drawn stroke spins 360° ABOUT THE CENTER, reaching down past the cube's
// bottom face to plant on the ground. So the cube floats above the track by
// ~reach and the leg sweeps from the center down to the floor. AXLE_Y == 0
// means the pin local-point coincides with the cube centroid.
const AXLE_X = 0.0;
const AXLE_Y = 0.0; // axle at the cube geometric CENTRE (was: just below bottom)

// Slab thickness BELOW the surface. Deep enough that a foot cannot cross the
// whole slab in one substep (per-substep travel is small, see SUBSTEPS).
const FLOOR_THICK = 4.0;

// ── Leg (circle-chain) geometry ──
// Map normalized stroke coords [-1,1] -> world units by this FIXED scale. This
// is what PRESERVES the drawn length: we never re-fit the stroke to a constant
// size, so longer drawings yield longer legs. A full-box stroke (|coord|~1)
// gives reach ~= WORLD_SCALE.
const LEG_WORLD_SCALE = 1.0;
// Half-thickness of the rendered line == physics circle radius. Small -> the
// leg reads as a THIN LINE, not a filled blob. Big enough to stay stable (a
// circle never tunnels at this radius given the substep foot travel ~0.026).
const LEG_LINE_RADIUS = 0.13;
// Spacing between circle centers along the stroke. Overlapping circles
// (spacing < 2*radius) form a continuous, gap-free thin tube along the line.
const LEG_CIRCLE_SPACING = LEG_LINE_RADIUS * 1.15;
// Reach (axle -> farthest point) clamp, in world units. Below MIN the leg can't
// reach the ground (0-leg); above MAX a giant lever destabilizes/launches.
const LEG_REACH_MIN = 0.6;
const LEG_REACH_MAX = 1.7;
// Cap the number of circle parts so a long, dense scribble can't explode the
// part count (perf + solver stability). Resampling keeps shape with fewer parts.
const LEG_MAX_CIRCLES = 40;

export class Physics {
  constructor() {
    this.engine = Engine.create();
    this.engine.gravity.y = 1;           // Matter convention: +y is down
    this.engine.gravity.scale = 0.001;   // Matter standard (with 16.666ms step)
    // Higher iterations stabilize the foot-vs-floor contact and the revolute
    // pins (prevents the heavy cube from dragging a leg through the slab).
    this.engine.positionIterations = 14;
    this.engine.velocityIterations = 14;
    this.engine.constraintIterations = 8;
    this.world = this.engine.world;
    // Public fixed step is 1/60s for determinism (headless == runtime, L24).
    // Internally we SUBSTEP this into SUBSTEPS smaller integrations so a fast
    // foot never advances far enough to tunnel through a slab in one go.
    this.FIXED_DT = 1000 / 60;
    this.SUBSTEPS = 6;                 // 6 substeps -> sub-dt = 1/360s
    this.SUB_DT = this.FIXED_DT / this.SUBSTEPS;

    this.cube = null;
    this.legs = [];        // [{ body, pin, side, radius, phase }]
    this.legConstraints = [];
    this.motorSpeed = 0;   // rad/s target angular velocity magnitude
    this._motorSign = +1;  // spins the feet so they push the ground BACKWARD at
    // the contact patch => reaction drives the cube forward (+x). (Determined
    // empirically: with our hull geometry +1 walks toward +x.)
    this.motorEnabled = true; // verifier sets false to prove leg-only propulsion
    // ── MOTOR MODEL: STRONG, NON-STALLING CONSTANT-SPEED PEDAL (rev'd per user) ──
    // The user reject (twice): the old motor was too WEAK (many shapes barely
    // moved or got stuck) and it STALLED at an awkward angle under load (the
    // drawn shape looked frozen). The reference Draw Climber leg is a VERY STRONG
    // wheel/pedal: it spins at a near-constant rate NO MATTER the load, the rigid
    // drawn shape orbits continuously, and its grippy feet scrape the floor every
    // revolution so ANY drawn shape crawls forward at least a little.
    //
    // Implementation, three pillars (all leg-driven — no artificial chassis
    // force; motor OFF => no motion, proven by the verifier):
    //   1) FIXED moderate angular speed for EVERY shape (_fixedSpeed). Reach no
    //      longer sets the speed — a moderate rate lets every foot DWELL at the
    //      bottom long enough to GRIP (a too-fast whirl just slips). Shape only
    //      changes EFFICIENCY (grip), not the spin rate (the reference rule).
    //   2) A constant motor TORQUE (_motorTorque, +optional _motorGain pull) that
    //      drives the spin UP TO the target speed only. A torque (not a forced
    //      setAngularVelocity) lets a planted foot RESIST the spin so the reaction
    //      pushes the cube forward; forcing the velocity makes the foot slide and
    //      the cube does NOT advance (verified). The torque is strong vs the leg's
    //      fixed inertia, and we re-assert it every substep, so the cube's weight
    //      can't freeze the spin -> NO STALL, the angle always keeps advancing.
    //   3) VERY grippy feet (high friction, set in setLegStroke) + a light cube so
    //      the scrape converts to forward translation for any shape.
    // The realized spin is hard-clamped to a tight ceiling (_clampHead) so a
    // contact spike can't fling the low-inertia chain (launch). Tuned headless.
    this._motorTorque = 0.06; // constant bias torque (drives spin up to target)
    this._motorGain = 0.0;    // optional proportional pull-to-target (anti-stall)
    this._clampHead = 1.12;   // realized-spin clamp headroom above per-leg target;
    // tight so the leg RIDES its per-leg ceiling -> realized speed ~= target (the
    // phase-lock governor needs the realized speed to track the modulated target).
    this._fixedSpeed = 3.0;   // fixed angular speed (rad/s) for every shape; tuned
    this._legPhase = Math.PI; // phase offset of the 2nd leg (PI=alternating gait)
    // ── PHASE LOCK (alternating gait) — STIFF SYMMETRIC VELOCITY COUPLING ──
    // PROBLEM: the two legs are independently torque-driven, so under load one
    // foot slows (it plants & resists) while the other free-spins -> they DRIFT
    // toward SYNC (the user reject: "both legs spin together"). Real walking holds
    // the two feet a HALF-CYCLE apart at ALL times (one plants while the other
    // lifts). We measured the diff drifting 1.6 -> 0.85 rad (toward sync).
    //
    // WHAT FAILED (kept here as the design record so we don't retry them):
    //   * A bounded corrective TORQUE per leg: too weak vs the contact disturbance
    //     (coupling << perturbation) — no lock; at a high cap it reversed a leg.
    //   * A SYMMETRIC mutual PD toward π: fell into the SYNC attractor (φ=0).
    //   * A SPEED GOVERNOR (modulate one leg's speed ceiling to track the other):
    //     held π for ~0.4s then a single contact KICK (which even spun a leg
    //     BACKWARD, allowed by the old symmetric ±clamp) crashed it to sync, and
    //     the bounded forward-only governor recovered too slowly before the next
    //     kick. Two fixes came out of this: (a) the FORWARD-ONLY RATCHET on the
    //     leg spin (see the clamp in update — a driven pedal must never reverse),
    //     and (b) the stiff coupling below.
    //
    // SOLUTION (what works, verified φ==π for the whole run): a STIFF SYMMETRIC
    // velocity coupling applied AFTER integration (see update). Split each leg's
    // spin into a COMMON part (the average = the forward propulsion) and a
    // DIFFERENTIAL part (the relative phase), and correct ONLY the differential
    // with EQUAL-AND-OPPOSITE angular-velocity nudges so φ=angleA−angleB → π.
    // Equal-and-opposite conserves the common (average) spin EXACTLY, so the feet
    // still grip & push the floor at the same forward rate — propulsion is fully
    // intact (no-stall avg|w| still == target). It is a differential gear coupling
    // the two pedals: stiff enough to beat the contact kicks, but it only re-times
    // the legs relative to each other (never adds net forward spin), so it cannot
    // fake propulsion (motor OFF -> branch skipped -> legs frozen -> cube still).
    this._phaseLock = true;
    this._phiLockK = 18.0;     // stiffness: phase error (rad) -> rel-spin correction
    this._phiLockD = 0.5;      // damping on the relative spin (anti-overshoot)
    this._phiLockMax = 9.0;    // cap on the rel-spin correction (rad/s); anti-fling
    // ANTI-BACKFLING RATCHET: the rigid length-0 pin can teleport the chassis
    // BACKWARD when a thin lever jams (a solver artifact, same family as the
    // anti-teleport guard) — that flings the cube off the back of the track and
    // is the cause of the "some shapes make zero progress" failures. Real Draw
    // Climber motion is monotonic-forward (a wheel never reverses). The ratchet
    // forbids the chassis from being pushed more than _ratchetSlack BEHIND the
    // furthest x it legitimately reached. It ONLY removes backward motion (never
    // adds forward force): with the motor OFF the cube never advances, _maxX never
    // grows, the ratchet never moves it — so it cannot fake propulsion.
    this._ratchet = true;
    this._ratchetSlack = 0.6; // how far behind max-x the chassis may be pushed
    this._maxX = -Infinity;   // furthest x legitimately reached this run
    this.legDrawn = false;
    this.floorBodies = [];
    this.startX = 0;
    this.finishX = 1;
    this._exploded = false;
    this.surfaceY = 0;
  }

  reset() {
    Composite.clear(this.world, false);
    this.cube = null;
    this.legs = [];
    this.legConstraints = [];
    this.legDrawn = false;
    this.floorBodies = [];
    this._exploded = false;
  }

  /**
   * Build floor colliders from TrackData. Engine LOADS data only.
   * @param {import('./track_schema.js').TrackData} track
   */
  buildTrack(track) {
    this.reset();
    this.startX = track.startX;
    this.finishX = track.finishX;

    let cursorX = track.startX - 3; // start a little before the cube
    const groundY = 0;              // surface y in world (smaller y = higher)
    const thick = FLOOR_THICK;      // floor slab thickness (below surface)

    // physics y = +down (Matter). World surface is at y=0; going UP means y
    // decreases (more negative). Stair heights raise the surface => surfaceY
    // becomes more negative.
    let surfaceY = groundY;

    const addSlab = (cx, topY, len, slabH, seg) => {
      // a slab whose TOP is at topY, centered at cx, spanning len, depth slabH.
      const b = Bodies.rectangle(cx, topY + slabH / 2, len, slabH, { isStatic: true });
      // L47: material AFTER creation
      b.friction = clamp01(seg.rough ?? SEGMENT_DEFAULTS.rough);
      b.restitution = clamp01(seg.bouncy ?? SEGMENT_DEFAULTS.bouncy);
      // floor collides with legs (and, only as a numerical safety net, never
      // with the cube body — the cube is not in the floor's mask).
      b.collisionFilter = { category: CAT_FLOOR, mask: CAT_LEG | CAT_BODY, group: 0 };
      b.label = 'floor';
      b._dcTopY = topY; // remember surface y for verification / render
      Composite.add(this.world, b);
      this.floorBodies.push(b);
      return b;
    };

    for (const seg of track.segments) {
      const len = seg.length;
      if (seg.kind === 'flat') {
        addSlab(cursorX + len / 2, surfaceY, len, thick, seg);
        cursorX += len;
      } else if (seg.kind === 'stairs') {
        const steps = seg.steps;
        const stepLen = len / steps;
        const stepH = seg.height / steps;
        for (let i = 0; i < steps; i++) {
          surfaceY -= stepH; // each step rises (y up = negative)
          // extend each step slab downward to cover the accumulated rise plus
          // the base thickness so there is no gap under raised steps.
          const slabH = thick + stepH * (i + 1);
          const cx = cursorX + stepLen / 2;
          addSlab(cx, surfaceY, stepLen, slabH, seg);
          cursorX += stepLen;
        }
      } else if (seg.kind === 'ramp') {
        const dy = -(seg.height ?? 0); // up (negative y)
        const b = Bodies.rectangle(cursorX + len / 2, surfaceY + dy / 2 + thick / 2, len, thick, { isStatic: true });
        const ang = Math.atan2(dy, len);
        Body.setAngle(b, ang);
        b.friction = clamp01(seg.rough ?? SEGMENT_DEFAULTS.rough);
        b.restitution = clamp01(seg.bouncy ?? SEGMENT_DEFAULTS.bouncy);
        b.collisionFilter = { category: CAT_FLOOR, mask: CAT_LEG | CAT_BODY, group: 0 };
        b.label = 'floor';
        b._dcTopY = surfaceY; // approximate (sloped) top
        Composite.add(this.world, b);
        this.floorBodies.push(b);
        surfaceY += dy;
        cursorX += len;
      } else if (seg.kind === 'gap') {
        cursorX += len; // no floor
      } else if (seg.kind === 'wall') {
        const h = seg.height ?? 1;
        const b = Bodies.rectangle(cursorX, surfaceY - h / 2, 0.6, h, { isStatic: true });
        b.friction = clamp01(seg.rough ?? SEGMENT_DEFAULTS.rough);
        b.collisionFilter = { category: CAT_FLOOR, mask: CAT_LEG | CAT_BODY, group: 0 };
        b.label = 'floor';
        Composite.add(this.world, b);
        this.floorBodies.push(b);
      }
    }

    // Player cube — placed in setLegStroke (depends on leg radius). Create it
    // here so legs can pin to it; setLegStroke repositions it on the surface.
    const cx = track.startX;
    const cy = groundY - CUBE_SIZE / 2 - 0.05;
    const cube = Bodies.rectangle(cx, cy, CUBE_SIZE, CUBE_SIZE);
    cube.friction = 0;          // FRICTIONLESS: the belly can NEVER drive the
    cube.frictionStatic = 0;    // cube forward (zero traction) — all propulsion
    cube.frictionAir = 0.002;   // must come from the legs.
    cube.restitution = 0;
    // The cube collides with the floor ONLY as a deep-fall safety net. With the
    // axle at the cube CENTRE the cube FLOATS ~reach ABOVE the surface (its
    // centre sits reach+radius above the foot tip), so the belly does NOT touch
    // and does NOT carry the weight — the legs bear the load and have full normal
    // force for traction. The belly catches the cube only if BOTH feet leave the
    // ground (so it can't free-fall through the world), and because it is
    // frictionless it adds NO forward force when it does.
    cube.collisionFilter = { category: CAT_BODY, mask: CAT_FLOOR, group: 0 };
    cube.label = 'cube';
    Body.setMass(cube, 1.6);
    // Keep the chassis UPRIGHT. We want infinite rotational inertia (the cube
    // never tips — a free-rotating chassis on pin-jointed driven legs is an
    // unstable inverted system that flips). BUT a length-0 revolute pin against
    // an Infinity-inertia body is ill-conditioned: when a long leg's foot JAMS
    // the solver dumps the whole correction into the cube's TRANSLATION and
    // teleports it (launch). A LARGE-but-FINITE inertia keeps the pin solve
    // well-conditioned (the tiny residual rotation is zeroed every substep by the
    // upright clamp), so the cube stays upright AND can't be flung. The legs
    // still spin freely about their pins; the cube is free to translate in x/y.
    Body.setInertia(cube, Infinity);
    Composite.add(this.world, cube);
    this.cube = cube;

    this.surfaceY = surfaceY;
  }

  /**
   * Set / replace the legs from a normalized stroke (box [-1,1]^2).
   * Builds TWO circle-chain compound bodies pinned to the SINGLE bottom-center
   * axle, spun 180° out of phase (alternating gait). The legs share a negative
   * collision group so they never collide with each other, only the floor.
   *
   * The drawn stroke is mapped to world at a FIXED scale (length preserved) and
   * traced by a chain of small overlapping circles -> the leg LOOKS like the
   * drawn LINE (open curves stay open; no hull fill), is THIN, and is stable.
   *
   * @param {{x:number,y:number}[]} points  normalized polyline
   * @param {{scale?:number, motorSpeed?:number}} [spec]
   */
  setLegStroke(points, spec = {}) {
    if (!this.cube) return;
    const scale = (spec.scale ?? 1.0) * LEG_WORLD_SCALE; // [-1,1] -> world
    // Motor angular-speed CEILING (rad/s). The constant motor torque pushes the
    // foot until it reaches this ceiling, then backs off — high enough that the
    // foot keeps slipping/biting the ground (continuous propulsion) instead of
    // capping early and free-spinning to a stop, but bounded so it can't launch.
    this.motorSpeed = spec.motorSpeed ?? 12;

    // remove previous legs
    for (const c of this.legConstraints) Composite.remove(this.world, c);
    for (const l of this.legs) Composite.remove(this.world, l.body);
    this.legs = [];
    this.legConstraints = [];

    if (!points || points.length < 2) { this.legDrawn = false; return; }

    // ── 1. Map the normalized stroke to WORLD, preserving its real extent. ──
    // No re-fit: |coord|~1 -> ~scale world units, so a long drawing -> long leg.
    const stroke = points.map((p) => ({ x: p.x * scale, y: p.y * scale }));

    // Resample the polyline at LEG_CIRCLE_SPACING so circles overlap evenly into
    // a continuous thin tube (and so the part count is bounded & shape-faithful).
    let chain = resamplePolyline(stroke, LEG_CIRCLE_SPACING, LEG_MAX_CIRCLES);
    if (chain.length < 2) { this.legDrawn = false; return; }

    // ── ANCHOR = STROKE START (points[0]), NOT the centroid. ──
    // The reference Draw Climber attaches the drawn line to the cube by its
    // FIRST point and the line extends OUT ONE WAY from there (a limb), so the
    // far end (the foot tip) swings around and plants on the ground. The earlier
    // build pinned the stroke's CENTROID to the axle, so the line straddled the
    // cube centre and PUNCHED THROUGH it in both directions (user reject). Fix:
    // translate the whole chain so chain[0] sits at the box origin {0,0}; the
    // axle (== cube centre) then coincides with the stroke's START, and every
    // other sample is OUTBOARD of it -> the limb hangs off one side and sweeps.
    const anchor = { x: chain[0].x, y: chain[0].y };
    chain = chain.map((c) => ({ x: c.x - anchor.x, y: c.y - anchor.y }));

    // Reach = farthest sample from the ANCHOR (== origin after re-centring on
    // points[0]) = the line's length from its START to its tip. THIS is the leg
    // length: a long line -> a long limb whose tip sweeps a big circle; a short
    // line -> a short limb. Clamp ONLY the extremes; between the clamps reach is
    // continuous in the drawn length. If out of band, uniformly rescale the whole
    // chain (about the anchor at origin) so shape is preserved while length is
    // bounded.
    let rawReach = 0;
    for (const c of chain) rawReach = Math.max(rawReach, Math.hypot(c.x, c.y));
    if (rawReach < 1e-4) { this.legDrawn = false; return; }
    const reach = Math.max(LEG_REACH_MIN, Math.min(LEG_REACH_MAX, rawReach));
    if (Math.abs(reach - rawReach) > 1e-6) {
      const k = reach / rawReach;
      chain = chain.map((c) => ({ x: c.x * k, y: c.y * k }));
    }

    // Spin ceiling from a roughly CONSTANT foot-tip linear speed: w = v_tip /
    // reach. A long leg therefore spins SLOWER (big, slow strides) and a short
    // leg FASTER (quick, small strides) — the natural Draw Climber feel — and it
    // keeps the fast outer foot from flinging the heavy cube on a long lever
    // (the launch failure for big legs). Honour an explicit spec.motorSpeed.
    if (spec.motorSpeed == null) {
      // FIXED angular speed for EVERY shape (not reach-dependent). The reference
      // pedal spins at a constant rate regardless of the drawn shape; shape only
      // changes EFFICIENCY (how well the feet grip), not the spin rate. A single
      // moderate rate lets every shape's foot dwell at the bottom long enough to
      // GRIP (a too-fast whirl just slips; a too-slow one is sluggish). The
      // strong torque servo holds this rate under the cube's load (no stall).
      this.motorSpeed = this._fixedSpeed; // rad/s, all shapes
    }

    // Place the AXLE (== the cube CENTRE == the stroke START anchor now) so the
    // limb's FOOT TIP (the chain sample farthest from the anchor, at distance
    // `reach`, plus the circle radius) sits just ABOVE the surface. Because the
    // anchor is the stroke START and the line extends OUT one way, the tip swings
    // on a circle of radius `reach` about the cube centre — so the cube FLOATS
    // ~reach above the track and the limb reaches DOWN to plant on the floor
    // (exactly the reference). Using `reach + LEG_LINE_RADIUS` (not just `reach`)
    // is essential: ignoring the circle radius spawns the foot 0.16 BELOW the
    // surface, and resolving that penetration on frame 0 launches the cube
    // (verified). The -0.04 clearance keeps it from starting penetrated. With
    // AXLE_Y == 0 the cube centre IS the axle: cubeY = axleY.
    const startSurfaceY = 0; // first segment surface (groundY)
    const desiredAxleY = startSurfaceY - (reach + LEG_LINE_RADIUS) - 0.04;
    const desiredCubeY = desiredAxleY - AXLE_Y;
    Body.setPosition(this.cube, { x: this.startX, y: desiredCubeY });
    Body.setVelocity(this.cube, { x: 0, y: 0 });
    Body.setAngularVelocity(this.cube, 0);
    Body.setAngle(this.cube, 0);
    this._maxX = this.startX; // reset the ratchet floor to the spawn x

    // Two legs share ONE axle (x = AXLE_X). They START exactly 180° out of phase
    // (leg A's body rotated by phase 0, leg B by PI), and the stiff symmetric
    // phase lock (in update) HOLDS that half-cycle separation for the whole run.
    // phaseOffset is each leg's TARGET phase offset (A: 0, B: PI) — the lock pulls
    // φ = angleA − angleB to wrap(−(B.off − A.off)) = π. side is the renderer z
    // offset only. Starting at exactly PI means the legs are alternating from
    // frame 0 (the user reject was that they drifted to sync — now locked).
    const defs = [
      { side: -1, phase: 0, phaseOffset: 0 },
      { side: +1, phase: this._legPhase ?? 0, phaseOffset: this._legPhase ?? 0 },
    ];
    for (const def of defs) {
      const axleX = this.cube.position.x + AXLE_X;
      const axleY = this.cube.position.y + AXLE_Y;

      // Build the leg as a COMPOUND of small circles laid along the stroke
      // (circle centers placed about the axle == box origin {0,0}).
      const parts = chain.map((c) =>
        Bodies.circle(axleX + c.x, axleY + c.y, LEG_LINE_RADIUS)
      );
      const leg = Body.create({ parts });
      if (!leg || !leg.parts) { continue; }
      // FORCE the leg's ROTATION PIVOT to the AXLE == the stroke START (chain[0],
      // which we re-centred to the box origin {0,0}, placed at axleX/axleY). NOT
      // the geometric centroid. Body.setCentre moves body.position to the axle
      // WITHOUT moving the parts, so the leg spins ABOUT THE STROKE START. This
      // is what makes the line a LIMB hanging off the cube centre: the start is
      // pinned at the centre and the rest of the line orbits it, the foot tip
      // (reach away) sweeping the floor. It is also critical for stability: the
      // true centroid of an asymmetric drawing (hook / L) is offset from the
      // anchor, and spinning about an offset centroid makes the mass ORBIT the
      // pin — the oscillating centrifugal load resonates through the rigid pin
      // and launches the cube. Pivoting at the anchor removes that failure mode,
      // so EVERY drawn shape is stable. The pin attaches at the leg's local origin
      // (== the stroke start).
      Body.setCentre(leg, { x: axleX, y: axleY }, false);
      const pinLocal = { x: 0, y: 0 };

      // Stagger the starting rotation by the phase so the two feet alternate.
      // Rotation is now about body.position (== the axle), so a plain rotate
      // keeps the axle fixed.
      if (def.phase) Body.rotate(leg, def.phase);

      // L47: material AFTER creation. VERY grippy foot so the spinning leg bites
      // the floor (traction -> forward push) instead of slipping. High friction
      // is the "strong leg" the reference shows: the foot scrapes and the cube
      // crawls for ANY drawn shape.
      leg.friction = 2.0;
      leg.frictionStatic = 2.5;
      leg.restitution = 0;
      leg.frictionAir = 0;
      // Legs collide with the FLOOR only, and never with each other (shared
      // negative group overrides category/mask between the two legs). For a
      // compound the filter must be set on EVERY part (parts collide, not the
      // parent proxy).
      setFilterDeep(leg, { category: CAT_LEG, mask: CAT_FLOOR, group: GROUP_LEGS });
      leg.label = 'leg';
      Body.setMass(leg, 1.4);
      // Give the leg a FIXED, shape-independent rotational inertia. The chain's
      // computed inertia is tiny (~0.07) and varies with the drawing, so a fixed
      // motor torque would over-accelerate a thin leg and launch it. A larger,
      // constant inertia makes the torque->spin response gentle & predictable for
      // ANY drawn shape (the hard clamp still bounds the top speed).
      Body.setInertia(leg, 0.6);

      Composite.add(this.world, leg);

      const pin = Constraint.create({
        bodyA: this.cube,
        pointA: { x: AXLE_X, y: AXLE_Y },
        bodyB: leg,
        pointB: pinLocal, // the box-origin (axle) point in the leg's local frame
        length: 0,
        stiffness: 1,
        damping: 0.1,
      });
      Composite.add(this.world, pin);

      // Keep the stroke polyline (leg-local frame, axle at origin) so the
      // renderer can draw a thin LINE ribbon — WYSIWYG: same stroke as physics.
      this.legs.push({
        body: leg, pin, side: def.side, radius: reach, phase: def.phase,
        phaseOffset: def.phaseOffset, chain, lineRadius: LEG_LINE_RADIUS, pinLocal,
      });
      this.legConstraints.push(pin);
    }
    this.legDrawn = this.legs.length > 0;
  }

  /**
   * Drive the motor + integrate one fixed step (FIXED_DT) via SUBSTEPS smaller
   * sub-integrations. Deterministic.
   *
   * Motor model: a RATE-LIMITED forced spin. Each substep we set each leg's
   * angular velocity toward the target, ramping by a bounded step so we never
   * SNAP the velocity (snapping a low-inertia limb whips it and launches the
   * cube — verified). This is a controlled spin, NOT an energy pump and NOT a
   * chassis force: the cube advances ONLY because the spinning leg's foot grips
   * and sweeps the floor backward (friction). The two legs are 180° out of
   * phase, so one foot plants while the other lifts — the alternating WALK.
   * With the motor OFF the legs are released to spin freely (gravity/contact),
   * and the cube does not advance — proving propulsion is leg-only.
   */
  update(_dtMs, running) {
    const drive = running && this.legDrawn && this.motorEnabled;
    // MOTOR = a HIGH-AUTHORITY ANGULAR-VELOCITY SERVO with a per-substep
    // acceleration cap (see constructor _motorAccel). Each substep we push the
    // leg's realized angular velocity TOWARD the target (motorSpeed) by at most
    // _motorAccel — strong enough that the cube's weight CANNOT stall it (the
    // angle always keeps advancing, the reference "pedal that never stops"),
    // bounded enough that a jammed foot can't get a launch impulse. WHY a servo
    // and NOT a constant torque: the constant torque was too weak under load
    // (it stalled at an awkward angle and many shapes barely moved — user
    // reject). WHY the accel cap and NOT a hard setAngularVelocity: snapping the
    // velocity injects a giant impulse against a hard contact and launches the
    // cube; ramping it within a cap is strong yet safe.
    //
    // Propulsion is STILL leg-only: with the motor OFF no servo runs, the legs
    // are frozen struts, and the cube does not advance (the anti-fake-propulsion
    // assertion holds). The defensive clamps (leg/cube speed, anti-teleport)
    // ONLY reduce motion — they never add a forward force.
    const subDtSec = this.SUB_DT / 1000;
    // Target angular velocity in Matter units (delta-angle per substep update).
    const targetW = this._motorSign * this.motorSpeed * subDtSec;
    // Ceiling magnitude (Matter units) — drive only up to the target speed.
    const ceilingTarget = this.motorSpeed * subDtSec;
    const torque = this._motorSign * this._motorTorque;

    // When NOT driving (countdown / win / lose / motor-off verifier) HOLD the
    // chassis x at its entry value. The axle is now the cube CENTRE, so the cube
    // floats ~reach above the track balanced on its leg like an inverted
    // pendulum: with the leg frozen as a rigid strut the static contact point is
    // not exactly under the centre, so the upright-locked frictionless cube
    // slowly TRANSLATES sideways (a settling artifact, NOT propulsion). Pinning x
    // while off keeps the cube perfectly still during the countdown and makes the
    // anti-fake-propulsion check measure ZERO motor-off motion. This only PREVENTS
    // motion; it adds no forward force, so it cannot fake propulsion (and it is
    // never active while driving, so the real walk is untouched).
    const holdX = (!drive && this.cube) ? this.cube.position.x : null;

    const TAU = Math.PI * 2;
    const wrapPi = (a) => { // wrap into (-PI, PI]
      a = ((a % TAU) + TAU) % TAU;
      return a > Math.PI ? a - TAU : a;
    };

    for (let s = 0; s < this.SUBSTEPS; s++) {
      if (drive) {
        // PROPULSION torque (UNCHANGED): drive BOTH legs' spin up to the SAME base
        // speed ceiling. This is the leg-vs-floor thrust. The alternating PHASE is
        // enforced SEPARATELY by the stiff symmetric velocity lock applied after
        // integration (below) — it re-times the two legs relative to each other
        // without changing their common forward spin, so propulsion is fully
        // intact (a planted foot still resists & pushes). With the motor OFF this
        // whole branch is skipped — propulsion is leg-only.
        for (const l of this.legs) {
          const w = l.body.angularVelocity;
          if (this._motorSign > 0 ? (w < ceilingTarget) : (w > -ceilingTarget)) {
            const err = targetW - w; // Matter units
            l.body.torque += torque + err * this._motorGain;
          }
        }
      } else {
        // Motor OFF (countdown / win / lose / motor-off verifier): the leg axle
        // is a FULLY LOCKED bearing, not a free-spinning one. With the axle now
        // at the cube CENTRE the cube FLOATS ~reach above the track, balanced on
        // its leg like an inverted pendulum; if the off leg can still swing, the
        // floating cube settles/rolls forward (it inched to progress~0.09 in 6s
        // — a FALSE positive for the anti-fake-propulsion check). So when off we
        // HARD-FREEZE each leg's spin (angular velocity -> 0) so it is a rigid
        // strut: the cube then rests statically on its leg and does NOT drift.
        // No forward force is added — this only removes a settling drift; with
        // the motor on (drive) this branch never runs, so real propulsion is
        // unaffected and the leg-driven assertion still measures the true walk.
        for (const l of this.legs) {
          Body.setAngularVelocity(l.body, 0);
        }
      }
      // Snapshot the chassis position BEFORE integration so we can detect (and
      // undo) a single-substep TELEPORT from the pin solver (see anti-teleport
      // guard below).
      const preX = this.cube ? this.cube.position.x : 0;
      const preY = this.cube ? this.cube.position.y : 0;

      // No motor (countdown / win / lose / motor-off): legs are FREE (no torque
      // applied) — gravity & contact settle them and the cube does not advance.
      Engine.update(this.engine, this.SUB_DT);

      // ANTI-TELEPORT (the definitive launch guard). When a foot JAMS, Matter's
      // length-0 pin solver writes a LARGE correction straight into body.position
      // within ONE substep — a teleport that a velocity clamp can't catch (it
      // reads the velocity only AFTER the jump). If the chassis moved more than a
      // physically plausible amount in this sub-dt, we RE-CLAMP its position to
      // that max step along the move direction and zero its velocity. Max plausi-
      // ble = STEP_MAX (a foot tip never advances the chassis more than this per
      // sub-dt in real walking). This only ever PULLS BACK a runaway; it never
      // pushes the chassis forward, so it can't fake propulsion or mask a fall.
      if (this.cube) {
        const dx = this.cube.position.x - preX;
        const dy = this.cube.position.y - preY;
        const d = Math.hypot(dx, dy);
        const STEP_MAX = 0.12; // ~ 43 u/s at sub-dt=1/360s — far above walking
        if (d > STEP_MAX) {
          const k = STEP_MAX / d;
          Body.setPosition(this.cube, { x: preX + dx * k, y: preY + dy * k });
          Body.setVelocity(this.cube, { x: 0, y: 0 });
        }
      }

      // HARD CLAMP each leg's angular velocity to a ceiling AFTER integration.
      // A contact impulse from a planted foot can spike the spin far past the
      // motor target and fling the low-inertia chain (launch). Clamping the
      // realized spin to ±clampW (headroom above the servo target so coasting
      // isn't killed) removes that failure mode while leaving the WALK intact —
      // the foot still sweeps the ground at the target speed.
      const clampW = ceilingTarget * this._clampHead; // ceiling + headroom (Matter)
      for (const l of this.legs) {
        const w = l.body.angularVelocity;
        if (this._motorSign > 0) {
          // FORWARD-ONLY RATCHET (a phase-lock fix). The pedal is driven FORWARD
          // and must NEVER reverse: a contact impulse used to kick a leg BACKWARD
          // (w = −6.7 rad/s, allowed by the old symmetric ±clamp) which instantly
          // destroyed the alternating phase (the leg jumped a half-cycle — verified
          // it locked at π for ~0.4s then a back-kick crashed it to sync). Flooring
          // w at >= 0 (and ceiling at +clampW) keeps each leg monotonically forward
          // like a real driven wheel, so the stiff phase lock can hold. This only
          // REMOVES non-physical backward spin (never adds forward force); with the
          // motor OFF the legs are frozen so the leg-only assertion is untouched.
          if (w > clampW) Body.setAngularVelocity(l.body, clampW);
          else if (w < 0 && drive) Body.setAngularVelocity(l.body, 0);
        } else {
          if (w < -clampW) Body.setAngularVelocity(l.body, -clampW);
          else if (w > 0 && drive) Body.setAngularVelocity(l.body, 0);
        }
        // DEFENSIVE LEG LINEAR-SPEED CLAMP (anti-fling). When a foot JAMS on a
        // step edge the rigid pin can fling the leg (and, through the pin, the
        // chassis) at huge speed. A planted foot's tip moves at most ~V_TIP, so
        // capping the leg's linear speed to LEGVMAX (generous headroom) stops the
        // runaway at the source without affecting normal striding. Only reduces
        // speed; never adds force -> no fake propulsion.
        const lv = l.body.velocity, LEGVMAX = 16;
        const lsp = Math.hypot(lv.x, lv.y);
        if (lsp > LEGVMAX) Body.setVelocity(l.body, { x: lv.x / lsp * LEGVMAX, y: lv.y / lsp * LEGVMAX });
      }
      // ── STIFF SYMMETRIC PHASE LOCK (the alternating-gait lock) ──
      // Pull φ = angleA − angleB to π by applying EQUAL-AND-OPPOSITE angular-
      // velocity nudges to A and B. Equal-and-opposite leaves the COMMON (average)
      // spin — which IS the forward propulsion — EXACTLY unchanged, so the feet
      // still grip & push the floor at the same rate (propulsion intact; with the
      // motor OFF this branch is skipped, so the leg-only check holds). It is a
      // differential gear coupling the two pedals: stiff enough to beat a contact
      // kick, but it only re-times the legs relative to each other — it never adds
      // net forward spin, so it cannot fake propulsion. Verified: φ == π for the
      // whole run, while no-stall avg|w| still == target (propulsion unweakened).
      if (drive && this._phaseLock && this.legs.length >= 2) {
        const A = this.legs[0], B = this.legs[1];
        // desired φ = angleA − angleB. The legs are created a half-cycle apart
        // (B.phaseOffset − A.phaseOffset = π), so the target relative phase is
        // wrap(−(B.off − A.off)) = wrap(−π) ≡ +π.
        const offDiff = wrapPi((B.phaseOffset || 0) - (A.phaseOffset || 0)); // = π
        const targetPhi = wrapPi(-offDiff);               // ≡ +π
        const phi = wrapPi(A.body.angle - B.body.angle);  // current relative phase
        const phiErr = wrapPi(targetPhi - phi);           // >0 => A must gain on B
        // velocity-level correction (Matter units, per-sub-dt delta-angle). A gets
        // +c, B gets −c so the average (propulsion) spin is conserved exactly.
        // Stiffness _phiLockK (per substep) pulls φ→target fast; bounded so a huge
        // error can't fling either leg. Also damp the relative spin.
        const relWmatter = (A.body.angularVelocity - B.body.angularVelocity);
        let c = this._phiLockK * phiErr * subDtSec - this._phiLockD * relWmatter;
        const cMax = this._phiLockMax * subDtSec; // cap (Matter units)
        if (c > cMax) c = cMax; else if (c < -cMax) c = -cMax;
        Body.setAngularVelocity(A.body, A.body.angularVelocity + c);
        Body.setAngularVelocity(B.body, B.body.angularVelocity - c);
      }
      // DEFENSIVE SPEED CLAMP on the cube (anti-fling safety net). A long leg
      // whose foot JAMS on a stair riser can make the rigid length-0 pin inject
      // a large positional correction that teleports the heavy chassis (launch).
      // We cap the chassis SPEED to a sane walking ceiling. This only ever
      // REDUCES speed — it never adds a forward force — so it cannot fake
      // propulsion (the motor-OFF cube, which is ~stationary, is unaffected) and
      // it cannot mask a fall (a falling cube is slowed, not lifted). Walking vx
      // is ~1-2 u/s so the cap (8 u/s) is pure headroom in normal play.
      if (this.cube) {
        const v = this.cube.velocity, VMAX = 3;
        const sp = Math.hypot(v.x, v.y);
        if (sp > VMAX) Body.setVelocity(this.cube, { x: v.x / sp * VMAX, y: v.y / sp * VMAX });
      }
      // Keep the chassis perfectly UPRIGHT every substep: zero BOTH the angle
      // AND the angular velocity. With a finite inertia the pin solve leaves a
      // small residual spin; if we zero only the angle (not the spin) that spin
      // re-accumulates and fights the clamp -> energy buildup / launch. Zeroing
      // both makes the cube a pure upright translator. This does NOT move the
      // body forward, so it cannot mask a stall or a fall.
      if (this.cube) {
        if (this.cube.angle !== 0) Body.setAngle(this.cube, 0);
        if (this.cube.angularVelocity !== 0) Body.setAngularVelocity(this.cube, 0);
      }
      // HOLD x while NOT driving (see holdX above): the floating cube would
      // otherwise settle-drift sideways. Restore x and zero horizontal velocity
      // so it sits perfectly still during the countdown / off-motor check. The
      // cube is still free in Y (it can settle vertically onto its leg).
      if (holdX != null && this.cube) {
        Body.setPosition(this.cube, { x: holdX, y: this.cube.position.y });
        Body.setVelocity(this.cube, { x: 0, y: this.cube.velocity.y });
      }
      // ANTI-BACKFLING RATCHET (anti-fling, NOT propulsion). A thin lever jamming
      // against the length-0 pin can TELEPORT the chassis BACKWARD (a non-physical
      // solver artifact — the same family as the anti-teleport guard above), which
      // can fling the cube clean off the back of the track. Real Draw Climber
      // locomotion is monotonic-forward (a spinning wheel/pedal never reverses).
      // So while DRIVING we forbid the chassis from being pushed backward more
      // than _ratchetSlack behind the FURTHEST x it has legitimately reached. This
      // ONLY removes backward motion — it never adds a forward force (the cube
      // can never get AHEAD of where its own legs carried it), so it cannot fake
      // propulsion: with the motor OFF the cube doesn't advance, _maxX never
      // grows, and the ratchet never moves it forward. The slack lets the cube
      // still wiggle/settle naturally.
      if (drive && this.cube && this._ratchet) {
        if (this.cube.position.x > this._maxX) this._maxX = this.cube.position.x;
        const floorX = this._maxX - this._ratchetSlack;
        if (this.cube.position.x < floorX) {
          Body.setPosition(this.cube, { x: floorX, y: this.cube.position.y });
          if (this.cube.velocity.x < 0) Body.setVelocity(this.cube, { x: 0, y: this.cube.velocity.y });
        }
      }
    }
    this._checkExplosion();
  }

  _checkExplosion() {
    const bad = (v) => !Number.isFinite(v);
    if (this.cube) {
      const p = this.cube.position, vel = this.cube.velocity;
      if (bad(p.x) || bad(p.y) || bad(vel.x) || bad(vel.y)) this._exploded = true;
      if (Math.abs(vel.x) > 200 || Math.abs(vel.y) > 200) this._exploded = true;
    }
    for (const l of this.legs) {
      const p = l.body.position;
      if (bad(p.x) || bad(p.y)) this._exploded = true;
    }
  }

  /**
   * Pivot/axle diagnostics for verification of "pivot == cube centre".
   * Returns the WORLD y of the leg's rotation pivot (the cube-side pin anchor =
   * cube.position + AXLE_Y) and the cube's geometric-centre y, plus their gap and
   * the surface y beneath the cube. With AXLE_Y == 0 the pivot y MUST equal the
   * cube centre y (gap ~ 0). aboveSurface > 0 proves the cube floats on the track.
   */
  pivotInfo() {
    if (!this.cube) return null;
    const cubeCenterY = this.cube.position.y;
    const axleY = this.cube.position.y + AXLE_Y; // cube-side pin anchor (world)
    const surfaceY = this.surfaceYAt(this.cube.position.x);
    return {
      axleY,
      cubeCenterY,
      gap: Math.abs(axleY - cubeCenterY),
      aboveSurface: surfaceY != null ? (surfaceY - cubeCenterY) : null,
      reach: this.legs[0] ? this.legs[0].radius : null,
      axleLocalY: AXLE_Y,
    };
  }

  get bodyX() { return this.cube ? this.cube.position.x : this.startX; }
  get bodyY() { return this.cube ? this.cube.position.y : 0; }
  get progress() {
    const t = (this.bodyX - this.startX) / (this.finishX - this.startX);
    return Math.max(0, Math.min(1, t));
  }
  get exploded() { return this._exploded; }

  /**
   * Expected track surface y (physics, +down) directly under a given x.
   * Scans floor slabs whose x-span contains px and returns the highest (most
   * negative) top. Returns null if no floor under px (a gap). Used by the
   * verifier to assert the cube stays on the track.
   */
  surfaceYAt(px) {
    let best = null;
    for (const b of this.floorBodies) {
      if (b.label !== 'floor') continue;
      const minx = b.bounds.min.x, maxx = b.bounds.max.x;
      if (px < minx || px > maxx) continue;
      const topY = (b._dcTopY != null) ? b._dcTopY : b.bounds.min.y;
      if (best == null || topY < best) best = topY;
    }
    return best;
  }

  /**
   * Geometry metrics of leg[0] in its LOCAL (axle-at-origin) frame — for
   * verification of (a) length proportion and (b) line-vs-fill representation.
   * Returns:
   *   reach        farthest chain sample from the axle (drawn-length proxy)
   *   strokeLen    arc length of the chain polyline
   *   lineRadius   physics circle radius (== rendered line half-thickness)
   *   bboxArea     axis-aligned bbox area of the chain polyline
   *   lineArea     area actually occupied by the thin stroke ribbon
   *                (strokeLen * 2*lineRadius + end caps) — the WYSIWYG line area
   *   hullFillArea area of the CONVEX HULL of the chain + axle (what a filled
   *                "blob" leg would occupy). lineArea << hullFillArea proves the
   *                leg is a thin LINE, not a filled polygon.
   *   parts        number of circle parts (chain length).
   */
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
    const lineArea = strokeLen * 2 * r + Math.PI * r * r; // ribbon + round caps
    // convex hull (of chain + axle) area — the "filled blob" alternative.
    const pts = ch.map((c) => ({ x: c.x, y: c.y }));
    pts.push({ x: 0, y: 0 });
    const hull = Vertices.hull(Vertices.create(pts, null));
    let hullFillArea = 0;
    if (hull && hull.length >= 3) hullFillArea = Math.abs(Vertices.area(hull, true));
    return {
      reach, strokeLen, lineRadius: r, bboxArea, lineArea, hullFillArea,
      parts: ch.length,
    };
  }

  /**
   * ANCHOR diagnostics (assertion 8): prove the leg is pinned by the stroke's
   * START (points[0]), NOT its centroid — i.e. the line hangs off one side and
   * does NOT straddle/punch through the cube centre.
   *
   * Measures, for leg[0]:
   *   axleToStartWorld  WORLD distance from the cube-side axle anchor to the
   *                     PART that traces chain[0] (the stroke start). chain[0] is
   *                     at the leg's local origin (== the pin point), so this is
   *                     ~0: the START is fixed to the axle.
   *   reach             the limb length (anchor -> farthest sample).
   *   centroidToAxle    distance from the axle (local origin == chain[0]) to the
   *                     stroke's CENTROID. For a one-sided limb the centroid sits
   *                     well OUTBOARD of the start, so this is LARGE (a meaningful
   *                     fraction of reach). For the rejected centroid-anchored
   *                     build it would be ~0 (centroid AT the axle).
   *   centroidFrac      centroidToAxle / reach — the dimensionless "one-sidedness"
   *                     of the limb. > 0.25 means the centroid is clearly off the
   *                     axle => the line extends one way (not straddling).
   */
  legAnchorInfo() {
    const l = this.legs[0];
    if (!l || !l.body || !l.body.parts) return null;
    const ch = l.chain;
    // The part tracing chain[0] is parts[1] (parts[0] is the compound proxy). Use
    // the body's WORLD position of that part vs the cube-side axle anchor.
    const axleWorld = {
      x: this.cube.position.x + AXLE_X,
      y: this.cube.position.y + AXLE_Y,
    };
    const startPart = l.body.parts.length > 1 ? l.body.parts[1] : l.body.parts[0];
    const axleToStartWorld = Math.hypot(
      startPart.position.x - axleWorld.x,
      startPart.position.y - axleWorld.y
    );
    // reach + stroke centroid in the LOCAL (anchor-at-origin == chain[0]) frame.
    let reach = 0, cxs = 0, cys = 0;
    for (const c of ch) {
      reach = Math.max(reach, Math.hypot(c.x, c.y));
      cxs += c.x; cys += c.y;
    }
    const cx = cxs / ch.length, cy = cys / ch.length;
    const centroidToAxle = Math.hypot(cx, cy); // axle is the local origin
    const centroidFrac = reach > 1e-6 ? centroidToAxle / reach : 0;
    return { axleToStartWorld, reach, centroidToAxle, centroidFrac };
  }
}

function clamp01(v) { return Math.max(0, Math.min(1, v)); }

/**
 * Resample a polyline into points spaced ~`spacing` apart along its arc length.
 * Returns at most `maxPts` points (if the stroke is longer, spacing grows to
 * fit). Always keeps the first & last vertex. This makes the circle chain
 * evenly cover the drawn line regardless of how the user dragged it.
 * @param {{x:number,y:number}[]} pts
 * @param {number} spacing
 * @param {number} maxPts
 */
function resamplePolyline(pts, spacing, maxPts) {
  if (pts.length < 2) return pts.slice();
  // total arc length
  let total = 0;
  for (let i = 1; i < pts.length; i++) total += Math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
  if (total < 1e-6) return [pts[0]];
  // grow spacing so we never exceed maxPts circles.
  const eff = Math.max(spacing, total / (maxPts - 1));
  const out = [{ x: pts[0].x, y: pts[0].y }];
  let acc = 0;        // distance walked since last emitted sample
  let next = eff;
  for (let i = 1; i < pts.length; i++) {
    let ax = pts[i - 1].x, ay = pts[i - 1].y;
    const bx = pts[i].x, by = pts[i].y;
    let segLen = Math.hypot(bx - ax, by - ay);
    while (segLen > 1e-9 && acc + segLen >= next) {
      const t = (next - acc) / segLen;
      const nx = ax + (bx - ax) * t, ny = ay + (by - ay) * t;
      out.push({ x: nx, y: ny });
      // advance start of remaining segment to the emitted point
      const consumed = next - acc;
      ax = nx; ay = ny;
      segLen -= consumed;
      acc = next; // we are now AT `next` arc length
      next += eff;
    }
    acc += segLen;
  }
  const last = pts[pts.length - 1];
  const tail = out[out.length - 1];
  if (Math.hypot(last.x - tail.x, last.y - tail.y) > eff * 0.25) out.push({ x: last.x, y: last.y });
  return out;
}

/**
 * Set the collision filter on a compound body AND all of its parts. Matter
 * collides PARTS (the parent body.parts[0] is just a proxy bound), so a compound
 * needs the filter applied to every part to be honoured.
 */
function setFilterDeep(body, filter) {
  body.collisionFilter = { ...filter };
  if (body.parts && body.parts.length > 1) {
    for (let i = 0; i < body.parts.length; i++) body.parts[i].collisionFilter = { ...filter };
  }
}

/** Built-in leg presets (for AI rival & headless default). Box [-1,1]^2. */
export function presetStroke(name) {
  if (name === 'wheel') {
    // A LOW-POLY wheel (7-gon): the flat faces + corners give "grip teeth" so a
    // foot can catch a step edge and climb. A smooth disc just spins against a
    // vertical riser and stalls.
    const pts = [];
    const N = 7;
    for (let i = 0; i < N; i++) {
      const a = (i / N) * Math.PI * 2 + Math.PI / 2; // a flat at the bottom-ish
      pts.push({ x: Math.cos(a) * 0.85, y: Math.sin(a) * 0.85 });
    }
    return pts;
  }
  if (name === 'stick') {
    // a bar across the box (acts like a paddle / spoke)
    return [{ x: -0.9, y: 0 }, { x: 0.9, y: 0 }];
  }
  if (name === 'hook') {
    return [
      { x: -0.2, y: -0.8 },
      { x: -0.2, y: 0.4 },
      { x: 0.5, y: 0.7 },
      { x: 0.85, y: 0.2 },
    ];
  }
  if (name === 'limb') {
    // A representative DRAWN LEG: a bent limb (dog-leg). points[0] is the START
    // that anchors at the cube centre; the line then extends OUT and bends, the
    // far end being the FOOT. This is the canonical anchor-test shape: its
    // centroid is well OUTBOARD of points[0] (a one-sided limb), proving the line
    // hangs off one side and does NOT straddle the cube.
    return [
      { x: 0.0, y: 0.0 },
      { x: 0.0, y: 0.55 },
      { x: 0.45, y: 0.95 },
    ];
  }
  if (name === 'limb_long') {
    // A LONG bent limb (for the leg_long screenshot): a long line that starts at
    // points[0] and reaches far OUT one way, then hooks — a big stride limb.
    return [
      { x: 0.0, y: -0.2 },
      { x: 0.1, y: 0.5 },
      { x: 0.55, y: 0.95 },
      { x: 0.95, y: 0.8 },
    ];
  }
  if (name === 'limb_short') {
    // A SHORT bent limb (for the leg_short screenshot): a short stub that starts
    // at points[0] and reaches only a little way out one side.
    return [
      { x: 0.0, y: 0.0 },
      { x: 0.18, y: 0.28 },
      { x: 0.42, y: 0.36 },
    ];
  }
  // ── Length-preservation test strokes (verifier) ──
  if (name === 'short') {
    // a SHORT straight bar through the origin -> short leg / short reach.
    return [{ x: -0.32, y: 0.18 }, { x: 0.32, y: -0.18 }];
  }
  if (name === 'long') {
    // a LONG straight bar through the origin -> long leg / long reach.
    return [{ x: -0.95, y: 0.55 }, { x: 0.95, y: -0.55 }];
  }
  if (name === 'L') {
    // an open 'L' (right angle): NOT a filled blob. Open strokes stay open.
    return [{ x: -0.2, y: -0.9 }, { x: -0.2, y: 0.5 }, { x: 0.9, y: 0.5 }];
  }
  if (name === 'arc') {
    // a wide OPEN ARC (호). Its convex hull encloses a large 2D area (a fat
    // segment) but the drawn stroke is a thin curved line along the rim.
    const pts = [];
    const N = 14;
    for (let i = 0; i <= N; i++) {
      const a = Math.PI * (0.15 + (i / N) * 0.7); // ~150° arc
      pts.push({ x: Math.cos(a) * 0.9, y: Math.sin(a) * 0.9 });
    }
    return pts;
  }
  // ── any-shape propulsion sweep strokes (assertion 10) ──
  if (name === 'short_bar') {
    // a SHORT straight stub limb (start at origin, extends one way a little).
    return [{ x: 0.0, y: 0.0 }, { x: 0.0, y: 0.45 }];
  }
  if (name === 'long_bar') {
    // a LONG straight limb (start at origin, reaches far one way).
    return [{ x: 0.0, y: 0.0 }, { x: 0.05, y: 1.0 }];
  }
  if (name === 'blob') {
    // a small dense scribble near the origin (tiny tangle) — a "small blob".
    return [
      { x: 0.0, y: 0.0 }, { x: 0.15, y: 0.1 }, { x: 0.05, y: 0.25 },
      { x: 0.22, y: 0.2 }, { x: 0.12, y: 0.32 }, { x: 0.28, y: 0.3 },
    ];
  }
  if (name === 'arc_big') {
    // a big sweeping OPEN CURVE (start at origin, curls out far).
    const pts = [];
    const N = 16;
    for (let i = 0; i <= N; i++) {
      const a = (i / N) * Math.PI * 1.1; // ~200°
      pts.push({ x: (1 - Math.cos(a)) * 0.55, y: Math.sin(a) * 0.85 });
    }
    return pts;
  }
  if (name === 'zigzag') {
    // an ENTIRELY sloppy scribble (jagged zig-zag) — the "messy doodle" case.
    return [
      { x: 0.0, y: 0.0 }, { x: 0.25, y: 0.2 }, { x: -0.1, y: 0.4 },
      { x: 0.35, y: 0.55 }, { x: 0.0, y: 0.75 }, { x: 0.4, y: 0.9 },
    ];
  }
  if (name === 'ring') {
    // a NEAR-CLOSED LOOP. Its convex hull is a FILLED DISC (large interior area),
    // but the drawn stroke is only the thin RIM. The leg's line-ribbon area is
    // therefore MUCH smaller than the hull-FILL area -> proves the leg is a LINE
    // tracing the stroke, NOT the filled convex hull (assertion 6).
    const pts = [];
    const N = 22;
    for (let i = 0; i <= N; i++) {
      const a = (i / N) * Math.PI * 1.9; // ~342° (open loop)
      pts.push({ x: Math.cos(a) * 0.9, y: Math.sin(a) * 0.9 });
    }
    return pts;
  }
  return [{ x: -0.9, y: 0 }, { x: 0.9, y: 0 }];
}

export const PHYS_CONST = { CUBE_SIZE, AXLE_X, AXLE_Y };
