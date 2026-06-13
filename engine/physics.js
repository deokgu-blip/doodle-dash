// engine/physics.js
// 2D side-view physics for Draw Climber (Matter.js).
//
// Mechanic (POC §4, L47):
//   - Body = a cube. On the bottom it has TWO axles (left/right).
//   - A "leg" = the user's drawn stroke -> the CONVEX HULL of the stroke points
//     (plus the axle center) given as a single SOLID convex body. This is the
//     key robustness fix: a solid convex disc/pie cannot tunnel through the
//     floor (no thin spokes -> no degenerate SAT normals) and cannot jam in a
//     thick slab (no gaps between spokes). Different strokes still produce
//     different wheel profiles -> the draw mechanic is preserved & data-driven.
//   - Each leg is pinned to its axle by a REVOLUTE pin constraint (length 0,
//     stiffness 1) so it can only spin about the axle.
//   - Motor = each step we set Body.setAngularVelocity(leg, omega). Friction
//     between the wheel polygon and the floor pushes the cube forward / climbs.
//   - Material props (friction / restitution) are set AFTER body creation as
//     properties (NOT in create options) — L47.
//
// Stability (root-cause fixes for the "fell through" bug, L24):
//   - Floor slabs are THICK (deep below the surface) so a wheel cannot cross a
//     full slab in one step.
//   - Engine.update is SUBSTEPPED (N small fixed sub-dts per public step) so the
//     wheel advances in small increments -> penetration per substep stays small.
//   - Motor angular speed is bounded so the wheel surface speed (and therefore
//     per-substep penetration) stays well under the slab thickness.
//   - The cube chassis also collides with the floor (safety belly) but is light
//     on friction so the grippy wheels do the driving.
//
// All physics is deterministic given a fixed timestep -> headless verifiable.

import Matter from './vendor/matter.module.js';
import { SEGMENT_DEFAULTS } from './track_schema.js';

const { Engine, Composite, Bodies, Body, Bodies: _B, Constraint, Vector, Vertices } = Matter;

// Collision categories so legs/body don't fight each other, only the floor.
const CAT_FLOOR = 0x0001;
const CAT_BODY  = 0x0002;
const CAT_LEG   = 0x0004;

const CUBE_SIZE = 1.2;          // world units (player cube edge)
const AXLE_INSET = 0.55;        // axle x-offset from cube center (wide wheelbase
// resists the torque-reaction wheelie: the two wheels' contact patches straddle
// the chassis CoM so motor torque pitches it far less than a narrow base).
// Axles sit just below the cube's bottom face. The wheel hangs below the axle
// by its radius; the cube bottom rests close to the wheel tops so the chassis
// is carried by the wheels with a small safety belly. Unlike the old design we
// do NOT float the chassis far above the floor — a solid disc wheel is stable
// enough to support the body directly, and a near-floor belly is a real safety
// net (no free-fall if a wheel briefly lifts).
const AXLE_Y = CUBE_SIZE * 0.5 + 0.10; // axle just below cube bottom face

// Slab thickness BELOW the surface. Deep enough that a wheel cannot cross the
// whole slab in one substep (per-substep travel is small, see SUBSTEPS).
const FLOOR_THICK = 4.0;

export class Physics {
  constructor() {
    this.engine = Engine.create();
    this.engine.gravity.y = 1;           // Matter convention: +y is down
    this.engine.gravity.scale = 0.001;   // Matter standard (with 16.666ms step)
    // Higher iterations stabilize the wheel-vs-floor contact and the revolute
    // pin (prevents the heavy cube from dragging the wheel through).
    this.engine.positionIterations = 12;
    this.engine.velocityIterations = 12;
    this.engine.constraintIterations = 6;
    this.world = this.engine.world;
    // Public fixed step is 1/60s for determinism (headless == runtime, L24).
    // Internally we SUBSTEP this into SUBSTEPS smaller integrations so the fast
    // wheel never advances far enough to tunnel through a slab in one go.
    this.FIXED_DT = 1000 / 60;
    this.SUBSTEPS = 4;                 // 4 substeps -> sub-dt = 1/240s
    this.SUB_DT = this.FIXED_DT / this.SUBSTEPS;

    this.cube = null;
    this.legs = [];        // [{ body, pin, side, radius }]
    this.legConstraints = [];
    this.motorSpeed = 0;   // rad/s target angular velocity magnitude
    this._motorSign = 1;   // +1 drives toward +x (forward) with our geometry
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
      b.collisionFilter = { category: CAT_FLOOR, mask: CAT_BODY | CAT_LEG, group: 0 };
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
        b.collisionFilter = { category: CAT_FLOOR, mask: CAT_BODY | CAT_LEG, group: 0 };
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
        b.collisionFilter = { category: CAT_FLOOR, mask: CAT_BODY | CAT_LEG, group: 0 };
        b.label = 'floor';
        Composite.add(this.world, b);
        this.floorBodies.push(b);
      }
    }

    // Player cube — placed in setLegStroke (depends on wheel radius). Create it
    // here so legs can pin to it; setLegStroke repositions it on the surface.
    const cx = track.startX;
    const cy = groundY - CUBE_SIZE / 2 - 0.05;
    const cube = Bodies.rectangle(cx, cy, CUBE_SIZE, CUBE_SIZE);
    // Car-style chassis: the cube collides with the floor (safety belly so it
    // can never fall through) but is light on friction so the grippy wheels do
    // the driving. Mass is low relative to the wheels so the wheels keep enough
    // normal force to bite.
    cube.friction = 0.05;
    cube.frictionStatic = 0.05;
    cube.frictionAir = 0.01;
    cube.restitution = 0;
    cube.collisionFilter = { category: CAT_BODY, mask: CAT_FLOOR, group: 0 };
    cube.label = 'cube';
    Body.setMass(cube, 4);
    // LOCK chassis rotation (infinite rotational inertia). In Draw Climber the
    // cube stays upright and only translates while the legs spin under it. A
    // free-rotating chassis on pin-jointed driven wheels is an unstable inverted
    // system that flips/launches no matter how the torque is tuned (verified
    // repeatedly). Locking rotation removes that whole failure mode and matches
    // the game's look. The wheels still spin freely about their pins.
    Body.setInertia(cube, Infinity);
    Composite.add(this.world, cube);
    this.cube = cube;

    this.surfaceY = surfaceY;
  }

  /**
   * Set / replace the leg from a normalized stroke (box [-1,1]^2).
   * Builds ONE solid convex body per axle (the convex hull of the stroke points
   * plus the axle center) and pins it with a revolute constraint.
   * @param {{x:number,y:number}[]} points  normalized polyline
   * @param {{thickness?:number, scale?:number, motorSpeed?:number}} [spec]
   */
  setLegStroke(points, spec = {}) {
    if (!this.cube) return;
    const scale = spec.scale ?? 1.0;        // [-1,1] box -> world radius
    // Motor angular speed (rad/s). Bounded below so the wheel SURFACE speed
    // (omega * radius) per substep stays small relative to FLOOR_THICK -> no
    // tunneling. With radius ~1 and SUB_DT 1/240, omega=9 gives ~0.037 u/substep.
    this.motorSpeed = spec.motorSpeed ?? 9;

    // remove previous legs
    for (const c of this.legConstraints) Composite.remove(this.world, c);
    for (const l of this.legs) Composite.remove(this.world, l.body);
    this.legs = [];
    this.legConstraints = [];

    if (!points || points.length < 2) { this.legDrawn = false; return; }

    // World-space stroke points (about the axle origin). Include the axle
    // center so even an open stroke yields a closed convex pie shape.
    const world = points.map((p) => ({ x: p.x * scale, y: p.y * scale }));
    world.push({ x: 0, y: 0 });

    // Convex hull -> solid convex polygon. A degenerate (collinear) hull falls
    // back to a small disc so locomotion still works.
    let hull = Vertices.hull(Vertices.create(world, null));
    if (!hull || hull.length < 3) {
      // collinear stroke (e.g. a flat stick): give it a thin lens by adding a
      // perpendicular offset so the hull has area.
      const a = world[0], b = world[world.length - 2] || world[0];
      const dx = b.x - a.x, dy = b.y - a.y;
      const len = Math.hypot(dx, dy) || 1;
      const nx = -dy / len * 0.12, ny = dx / len * 0.12;
      const padded = [];
      for (const p of world) padded.push({ x: p.x + nx, y: p.y + ny }, { x: p.x - nx, y: p.y - ny });
      hull = Vertices.hull(Vertices.create(padded, null));
    }

    // Wheel radius = farthest hull vertex from the axle center (used to lift the
    // cube so the wheel bottom rests on the surface, not penetrated).
    let radius = 0;
    for (const v of hull) radius = Math.max(radius, Math.hypot(v.x, v.y));
    if (radius < 1e-3) { this.legDrawn = false; return; }

    // Place the AXLE one radius above the surface so the wheel bottom sits on
    // the surface; +0.02 keeps it from starting penetrated. The cube center is
    // AXLE_Y above the axle.
    const startSurfaceY = 0; // first segment surface (groundY)
    const desiredAxleY = startSurfaceY - radius + 0.02;
    const desiredCubeY = desiredAxleY - AXLE_Y;
    Body.setPosition(this.cube, { x: this.startX, y: desiredCubeY });
    Body.setVelocity(this.cube, { x: 0, y: 0 });
    Body.setAngularVelocity(this.cube, 0);
    Body.setAngle(this.cube, 0);

    for (const side of [-1, 1]) {
      const axleX = this.cube.position.x + side * AXLE_INSET;
      const axleY = this.cube.position.y + AXLE_Y;

      // Solid convex wheel centered at the axle. fromVertices builds a convex
      // body (single part) from the hull; position is the axle.
      const verts = hull.map((v) => ({ x: v.x, y: v.y }));
      const leg = Bodies.fromVertices(axleX, axleY, [verts], {}, true);
      if (!leg) { continue; }
      // Recenter so the wheel pivots about the axle (fromVertices centers on
      // the centroid; shift the body so its centroid sits at the axle, then the
      // pin at the axle gives clean rotation).
      Body.setPosition(leg, { x: axleX, y: axleY });

      // L47: material AFTER creation. Grippy wheel; heavy so it carries the
      // body's weight into the contact (traction instead of slip).
      leg.friction = 1.0;
      leg.frictionStatic = 2.0;
      leg.restitution = 0;
      leg.frictionAir = 0;
      leg.collisionFilter = { category: CAT_LEG, mask: CAT_FLOOR, group: 0 };
      leg.label = 'leg';
      Body.setMass(leg, 1.5);

      Composite.add(this.world, leg);

      const pin = Constraint.create({
        bodyA: this.cube,
        pointA: { x: side * AXLE_INSET, y: AXLE_Y },
        bodyB: leg,
        pointB: { x: 0, y: 0 }, // leg centroid == axle (we recentered)
        length: 0,
        stiffness: 1,
        damping: 0.1,
      });
      Composite.add(this.world, pin);

      this.legs.push({ body: leg, pin, side, radius });
      this.legConstraints.push(pin);
    }
    this.legDrawn = this.legs.length > 0;
  }

  /**
   * Drive the motor + integrate one fixed step (FIXED_DT) via SUBSTEPS smaller
   * sub-integrations. Deterministic. dtMs is accepted for API symmetry but we
   * always step a fixed slice; the caller decides how many public steps to run.
   */
  update(_dtMs, running) {
    // Motor model: a GOVERNED TORQUE drive (not a kinematic setAngularVelocity).
    // Forcing angular velocity makes a blocked wheel merely spin in place (pure
    // slip) — it can never climb a step because the contact force doesn't grow.
    // Instead we apply torque toward a target spin: on flat ground the wheel
    // reaches the target, but when it stalls against a step the speed error
    // grows so MAX torque is applied, pressing the wheel up the step edge via
    // friction. This is what actually lets a round wheel climb a riser.
    //
    // Target angular velocity in Matter's per-timestep convention at SUB_DT.
    // Hybrid "car" drive (robust = stable AND climbs):
    //  (1) STABLE forced spin: setAngularVelocity turns the wheels at a constant
    //      rate. This never pumps energy into the contact, so the sim can't
    //      launch (a torque governor backflips/catapults the chassis — verified).
    //      The spin gives the rolling look and rolling-contact friction.
    //  (2) FORWARD DRIVE FORCE on the chassis, applied only while running and
    //      only when a wheel is actually touching the floor. This is the engine
    //      power that pushes the cube up a step (a forced-spin wheel alone just
    //      slips against a riser and stalls — verified). The force is bounded and
    //      only acts forward, so it cannot mask a fall (gravity still pulls the
    //      cube down a gap) and is fully deterministic.
    const omegaStep = this._motorSign * (this.motorSpeed * this.SUB_DT) / 1000;
    const grounded = this._wheelsGrounded();
    // Forward drive force (Matter force units). Applied at the AXLE height (below
    // the CoM) so it both pushes forward and lifts the front over a step instead
    // of just shoving the chassis into the riser. Bounded so it can't launch.
    const driving = running && this.legDrawn && grounded;
    // STRONG drive force so it reliably mounts a step riser, but the forward
    // SPEED is capped (VXMAX below) so the strong force converts into climbing
    // torque at an obstacle without accelerating the cube to a launch speed on
    // the flats. Force climbs; speed cap keeps the roll believable & on-track.
    const DRIVE = 0.020; // forward force magnitude while grounded
    for (let s = 0; s < this.SUBSTEPS; s++) {
      if (running && this.legDrawn) {
        for (const l of this.legs) Body.setAngularVelocity(l.body, omegaStep);
        if (driving) {
          // Forward drive (chassis rotation is locked, so apply through the CoM).
          // This is the engine power that walks the cube up a step; the spinning
          // wheels provide the contact/traction and the rolling visual.
          const fwd = this._motorSign;
          Body.applyForce(this.cube, this.cube.position, { x: fwd * DRIVE, y: 0 });
        }
      } else {
        for (const l of this.legs) Body.setAngularVelocity(l.body, 0);
      }
      Engine.update(this.engine, this.SUB_DT);
      // The chassis rotation is locked (infinite inertia, set at build), so no
      // tilt/flip handling is needed. Cap forward speed to a sane roll (so the
      // strong drive force climbs rather than launches) and keep a vertical
      // envelope as a numerical-spike safety. Neither cap moves the body
      // forward on its own, so they can't mask a stall or a fall.
      if (this.cube) {
        if (this.cube.angle !== 0) Body.setAngle(this.cube, 0); // stay perfectly upright
        const v = this.cube.velocity;
        const VXMAX = 0.10;  // forward speed cap (~6 u/s) — believable roll
        const VYMAX = 0.45;  // vertical safety envelope
        let vx = v.x, vy = v.y;
        if (vx > VXMAX) vx = VXMAX; else if (vx < -VXMAX) vx = -VXMAX;
        if (vy > VYMAX) vy = VYMAX; else if (vy < -VYMAX) vy = -VYMAX;
        if (vx !== v.x || vy !== v.y) Body.setVelocity(this.cube, { x: vx, y: vy });
      }
    }
    this._checkExplosion();
  }

  /** True if at least one wheel is resting on (or just above) the floor below
   * it — gates the forward drive so the cube can't "swim" through the air or
   * up out of a gap. */
  _wheelsGrounded() {
    for (const l of this.legs) {
      const wb = l.body.bounds.max.y;       // wheel bottom (physics +down)
      const surf = this.surfaceYAt(l.body.position.x);
      if (surf == null) continue;            // over a gap
      if (wb >= surf - 0.25) return true;    // bottom at/below surface = contact
    }
    return false;
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
}

function clamp01(v) { return Math.max(0, Math.min(1, v)); }

/** Built-in leg presets (for AI rival & headless default). Box [-1,1]^2. */
export function presetStroke(name) {
  if (name === 'wheel') {
    // A LOW-POLY wheel (7-gon), not a smooth circle: the flat faces + corners
    // give the wheel "grip teeth" so it can catch a step edge and climb it. A
    // smooth disc just spins against a vertical riser and stalls (verified).
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
  return [{ x: -0.9, y: 0 }, { x: 0.9, y: 0 }];
}

export const PHYS_CONST = { CUBE_SIZE, AXLE_INSET, AXLE_Y };
