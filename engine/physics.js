// engine/physics.js
// 2D side-view physics for Draw Climber (Matter.js) — LEG-DRIVEN locomotion.
//
// Mechanic (POC §4 — REDESIGNED per user reject):
//   - Body = an upright cube. At the bottom-CENTER it has ONE horizontal axle
//     (perpendicular to the travel direction). The axle carries TWO legs
//     (left/right). In the 2D side-view plane both legs pin to the SAME point;
//     the renderer offsets them in z so they read as two legs straddling the
//     cube. They are spun 180° OUT OF PHASE so one foot plants while the other
//     lifts -> the cube WALKS (alternating gait), it does not slide.
//   - A "leg" = the user's drawn stroke -> the CONVEX HULL of the stroke points
//     (plus the axle center) as a single SOLID convex body, pinned to the axle
//     by a revolute (length-0) constraint so it can only spin about the axle. A
//     solid polygon has real rotational inertia and is numerically STABLE; a
//     thin one-end-pinned limb whips and launches the cube (verified). The hull
//     of a drawn line is a thin paddle whose far corner is a FOOT.
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

const CUBE_SIZE = 1.2;              // world units (player cube edge)
// Single axle at the bottom-CENTER of the cube (x offset = 0). It sits just
// below the cube's bottom face; the legs hang below the axle by their radius.
const AXLE_X = 0.0;
const AXLE_Y = CUBE_SIZE * 0.5 + 0.06; // axle just below the cube bottom face

// Slab thickness BELOW the surface. Deep enough that a foot cannot cross the
// whole slab in one substep (per-substep travel is small, see SUBSTEPS).
const FLOOR_THICK = 4.0;

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
    // Motor tuning (exposed for headless sweeps). Strong torque so a lever-leg
    // can rotate while bearing the cube's weight; cap keeps a stall from
    // launching. Units are Matter torque (applied as angular accel * dt²).
    // Constant motor torque (Matter torque units) + angular-speed ceiling
    // (motorSpeed, rad/s). Tuned in headless sweeps for a genuine, VISIBLE
    // leg-driven walk: enough torque that a foot mounts a stair riser (a weak
    // foot stalls AT the step — that's the real difficulty curve), with a
    // moderate ceiling so the legs turn at a watchable ~2 rev/s, not a blur.
    this._motorTorque = 0.012;
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
    // The cube collides with the floor ONLY as a deep-fall safety net. In normal
    // walking the cube rides ~ (legRadius + AXLE_Y) ABOVE the surface so the
    // belly does NOT touch and does NOT carry the weight — the legs bear the load
    // and have full normal force for traction. The belly catches the cube only
    // if BOTH feet leave the ground (so it can't free-fall through the world),
    // and because it is frictionless it adds NO forward force when it does.
    cube.collisionFilter = { category: CAT_BODY, mask: CAT_FLOOR, group: 0 };
    cube.label = 'cube';
    Body.setMass(cube, 2.2);
    // LOCK chassis rotation (infinite rotational inertia). In Draw Climber the
    // cube stays upright and only translates while the legs spin under it. A
    // free-rotating chassis on pin-jointed driven legs is an unstable inverted
    // system that flips/launches. Locking rotation removes that failure mode and
    // matches the game's look. The legs still spin freely about their pins, and
    // the cube is still free to translate in x and y from the feet pushing.
    Body.setInertia(cube, Infinity);
    Composite.add(this.world, cube);
    this.cube = cube;

    this.surfaceY = surfaceY;
  }

  /**
   * Set / replace the legs from a normalized stroke (box [-1,1]^2).
   * Builds TWO solid convex bodies pinned to the SINGLE bottom-center axle, spun
   * 180° out of phase (alternating gait). The legs share a negative collision
   * group so they never collide with each other, only the floor.
   * @param {{x:number,y:number}[]} points  normalized polyline
   * @param {{thickness?:number, scale?:number, motorSpeed?:number}} [spec]
   */
  setLegStroke(points, spec = {}) {
    if (!this.cube) return;
    const scale = spec.scale ?? 1.0;        // [-1,1] box -> world radius
    // Motor angular-speed CEILING (rad/s). The constant motor torque pushes the
    // foot until it reaches this ceiling, then backs off — high enough that the
    // foot keeps slipping/biting the ground (continuous propulsion) instead of
    // capping early and free-spinning to a stop, but bounded so it can't launch.
    this.motorSpeed = spec.motorSpeed ?? 14;

    // remove previous legs
    for (const c of this.legConstraints) Composite.remove(this.world, c);
    for (const l of this.legs) Composite.remove(this.world, l.body);
    this.legs = [];
    this.legConstraints = [];

    if (!points || points.length < 2) { this.legDrawn = false; return; }

    // ── Build the leg as a SOLID CONVEX foot (the convex hull of the stroke +
    // the axle center). A solid polygon has real rotational inertia, so it is
    // numerically STABLE (a thin one-end-pinned limb whips and launches the
    // cube — verified). The hull of a DRAWN LINE is a thin paddle/triangle whose
    // far corner is a FOOT; spun about the axle that foot plants and sweeps the
    // ground backward -> friction WALKS the cube. The leg shape (so the gait)
    // still varies with the drawing -> data-driven mechanic preserved. ──
    const world = points.map((p) => ({ x: p.x * scale, y: p.y * scale }));
    world.push({ x: 0, y: 0 }); // include the axle center -> closed shape

    let hull = Vertices.hull(Vertices.create(world, null));
    if (!hull || hull.length < 3) {
      // collinear stroke (e.g. a flat bar): give it a thin lens so it has area.
      const a = world[0], b = world[world.length - 2] || world[0];
      const dx = b.x - a.x, dy = b.y - a.y;
      const len = Math.hypot(dx, dy) || 1;
      const nx = -dy / len * 0.12, ny = dx / len * 0.12;
      const padded = [];
      for (const p of world) padded.push({ x: p.x + nx, y: p.y + ny }, { x: p.x - nx, y: p.y - ny });
      hull = Vertices.hull(Vertices.create(padded, null));
    }

    // farthest hull vertex from the axle (for placing the cube on the surface).
    let radius = 0;
    for (const v of hull) radius = Math.max(radius, Math.hypot(v.x, v.y));
    if (!hull || hull.length < 3 || radius < 1e-3) { this.legDrawn = false; return; }

    // Place the AXLE one radius above the surface so a foot bottom sits on the
    // surface; +0.02 keeps it from starting penetrated. Cube center is AXLE_Y
    // above the axle.
    const startSurfaceY = 0; // first segment surface (groundY)
    const desiredAxleY = startSurfaceY - radius + 0.02;
    const desiredCubeY = desiredAxleY - AXLE_Y;
    Body.setPosition(this.cube, { x: this.startX, y: desiredCubeY });
    Body.setVelocity(this.cube, { x: 0, y: 0 });
    Body.setAngularVelocity(this.cube, 0);
    Body.setAngle(this.cube, 0);

    // Two legs share ONE axle (x = AXLE_X). They are spun 180° out of phase:
    // phase 0 and phase PI. side is for the renderer's z offset only.
    const defs = [
      { side: -1, phase: 0 },
      { side: +1, phase: Math.PI },
    ];
    for (const def of defs) {
      const axleX = this.cube.position.x + AXLE_X;
      const axleY = this.cube.position.y + AXLE_Y;

      // Solid convex foot from the hull, centered at the axle.
      const verts = hull.map((v) => ({ x: v.x, y: v.y }));
      const leg = Bodies.fromVertices(axleX, axleY, [verts], {}, true);
      if (!leg) { continue; }
      Body.setPosition(leg, { x: axleX, y: axleY });
      // Stagger the starting rotation by the phase so the two feet alternate.
      Body.setAngle(leg, def.phase);

      // L47: material AFTER creation. Grippy foot; heavy so it carries the
      // body's weight into the contact (traction instead of slip).
      leg.friction = 1.6;
      leg.frictionStatic = 2.0;
      leg.restitution = 0;
      leg.frictionAir = 0;
      // Legs collide with the FLOOR only, and never with each other (shared
      // negative group overrides category/mask between the two legs).
      leg.collisionFilter = { category: CAT_LEG, mask: CAT_FLOOR, group: GROUP_LEGS };
      leg.label = 'leg';
      Body.setMass(leg, 1.4);

      Composite.add(this.world, leg);

      const pin = Constraint.create({
        bodyA: this.cube,
        pointA: { x: AXLE_X, y: AXLE_Y },
        bodyB: leg,
        pointB: { x: 0, y: 0 }, // leg centroid == axle (recentered)
        length: 0,
        stiffness: 1,
        damping: 0.1,
      });
      Composite.add(this.world, pin);

      this.legs.push({ body: leg, pin, side: def.side, radius, phase: def.phase });
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
    // Governed CONSTANT torque (open-loop) with an angular-speed ceiling. A
    // constant torque keeps the foot pushing the ground backward every instant,
    // so propulsion is CONTINUOUS (a PD-to-target torque drops to ~0 once at
    // speed and the wheel then coasts/slips to a stop — verified). The ceiling
    // (targetW) stops the spin running away into a launch: while |w| < ceiling
    // apply the motor torque; once past it, stop adding torque. The torque is
    // applied to the LEG only — the cube moves solely via the foot's friction.
    const targetW = this._motorSign * (this.motorSpeed * this.SUB_DT) / 1000;
    const torque = this._motorSign * this._motorTorque;

    for (let s = 0; s < this.SUBSTEPS; s++) {
      if (drive) {
        for (const l of this.legs) {
          const w = l.body.angularVelocity;
          // only push while below the ceiling (in the drive direction).
          if (this._motorSign > 0 ? (w < targetW) : (w > targetW)) {
            l.body.torque += torque;
          }
        }
      }
      // No motor (countdown / win / lose / motor-off): legs are FREE (no torque
      // applied) — gravity & contact settle them and the cube does not advance.
      Engine.update(this.engine, this.SUB_DT);
      // Keep the chassis perfectly upright (inertia is locked, but clamp the
      // angle defensively against constraint drift). This does NOT move the body
      // forward, so it cannot mask a stall or a fall.
      if (this.cube && this.cube.angle !== 0) Body.setAngle(this.cube, 0);
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
  return [{ x: -0.9, y: 0 }, { x: 0.9, y: 0 }];
}

export const PHYS_CONST = { CUBE_SIZE, AXLE_X, AXLE_Y };
