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

import { SEGMENT_DEFAULTS, segTurnDeg } from './track_schema.js';

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
  // ── FORGIVING MOMENTUM (관성) — cube speed is an integrated state easing toward the motor
  // target (the leg/terrain-derived speed). Ramps from rest + builds over continuous running;
  // drag bleeds it; capped (no runaway). momAccel > momDrag ⇒ speeds up faster than it coasts down.
  momAccel: 5.0,         // 1/s — peak propulsion ease rate toward the motor target (inertia build-up). Modulated
                         // by the planted-foot stance (pushStanceFloor..1) so the push ripples with the gait but
                         // never gaps — continuous alternating-foot drive, not a steady force or a stutter.
  pushStanceFloor: 0.4,  // min propulsion fraction when no foot is well-planted (the hand-off moment). 0.4 ⇒ the
                         // push clearly RIPPLES with each footfall (gait feel) but never stutters/dies to a stop.
  momDrag: 2.0,          // 1/s — ease rate toward a LOWER target (coast-down / drag when the motor eases)
  momMaxV: 11.0,         // world-u/s hard cap on the cube's forward speed (forgiving — no runaway)
  // JAM (끼임-정지): when a leg is physically caught on geometry (too-long leg hits a tunnel ceiling,
  // or a step too tall to climb), the motor STALLS — the leg holds against the obstacle instead of
  // free-rolling through. A small backward recoil bounces the cube on the impact (뒤로 살짝).
  jamMotorStop: 22.0,    // 1/s — how fast the leg spin decays to a stop when jammed (motor stall ease)
  jamRecoil: 1.2,        // world-u/s — small backward bounce velocity on the jam-impact frame (then eases to rest)
  ceilBackStep: 0.04,    // rad — step size for unwinding the leg spin off a tunnel ceiling (the jam clamp); small ⇒ tight contact, no penetration
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
  // ── HOOK (shape) RULE — the STEEP-RAMP gate is by SHAPE, not length ──
  // A gated STEEP UPHILL climb can only be grip-and-stepped (짚고) over by a leg whose
  // drawn shape is a GENUINE HOOK / ㄱ / J / L / claw — i.e. ONE clear directional bend
  // that can CATCH on a step edge. A messy zigzag / scribble must NOT qualify (you can't
  // grip a step with a wiggly line). So "hookiness" is TWO conditions, both required:
  //   (1) a SHARP enough bend somewhere — maxTurnAngle >= hookAngleDeg, AND
  //   (2) the bend is DIRECTIONALLY COHERENT (curls ONE way — a clean ㄱ/J/L), NOT a
  //       back-and-forth scribble. We measure the SIGNED turn (cross product) at each SHARP
  //       vertex and count DIRECTION REVERSALS (sign flips). A genuine hook turns the SAME
  //       way throughout (0 reversals); a zigzag alternates left-right-left (>=2 reversals).
  //       So a HOOK requires signReversals <= hookMaxReversals. This is LENGTH-ROBUST (a
  //       SHORT clean ㄱ still has ~0 reversals — unlike a sharp-fraction test that penalizes
  //       short strokes), so a short hook IS a hook and a long scribble is NOT.
  //       HAND-DRAW FORGIVING: a phone-drawn hook wobbles, so we only count REVERSALS among
  //       corners sharper than `sharpAngleDeg` (40°, above finger-jitter) and tolerate up to
  //       `hookMaxReversals` (2) honest wobble bends. A true scribble has MANY large (>40°)
  //       alternating bends ⇒ rev=3 ⇒ still rejected. (Reproduced/tuned in _repro_hook.mjs.)
  // Measured on the normalized leg chain with a small lookahead window (so resampling
  // micro-jitter on a smooth arc/circle does not fake a corner). reach is IRRELEVANT — a
  // long straight leg is NOT a hook, a short clean ㄱ IS a hook, a long scribble is NOT.
  //   isHook = (maxTurnAngle >= hookAngleDeg) && (signReversals <= hookMaxReversals)
  hookAngleDeg: 70,      // degrees — a clear corner this sharp is needed (genuine hooks≈75–78°; non-hooks: wheel 50, limb_long 52, arc_big 44 ⇒ all below)
  // FORGIVING SCRIBBLE TEST (hand-draw tuned): a phone-drawn hook has finger WOBBLE — small
  // perpendicular jitter on the shaft. At a LOW sharp threshold (old 20°) that jitter fakes
  // many tiny "sharp corners" that flip direction, so a clean ㄱ measured rev=3–5 and was
  // WRONGLY rejected (the live-test regression). Reproduction (scripts/_repro_hook.mjs) with
  // 0.05 finger-noise: every genuine hook stays at rev≤1 once micro-turns under 40° are
  // ignored, while a true scribble/zigzag keeps rev=3 (many LARGE alternating bends). So we
  // (a) raise the sharp-corner threshold to 40° (finger jitter is < this; only genuine bends
  // count for the direction-reversal test) and (b) tolerate up to 2 reversals (a couple of
  // honest wobble bends are forgiven). Sweep: sharp=40/maxRev=2 ⇒ 36/36 hooks PASS, 9/9
  // scribbles REJECTED (clean separation). maxRev=3 would let scribbles through ⇒ stays 2.
  sharpAngleDeg: 40,     // degrees — a chain vertex must turn at LEAST this much to count as a "sharp" corner whose TURN DIRECTION we track (above finger-jitter; only genuine bends count for the scribble/reversal test)
  hookMaxReversals: 2,   // max direction reversals among sharp corners for a HOOK (clean/wobbly ㄱ/J/L ⇒ rev≤1; a real scribble alternates left-right-left across many big bends ⇒ rev=3 ⇒ FAILS — can't grip with a wiggle)
  hookTurnWindow: 2,     // chain-sample lookahead each side for the turn-angle measure (smooths arc jitter, keeps sharp corners)
  // ── ICE RULE (아이젠/crampon gate — a NEW gimmick, the inverse-of-smooth) ──
  // An ICE stretch is SLIPPERY: a smooth leg (wheel/arc/straight/limb) and even a HOOK find no
  // grip and slip in place (struggle, legs spin like wheels on ice, no advance). ONLY a TOOTHED
  // leg bites in: a MANY-corner zigzag (아이젠처럼 뾰족) — nSharp sharp corners that ALTERNATE
  // (signReversals). So the player must DRAW A SPIKY LEG to cross (forces a distinct redraw from
  // long/short/hook). Thresholds chosen so zigzag/scribble PASS (nSharp~4-5, rev~2-3) while every
  // smooth shape (nSharp 0) and a clean hook (nSharp 1, rev≤1) are BLOCKED — clean separation.
  iceGripSharpMin: 3,    // min sharp corners (teeth) for grip — a wheel/arc/straight=0, hook=1 ⇒ blocked; a zigzag/scribble≥3 ⇒ grips
  iceGripRevMin: 2,      // min direction reversals among those teeth (a true zigzag alternates; a coherent curl does not) ⇒ excludes a multi-bend-but-one-way curl
  iceEnterGap: 0.0,      // world-u — stop a non-grippy leg this far before the ice foot (0 = right at the slippery edge)
  // STEEP-RAMP GATE: an UPHILL ramp (physics slope<0) is "steep" (hook-gated) iff its
  // |slope| >= steepThresh. Gentle/medium uphills (|slope| < this) are NOT gated — any
  // leg climbs them (unchanged). GAP ramps are EXEMPT (a short leg must escape the
  // V-trench — never gated, no soft-lock). The downhill ramps in T01 are |slope|≈0.5–0.58
  // and are NEVER gated (downhill is not a climb); only steep UPHILLs gate.
  steepThresh: 0.9,      // |physics-slope| at/above which an UPHILL climb is HOOK-gated (was 0.6). Raised so
                         // the new STEEPER staircases (slope ~0.8) stay LENGTH-gated — a long leg STEP-climbs
                         // them tread-by-tread (the new physics) instead of forcing a hook. Only very steep
                         // (~50°+, slope>=0.9) climbs are hook-gated now.
  steepEnterGap: 0.0,    // world-u — stop a non-hook leg this far before the steep ramp foot (0 = right at the foot)
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
  // ── PLANT "쿵" SQUASH (땅을 딱딱 짚는 손맛) — COSMETIC weight accent, render-only ──
  // The walker is kinematic, so a foot-plant had no IMPACT — the body just smoothly bobbed
  // ("너무 가볍다"). We add a SQUASH-AND-STRETCH on the CUBE MESH synced to the foot-PLANT phase
  // (cos(2θ) peaks at θ≈0,π — a leg straight down = a footfall, twice per rotation = each leg's
  // plant): the cube COMPRESSES (squashes flatter) the instant a foot lands and springs back
  // between plants, so every step reads as a weighted "쿵". Pure render scale on cubeMesh — it
  // does NOT move the physics body / foot / gates (zero penetration & all gates untouched). Faded
  // in with forward speed so an idle cube doesn't sit squashed. Applied to player + rival alike.
  plantSquash: 0.17,      // peak vertical compression fraction at a footfall (0.17 ⇒ cube 17% flatter, x/z widen to compensate) — firm, reference-like
  plantSquashPow: 1.7,    // sharpen the pulse so the squash is a CRISP impact at the plant (higher ⇒ briefer, snappier "쿵"), not a constant throb
  plantSquashSpeedRef: 2.0, // forward speed (u/s) at which the squash fades fully IN (slower ⇒ less; idle ⇒ 0, no frozen squash)
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
  strideStretch: 0.4,    // max extra rolling-radius factor at full loft. LOWERED 1.6→0.4: at 1.6 the legs rolled only ~0.38× mid-float, so on low lofts (body barely up, looks grounded) the leg looked FROZEN while sliding forward. 0.4 ⇒ ω stays ≥~0.71× of v/r everywhere ⇒ legs always VISIBLY roll (no frozen-slide). no-slip is unaffected (at a foot-plant hop=0 ⇒ stretch=1 ⇒ ω=v/r regardless).
  loftLerp: 16.0,        // 1/s — how fast the live loft eases toward its phase target (smooth, no pop at slope onset)
  airTiltLerp: 7.0,      // 1/s — body eases its lean a touch faster during the hovering float (nose follows the gentle arc)
  landMergeLerp: 14.0,   // 1/s — residual touchdown re-settle ease (kept for cosmetic continuity)
  // reach factor floor: even the shortest leg still runs and lofts a little (never 0).
  loftReachFloor: 0.55,  // min reach-factor (so short legs hop less, but are not frozen-grounded)

  // ── STEEP-STAIR GRIP CADENCE (짚고 올라가기 — the hook PLANTS on each tread & pulls up) ──
  // On a HOOK-gated STEEP staircase the body would otherwise glide smoothly up the stepped
  // surface, which reads as a float/auto-climb. To make it READ as the hook GRIPPING each
  // step and pulling the body up tread-by-tread, we add a small PHASE-LOCKED UPWARD pulse on
  // top of the grounded pose, peaking at each foot-PLANT (the moment the hook catches an
  // edge). It is UPWARD-ONLY (it only RAISES the body — subtracts from physics-y), so it can
  // NEVER push the body into a riser ⇒ structurally penetration-free. The amplitude is small
  // and eased in/out so there is no jitter; it is the "tasteful middle ground" cadence layered
  // on the smooth climb (NOT a hard teleport-per-step that would jitter). Active ONLY while on
  // a steep-gated stair run with a hook actually driving the climb.
  gripLiftMax: 0.16,     // world-u — peak per-plant upward hitch on a steep-stair grip climb (small ⇒ no motion sickness)
  gripLerp: 12.0,        // 1/s — how fast the grip-lift eases in/out at the run boundaries (smooth, no pop)
  // ── FIX ㉡ — STEPPED grip climb (계단을 한 칸씩 짚고 올라가기, NOT a smooth slide up the hypotenuse) ──
  // On a HOOK-gated STEEP staircase the body must ascend in DISCRETE TREAD STEPS synchronized
  // with the leg's PLANT, instead of riding the smooth stepped-grounded glide (which read as a
  // slide). We hold the body at a tread level, then STEP it up to the NEXT tread the instant the
  // hook PLANTS (the phase peak), then hold, step, hold — a visible stair-climbing cadence. The
  // step-up is EASED over a short time (no harsh jitter) but the STEP rhythm stays clearly
  // visible (it is NOT smoothed back into a continuous glide). It is anchored to the REAL tread
  // tops (a discrete level the cube has reached), and clamped to the grounded pose, so the foot is
  // ALWAYS on/above the current tread — never dips into a riser ⇒ structurally penetration-free.
  stepClimbEnable: true, // turn the stepped (vs glide) climb on for steep-gated stair runs
  stepRiseLerp: 22.0,    // 1/s — how fast the body/camera EASES up to the next committed tread after a plant (higher ⇒ a snappier, more visible step; τ≈0.045s ⇒ the rise completes in ~a few frames, then HOLDS until the next plant)
  // The camera base is normally slew-limited (surfaceSlewMax 0.04/frame) so it GLIDES — that is
  // exactly what made the steep climb read as a smooth slide. On a stepped steep climb we relax the
  // camera slew so it can RISE a tread quickly right after a plant, then HOLD until the next plant
  // (the visible "한 칸씩" cadence on screen). Still bounded (no instant snap ⇒ no nausea).
  stepCamSlewMax: 0.3,   // world-u/frame cap on the camera base during a stepped climb (≈ a full tread over ~3 frames ⇒ a quick, eased, clearly-visible step UP, then a flat HOLD until the next plant — the cadence reads as "한 칸씩", not a glide)
  stepCamLerp: 40.0,     // 1/s — the camera-base ease rate during a stepped climb (faster than surfaceLerp so the step REACHES the tread quickly, leaving a flat HOLD before the next plant; still eased ⇒ no instant snap)
  stepPlantPhaseCos: 0.5,// the plant is detected when cos(2θ) rises above this (θ≈0,π ⇒ a foot straight down = a plant); a hysteresis edge advances the tread level once per plant

  // ── FIX ㉣ — GRIP-CLIMB LEG GAIT (the leg DWELLS at the plant, then REACHES — NOT a constant roll) ──
  // The previous fixes stepped the BODY (camera base) but the LEG kept rotating at the no-slip constant
  // rate (ω = v/r), so on screen the leg looked like a WHEEL rolling up — the user's "그냥 미끄러지면서
  // 올라가버려" (it slides). The genuine grip-climb the user wants is a HOLD–REACH–HOLD leg rhythm: the
  // foot PLANTS on a tread edge → the leg HOLDS there (dwell) while the body pulls up → the leg quickly
  // REACHES to the next tread edge and plants again. We get this by PHASE-WARPING the leg's θ advance
  // during a steep-hook climb: θ advances SLOWLY through the plant phase (a visible dwell, foot down at
  // the tread edge) and QUICKLY through the swing/reach phase. CRITICAL: the warp's MEAN over a full
  // half-stride is EXACTLY 1, so the leg's AVERAGE angular speed equals the no-slip value — the climb
  // advances at the SAME pace, never stalls (the dwell is a visual redistribution of the SAME rotation,
  // not a brake). At a plant the instantaneous ω is still ≈ v/r (no foot slip on contact). Active ONLY
  // on a steep-gated hook stair climb; everywhere else warp≡1 (normal rolling gait UNCHANGED).
  gripGaitEnable: true,   // turn the dwell-reach leg phase-warp on for steep-gated stair hook climbs
  gripDwell: 0.78,        // 0..~0.95 — DWELL strength. The phase rate near a plant is multiplied DOWN by ~(1−gripDwell) and the reach phase sped UP to compensate (mean stays 1). 0.78 ⇒ the leg nearly HOLDS at the plant (≈0.22× rate) then snaps through the reach — a clear "짚고" beat. 0 = constant roll (off).
  gripGaitLerp: 11.0,     // 1/s — ease the warp depth in/out at the climb boundaries (no pop entering/leaving the steep stair)
  // ── LEG LENGTH → REACH (treads-per-plant): a LONGER leg grabs FARTHER/MORE treads per plant ──
  // The user's idea: "다리 길이에 따라 몇 번째 계단을 밟는다는 설정". A long leg REACHES over several treads
  // and plants on a higher edge (covers more treads per stride); a short leg plants on the immediate
  // next tread (one at a time). We map reach → treadsPerPlant: at each foot-PLANT the committed body
  // tread level jumps UP by up to `treadsPerPlant` real tread edges (clamped to the actual staircase, so
  // it never overshoots the top ⇒ still penetration-free, still reaches the top reliably). The leg's
  // SLOWER ω (a longer lever ⇒ lower v/r) already spaces its plants farther apart, so grabbing more
  // treads per plant keeps the body in sync with the foot (no float). Structurally O(1) per plant.
  gripTreadsShort: 1,     // treads a SHORT leg (reach≈MIN) grabs per plant (one at a time)
  gripTreadsLong: 3,      // treads a LONG leg (reach≈MAX) grabs per plant (a big reaching stride over several steps)

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

  // ── BALL-FIELD (a pile of dynamic physics spheres that BLOCKS the cube) ──
  // A `balls` segment lays N light physics balls on a FLAT stretch. They are NOT real
  // rigid bodies — each is a single sphere with pos/vel under gravity, clamped onto the
  // track surface (surfaceYAt), separated from its neighbours, and PUSHED by the cube.
  // The cube does NOT auto-stop on them (no soft-lock): plowing through the pile costs
  // SPEED (a resistance multiplier ∝ how many balls it is shoving), so the cube visibly
  // slows in the pile and speeds back up once clear. The leg推進 (drive) is untouched —
  // only an extra deceleration is added, so no-slip / no-penetration / every gate stay
  // intact (the ball system is purely additive). All knobs are designer-tunable here.
  ballGravity: 26.0,     // world u/s² downward (physics +down) — light, settles fast
  ballFriction: 5.0,     // 1/s linear velocity damping (rolling/ground friction) — settles balls
  ballRestitution: 0.18, // 0..1 bounce on ground / separation (a little lively, mostly damped)
  ballSepStiff: 22.0,    // 1/s ball-ball separation push rate (positional, impulse-like)
  // USER FIX (억지 밀림) — the SHOVE was over-tuned (1.9× + 0.55 lift) so the balls LAUNCHED
  // away unnaturally ("너무 억지로 밀려"). The push is now GENTLE & natural: the cube nudges balls
  // forward at ~1× its own speed with only a little lift, so they ROLL/scatter believably rather
  // than fly off. The PLOW DRAG (방해) is UNCHANGED — it comes from the contact count (ballCubeHalf
  // width + ballContactPad + the per-contact slow below), NOT from the push velocity — so the
  // "헤쳐나가야" feel and the race gates are preserved while the motion reads soft.
  ballPushSpeed: 1.35,   // (was 1.9) ~1.35× the cube speed imparted forward — clears balls out of the path (so the cube doesn't wallow / lose the race) WITHOUT the violent fling of 1.9
  ballPushUp: 0.35,      // (was 0.55) the 억지 fix: balls roll up & OVER the cube edge in a gentle pop (launch velocity ~halved) instead of LEAPING into the air — still lifts them clear of the contact band so the cube doesn't wallow
  ballPushPosFrac: 0.85, // fraction of the overlap ejected POSITIONALLY per frame — softens the snap-out a touch while still clearing the ball ahead (no lingering/wallow)
  ballCubeHalf: 0.7,     // WIDER push circle ⇒ the cube contacts MORE of the pile at once (drives the DRAG/방해, kept)
  // RESISTANCE: each ball the cube is in contact with multiplies its speed DOWN. The
  // factor is (1 − ballSlowPerContact)^contacts, clamped at ballSlowMin so the cube
  // NEVER fully stops (no soft-lock) — it always grinds through. FIX ㉢: STRENGTHENED so
  // the thick of the pile drops the cube to ~0.4–0.55 of clear speed (obviously struggling),
  // floored so it always breaks through and recovers after. More contacts ⇒ more drag
  // (the slowdown ramps with the contact count, compounding per contact).
  ballSlowPerContact: 0.42, // (was 0.20) each contacting ball costs 42% of speed (compounding) — a STRONG, obvious "방해"
  ballSlowMin: 0.34,        // (was 0.30) hard floor on the slow factor (cube keeps ≥34% pace in the thick of it ⇒ obviously struggling but never soft-locks)
  ballContactPad: 0.22,     // (was 0.10) extra world-u so a ball "in contact" is counted a hair earlier (more of the heap drags)

  // ── BREAKING BLOCKS (a wall of standing boxes the cube SMASHES into debris) ──
  // A `blocks` segment stands N boxes UPRIGHT on a flat run, barring the path. While INTACT
  // a block is STATIC and STOPS the cube at its face (no auto-advance, no penetration). On
  // contact the block BREAKS: it is replaced by `debrisPerBlock` small box fragments that
  // INHERIT the cube's forward push (an outward impulse), then fall + litter the floor using
  // the SAME debris physics as the balls (gravity, ground clamp, fragment-fragment &
  // fragment-cube separation, linear friction). The settled rubble then SLOWS any cube that
  // grinds over it (resistance ∝ contacts, floored ⇒ never a soft-lock). The cube ALWAYS
  // smashes through. These knobs mirror the ball knobs (debris IS a ball with box render).
  blockBreakSpeedMin: 0.6,  // min forward cube speed to register a "smash" (so a creeping touch still breaks but reads as effort)
  debrisGravity: 26.0,      // world u/s² downward (physics +down) — same as the balls
  debrisFriction: 5.6,      // (was 5.2) a touch higher linear damping so the chips tumble to rest near the wall (don't drift far downstream into the run-out, so the cube cleanly recovers speed after the window)
  debrisRestitution: 0.12,  // ground/separation bounce (boxy chips bounce less than balls)
  debrisSepStiff: 26.0,     // (was 22) stronger fragment-fragment separation so chips SPREAD OUT flat on the floor (litter), not pile up vertically (keeps the settled rubble bottom on the floor)
  // USER FIX (억지 밀림) — the smash burst was over-tuned (4.6) so chips flew off too violently
  // ("큐브가 너무 억지로 밀려"). Softened to a believable break: chips scatter a bit and litter the
  // floor, the cube nudges them gently. The rubble DRAG (방해) is UNCHANGED (contact count: cubeHalf
  // width + contactPad + per-contact slow), so the smash-through effort and race gates are kept.
  debrisBurstSpeed: 2.8,    // (was 4.6) base outward speed (u/s) on a block break — a believable "흩날림", not an explosion
  debrisBurstUp: 0.3,       // (was 0.34) low lift ⇒ chips litter FLAT on the floor (not stacked)
  debrisPushSpeed: 0.5,     // (was 0.9) a soft later nudge — the cube leaves the rubble behind near the wall and recovers cleanly. Drag is from CONTACTS, not from carrying the pile.
  debrisPushUp: 0.18,       // (was 0.3) barely any up on the later shove ⇒ chips slide/spread along the floor, don't pile
  debrisPushPosFrac: 0.6,   // fraction of overlap ejected POSITIONALLY per frame — eases chips out (no snap-out) while staying no-clip
  debrisCubeHalf: 0.7,      // WIDER push circle ⇒ the cube contacts MORE rubble at once (drives the DRAG/방해, kept)
  debrisContactPad: 0.24,   // (was 0.10) count a fragment "in contact" a hair earlier (more of the rubble drags)
  // FIX ㉢ — STRONGER rubble drag so plowing the debris is CLEARLY felt (~0.4–0.55 of clear),
  // floored so it never soft-locks; ramps with the contact count (compounding per fragment).
  debrisSlowPerContact: 0.34, // (was 0.16) each contacting fragment costs 34% of speed (compounding) — a strong "방해"
  debrisSlowMin: 0.38,      // (was 0.34) hard floor on the rubble slow factor (cube keeps ≥38% pace ⇒ obviously struggling but never soft-locks)
  // BLOCK SMASH GATE: while a block is intact and the cube has not reached its face, the
  // cube is held at the face (struggle-in-place is NOT used — a block is broken on the very
  // frame the cube touches it, so the hold is a single sub-step before the break). The cube
  // stops at (blockFaceX − cubeHalf) until it breaks. blockContactPad = how close (world-u)
  // the cube must get to a block's near face to trigger the break.
  blockContactPad: 0.06,
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
    this._isHook = false;            // SHAPE gate: true ⇒ the drawn leg is a HOOK (sharp bend) — can grip-climb a steep ramp
    this._isGrippy = false;          // SHAPE gate: true ⇒ a TOOTHED/zigzag (아이젠) leg — grips ICE; smooth/hook legs slip
    this._maxTurnDeg = 0;            // measured max turn angle (deg) of the current chain (hookiness metric)
    this._signReversals = 0;         // measured # of sharp-corner direction reversals (zigzag/teeth metric)
    this._nSharp = 0;                // measured # of sharp corners (teeth count) — drives the ice grip gate
    this._blockedBySteep = false;    // blocked specifically by a steep ramp (non-hook leg) — legs keep trying (struggle in place)
    this._blockedByIce = false;      // blocked specifically by ICE (non-grippy/smooth leg slips) — legs keep trying (spin)
    this._legPhaseOffset = Math.PI;  // second leg is 180° out of phase
    this._blocked = false;           // true when stopped at an unclimbable step
    this._blockedByRiser = false;    // §C: blocked specifically by a riser (climb) — legs keep trying
    this._blockedByTunnel = false;   // blocked specifically by a low ceiling (too-long leg) — legs keep trying
    this._trying = false;            // §C: true while struggling in place (legs churn, x≈0)
    this._vx = 0;                    // last realized forward speed (u/s)
    this._v = 0;                     // FORGIVING MOMENTUM: integrated forward speed (eases toward the motor target)
    this._wasJammed = false;         // JAM tracker: true last frame the leg was caught on geometry (for the recoil impulse)
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
    this._grip = 0;                  // live STEEP-STAIR grip-cadence lift (world u, upward-only) — the per-plant hitch on a hook climb (짚고)
    this._gripLiveAmp = 0;           // eased grip-lift amplitude (0 off steep stairs ⇒ a clean smooth ramp/flat; ramps in on a steep hook climb)
    // ── FIX ㉡ — STEPPED grip-climb state (the body holds a tread, then steps up at each plant) ──
    this._stepClimbActive = false;   // true while doing the stepped (vs glide) climb on a steep-gated stair run
    this._stepCommitY = null;        // physics-y of the tread top the body is currently STANDING on (advances one tread per plant)
    this._stepLevelY = null;         // eased body-stand level toward _stepCommitY (the visible hold-then-step-up motion)
    this._stepPlantArmed = true;     // plant-edge hysteresis: armed between plants, fires once per plant to advance the tread
    this._stepProfile = 0;           // last stepped-vs-grounded lift applied (diagnostic for the verifier)
    // ── FIX ㉣ — GRIP-CLIMB leg gait (dwell-reach phase warp) state ──
    this._gripGaitLive = 0;          // eased warp DEPTH (0 off a steep-hook climb ⇒ constant roll; ramps to gripDwell on the climb)
    this._gripWarp = 1;              // last applied phase-rate warp (1 = no warp; <1 dwell at plant, >1 reach between) — diagnostic for the verifier
    this._gripDwelling = false;      // true on a frame the leg is in the DWELL (plant-hold) part of the climb gait — diagnostic

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

    // ── BALL-FIELD (dynamic physics-ball piles) ──
    // Each `balls` segment spawns N light spheres onto a flat run. We keep them in flat
    // arrays for cache-friendly stepping (no per-ball object churn) plus a small `_ballR`
    // array of radii. The renderer reads `balls` (an array of {x,y,r} views rebuilt
    // each frame from these arrays — see ballRenderList) to place its reused sphere
    // meshes. `_ballResist` is the last-applied resistance slow-factor (1 = no pile) the
    // verifier reads to prove the cube slows in a pile. All allocated ONCE per build.
    this._ballN = 0;                 // number of balls in the field (0 ⇒ none)
    this._ballX = null;              // Float64Array — ball x (world)
    this._ballY = null;              // Float64Array — ball y (physics +down; surface clamp keeps it on track)
    this._ballVX = null;            // Float64Array — ball x velocity
    this._ballVY = null;            // Float64Array — ball y velocity (+down)
    this._ballR = null;              // Float64Array — ball radius
    this._ballResist = 1;            // last cube speed slow-factor from the pile (1 = clear)
    this._ballContacts = 0;          // # balls the cube was in contact with last step (diagnostic)
    this._ballRenderList = null;     // reusable [{x,y,r}] views for the renderer (no per-frame alloc)

    // ── BREAKING BLOCKS (standing boxes the cube smashes) + their DEBRIS (boxy balls). ──
    // STANDING BLOCKS: a small array of upright boxes (x,faceX,topY,baseY,w,h, broken). While
    // a block is NOT broken it is STATIC and bars the cube (held at its face, no penetration);
    // on contact `broken` flips true and the block's render mesh is hidden. DEBRIS: flat
    // arrays exactly like the balls (x,y,vx,vy,r) + an `active` flag (a fragment is inert until
    // its block breaks). Debris physics IS the ball physics (gravity/clamp/separation/cube
    // push) with a box render. `_blockResist` is the cube slow-factor from the rubble it is
    // grinding over. All allocated ONCE in buildTrack; update() only mutates values.
    this._blockN = 0;                // number of standing blocks (0 ⇒ none)
    this._blocks = null;             // [{ x,faceX,topY,baseY,w,h, broken, segX0,segX1 }] standing blocks (render reads via blocksRenderList)
    this._brokenCount = 0;           // # blocks broken so far (diagnostic)
    this._debN = 0;                  // number of debris fragments (0 ⇒ none)
    this._debX = null;               // Float64Array — fragment x
    this._debY = null;               // Float64Array — fragment y (+down)
    this._debVX = null;              // Float64Array — fragment x velocity
    this._debVY = null;              // Float64Array — fragment y velocity (+down)
    this._debR = null;               // Float64Array — fragment half-size (collision radius)
    this._debActive = null;         // Uint8Array — 1 once its block has broken (inert before)
    this._debResist = 1;             // last cube speed slow-factor from the rubble (1 = clear)
    this._debContacts = 0;           // # debris the cube was in contact with last step (diagnostic)
    this._debRenderList = null;      // reusable [{x,y,r,active}] views for the renderer (no per-frame alloc)
    this._blockRenderList = null;    // reusable [{x,topY,baseY,w,h,broken}] views for standing blocks

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
    this._steepHit = { found: false, x: 0, slope: 0 };
    this._iceHit = { found: false, x: 0 };
    this._steepStairRuns = [];   // build-time list of steep-gated stairs run feet/windows

    // ── SPLIT-PATH FORK (a BRANCH the LEG SHAPE routes — commit-at-entrance) ──
    // At a `fork` the track splits into a HIGH route (hook-gated steep staircase up → flat top
    // → staircase down) and a LOW route (a shallow underpass valley) that REJOIN at the same x
    // and base y. Because x is forward-only and the terrain is a SINGLE-VALUED height function,
    // we do NOT carry two simultaneous physics surfaces: we COMMIT the route ONCE when the cube
    // first crosses the fork's x0 (HOOK ⇒ high, anything else ⇒ low), then the active surface for
    // the whole fork == the committed route until the rejoin. This is a BRANCH, NOT a gate — both
    // routes always reach the rejoin (no soft-lock). Each fork's geometry is built ONCE in
    // buildTrack: the LOW route lives in the MAIN _segs (so the default O(log n) lookup +
    // lookahead probe is always valid and always the always-passable road), and the HIGH route
    // lives in its own indexed seg list inside the fork record. surfaceYAt/_segAt/etc. swap to the
    // HIGH segs ONLY for x inside a fork that is COMMITTED high. Before commit (lookahead probes)
    // the LOW route is sampled, so the steep gate is never falsely tripped before the cube enters.
    this._forks = [];            // [{ id, x0, x1, baseY, highSegs:[…], highX0:Float64Array, highRuns:[…] }]
    this._forkRoute = null;      // Map forkId → 'high' | 'low' (committed at entrance; null until built)
    this._activeForkHint = -1;   // last fork index the body x fell inside (O(1) commit/lookup cache)

    // ── RENDER INTERPOLATION (mobile micro-stutter fix) ──
    // The SIM ticks on a FIXED timestep (FIXED_DT) with an accumulator that carries the
    // sub-tick remainder. On a real-device rAF the inter-frame interval jitters around
    // 16.67ms, so the accumulator drains 0 ticks on some frames (cube frozen) and 2 on
    // the next (cube double-jumps) — the "지지직" the user sees even at a steady 60fps.
    // We FIX it by RENDERING an INTERPOLATED pose between the PREVIOUS tick's state and
    // the CURRENT tick's state, weighted by alpha = _acc/FIXED_DT (the un-drained
    // remainder). The SIM itself is UNTOUCHED (still fixed-step, still deterministic) —
    // only the drawn pose is a lerp, so the on-screen motion is continuous regardless of
    // rAF jitter. `_interpPrev` snapshots the render-relevant state RIGHT BEFORE each
    // tick (captureInterp), and `_interpHasPrev` guards the first frame (prev==curr).
    // The leg angles are the only per-leg interp targets (position == the axle == cube
    // x/y, so legs reuse the cube's interpolated x/y).
    this._interpPrev = { x: 0, bodyY: 0, bodyBaseY: 0, angle: 0, theta: 0 };
    this._interpHasPrev = false;
  }

  /** RENDER INTERPOLATION: snapshot the render-relevant CURRENT state into `_interpPrev`
   * RIGHT BEFORE a sim tick advances it. The renderer then lerps prev→curr by the
   * leftover-accumulator alpha so the drawn pose is continuous under rAF jitter. The
   * snapshot is the cube forward x, the FULL body y (with bob/loft), the bob-free camera
   * base y, the body tilt and the master leg phase θ — every quantity the renderer reads
   * to place the cube + legs + camera. Pure copy (no allocation), zero sim effect. */
  captureInterp() {
    const p = this._interpPrev;
    p.x = this._x;
    p.bodyY = this._bodyY;
    p.bodyBaseY = Number.isFinite(this._bodyBaseY) ? this._bodyBaseY : this._bodyY;
    p.angle = this._angle;
    p.theta = this._theta;
    this._interpHasPrev = true;
  }

  /** Mark the prev-snapshot stale (e.g. after a teleport / fresh placement / track
   * rebuild) so the next render uses curr directly (no lerp from an unrelated pose). */
  resetInterp() { this._interpHasPrev = false; }

  reset() {
    this.cube = null;
    this.legs = [];
    this.floorBodies = [];
    this.ceilingBodies = [];
    this.iceBodies = [];            // ICE slippery render slabs ({ x0,x1, surfaceY }) — overlaid pale-cyan
    this.legDrawn = false;
    this._exploded = false;
    this._segs = [];
    this._segX0 = null;
    this._segHint = 0;
    this.turnRegions = [];          // PATH HEADING: per-segment arc-length turn spans (render-only)
    this._blocked = false;
    this._blockedByRiser = false;
    this._blockedByTunnel = false;
    this._blockedBySteep = false;
    this._blockedByIce = false;
    this._isHook = false;
    this._isGrippy = false;
    this._maxTurnDeg = 0;
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
    this._stepClimbActive = false;
    this._stepCommitY = null;
    this._stepLevelY = null;
    this._stepPlantArmed = true;
    this._stepProfile = 0;
    this._gripGaitLive = 0;
    this._gripWarp = 1;
    this._gripDwelling = false;
    this._idleFloat = false;
    this._idlePhase = 0;
    // BALL-FIELD: cleared here and rebuilt in buildTrack (the segment scan spawns them).
    this._ballN = 0;
    this._ballX = null; this._ballY = null; this._ballVX = null; this._ballVY = null; this._ballR = null;
    this._ballResist = 1; this._ballContacts = 0; this._ballRenderList = null;
    // BREAKING BLOCKS + DEBRIS: cleared here and rebuilt in buildTrack.
    this._blockN = 0; this._blocks = null; this._brokenCount = 0; this._blockRenderList = null;
    this._debN = 0;
    this._debX = null; this._debY = null; this._debVX = null; this._debVY = null; this._debR = null;
    this._debActive = null; this._debResist = 1; this._debContacts = 0; this._debRenderList = null;
    this._blockSpecs = null;
    // SPLIT-PATH FORK: cleared here and rebuilt in buildTrack. _forkRoute (the committed
    // route per fork) is reset to a fresh Map so a restart re-chooses the road at the entrance.
    this._forks = [];
    this._forkRoute = new Map();
    this._activeForkHint = -1;
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

    // PATH HEADING: collect each segment's arc-length span [x0,x1] + its `turn` (degrees)
    // so the renderer can build a heading-based curving path. Physics is UNCHANGED — these
    // regions only describe the centre-line's world-XZ shape (render-only). Cleared in
    // reset(); populated here as we advance the cursor per segment.
    this.turnRegions = [];
    for (const seg of track.segments) {
      const len = seg.length;
      const _turnX0 = cursorX;          // arc-length at this segment's start
      const _turnDeg = segTurnDeg(seg); // heading change (degrees) carried by this segment
      if (seg.kind === 'flat') {
        addSlab(cursorX + len / 2, surfaceY, len, thick);
        addFlatSeg(cursorX, cursorX + len, surfaceY, 'flat');
        cursorX += len;
      } else if (seg.kind === 'curve') {
        // CURVE = a bend in the PATH HEADING. Physically it is a flat run (or a gentle
        // slope when `height` is given) — the walker's 1-D arc-length model is UNCHANGED;
        // the turn is purely the centre-line's world-XZ shape (render-only, applied by the
        // renderer's heading path). So build it exactly like a flat (or a ramp if sloped).
        const dy = -(seg.height ?? 0);     // up = negative y (0 ⇒ flat curve)
        const x0 = cursorX, x1 = cursorX + len;
        if (dy === 0) {
          // CAPTURE the level in a per-segment CONST — `surfaceY` is a mutating loop var;
          // a closure over it would read the FINAL track level (closure bug). Freeze it.
          const curveY = surfaceY;
          addSlab(cursorX + len / 2, curveY, len, thick);
          this._segs.push({ x0, x1, kind: 'curve', topYa: curveY, topYb: curveY,
            surfFn: () => curveY });
          if (curveY < this._maxSurfaceTopY) this._maxSurfaceTopY = curveY;
        } else {
          const topY0 = surfaceY, topY1 = surfaceY + dy;
          addRampSlab(x0, x1, topY0, topY1, thick);
          const slope = dy / len;
          this._segs.push({ x0, x1, kind: 'curve', topYa: topY0, topYb: topY1, slope,
            surfFn: (px) => topY0 + slope * (px - x0) });
          if (topY1 < this._maxSurfaceTopY) this._maxSurfaceTopY = topY1;
          surfaceY += dy;
        }
        cursorX += len;
      } else if (seg.kind === 'stairs') {
        const steps = seg.steps;
        const stepLen = len / steps;
        const stepH = seg.height / steps;   // per-step rise (designed climb unit)
        // STEEP-STAIRCASE GATE (hook by SHAPE): a stairs run whose OVERALL slope (the
        // total rise over the run length) is >= steepThresh is a STEEP CLIMB. Its tall
        // step edges are GRIP POINTS — only a HOOK-shaped leg can grip & climb over; a
        // straight / round leg slips and STRUGGLES in place at the foot (no advance)
        // until a hook is redrawn. We tag EVERY tread of such a run with steepGate so
        // _nextSteep/_nextRiser key off it (the hook gate then SUPERSEDES the per-riser
        // length gate). GENTLE staircases (overall slope < steepThresh) get NO steepGate
        // and stay length-gated/passable by all legs, exactly as before. Computed ONCE
        // here at build time (no per-frame cost). Sign: stairs always rise (height>0),
        // so the run climbs up — |overall slope| = height/length.
        const stairSlope = seg.height / len;             // overall run slope (rise/run, positive = up)
        const stairSteep = Math.abs(stairSlope) >= TUNE.steepThresh;
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
          // walker must mount, carrying stepH for the climb rule. On a STEEP run the
          // tread also carries steepGate:true (the hook gate) — the run's foot is then
          // a steep-staircase block for a non-hook leg.
          const x0 = cursorX, x1 = cursorX + stepLen;
          this._segs.push({ x0, x1, kind: 'stairs', topYa: treadY, topYb: treadY,
            stepH, steepGate: stairSteep, surfFn: () => treadY });
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
        // STEEP-RAMP GATE (hook by SHAPE): a regular (non-gap) UPHILL ramp (physics
        // slope<0 ⇒ going up) is "steep" iff |slope| >= steepThresh. A steep ramp can
        // only be grip-and-stepped over by a HOOK-shaped leg; a non-hook leg STRUGGLES
        // in place at its foot (struggle, no advance — redraw a hook to climb). Gentle
        // uphills and ALL downhills are NEVER gated. GAP ramps (built below) never set
        // this flag (a short leg must escape the V-trench — no soft-lock).
        const steepGate = (slope < 0) && (Math.abs(slope) >= TUNE.steepThresh);
        this._segs.push({ x0, x1, kind: 'ramp', topYa: topY0, topYb: topY1, slope, steepGate,
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
      } else if (seg.kind === 'planks') {
        // PLANKS = a run of short flat BOARDS separated by small VOID GAPS the walker
        // must STRIDE across (the reference: a row of disconnected boards). Each board
        // is a FLAT segment (rendered as a closed box); each gap between boards is a
        // small RECOVERY V-TRENCH (a steep descent ramp + steep ascent ramp, tagged
        // gap:true so it is EXEMPT from the steep-ramp hook gate and reuses the proven
        // gap escape machinery — a short leg drops in & crawls out, NO soft-lock; a
        // long/fast leg's bigger stride LOFTS over the void). The gap ramps are ALSO
        // tagged plankGap:true so the RENDERER omits them from the surface ribbon (the
        // boards read as SEPARATE closed boxes with the green void showing between),
        // while PHYSICS keeps the recovery floor (surfFn) so a fallen foot is caught.
        // The board top stays at `surfaceY` throughout (a level run of boards).
        const count = Math.max(2, seg.count | 0);
        const plankLen = (seg.plankLen != null) ? seg.plankLen : SEGMENT_DEFAULTS.plankLen;
        const gapLen = (seg.gapLen != null) ? seg.gapLen : SEGMENT_DEFAULTS.gapLen;
        const gapDepth = (seg.gapDepth != null) ? seg.gapDepth : SEGMENT_DEFAULTS.plankGapDepth;
        const boardTopY = surfaceY;                     // every board sits at the run level
        const boardThick = thick;                        // closed-box board slab depth
        for (let p = 0; p < count; p++) {
          // BOARD: a flat surface segment + its own closed-box render slab.
          const bx0 = cursorX, bx1 = cursorX + plankLen;
          addSlab(bx0 + plankLen / 2, boardTopY, plankLen, boardThick);
          this._segs.push({ x0: bx0, x1: bx1, kind: 'planks', plank: true,
            topYa: boardTopY, topYb: boardTopY, surfFn: () => boardTopY });
          if (boardTopY < this._maxSurfaceTopY) this._maxSurfaceTopY = boardTopY;
          cursorX += plankLen;
          // GAP (between boards only — count-1 of them): a shallow recovery V-trench
          // with NO render slab (the void shows). Built as a descent+ascent ramp pair
          // sharing the gap escape machinery (gap:true ⇒ steep-gate exempt, no soft-lock).
          if (p < count - 1) {
            const halfW = gapLen / 2;
            const dx0 = cursorX, dx1 = cursorX + halfW;
            const dTopY0 = boardTopY, dTopY1 = boardTopY + gapDepth; // +down ⇒ deeper
            const dSlope = gapDepth / halfW;
            this._segs.push({ x0: dx0, x1: dx1, kind: 'ramp', gap: true, plankGap: true,
              topYa: dTopY0, topYb: dTopY1, slope: dSlope,
              surfFn: (px) => dTopY0 + dSlope * (px - dx0) });
            if (dTopY1 < this._maxSurfaceTopY) this._maxSurfaceTopY = dTopY1;
            const ax0 = cursorX + halfW, ax1 = cursorX + gapLen;
            const aTopY0 = boardTopY + gapDepth, aTopY1 = boardTopY;
            const aSlope = -gapDepth / halfW;
            this._segs.push({ x0: ax0, x1: ax1, kind: 'ramp', gap: true, plankGap: true,
              topYa: aTopY0, topYb: aTopY1, slope: aSlope,
              surfFn: (px) => aTopY0 + aSlope * (px - ax0) });
            cursorX += gapLen;
          }
        }
        // surfaceY unchanged (the boards run level).
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
      } else if (seg.kind === 'ice') {
        // ICE = a flat SLIPPERY stretch (a NEW gimmick, inverse-of-smooth). The floor is an
        // ordinary flat surface; the OBSTACLE is the slip: a smooth leg (wheel/arc/straight/limb)
        // or a HOOK finds no grip and is BLOCKED at the ice MOUTH (struggle-in-place — the legs
        // SPIN like wheels on ice, no advance). The user must redraw a TOOTHED/zigzag (아이젠) leg
        // that bites in (_isGrippy); a grippy leg crosses at full speed. Gated in update().
        const ix0 = cursorX, ix1 = cursorX + len;
        const iceY = surfaceY;                 // freeze the level (mutating-loop-var closure-bug guard)
        addSlab(ix0 + len / 2, iceY, len, thick);
        this._segs.push({ x0: ix0, x1: ix1, kind: 'ice', topYa: iceY, topYb: iceY, surfFn: () => iceY });
        if (iceY < this._maxSurfaceTopY) this._maxSurfaceTopY = iceY;
        // render body: a pale-cyan glossy overlay the renderer draws ON TOP of the flat road.
        this.iceBodies.push({ x0: ix0, x1: ix1, surfaceY: iceY });
        cursorX += len;
        // surfaceY unchanged (flat slippery floor).
      } else if (seg.kind === 'balls') {
        // BALLS = a FLAT run with a PILE OF DYNAMIC PHYSICS SPHERES lying on it. The
        // floor is an ordinary flat surface (the cube walks it with the normal gait —
        // no special terrain factor, no climb/tunnel gate). The OBSTACLE is the balls:
        // they sit on the surface under gravity and the cube must SHOVE them aside to
        // pass (a resistance that SLOWS the cube, applied in update()). We add the flat
        // surface here and RECORD the pile spec; the actual balls are spawned AFTER the
        // loop (once every surfaceFn exists so we can clamp them onto the surface).
        const bx0 = cursorX, bx1 = cursorX + len;
        // CAPTURE the floor level in a per-segment CONST — `surfaceY` is a mutating loop var;
        // a closure `() => surfaceY` would read the FINAL track level (closure bug ⇒ the balls
        // floor / surfaceYAt would teleport up to the end-of-track height). Freeze it here.
        const ballsY = surfaceY;
        addSlab(bx0 + len / 2, ballsY, len, thick);
        this._segs.push({ x0: bx0, x1: bx1, kind: 'balls', topYa: ballsY, topYb: ballsY,
          surfFn: () => ballsY });
        if (ballsY < this._maxSurfaceTopY) this._maxSurfaceTopY = ballsY;
        // pile spec (spawned post-loop). Re-uses `count` as the ball count (schema).
        const bCount = Math.min(SEGMENT_DEFAULTS.ballCountMax,
          Math.max(1, (seg.count != null ? seg.count : SEGMENT_DEFAULTS.ballCount) | 0));
        const bR = (seg.ballR != null) ? seg.ballR : SEGMENT_DEFAULTS.ballR;
        const bSpread = (seg.ballSpread != null) ? seg.ballSpread
          : len * SEGMENT_DEFAULTS.ballSpreadFrac;
        (this._ballSpecs || (this._ballSpecs = [])).push({ x0: bx0, x1: bx1, count: bCount, r: bR, spread: bSpread, surfaceY: ballsY });
        cursorX += len;
        // surfaceY unchanged (flat run under the pile).
      } else if (seg.kind === 'blocks') {
        // BLOCKS = a FLAT run with a WALL OF STANDING BOXES upright on it. The floor is an
        // ordinary flat surface (the cube walks it with the normal gait — no climb/tunnel
        // gate). The OBSTACLE is the standing blocks: while INTACT each block bars the path
        // (the cube is held at its face, no penetration). On contact a block BREAKS into
        // debris fragments that fall + litter the floor and SLOW the cube grinding over them
        // (resistance ∝ contacts, floored ⇒ no soft-lock). We add the flat surface here and
        // RECORD the wall spec; the blocks + debris are built AFTER the loop (once every
        // surfaceFn exists so we can clamp debris onto the surface).
        const bx0 = cursorX, bx1 = cursorX + len;
        // CAPTURE the floor level in a per-segment CONST — `surfaceY` is a mutating loop var;
        // a closure `() => surfaceY` would read the FINAL track level (closure bug ⇒ the blocks
        // floor / surfaceYAt + the standing-block bases would float to the end-of-track height,
        // making the cube + blocks read as sunk/teleported). Freeze it here.
        const blocksY = surfaceY;
        addSlab(bx0 + len / 2, blocksY, len, thick);
        this._segs.push({ x0: bx0, x1: bx1, kind: 'blocks', topYa: blocksY, topYb: blocksY,
          surfFn: () => blocksY });
        if (blocksY < this._maxSurfaceTopY) this._maxSurfaceTopY = blocksY;
        const blkCount = Math.max(1, (seg.blockCount != null ? seg.blockCount : SEGMENT_DEFAULTS.blockCount) | 0);
        const blkW = (seg.blockW != null) ? seg.blockW : SEGMENT_DEFAULTS.blockW;
        const blkH = (seg.blockH != null) ? seg.blockH : SEGMENT_DEFAULTS.blockH;
        const debPer = Math.max(1, (seg.debrisPerBlock != null ? seg.debrisPerBlock : SEGMENT_DEFAULTS.debrisPerBlock) | 0);
        (this._blockSpecs || (this._blockSpecs = [])).push({
          x0: bx0, x1: bx1, count: blkCount, w: blkW, h: blkH, debrisPer: debPer, surfaceY: blocksY });
        cursorX += len;
        // surfaceY unchanged (flat run under the wall).
      } else if (seg.kind === 'fork') {
        // FORK = a SPLIT-PATH the LEG SHAPE routes (commit-at-entrance). The track splits into:
        //   • HIGH route: a HOOK-GATED STEEP STAIRCASE up (overall slope >= steepThresh ⇒ auto
        //     hook-gated, EXACTLY the steep-staircase mechanic) → a flat top on the arch → a
        //     staircase back DOWN → rejoin at the base.  A HOOK commits here (a hook is exactly
        //     what climbs the steep staircase — fully consistent).
        //   • LOW route: base → a shallow DIP under the arch (a valley / underpass) → back up to
        //     the base → rejoin. The always-passable "anyone" road (any leg).
        // Both rejoin at the SAME x1 and SAME base y; normal segments continue after.
        //
        // SINGLE-HEIGHT-FUNCTION MODEL: x is forward-only and the surface is single-valued, so we
        // do NOT keep two live physics surfaces. The LOW route is built into the MAIN _segs (so the
        // default O(log n) lookup + the lookahead probe ALWAYS see the always-passable road and the
        // steep gate is never falsely tripped before the cube enters). The HIGH route is built into
        // its OWN indexed seg list stored in the fork record; surfaceYAt/_segAt swap to it ONLY for
        // x inside a fork COMMITTED high (the commit happens in update() at the entrance). Both
        // routes are also RENDERED (see renderer) so the fork visibly splits then merges.
        const fx0 = cursorX, fx1 = cursorX + len;
        const baseY = surfaceY;                                   // the rejoin base level (physics +down)
        const highRise = (seg.highRise != null) ? seg.highRise : SEGMENT_DEFAULTS.forkHighRise;
        const highSteps = Math.max(1, (seg.highSteps != null ? seg.highSteps : SEGMENT_DEFAULTS.forkHighSteps) | 0);
        const flatTop = (seg.flatTop != null) ? seg.flatTop : SEGMENT_DEFAULTS.forkFlatTop;
        const lowDip = (seg.lowDip != null) ? seg.lowDip : SEGMENT_DEFAULTS.forkLowDip;
        const highLat = (seg.highLat != null) ? seg.highLat : SEGMENT_DEFAULTS.forkHighLat;
        const latHoldFrac = (seg.latHoldFrac != null) ? seg.latHoldFrac : SEGMENT_DEFAULTS.forkLatHoldFrac;
        const reachThresh = (seg.reachThresh != null) ? seg.reachThresh : SEGMENT_DEFAULTS.forkReachThresh;
        const forkId = this._forks.length;

        // ── LOW ROUTE (into the MAIN _segs): base flat lead-in → dip-down ramp → dip-up ramp →
        //    base flat lead-out, spanning the whole fork [fx0,fx1] and rejoining at baseY. The
        //    dip is a SHALLOW symmetric valley (a V with a floor) — a short flat lead-in/out so the
        //    mouth + rejoin read cleanly, the dip in the middle. NEVER steep-gated (always passable).
        // FIX ㉠: a SHORT flat lead-in so the LOW road starts DIPPING almost immediately at the
        // mouth — combined with the high road rising + the lateral z-split, the two lanes separate in
        // BOTH y and z within a fraction of a unit (no body-box overlap at the still-low mouth).
        const lowLead = 0.4;      // a SHORT flat at the mouth/rejoin so the LOW road starts DIPPING quickly (drops below baseY soon ⇒ y-separates from the rising high road), while the Y-junction itself still reads cleanly
        const dipSpan = Math.max(1e-3, len - 2 * lowLead);            // the valley span (descent + ascent)
        const dipHalf = dipSpan / 2;
        // mouth flat
        addSlab(fx0 + lowLead / 2, baseY, lowLead, thick);
        this._segs.push({ x0: fx0, x1: fx0 + lowLead, kind: 'fork', forkId, route: 'low',
          topYa: baseY, topYb: baseY, surfFn: () => baseY });
        // descent into the valley (down = +y)
        {
          const dx0 = fx0 + lowLead, dx1 = dx0 + dipHalf;
          const dTopY0 = baseY, dTopY1 = baseY + lowDip;
          addRampSlab(dx0, dx1, dTopY0, dTopY1, thick);
          const dSlope = lowDip / dipHalf;
          this._segs.push({ x0: dx0, x1: dx1, kind: 'fork', forkId, route: 'low', gap: true,
            topYa: dTopY0, topYb: dTopY1, slope: dSlope, surfFn: (px) => dTopY0 + dSlope * (px - dx0) });
          if (dTopY1 < this._maxSurfaceTopY) this._maxSurfaceTopY = dTopY1;
        }
        // ascent back out of the valley
        {
          const ax0 = fx0 + lowLead + dipHalf, ax1 = fx0 + lowLead + dipSpan;
          const aTopY0 = baseY + lowDip, aTopY1 = baseY;
          addRampSlab(ax0, ax1, aTopY0, aTopY1, thick);
          const aSlope = -lowDip / dipHalf;
          this._segs.push({ x0: ax0, x1: ax1, kind: 'fork', forkId, route: 'low', gap: true,
            topYa: aTopY0, topYb: aTopY1, slope: aSlope, surfFn: (px) => aTopY0 + aSlope * (px - ax0) });
        }
        // rejoin flat lead-out (the LAST low seg ends exactly at fx1 at baseY ⇒ continuous rejoin)
        addSlab(fx1 - lowLead / 2, baseY, lowLead, thick);
        this._segs.push({ x0: fx1 - lowLead, x1: fx1, kind: 'fork', forkId, route: 'low',
          topYa: baseY, topYb: baseY, surfFn: () => baseY });

        // ── HIGH ROUTE (into its OWN list `highSegs`, NOT the main _segs): a MODERATE step-UP
        //    staircase onto a RAISED parallel road → flat top → step DOWN → rejoin at baseY. FIX ㉠:
        //    the fork is a LENGTH branch, NOT the steep-staircase gimmick, so the high climb is
        //    FORCED non-hook-gated (highSteep=false) regardless of its data-derived slope; its
        //    moderate per-tread risers are a LENGTH gate a LONG reach clears via canClimb (a short
        //    reach can't, but a short leg already committed LOW at the mouth). The up + down
        //    staircases symmetrically frame a flat top so the high road reads as a raised lane.
        const highSegs = [];
        const highRuns = [];                                          // (kept; empty — the high route is NOT steep-gated)
        const climbSpan = (len - flatTop) / 2;                        // x-length of EACH staircase (up + down)
        const stairSlope = highRise / Math.max(1e-3, climbSpan);      // overall up-slope (rise/run) — reported only
        const highSteep = false;                                      // FIX ㉠: a LENGTH branch, never hook-gated (NOT the steep gimmick)
        let hcx = fx0;
        let hy = baseY;
        // UP staircase (rises by highRise over climbSpan): each tread is a steep-gated stairs seg.
        {
          const stepLen = climbSpan / highSteps;
          const stepH = highRise / highSteps;
          const upX0 = hcx;
          for (let i = 0; i < highSteps; i++) {
            hy -= stepH;                                              // up = negative y
            const treadY = hy;
            highSegs.push({ x0: hcx, x1: hcx + stepLen, kind: 'stairs', forkId, route: 'high',
              topYa: treadY, topYb: treadY, stepH, steepGate: highSteep, surfFn: () => treadY });
            if (treadY < this._maxSurfaceTopY) this._maxSurfaceTopY = treadY;
            hcx += stepLen;
          }
          if (highSteep) highRuns.push({ x0: upX0, x1: hcx, slope: -stairSlope });
        }
        const topY = hy;                                              // arch top level (most negative y)
        // FLAT TOP on the arch.
        highSegs.push({ x0: hcx, x1: hcx + flatTop, kind: 'flat', forkId, route: 'high',
          topYa: topY, topYb: topY, surfFn: () => topY });
        hcx += flatTop;
        // DOWN staircase (descends back to baseY over climbSpan) — descent is NOT hook-gated.
        {
          const stepLen = climbSpan / highSteps;
          const stepH = highRise / highSteps;
          for (let i = 0; i < highSteps; i++) {
            hy += stepH;                                              // down = positive y
            const treadY = hy;
            highSegs.push({ x0: hcx, x1: hcx + stepLen, kind: 'stairs', forkId, route: 'high',
              topYa: treadY, topYb: treadY, stepH, steepGate: false, surfFn: () => treadY });
            hcx += stepLen;
          }
        }
        // clamp the final high seg to end EXACTLY at fx1 at baseY (continuous rejoin; absorb any
        // rounding from the per-step division).
        if (highSegs.length) {
          const lastSeg = highSegs[highSegs.length - 1];
          lastSeg.x1 = fx1;
          const endY = baseY;
          const a = lastSeg.topYa, run = Math.max(1e-3, lastSeg.x1 - lastSeg.x0);
          const sl = (endY - a) / run;
          lastSeg.topYb = endY;
          lastSeg.surfFn = (px) => a + sl * (px - lastSeg.x0);       // tiny linear correction tread
        }
        // build the high route's O(log n) x0 index (ascending — the high segs are pushed in x order).
        const hxs = new Float64Array(highSegs.length);
        for (let i = 0; i < highSegs.length; i++) hxs[i] = highSegs[i].x0;
        // LATERAL SPLIT PROFILE (z) of the HIGH arch. Control x's: the lateral is 0 at the fork
        // mouth (fx0) and rejoin (fx1) — the roads MEET on the lane centre there — ramps OUT to
        // highLat across the entry up-staircase (so the two roads are fully z-separated at the
        // still-low feet, where a vertical-only pass would overlap), HOLDS, then eases back to the
        // lane CENTRE (0) by the flat top (where the arch is high enough to clear the low cube
        // vertically), and MIRRORS for the descent. latUpEnd/latDownStart bracket the high (held)
        // region; latRampX is the entry/exit ramp length. forkLateralAt() reads these (O(1)).
        const climbSpanRec = (len - flatTop) / 2;
        // FAST entry/exit lateral ramp: the routes must be FULLY z-separated within the first ~quarter
        // of a unit of the fork (where both roads are still near baseY and a vertical-only pass would
        // let a low cube clip the high road's first still-low step). A SHORT ramp scissors them apart
        // at once (FIX ㉠: shortened 0.6→0.22 so the two lanes clear in z before the low cube's box can
        // overlap the high road's mouth — the roads are SEPARATE lanes from the very first step). The
        // ramp is eased (smoothstep) so the sideways move is smooth, just quick.
        const latRampX = Math.min(climbSpanRec * 0.15, 0.18);
        const topStartX = fx0 + climbSpanRec;          // x where the flat top begins (up-climb done)
        const topEndX = topStartX + flatTop;           // x where the flat top ends (down-climb begins)
        this._forks.push({ id: forkId, x0: fx0, x1: fx1, baseY, highSegs, highX0: hxs, highRuns,
          highLat, latHoldFrac, latRampX, topStartX, topEndX, climbSpan: climbSpanRec, reachThresh });

        cursorX += len;
        surfaceY = baseY;                                            // both routes rejoin at the base
      }
      // PATH HEADING: record this segment's arc-length span + turn (if any). cursorX has
      // advanced to the segment's end by here, so [_turnX0, cursorX] is its arc-length span.
      if (_turnDeg !== 0 && cursorX > _turnX0 + 1e-6) {
        this.turnRegions.push({ x0: _turnX0, x1: cursorX, turnRad: _turnDeg * Math.PI / 180 });
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

    // ── PRECOMPUTE STEEP-STAIRCASE RUN FEET (build-time, O(n) once — never per frame). ──
    // A steep-gated stairs run is a sequence of abutting treads all tagged steepGate. Its
    // FOOT is the x0 of the FIRST tread (the riser a non-hook leg is stopped at). We record
    // each run's foot x (and the run's overall slope) so _nextSteep can find the next steep
    // CLIMB foot with a tiny bounded scan (#runs, not #treads). Also record the run window
    // (footX0 .. lastX1) so the per-frame gate can ask "is px on a steep-gated stairs run?"
    // without re-deriving it. A tread is a run foot iff the immediately-preceding segment is
    // NOT a steep-gated stairs tread abutting it (x1 ≈ this.x0). Treads are pushed in x order.
    this._steepStairRuns = [];   // [{ x0, x1, slope }] — one entry per steep staircase run
    {
      let prevSteepStairX1 = -Infinity, cur = null;
      for (const s of this._segs) {
        if (s.kind === 'stairs' && s.steepGate) {
          if (cur && Math.abs(s.x0 - prevSteepStairX1) < 1e-6) {
            cur.x1 = s.x1;                       // extend the current run (abutting tread)
          } else {
            // a new run starts: its foot is this tread's x0, slope from the run rise/run.
            cur = { x0: s.x0, x1: s.x1, slope: 0 };
            this._steepStairRuns.push(cur);
          }
          prevSteepStairX1 = s.x1;
        } else {
          cur = null; prevSteepStairX1 = -Infinity;
        }
      }
      // fill in each run's overall slope (rise/run, physics +down ⇒ up is negative). The
      // surface at the foot is the run's HIGH y (lowest step); at the top it is the most
      // negative y. slope = (topY − footTopY)/(x1 − x0) < 0 for an ascent.
      for (const r of this._steepStairRuns) {
        const yFoot = this.surfaceYAt(r.x0 + 1e-3);
        const yTop = this.surfaceYAt(r.x1 - 1e-3);
        const run = Math.max(1e-3, r.x1 - r.x0);
        r.slope = (yFoot != null && yTop != null) ? (yTop - yFoot) / run : 0;
      }
    }

    // ── SPAWN THE BALL PILES (AFTER the lookup index so surfaceYAt resolves). ──
    // We pack all piles into ONE set of flat arrays (cache-friendly stepping). Each ball
    // is scattered over the pile's mid-segment x window, in a few rows so they READ as a
    // heap (not a flat line), then dropped — gravity + the post-build settle land them on
    // the surface. Allocated ONCE here; update() only mutates values (zero per-frame alloc).
    if (this._ballSpecs && this._ballSpecs.length) {
      let total = 0;
      for (const sp of this._ballSpecs) total += sp.count;
      this._ballN = total;
      this._ballX = new Float64Array(total);
      this._ballY = new Float64Array(total);
      this._ballVX = new Float64Array(total);
      this._ballVY = new Float64Array(total);
      this._ballR = new Float64Array(total);
      this._ballRenderList = new Array(total);
      let idx = 0;
      for (const sp of this._ballSpecs) {
        // centre the pile in the segment so the cube hits it mid-stretch. Scatter over
        // `spread` in x; stack a few rows in y so it looks like a heap. Deterministic
        // pseudo-random jitter (index-seeded) so a rebuild reproduces the same pile.
        const cx = (sp.x0 + sp.x1) / 2;
        const x0 = cx - sp.spread / 2;
        const perRow = Math.max(1, Math.ceil(Math.sqrt(sp.count)));
        for (let k = 0; k < sp.count; k++) {
          const col = k % perRow;
          const row = Math.floor(k / perRow);
          // jitter: cheap hash on (idx) → [-0.5,0.5)
          const j = ((Math.sin(idx * 12.9898) * 43758.5453) % 1 + 1) % 1 - 0.5;
          const bx = x0 + (col + 0.5 + j * 0.5) * (sp.spread / perRow);
          const surf = this.surfaceYAt(bx);
          const baseY = (surf == null ? sp.surfaceY : surf) - sp.r; // ball centre rests at surf - r
          // stack rows ABOVE the surface (physics +down ⇒ subtract per row) so they drop
          // and settle into a heap.
          const by = baseY - row * sp.r * 1.7;
          this._ballX[idx] = bx;
          this._ballY[idx] = by;
          this._ballVX[idx] = 0;
          this._ballVY[idx] = 0;
          this._ballR[idx] = sp.r;
          this._ballRenderList[idx] = { x: bx, y: by, r: sp.r };
          idx++;
        }
      }
      // SETTLE the pile so it starts at rest (no drop animation at the start line). Run a
      // handful of fixed steps of the ball-only physics (cube far away ⇒ no push) so the
      // heap is already resting on the surface when the race begins.
      for (let s = 0; s < 60; s++) this._stepBalls(this.FIXED_DT / 1000, -1e9, 0, 0);
      this._syncBallRenderList(); // reflect the settled rest pose for the first render
      this._ballSpecs = null; // specs consumed
    } else {
      this._ballN = 0;
    }

    // ── SPAWN THE STANDING BLOCKS + their (latent) DEBRIS (AFTER the lookup index). ──
    // Each wall lays its blocks UPRIGHT in a row across the segment window, resting their
    // BASE on the surface. Debris is pre-allocated (one slot per block × debrisPer, capped
    // at debrisCapTotal) but INACTIVE — a fragment only comes alive when its parent block
    // breaks (on cube contact), where it is given an outward burst. All arrays allocated
    // ONCE here; update()/_stepBlocks only mutate values (zero per-frame allocation).
    if (this._blockSpecs && this._blockSpecs.length) {
      // first pass: count blocks + plan debris (capped), build the standing-block list.
      const blocks = [];
      let debTotal = 0;
      const cap = SEGMENT_DEFAULTS.debrisCapTotal;
      for (const sp of this._blockSpecs) {
        const w = sp.w, h = sp.h, n = sp.count;
        // lay the blocks in a row centred in the segment. A small inter-block gap so the
        // wall reads as separate boxes (not one slab). Row width = n*w + (n-1)*gap.
        const gap = w * 0.35;
        const rowW = n * w + (n - 1) * gap;
        const cx = (sp.x0 + sp.x1) / 2;
        const rowX0 = cx - rowW / 2;
        for (let k = 0; k < n; k++) {
          const bx0 = rowX0 + k * (w + gap);   // block's near (left) face x
          const bxc = bx0 + w / 2;             // block centre x
          const surf = this.surfaceYAt(bxc);
          const base = (surf == null ? sp.surfaceY : surf); // surface (top) the block stands on
          const topY = base - h;               // block top (physics +down ⇒ up = negative)
          blocks.push({ x: bxc, faceX: bx0, topY, baseY: base, w, h,
            broken: false, debrisPer: sp.debrisPer, segX0: sp.x0, segX1: sp.x1 });
          debTotal += sp.debrisPer;
        }
      }
      // cap debris: if the plan exceeds the cap, scale each block's debris down proportionally.
      let debN = debTotal;
      if (debN > cap) {
        const scale = cap / debTotal;
        let acc = 0;
        for (const b of blocks) { b.debrisPer = Math.max(1, Math.round(b.debrisPer * scale)); }
        for (const b of blocks) acc += b.debrisPer;
        debN = acc;
      }
      this._blockN = blocks.length;
      this._blocks = blocks;
      this._brokenCount = 0;
      this._blockRenderList = new Array(this._blockN);
      for (let i = 0; i < this._blockN; i++) {
        const b = this._blocks[i];
        this._blockRenderList[i] = { x: b.x, topY: b.topY, baseY: b.baseY, w: b.w, h: b.h, broken: false };
      }
      // pre-allocate debris (inactive). Assign each block a contiguous debris index range so
      // a break only touches its own slots. r = a small chip half-size from the block width.
      this._debN = debN;
      this._debX = new Float64Array(debN);
      this._debY = new Float64Array(debN);
      this._debVX = new Float64Array(debN);
      this._debVY = new Float64Array(debN);
      this._debR = new Float64Array(debN);
      this._debActive = new Uint8Array(debN);
      this._debRenderList = new Array(debN);
      let di = 0;
      for (const b of this._blocks) {
        const r = b.w * SEGMENT_DEFAULTS.debrisRfrac; // chip half-size
        b.debIdx0 = di; b.debIdx1 = di + b.debrisPer;
        for (let k = 0; k < b.debrisPer; k++) {
          // park the (inactive) fragment at the block's body so a stray render before break
          // would still be inside the block. Activated + burst on break.
          this._debX[di] = b.x;
          this._debY[di] = b.topY + b.h * 0.5; // block mid-height
          this._debVX[di] = 0; this._debVY[di] = 0;
          this._debR[di] = r;
          this._debActive[di] = 0;
          this._debRenderList[di] = { x: this._debX[di], y: this._debY[di], r, active: false };
          di++;
        }
      }
      this._debResist = 1; this._debContacts = 0;
      this._syncBlockRenderList();
      this._syncDebrisRenderList();
      this._blockSpecs = null; // specs consumed
    } else {
      this._blockN = 0; this._debN = 0;
    }

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
    this._grip = 0;
    this._gripLiveAmp = 0;
    const startSurf0 = this.surfaceYAt(track.startX);
    this._footBaseY = (startSurf0 == null) ? groundY : startSurf0;
    this._prevFootBaseY = this._footBaseY;
    // a rebuild clears any prior idle-float (the game re-arms it via setIdleFloat for
    // the interactive start; headless/forceStart paths build + start immediately).
    this._idleFloat = false;
    this._idlePhase = 0;
    // RENDER INTERPOLATION: a fresh track build teleports the cube to startX, so any
    // prior prev-snapshot is unrelated — mark it stale so the next render draws curr
    // directly (no lerp across the teleport).
    this.resetInterp();
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

  /** Index of the fork whose [x0,x1] contains px, or -1. Bounded #forks scan (forks are
   * a handful) with an O(1) hint for the monotone-forward hot path. No allocation. */
  _forkIdxAt(px) {
    const F = this._forks;
    if (!F || !F.length) return -1;
    const h = this._activeForkHint;
    if (h >= 0 && h < F.length && px >= F[h].x0 && px <= F[h].x1) return h;
    for (let i = 0; i < F.length; i++) {
      if (px >= F[i].x0 && px <= F[i].x1) { this._activeForkHint = i; return i; }
    }
    return -1;
  }

  /** The COMMITTED route ('high' | 'low') of fork `id`, or null if not yet committed.
   * Before commit (lookahead probes from outside the fork) callers DEFAULT to the LOW
   * route (always-passable) so the steep gate is never falsely tripped before entry. */
  _committedRoute(id) {
    return (this._forkRoute && this._forkRoute.has(id)) ? this._forkRoute.get(id) : null;
  }

  /** COMMIT the fork route at the ENTRANCE. Called each step with the cube's forward x: for the
   * fork the cube is currently inside, if it has NOT been committed yet, commit it NOW from the
   * CURRENT leg LENGTH (FIX ㉠): reach >= the fork's reachThresh ⇒ the HIGH road (a LONG leg mounts
   * the raised step-up via canClimb), else the LOW ground road (a SHORT leg can't mount it). Once
   * committed the route is FIXED for the whole fork (the rejoin is close), so REDRAWING the leg
   * MID-fork does NOT switch roads (no surface teleport). Idempotent + O(1) (forks are a handful,
   * the fork-index hint makes the common case O(1)); zero allocation. Returns the committed route,
   * or null when not inside a fork. */
  _commitForkAt(px) {
    const fi = this._forkIdxAt(px);
    if (fi < 0) return null;
    const f = this._forks[fi];
    if (this._forkRoute.has(f.id)) return this._forkRoute.get(f.id);
    // LENGTH branch: a long leg (reach >= threshold) takes the HIGH raised road; a short leg the LOW.
    const thresh = (f.reachThresh != null) ? f.reachThresh : SEGMENT_DEFAULTS.forkReachThresh;
    const route = (this._reach >= thresh) ? 'high' : 'low';
    this._forkRoute.set(f.id, route);
    return route;
  }

  /** If px lies inside a fork that is COMMITTED to the HIGH route, return that fork's
   * record (so surfaceYAt/_segAt sample the high segs). Otherwise null (⇒ the main _segs,
   * which hold the LOW route over the fork, are used — the always-passable default). */
  _highForkAt(px) {
    const fi = this._forkIdxAt(px);
    if (fi < 0) return null;
    const f = this._forks[fi];
    return this._committedRoute(f.id) === 'high' ? f : null;
  }

  /** Smoothstep on [0,1] (C¹, zero-slope ends — no kink). */
  _smoothstep01(t) { if (t <= 0) return 0; if (t >= 1) return 1; return t * t * (3 - 2 * t); }

  /** The HIGH road's LATERAL z-offset profile at px FOR A GIVEN FORK record `f` — ALWAYS defined
   * (the high road is always rendered). FIX ㉠: the high road is a SEPARATE PARALLEL LANE — it
   * veers OUT to f.highLat right at the mouth and STAYS fully offset for the WHOLE fork (climb,
   * flat top AND descent), ramping back to the lane centre only over the final exit so the two
   * roads REJOIN cleanly. So the high lane runs alongside the low (centre) lane the entire way —
   * two distinct roads, separated in z the whole length (and in y by the raise/dip). The
   * HIGH-committed cube + legs ride this same lateral (WYSIWYG). O(1), alloc-free. 0 outside f. */
  _forkHighLatRec(f, px) {
    if (!f || px < f.x0 || px > f.x1) return 0;
    const L = f.highLat || 0;
    if (L === 0) return 0;
    const ramp = f.latRampX;
    // ENTRY: ramp 0→L over [x0, x0+ramp] (the roads split apart at the mouth). HOLD the full lateral
    // L across the ENTIRE middle of the fork (climb + flat top + descent — the two roads run parallel
    // and clearly apart the whole way). EXIT: ramp L→0 over [x1−ramp, x1] so they rejoin at the lane
    // centre. No easing back to centre on the flat top (that was the old "arch over the lane" look —
    // the user said the over/under being glued together is weird; now they stay SEPARATE roads).
    if (px <= f.x0 + ramp) {                            // entry ramp 0 → L (roads split apart)
      return L * this._smoothstep01((px - f.x0) / ramp);
    }
    if (px < f.x1 - ramp) return L;                    // HELD full lateral the whole way (parallel lanes)
    return L * (1 - this._smoothstep01((px - (f.x1 - ramp)) / ramp)); // exit ramp L → 0 at the rejoin
  }

  /** The HIGH arch's lateral z-offset at px (for the RENDERER — the arch is always drawn). 0
   * outside any fork. O(1), alloc-free. */
  forkHighLatAt(px) {
    const fi = this._forkIdxAt(px);
    if (fi < 0) return 0;
    return this._forkHighLatRec(this._forks[fi], px);
  }

  /** The lateral z-offset the CUBE (+ its legs) rides at px: the HIGH arch's lateral profile when
   * px is inside a fork COMMITTED to the HIGH route, else 0 (the low road / normal track is on the
   * lane centre). So a high-committed cube tracks the arch sideways (WYSIWYG) while a low cube and
   * all normal segments stay centred. O(1), alloc-free — safe on the per-frame render hot path. */
  forkLateralAt(px) {
    const f = this._highForkAt(px);
    return f ? this._forkHighLatRec(f, px) : 0;
  }

  /** Binary-search the HIGH route's own ascending x0 index for the seg covering px.
   * Mirrors _segIdxAt but over the fork's private highX0 (no shared hint — high forks are
   * short, the linear-then-binary cost is trivial). Returns the seg or null. */
  _highSegAt(f, px) {
    const xs = f.highX0, segs = f.highSegs, n = xs.length;
    if (!n) return null;
    let lo = 0, hi = n - 1, res = -1;
    while (lo <= hi) { const mid = (lo + hi) >> 1; if (xs[mid] <= px) { res = mid; lo = mid + 1; } else hi = mid - 1; }
    if (res < 0) return null;
    // check the matched seg + seam neighbours (treads abut; px can equal a seam x).
    let found = null, foundY = null;
    for (let j = res - 1; j <= res + 1; j++) {
      if (j < 0 || j >= n) continue;
      const s = segs[j];
      if (px < s.x0 || px > s.x1) continue;
      const y = s.surfFn(px);
      if (y == null) { if (!found) found = s; continue; }
      if (!found || foundY == null || y < foundY) { found = s; foundY = y; }
    }
    return found;
  }

  /**
   * Expected track surface y (physics, +down) at any x. Returns the HIGHEST (most
   * negative) surface covering px, or null over a gap. O(log n) via _segIdxAt; the
   * "highest covering surface" semantics are preserved by also testing the seam
   * neighbours of the matched segment (the only place two segments can share an x).
   * FORK: for x inside a fork COMMITTED to the HIGH route, the high profile is returned
   * instead (the main _segs hold the LOW route — the always-passable default).
   */
  surfaceYAt(px) {
    const hf = this._highForkAt(px);
    if (hf) { const s = this._highSegAt(hf, px); return s ? s.surfFn(px) : null; }
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
    // FORK low-route ramps (the dip/underpass) carry an analytic slope too; high-route segs
    // are stairs/flat (slope 0, handled below). A fork flat lead-in/out has no slope ⇒ 0.
    if (s && s.kind === 'fork') return (typeof s.slope === 'number') ? s.slope : 0;
    if (s && (s.kind === 'flat' || s.kind === 'stairs' || s.kind === 'ice')) return 0;
    // CURVE: a flat curve has no slope (0); a sloped curve carries an analytic slope.
    if (s && s.kind === 'curve') return (typeof s.slope === 'number') ? s.slope : 0;
    if (s && s.kind === 'planks' && s.plank) return 0; // a plank board is level
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
  /** The seg list a stairs seg's RUN lives in: a HIGH-route fork stair scans its fork's
   * own highSegs (the high staircases are NOT in the main _segs); any other stairs scans
   * the main _segs. Keeps the contiguous-run lean/float scan correct for fork stairs. */
  _stairScanList(seg) {
    if (seg && seg.route === 'high' && seg.forkId != null) {
      const f = this._forks[seg.forkId];
      if (f) return f.highSegs;
    }
    return this._segs;
  }

  bodySurfaceYAt(px) {
    const seg = this._segAt(px);
    if (seg && seg.kind === 'stairs') {
      // find the contiguous stair RUN containing px (abutting stair treads).
      let x0 = seg.x0, x1 = seg.x1, ya = seg.surfFn(seg.x0), yb = seg.surfFn(seg.x1);
      for (const s of this._stairScanList(seg)) {
        if (s.kind !== 'stairs') continue;
        if (s.x1 > x0 - 1e-6 && s.x0 < x1 + 1e-6) {
          if (s.x0 < x0) { x0 = s.x0; ya = s.surfFn(s.x0); }
          if (s.x1 > x1) { x1 = s.x1; yb = s.surfFn(s.x1); }
        }
      }
      const run = Math.max(1e-3, x1 - x0);
      // NOTE: this helper is currently UNREFERENCED (the cube body height comes from
      // _groundedCubeY, which grazes the DISCRETE treads; the stepped 한-칸씩 climb is driven by
      // the dwell-reach gait + stepped commit level). Kept as a smooth tread-top-chord sampler in
      // case a body-float path needs it. Ride the staircase HYPOTENUSE biased up one step (always
      // ≥ every tread top ⇒ penetration-free).
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
      for (const s of this._stairScanList(seg)) {
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
    // FORK low-route ramp (the dip/underpass) leans to its analytic slope; a fork flat
    // lead-in/out (no slope) is level. High-route segs are stairs/flat (handled above/here).
    if (seg && seg.kind === 'fork') {
      return (typeof seg.slope === 'number') ? clampMag(Math.atan(seg.slope) * TUNE.tiltGain, TUNE.tiltMax) : 0;
    }
    if (seg && (seg.kind === 'flat' || seg.kind === 'ice')) return 0; // flat / slippery floor is level
    if (seg && seg.kind === 'planks' && seg.plank) return 0; // a plank board is level
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

  /** Max ceiling PENETRATION (world-u) over BOTH legs' chain points at a CANDIDATE spin angle
   * `theta` (+ body `tilt`), computed from the axle-local chain directly (no leg-body rebuild,
   * zero alloc). A point at world (px,py) with leg-line top (py − lineRadius) ABOVE (smaller y
   * than) the ceiling there ⇒ penetration = ceilingY − (py − r) > 0. Used to JAM the leg at a
   * tunnel ceiling (revert the spin) so a too-long leg never rolls THROUGH it. 0 if no ceiling. */
  _maxCeilingPen(theta, tilt) {
    const ch = this._chain;
    if (!ch || !ch.length || !this.cube) return 0;
    const axleX = this.cube.position.x + AXLE_X, axleY = this.cube.position.y + AXLE_Y;
    const r = LEG_LINE_RADIUS;
    let maxPen = 0;
    for (let s = 0; s < 2; s++) {
      const a = theta + (s === 0 ? 0 : this._legPhaseOffset) + (tilt || 0);
      const ca = Math.cos(a), sa = Math.sin(a);
      for (let i = 0; i < ch.length; i++) {
        const px = axleX + (ch[i].x * ca - ch[i].y * sa);
        const py = axleY + (ch[i].x * sa + ch[i].y * ca);
        const cy = this.ceilingYAt(px);
        if (cy == null) continue;
        const pen = cy - (py - r);   // >0 ⇒ leg-line top is ABOVE the ceiling = penetration
        if (pen > maxPen) maxPen = pen;
      }
    }
    return maxPen;
  }

  /** Local segment under px (for terrain / climb decisions). O(log n) via _segIdxAt;
   * preserves the "prefer the highest surface" semantics by testing the matched
   * segment plus its seam neighbours (the only place two segments can share an x).
   * FORK: for x inside a COMMITTED-HIGH fork, the committed high seg is returned (the
   * main _segs hold the LOW route — the always-passable default before/without commit). */
  _segAt(px) {
    const hf = this._highForkAt(px);
    if (hf) return this._highSegAt(hf, px);
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
    // a fresh stroke replaces any prior hook classification (recomputed at step 4b below).
    this._isHook = false; this._maxTurnDeg = 0;
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

    // 4b. HOOK CLASSIFICATION (shape, NOT length). Measure the MAX TURN ANGLE between
    //     consecutive direction vectors along the chain (with a small lookahead window
    //     so resampling micro-jitter on a smooth arc/circle does not fake a corner). A
    //     hook / ㄱ / L / claw has a sharp (~75°+) bend; a straight stick / bar / smooth
    //     arc / circle / wheel / blobby limb is < ~53°. isHook = bend >= hookAngleDeg.
    //     This is fully independent of reach — used ONLY by the STEEP-RAMP gate.
    {
      const hm = hookMetrics(chain, TUNE.hookTurnWindow);
      this._maxTurnDeg = hm.maxTurnDeg;
      this._signReversals = hm.signReversals;
      this._nSharp = hm.nSharp;
      // GENUINE HOOK: a sharp enough corner AND a directionally-coherent bend (curls ONE way
      // — a clean ㄱ/J/L), NOT a back-and-forth zigzag/scribble (you can't grip with a wiggle).
      this._isHook = (hm.maxTurnDeg >= TUNE.hookAngleDeg) && (hm.signReversals <= TUNE.hookMaxReversals);
      // GRIPPY (아이젠/crampon): a MANY-TOOTHED zigzag — several sharp corners that ALTERNATE
      // direction (left-right-left). This is the SHAPE that bites into ICE (the opposite of a
      // smooth wheel/arc which slips). Distinct from a HOOK (one coherent bend, low reversals):
      // a hook does NOT grip ice (it's not toothed), so ice forces yet ANOTHER leg shape.
      this._isGrippy = (hm.nSharp >= TUNE.iceGripSharpMin) && (hm.signReversals >= TUNE.iceGripRevMin);
    }

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
      this._grip = 0;
      this._gripLiveAmp = 0;
      this._stepClimbActive = false;
      this._stepCommitY = null;
      this._stepLevelY = null;
      this._stepPlantArmed = true;
      this._stepProfile = 0;
      this._gripGaitLive = 0;
      this._gripWarp = 1;
      this._gripDwelling = false;
      this._footBaseY = surf;
      this._prevFootBaseY = surf;
      // RENDER INTERPOLATION: a fresh leg teleports x to startX — drop any stale prev so
      // the next render draws curr directly (no lerp across the teleport).
      this.resetInterp();
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

  /** Find the next ICE foot (an ice segment's x0) in (fromX, toX], or null. The foot is where
   * a NON-GRIPPY (smooth/hook) leg slips and is stopped (struggle-in-place) until a TOOTHED
   * zigzag (아이젠) leg is drawn. Mirrors _nextTunnel; reuses a scratch object (no per-frame alloc). */
  _nextIce(fromX, toX) {
    let bestX = Infinity, any = false;
    for (const s of this._segs) {
      if (s.kind !== 'ice') continue;
      if (s.x0 > fromX && s.x0 <= toX) { if (!any || s.x0 < bestX) { bestX = s.x0; any = true; } }
    }
    if (!any) { this._iceHit.found = false; return null; }
    const h = this._iceHit; h.found = true; h.x = bestX; return h;
  }

  /** Find the next STEEP UPHILL CLIMB foot whose x lies in (fromX, toX]. This covers BOTH
   * (a) a steep-gated regular ramp (steepGate=true, not a gap — foot = x0) AND (b) a steep-
   * gated STAIRCASE run (its FOOT = the first tread's x0, precomputed in _steepStairRuns).
   * Returns { x, slope } or null. The foot is where a NON-HOOK leg is stopped (struggle-in-
   * place — the step edges / slope have no grip for a straight or round leg). REUSEs a scratch
   * object (no per-frame allocation). The staircase scan is over #runs (a handful), not #treads,
   * so the hot path stays cheap. A steep STAIRCASE is hook-gated EXACTLY like a steep ramp. */
  _nextSteep(fromX, toX) {
    let bestX = Infinity, bestSlope = 0, any = false;
    // (a) steep-gated regular ramps (the original behaviour).
    for (const s of this._segs) {
      if (s.kind !== 'ramp' || !s.steepGate || s.gap) continue;
      if (s.x0 > fromX && s.x0 <= toX) {
        if (!any || s.x0 < bestX) { bestX = s.x0; bestSlope = s.slope; any = true; }
      }
    }
    // (b) steep-gated STAIRCASE run feet (precomputed; bounded #runs scan).
    for (let i = 0; i < this._steepStairRuns.length; i++) {
      const r = this._steepStairRuns[i];
      if (r.x0 > fromX && r.x0 <= toX) {
        if (!any || r.x0 < bestX) { bestX = r.x0; bestSlope = r.slope; any = true; }
      }
    }
    // (c) FORK HIGH-route steep staircase runs — ONLY for a fork COMMITTED to the HIGH route.
    //     Before commit the high road isn't the active surface (the LOW road is), so its steep
    //     gate must NOT fire on a lookahead probe (invariant: lookahead never trips the gate
    //     before entry). A non-hook never commits HIGH, so in practice this gate only confirms
    //     consistency; a hook on the high road passes it. Bounded #forks × #runs scan.
    for (let i = 0; i < this._forks.length; i++) {
      const f = this._forks[i];
      if (this._committedRoute(f.id) !== 'high') continue;
      for (let k = 0; k < f.highRuns.length; k++) {
        const r = f.highRuns[k];
        if (r.x0 > fromX && r.x0 <= toX) {
          if (!any || r.x0 < bestX) { bestX = r.x0; bestSlope = r.slope; any = true; }
        }
      }
    }
    if (!any) { this._steepHit.found = false; return null; }
    const h = this._steepHit;
    h.found = true; h.x = bestX; h.slope = bestSlope;
    return h;
  }

  /** Is px inside a steep-gated STAIRCASE run (foot..top)? Bounded #runs scan (no per-frame
   * alloc). Used by the per-frame gate so the per-RISER length gate is SUPPRESSED on a steep
   * staircase (the hook gate is the only thing that matters there — reach is irrelevant).
   * FORK: also true on a COMMITTED-HIGH fork's steep staircase run (so its risers are
   * hook-gated, not length-gated, exactly like the standalone steep staircase). */
  _onSteepStairRun(px) {
    for (let i = 0; i < this._steepStairRuns.length; i++) {
      const r = this._steepStairRuns[i];
      if (px >= r.x0 - 1e-6 && px <= r.x1 + 1e-6) return true;
    }
    for (let i = 0; i < this._forks.length; i++) {
      const f = this._forks[i];
      if (this._committedRoute(f.id) !== 'high') continue;
      for (let k = 0; k < f.highRuns.length; k++) {
        const r = f.highRuns[k];
        if (px >= r.x0 - 1e-6 && px <= r.x1 + 1e-6) return true;
      }
    }
    return false;
  }

  /** On ANY ascending staircase (gentle/length-gated OR steep/hook-gated, incl. a committed
   * high-fork run) at px? Drives the STEP-CLIMB CADENCE (dwell-reach leg gait + per-plant
   * bite-lift + stepped body ascent) so the cube climbs tread-by-tread (한 칸씩 짚고) on EVERY
   * staircase, not only the steep hook one — the user's reference feel. Stairs always rise in
   * our tracks (descents are ramps), so "on a stairs seg" == "ascending". */
  _onStairRun(px) {
    if (this._onSteepStairRun(px)) return true;     // steep-gated runs (incl. committed high-fork)
    const s = this._segAt(px);
    return !!(s && s.kind === 'stairs');
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

  /** FIX ㉣ — GRIP-CLIMB phase-rate WARP ∈ (0, …): how much faster/slower the leg phase θ
   * advances at the current phase during a steep-hook climb. With the two legs 180° apart a
   * foot PLANTS twice per π of θ (θ ≡ 0 mod π = a foot straight down). We want the leg to DWELL
   * (advance SLOWLY) through the plant phase — a visible "짚고" hold at the tread edge — and
   * REACH (advance QUICKLY) through the swing between plants. cos(2θ) is +1 at the plants and −1
   * between, so warp = 1 − depth·cos(2θ) is (1−depth) at a plant (slow dwell) and (1+depth)
   * mid-swing (fast reach). CRUCIALLY its MEAN over a half-stride is EXACTLY 1 (the cos term
   * integrates to 0), so the leg's AVERAGE ω is the no-slip v/r — the climb advances at the SAME
   * pace, never stalls; the dwell only REDISTRIBUTES the rotation in time (a visual beat), it does
   * not brake the climb. `depth` 0 ⇒ warp ≡ 1 (constant roll, off). At a plant warp·ω is the slow
   * dwell but the foot is on the ground only momentarily there, so no-slip on contact is preserved
   * (the instantaneous contact still has ω = warp·v/r matched to the body's forward motion). */
  _gripPhaseWarp(theta, depth) {
    if (depth <= 1e-4) return 1;
    return 1 - depth * Math.cos(2 * theta);
  }

  /** FIX ㉣ — LEG LENGTH → TREADS-PER-PLANT (the user's "다리 길이에 따라 몇 번째 계단을 밟는다").
   * A LONGER reach REACHES over MORE treads per plant (a big climbing stride); a SHORT reach plants
   * on the immediate next tread (one at a time). Maps reach (MIN..MAX) → [gripTreadsShort .. gripTreadsLong],
   * rounded to a whole number of tread edges (an integer count of footholds). O(1), no allocation. */
  _gripTreadsPerPlant(reach) {
    const t = this.reachNorm(reach); // 0 (short) .. 1 (long)
    const n = TUNE.gripTreadsShort + (TUNE.gripTreadsLong - TUNE.gripTreadsShort) * t;
    return Math.max(1, Math.round(n));
  }

  // ── BALL-FIELD PHYSICS (light single-sphere model) ──
  /** Advance the dynamic ball pile by `dt` seconds. The cube (centre cubeX,cubeY of
   * half-extent ballCubeHalf, moving forward at vCube) PUSHES any ball it overlaps in
   * the +x direction (and a little up) so the cube shoves the pile aside. Each ball:
   *   • falls under gravity, clamps onto the track surface beneath it (surfaceYAt) with a
   *     small restitution bounce (never sinks below — structural, like the cube's foot),
   *   • separates from neighbours with a positional impulse (O(N²), N<=20 ⇒ cheap),
   *   • is damped by linear friction so it rolls to rest after being shoved.
   * Returns the number of balls the cube was IN CONTACT with this step (for the cube
   * resistance / verifier). Pure value mutation of the flat arrays — ZERO allocation.
   * `cubeX = -1e9` (the settle path) ⇒ the cube is "infinitely far" ⇒ no push. */
  _stepBalls(dt, cubeX, cubeY, vCube) {
    const N = this._ballN;
    if (!N) { this._ballContacts = 0; return 0; }
    const X = this._ballX, Y = this._ballY, VX = this._ballVX, VY = this._ballVY, R = this._ballR;
    const g = TUNE.ballGravity, fr = TUNE.ballFriction, rest = TUNE.ballRestitution;
    const cubeHalf = TUNE.ballCubeHalf;
    let contacts = 0;
    // 1. integrate gravity + friction, advect, ground-clamp.
    const damp = Math.exp(-fr * dt);
    for (let i = 0; i < N; i++) {
      VY[i] += g * dt;          // gravity (+down)
      VX[i] *= damp;            // linear friction (rolling resistance)
      VY[i] *= damp;
      X[i] += VX[i] * dt;
      Y[i] += VY[i] * dt;
      // GROUND CLAMP: the ball CENTRE must stay >= r above the surface under it (so the
      // ball never sinks into the track). surfaceY is +down; "above" = smaller y. The
      // ball rests when its centre y == surfaceY - r.
      const surf = this.surfaceYAt(X[i]);
      if (surf != null) {
        const restY = surf - R[i];     // centre y at rest on the surface
        if (Y[i] > restY) {            // below the surface (penetrating) ⇒ pop back up
          Y[i] = restY;
          if (VY[i] > 0) VY[i] = -VY[i] * rest; // bounce a little, mostly absorbed
        }
      }
    }
    // 2. ball-ball separation (positional, O(N²) — N<=20). Push overlapping pairs apart
    //    along their centre line; share the correction; add a damped impulse so a shoved
    //    ball nudges its neighbours (the pile spreads when the cube plows in).
    const sepStiff = TUNE.ballSepStiff;
    for (let i = 0; i < N; i++) {
      for (let j = i + 1; j < N; j++) {
        let dx = X[j] - X[i], dy = Y[j] - Y[i];
        let dist = Math.hypot(dx, dy);
        const minD = R[i] + R[j];
        if (dist < minD && dist > 1e-6) {
          const overlap = minD - dist;
          const nx = dx / dist, ny = dy / dist;
          const corr = overlap * 0.5;
          X[i] -= nx * corr; Y[i] -= ny * corr;
          X[j] += nx * corr; Y[j] += ny * corr;
          // velocity exchange along the normal (separation impulse, damped).
          const rel = (VX[j] - VX[i]) * nx + (VY[j] - VY[i]) * ny;
          if (rel < 0) {
            const imp = -rel * sepStiff * dt * 0.5;
            VX[i] -= nx * imp; VY[i] -= ny * imp;
            VX[j] += nx * imp; VY[j] += ny * imp;
          }
        } else if (dist <= 1e-6) {
          // exact overlap (degenerate) — nudge apart along x deterministically.
          X[j] += R[j] * 0.5; X[i] -= R[i] * 0.5;
        }
      }
    }
    // 3. CUBE → BALL PUSH. The cube (centre, half-extent cubeHalf) shoves any ball whose
    //    centre is within (cubeHalf + r) — push it FORWARD (+x, the cube's travel) and a
    //    little UP so it pops over/rolls off, and count it as a "contact" (the cube's
    //    resistance scales with the contact count). The push speed scales with the cube's
    //    own forward speed (a fast cube flings them harder). No push during the settle
    //    pass (cubeX == -1e9) or when the cube is behind the ball (only shove ahead).
    if (cubeX > -1e8) {
      for (let i = 0; i < N; i++) {
        const dx = X[i] - cubeX, dy = Y[i] - cubeY;
        const reach = cubeHalf + R[i] + TUNE.ballContactPad;
        const dist = Math.hypot(dx, dy);
        if (dist < reach) {
          contacts++;
          // only PUSH balls at/ahead of the cube (dx >= a small negative) so it shoves
          // the pile forward; balls already behind it are left to settle.
          if (dx > -R[i]) {
            // ease the ball out of the cube along the contact normal (positional) — a FRACTION
            // of the overlap per frame (ballPushPosFrac) so it slides out softly instead of
            // snapping out (the "억지 밀림" fix); ball-ball sep + the cube's forward motion finish it.
            const over = (reach - dist) * TUNE.ballPushPosFrac;
            let nx = dist > 1e-6 ? dx / dist : 1, ny = dist > 1e-6 ? dy / dist : 0;
            X[i] += nx * over; Y[i] += ny * over;
            // impart the cube's forward speed (× ballPushSpeed) + a little lift so the
            // ball accelerates ahead and pops up (rolls off), not just slides flat.
            const push = Math.max(0, vCube) * TUNE.ballPushSpeed;
            if (push > VX[i]) VX[i] = push;            // never slow a faster ball
            VY[i] -= push * TUNE.ballPushUp;           // up (physics +down ⇒ subtract)
          }
        }
      }
    }
    // 4. FINAL GROUND CLAMP. Separation (step 2) and the cube push (step 3) move balls
    //    POSITIONALLY without a ground constraint, so a shoved/separated ball can end the
    //    step below its surface. Re-clamp every ball onto the surface here so the pile can
    //    NEVER sink into the track (structural — same guarantee as the cube's foot). Zero
    //    allocation; one pass.
    for (let i = 0; i < N; i++) {
      const surf = this.surfaceYAt(X[i]);
      if (surf == null) continue;
      const restY = surf - R[i];
      if (Y[i] > restY) {            // below the surface ⇒ pop up to rest
        Y[i] = restY;
        if (VY[i] > 0) VY[i] = 0;    // kill downward velocity at the floor (settled)
      }
    }
    this._ballContacts = contacts;
    return contacts;
  }

  /** Resistance slow-factor for the cube from the pile it is currently shoving. Each
   * contacting ball compounds a slowdown (1−ballSlowPerContact)^contacts, floored at
   * ballSlowMin so the cube NEVER stops (no soft-lock) — it always grinds through and
   * accelerates back out once the pile thins. 1 = no pile / clear. */
  _ballResistFactor(contacts) {
    if (!contacts) return 1;
    const f = Math.pow(1 - TUNE.ballSlowPerContact, contacts);
    return Math.max(TUNE.ballSlowMin, f);
  }

  /** Refresh the renderer's [{x,y,r}] views from the flat arrays (reused objects, no
   * per-frame allocation). Called once per update() when a field exists. */
  _syncBallRenderList() {
    if (!this._ballN || !this._ballRenderList) return;
    for (let i = 0; i < this._ballN; i++) {
      const v = this._ballRenderList[i];
      v.x = this._ballX[i]; v.y = this._ballY[i]; v.r = this._ballR[i];
    }
  }

  // ── BREAKING-BLOCK + DEBRIS PHYSICS (debris IS a ball with a box render) ──
  /** Break a single standing block into its debris fragments. The fragments are ACTIVATED
   * (already pre-allocated in the block's index range) and given an OUTWARD burst — mostly
   * forward (the cube's travel +x) and UP (the "흩날림"), scattered deterministically so a
   * rebuild reproduces the same break. After this the fragments fall + litter the floor via
   * _stepBlocks' ball physics. ZERO allocation. `vCube` scales the burst (a faster smash
   * flings the chips harder). Called from _stepBlocks the frame the cube reaches the face. */
  _breakBlock(b, vCube) {
    if (b.broken) return;
    b.broken = true;
    this._brokenCount++;
    const burst = TUNE.debrisBurstSpeed;
    const upFrac = TUNE.debrisBurstUp;
    // a smashing cube adds some of its forward speed to the burst (more violent smash).
    const extra = Math.max(0, vCube) * 0.6;
    for (let i = b.debIdx0; i < b.debIdx1; i++) {
      this._debActive[i] = 1;
      // scatter the fragment within the block volume so they start spread (not co-located).
      const j = ((Math.sin(i * 12.9898 + 4.123) * 43758.5453) % 1 + 1) % 1; // [0,1)
      const j2 = ((Math.sin(i * 78.233 + 1.77) * 12543.213) % 1 + 1) % 1;   // [0,1)
      this._debX[i] = b.faceX + (0.15 + 0.7 * j) * b.w;          // across the block width
      this._debY[i] = b.topY + (0.1 + 0.8 * j2) * b.h;           // up the block height
      // outward velocity: mostly forward (+x) with a spread, and up (physics +down ⇒ -y).
      const spd = burst * (0.6 + 0.8 * j) + extra;
      this._debVX[i] = spd * (0.55 + 0.5 * j2);                  // forward (always +x ⇒ flies ahead)
      this._debVY[i] = -spd * upFrac * (0.6 + 0.9 * j);          // up (then gravity pulls it down)
    }
  }

  /** Advance the breaking-block wall + its debris by `dt` seconds. Two parts:
   *   (1) STANDING BLOCKS: while a block is intact it BARS the cube. If the cube's near edge
   *       has reached the block's near face, the block BREAKS into debris (no soft-lock — the
   *       break happens on contact, the cube then plows the rubble). The first intact block
   *       whose face is AHEAD of (or at) the cube sets `barFaceX` — the x the cube is held at
   *       this frame (so it cannot pass an unbroken block). Returns barFaceX (or +Infinity).
   *   (2) DEBRIS: the SAME light single-body physics as the balls (gravity, ground clamp,
   *       fragment-fragment separation, cube push, friction). Counts the fragments the cube
   *       is in contact with (for the rubble resistance). Pure value mutation — ZERO alloc.
   * `cubeX = -1e9` (the settle path) ⇒ no break, no push. */
  _stepBlocks(dt, cubeX, cubeY, vCube) {
    let barFaceX = Infinity;
    // (1) STANDING BLOCKS: break on contact; the nearest intact block ahead bars the cube.
    if (this._blockN) {
      const half = TUNE.debrisCubeHalf;
      const pad = TUNE.blockContactPad;
      for (let i = 0; i < this._blockN; i++) {
        const b = this._blocks[i];
        if (b.broken) continue;
        // the cube's leading edge reaches the block's near face?
        if (cubeX > -1e8 && (cubeX + half + pad) >= b.faceX) {
          // BREAK: the cube has smashed into this block. (We break even on a slow touch so a
          // creeping cube still clears the wall — no soft-lock; the burst just scales with v.)
          this._breakBlock(b, vCube);
          continue;
        }
        // still intact and ahead ⇒ it bars the cube at its near face.
        if (b.faceX < barFaceX) barFaceX = b.faceX;
      }
    }
    // (2) DEBRIS physics (ball model with a box render). Only ACTIVE fragments are stepped.
    const N = this._debN;
    if (!N) { this._debContacts = 0; return barFaceX; }
    const X = this._debX, Y = this._debY, VX = this._debVX, VY = this._debVY, R = this._debR, A = this._debActive;
    const g = TUNE.debrisGravity, fr = TUNE.debrisFriction, rest = TUNE.debrisRestitution;
    const cubeHalf = TUNE.debrisCubeHalf;
    let contacts = 0;
    const damp = Math.exp(-fr * dt);
    // integrate gravity + friction, advect, ground-clamp.
    for (let i = 0; i < N; i++) {
      if (!A[i]) continue;
      VY[i] += g * dt;
      VX[i] *= damp; VY[i] *= damp;
      X[i] += VX[i] * dt; Y[i] += VY[i] * dt;
      const surf = this.surfaceYAt(X[i]);
      if (surf != null) {
        const restY = surf - R[i];
        if (Y[i] > restY) { Y[i] = restY; if (VY[i] > 0) VY[i] = -VY[i] * rest; }
      }
    }
    // fragment-fragment separation (positional, O(N²) — N<=24). Only active pairs.
    const sepStiff = TUNE.debrisSepStiff;
    for (let i = 0; i < N; i++) {
      if (!A[i]) continue;
      for (let j = i + 1; j < N; j++) {
        if (!A[j]) continue;
        let dx = X[j] - X[i], dy = Y[j] - Y[i];
        let dist = Math.hypot(dx, dy);
        const minD = R[i] + R[j];
        if (dist < minD && dist > 1e-6) {
          const overlap = minD - dist;
          const nx = dx / dist, ny = dy / dist;
          const corr = overlap * 0.5;
          X[i] -= nx * corr; Y[i] -= ny * corr;
          X[j] += nx * corr; Y[j] += ny * corr;
          const rel = (VX[j] - VX[i]) * nx + (VY[j] - VY[i]) * ny;
          if (rel < 0) {
            const imp = -rel * sepStiff * dt * 0.5;
            VX[i] -= nx * imp; VY[i] -= ny * imp;
            VX[j] += nx * imp; VY[j] += ny * imp;
          }
        } else if (dist <= 1e-6) {
          X[j] += R[j] * 0.5; X[i] -= R[i] * 0.5;
        }
      }
    }
    // CUBE → DEBRIS push (shove the rubble ahead) + contact count.
    if (cubeX > -1e8) {
      for (let i = 0; i < N; i++) {
        if (!A[i]) continue;
        const dx = X[i] - cubeX, dy = Y[i] - cubeY;
        const reach = cubeHalf + R[i] + TUNE.debrisContactPad;
        const dist = Math.hypot(dx, dy);
        if (dist < reach) {
          contacts++;
          if (dx > -R[i]) {
            const over = (reach - dist) * TUNE.debrisPushPosFrac;   // ease chips out softly (억지 밀림 fix)
            let nx = dist > 1e-6 ? dx / dist : 1, ny = dist > 1e-6 ? dy / dist : 0;
            X[i] += nx * over; Y[i] += ny * over;
            const push = Math.max(0, vCube) * TUNE.debrisPushSpeed;
            if (push > VX[i]) VX[i] = push;
            VY[i] -= push * TUNE.debrisPushUp;
          }
        }
      }
    }
    // FINAL GROUND CLAMP (separation / push moved chips positionally — re-seat them).
    for (let i = 0; i < N; i++) {
      if (!A[i]) continue;
      const surf = this.surfaceYAt(X[i]);
      if (surf == null) continue;
      const restY = surf - R[i];
      if (Y[i] > restY) { Y[i] = restY; if (VY[i] > 0) VY[i] = 0; }
    }
    this._debContacts = contacts;
    return barFaceX;
  }

  /** Resistance slow-factor for the cube from the rubble it is grinding over. Same form as
   * the ball resistance: (1−debrisSlowPerContact)^contacts, floored at debrisSlowMin so the
   * cube NEVER stops (no soft-lock). 1 = clear / no rubble. */
  _debrisResistFactor(contacts) {
    if (!contacts) return 1;
    const f = Math.pow(1 - TUNE.debrisSlowPerContact, contacts);
    return Math.max(TUNE.debrisSlowMin, f);
  }

  /** Refresh the renderer's standing-block views from the live block list (reused objects,
   * no per-frame allocation). The only mutating field is `broken` (geometry is static). */
  _syncBlockRenderList() {
    if (!this._blockN || !this._blockRenderList) return;
    for (let i = 0; i < this._blockN; i++) {
      const v = this._blockRenderList[i], b = this._blocks[i];
      v.broken = b.broken;
      v.x = b.x; v.topY = b.topY; v.baseY = b.baseY; v.w = b.w; v.h = b.h;
    }
  }

  /** Refresh the renderer's debris views from the flat arrays (reused objects, no per-frame
   * allocation). `active` tells the renderer whether to show the fragment yet. */
  _syncDebrisRenderList() {
    if (!this._debN || !this._debRenderList) return;
    for (let i = 0; i < this._debN; i++) {
      const v = this._debRenderList[i];
      v.x = this._debX[i]; v.y = this._debY[i]; v.r = this._debR[i]; v.active = !!this._debActive[i];
    }
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
      this._vx = 0; this._vTip = 0; this._omega = 0; this._vy = 0; this._v = 0;
      this._air = false; this._airFrames = 0; this._loft = 0;
      this._grip = 0; this._gripLiveAmp = 0;
      this._footBaseY = surf; this._prevFootBaseY = surf;
      this.cube.position.x = this._x;
      this.cube.position.y = this._bodyY;
      this.cube.velocity.x = 0; this.cube.velocity.y = 0;
      this.cube.angle = 0;
      // keep the (frozen) legs glued under the floating body so they hover too.
      this._syncLegs();
      // BALL-FIELD: keep the pile settling/resting while the cube floats pre-race (the
      // cube is far away ⇒ no push). Sync the render views so the balls show during the
      // countdown too. No resistance while idle (cube isn't advancing).
      if (this._ballN) {
        this._stepBalls(dt, this._x, this.cube.position.y, 0);
        this._ballResist = 1;
        this._syncBallRenderList();
      }
      // BLOCKS: keep the standing wall + any (pre-race) debris settling while the cube
      // floats. cubeX == -1e9 ⇒ no break, no push (the cube is "far away" pre-race), so the
      // wall stays intact and upright at the start. No resistance while idle.
      if (this._blockN || this._debN) {
        this._stepBlocks(dt, -1e9, 0, 0);
        this._debResist = 1;
        this._syncBlockRenderList();
        this._syncDebrisRenderList();
      }
      return;
    }

    const drive = running && this.legDrawn && this.motorEnabled;

    // SPLIT-PATH FORK: COMMIT the route at the ENTRANCE. The instant the cube's forward x is
    // inside a fork (it just crossed x0), commit the road from the CURRENT leg shape (HOOK ⇒
    // high, else low) if not already committed. From then on the committed route IS the active
    // surface for the whole fork (surfaceYAt/_segAt/steep-gate swap to it), and a mid-fork
    // redraw can NOT switch roads (the road is ridden out — the rejoin is close ⇒ no teleport).
    // Done every step (cheap, O(1) hint) so the commit also lands on the headless/forceStart
    // path and on a leg drawn mid-fork. Only commits while DRIVING (a leg exists + running) so a
    // pre-race/idle probe never pre-commits the road before the player actually enters.
    if (drive && this._forks.length) this._commitForkAt(this._x);

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
      this._blockedBySteep = false;  // a STEEP-RAMP block (non-hook leg) ⇒ legs keep trying
      this._blockedByIce = false;    // an ICE block (non-grippy/smooth leg slips) ⇒ legs keep trying

      // climb rule: if a stairs/wall step lies just ahead, gate on reach (LONG leg needed).
      const riser = this._nextRiser(this._x, lookX + 0.5);
      // tunnel rule (INVERSE): if a low-ceiling mouth lies just ahead, gate on reach
      // (SHORT leg needed). A too-long leg's rotating sweep strikes the ceiling, so we
      // stop it at the mouth (struggle-in-place), the user must redraw shorter.
      const tunnel = this._nextTunnel(this._x, lookX + 0.5);
      // ALSO block a too-long leg that is ALREADY INSIDE a tunnel (e.g. entered short, then redrawn
      // LONG mid-tunnel): the ahead-scan misses it (the mouth is now behind), so without this the
      // leg jams on the ceiling but the body keeps SLIDING forward on momentum (the user's bug /
      // the "auto-push" they want gone). Treat "inside a tunnel with reach > clearance" as a block;
      // the jam then kills the forward momentum (recoil) so the body stops instead of sliding.
      const curTun = (seg && seg.kind === 'tunnel') ? seg : null;
      const inTunnelBlocks = !!curTun && !this.canPassTunnel(reach, curTun.clearance);
      const tunnelBlocks = (tunnel && !this.canPassTunnel(reach, tunnel.clearance)) || inTunnelBlocks;
      // STEEP-CLIMB rule (by SHAPE, not length): if a steep UPHILL foot lies just ahead
      // — a steep RAMP foot OR a steep-gated STAIRCASE run foot — gate on HOOK-NESS. A
      // HOOK leg grips-and-steps over (climbs); a NON-HOOK leg (straight / arc / circle /
      // wheel) slips and STRUGGLES in place at the foot (no advance) until a hook is drawn.
      // NO reach involvement: the hook gate SUPERSEDES the per-riser length gate here.
      const steep = this._nextSteep(this._x, lookX + 0.5);
      const steepBlocks = steep && !this._isHook;
      // ICE rule (by SHAPE): if a slippery ICE foot lies just ahead, gate on GRIPPINESS. A
      // TOOTHED zigzag (아이젠) crosses; a smooth/hook leg slips and STRUGGLES in place at the
      // ice mouth (legs spin, no advance) until a spiky leg is drawn. Like steep, no reach.
      const ice = this._nextIce(this._x, lookX + 0.5);
      const iceBlocks = ice && !this._isGrippy;
      // SUPPRESS the per-riser LENGTH gate when the riser belongs to a STEEP-GATED STAIRCASE
      // run: on a steep staircase the HOOK gate is the only thing that matters (a non-hook is
      // already stopped at the foot by `steep`; a hook of ANY reach must climb — the tall step
      // edges are its grip points). Without this a short hook would ALSO trip _blockedByRiser
      // (double-gating). A GENTLE (ungated) staircase keeps its normal length gate unchanged.
      const riserOnSteepStair = riser && this._onSteepStairRun(riser.x);
      const riserBlocks = riser && !riserOnSteepStair && !this.canClimb(reach, riser.h);
      // whichever gate's stop-point comes FIRST along x wins (so a wall just before a
      // tunnel, or a steep climb before a wall, blocks at the nearer obstacle).
      const riserStopX = riserBlocks ? (riser.x - CUBE_SIZE * 0.5) : Infinity;
      // A too-long leg is stopped FAR ENOUGH BACK that its rotating sweep (radius ≈ reach) cannot
      // poke forward-up INTO the tunnel ceiling region [x0,x1] — guaranteeing ZERO ceiling
      // penetration (the user's hard rule) while it JAMS at the mouth. (The θ-clamp in section (b)
      // is the belt-and-braces backstop.) Bound below by the old mouth offset so a barely-too-long
      // leg still stops right at the mouth, not absurdly far.
      const tunnelStopX = !tunnelBlocks ? Infinity
        : tunnel ? Math.min(tunnel.x - CUBE_SIZE * 0.5 - TUNE.tunnelEnterGap,
                            tunnel.x - (this._reach + LEG_LINE_RADIUS + TUNE.tunnelEnterGap))
        : this._x;   // already INSIDE the tunnel ⇒ stop right here (no further slide)
      const steepStopX = steepBlocks ? (steep.x - CUBE_SIZE * 0.5 - TUNE.steepEnterGap) : Infinity;
      const iceStopX = iceBlocks ? (ice.x - CUBE_SIZE * 0.5 - TUNE.iceEnterGap) : Infinity;
      if (steepBlocks && steepStopX <= riserStopX && steepStopX <= tunnelStopX && steepStopX <= iceStopX) {
        // BLOCKED by the steep ramp: the leg is not a hook — it can't grip the slope, so
        // it slips and churns in place at the ramp foot (struggle, NO net advance) until
        // a HOOK leg is drawn. Not a soft-lock: redraw a hook ⇒ immediately climbs.
        v = 0;
        this._blocked = true;
        this._blockedBySteep = true;
        if (this._x > steepStopX) this._x = steepStopX;
      } else if (tunnelBlocks && tunnelStopX <= riserStopX && tunnelStopX <= iceStopX) {
        // BLOCKED by the low ceiling: stop just before the tunnel mouth. The leg is too
        // long — it churns in place (struggle) and makes NO net forward progress until a
        // SHORTER leg is drawn (canPassTunnel). No artificial advance, not a soft-lock.
        v = 0;
        this._blocked = true;
        this._blockedByTunnel = true;
        if (this._x > tunnelStopX) this._x = tunnelStopX;
      } else if (iceBlocks && iceStopX <= riserStopX) {
        // BLOCKED by ICE: the leg is smooth (or a hook) — it finds no grip and slips, churning
        // in place at the ice mouth (legs SPIN like wheels on ice, NO net advance) until a
        // TOOTHED zigzag (아이젠) leg is drawn (_isGrippy). Not a soft-lock: redraw spiky ⇒ crosses.
        v = 0;
        this._blocked = true;
        this._blockedByIce = true;
        if (this._x > iceStopX) this._x = iceStopX;
      } else if (riserBlocks) {
        // blocked: stop just before the riser (a GENTLE staircase/wall step too tall for this
        // reach). A steep-gated staircase riser is NOT a riserBlocks (the hook gate governs it
        // — see riserOnSteepStair above), so this is only the length gate of ungated steps.
        v = 0;
        this._blocked = true;
        this._blockedByRiser = true; // a step we can't clear — struggle against it
        if (this._x > riser.x - CUBE_SIZE * 0.5) {
          this._x = riser.x - CUBE_SIZE * 0.5;
        }
      } else if (seg) {
        if (seg.kind === 'ramp' && seg.steepGate && !seg.gap && this._isHook) {
          // climbing a STEEP ramp WITH a hook leg — grip-and-step pace (like stairs).
          terrain = TUNE.stairClimbSlow;
        } else if (seg.kind === 'ramp' || seg.kind === 'bumps'
                   || (seg.kind === 'fork' && typeof seg.slope === 'number')) {
          // slope > 0 means descending (topY increases downhill since y is +down…
          // careful: y +down so going UP is slope<0). Use sign of slope. The GAP's
          // descent/ascent ramps, the BUMPS sub-ramps AND the FORK low-route dip ramps
          // (underpass valley) ride this same blend, so a leg slows up the dip exit and
          // speeds down into it (a short leg crawls, a long leg rolls) — no soft-lock.
          terrain = seg.slope < 0 ? lerpSlow(seg.slope, TUNE.uphillSlow) : lerpFast(seg.slope, TUNE.downhillFast);
        } else if (seg.kind === 'wall') {
          // climbing the steep wall face (a climbable long leg) — slow, like stairs.
          terrain = TUNE.stairClimbSlow;
        } else if (seg.kind === 'stairs' || (riser && this.canClimb(reach, riser.h))) {
          // climbing stairs (gentle = any leg by length; steep = a HOOK leg whose grip on the
          // tall step edges carried it past the foot gate above) — grip-and-step pace. On a
          // steep-gated staircase a non-hook leg never reaches this branch (it is stopped at
          // the foot by `steepBlocks`), so only a hook rides the steep stairs hypotenuse up.
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

      // BALL-FIELD RESISTANCE: if the cube is shoving a pile of dynamic balls, slow it
      // by the resistance factor (∝ how many it is in contact with, floored so it never
      // soft-locks). This is the ONLY effect the balls have on the cube — the leg推進
      // (drive), no-slip ω, body height / contact are all unchanged, so every other gate
      // and invariant is intact. `_ballResist` is updated AFTER the ball step below using
      // THIS frame's contact count; applying last frame's factor here is a 1-step lag of
      // no consequence (sub-16ms). 1 = clear (no slowdown).
      v *= this._ballResist;

      // BREAKING-BLOCK RUBBLE RESISTANCE: once a block is broken its debris litters the floor
      // and SLOWS the cube grinding over it — same additive resistance as the balls (∝ how
      // many fragments the cube is in contact with, floored so it never soft-locks). The
      // standing-block BAR (cube held at an intact block's face) is applied as an x-clamp on
      // the advance below; the break itself happens in _stepBlocks at the END of this frame.
      // `_debResist` is last frame's factor (1-step lag of no consequence). 1 = clear.
      v *= this._debResist;

      // FIX ㉣ — GRIP-CLIMB GAIT: while a HOOK drives a steep-gated stair climb, ease the leg
      // phase-warp DEPTH in (and out at the boundaries). The warp REDISTRIBUTES this frame's
      // motion in time — the body DWELLS (advances slowly) while the foot is planted at a tread
      // edge, then PULLS UP/forward quickly during the leg's reach to the next edge. We compute
      // the warp from the CURRENT (start-of-frame) θ and apply the SAME factor to BOTH the forward
      // advance (here) AND the θ rotation (section b), so the planted foot stays put (no-slip
      // PRESERVED: foot world speed = v_body − ω·r = v·warp − (v/r·warp)·r ≈ 0 at every phase). The
      // warp's MEAN over a half-stride is EXACTLY 1, so the per-stride distance is UNCHANGED — the
      // climb advances at the same average pace and ALWAYS reaches the top (no stall, no soft-lock).
      // STAIR-CLIMB CADENCE: dwell-and-reach (한 칸씩 짚고) on ANY ascending staircase — gentle or
      // steep, ANY leg shape (was: steep-gated + hook only, which left gentle stairs gliding). The
      // dwell-reach warp is mean-1 over a stride ⇒ no-slip & climb pace are unchanged regardless of
      // leg shape; it just redistributes each stride into a HOLD (foot planted on a tread) + a quick
      // REACH to the next tread edge — the reference's tread-by-tread climb.
      const gripGate = TUNE.gripGaitEnable && v > 1e-6 && this._onStairRun(this._x);
      {
        const depthTarget = gripGate ? TUNE.gripDwell : 0;
        const ag = 1 - Math.exp(-TUNE.gripGaitLerp * dt);
        this._gripGaitLive += (depthTarget - this._gripGaitLive) * ag;
        if (this._gripGaitLive < 1e-4) this._gripGaitLive = 0;
      }
      this._gripWarp = this._gripPhaseWarp(this._theta, this._gripGaitLive);
      this._gripDwelling = (this._gripGaitLive > 1e-3) && (this._gripWarp < 0.85);
      v *= this._gripWarp; // dwell (slow) at the plant, reach (fast) between — mean-1 ⇒ same avg pace

      // 3. advance — FORGIVING MOMENTUM (관성). `v` here is the MOTOR TARGET speed for this frame
      //    (0 when blocked/idle; lowered by ball/debris resistance ⇒ the strong motor plows but
      //    SLOWS). The cube's ACTUAL forward speed `_v` EASES toward it, so speed BUILDS UP over
      //    continuous running and ramps from rest (inertia), and bleeds via drag when the motor
      //    eases. The leg ω (section b) is driven by the motor target, so while `_v` lags the foot
      //    SLIPS (ω·r > _v = wheel-spin); at steady state `_v` catches up and the foot grips.
      //    Capped (forgiving — no runaway). [backward-recoil on a hard jam: a later step.]
      {
        // JAM RECOIL (뒤로 살짝): the frame the leg first JAMS into geometry (tunnel ceiling / a step
        // it can't climb), bounce the cube back a touch, then it eases back to rest. Otherwise:
        // ── TIP-DRIVEN PROPULSION (발끝 추진) ── the motor only PROPELS while a foot TIP is planted on
        // the ground (plant phase: a leg straight down, cos(2θ)≈1). Two legs 180° out ⇒ a tip plants
        // twice per rotation ⇒ propulsion PULSES with the gait (surge on each footfall, coast between)
        // — forward on tip-contact, not on a mid-leg/body touch. Drag always applies; floored so a
        // planted stride always bites.
        // SMOOTH GAIT MOMENTUM (continuous, alternating-foot push — NOT a stuttering pulse): the
        // cube's speed EASES toward the motor target (the back foot's push hands off to the front
        // foot, so propulsion is continuous — never a dead gap), bleeding via drag when the motor
        // eases. Builds from rest (inertia ⇒ 점점 빨라짐). The leg ω (section b) TRACKS this _v so
        // the gripping feet (no-slip) drive it continuously — gait-driven, not a steady force, not
        // a gappy pulse (the user: '속도가 죽거나 멈칫거리는건 안 돼').
        const jammedNow = this._blockedByTunnel || this._blockedByRiser;
        if (jammedNow && !this._wasJammed) {
          this._v = -TUNE.jamRecoil;                                 // JAM IMPACT — small backward bounce
        } else {
          const tgt = v;
          // GAIT-DRIVEN PUSH (planted-foot, alternating, CONTINUOUS — not a steady force, not a
          // gappy pulse): scale the propulsion by how well a foot is PLANTED right now. _supportDepth
          // is the MAX of the two legs' down-reach, so as the back foot lifts the front foot is
          // already planting (hand-off) ⇒ it stays high and only RIPPLES → continuous, no stutter,
          // yet the force comes from the pushing foot. Floored so it never fully gaps.
          const sup = this._supportDepth(this._theta, this._angle);
          const stance = clamp01(sup / Math.max(1e-4, this._reach + LEG_LINE_RADIUS));
          const stanceGate = TUNE.pushStanceFloor + (1 - TUNE.pushStanceFloor) * stance;
          const rate = (tgt > this._v) ? (TUNE.momAccel * stanceGate) : TUNE.momDrag;  // gait-modulated accel; drag always
          this._v += (tgt - this._v) * (1 - Math.exp(-rate * dt));
        }
        this._wasJammed = jammedNow;
        if (this._v > TUNE.momMaxV) this._v = TUNE.momMaxV;
        if (this._v < -TUNE.jamRecoil) this._v = -TUNE.jamRecoil;    // allow a SMALL backward recoil, bounded
        if (Math.abs(this._v) < 1e-4) this._v = 0;
      }
      const adv = this._v * dt;
      let nextX = this._x + adv;
      // STANDING-BLOCK BAR: do not let the cube pass the near face of an INTACT block this
      // frame. We clamp the advance so the cube's leading edge sits AT the nearest intact
      // block's face (cube centre == faceX − half). _stepBlocks (END of this frame) then sees
      // the contact (cubeX + half + pad >= faceX) and BREAKS the block the SAME frame, so the
      // cube plows through with no perceptible stall. This is NOT a soft-lock and never lets
      // the cube penetrate an intact block. (Includes the blockContactPad slack so the clamp
      // and the break trigger agree to within a chip.)
      if (this._blockN) {
        const half = TUNE.debrisCubeHalf;
        let barStopX = Infinity;
        for (let i = 0; i < this._blockN; i++) {
          const b = this._blocks[i];
          if (b.broken) continue;
          const stopX = b.faceX - half; // cube centre x where its leading edge meets the face
          if (stopX < barStopX) barStopX = stopX;
        }
        if (barStopX < Infinity && nextX > barStopX) nextX = Math.max(this._x, barStopX);
      }
      if (nextX < this.finishX + 1) this._x = nextX;
    } else {
      // not driving (idle/blocked-no-leg): bleed momentum to rest.
      this._v += (0 - this._v) * (1 - Math.exp(-TUNE.momDrag * dt));
      if (Math.abs(this._v) < 1e-4) this._v = 0;
    }
    this._vx = this._v;   // report the ACTUAL (momentum) speed — drives cube.velocity + ball/debris push

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
    // FIX ㉣ — GRIP-CLIMB GAIT GATE (reused below for the grip-lift + stepped body ascent): true
    // while a HOOK is actually driving a steep-gated stair climb. The leg phase-WARP itself was
    // already eased + applied to `v` in the advance step above; here ω is derived from that ALREADY-
    // WARPED `v`, so `_theta += ω·dt` reproduces the SAME dwell-reach phase rate as the body — the
    // planted foot stays put (no-slip preserved) and the climb pace is unchanged (mean warp = 1).
    const gripping = drive && this._onStairRun(this._x) && this._gripGaitLive > 1e-3;
    if (drive && (v > 1e-6 || this._v > 1e-6)) {
      // NO-SLIP GRIP: spin the leg to match the cube's ACTUAL momentum speed `_v` (not the motor
      // target), so the planted foot's world speed = v_body − ω·r ≈ 0 — the foot GRIPS the ground
      // firmly instead of spinning faster than the body (the flat-ground "헛돎" the user removed).
      // Momentum + tip-propulsion still live in `_v` (section 3); the leg just tracks it. Spin-in-
      // place (real 헛돎) remains ONLY in the blocked branch below (steep slope, no grip).
      const vSurf = this._v / cosA;
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
      // ω from the (already grip-warped) v ⇒ the leg DWELLS at the plant and REACHES between,
      // EXACTLY in step with the warped forward advance ⇒ the planted foot does not slip and the
      // visible leg motion is the hold-reach-hold "짚고" rhythm (NOT a constant roll) on the climb.
      omega = vSurf / (ryContact * stretch);
      this._theta += omega * dt;
      this._vTip = omega * ryContact; // == v_surface at a plant (planted foot stationary)
      this._trying = false;
    } else if (drive && (this._blockedByRiser || this._blockedByTunnel || this._blockedBySteep || this._blockedByIce) && this._reach > CUBE_SIZE * 0.5) {
      // BLOCKED — two flavors of "physics":
      //  • JAM (tunnel: a too-long leg hits the ceiling; riser: the leg can't reach over a step) —
      //    the leg is physically CAUGHT on geometry ⇒ the MOTOR STALLS: ω decays to ~0 so the leg
      //    HOLDS against the obstacle (끼임-정지), it does NOT free-roll through it. Redraw a fitting
      //    leg to free it (no soft-lock). A small backward recoil on the impact is applied in (3).
      //  • SLIP (steep ramp, non-hook / ice): the leg finds NO grip ⇒ it SPINS (헛돎) at the natural
      //    cadence while the body makes no forward progress (wheel spinning on the slope/ice).
      const jammed = this._blockedByTunnel || this._blockedByRiser;
      if (jammed) {
        omega = (this._omega || 0) * Math.exp(-TUNE.jamMotorStop * dt);  // motor stalls — ease the spin to a stop, leg held
        this._theta += omega * dt;
        this._vTip = 0;
        this._trying = false;     // it is JAMMED/held, not churning
      } else {
        const vNat = TUNE.baseSpeed * this.legSpeedFactor(this._reach) * this.paceFactor;
        const ryContact = Math.max(TUNE.effRadiusMin, this._supportDepth(this._theta, this._angle));
        omega = vNat / ryContact;   // SLIP: legs spin, no grip, no net progress
        this._theta += omega * dt;
        this._vTip = 0;
        this._trying = true;
      }
      this._gripWarp = 1; this._gripDwelling = false; // no climb gait while blocked
    } else {
      this._vTip = 0;
      this._trying = false;
      this._gripWarp = 1; this._gripDwelling = false;
    }
    this._omega = omega; // rad/s — recorded on each leg.body in _syncLegs
    // (CEILING JAM is enforced after the body Y is finalized — see the clamp just before _syncLegs.)

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
    // ── STEEP-STAIR GRIP CADENCE (짚고 — the hook PLANTS on each tread & pulls the body up) ──
    // While the cube is HOOK-climbing a steep-gated staircase, add a small UPWARD pulse that
    // peaks at each foot-PLANT (the instant the hook catches a step edge). gripPulse = (1−hop)
    // is 1 at a plant (θ=0,π…) and 0 mid-float, so the body HITCHES UP as the hook bites and
    // settles between bites — it READS as gripping/stepping tread-by-tread instead of a smooth
    // glide. The amplitude eases in/out at the run boundaries (no pop) and is small (no
    // jitter / motion-sickness). It is UPWARD-ONLY (RAISES the body), so it can never push the
    // body INTO a riser ⇒ zero penetration is structurally preserved. Active only when a HOOK
    // is actually driving a steep-stair climb (`gripping`, computed once in section (b) above).
    {
      const gripTarget = gripping ? TUNE.gripLiftMax : 0;
      const ag = 1 - Math.exp(-TUNE.gripLerp * dt);
      this._gripLiveAmp += (gripTarget - this._gripLiveAmp) * ag;
      if (this._gripLiveAmp < 1e-4) this._gripLiveAmp = 0;
      const gripPulse = 0.5 * (1 + Math.cos(2 * this._theta)); // 1 at plant, 0 mid-float
      this._grip = this._gripLiveAmp * gripPulse;
    }
    // ── FIX ㉡ — STEPPED body ascent (계단을 한 칸씩 짚고 올라가기) ──
    // On a steep-gated stair run a HOOK is climbing tread-by-tread. The smooth grounded glide
    // (_groundedCubeY) read as a SLIDE up the hypotenuse, so here we drive the body's STAND LEVEL
    // in DISCRETE TREAD STEPS: the body HOLDS at the tread it stands on, then STEPS UP to the next
    // tread the instant the hook PLANTS (the phase peak). The step-up is EASED over a short time
    // (no jitter) but the rhythm stays clearly visible (hold→step→hold→step). It is anchored to the
    // REAL tread tops (a discrete level the cube has reached) and CLAMPED never below the grounded
    // pose, so the foot is ALWAYS on/above the current tread ⇒ structurally penetration-free.
    // STEPPED-CLIMB COMMIT LEVEL (한 칸씩): on a steep-gated stair run a HOOK is climbing
    // tread-by-tread. We track a DISCRETE committed tread level `_stepCommitY` that advances ONE
    // tread at each foot-PLANT and HOLDS between plants. The camera base then RISES quickly to that
    // level after a plant and HOLDS (the relaxed step-slew), so on screen the climb reads as "step
    // up, hold, step up, hold" instead of a smooth glide. The committed level is a REAL discrete
    // tread top the cube has reached (or one ahead), so the cube body (grounded pose) is always
    // on/above it ⇒ penetration-free. `stepActive` gates the relaxed camera slew below.
    let stepActive = false;
    if (TUNE.stepClimbEnable && gripping) {
      stepActive = true;
      const realSurf = this.surfaceYAt(this._x);          // current tread top under the body (physics +down)
      const treadTop = (realSurf != null) ? realSurf : groundSurf;
      // FIX ㉣ — LEG LENGTH → TREADS-PER-PLANT (the user's "다리 길이에 따라 몇 번째 계단을 밟는다"):
      // a SHORT leg plants on the IMMEDIATE next tread (one ahead), a LONG leg REACHES over MORE
      // treads (its lookahead extends with reach), so each plant of a long leg lands on a higher/
      // farther tread edge — a bigger climbing stride. `nReach` whole footholds; the lookahead is
      // a cube-width per foothold so the sampled surface is a real tread top `nReach` steps up.
      const nReach = this._gripTreadsPerPlant(this._reach);
      // the tread the cube will climb ONTO next: sample the surface `nReach` cube-widths ahead (a
      // long leg looks farther). It is a REAL tread top (surfaceYAt is the stepped staircase), so the
      // committed level is always a genuine foothold — the body lands ON it (penetration-free).
      const aheadSurf = this.surfaceYAt(this._x + CUBE_SIZE * 0.8 * nReach);
      const nextTreadTop = (aheadSurf != null && aheadSurf < treadTop) ? aheadSurf : treadTop;
      // ALSO clamp how far the commit may lead the real ground beneath the body — never more than the
      // tread the cube is GROUNDED on (so the body never floats above a real foot contact ⇒ no float
      // bug). For the long-leg multi-tread reach we allow the commit to lead up to the `nReach`-ahead
      // tread, which the FOOT (a long leg) physically reaches.
      if (!this._stepClimbActive) {
        this._stepClimbActive = true;
        this._stepCommitY = treadTop;                      // seed at the current tread (no pop)
        this._stepLevelY = treadTop;
        this._stepPlantArmed = true;
      }
      // PLANT EDGE (cos(2θ) peaks ≈1 at θ≈0,π — a leg straight down catching a step edge): on the
      // rising edge of a plant, COMMIT up to `nReach` treads UP (to nextTreadTop — a SHORT leg's
      // nextTreadTop is one tread up, a LONG leg's is several), then HOLD until the next plant
      // (hysteresis arms between plants). A long leg therefore grabs MORE steps per plant.
      const plantCos = 0.5 * (1 + Math.cos(2 * this._theta)); // 1 at plant
      if (plantCos >= TUNE.stepPlantPhaseCos && this._stepPlantArmed) {
        if (nextTreadTop < this._stepCommitY) this._stepCommitY = nextTreadTop; // step UP (more-negative y); never DOWN
        this._stepPlantArmed = false;
      } else if (plantCos < TUNE.stepPlantPhaseCos * 0.6) {
        this._stepPlantArmed = true;                       // re-arm in the float between plants
      }
      // The commit changes ONLY at a plant (above) ⇒ it HOLDS perfectly flat between plants, so the
      // camera reaches it and STOPS (a clean flat hold) until the next plant jumps it up — the crisp
      // "한 칸씩" cadence. We only guard it from running AHEAD of the next reachable tread (the leg's
      // `nReach`-ahead foothold) and never DROP below the tread the cube has reached (no sag).
      if (this._stepCommitY < nextTreadTop) this._stepCommitY = nextTreadTop; // never lead beyond the leg's reach
      if (this._stepCommitY > treadTop) this._stepCommitY = treadTop;         // never sag below the reached tread (the cube already stands here)
      // The stand level is the DISCRETE committed tread — it JUMPS at a plant and HOLDS flat between
      // plants (it does NOT continuously ease, or the holds would smear back into a glide). The
      // camera base (below) then provides the eased rise to it, so each step = a quick camera rise
      // then a flat hold (한 칸씩), while the discrete commit keeps the rhythm crisp.
      this._stepLevelY = this._stepCommitY;
    } else if (this._stepClimbActive) {
      this._stepClimbActive = false;
      this._stepCommitY = null;
      this._stepLevelY = null;
      this._stepPlantArmed = true;
    }
    // body sits at the grounded (foot-grazing) pose RAISED by the loft AND the grip-cadence hitch
    // (physics +down ⇒ subtract to raise). Both lifts are upward-only ⇒ the foot never dips below
    // the surface (no-penetration by construction). The DISCRETE stepping is carried by the camera
    // base (below) so the on-screen climb reads as steps; the cube body keeps grazing the real
    // treads (so the foot stays planted, no float-bug).
    this._stepProfile = stepActive ? (this._stepLevelY - reachR - this._bodyBaseY) : 0; // signed: how far the held tread leads the camera (>0 in physics-y = camera below the held tread)
    this._bodyY = groundedY - this._loft - this._grip;
    // vertical velocity of the body (for cube.velocity / render) — the loft's rate.
    this._vy = -(this._loft - (this._prevLoft || 0)) / Math.max(1e-4, dt);
    this._prevLoft = this._loft;
    // "airborne" === the foot is legitimately lifted clear of the surface in THIS stride
    // phase — either by the downhill gait LOFT (hop) or by the steep-stair GRIP pull-up (the
    // hook bites a step edge and the body rises off the tread). Legs STILL roll/grip
    // throughout (ω>0); this flag only tells the verifier the foot is intentionally off the
    // ground (so it skips the contact-gap / slip / penetration checks for these frames — they
    // are a deliberate stride phase, not a float bug). The grip lift is upward-only, so it
    // NEVER drives the foot BELOW the surface (zero penetration is still asserted everywhere).
    const wasAir = this._air;
    this._air = (this._loft > LOFT_AIR_EPS) || (this._grip > LOFT_AIR_EPS);
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
      // FIX ㉡: on the stepped steep-stair climb the camera base FOLLOWS the DISCRETE committed
      // tread level (so the whole cube visibly STEPS up tread-by-tread on screen, not just the
      // in-frame body), with a RELAXED slew (stepCamSlewMax) so it can rise a tread quickly right
      // after a plant and then HOLD until the next plant — the "한 칸씩" cadence. Off a stepped
      // climb it tracks the smooth grounded trend with the normal tight slew (no behaviour change).
      const baseTargetY = stepActive ? (this._stepLevelY - reachR) : (groundSurf - reachR);
      const lerpRate = stepActive ? TUNE.stepCamLerp : TUNE.surfaceLerp;
      const a = 1 - Math.exp(-lerpRate * dt);
      let step = (baseTargetY - this._bodyBaseY) * a;
      const slewCap = stepActive ? TUNE.stepCamSlewMax : TUNE.surfaceSlewMax;
      step = clampMag(step, slewCap); // bounded slew → no harsh snap (relaxed during a stepped climb so the step shows)
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

    // ── CEILING JAM (no penetration + 끼임) ── with the body Y now FINAL, if the current spin angle
    //    pushes any leg point UP INTO a tunnel ceiling, ROLL the spin BACK a hair at a time until it
    //    clears: the leg HOLDS against the ceiling and never rolls THROUGH it (the user's hard rule).
    //    Next frame the motor advances θ again → re-clamps here ⇒ the leg oscillates at the contact =
    //    a JAM (redraw a shorter leg to pass — no soft-lock). Bounded iterations; zero alloc.
    if (this.legDrawn && this._chain && this._maxCeilingPen(this._theta, this._angle) > 0.001) {
      // Roll θ toward whichever side REDUCES the ceiling penetration ⇒ converge to the nearest
      // angle where the leg fits under the ceiling (e.g. pointing ALONG the tunnel). Bidirectional
      // so it always finds a clear angle even when the leg was redrawn deep into the ceiling
      // mid-tunnel (the one-direction roll could climb away from the fit). Bounded iterations.
      let guard = 0;
      while (this._maxCeilingPen(this._theta, this._angle) > 0.001 && guard < 90) {
        const penPlus = this._maxCeilingPen(this._theta + TUNE.ceilBackStep, this._angle);
        const penMinus = this._maxCeilingPen(this._theta - TUNE.ceilBackStep, this._angle);
        this._theta += (penMinus <= penPlus ? -TUNE.ceilBackStep : TUNE.ceilBackStep);
        guard++;
      }
      this._omega = 0;   // motor stalled — leg jammed at the ceiling
    }

    // 6. sync the two leg visuals + their world parts. Foot lowest point is
    //    clamped to sit ON the surface (never below) — structural 0 penetration.
    this._syncLegs();

    // 7. BALL-FIELD: step the dynamic pile with THIS frame's cube position + forward
    //    speed so the cube shoves the balls aside, then recompute the resistance factor
    //    (applied to v NEXT frame — a 1-step lag of no consequence). The push uses the
    //    cube's realized forward velocity (v) so a faster cube flings them harder. The
    //    render views are refreshed for the sphere meshes. The ball system is purely
    //    additive: it never touches _x / _bodyY / legs / gates beyond the v slowdown.
    if (this._ballN) {
      const contacts = this._stepBalls(dt, this._x, this.cube.position.y, v);
      this._ballResist = this._ballResistFactor(contacts);
      this._syncBallRenderList();
    }

    // 8. BREAKING BLOCKS: step the standing wall + its debris with THIS frame's cube position
    //    + forward speed. _stepBlocks (a) BREAKS any intact block the cube's leading edge has
    //    reached (turning it into bursting debris), and (b) advances the debris with the ball
    //    physics (gravity / ground clamp / separation / cube push), counting how many chips
    //    the cube is shoving for the rubble resistance (applied to v NEXT frame). Like the
    //    balls this is purely additive beyond the standing-block bar (clamped on _x above).
    if (this._blockN || this._debN) {
      this._stepBlocks(dt, this._x, this.cube.position.y, v);
      this._debResist = this._debrisResistFactor(this._debContacts);
      this._syncBlockRenderList();
      this._syncDebrisRenderList();
    }

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
    // points down past the surface), so the clamp must NOT yank it down to ground. NOTE the
    // STEEP-STAIR GRIP lift is NOT skipped here: it is upward-only (it can never push a foot
    // BELOW the surface), so we KEEP the up-only clamp running through a grip pull-up — a
    // belt-and-braces guarantee of ZERO penetration on the steep climb.
    if (this._loft > LOFT_AIR_EPS || this._idleFloat) return;
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

  /** RENDER INTERPOLATION getters — the renderer reads these (with the game's alpha)
   * INSTEAD of the raw cube/leg fields, so the drawn pose is a continuous lerp between
   * the previous and current sim ticks even when rAF jitters (the micro-stutter fix).
   * alpha ∈ [0,1] is the leftover-accumulator ratio (game exposes _acc/FIXED_DT). When
   * there is no valid prev snapshot (first frame / after a reset) we fall back to the
   * CURRENT value (no lerp) so the pose is always sane. The leg world angle = θ + the
   * leg's phaseOffset + the body tilt — we interpolate θ and tilt and recombine, so the
   * caller passes the per-leg phaseOffset. Angles use a plain lerp (per-tick deltas are
   * tiny so wraparound is a non-issue at the render alpha). */
  interpX(alpha) {
    if (!this.cube) return this.startX;
    if (!this._interpHasPrev) return this._x;
    const a = this._interpPrev;
    return a.x + (this._x - a.x) * alpha;
  }
  interpBodyY(alpha) {
    if (!this.cube) return 0;
    if (!this._interpHasPrev) return this._bodyY;
    const a = this._interpPrev;
    return a.bodyY + (this._bodyY - a.bodyY) * alpha;
  }
  interpBodyCamY(alpha) {
    if (!this.cube) return 0;
    const cur = Number.isFinite(this._bodyBaseY) ? this._bodyBaseY : this._bodyY;
    if (!this._interpHasPrev) return cur;
    const a = this._interpPrev;
    return a.bodyBaseY + (cur - a.bodyBaseY) * alpha;
  }
  interpAngle(alpha) {
    if (!this.cube) return 0;
    if (!this._interpHasPrev) return this._angle;
    const a = this._interpPrev;
    return a.angle + (this._angle - a.angle) * alpha;
  }
  /** Plant "쿵" squash amount [0..plantSquash] for the render frame — peaks at each foot-PLANT
   * (cos(2θ)≈1) and is ~0 between plants, faded in with forward speed. The renderer reads this
   * and SCALES the cube mesh (flatter on impact) for a weighted footfall. Derived from the SAME
   * interpolated θ the legs use, so the squash stays in lock-step with the visible footfall under
   * rAF jitter. COSMETIC — never touches the physics body/foot (no penetration / gate effect). */
  interpSquash(alpha) {
    if (!this.cube) return 0;
    let th;
    if (!this._interpHasPrev) th = this._theta;
    else { const a = this._interpPrev; th = a.theta + (this._theta - a.theta) * alpha; }
    const pulse = 0.5 * (1 + Math.cos(2 * th));          // 1 at a foot-plant (leg straight down), 0 mid-stride
    const moving = clamp01(Math.abs(this._vx || 0) / TUNE.plantSquashSpeedRef);
    return TUNE.plantSquash * Math.pow(pulse, TUNE.plantSquashPow) * moving;
  }
  /** Interpolated world leg angle for a leg with the given phaseOffset: lerp θ and the
   * body tilt, then add phaseOffset (which is constant — it never changes per tick). */
  interpLegAngle(alpha, phaseOffset) {
    if (!this.cube) return phaseOffset || 0;
    if (!this._interpHasPrev) return this._theta + (phaseOffset || 0) + (this._angle || 0);
    const a = this._interpPrev;
    const theta = a.theta + (this._theta - a.theta) * alpha;
    const tilt = a.angle + (this._angle - a.angle) * alpha;
    return theta + (phaseOffset || 0) + tilt;
  }
  /** Current geometric bob excursion (render-positive UP amount the cube sits BELOW
   * the bob-free base; ≥ 0). Exposed for the verifier's (J) bob-amplitude report. */
  get bob() { return this._bob || 0; }
  get trying() { return !!this._trying; }
  /** True while blocked specifically by a low ceiling (a too-long leg at a tunnel mouth). */
  get blockedByTunnel() { return !!this._blockedByTunnel; }
  /** True while blocked specifically by a riser (a too-short leg at a wall/stairs step). */
  get blockedByRiser() { return !!this._blockedByRiser; }
  /** True while blocked specifically by a STEEP ramp (a NON-HOOK leg can't grip-climb it). */
  get blockedBySteep() { return !!this._blockedBySteep; }
  /** True while blocked specifically by ICE (a smooth/hook leg slips — needs a toothed zigzag). */
  get blockedByIce() { return !!this._blockedByIce; }
  /** Is the CURRENT drawn leg a HOOK (sharp bend ⇒ can grip-and-step over a steep ramp)?
   * SHAPE gate — independent of reach. False for straight bars / smooth arcs / circles. */
  get isHook() { return !!this._isHook; }
  /** Is the CURRENT drawn leg GRIPPY (a toothed/zigzag 아이젠 ⇒ grips ICE)? SHAPE gate — many
   * sharp corners that alternate. False for smooth shapes AND for a clean (one-way) hook. */
  get isGrippy() { return !!this._isGrippy; }

  // ── SPLIT-PATH FORK getters (renderer + verifier read these) ──
  /** Number of forks on the track (0 ⇒ none). */
  get forkCount() { return this._forks ? this._forks.length : 0; }
  /** The COMMITTED route ('high'|'low') of fork `id`, or null if not committed yet (the cube
   * hasn't entered). The verifier reads this to confirm the entrance commit + no mid-fork switch. */
  forkRoute(id) { return this._committedRoute(id); }
  /** Read-only fork records for the renderer (it draws BOTH routes always) + the verifier.
   * Each entry: { id, x0, x1, baseY, highSegs:[{x0,x1,kind,topYa,topYb}], lowSegs:[…], route }.
   * Built once; this getter assembles the light view list lazily (NOT on the per-frame hot path —
   * the renderer calls it once at buildTrack). */
  get forks() {
    if (!this._forks || !this._forks.length) return null;
    const out = new Array(this._forks.length);
    for (let i = 0; i < this._forks.length; i++) {
      const f = this._forks[i];
      // the LOW route segs for this fork live in the main _segs tagged forkId+route='low'.
      const lowSegs = this._segs.filter((s) => s.kind === 'fork' && s.forkId === f.id)
        .map((s) => ({ x0: s.x0, x1: s.x1, topYa: s.surfFn(s.x0), topYb: s.surfFn(s.x1) }));
      const highSegs = f.highSegs.map((s) => ({ x0: s.x0, x1: s.x1, kind: s.kind,
        topYa: s.surfFn(s.x0), topYb: s.surfFn(s.x1), stepH: s.stepH || 0 }));
      out[i] = { id: f.id, x0: f.x0, x1: f.x1, baseY: f.baseY, lowSegs, highSegs,
        route: this._committedRoute(f.id) };
    }
    return out;
  }
  /** Measured max turn angle (deg) of the current chain — the hookiness metric (diagnostic). */
  get maxTurnDeg() { return this._maxTurnDeg || 0; }
  /** Hook classification of an ARBITRARY normalized stroke (box [-1,1]) WITHOUT mutating
   * the live leg — used by the verifier's preset-classification table. Runs the same
   * resample+recenter+reach-clamp pipeline as setLegStroke, then measures the bend. */
  classifyHook(points, scale = 1.0) {
    if (!points || points.length < 2) return { isHook: false, maxTurnDeg: 0, reach: 0, n: 0 };
    const s = (scale || 1) * LEG_WORLD_SCALE;
    const stroke = points.map((p) => ({ x: p.x * s, y: p.y * s }));
    let chain = resamplePolyline(stroke, LEG_CIRCLE_SPACING, LEG_MAX_CIRCLES);
    if (chain.length < 2) return { isHook: false, maxTurnDeg: 0, reach: 0, n: chain.length };
    const anchor = { x: chain[0].x, y: chain[0].y };
    chain = chain.map((c) => ({ x: c.x - anchor.x, y: c.y - anchor.y }));
    let rawReach = 0; for (const c of chain) rawReach = Math.max(rawReach, Math.hypot(c.x, c.y));
    const reach = Math.max(LEG_REACH_MIN, Math.min(LEG_REACH_MAX, rawReach));
    if (rawReach > 1e-6 && Math.abs(reach - rawReach) > 1e-6) {
      const k = reach / rawReach; chain = chain.map((c) => ({ x: c.x * k, y: c.y * k }));
    }
    const hm = hookMetrics(chain, TUNE.hookTurnWindow);
    const isHook = (hm.maxTurnDeg >= TUNE.hookAngleDeg) && (hm.signReversals <= TUNE.hookMaxReversals);
    return { isHook, maxTurnDeg: +hm.maxTurnDeg.toFixed(2), signReversals: hm.signReversals,
      nSharp: hm.nSharp, reach: +reach.toFixed(3), n: chain.length };
  }
  // ── BALL-FIELD getters (renderer + verifier read these) ──
  /** Number of dynamic balls in the field (0 ⇒ none). */
  get ballCount() { return this._ballN || 0; }
  /** Reusable [{x,y,r}] views of every ball (physics y +down) for the renderer's reused
   * sphere meshes. Same array object across frames (only the field values change) ⇒ zero
   * per-frame allocation. null when there are no balls. */
  get balls() { return this._ballN ? this._ballRenderList : null; }
  /** Last-applied cube speed slow-factor from the pile (1 = clear, <1 = being shoved /
   * slowed; floored at TUNE.ballSlowMin so it never reaches 0 — no soft-lock). */
  get ballResist() { return this._ballResist; }
  /** # balls the cube was in contact with last step (diagnostic). */
  get ballContacts() { return this._ballContacts || 0; }
  /** Snapshot of ball centres as {x,y,r} (NEW objects — for the verifier's movement
   * delta; not used on the hot render path). */
  ballSnapshot() {
    if (!this._ballN) return [];
    const out = new Array(this._ballN);
    for (let i = 0; i < this._ballN; i++) out[i] = { x: this._ballX[i], y: this._ballY[i], r: this._ballR[i] };
    return out;
  }

  // ── BREAKING-BLOCK getters (renderer + verifier read these) ──
  /** Number of STANDING blocks (intact + broken) in the wall (0 ⇒ none). */
  get blockCount() { return this._blockN || 0; }
  /** Reusable [{x,topY,baseY,w,h,broken}] views of every standing block for the renderer's
   * reused box meshes (hide the mesh once `broken`). Same array object across frames. */
  get blocks() { return this._blockN ? this._blockRenderList : null; }
  /** # blocks broken (smashed) so far (diagnostic). */
  get brokenBlocks() { return this._brokenCount || 0; }
  /** Number of debris fragment slots (active + inactive) (0 ⇒ none). */
  get debrisCount() { return this._debN || 0; }
  /** Reusable [{x,y,r,active}] views of every debris fragment (physics y +down) for the
   * renderer's reused box meshes (show only `active` fragments). Same array each frame. */
  get debris() { return this._debN ? this._debRenderList : null; }
  /** Last-applied cube speed slow-factor from the rubble (1 = clear, <1 = grinding over
   * debris; floored at TUNE.debrisSlowMin so it never reaches 0 — no soft-lock). */
  get debrisResist() { return this._debResist; }
  /** # debris the cube was in contact with last step (diagnostic). */
  get debrisContacts() { return this._debContacts || 0; }
  /** Snapshot of ACTIVE debris centres as {x,y,r} (NEW objects — for the verifier's
   * movement delta; not used on the hot render path). */
  debrisSnapshot() {
    if (!this._debN) return [];
    const out = [];
    for (let i = 0; i < this._debN; i++) if (this._debActive[i]) out.push({ x: this._debX[i], y: this._debY[i], r: this._debR[i] });
    return out;
  }

  /** True while the walker is in a ballistic flight (off the ground at a crest). */
  get airborne() { return !!this._air; }
  /** Live STEEP-STAIR GRIP-cadence lift (world u, upward-only) — the per-plant hitch as the
   * hook bites each tread and pulls the body up (짚고). 0 off a steep-stair hook climb. */
  get gripLift() { return this._grip || 0; }
  /** FIX ㉡: the STEPPED-vs-glide body lift on a steep-stair hook climb (world u, >0 ⇒ the body
   * is RAISED above the smooth grounded glide because it is HOLDING on a tread between plant
   * steps). 0 off a stepped climb. The verifier reads this + bodyCamY to assert the body
   * ascends in discrete tread steps (hold-then-rise), not a straight diagonal slide. */
  get stepProfile() { return this._stepProfile || 0; }
  /** True while the body is doing the stepped (vs glide) steep-stair climb. */
  get stepClimbActive() { return !!this._stepClimbActive; }
  /** FIX ㉣: the live LEG PHASE-WARP factor (1 = constant roll; <1 = the leg DWELLS at the plant,
   * >1 = the leg REACHES between plants). On a steep-hook climb this PULSES between (1−depth) at the
   * plant and (1+depth) mid-swing (mean 1) — the verifier reads it to prove the leg does NOT rotate
   * at a constant rate during the climb (the dwell-reach "짚고" gait, not a slide). 1 everywhere else. */
  get gripWarp() { return (this._gripWarp == null) ? 1 : this._gripWarp; }
  /** FIX ㉣: true on a frame the climb leg is in its DWELL (plant-hold) part. */
  get gripDwelling() { return !!this._gripDwelling; }
  /** FIX ㉣: the eased grip-gait depth ∈ [0, gripDwell] (0 ⇒ normal roll; >0 ⇒ the dwell-reach gait is
   * active on a steep-hook climb). Diagnostic for the verifier (the gait is engaged on the climb). */
  get gripGaitDepth() { return this._gripGaitLive || 0; }
  /** FIX ㉣: how many TREAD edges the CURRENT leg grabs per plant (1 = short leg, more = long leg).
   * The user's "다리 길이에 따라 몇 번째 계단을 밟는다" — reported so the verifier proves long > short. */
  get treadsPerPlant() { return this._gripTreadsPerPlant(this._reach); }
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

/** MAX TURN ANGLE (degrees) along a polyline chain — the "hookiness" metric.
 * For each interior sample i we form two direction vectors: in = chain[i]−chain[i−w]
 * and out = chain[i+w]−chain[i] (a lookahead window `w` each side so the dense
 * resampling jitter on a smooth arc does not accumulate into a false corner). The
 * turn at i is the angle between in and out (0 = straight, 180 = a fold-back). We
 * return the MAX over all interior samples. A hook / ㄱ / L / claw has one sharp
 * (~75°+) corner; a straight bar / smooth arc / circle / wheel stays low (< ~53°).
 * Chains too short to have an interior sample (n <= 2w) return 0 (cannot be a hook). */
function maxTurnAngleDeg(chain, w) {
  return hookMetrics(chain, w).maxTurnDeg;
}

/** Shape metrics for the HOOK classifier (single source). Walks the chain measuring the
 * turn at each interior vertex (with the lookahead window `w` to smooth resample jitter)
 * and returns:
 *   • maxTurnDeg     — the SHARPEST bend anywhere (degrees) — "is there a corner at all?"
 *   • signReversals  — # of TURN-DIRECTION reversals among SHARP corners (>= sharpAngleDeg).
 *                      We take the signed turn (cross product) at each sharp vertex and count
 *                      how many times its sign flips along the chain. A genuine ㄱ/J/L curls
 *                      ONE way ⇒ 0 reversals; a zigzag alternates left-right-left ⇒ >=2. This
 *                      is the SCRIBBLE test and is LENGTH-ROBUST (a short clean hook stays 0).
 *   • nSharp         — # sharp corners measured (for transparency).
 * Zero allocation beyond the returned plain object (called only on a redraw / in the
 * verifier — never on the per-frame hot path). */
function hookMetrics(chain, w) {
  const win = Math.max(1, w | 0);
  let mx = 0, nSharp = 0, reversals = 0, prevSign = 0;
  const sharpRad = TUNE.sharpAngleDeg * Math.PI / 180;
  for (let i = win; i < chain.length - win; i++) {
    const ax = chain[i].x - chain[i - win].x, ay = chain[i].y - chain[i - win].y;
    const bx = chain[i + win].x - chain[i].x, by = chain[i + win].y - chain[i].y;
    const la = Math.hypot(ax, ay), lb = Math.hypot(bx, by);
    if (la < 1e-6 || lb < 1e-6) continue;
    let cos = (ax * bx + ay * by) / (la * lb);
    if (cos > 1) cos = 1; else if (cos < -1) cos = -1;
    const ang = Math.acos(cos);
    if (ang > mx) mx = ang;
    if (ang >= sharpRad) {
      nSharp++;
      const cross = ax * by - ay * bx;          // signed turn direction (+left / -right)
      const sign = cross > 0 ? 1 : (cross < 0 ? -1 : 0);
      if (sign !== 0) {
        if (prevSign !== 0 && sign !== prevSign) reversals++;
        prevSign = sign;
      }
    }
  }
  return { maxTurnDeg: mx * 180 / Math.PI, signReversals: reversals, nSharp };
}

/** True iff `chain` reads as a GENUINE HOOK (one clear graspable bend), NOT a scribble.
 * isHook = (maxTurnDeg >= hookAngleDeg) && (signReversals <= hookMaxReversals). See the TUNE
 * HOOK RULE comment. Single source for setLegStroke + classifyHook. */
function isHookChain(chain, w) {
  const m = hookMetrics(chain, w);
  return (m.maxTurnDeg >= TUNE.hookAngleDeg) && (m.signReversals <= TUNE.hookMaxReversals);
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
  // a SHORT genuine ㄱ hook (single dominant bend, short reach ≈0.7) — proves the hook gate
  // is by SHAPE not reach: this short hook CLIMBS the steep staircase while a long straight is
  // blocked. ONE clean ~90° corner that curls ONE way (0 direction reversals) ⇒ a HOOK; not a scribble.
  if (name === 'hook_short') return [{ x: 0.0, y: -0.5 }, { x: 0.0, y: 0.0 }, { x: 0.5, y: 0.0 }];
  // a REALISTIC hand-drawn ㄱ hook with FINGER WOBBLE — a phone-drawn stroke is densely
  // sampled with slight perpendicular jitter on the shaft and foot. A clean ㄱ (down-shaft,
  // right-foot) with small zig-zag noise (~0.05u peak). This MUST still classify as a HOOK
  // (the live-test regression was that wobble fooled the old strict gate). The single
  // dominant ~90° bend dominates; the jitter never produces >40° genuine corners ⇒ rev≤1.
  if (name === 'hook_wobble') return [
    { x: -0.10, y: -0.70 }, { x: -0.07, y: -0.55 }, { x: -0.12, y: -0.40 }, { x: -0.08, y: -0.25 },
    { x: -0.11, y: -0.10 }, { x: -0.07, y: 0.05 }, { x: -0.12, y: 0.20 }, { x: -0.08, y: 0.35 },
    { x: -0.10, y: 0.48 }, { x: 0.04, y: 0.52 }, { x: 0.18, y: 0.47 }, { x: 0.32, y: 0.53 },
    { x: 0.46, y: 0.48 }, { x: 0.60, y: 0.52 },
  ];
  // a CLEAR multi-zigzag scribble (NOT a hook): many LARGE alternating bends — must be
  // REJECTED by the scribble gate (rev=3+ at sharp corners). Distinct alias of `zigzag`.
  if (name === 'scribble') return [
    { x: 0.0, y: 0.0 }, { x: 0.30, y: 0.18 }, { x: -0.12, y: 0.36 }, { x: 0.34, y: 0.54 },
    { x: -0.08, y: 0.72 }, { x: 0.36, y: 0.90 }, { x: -0.05, y: 1.05 },
  ];
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
