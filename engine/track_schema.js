// engine/track_schema.js
// Single source of truth for TrackData (POC §4 / §9).
// Coordinate system: physics 2D plane (x = forward, y = up).
// The renderer extrudes this z=0 plane into 3D.
//
// Engine MUST only LOAD this data; never hardcode level values.

/**
 * @typedef {'flat'|'stairs'|'ramp'|'gap'|'wall'|'bumps'|'tunnel'|'planks'|'balls'|'blocks'|'fork'} SegmentKind
 */

/**
 * Track segment — source data for floor colliders.
 * @typedef {Object} TrackSegment
 * @property {SegmentKind} kind
 * @property {number}  length          x-direction length (world units)
 * @property {number} [height]         ramp rise/fall, wall step-up height, or stairs total height
 * @property {number} [steps]          number of steps (stairs)
 * @property {number} [width]          GAP only: horizontal width of the V-trench (overrides length if given)
 * @property {number} [depth]          GAP only: how deep the V-trench bottom drops below the lip
 * @property {number} [amp]            BUMPS only: half-amplitude of the sine surface (peak-to-trough = 2*amp)
 * @property {number} [freq]           BUMPS only: number of full sine periods over the segment length
 * @property {number} [clearance]      TUNNEL only: the LOW CEILING's max passable leg reach. A leg passes
 *                                     iff reach <= clearance (the LONGER the leg, the more it strikes the
 *                                     ceiling). Smaller clearance ⇒ only shorter legs fit. The inverse of
 *                                     a WALL (which needs a LONG leg). Default SEGMENT_DEFAULTS.tunnelClearance.
 * @property {number} [count]          PLANKS only: number of plank boards in the run (gaps = count-1, BETWEEN
 *                                     boards). Each board is a flat segment; each gap is a small void the
 *                                     walker must STRIDE across (a long/fast leg sails over, a short leg
 *                                     drops into the recovery trench and crawls out — no soft-lock).
 * @property {number} [plankLen]       PLANKS only: x-length of each plank board (world units). Default SEGMENT_DEFAULTS.plankLen.
 * @property {number} [gapLen]         PLANKS only: x-length of each gap between boards (world units). Default SEGMENT_DEFAULTS.gapLen.
 * @property {number} [gapDepth]       PLANKS only: how deep the gap's recovery trench drops below the board top
 *                                     (a short leg falls this far, then crawls out — never a bottomless pit).
 *                                     Default SEGMENT_DEFAULTS.plankGapDepth.
 * @property {number} [count]          BALLS only: number of dynamic physics balls piled on the flat (re-used key;
 *                                     for BALLS this is the ball count, default SEGMENT_DEFAULTS.ballCount, capped
 *                                     at SEGMENT_DEFAULTS.ballCountMax for performance).
 * @property {number} [ballR]          BALLS only: ball radius (world units). Default SEGMENT_DEFAULTS.ballR.
 * @property {number} [ballSpread]     BALLS only: x-extent the pile is initially scattered over (default = a
 *                                     fraction of the segment length so the pile blocks the path mid-segment).
 * @property {number} [blockCount]     BLOCKS only: number of STANDING blocks in the wall the cube must smash
 *                                     (default SEGMENT_DEFAULTS.blockCount). Each intact block bars the path;
 *                                     on cube contact it BREAKS into debris that falls + litters the floor.
 * @property {number} [blockW]         BLOCKS only: each standing block's x-width (world units). Default SEGMENT_DEFAULTS.blockW.
 * @property {number} [blockH]         BLOCKS only: each standing block's height (world units). Default SEGMENT_DEFAULTS.blockH.
 * @property {number} [debrisPerBlock] BLOCKS only: how many debris fragments a broken block becomes (default
 *                                     SEGMENT_DEFAULTS.debrisPerBlock). Total debris is capped at debrisCapTotal.
 * @property {number} [highRise]       FORK only: the high route's staircase CLIMB height (total rise of the
 *                                     hook-gated steep staircase up onto the arch). Default SEGMENT_DEFAULTS.forkHighRise.
 *                                     The staircase auto-hook-gates (overall slope >= steepThresh) so ONLY a HOOK
 *                                     leg takes the high road — exactly the steep-staircase mechanic.
 * @property {number} [highSteps]      FORK only: number of treads on EACH of the high route's up + down staircases.
 *                                     Default SEGMENT_DEFAULTS.forkHighSteps.
 * @property {number} [flatTop]        FORK only: x-length of the flat high path on top of the arch (between the up
 *                                     and down staircases). Default SEGMENT_DEFAULTS.forkFlatTop.
 * @property {number} [lowDip]         FORK only: how far the LOW route dips DOWN under the arch (a shallow valley /
 *                                     underpass — always passable by any leg). Default SEGMENT_DEFAULTS.forkLowDip.
 * @property {number} [rough]          0..1 friction (rough floor), default 0.6
 * @property {number} [bouncy]         0..1 restitution (rubber), default 0
 */

/**
 * @typedef {string} LegPreset   one of presetStroke()'s names (e.g. 'limb','limb_long')
 */

/**
 * Computer opponent (the rival). Runs the SAME procedural walker on a PARALLEL
 * lane with a fixed designed leg preset, its forward speed scaled by `pace`.
 * @typedef {Object} RivalSpec
 * @property {LegPreset} legPreset   the rival's drawn-leg shape (fixed, designed)
 * @property {number}    pace        speed multiplier on the rival's walker (1 = same feel as that leg)
 * @property {number}   [laneOffset] z separation of the rival lane from the player lane (world units; +behind)
 * @property {string}   [name]       our own rival nickname for the HUD/label (NOT the original game's)
 */

/**
 * @typedef {Object} TrackData
 * @property {string} id              e.g. 'T01'
 * @property {string} theme
 * @property {number} startX          character start x
 * @property {number} finishX         finish-line x (progress-bar end)
 * @property {TrackSegment[]} segments
 * @property {RivalSpec|null} [rival]
 */

/** Default rival used when a track omits `rival` (but a rival is requested). */
export const RIVAL_DEFAULTS = Object.freeze({
  legPreset: 'limb',
  pace: 1.0,
  laneOffset: 2.8,
  name: 'BOLT',
});

/** Default segment material values (used when a segment omits them). */
export const SEGMENT_DEFAULTS = Object.freeze({
  rough: 0.6,
  bouncy: 0.0,
  // TUNNEL: default LOW-CEILING max passable reach. A leg passes iff reach <= this.
  // The walls require reach >= ~1.15 (Rw), so a tunnel clearance below that makes the
  // two gimmicks MUTUALLY EXCLUSIVE — no single reach passes both (see walker.js TUNE).
  tunnelClearance: 0.95,
  // PLANKS: default board length, gap length, and the gap's recovery-trench depth.
  // The gaps are the gimmick: a long/fast leg (big stride) sails over them, a short
  // leg drops into the shallow recovery trench and crawls out (slow, no soft-lock).
  plankLen: 1.4,
  gapLen: 1.0,
  plankGapDepth: 1.6,
  // BALLS: a pile of dynamic physics spheres lying on a flat stretch that the cube must
  // shove out of the way. The cube does NOT auto-stop (no soft-lock) — pushing the balls
  // costs SPEED (resistance ∝ how many it is in contact with), so the cube slows through
  // the pile and accelerates back out once clear. Light single-sphere physics (pos/vel +
  // gravity + ground clamp + ball-ball separation + ball-cube push + linear friction).
  ballCount: 14,        // default ball count for a `balls` segment
  ballCountMax: 20,     // PERF cap — O(N²) separation stays cheap for N<=20
  ballR: 0.34,          // ball radius (world units) — a touch under the cube half-size
  ballSpreadFrac: 0.5,  // initial x-scatter = this fraction of the segment length (pile mid-segment)
  // BLOCKS: a wall of STANDING boxes that bar the cube's path. On contact a block BREAKS
  // into small debris fragments that fall (gravity), litter the floor, and SLOW the cube
  // that grinds over them (reusing the ball-debris physics: gravity + ground clamp +
  // fragment-fragment / fragment-cube separation + a resistance slow-factor). The cube
  // ALWAYS smashes through (no soft-lock) — breaking + plowing the rubble just costs speed.
  blockCount: 4,        // default standing blocks per `blocks` segment
  blockW: 0.7,          // each standing block's x-width (world units)
  blockH: 1.4,          // each standing block's height (world units) — taller than the cube ⇒ a real wall
  debrisPerBlock: 5,    // fragments a broken block becomes
  debrisCapTotal: 24,   // PERF hard cap on total debris (O(N²) separation stays cheap)
  debrisRfrac: 0.26,    // debris fragment half-size as a fraction of blockW (small chips)
  // FORK (SPLIT-PATH): the track splits into a HIGH route (hook-gated steep staircase up → flat
  // top → staircase down) and a LOW route (a shallow valley / underpass under the arch) that
  // REJOIN at the fork end. The LEG SHAPE decides the route, committed at the fork ENTRANCE:
  // a HOOK takes the high road (a hook is exactly what climbs the steep staircase); any other
  // leg takes the always-passable low road (no soft-lock — both routes always reach the rejoin).
  forkHighRise: 4.4,    // high route staircase climb height (overall slope >= steepThresh ⇒ auto hook-gate)
  forkHighSteps: 5,     // treads on EACH of the up + down staircases (TALL grip-point edges: with the
                        // default ~50° geometry below, each step rise 0.88u > its run ⇒ clearly stepped)
  forkFlatTop: 4.0,     // x-length of the flat high path on top of the arch
  // The high route's per-staircase RUN = (fork length − flatTop) / 2; the climb slope = forkHighRise/run.
  // To match the standalone steep climb at ~50° (slope ≈ 1.19) with forkHighRise 4.4, author the fork
  // `length` ≈ 2·(4.4/1.19) + flatTop ≈ 11.4 (run ≈ 3.7 per staircase). The slope is DATA-derived from
  // the authored length (not a fixed default), and auto-hook-gates whenever it is >= steepThresh.
  forkLowDip: 2.2,      // how far the low route dips DOWN under the arch (a shallow underpass valley)
});

/** @type {SegmentKind[]} */
export const SEGMENT_KINDS = ['flat', 'stairs', 'ramp', 'gap', 'wall', 'bumps', 'tunnel', 'planks', 'balls', 'blocks', 'fork'];

/** @type {LegPreset[]} */
export const LEG_PRESETS = ['wheel', 'stick', 'hook'];

/**
 * Validate a TrackData object. Throws on structural errors so loading a bad
 * track fails loudly instead of producing NaN physics later.
 * @param {any} t
 * @returns {TrackData}
 */
export function validateTrack(t) {
  if (!t || typeof t !== 'object') throw new Error('TrackData: not an object');
  if (typeof t.id !== 'string') throw new Error('TrackData.id must be a string');
  if (!Array.isArray(t.segments) || t.segments.length === 0)
    throw new Error('TrackData.segments must be a non-empty array');
  for (const n of ['startX', 'finishX']) {
    if (typeof t[n] !== 'number' || !Number.isFinite(t[n]))
      throw new Error(`TrackData.${n} must be a finite number`);
  }
  if (t.finishX <= t.startX)
    throw new Error('TrackData.finishX must be greater than startX');
  t.segments.forEach((s, i) => {
    if (!SEGMENT_KINDS.includes(s.kind))
      throw new Error(`segment[${i}].kind invalid: ${s.kind}`);
    // a GAP carries its horizontal extent as `width` (the V-trench span); a PLANKS
    // run DERIVES its extent from count×plankLen + (count-1)×gapLen; all other kinds
    // use `length`. Either way the extent must be a positive number.
    if (s.kind === 'gap') {
      if (!(s.width > 0) && !(s.length > 0))
        throw new Error(`segment[${i}] gap needs width>0 (or length>0)`);
    } else if (s.kind === 'planks') {
      // length is OPTIONAL for planks (computed from count/plankLen/gapLen).
    } else if (typeof s.length !== 'number' || s.length <= 0) {
      throw new Error(`segment[${i}].length must be a positive number`);
    }
    if (s.kind === 'stairs') {
      if (!(s.steps > 0)) throw new Error(`segment[${i}] stairs needs steps>0`);
      if (!(s.height > 0)) throw new Error(`segment[${i}] stairs needs height>0`);
    }
    if ((s.kind === 'ramp' || s.kind === 'wall') && !(typeof s.height === 'number'))
      throw new Error(`segment[${i}] ${s.kind} needs height`);
    if (s.kind === 'gap') {
      // a GAP is a V-trench with a floor (no soft-lock) — needs a depth to drop.
      if (!(s.depth > 0)) throw new Error(`segment[${i}] gap needs depth>0`);
    }
    if (s.kind === 'bumps') {
      if (!(s.amp > 0)) throw new Error(`segment[${i}] bumps needs amp>0`);
      if (!(s.freq > 0)) throw new Error(`segment[${i}] bumps needs freq>0`);
    }
    if (s.kind === 'tunnel') {
      // a TUNNEL is a low-ceiling stretch gated by leg reach. clearance is optional
      // (falls back to SEGMENT_DEFAULTS.tunnelClearance); if given it must be positive.
      if (s.clearance != null && !(s.clearance > 0))
        throw new Error(`segment[${i}] tunnel clearance must be > 0`);
    }
    if (s.kind === 'planks') {
      // a PLANKS run needs at least 2 boards (so there is at least 1 gap to stride).
      if (!(s.count >= 2)) throw new Error(`segment[${i}] planks needs count>=2`);
      if (s.plankLen != null && !(s.plankLen > 0))
        throw new Error(`segment[${i}] planks plankLen must be > 0`);
      if (s.gapLen != null && !(s.gapLen > 0))
        throw new Error(`segment[${i}] planks gapLen must be > 0`);
      if (s.gapDepth != null && !(s.gapDepth > 0))
        throw new Error(`segment[${i}] planks gapDepth must be > 0`);
    }
    if (s.kind === 'balls') {
      // a BALLS pile lies on a flat run — needs a positive length to walk over. count /
      // ballR / ballSpread are optional (fall back to SEGMENT_DEFAULTS); if present they
      // must be positive. The count is later clamped to ballCountMax for performance.
      if (s.count != null && !(s.count > 0))
        throw new Error(`segment[${i}] balls count must be > 0`);
      if (s.ballR != null && !(s.ballR > 0))
        throw new Error(`segment[${i}] balls ballR must be > 0`);
      if (s.ballSpread != null && !(s.ballSpread > 0))
        throw new Error(`segment[${i}] balls ballSpread must be > 0`);
    }
    if (s.kind === 'blocks') {
      // a BLOCKS wall stands on a flat run — needs a positive length to walk over. count /
      // dims are optional (fall back to SEGMENT_DEFAULTS); if present they must be positive.
      if (s.blockCount != null && !(s.blockCount > 0))
        throw new Error(`segment[${i}] blocks blockCount must be > 0`);
      if (s.blockW != null && !(s.blockW > 0))
        throw new Error(`segment[${i}] blocks blockW must be > 0`);
      if (s.blockH != null && !(s.blockH > 0))
        throw new Error(`segment[${i}] blocks blockH must be > 0`);
      if (s.debrisPerBlock != null && !(s.debrisPerBlock > 0))
        throw new Error(`segment[${i}] blocks debrisPerBlock must be > 0`);
    }
    if (s.kind === 'fork') {
      // a FORK is a SPLIT-PATH over `length` (the total span to the rejoin). All shaping
      // fields are optional (fall back to SEGMENT_DEFAULTS); if present they must be positive.
      // `length` is already validated as positive above (the generic length check).
      if (s.highRise != null && !(s.highRise > 0))
        throw new Error(`segment[${i}] fork highRise must be > 0`);
      if (s.highSteps != null && !(s.highSteps > 0))
        throw new Error(`segment[${i}] fork highSteps must be > 0`);
      if (s.flatTop != null && !(s.flatTop > 0))
        throw new Error(`segment[${i}] fork flatTop must be > 0`);
      if (s.lowDip != null && !(s.lowDip > 0))
        throw new Error(`segment[${i}] fork lowDip must be > 0`);
    }
  });
  if (t.rival != null) {
    const r = t.rival;
    if (typeof r !== 'object') throw new Error('TrackData.rival must be an object or null');
    if (typeof r.legPreset !== 'string') throw new Error('rival.legPreset must be a string');
    if (typeof r.pace !== 'number' || !(r.pace > 0)) throw new Error('rival.pace must be a positive number');
    if (r.laneOffset != null && typeof r.laneOffset !== 'number')
      throw new Error('rival.laneOffset must be a number');
  }
  return /** @type {TrackData} */ (t);
}
