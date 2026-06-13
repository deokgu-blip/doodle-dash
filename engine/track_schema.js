// engine/track_schema.js
// Single source of truth for TrackData (POC §4 / §9).
// Coordinate system: physics 2D plane (x = forward, y = up).
// The renderer extrudes this z=0 plane into 3D.
//
// Engine MUST only LOAD this data; never hardcode level values.

/**
 * @typedef {'flat'|'stairs'|'ramp'|'gap'|'wall'} SegmentKind
 */

/**
 * Track segment — source data for floor colliders.
 * @typedef {Object} TrackSegment
 * @property {SegmentKind} kind
 * @property {number}  length          x-direction length (world units)
 * @property {number} [height]         ramp rise/fall, wall height, or stairs total height
 * @property {number} [steps]          number of steps (stairs)
 * @property {number} [rough]          0..1 friction (rough floor), default 0.6
 * @property {number} [bouncy]         0..1 restitution (rubber), default 0
 */

/**
 * @typedef {'wheel'|'stick'|'hook'} LegPreset
 */

/**
 * @typedef {Object} RivalSpec
 * @property {LegPreset} legPreset
 * @property {number}    speed
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

/** Default segment material values (used when a segment omits them). */
export const SEGMENT_DEFAULTS = Object.freeze({
  rough: 0.6,
  bouncy: 0.0,
});

/** @type {SegmentKind[]} */
export const SEGMENT_KINDS = ['flat', 'stairs', 'ramp', 'gap', 'wall'];

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
    if (typeof s.length !== 'number' || s.length <= 0)
      throw new Error(`segment[${i}].length must be a positive number`);
    if (s.kind === 'stairs') {
      if (!(s.steps > 0)) throw new Error(`segment[${i}] stairs needs steps>0`);
      if (!(s.height > 0)) throw new Error(`segment[${i}] stairs needs height>0`);
    }
    if ((s.kind === 'ramp' || s.kind === 'wall') && !(typeof s.height === 'number'))
      throw new Error(`segment[${i}] ${s.kind} needs height`);
  });
  return /** @type {TrackData} */ (t);
}
