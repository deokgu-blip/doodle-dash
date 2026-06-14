// engine/path.js
// HEADING-BASED PATH abstraction (Phase 1 of the "genuinely turning road" feature).
//
// The game's forward coordinate `x` is the ARC-LENGTH along the track centre-line
// (it equals the physics forward coordinate the walker advances along — physics is
// 1-D, z is render-only). This module turns that 1-D arc-length into a 2-D world
// ground-plane curve by integrating a HEADING (yaw) angle:
//
//   pathHeading(x) → yaw angle (radians, 0 = +x) of the centre-line tangent at x.
//   pathX(x), pathZ(x) = ∫₀ˣ (cos h, sin h) dx'   (cumulative tangent integral)
//
// A point at forward `x`, lateral offset `L` (the z-role: +L = the TOWARD-CAMERA side,
// matching today's +z), height `y` maps to WORLD coordinates via the transform:
//
//   h      = pathHeading(x)
//   worldX = pathX(x) + L * (-sin h)
//   worldY = -y                      (render y is up; physics y is +down)
//   worldZ = pathZ(x) + L * ( cos h)
//
// At h ≡ 0 this reduces to (x, -y, L) — BYTE-IDENTICAL to the old
// `(x, -y, laneZ + laneCurveZ(x))` placement when laneCurveZ ≡ 0, and a faithful
// winding-road look when the gentle serpentine is folded into the heading.
//
// PERF: the path is sampled into a FINE LUT at BUILD time (one cumulative integration
// pass). Per-frame lookups are O(1) (uniform step ⇒ index = (x - x0)/step) with a
// linear blend between the two bracketing samples, and ZERO allocation. There is NO
// per-frame integration and NO per-frame object creation — `transform()` writes into a
// caller-supplied {x,y,z} scratch (or a shared internal scratch) so the hot path is
// alloc-free.

// ── Folded serpentine (reproduces the old render-only laneCurveZ look EXACTLY) ───────
// The old cosmetic winding was a pure z-offset:  laneCurveZ(x) = AMP·sin(FREQ·x).
// We reproduce that SAME centre-line z purely through HEADING. Because x is the curve's
// ARC-LENGTH, the centre-line obeys dZ/dx = sin(h). To make pathZ(x) = AMP·sin(FREQ·x)
// (centred on 0, same amplitude/period as before — NO z-bias, NO secular drift) we set
//   dZ/dx = d/dx[AMP·sin(FREQ·x)] = AMP·FREQ·cos(FREQ·x)  ⇒  h(x) = asin(AMP·FREQ·cos(FREQ·x)).
// With AMP=2.5, FREQ=0.12 the peak |AMP·FREQ| = 0.30 < 1 (asin well-defined) and peak
// heading ≈ 0.305 rad (~17.5°) — a GENTLE sway. The integrated pathZ then oscillates
// EXACTLY between ±AMP, crossing 0 at x=0 just like the old sine, so straights read as
// the same gently winding ribbon. (pathX contracts only ~0.3u per ~52u period — the
// honest arc-length cost of winding — imperceptible and never z-drifts.)
export const SERP_AMP = 2.5;     // matches the old CURVE_AMP (peak z deflection)
export const SERP_FREQ = 0.12;   // matches the old CURVE_FREQ (spatial frequency)
const _SERP_K = SERP_AMP * SERP_FREQ;   // < 1 ⇒ asin defined for all x
export function serpHeading(x) { return Math.asin(_SERP_K * Math.cos(SERP_FREQ * x)); }

const TWO_PI = Math.PI * 2;

/**
 * A precomputed heading path over [x0, x1] (uniform `step`). Built ONCE from a heading
 * function; provides O(1) alloc-free lookups + a world transform.
 */
export class Path {
  /**
   * @param {number} x0      first sampled arc-length (≤ track start − margin)
   * @param {number} x1      last sampled arc-length  (≥ track finish + margin)
   * @param {number} step    LUT sample spacing in arc-length units (fine; e.g. 0.25)
   * @param {(x:number)=>number} headingFn  yaw(rad) at arc-length x (0 = +x = straight)
   * @param {object} [opts]
   * @param {number} [opts.anchorX]  arc-length at which the centre-line is RE-ZEROED to
   *   (anchorX, anchorZ): the integration constant is chosen so pathX(anchorX)=anchorX and
   *   pathZ(anchorX)=anchorZ. This makes STRAIGHT sections sit EXACTLY where the old world
   *   put them (no global lateral/forward shift from integrating across the lead-in margin).
   *   Default: x0 (so pathX(x0)=x0, pathZ(x0)=0 — back-compat).
   * @param {number} [opts.anchorZ]  the centre-line z at anchorX (default 0). For the folded
   *   serpentine this is the OLD laneCurveZ(anchorX) so the look is byte-faithful.
   */
  constructor(x0, x1, step, headingFn, opts = {}) {
    this.x0 = x0;
    this.step = step;
    const n = Math.max(1, Math.ceil((x1 - x0) / step));
    this.n = n;                       // number of intervals (samples = n+1)
    this.x1 = x0 + n * step;
    // Typed arrays (one contiguous block each) ⇒ no per-sample object garbage.
    this._h = new Float64Array(n + 1);   // heading at each sample
    this._px = new Float64Array(n + 1);  // ∫cos h  (world X of the centre-line)
    this._pz = new Float64Array(n + 1);  // ∫sin h  (world Z of the centre-line)
    // Trapezoidal cumulative integration of the tangent (cos h, sin h). Trapezoid keeps
    // the centre-line smooth and the integral accurate at the fine step we use.
    let px = x0;   // provisional anchor (re-zeroed below at anchorX). At h≡0 ⇒ pathX(x)=x.
    let pz = 0;    //                                                     pathZ(x)=0.
    let prevC = Math.cos(headingFn(x0));
    let prevS = Math.sin(headingFn(x0));
    this._h[0] = headingFn(x0);
    this._px[0] = px;
    this._pz[0] = pz;
    for (let i = 1; i <= n; i++) {
      const x = x0 + i * step;
      const h = headingFn(x);
      const c = Math.cos(h), s = Math.sin(h);
      px += (prevC + c) * 0.5 * step;
      pz += (prevS + s) * 0.5 * step;
      this._h[i] = h;
      this._px[i] = px;
      this._pz[i] = pz;
      prevC = c; prevS = s;
    }
    // RE-ZERO at anchorX so the curve passes through (anchorX, anchorZ): shift the whole
    // centre-line by a constant (shape unchanged). dx makes pathX(anchorX)=anchorX (cancels
    // the lead-in contraction); dz makes pathZ(anchorX)=anchorZ (cancels the integration
    // constant ⇒ straights sit exactly where the old world placed them).
    const anchorX = (opts.anchorX != null) ? opts.anchorX : x0;
    const anchorZ = (opts.anchorZ != null) ? opts.anchorZ : 0;
    const fiA = Math.min(this.n, Math.max(0, (anchorX - x0) / step));
    const iA = fiA | 0, tA = fiA - iA;
    const pxA = (iA >= this.n) ? this._px[this.n] : this._px[iA] + (this._px[iA + 1] - this._px[iA]) * tA;
    const pzA = (iA >= this.n) ? this._pz[this.n] : this._pz[iA] + (this._pz[iA + 1] - this._pz[iA]) * tA;
    const dx = anchorX - pxA, dz = anchorZ - pzA;
    if (dx !== 0 || dz !== 0) {
      for (let i = 0; i <= n; i++) { this._px[i] += dx; this._pz[i] += dz; }
    }
    // Shared scratch returned by transform() when the caller passes no `out` — lets the
    // hot path stay alloc-free even for callers that don't manage their own object.
    this._scratch = { x: 0, y: 0, z: 0 };
  }

  /** Fractional sample index for arc-length x, clamped to the LUT range. */
  _fi(x) {
    let fi = (x - this.x0) / this.step;
    if (fi < 0) fi = 0;
    else if (fi > this.n) fi = this.n;
    return fi;
  }

  /** Heading (yaw, rad) at arc-length x — O(1), alloc-free. */
  heading(x) {
    const fi = this._fi(x);
    const i = fi | 0;
    if (i >= this.n) return this._h[this.n];
    const t = fi - i;
    const a = this._h[i], b = this._h[i + 1];
    return a + (b - a) * t;
  }

  /** Centre-line world X at arc-length x — O(1), alloc-free. */
  pathX(x) {
    const fi = this._fi(x);
    const i = fi | 0;
    if (i >= this.n) return this._px[this.n];
    const t = fi - i;
    const a = this._px[i], b = this._px[i + 1];
    return a + (b - a) * t;
  }

  /** Centre-line world Z at arc-length x — O(1), alloc-free. */
  pathZ(x) {
    const fi = this._fi(x);
    const i = fi | 0;
    if (i >= this.n) return this._pz[this.n];
    const t = fi - i;
    const a = this._pz[i], b = this._pz[i + 1];
    return a + (b - a) * t;
  }

  /**
   * World transform of a path-local point (forward x, lateral L, height y).
   * Writes into `out` (or a shared scratch) and returns it — ALLOC-FREE.
   *   worldX = pathX(x) + L·(-sin h)
   *   worldY = -y
   *   worldZ = pathZ(x) + L·( cos h)
   * At h ≡ 0 ⇒ (x, -y, L), identical to the old (x, -y, laneZ+laneCurveZ) at curve 0.
   * @param {number} x forward arc-length
   * @param {number} L lateral offset (+ = toward-camera / +z side, matching old +z)
   * @param {number} y physics height (+down); worldY = -y
   * @param {{x:number,y:number,z:number}} [out]
   */
  transform(x, L, y, out) {
    const o = out || this._scratch;
    const fi = this._fi(x);
    const i = fi | 0;
    let h, pxc, pzc;
    if (i >= this.n) { h = this._h[this.n]; pxc = this._px[this.n]; pzc = this._pz[this.n]; }
    else {
      const t = fi - i;
      h = this._h[i] + (this._h[i + 1] - this._h[i]) * t;
      pxc = this._px[i] + (this._px[i + 1] - this._px[i]) * t;
      pzc = this._pz[i] + (this._pz[i + 1] - this._pz[i]) * t;
    }
    const sh = Math.sin(h), ch = Math.cos(h);
    o.x = pxc + L * (-sh);
    o.y = -y;
    o.z = pzc + L * ch;
    return o;
  }

  /**
   * The LATERAL world-z deflection of the centre-line at arc-length x, i.e. the value
   * the old render code called `laneCurveZ(x)`. With the serpentine folded into the
   * heading this is just `pathZ(x)` (the centre-line z relative to the z=0 anchor). It
   * is a thin COMPATIBILITY accessor for callers/tests that still think in "curve z".
   */
  laneCurveZ(x) { return this.pathZ(x); }
}

/**
 * Build the heading function for a track: the gentle serpentine sway folded in for the
 * whole track, PLUS any data-driven turn regions (segments carrying a `turn`/`turnDeg`).
 * The returned function is pure (no allocation) and is sampled once into the LUT.
 *
 * Turn regions: a segment may carry a heading change `turnDeg` (degrees, +left toward
 * +z) ramped SMOOTHLY (smoothstep) across its arc-length [x0,x1]. Heading accumulates,
 * so a big bend is just a region where the heading ramps through a large angle; after
 * the region the path continues straight at the new heading. Multiple turns compose.
 *
 * @param {{x0:number,x1:number,turnRad:number}[]} turns  arc-length turn regions
 * @param {boolean} serpentine  fold the gentle serpentine sway into the heading
 * @returns {(x:number)=>number}
 */
export function makeHeadingFn(turns, serpentine = true) {
  // Sort + precompute the cumulative heading BEFORE each region so heading(x) is the
  // serpentine sway + the sum of all completed turns + the partial (smoothstep) ramp of
  // the region currently containing x. All captured in the closure (no per-call alloc).
  const regs = (turns || []).slice().sort((a, b) => a.x0 - b.x0);
  const before = new Float64Array(regs.length); // accumulated turn heading at each region start
  let acc = 0;
  for (let i = 0; i < regs.length; i++) { before[i] = acc; acc += regs[i].turnRad; }
  return function heading(x) {
    let h = serpentine ? serpHeading(x) : 0;
    for (let i = 0; i < regs.length; i++) {
      const r = regs[i];
      if (x <= r.x0) break;            // regions are sorted; nothing further applies yet
      if (x >= r.x1) { h += r.turnRad; continue; }  // fully past this region
      // inside the region: smoothstep ramp from 0→turnRad across [x0,x1]
      const t = (x - r.x0) / (r.x1 - r.x0);
      const s = t * t * (3 - 2 * t);   // smoothstep (C¹ — zero slope at both ends ⇒ no kink)
      h += r.turnRad * s;
    }
    return h;
  };
}

/** Normalise an angle to (-π, π] (used for the eased camera heading low-pass). */
export function wrapAngle(a) {
  a = a % TWO_PI;
  if (a > Math.PI) a -= TWO_PI;
  else if (a <= -Math.PI) a += TWO_PI;
  return a;
}
