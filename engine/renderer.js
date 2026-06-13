// engine/renderer.js
// Three.js 3D render of the 2D physics plane (POC §4/§8).
//   - cube (dot-eye face) + extruded legs synced to Matter bodies each frame
//   - purple/magenta checkerboard track ribbon extruded from segments
//   - lime-green gradient background + 3/4 chase camera
// Colors use the POC §8 design tokens (placeholder materials; image assets later).

import * as THREE from './vendor/three.module.js';
import { PHYS_CONST } from './physics.js';

// POC §8 tokens
const COL = {
  bgTop: 0xA6E05A, bgBottom: 0x7FC93C, hill: 0x6FB836,
  trackA: 0x8E3AAE, trackB: 0xC24FD6, trackEdge: 0x5E2480,
  player: 0x3CA5E5, playerFace: 0x0E2A3A, leg: 0x1A1A1A,
  // RIVAL: our own opponent colour (NOT the original game's character). A bright
  // lime-emerald cube so it reads as a distinct racer on the parallel lane.
  rival: 0x35C44A, rivalFace: 0x0B3A18,
};

// The rival lane sits on the -z side (farther from the +z camera) so it reads as
// the parallel lane BEHIND the player and appears slightly smaller (perspective).
const RIVAL_LANE_SIGN = -1;

const RIBBON_DEPTH = 3.6;     // z-extrusion of the track (narrower lane width — reference-like path)
const RIBBON_DOWN = 0.9;      // how far below the surface the ribbon's side face drops (visible thickness)

// ── SERPENTINE CURVE (render-only, §C) ──────────────────────────────────────
// The original track snakes left/right as it recedes. We bend the WHOLE render
// world in z by laneCurveZ(x) = AMP * sin(x*FREQ). Physics x / progress / win are
// untouched — this only offsets the ribbon centre-line, the cubes, the legs and
// the camera target in z. (Verified: finish time identical curve on/off.)
const CURVE_AMP = 2.5;        // peak z deflection (world units)
const CURVE_FREQ = 0.12;      // spatial frequency (rad per world-x unit)
export function laneCurveZ(x) { return CURVE_AMP * Math.sin(x * CURVE_FREQ); }

// Ribbon sampling step along x (world units). Smaller = smoother curve but more
// verts; 0.5 keeps the continuous band smooth while staying draw-call cheap (one
// mesh per lane). Stairs are sampled at their riser/tread edges on top of this.
const RIBBON_DX = 0.5;

// Two legs straddle the cube in DEPTH (z). The cube is ~1.08 deep; legs sit
// just outside its faces so they clearly read as a left and a right leg.
const LEG_THICK = 0.30;       // per-leg extrusion depth (slim — two legs, not 1 wheel)
const LEG_Z_OFFSET = 0.62;    // z distance of each leg from the cube center

export class Renderer {
  constructor(canvas) {
    this.canvas = canvas;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
    // L1: keep backbuffer reasonable; input stays in CSS px.
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.shadowMap.enabled = false;
    // L13: don't sync-read shader logs in production.
    this.renderer.debug.checkShaderErrors = false;

    this.scene = new THREE.Scene();
    this._buildBackground();
    this._buildLights();

    // y is UP on screen; physics y is +down, so we render with y = -physY.
    // §D: a slightly WIDER FOV (was 50) so the cube reads bigger from a closer,
    // lower 3/4 chase (reference framing: character ~1/3 of the screen, big sky).
    this.camera = new THREE.PerspectiveCamera(58, 1, 0.1, 200);

    this.cubeMesh = null;    // player cube (smiley face is a child mesh)
    this.legGroups = [];     // [{ mesh, body, side }] — one extruded disc per wheel
    this.trackGroup = null;

    // RIVAL (computer opponent) — parallel lane, own cube + legs, offset in z.
    this.rivalCubeMesh = null;
    this.rivalLegGroups = [];
    this.rivalLaneZ = 0;     // z centre of the rival lane (set in buildTrack)

    // The leg is a DRAWN PEN LINE, not a shaded 3D object. A flat, unlit material
    // keeps it reading as a single solid black stroke (no per-bump specular
    // highlights that made the old circle-chain look like a beaded wheel).
    this._legMat = new THREE.MeshBasicMaterial({ color: COL.leg, side: THREE.DoubleSide });
  }

  _buildBackground() {
    // procedural lime gradient sky (POC §6: background is procedural)
    const c = document.createElement('canvas');
    c.width = 8; c.height = 256;
    const ctx = c.getContext('2d');
    const g = ctx.createLinearGradient(0, 0, 0, 256);
    g.addColorStop(0, '#A6E05A');
    g.addColorStop(0.7, '#8FD24A');
    g.addColorStop(1, '#7FC93C');
    ctx.fillStyle = g; ctx.fillRect(0, 0, 8, 256);
    const tex = new THREE.CanvasTexture(c);
    tex.colorSpace = THREE.SRGBColorSpace;
    this.scene.background = tex;
  }

  _buildLights() {
    const amb = new THREE.AmbientLight(0xffffff, 0.75);
    this.scene.add(amb);
    const dir = new THREE.DirectionalLight(0xffffff, 0.9);
    dir.position.set(-4, 8, 6);
    this.scene.add(dir);
    const fill = new THREE.DirectionalLight(0xC24FD6, 0.18);
    fill.position.set(4, 2, -4);
    this.scene.add(fill);
  }

  /** Build the track ribbon + ground from the physics floor bodies. When a
   * `rivalSpec` is given, also build a PARALLEL lane (offset in z) + the rival
   * cube so the two racers read side-by-side in a diagonal 3/4 view. */
  buildTrack(physics, track, rivalSpec = null) {
    if (this.trackGroup) { this.scene.remove(this.trackGroup); this._disposeGroup(this.trackGroup); }
    if (this.cubeMesh) { this.scene.remove(this.cubeMesh); }
    if (this.rivalCubeMesh) { this.scene.remove(this.rivalCubeMesh); }
    this.trackGroup = new THREE.Group();

    // checkerboard texture for the ribbon top (purple check, as in the reference).
    // One material set, reused across both lanes (draw-call friendly).
    const checker = this._makeChecker();
    const sideMat = new THREE.MeshStandardMaterial({ color: COL.trackEdge, roughness: 0.85 });
    // emissive-tinted top so the purple check survives the grazing 3/4 light
    // (a pure StandardMaterial top went near-flat under one steep directional).
    const topMat = new THREE.MeshStandardMaterial({
      map: checker, roughness: 0.6, emissive: 0x3A1450, emissiveMap: checker, emissiveIntensity: 0.55,
    });

    // rival lane z-centre (parallel lane, one offset away on the -z side).
    this.rivalLaneZ = rivalSpec
      ? RIVAL_LANE_SIGN * (rivalSpec.laneOffset ?? 7.0) : 0;

    // §B+§C: each lane is ONE CONTINUOUS ribbon mesh (a constant-width band of
    // RIBBON_DEPTH) whose centre-line follows (x, surfaceY(x), laneZ+laneCurveZ(x)).
    // The surface (flats, ramps, stair steps) is traced unbroken, and the
    // serpentine z-curve is baked into the same vertices — so B (continuous) and C
    // (winding) are solved in one builder. Physics x is untouched.
    this._buildRibbon(physics, 0, topMat, sideMat);
    if (rivalSpec) this._buildRibbon(physics, this.rivalLaneZ, topMat, sideMat);
    this.scene.add(this.trackGroup);

    // ── player cube ──
    this.cubeMesh = this._buildCharacterCube(COL.player, COL.playerFace);
    this.scene.add(this.cubeMesh);

    // ── rival cube (our own opponent — distinct colour, on the parallel lane) ──
    if (rivalSpec) {
      this.rivalCubeMesh = this._buildCharacterCube(COL.rival, COL.rivalFace);
      this.rivalCubeMesh.position.z = this.rivalLaneZ;
      this.scene.add(this.rivalCubeMesh);
    } else {
      this.rivalCubeMesh = null;
    }

    // finish flag (simple marker at finishX) — spans both lanes when racing.
    this._buildFinish(track, rivalSpec, physics);
  }

  /** Build the ordered (x, surfaceY) profile of the track surface from the physics
   * segment model — a CONTINUOUS polyline along x. Flats/ramps emit their two
   * endpoints; stairs emit each tread as a flat run plus a VERTICAL riser (two
   * samples at the same x), so steps read as sharp steps inside the smooth band.
   * Returns [{x, y}] with physics y (+down). Used to extrude the continuous ribbon. */
  _surfaceProfile(physics) {
    const segs = physics._segs;
    if (!segs || !segs.length) return [];
    const pts = [];
    const push = (x, y) => {
      const last = pts[pts.length - 1];
      if (last && Math.abs(last.x - x) < 1e-6 && Math.abs(last.y - y) < 1e-6) return;
      pts.push({ x, y });
    };
    for (const s of segs) {
      if (s.kind === 'gap') continue;           // no surface over a gap (skip)
      const y0 = s.surfFn(s.x0), y1 = s.surfFn(s.x1);
      if (y0 == null || y1 == null) continue;
      // a riser: if this sample starts ABOVE (more negative) where the previous
      // one ended, drop a vertical wall first so a stair step is a true step.
      push(s.x0, y0);
      push(s.x1, y1);
    }
    pts.sort((a, b) => a.x - b.x);
    return pts;
  }

  /** §B+§C — build ONE continuous ribbon mesh for a lane. The band has constant
   * width RIBBON_DEPTH (in z), its centre-line follows (x, -surfaceY(x), laneZ +
   * laneCurveZ(x)); the top face carries the purple checker, the front/back/under
   * faces carry the edge colour (visible thickness). The serpentine z-curve is
   * baked into every vertex so the whole band snakes left/right while the physics
   * x stays a clean 1-D line. One mesh per lane ⇒ draw-call cheap. */
  _buildRibbon(physics, laneZ, topMat, sideMat) {
    const prof = this._surfaceProfile(physics);
    if (prof.length < 2) return;
    // Densify: insert intermediate x samples (RIBBON_DX) between profile points so
    // the z-curve bends smoothly over long flats/ramps; keep the exact profile pts
    // (incl. stair risers) so steps stay sharp.
    const xs = [];
    for (let i = 0; i < prof.length - 1; i++) {
      const a = prof[i], b = prof[i + 1];
      xs.push(a.x);
      const span = b.x - a.x;
      if (span > RIBBON_DX * 1.5) {
        const n = Math.floor(span / RIBBON_DX);
        for (let k = 1; k < n; k++) {
          const t = k / n;
          xs.push(a.x + span * t);
        }
      }
    }
    xs.push(prof[prof.length - 1].x);
    // surfaceY at an arbitrary x via the physics sampler (highest surface), with a
    // hold-last fallback so a momentary null (segment seam round-off) never gaps.
    let lastY = prof[0].y;
    const surfY = (x) => {
      const y = physics.surfaceYAt(x);
      if (y != null) { lastY = y; return y; }
      return lastY;
    };
    const half = RIBBON_DEPTH / 2;

    // Build a top strip (2 rails) + a bottom strip (2 rails, dropped RIBBON_DOWN) so
    // we get a top face (check) and the two side walls + a bottom (edge colour).
    const topPos = [], topUV = [], topIdx = [];
    const sidePos = [], sideIdx = [];
    let topV = 0, sideV = 0;
    let prevX = null;
    const N = xs.length;
    // checker repeat: ~0.7 cell/world-u along x, and ~2 cells across the band width
    // (v 0→2) so the purple check reads clearly on the top (reference look).
    const uScale = 0.7;
    for (let i = 0; i < N; i++) {
      const x = xs[i];
      const ry = -surfY(x);                 // render y (up)
      const cz = laneZ + laneCurveZ(x);     // serpentine z centre
      const zN = cz - half, zF = cz + half; // near / far edges
      // TOP strip: 2 verts (near, far) at the surface.
      topPos.push(x, ry, zN, x, ry, zF);
      topUV.push(x * uScale, 0, x * uScale, 2);
      // SIDE/bottom strip: top edges (= surface) + bottom edges (dropped).
      const by = ry - RIBBON_DOWN;
      sidePos.push(
        x, ry, zN,  x, by, zN,   // near wall top, bottom
        x, ry, zF,  x, by, zF    // far wall top, bottom
      );
      if (i > 0) {
        // TOP quad between ring i-1 and i.
        const a = (i - 1) * 2, b = a + 1, c = i * 2, d = c + 1;
        topIdx.push(a, c, b, b, c, d);
        // SIDE walls (near + far) + bottom quad. Each ring has 4 side verts:
        //   base+0 nearTop, +1 nearBot, +2 farTop, +3 farBot.
        const p = (i - 1) * 4, q = i * 4;
        // near wall (facing -z / camera): nearTop/nearBot
        sideIdx.push(p + 0, p + 1, q + 0, q + 0, p + 1, q + 1);
        // far wall (facing +z)
        sideIdx.push(p + 2, q + 2, p + 3, p + 3, q + 2, q + 3);
        // bottom (facing down): nearBot/farBot
        sideIdx.push(p + 1, p + 3, q + 1, q + 1, p + 3, q + 3);
      }
      prevX = x;
    }
    const topGeo = new THREE.BufferGeometry();
    topGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(topPos), 3));
    topGeo.setAttribute('uv', new THREE.BufferAttribute(new Float32Array(topUV), 2));
    topGeo.setIndex(topIdx);
    topGeo.computeVertexNormals();
    this.trackGroup.add(new THREE.Mesh(topGeo, topMat));

    const sideGeo = new THREE.BufferGeometry();
    sideGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(sidePos), 3));
    sideGeo.setIndex(sideIdx);
    sideGeo.computeVertexNormals();
    this.trackGroup.add(new THREE.Mesh(sideGeo, sideMat));
  }

  /** Build a smiley character cube (body colour + face colour) with a dot-eye
   * face on +z. Shared by player and rival (different palette). */
  _buildCharacterCube(bodyColor, faceColor) {
    const cubeGeo = new THREE.BoxGeometry(PHYS_CONST.CUBE_SIZE, PHYS_CONST.CUBE_SIZE, PHYS_CONST.CUBE_SIZE * 0.9);
    const cubeMat = new THREE.MeshStandardMaterial({ color: bodyColor, roughness: 0.45, metalness: 0.05 });
    const cube = new THREE.Mesh(cubeGeo, cubeMat);
    const faceTex = this._makeFace(faceColor);
    const faceMat = new THREE.MeshBasicMaterial({ map: faceTex, transparent: true });
    const faceGeo = new THREE.PlaneGeometry(PHYS_CONST.CUBE_SIZE * 0.92, PHYS_CONST.CUBE_SIZE * 0.92);
    const face = new THREE.Mesh(faceGeo, faceMat);
    face.position.z = PHYS_CONST.CUBE_SIZE * 0.46;
    cube.add(face);
    return cube;
  }

  _buildFinish(track, rivalSpec = null, physics = null) {
    const flagTex = this._makeChecker(6);
    const lanes = rivalSpec ? [0, this.rivalLaneZ] : [0];
    // sit the flag ON the surface at finishX (render y = -surfaceY) and on the
    // serpentine band (z + laneCurveZ) so it stays planted as the track winds.
    const surfY = physics && physics.surfaceYAt(track.finishX);
    const baseY = (surfY != null) ? -surfY : 0;
    const cz = laneCurveZ(track.finishX);
    for (const laneZ of lanes) {
      const z = laneZ + cz;
      const pole = new THREE.Mesh(
        new THREE.PlaneGeometry(0.15, 3),
        new THREE.MeshBasicMaterial({ color: 0xffffff })
      );
      pole.position.set(track.finishX, baseY + 1.5, z);
      this.trackGroup.add(pole);
      const flag = new THREE.Mesh(
        new THREE.PlaneGeometry(1.2, 0.8),
        new THREE.MeshBasicMaterial({ map: flagTex, side: THREE.DoubleSide })
      );
      flag.position.set(track.finishX + 0.6, baseY + 2.5, z);
      this.trackGroup.add(flag);
    }
  }

  /** Rebuild leg meshes to match the current physics legs (after a redraw).
   * Both legs share the SAME single bottom-center axle in physics, so we offset
   * them in z (depth) by their `side` (-1 = far / left, +1 = near / right) to
   * read as two legs straddling the cube. They are 180° out of phase so one
   * foot plants while the other lifts — the alternating walking gait.
   *
   * WYSIWYG: each leg is drawn as a THIN LINE RIBBON tracing the user's stroke
   * polyline (NOT a filled convex hull). The same polyline the physics models as
   * an overlapping circle-chain is extruded here with a small width (==2x the
   * physics circle radius), so the visible line == the physics shape == the
   * drawn stroke. Open/curved strokes stay open & curved; long strokes -> long
   * legs (the polyline carries the preserved drawn length). */
  rebuildLegs(physics) {
    this.legGroups = this._buildLegGroups(physics, this.legGroups, 0);
  }

  /** Rebuild the RIVAL's leg meshes (same builder, centred on the rival lane z). */
  rebuildRivalLegs(rival) {
    this.rivalLegGroups = this._buildLegGroups(rival, this.rivalLegGroups, this.rivalLaneZ);
  }

  /** Shared leg-group builder for both racers. `laneZ` is the lane centre; each
   * leg is straddled ±LEG_Z_OFFSET about it. Disposes the old groups, returns
   * the new array. */
  _buildLegGroups(physics, oldGroups, laneZ) {
    for (const lg of oldGroups) { this.scene.remove(lg.mesh); this._disposeMesh(lg.mesh); }
    const groups = [];
    for (const l of physics.legs) {
      const body = l.body;
      const grp = new THREE.Group();
      // WYSIWYG, RIGID: the leg visual is the user's ORIGINAL stroke polyline,
      // built ONCE here as a single smooth thin pen line. The stored chain is in
      // the leg's AXLE-LOCAL frame (axle == origin == stroke start). pinLocal is
      // {0,0}. We build the line in that local frame and sync() then just places
      // the group at the axle and rotates it by the physics leg angle — the drawn
      // shape NEVER deforms, it only spins as a whole.
      const pin = l.pinLocal || { x: 0, y: 0 };
      const pts = l.chain.map((c) => ({ x: pin.x + c.x, y: pin.y + c.y }));
      // Visible half-width ~= the physics circle radius so the drawn line ≈ the
      // collision shape (WYSIWYG). A touch under the radius keeps it slim.
      const halfW = (l.lineRadius || 0.13) * 0.95;
      const mesh = this._buildStrokeRibbon(pts, halfW);
      if (mesh) grp.add(mesh);
      this.scene.add(grp);
      // z offset by side (straddle) PLUS the lane centre offset. The serpentine
      // curve z (laneCurveZ(x)) is ADDED per-frame in _syncLegGroups (it depends
      // on the live x), so we keep the static part here and the dynamic part there.
      groups.push({ mesh: grp, body, side: l.side, laneZ, z: laneZ + l.side * LEG_Z_OFFSET });
    }
    return groups;
  }

  /** Build a SMOOTH constant-width pen LINE of half-width `halfW` tracing the
   * polyline `pts` (physics frame, y +down). Returns a single mesh.
   *
   * The earlier builder stacked a full DISC at every chain sample; with the
   * samples spaced ~ the disc radius those discs bulged perpendicular to the
   * line and produced a "string of beads / wheel-tread" look that churned as the
   * leg spun (user reject). This builder instead OFFSETS the centerline by ±halfW
   * using the MITER of adjacent segment normals, so the two edges run parallel to
   * the stroke at a uniform width — a clean drawn line. Round CAPS are added at
   * the two ENDS only (semicircles), so ends are rounded like a felt-tip pen but
   * the body of the line stays smooth with no interior bumps.
   */
  _buildStrokeRibbon(pts, halfW) {
    if (!pts || pts.length < 1) return null;
    // Render-frame centerline (render y = -physY). De-duplicate coincident pts.
    const C = [];
    for (const p of pts) {
      const q = { x: p.x, y: -p.y };
      if (C.length === 0 || Math.hypot(q.x - C[C.length - 1].x, q.y - C[C.length - 1].y) > 1e-5) C.push(q);
    }
    const positions = [];
    const idx = [];
    let base = 0;
    const pushTri = (a, b, c) => { idx.push(a, b, c); };

    if (C.length === 1) {
      // A dot — just a disc.
      this._pushDisc(positions, idx, C[0].x, C[0].y, halfW, 0, Math.PI * 2, () => base, (n) => { base = n; });
      const geo = this._thicken2D(positions, idx, LEG_THICK);
      return new THREE.Mesh(geo, this._legMat);
    }

    // Per-segment unit tangents.
    const seg = [];
    for (let i = 0; i < C.length - 1; i++) {
      let dx = C[i + 1].x - C[i].x, dy = C[i + 1].y - C[i].y;
      const len = Math.hypot(dx, dy) || 1;
      seg.push({ x: dx / len, y: dy / len });
    }
    // Per-vertex left-offset vector (miter). Interior vertices use the averaged
    // (miter) normal so the band keeps a constant perpendicular width through
    // bends; the miter length is clamped so a sharp corner doesn't spike out.
    const off = [];
    const MITER_MAX = 2.5; // clamp factor on the miter so sharp hooks stay bounded
    for (let i = 0; i < C.length; i++) {
      const tA = seg[Math.max(0, i - 1)];
      const tB = seg[Math.min(seg.length - 1, i)];
      // segment normals (left side): n = (-t.y, t.x)
      const nAx = -tA.y, nAy = tA.x;
      const nBx = -tB.y, nBy = tB.x;
      let mx = nAx + nBx, my = nAy + nBy;
      const ml = Math.hypot(mx, my);
      if (ml < 1e-4) { mx = nBx; my = nBy; } // 180° reversal — fall back
      else { mx /= ml; my /= ml; }
      // scale so the projection onto the segment normal equals halfW (constant width)
      let scale = halfW / Math.max(1e-3, (mx * nBx + my * nBy));
      const cap = halfW * MITER_MAX;
      if (scale > cap) scale = cap;
      off.push({ x: mx * scale, y: my * scale });
    }
    // Two parallel rails -> quads per segment.
    for (let i = 0; i < C.length; i++) {
      const L = { x: C[i].x + off[i].x, y: C[i].y + off[i].y };
      const R = { x: C[i].x - off[i].x, y: C[i].y - off[i].y };
      positions.push(L.x, L.y, R.x, R.y);
    }
    for (let i = 0; i < C.length - 1; i++) {
      const a = i * 2;            // L[i]
      const b = i * 2 + 1;        // R[i]
      const c = (i + 1) * 2;      // L[i+1]
      const d = (i + 1) * 2 + 1;  // R[i+1]
      pushTri(a, b, c);
      pushTri(b, d, c);
    }
    base = C.length * 2;

    // Round CAPS at the two ends (semicircle fans), oriented outward.
    const startT = seg[0];
    const endT = seg[seg.length - 1];
    // start cap: faces backward along -startT, sweeping from +normal to -normal
    {
      const a0 = Math.atan2(startT.x, -startT.y); // angle of the +left normal
      this._pushDisc(positions, idx, C[0].x, C[0].y, halfW, a0, a0 + Math.PI,
        () => base, (n) => { base = n; });
    }
    {
      const a0 = Math.atan2(-endT.x, endT.y); // angle of the -left normal at the tip
      this._pushDisc(positions, idx, C[C.length - 1].x, C[C.length - 1].y, halfW, a0, a0 + Math.PI,
        () => base, (n) => { base = n; });
    }

    // 2D triangle soup -> extrude in z into a slim 3D leg (front+back faces).
    const geo = this._thicken2D(positions, idx, LEG_THICK);
    return new THREE.Mesh(geo, this._legMat);
  }

  /** Append a filled circular fan (cx,cy radius r) sweeping from angle a0..a1 to
   * the 2D triangle soup. getBase()/setBase track the running vertex index. */
  _pushDisc(positions, idx, cx, cy, r, a0, a1, getBase, setBase) {
    const SEG = 10;
    let base = getBase();
    const center = base;
    positions.push(cx, cy);
    for (let k = 0; k <= SEG; k++) {
      const a = a0 + (a1 - a0) * (k / SEG);
      positions.push(cx + Math.cos(a) * r, cy + Math.sin(a) * r);
    }
    for (let k = 0; k < SEG; k++) idx.push(center, center + 1 + k, center + 2 + k);
    setBase(base + SEG + 2);
  }

  /** Turn a flat 2D triangle soup (positions[x,y...], idx) into a thin extruded
   * slab of `depth` (front face at +d/2, back at -d/2, plus side walls along the
   * outline is skipped — front+back read fine for a slim leg at this scale). */
  _thicken2D(positions2, idx2, depth) {
    const n = positions2.length / 2;
    const d = depth / 2;
    const pos = new Float32Array(n * 2 * 3);
    for (let i = 0; i < n; i++) {
      const x = positions2[i * 2], y = positions2[i * 2 + 1];
      pos[i * 3] = x; pos[i * 3 + 1] = y; pos[i * 3 + 2] = d;          // front
      pos[(n + i) * 3] = x; pos[(n + i) * 3 + 1] = y; pos[(n + i) * 3 + 2] = -d; // back
    }
    const tri = [];
    for (let i = 0; i < idx2.length; i += 3) {
      const a = idx2[i], b = idx2[i + 1], c = idx2[i + 2];
      tri.push(a, b, c);                       // front (CCW)
      tri.push(n + a, n + c, n + b);           // back (reversed)
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    geo.setIndex(tri);
    geo.computeVertexNormals();
    return geo;
  }

  /** Sync all meshes from physics bodies (call every frame). */
  sync(physics) {
    if (this.cubeMesh && physics.cube) {
      const p = physics.cube.position;
      // The axle is the cube's geometric CENTRE, and the leg stroke ribbons are
      // drawn in the leg-local frame whose origin == that axle. So the cube mesh
      // centre must sit EXACTLY at the physics cube centre (no drop) — then the
      // drawn leg lines emanate from the cube's middle and sweep down to the
      // ground (the reference look). render y = -physY. §C: the player lane is at
      // z = laneCurveZ(x) so the cube rides the same serpentine band.
      this.cubeMesh.position.set(p.x, -p.y, laneCurveZ(p.x));
      this.cubeMesh.rotation.z = -physics.cube.angle;
    }
    this._syncLegGroups(this.legGroups);
  }

  /** Sync the rival cube + its legs (call every frame when racing). */
  syncRival(rival) {
    if (this.rivalCubeMesh && rival.cube) {
      const p = rival.cube.position;
      // §C: rival lane base + serpentine curve at its own x.
      this.rivalCubeMesh.position.set(p.x, -p.y, this.rivalLaneZ + laneCurveZ(p.x));
      this.rivalCubeMesh.rotation.z = -rival.cube.angle;
    }
    this._syncLegGroups(this.rivalLegGroups);
  }

  _syncLegGroups(groups) {
    for (const lg of groups) {
      const body = lg.body;
      // Both legs share the same physics x/y; the static z offset gives the
      // two-leg (left/right) straddle, and §C adds laneCurveZ(x) so the legs ride
      // the serpentine band with the cube. Each leg spins at its own 180°-offset
      // angle — the alternating walk.
      lg.mesh.position.set(body.position.x, -body.position.y, lg.z + laneCurveZ(body.position.x));
      // render y = -physY -> a CCW physics rotation appears CW on screen.
      lg.mesh.rotation.z = -body.angle;
    }
  }

  /** §D — 3/4 chase camera, CLOSER + LOWER than before (reference framing: the
   * cube fills ~1/3 of the screen, sat in the lower-centre with a big sky above,
   * the winding ribbon receding ahead). Follows the serpentine z-curve so the
   * cube stays framed as the band snakes. Keeps the smooth _camY vertical glide. */
  updateCamera(physics) {
    const x = physics.cube ? physics.cube.position.x : physics.startX;
    const cubeRenderY = (physics.cube ? -physics.cube.position.y : 0.9);
    // SMOOTH VERTICAL FOLLOW (kept): the body snaps up at each stair step (so the
    // foot never penetrates). Easing a separate _camY toward the cube's render-y
    // turns the step-up into a glide so the screen never jolts.
    if (this._camY == null || !Number.isFinite(this._camY)) this._camY = cubeRenderY;
    else this._camY += (cubeRenderY - this._camY) * 0.10;
    // §C: the cube rides the serpentine band at z = laneCurveZ(x). Smoothly follow
    // that z too (ease) so the camera tracks the winding without snapping at the
    // S-curve peaks. (Camera lateral follow is a render-only effect — physics z=0.)
    const curveZ = laneCurveZ(x);
    if (this._camCurveZ == null || !Number.isFinite(this._camCurveZ)) this._camCurveZ = curveZ;
    else this._camCurveZ += (curveZ - this._camCurveZ) * 0.10;

    const racing = Math.abs(this.rivalLaneZ) > 1e-3;
    // CLOSER + LOWER 3/4 chase (was camX=x-9, camY=+11/12, camZ=13/15). Pull in to
    // ~6 behind, ~6.5 up, ~7.5 to the +z side: the cube gets bigger and the camera
    // looks more along the track (lower, flatter) so a tall sky sits above it.
    const camX = x - 6.0;
    const camY = this._camY + (racing ? 7.2 : 6.4);
    const camZ = (racing ? 8.5 : 7.6) + this._camCurveZ;
    this.camera.position.set(camX, camY, camZ);
    // Look-at: aim a bit AHEAD (x+3) and at a z biased toward the player lane (plus
    // the curve offset). Aiming the look-y BELOW the cube pushes the cube up into
    // the lower-centre of the frame and opens up the sky above (reference look).
    const lookZ = (racing ? this.rivalLaneZ * 0.30 : 0) + this._camCurveZ;
    this.camera.lookAt(x + 3.0, this._camY - 0.6, lookZ);
  }

  /** Debug/verify info: how many meshes the track group holds (continuous ribbon
   * ⇒ a small fixed count: per lane 1 top + 1 side, plus finish poles/flags — NOT
   * one slab per segment), and a few serpentine-curve z samples (evidence §B/§C). */
  ribbonInfo(physics) {
    const meshes = this.trackGroup ? this.trackGroup.children.filter((c) => c.isMesh).length : 0;
    const x0 = physics ? physics.startX : 0, x1 = physics ? physics.finishX : 1;
    const samples = [];
    for (let k = 0; k <= 8; k++) {
      const x = x0 + (x1 - x0) * (k / 8);
      samples.push({ x: +x.toFixed(1), z: +laneCurveZ(x).toFixed(3) });
    }
    return { trackMeshes: meshes, rivalLaneZ: this.rivalLaneZ, curveSamples: samples };
  }

  render() { this.renderer.render(this.scene, this.camera); }

  resize(wCss, hCss) {
    this.renderer.setSize(wCss, hCss, false);
    this.camera.aspect = wCss / hCss;
    this.camera.updateProjectionMatrix();
  }

  // ── procedural textures ──
  _makeChecker(n = 8) {
    const c = document.createElement('canvas');
    c.width = c.height = 128;
    const ctx = c.getContext('2d');
    const s = 128 / n;
    // higher-contrast purple check so the band reads as a checkered track (the
    // earlier two purples were too close and looked solid from the top-down view).
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
      ctx.fillStyle = ((i + j) % 2 === 0) ? '#7A2C9C' : '#C95FE0';
      ctx.fillRect(i * s, j * s, s, s);
    }
    const tex = new THREE.CanvasTexture(c);
    tex.colorSpace = THREE.SRGBColorSpace;
    tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
    return tex;
  }

  _makeFace(faceColor) {
    const hex = '#' + ((faceColor == null ? COL.playerFace : faceColor) >>> 0).toString(16).padStart(6, '0');
    const c = document.createElement('canvas');
    c.width = c.height = 128;
    const ctx = c.getContext('2d');
    ctx.clearRect(0, 0, 128, 128);
    ctx.fillStyle = hex;
    // two dot eyes
    ctx.beginPath(); ctx.arc(46, 54, 10, 0, Math.PI * 2); ctx.fill();
    ctx.beginPath(); ctx.arc(82, 54, 10, 0, Math.PI * 2); ctx.fill();
    // smile
    ctx.lineWidth = 6; ctx.strokeStyle = hex; ctx.lineCap = 'round';
    ctx.beginPath(); ctx.arc(64, 72, 18, 0.15 * Math.PI, 0.85 * Math.PI); ctx.stroke();
    const tex = new THREE.CanvasTexture(c);
    tex.colorSpace = THREE.SRGBColorSpace;
    return tex;
  }

  _disposeMesh(m) {
    if (!m) return;
    // a leg may be a Group of box meshes (compound limb) or a single mesh.
    if (m.isGroup || (m.children && m.children.length)) {
      m.traverse((o) => { if (o.isMesh && o.geometry) o.geometry.dispose(); });
      return;
    }
    if (m.geometry) m.geometry.dispose();
    if (Array.isArray(m.material)) m.material.forEach((mm) => mm.dispose && mm !== this._legMat && mm.dispose());
    else if (m.material && m.material !== this._legMat) m.material.dispose();
  }

  _disposeGroup(g) {
    g.traverse((o) => {
      if (o.isMesh) {
        if (o.geometry) o.geometry.dispose();
        if (Array.isArray(o.material)) o.material.forEach((m) => m.dispose && m.dispose());
        else if (o.material && o.material !== this._legMat) o.material.dispose();
      }
    });
  }
}
