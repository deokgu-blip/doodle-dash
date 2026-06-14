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

const RIBBON_DEPTH = 1.5;     // z-extrusion of the track — a THIN WINDING RIBBON like the reference (was 2.6, too wide; the cube now nearly fills the band width). lane offset / leg straddle / camera z are scaled to this below.
const RIBBON_DOWN = 0.8;      // SLAB DEPTH below the surface — a THINNER board like the reference (was 1.2, too chunky; the user asked for a slimmer track). The width (z-depth RIBBON_DEPTH 1.5) stays NARROW and UNCHANGED; only this vertical slab thickness shrank so the path reads as a slim plank, not a deep wall. The ceiling ROOF track + the rival lane reuse the SAME value (consistent thickness, all closed boxes).

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

// ── STRIPE TOP texture mapping (shared by the floor ribbon AND the tunnel roof so the
//    band rhythm is identical on both). The stripe texture (see _makeStripes) has
//    STRIPE_BANDS bands per repeat that alternate along the texture's U axis. We map
//    U = worldX * RIBBON_USCALE, so a single band spans (1/RIBBON_USCALE)/STRIPE_BANDS
//    world-x. With USCALE 0.42 and 2 bands/repeat ⇒ ≈1.19u per band → a clean transverse
//    bar a touch wider than the 1.5u path width (the reference's calm striped look). ──
const RIBBON_USCALE = 0.42;
const STRIPE_BANDS = 2;          // bands per texture repeat (even ⇒ seamless wrap)

// Two legs straddle the cube in DEPTH (z). The cube is ~1.08 deep; legs sit
// just outside its faces so they clearly read as a left and a right leg.
const LEG_THICK = 0.30;       // per-leg extrusion depth (slim — two legs, not 1 wheel)
// z distance of each leg from the cube centre. The two legs still sit CLEARLY to the
// LEFT (z<0) and RIGHT (z>0) of the cube (an unambiguous left foot + right foot), but
// scaled DOWN with the narrower ribbon (was 0.82 on a 2.6-wide band) so the pair
// straddles the cube just outside its faces (cube z-depth ≈0.81 ⇒ legs at ±0.50 sit
// just past it) and the whole walker fits the thin reference-width path.
const LEG_Z_OFFSET = 0.50;

export class Renderer {
  constructor(canvas) {
    this.canvas = canvas;
    // PERF: antialias OFF on high-DPR mobile. MSAA multiplies the per-pixel cost, and
    // on a retina/phone panel (DPR ≥ 2) the supersampling from the high pixel ratio
    // already hides aliasing — so MSAA is near-invisible there but still costs fill.
    // On low-DPR (desktop) panels we KEEP antialias (edges would otherwise show).
    const dpr = window.devicePixelRatio || 1;
    const wantAA = dpr < 2;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: wantAA, alpha: false, powerPreference: 'high-performance' });
    // L1 + PERF: the dominant mobile frame-drop cause is fill rate — backbuffer pixels
    // scale with pixelRatio². A retina phone reports DPR 3 ⇒ 9× the CSS pixels, which a
    // mobile GPU cannot fill at 60fps for a full-screen gradient + overdraw. We CAP the
    // ratio: ≤1.5 on high-DPR mobile (DPR≥2.5 ⇒ a ~66% pixel cut vs native, ~44% vs the
    // old cap of 2 with NO visible quality loss on this flat-shaded art), ≤2 otherwise.
    // Input stays in CSS px (setSize uses CSS units), so this is purely a render-res cap.
    const ratioCap = dpr >= 2.5 ? 1.5 : 2;
    this.renderer.setPixelRatio(Math.min(dpr, ratioCap));
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

    // BALL-FIELD meshes — one reused Mesh per ball (a sphere), sharing ONE SphereGeometry
    // (unit radius, scaled per ball) and ONE material so the whole pile is cheap. Built in
    // buildTrack from the physics ball count; positions updated each frame in syncBalls
    // (NO per-frame geometry/alloc). Player + rival piles each get their own mesh list.
    this.ballMeshes = [];      // player lane balls
    this.rivalBallMeshes = []; // rival lane balls

    // The leg is a DRAWN PEN LINE, not a shaded 3D object. A flat, unlit material
    // keeps it reading as a single solid black stroke (no per-bump specular
    // highlights that made the old circle-chain look like a beaded wheel).
    this._legMat = new THREE.MeshBasicMaterial({ color: COL.leg, side: THREE.DoubleSide });

    // ── SHARED TRACK MATERIALS / TEXTURE (built ONCE, reused on every buildTrack) ──
    // The stripe texture + the two track materials never change between rebuilds, so
    // making them once (instead of per buildTrack) removes a per-restart GPU
    // texture-upload + material-compile churn. The dispose paths skip these (kept).
    // STRIPES (was a checker): a calmer, cleaner reference look — TRANSVERSE bands that
    // run ACROSS the path (perpendicular to travel), alternating two purple tones. The
    // band colour changes along the path's U axis (forward x), so as the cube advances it
    // crosses one band after another (a striped road), NOT a busy checker grid.
    this._stripeTex = this._makeStripes();
    // TOP is UNLIT (MeshBasicMaterial): the stripe top ignores scene lighting and
    // always renders at the texture's full vivid colour at ANY camera angle.
    // DoubleSide so the top face shows regardless of triangle winding.
    this._topMat = new THREE.MeshBasicMaterial({ map: this._stripeTex, side: THREE.DoubleSide });
    // The side/under/end faces of the CLOSED box. PERF (§B.2): FrontSide (default
    // back-face culling) — the box is FULLY CLOSED (near+far walls, bottom, AND first/
    // last end-caps), so every visible face has its outward winding and the back faces
    // (the hidden interior) need never be rasterised. DoubleSide rasterised BOTH faces
    // of every wall/bottom/cap (≈98% of the scene's triangles), doubling the fragment
    // fill in exactly the bumps/serpentine region the user reports choppy. Back-face
    // culling halves that fragment work with NO visual change as long as the box is
    // closed (verified by the closed-section screenshot — no see-through). The ribbon
    // builder winds near-wall / far-wall / bottom / end-caps consistently outward.
    // PERF (UNLIT): the side/end/under faces are a flat SOLID edge colour — no texture,
    // just a single trackEdge tint shaded by the scene lights. These side faces span the
    // ENTIRE length of both lanes (≈98% of the scene's triangles), so MeshStandardMaterial
    // meant a full per-pixel PBR lighting eval (ambient + 2 dir lights, normal-based) on
    // the LARGEST fill area in the frame — on a 120Hz high-DPI panel that dominated GPU.
    // The art is flat single-colour, so an UNLIT MeshBasicMaterial renders the IDENTICAL
    // solid trackEdge colour at zero lighting cost (the big PBR→unlit win is unchanged).
    // SIDE: DoubleSide — FrontSide back-face-culled the cross-section END-CAPS from the
    // camera, so the box read as an OPEN ㄷ (user: "단면이 아직 ㄷ자, ㅁ으로 채워줘"). With the
    // sides/caps DoubleSide every face shows from any angle ⇒ a FILLED solid ㅁ. The fill
    // cost is minor (unlit shader, thin ribbon, DPR≤1.5) and the real frame-drop causes
    // (render-cap beat-skip + fixed-timestep judder) were fixed separately, not by this cull.
    this._sideMat = new THREE.MeshBasicMaterial({ color: COL.trackEdge, side: THREE.DoubleSide });

    // ── BALL-FIELD shared geometry + material (built ONCE, reused for every ball). ──
    // A modest-poly unit sphere (radius 1, scaled per ball) keeps the pile light: one
    // geometry + one material shared across all balls (player + rival), so a 14-ball pile
    // adds 14 draw calls of a tiny shared sphere. Lambert so the balls catch the key light
    // and read as 3D spheres (top brighter), at a fraction of PBR cost. A bright warm tone
    // (golden-amber) so the pile POPS against the lime field + purple track (palette-aware,
    // distinct from the cube blue / rival green). Shared ⇒ never disposed on rebuild.
    this._ballGeo = new THREE.SphereGeometry(1, 12, 10); // unit sphere, ~12×10 (low-poly, scaled per ball)
    this._ballMat = new THREE.MeshLambertMaterial({ color: 0xF2A93B }); // golden-amber pile
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
    // PERF (UNLIT pass): the track sides/top + leg + face + finish are all UNLIT
    // (MeshBasic) now — they ignore these lights entirely. Only the two cubes + axle
    // shafts are MeshLambert and need lighting. Lambert cost scales with the light
    // count, so we keep just AMBIENT + ONE directional (the key light) and DROP the old
    // magenta fill light (it existed mainly to tint the PBR side walls, which are now
    // flat Basic). Ambient bumped a touch so the unlit dropping of the fill doesn't make
    // the cube's shadowed faces too dark. Result: a clear top-bright / side-darker box.
    const amb = new THREE.AmbientLight(0xffffff, 0.82);
    this.scene.add(amb);
    const dir = new THREE.DirectionalLight(0xffffff, 0.85);
    dir.position.set(-4, 8, 6);
    this.scene.add(dir);
  }

  /** Build the track ribbon + ground from the physics floor bodies. When a
   * `rivalSpec` is given, also build a PARALLEL lane (offset in z) + the rival
   * cube so the two racers read side-by-side in a diagonal 3/4 view. */
  buildTrack(physics, track, rivalSpec = null) {
    // PERF / LEAK FIX: every restart()/loadTrack() rebuilds the track group + BOTH
    // character cubes. The old code only scene.remove()'d the cubes (and the trackGroup
    // freed geometry+materials but NOT textures), so each rebuild leaked the cube box +
    // face plane + axle-hub geometries, the cube/face materials, the per-cube face
    // CanvasTexture and the finish-flag CanvasTexture — measured at +12 geometries and
    // +2 textures PER restart (geo 33→753, tex 6→126 over 60 restarts). On the device
    // that GPU memory + the GC pressure compounds with every retry/next, which is the
    // "still too choppy" the user reports. We now FULLY dispose what we rebuild
    // (geometry + non-shared materials + their non-shared textures). Shared materials/
    // textures built once in the constructor are protected by _isSharedMat/_isSharedTex.
    if (this.trackGroup) { this.scene.remove(this.trackGroup); this._disposeGroup(this.trackGroup); }
    if (this.cubeMesh) { this.scene.remove(this.cubeMesh); this._disposeGroup(this.cubeMesh); }
    if (this.rivalCubeMesh) { this.scene.remove(this.rivalCubeMesh); this._disposeGroup(this.rivalCubeMesh); }
    // BALL-FIELD: drop old ball meshes (the shared geo/material are kept — see _isSharedMat).
    for (const m of this.ballMeshes) this.scene.remove(m);
    for (const m of this.rivalBallMeshes) this.scene.remove(m);
    this.ballMeshes = []; this.rivalBallMeshes = [];
    // PERF (SPIKE FIX): a track (re)build invalidates the prebuilt rival leg variant
    // geometry (lane z / chain can change), so free the cache here. The game re-arms it
    // via prebuildRivalLegVariants() right after this build, before the run starts.
    this._disposeRivalLegVariants();
    this.trackGroup = new THREE.Group();

    // Shared track materials/texture (built once in the constructor, reused here).
    const topMat = this._topMat;
    const sideMat = this._sideMat;

    // rival lane z-centre (parallel lane, one offset away on the -z side).
    this.rivalLaneZ = rivalSpec
      ? RIVAL_LANE_SIGN * (rivalSpec.laneOffset ?? 2.8) : 0;

    // §B+§C: each lane is ONE CONTINUOUS ribbon mesh (a constant-width band of
    // RIBBON_DEPTH) whose centre-line follows (x, surfaceY(x), laneZ+laneCurveZ(x)).
    // The surface (flats, ramps, stair steps) is traced unbroken, and the
    // serpentine z-curve is baked into the same vertices — so B (continuous) and C
    // (winding) are solved in one builder. Physics x is untouched.
    this._buildRibbon(physics, 0, topMat, sideMat);
    if (rivalSpec) this._buildRibbon(physics, this.rivalLaneZ, topMat, sideMat);

    // TUNNEL low ceilings — a ROOF made of the SAME track ribbon (same purple checker,
    // same narrow width, same thick slab) laid OVER the tunnel at the head-room line.
    // The cube passes through the GAP between the floor track and this ceiling track;
    // the gap height reads "how low" at a glance (WYSIWYG). One per lane.
    this._buildCeilings(physics, 0, topMat, sideMat);
    if (rivalSpec) this._buildCeilings(physics, this.rivalLaneZ, topMat, sideMat);
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

    // ── BALL-FIELD: build one reused sphere mesh per physics ball (player lane). The
    //    rival pile (its own physics field) is built when syncRival first sees it. ──
    this._buildBalls(physics, 0);
  }

  /** Build the reusable sphere meshes for a walker's ball field on the given lane.
   * One Mesh per ball sharing the constructor's _ballGeo + _ballMat; scaled to the
   * ball's radius. Old meshes are removed/freed (geometry is shared ⇒ only the Mesh
   * wrapper is dropped). Positions are set every frame in _syncBalls (no rebuild). */
  _buildBalls(physics, laneZ) {
    const isRival = Math.abs(laneZ) > 1e-3;
    const listRef = isRival ? 'rivalBallMeshes' : 'ballMeshes';
    // tear down any previous meshes (geometry/material are shared — only remove the Mesh).
    for (const m of this[listRef]) this.scene.remove(m);
    this[listRef] = [];
    const balls = physics.balls;
    if (!balls || !balls.length) return;
    for (let i = 0; i < balls.length; i++) {
      const b = balls[i];
      const mesh = new THREE.Mesh(this._ballGeo, this._ballMat);
      const r = b.r || 0.34;
      mesh.scale.set(r, r, r);                 // unit sphere → ball radius
      mesh.position.set(b.x, -b.y, laneZ + laneCurveZ(b.x));
      this.scene.add(mesh);
      this[listRef].push(mesh);
    }
  }

  /** Update every ball mesh's position from the live physics field (call every frame).
   * Render y = -physY; the ball rides the serpentine band at its own x. Position-only
   * write (no geometry, no allocation). The pile is small (N<=20) ⇒ trivially cheap. */
  _syncBalls(physics, laneZ, meshes) {
    const balls = physics.balls;
    if (!balls || !meshes.length) return;
    const n = Math.min(balls.length, meshes.length);
    for (let i = 0; i < n; i++) {
      const b = balls[i];
      meshes[i].position.set(b.x, -b.y, laneZ + laneCurveZ(b.x));
    }
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
      // PLANKS VOID: the gap between boards is a recovery trench in PHYSICS (gap:true,
      // tagged plankGap) but a VISUAL VOID — omit it from the render surface so the
      // boards read as SEPARATE closed boxes with the green void showing between them.
      if (s.plankGap) continue;
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
    // ── PERF (render-tessellation ↔ physics DECOUPLE, §B.1) ──────────────────────
    // The PHYSICS bumps fields are sampled into MANY short sub-ramps (~0.45u apart,
    // ~8/period) so the collision surface is analytic with zero penetration. But the
    // RENDER only needs enough rings to look like a smooth rolling wave — the bumps
    // amplitude is small (≈0.5), so the band reads smooth at HALF that ring density.
    // The bumps fields are ~65% of the course, so their dense profile points dominate
    // the ribbon vert/tri count AND the per-frame fragment fill in exactly the region
    // the user reports choppy ("구불구불한 길"). We therefore DECIMATE the profile
    // points that fall inside a bumps x-window (keep every BUMP_STRIDE-th), sampling
    // the TRUE physics surfaceYAt(x) only at the kept x's. Physics _segs / surfaceYAt /
    // every gate are UNTOUCHED — this is purely how densely we tessellate the render
    // surface. Stair risers / ramp / flat / gap / tunnel points are ALL kept (only the
    // gentle bumps wave is thinned). Endpoints of each bumps run are always kept so the
    // wave start/end aligns with the neighbouring segment exactly (no seam gap).
    const segs = physics._segs || [];
    const bumpWins = [];
    for (const s of segs) if (s.kind === 'bumps') {
      const last = bumpWins[bumpWins.length - 1];
      if (last && Math.abs(s.x0 - last.x1) < 1e-3) last.x1 = s.x1;       // merge contiguous
      else bumpWins.push({ x0: s.x0, x1: s.x1 });
    }
    const inBumps = (x) => {
      for (const w of bumpWins) if (x >= w.x0 - 1e-6 && x <= w.x1 + 1e-6) return w;
      return null;
    };
    const BUMP_STRIDE = 2;   // keep 1 of every 2 bumps-region profile points (≈half the rings)
    // Thin the PROFILE points first (drop interior bumps points), keeping run ends.
    const thinned = [];
    let bumpKeepCount = 0; let curWin = null;
    for (let i = 0; i < prof.length; i++) {
      const p = prof[i];
      const w = inBumps(p.x);
      if (!w) { thinned.push(p); curWin = null; continue; }
      // bumps point: always keep the window's first/last; otherwise keep every Nth.
      const isEnd = Math.abs(p.x - w.x0) < 1e-6 || Math.abs(p.x - w.x1) < 1e-6;
      if (w !== curWin) { curWin = w; bumpKeepCount = 0; }
      if (isEnd || (bumpKeepCount % BUMP_STRIDE) === 0) thinned.push(p);
      bumpKeepCount++;
    }
    // PLANKS VOID detection: a span between two consecutive profile points whose
    // MIDPOINT falls in a VISUAL VOID (a plankGap recovery trench, or a bare gap with
    // no rendered surface) is the empty space between boards. We mark a BREAK there so
    // the ribbon is CLOSED (end-capped) on both sides and never bridges a quad across
    // the void — the boards read as separate closed boxes. NB: a plankGap DOES carry a
    // physics recovery surfFn (so surfaceYAt isn't null there), so we test the SEGMENT
    // KIND at the midpoint, not surfaceYAt, to find the visual void.
    const segCoversMid = (mid) => {
      for (const s of segs) {
        if (mid < s.x0 - 1e-6 || mid > s.x1 + 1e-6) continue;
        if (s.plankGap || s.kind === 'gap') return false; // a void (not rendered)
        return true;                                      // a real rendered surface
      }
      return false;                                       // outside any segment ⇒ void
    };
    const isVoidSpan = (a, b) => !segCoversMid((a + b) / 2);
    // Densify: insert intermediate x samples (RIBBON_DX) between profile points so
    // the z-curve bends smoothly over long flats/ramps; keep the exact profile pts
    // (incl. stair risers) so steps stay sharp. Bumps gaps are now ~2× wider after the
    // decimation; we DON'T re-densify inside a bumps window (that would undo the cut),
    // so the bumps span between two kept points stays a single straight render segment.
    // `breakAfter[i]` ⇒ a void follows ring i (close this box, start a new one at i+1).
    const xs = [];
    const breakAfter = [];
    for (let i = 0; i < thinned.length - 1; i++) {
      const a = thinned[i], b = thinned[i + 1];
      xs.push(a.x); breakAfter.push(false);
      const span = b.x - a.x;
      const mid = (a.x + b.x) / 2;
      // a void (plank gap) span: mark a break, do NOT densify or bridge across it.
      if (isVoidSpan(a.x, b.x)) { breakAfter[breakAfter.length - 1] = true; continue; }
      // skip re-densification when this span lies inside a bumps window (keep it coarse).
      if (span > RIBBON_DX * 1.5 && !inBumps(mid)) {
        const n = Math.floor(span / RIBBON_DX);
        for (let k = 1; k < n; k++) {
          const t = k / n;
          xs.push(a.x + span * t); breakAfter.push(false);
        }
      }
    }
    xs.push(thinned[thinned.length - 1].x); breakAfter.push(false);
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
    // we get a top face (stripes) + the two side walls + a bottom (edge colour).
    const topPos = [], topUV = [], topIdx = [];
    const sidePos = [], sideIdx = [];
    let topV = 0, sideV = 0;
    let prevX = null;
    const N = xs.length;
    // STRIPE mapping: the stripe texture has STRIPE_BANDS bands per repeat that change
    // along the U axis (the path's FORWARD x). U = x*RIBBON_USCALE, so one band spans
    // (1/RIBBON_USCALE)/STRIPE_BANDS ≈ 1.19u of world-x — a clean transverse rhythm (a
    // band a touch under the 1.5u path width reads as the reference's chunky bars, not a
    // busy grid). V is the across-path coord (0→1) — the stripes are CONSTANT in V (they
    // run straight across the band), so vRepeat is just a single span.
    const uScale = RIBBON_USCALE;
    const vRepeat = 1;
    for (let i = 0; i < N; i++) {
      const x = xs[i];
      const ry = -surfY(x);                 // render y (up)
      const cz = laneZ + laneCurveZ(x);     // serpentine z centre
      const zN = cz - half, zF = cz + half; // near / far edges
      // TOP strip: 2 verts (near, far) at the surface.
      topPos.push(x, ry, zN, x, ry, zF);
      topUV.push(x * uScale, 0, x * uScale, vRepeat);
      // SIDE/bottom strip: top edges (= surface) + bottom edges (dropped).
      const by = ry - RIBBON_DOWN;
      sidePos.push(
        x, ry, zN,  x, by, zN,   // near wall top, bottom
        x, ry, zF,  x, by, zF    // far wall top, bottom
      );
      // PLANKS VOID: a break BEFORE this ring (the previous span was a void) ⇒ do NOT
      // bridge a quad across the empty space, and the previous ring CLOSES its board.
      const breakBefore = i > 0 && breakAfter[i - 1];
      if (i > 0 && !breakBefore) {
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
      // ── END CAPS (ㅁ closure): the cross-section quad so every box is fully closed —
      //    no hollow see-through at the path ends OR at a PLANKS void boundary. We cap
      //    the FIRST/LAST ring of the whole lane AND the ring on EACH side of a void
      //    (this ring if a break is BEFORE it = a board's left face; or if a break is
      //    AFTER it = a board's right face). Each ring's 4 side verts (nearTop +0,
      //    nearBot +1, farTop +2, farBot +3) form the cap quad; DoubleSide ⇒ solid. ──
      const capHere = (i === 0 || i === N - 1) || breakBefore || breakAfter[i];
      if (capHere) {
        const b4 = i * 4;
        sideIdx.push(b4 + 0, b4 + 2, b4 + 3, b4 + 0, b4 + 3, b4 + 1);
        // ...AND the reverse winding so the cap is solid from EITHER face. The side
        // material is FrontSide (perf): a single-winding cap got back-face-culled from
        // the camera ⇒ the cross-section read as an OPEN ㄷ. Both windings ⇒ filled ㅁ.
        // (Only the cap quads are double-wound — negligible fill; walls stay culled.)
        sideIdx.push(b4 + 0, b4 + 3, b4 + 2, b4 + 0, b4 + 1, b4 + 3);
      }
      prevX = x;
    }
    const topGeo = new THREE.BufferGeometry();
    topGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(topPos), 3));
    topGeo.setAttribute('uv', new THREE.BufferAttribute(new Float32Array(topUV), 2));
    topGeo.setIndex(topIdx);
    topGeo.computeVertexNormals();
    const topMesh = new THREE.Mesh(topGeo, topMat);
    topMesh.userData.ribbon = true;
    this.trackGroup.add(topMesh);

    const sideGeo = new THREE.BufferGeometry();
    sideGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(sidePos), 3));
    sideGeo.setIndex(sideIdx);
    sideGeo.computeVertexNormals();
    const sideMesh = new THREE.Mesh(sideGeo, sideMat);
    sideMesh.userData.ribbon = true;
    this.trackGroup.add(sideMesh);
  }

  /** Build the TUNNEL as a ROOF made of the SAME TRACK RIBBON laid OVERHEAD (the
   * reference's "low route on top" look) — NOT a front-facing box/bar you crash into.
   * Each tunnel renders as one extra ribbon segment, identical in appearance to the
   * floor track (same purple stripe top, same narrow width RIBBON_DEPTH, same thin
   * slab RIBBON_DOWN), but flipped so its UNDERSIDE sits exactly at the physics
   * ceilingY (the head-room gate, WYSIWYG). It rides the SAME lane centre + serpentine
   * curve as the floor track, so it sits DIRECTLY ABOVE the floor track (same z) and
   * never covers the cube's front: the cube + low legs pass through the GAP between
   * the floor track below and this ceiling track above, and that gap height reads
   * "how low this passage is" at a glance. A too-LONG leg's rotating sweep strikes
   * this overhead track's underside (BLOCKED); a SHORT leg ducks through the gap.
   * Render-only — ceilingY is the physics gate; the gate logic is untouched.
   *
   * Geometry: I sample x across [x0,x1] (densified for the serpentine bend), put the
   * stripe TOP face at the slab's TOP (ceilingY − RIBBON_DOWN, i.e. render y =
   * -ceilingY + RIBBON_DOWN) and the slab walls/underside dropping down to the
   * head-room line (render y = -ceilingY). So the bright stripe is the visible top
   * of the overhead board and the underside is the surface a long leg hits. */
  _buildCeilings(physics, laneZ, topMat, sideMat) {
    const ceils = physics.ceilingBodies;
    if (!ceils || !ceils.length) return;
    const half = RIBBON_DEPTH / 2;
    const uScale = RIBBON_USCALE, vRepeat = 1;   // SAME stripe mapping as the floor ribbon
    for (const c of ceils) {
      // densify x across the tunnel span so the overhead board snakes with the lane.
      const xs = [];
      const span = c.x1 - c.x0;
      const n = Math.max(1, Math.floor(span / RIBBON_DX));
      for (let k = 0; k <= n; k++) xs.push(c.x0 + span * (k / n));

      // The board's UNDERSIDE sits at the head-room line (render y = -ceilingY); its
      // stripe TOP is RIBBON_DOWN above that (same slab thickness as the floor track).
      const underRy = -c.ceilingY;          // underside (the surface a too-long leg hits)
      const topRy = underRy + RIBBON_DOWN;   // bright stripe top of the overhead board

      const topPos = [], topUV = [], topIdx = [];
      const sidePos = [], sideIdx = [];
      const N = xs.length;
      for (let i = 0; i < N; i++) {
        const x = xs[i];
        const cz = laneZ + laneCurveZ(x);    // SAME lane centre + serpentine as the floor
        const zN = cz - half, zF = cz + half;
        // TOP strip (checker) at the board's top face.
        topPos.push(x, topRy, zN, x, topRy, zF);
        topUV.push(x * uScale, 0, x * uScale, vRepeat);
        // SIDE/underside strip: top edges (= board top) + bottom edges (= underside).
        sidePos.push(
          x, topRy, zN,  x, underRy, zN,    // near wall top, bottom
          x, topRy, zF,  x, underRy, zF     // far wall top, bottom
        );
        if (i > 0) {
          const a = (i - 1) * 2, b = a + 1, cc = i * 2, d = cc + 1;
          topIdx.push(a, cc, b, b, cc, d);
          const p = (i - 1) * 4, q = i * 4;
          // near wall
          sideIdx.push(p + 0, p + 1, q + 0, q + 0, p + 1, q + 1);
          // far wall
          sideIdx.push(p + 2, q + 2, p + 3, p + 3, q + 2, q + 3);
          // UNDERSIDE (faces down — the head-room surface a long leg strikes)
          sideIdx.push(p + 1, p + 3, q + 1, q + 1, p + 3, q + 3);
        }
        // END CAPS (ㅁ closure): cross-section quad at the FIRST and LAST ring so the
        // overhead board is a fully closed box too — no hollow see-through at the ends.
        if (i === 0 || i === N - 1) {
          const b4 = i * 4;
          sideIdx.push(b4 + 0, b4 + 2, b4 + 3, b4 + 0, b4 + 3, b4 + 1);
          // reverse winding too ⇒ cap solid from either face (FrontSide side material).
          sideIdx.push(b4 + 0, b4 + 3, b4 + 2, b4 + 0, b4 + 1, b4 + 3);
        }
      }
      const topGeo = new THREE.BufferGeometry();
      topGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(topPos), 3));
      topGeo.setAttribute('uv', new THREE.BufferAttribute(new Float32Array(topUV), 2));
      topGeo.setIndex(topIdx);
      topGeo.computeVertexNormals();
      const topMesh = new THREE.Mesh(topGeo, topMat);
      this.trackGroup.add(topMesh);

      const sideGeo = new THREE.BufferGeometry();
      sideGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(sidePos), 3));
      sideGeo.setIndex(sideIdx);
      sideGeo.computeVertexNormals();
      const sideMesh = new THREE.Mesh(sideGeo, sideMat);
      this.trackGroup.add(sideMesh);
    }
  }

  /** Build a smiley character cube (body colour + face colour) with a dot-eye
   * face on +z. Shared by player and rival (different palette). The legs both
   * spin about the cube CENTRE (the physics axle, AXLE_X=AXLE_Y=0), so we add a
   * visible AXLE HUB at that centre so the two legs read as spokes on ONE axle
   * (reference close-up) instead of two strokes overlapping in the middle. */
  _buildCharacterCube(bodyColor, faceColor) {
    const cubeGeo = new THREE.BoxGeometry(PHYS_CONST.CUBE_SIZE, PHYS_CONST.CUBE_SIZE, PHYS_CONST.CUBE_SIZE * 0.9);
    // PERF (UNLIT-ish): the cube is small (few pixels), but PBR (Standard) is the most
    // expensive shader. We drop to MeshLambertMaterial — a CHEAP single-pass diffuse
    // (1-light Gouraud-ish, no specular/roughness/metalness sampling) — so the cube
    // still catches the directional light and reads as a 3D box (top face brighter than
    // the side faces), NOT a flat single-colour sticker, but at a fraction of the PBR
    // cost. The ambient + 1 directional light kept in _buildLights drive this shading.
    const cubeMat = new THREE.MeshLambertMaterial({ color: bodyColor });
    const cube = new THREE.Mesh(cubeGeo, cubeMat);
    const faceTex = this._makeFace(faceColor);
    const faceMat = new THREE.MeshBasicMaterial({ map: faceTex, transparent: true });
    const faceGeo = new THREE.PlaneGeometry(PHYS_CONST.CUBE_SIZE * 0.92, PHYS_CONST.CUBE_SIZE * 0.92);
    const face = new THREE.Mesh(faceGeo, faceMat);
    face.position.z = PHYS_CONST.CUBE_SIZE * 0.46;
    cube.add(face);
    cube.add(this._buildAxleHub());
    return cube;
  }

  /** Build the AXLE HUB — a short cylinder lying along z that spans the two legs
   * (z = ±LEG_Z_OFFSET = ±0.82) so it reads as a single axle the two legs are
   * threaded onto, plus a bright bolt-cap (bright disc + dark centre dot) on the
   * cube's FRONT face (+z) — the reference's "bright circle with a centre dot"
   * the legs spin around. Children of the cube ⇒ it inherits the cube's
   * position/rotation/bob automatically. Render-only. */
  _buildAxleHub() {
    const hub = new THREE.Group();
    const AXLE_R = 0.085;                          // axle-bar radius (slim shaft)
    const HALF_Z = LEG_Z_OFFSET;                  // reach the two leg planes (±0.82)
    const CAP_R = 0.16;                            // bright bolt-cap disc radius
    const FRONT_Z = PHYS_CONST.CUBE_SIZE * 0.46;   // cube front face plane (= face plane)
    const HUB_LIGHT = 0xCDEBFF;                   // bright sky/white (our blue palette)
    const HUB_RING = 0x8FCBF0;                     // mid-blue ring shading
    const HUB_BOLT = 0x0E2A3A;                     // dark centre dot (== playerFace dark)

    // axle BAR: a slim shaft laid along z, threading the two leg planes so the
    // two strokes read as spokes on ONE axle. Sits behind the front face.
    // PERF (UNLIT-ish): same as the cube — drop the axle shaft from PBR to the cheap
    // 1-light diffuse MeshLambertMaterial. It still gets a touch of cylindrical shading
    // from the directional light (so the round shaft reads), at a fraction of the cost.
    const barMat = new THREE.MeshLambertMaterial({ color: HUB_RING });
    const bar = new THREE.Mesh(
      new THREE.CylinderGeometry(AXLE_R, AXLE_R, HALF_Z * 2, 16),
      barMat
    );
    bar.rotation.x = Math.PI / 2;                  // lay the cylinder along z
    hub.add(bar);

    // FRONT bolt cap: a bright disc + dark centre dot on the cube's front face,
    // sitting BELOW the smiley (the legs' axle is the cube centre, and the face
    // dots/smile are in the upper half, so the cap reads as the "wheel hub" the
    // legs pivot on without covering the eyes). Slightly proud of the face plane.
    const capMat = new THREE.MeshBasicMaterial({ color: HUB_LIGHT });
    const ringMat = new THREE.MeshBasicMaterial({ color: HUB_RING });
    const boltMat = new THREE.MeshBasicMaterial({ color: HUB_BOLT });
    const ring = new THREE.Mesh(new THREE.CircleGeometry(CAP_R, 22), ringMat);
    ring.position.set(0, 0, FRONT_Z + 0.012);
    hub.add(ring);
    const disc = new THREE.Mesh(new THREE.CircleGeometry(CAP_R * 0.78, 22), capMat);
    disc.position.set(0, 0, FRONT_Z + 0.014);
    hub.add(disc);
    const bolt = new THREE.Mesh(new THREE.CircleGeometry(CAP_R * 0.34, 16), boltMat);
    bolt.position.set(0, 0, FRONT_Z + 0.016);
    hub.add(bolt);
    return hub;
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

  /** Rebuild the RIVAL's leg meshes (same builder, centred on the rival lane z).
   *
   * PERF (SPIKE FIX): the rival (BOLT) only ever alternates between TWO fixed leg
   * presets ('long' for wall/gap/stairs, 'short' for tunnel) as it crosses each gate.
   * The old per-swap path called this on every gate, and _buildLegGroups disposed the
   * old leg meshes and built TWO new BufferGeometries (+ computeVertexNormals + GPU
   * upload + scene add/remove) PER SWAP — a periodic per-gate geometry-rebuild that
   * showed up as the running (not-drawing) ms-max spike. We instead PRE-BUILD both
   * variants' leg-groups ONCE (prebuildRivalLegVariants) and, on a swap, just TOGGLE
   * which variant is visible (zero new geometry). This rebuild path is now only used
   * when no prebuilt variant matches (defensive fallback / non-preset rival). */
  rebuildRivalLegs(rival) {
    // If a prebuilt variant cache exists, prefer the visibility-toggle path so a swap
    // never creates geometry. The caller (game.swapRivalLegVariant) normally uses the
    // cache directly; this guards any legacy rebuildRivalLegs call.
    if (this._rivalLegVariants) {
      const v = this._rivalLegVariants.get(rival._lastPreset);
      if (v) { this._activateRivalVariant(rival._lastPreset, rival); return; }
    }
    this.rivalLegGroups = this._buildLegGroups(rival, this.rivalLegGroups, this.rivalLaneZ);
  }

  /** PRE-BUILD the rival's leg-group geometry for EACH leg preset it will ever use,
   * ONCE, so a mid-run gate swap is a pure visibility toggle (no new BufferGeometry,
   * no computeVertexNormals, no scene add/remove — the periodic spike's root cause).
   *
   * `variants` is [{ preset, legs }] where `legs` is the physics legs array a fresh
   * walker produces for that preset (built by the game from a throwaway Physics so the
   * live rival is untouched). Each variant's two leg meshes are built here and added to
   * the scene HIDDEN; activating a variant flips visible=true on it and false on the
   * others, and (re)binds its groups' `body` refs to the LIVE rival legs so
   * _syncLegGroups drives the visible variant from the live physics each frame. */
  prebuildRivalLegVariants(variants) {
    // tear down any previous cache (track reload / restart) so we don't leak.
    this._disposeRivalLegVariants();
    // also remove any legacy per-build rival leg groups (we now drive via the cache).
    for (const lg of this.rivalLegGroups) { this.scene.remove(lg.mesh); this._disposeMesh(lg.mesh); }
    this.rivalLegGroups = [];

    this._rivalLegVariants = new Map();
    for (const { preset, legs } of variants) {
      // build the leg-group meshes for THIS preset's legs (geometry built ONCE here).
      const groups = this._buildLegGroupsFromLegs(legs, this.rivalLaneZ, /*hidden*/ true);
      this._rivalLegVariants.set(preset, groups);
    }
    this._activeRivalPreset = null;
  }

  /** Activate (make visible) the rival leg variant for `preset`, hide the rest, and
   * rebind that variant's groups to the LIVE rival legs (by matching side) so the
   * per-frame _syncLegGroups reads the live physics body. ZERO geometry work. */
  _activateRivalVariant(preset, rival) {
    if (!this._rivalLegVariants) return false;
    const target = this._rivalLegVariants.get(preset);
    if (!target) return false;
    // hide every variant, show the target.
    for (const [p, groups] of this._rivalLegVariants) {
      const show = (p === preset);
      for (const lg of groups) lg.mesh.visible = show;
    }
    // bind the visible variant's groups to the live rival leg bodies (match by side),
    // so _syncLegGroups(this.rivalLegGroups) drives the shown meshes from live physics.
    // RENDER INTERPOLATION: also refresh the leg's phaseOffset from the live leg so the
    // interpolated world angle (built from the walker's θ/tilt + this offset) is correct.
    for (const lg of target) {
      const live = rival.legs.find((l) => l.side === lg.side);
      if (live) { lg.body = live.body; lg.phaseOffset = live.phaseOffset || 0; }
    }
    this.rivalLegGroups = target;
    this._activeRivalPreset = preset;
    return true;
  }

  _disposeRivalLegVariants() {
    if (!this._rivalLegVariants) return;
    for (const [, groups] of this._rivalLegVariants) {
      for (const lg of groups) { this.scene.remove(lg.mesh); this._disposeMesh(lg.mesh); }
    }
    this._rivalLegVariants = null;
    this._activeRivalPreset = null;
  }

  /** Shared leg-group builder for both racers (the PLAYER path: redraw rebuilds).
   * `laneZ` is the lane centre; each leg is straddled ±LEG_Z_OFFSET about it.
   * Disposes the old groups, returns the new array. */
  _buildLegGroups(physics, oldGroups, laneZ) {
    for (const lg of oldGroups) { this.scene.remove(lg.mesh); this._disposeMesh(lg.mesh); }
    return this._buildLegGroupsFromLegs(physics.legs, laneZ, false);
  }

  /** Build leg-group meshes from an explicit `legs` array (no disposal of anything).
   * `hidden` adds them to the scene with visible=false (used to pre-build the rival's
   * preset variants). Geometry is built here ONCE per leg. */
  _buildLegGroupsFromLegs(legs, laneZ, hidden) {
    const groups = [];
    for (const l of legs) {
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
      if (hidden) grp.visible = false;
      this.scene.add(grp);
      // z offset by side (straddle) PLUS the lane centre offset. The serpentine
      // curve z (laneCurveZ(x)) is ADDED per-frame in _syncLegGroups (it depends
      // on the live x), so we keep the static part here and the dynamic part there.
      // RENDER INTERPOLATION: store the leg's phaseOffset so _syncLegGroups can rebuild
      // the INTERPOLATED world angle (= lerp(θ) + phaseOffset + lerp(tilt)) from the
      // walker, instead of reading the raw (un-interpolated) body.angle.
      groups.push({ mesh: grp, body, side: l.side, phaseOffset: l.phaseOffset || 0, laneZ, z: laneZ + l.side * LEG_Z_OFFSET });
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
    // PERF: the leg uses _legMat (MeshBasicMaterial, UNLIT) which ignores normals,
    // and _thicken2D is leg-only — so computeVertexNormals() was pure waste: it
    // allocated a normal Float32Array (GC garbage) + CPU on EVERY redraw. Dropped.
    return geo;
  }

  /** Sync all meshes from physics bodies (call every frame).
   *
   * RENDER INTERPOLATION: `alpha` (0..1, the game's leftover-accumulator ratio) blends the
   * PREVIOUS sim tick's pose with the CURRENT one so the drawn cube/legs/camera advance
   * CONTINUOUSLY even when rAF jitter makes a frame drain 0 ticks then the next drain 2
   * (the mobile micro-stutter fix). When alpha is omitted/0 with no prev snapshot the
   * walker getters return the current value — i.e. identical to the old direct read. The
   * SIM is untouched; only the drawn pose lerps. */
  sync(physics, alpha = 0) {
    if (this.cubeMesh && physics.cube) {
      // The axle is the cube's geometric CENTRE, and the leg stroke ribbons are
      // drawn in the leg-local frame whose origin == that axle. So the cube mesh
      // centre must sit EXACTLY at the physics cube centre (no drop) — then the
      // drawn leg lines emanate from the cube's middle and sweep down to the
      // ground (the reference look). render y = -physY. §C: the player lane is at
      // z = laneCurveZ(x) so the cube rides the same serpentine band.
      // INTERPOLATED forward x + full body y + tilt (lerp prev→curr by alpha).
      const ix = physics.interpX(alpha);
      const iy = physics.interpBodyY(alpha);
      this.cubeMesh.position.set(ix, -iy, laneCurveZ(ix));
      this.cubeMesh.rotation.z = -physics.interpAngle(alpha);
    }
    this._syncLegGroups(this.legGroups, physics, alpha);
    // BALL-FIELD: drive the reused player-lane sphere meshes from the live pile.
    if (this.ballMeshes.length) this._syncBalls(physics, 0, this.ballMeshes);
  }

  /** Sync the rival cube + its legs (call every frame when racing). alpha: see sync(). */
  syncRival(rival, alpha = 0) {
    if (this.rivalCubeMesh && rival.cube) {
      // §C: rival lane base + serpentine curve at its own x. INTERPOLATED (see sync()).
      const ix = rival.interpX(alpha);
      const iy = rival.interpBodyY(alpha);
      this.rivalCubeMesh.position.set(ix, -iy, this.rivalLaneZ + laneCurveZ(ix));
      this.rivalCubeMesh.rotation.z = -rival.interpAngle(alpha);
    }
    this._syncLegGroups(this.rivalLegGroups, rival, alpha);
    // BALL-FIELD (rival lane): lazily build the rival pile meshes the first time we see
    // its field (the rival walker is built after the player buildTrack), then drive them.
    if (rival.ballCount && this.rivalBallMeshes.length !== rival.ballCount) {
      this._buildBalls(rival, this.rivalLaneZ);
    }
    if (this.rivalBallMeshes.length) this._syncBalls(rival, this.rivalLaneZ, this.rivalBallMeshes);
  }

  _syncLegGroups(groups, physics = null, alpha = 0) {
    // RENDER INTERPOLATION: the legs pivot about the cube axle (== cube x/y), so they
    // ride the INTERPOLATED cube x/y; each leg's world spin angle is rebuilt from the
    // walker's interpolated θ + the leg's constant phaseOffset + interpolated tilt
    // (interpLegAngle). Falls back to the raw body fields when no walker is passed
    // (defensive — keeps any legacy direct call working).
    const ax = physics ? physics.interpX(alpha) : null;
    const ay = physics ? physics.interpBodyY(alpha) : null;
    for (const lg of groups) {
      const body = lg.body;
      const bx = (ax != null) ? ax : body.position.x;
      const by = (ay != null) ? ay : body.position.y;
      // Both legs share the same physics x/y; the static z offset gives the
      // two-leg (left/right) straddle, and §C adds laneCurveZ(x) so the legs ride
      // the serpentine band with the cube. Each leg spins at its own 180°-offset
      // angle — the alternating walk.
      lg.mesh.position.set(bx, -by, lg.z + laneCurveZ(bx));
      // render y = -physY -> a CCW physics rotation appears CW on screen.
      const ang = physics ? physics.interpLegAngle(alpha, lg.phaseOffset) : body.angle;
      lg.mesh.rotation.z = -ang;
    }
  }

  /** §D — 3/4 chase camera, CLOSER + LOWER than before (reference framing: the
   * cube fills ~1/3 of the screen, sat in the lower-centre with a big sky above,
   * the winding ribbon receding ahead). Follows the serpentine z-curve so the
   * cube stays framed as the band snakes. Keeps the smooth _camY vertical glide. */
  updateCamera(physics, dt = 1 / 60, alpha = 0) {
    // RENDER INTERPOLATION: follow the INTERPOLATED forward x + bob-free base y (same
    // alpha the cube/legs use) so the camera tracks the cube in lock-step — no relative
    // drift between the (interpolated) cube and the camera under rAF jitter. The camera
    // additionally low-passes these (the _camY / _camCurveZ ease below), so the interp is
    // a tiny sub-tick refinement on an already-smoothed follow.
    const x = physics.cube ? physics.interpX(alpha) : physics.startX;
    // Track the BOB-FREE base height (physics.bodyCamY), NOT the bobbing cube y, so
    // the camera glides with the terrain trend only — the cube bobs IN-FRAME (a
    // walking juice) while the SCREEN never jolts up/down with each foot-plant.
    const baseRenderY = (physics.cube ? -physics.interpBodyCamY(alpha) : 0.9);
    const cubeRenderY = baseRenderY;
    // SMOOTH VERTICAL FOLLOW (kept): the body snaps up at each stair step (so the
    // foot never penetrates). Easing a separate _camY toward the bob-free base
    // turns the step-up into a glide so the screen never jolts.
    // HEAVY dt-based low-pass: filter the per-stride BOB/LOFT + high-freq BUMP terrain
    // wobble (the old fixed 0.10/frame ease let them leak ⇒ the screen shook with the
    // cube). tau≈0.5s follows real slopes/hills but smooths the sub-second jitter.
    // dt-based ⇒ frame-rate independent (we now render every rAF: 60 OR 120Hz).
    const camA = 1 - Math.exp(-2.0 * dt);
    if (this._camY == null || !Number.isFinite(this._camY)) this._camY = cubeRenderY;
    else this._camY += (cubeRenderY - this._camY) * camA;
    // §C: the cube rides the serpentine band at z = laneCurveZ(x). Smoothly follow
    // that z too (ease) so the camera tracks the winding without snapping at the
    // S-curve peaks. (Camera lateral follow is a render-only effect — physics z=0.)
    const curveZ = laneCurveZ(x);
    if (this._camCurveZ == null || !Number.isFinite(this._camCurveZ)) this._camCurveZ = curveZ;
    else this._camCurveZ += (curveZ - this._camCurveZ) * camA;

    const racing = Math.abs(this.rivalLaneZ) > 1e-3;
    // 3/4 chase framed like the reference: a narrow checker PATH winding across the
    // lower-middle of the frame (~1/3–1/2 of the width) with a big lime field/sky
    // around it, AND a steep enough DEPRESSION angle that the camera looks DOWN
    // onto the band's TOP face (so the bright checker — not the dark side walls —
    // is the dominant visible surface). Earlier the eye was high but it AIMED far
    // ahead at cube height, skimming the band edge-on ⇒ only the dark sides showed.
    //   eye ~7 behind, ~9 up, ~9 to the +z side.
    // CAMERA A (user pick): LOWER + more BEHIND so the two legs (z = ±LEG_Z_OFFSET)
    // read as a clear LEFT/RIGHT pair straddling the cube, instead of stacking
    // near/far and overlapping. Less +z side-offset + lower eye = the depth between
    // the legs maps to horizontal screen separation. The track top is now an unlit
    // double-sided checker (always visible), so the lower angle no longer hides it.
    // Reference framing: FAR (cube small ~1/7 screen, long winding track visible)
    // AND a moderately LOW depression (~22°) — not the steep top-down we had. Far
    // distance keeps the leg overlap a non-issue while matching the reference's
    // distant, low, over-the-track view.
    // SIDE-ON (옆면) like the reference: camera mostly to the +z SIDE with only a
    // little behind (-x), low height → we see the cube's side profile + both legs,
    // travel reads left→right, not "diagonal from behind." Far + low.
    // The band is now THIN (RIBBON_DEPTH 1.5). We pull the side camera CLOSER in z
    // (was 13.0/13.5 framed for a 2.6 band) so the narrow ribbon still reads as a clear
    // checker path filling a meaningful slice of the frame — not a far-off hairline —
    // while keeping the low, side-on, winding-track reference framing. camY is lowered a
    // touch to keep the cube + its two (now closer) legs reading as a left/right pair.
    const camX = x - 3.2;
    const camY = this._camY + (racing ? 5.0 : 4.5);
    const camZ = (racing ? 11.2 : 10.6) + this._camCurveZ;
    this.camera.position.set(camX, camY, camZ);
    const lookZ = (racing ? this.rivalLaneZ * 0.30 : 0) + this._camCurveZ;
    this.camera.lookAt(x + 1.5, this._camY + 0.2, lookZ);
  }

  /** Debug/verify info: how many meshes the track group holds (continuous ribbon
   * ⇒ a small fixed count: per lane 1 top + 1 side, plus finish poles/flags — NOT
   * one slab per segment), and a few serpentine-curve z samples (evidence §B/§C). */
  ribbonInfo(physics) {
    // count only the continuous RIBBON meshes (top+side per lane) — NOT the tunnel
    // ceilings / finish flags (those are separate obstacle/marker geometry, not the
    // band-continuity evidence the §B assertion is about).
    const meshes = this.trackGroup ? this.trackGroup.children.filter((c) => c.isMesh && c.userData.ribbon).length : 0;
    const x0 = physics ? physics.startX : 0, x1 = physics ? physics.finishX : 1;
    const samples = [];
    for (let k = 0; k <= 8; k++) {
      const x = x0 + (x1 - x0) * (k / 8);
      samples.push({ x: +x.toFixed(1), z: +laneCurveZ(x).toFixed(3) });
    }
    return { trackMeshes: meshes, rivalLaneZ: this.rivalLaneZ, curveSamples: samples };
  }

  render() { this.renderer.render(this.scene, this.camera); }

  /** Live three.js render/memory stats (draw calls, triangles, geometries, textures)
   * for the debug overlay. Returns the WebGLRenderer's `.info` object directly (read-
   * only consumption — no behaviour change). Null-safe for the headless path. */
  info() { return this.renderer ? this.renderer.info : null; }

  /** START-HITCH PRE-WARM: compile every material's shader program + upload textures
   * NOW (right after the scene is built) instead of lazily on the first render(). The
   * first frame otherwise pays a ~10ms synchronous program-link + texture-upload stall
   * (a visible entry hitch). compile() walks the scene and links all programs up front;
   * render-behaviour is unchanged (it only moves the cost off the first frame). Guarded
   * so a missing GL context (headless edge) never throws. */
  prewarm() {
    try {
      if (this.renderer && this.scene && this.camera && typeof this.renderer.compile === 'function') {
        this.renderer.compile(this.scene, this.camera);
      }
    } catch (e) { /* non-fatal: first render() will compile lazily */ }
  }

  resize(wCss, hCss) {
    this.renderer.setSize(wCss, hCss, false);
    this.camera.aspect = wCss / hCss;
    this.camera.updateProjectionMatrix();
  }

  // ── procedural textures ──
  /** TRANSVERSE STRIPE texture for the track top (replaces the old checker) — a calm,
   * clean reference look: two purple TONES alternating in bands that run ACROSS the path
   * (perpendicular to travel). The bands alternate along the texture's U axis (which we
   * map to the path's forward x in _buildRibbon), so as the cube advances it crosses one
   * band after another — a striped road, NOT a busy grid. Built ONCE (cached in the
   * constructor) and reused on every buildTrack (no per-rebuild regeneration). Small
   * (64×8 px): the stripe is 1-D along U, so width carries the bands and height is
   * trivial; NEAREST filtering keeps the band edges crisp at any distance. The colours
   * are the design-token track purples (--track-a #8E3AAE deep / --track-b #C24FD6
   * bright), staying vivid + on-palette. */
  _makeStripes() {
    const c = document.createElement('canvas');
    c.width = 64; c.height = 8;            // small (1-D stripe along U); cached once
    const ctx = c.getContext('2d');
    // STRIPE_BANDS bands across the texture width; each band is a flat purple tone. With
    // an even band count the wrap is seamless (last band of one repeat ≠ first of the next
    // only if odd, so we keep it even).
    const bandW = c.width / STRIPE_BANDS;
    const TONES = ['#8E3AAE', '#C24FD6'];  // --track-a (deep) ↔ --track-b (bright)
    for (let i = 0; i < STRIPE_BANDS; i++) {
      ctx.fillStyle = TONES[i % 2];
      ctx.fillRect(Math.round(i * bandW), 0, Math.ceil(bandW), c.height);
    }
    const tex = new THREE.CanvasTexture(c);
    tex.colorSpace = THREE.SRGBColorSpace;
    tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
    tex.magFilter = THREE.NearestFilter;   // crisp band edges (no muddy blur)
    tex.minFilter = THREE.NearestFilter;   // and crisp at distance (avoid mip-averaging to a blob)
    tex.generateMipmaps = false;
    return tex;
  }

  _makeChecker(n = 8) {
    const c = document.createElement('canvas');
    c.width = c.height = 128;
    const ctx = c.getContext('2d');
    const s = 128 / n;
    // VIVID purple check with strong light/dark contrast so the band reads as a
    // crisp checkered PATH (reference look). The top material is unlit
    // (MeshBasicMaterial) so these colours show at full brightness regardless of
    // the grazing 3/4 light — the check never dies into a solid blob.
    //   bright lilac  #D98CF0  ↔  deep magenta  #8A2BB0
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
      ctx.fillStyle = ((i + j) % 2 === 0) ? '#8A2BB0' : '#D98CF0';
      ctx.fillRect(i * s, j * s, s, s);
    }
    const tex = new THREE.CanvasTexture(c);
    tex.colorSpace = THREE.SRGBColorSpace;
    tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
    tex.magFilter = THREE.NearestFilter; // keep crisp square cells (no muddy blur)
    return tex;
  }

  _makeFace(faceColor) {
    const hex = '#' + ((faceColor == null ? COL.playerFace : faceColor) >>> 0).toString(16).padStart(6, '0');
    const c = document.createElement('canvas');
    c.width = c.height = 128;
    const ctx = c.getContext('2d');
    ctx.clearRect(0, 0, 128, 128);
    ctx.fillStyle = hex;
    // Face sits in the UPPER half so the axle hub (rendered at the cube centre,
    // where both legs pivot) sits BELOW the smile without covering the eyes.
    // two dot eyes (raised)
    ctx.beginPath(); ctx.arc(46, 38, 10, 0, Math.PI * 2); ctx.fill();
    ctx.beginPath(); ctx.arc(82, 38, 10, 0, Math.PI * 2); ctx.fill();
    // smile (raised, above the hub)
    ctx.lineWidth = 6; ctx.strokeStyle = hex; ctx.lineCap = 'round';
    ctx.beginPath(); ctx.arc(64, 54, 16, 0.15 * Math.PI, 0.85 * Math.PI); ctx.stroke();
    const tex = new THREE.CanvasTexture(c);
    tex.colorSpace = THREE.SRGBColorSpace;
    return tex;
  }

  _disposeMesh(m) {
    if (!m) return;
    // a leg may be a Group of box meshes (compound limb) or a single mesh. Geometry is
    // per-build (must be freed); materials are the SHARED _legMat (kept). Route through
    // _disposeGroup so geometry + any non-shared material/texture are handled uniformly.
    if (m.isGroup || (m.children && m.children.length)) { this._disposeGroup(m); return; }
    if (m.geometry) m.geometry.dispose();
    if (Array.isArray(m.material)) m.material.forEach((mm) => this._disposeMaterial(mm));
    else if (m.material) this._disposeMaterial(m.material);
  }

  /** True for the SHARED materials that live for the renderer's lifetime (built once
   * in the constructor, reused on every rebuild). They must NEVER be disposed when a
   * track/leg group is torn down, or the next build would render with a dead material. */
  _isSharedMat(m) {
    return m === this._legMat || m === this._topMat || m === this._sideMat || m === this._ballMat;
  }

  /** True for the SHARED textures (built once in the constructor, reused forever). The
   * track stripe texture is the only long-lived texture; the per-build face + finish-flag
   * CanvasTextures are throwaway and MUST be disposed on rebuild (leak fix). */
  _isSharedTex(t) {
    return t === this._stripeTex;
  }

  /** Dispose a material AND its (non-shared) textures. `material.dispose()` does NOT
   * free the GPU texture referenced by `.map` — that is what leaked the per-cube face
   * + finish-flag CanvasTextures across rebuilds. Shared materials/textures are kept. */
  _disposeMaterial(m) {
    if (!m || this._isSharedMat(m)) return;
    // free any texture maps the material holds (these are the leaked CanvasTextures).
    for (const k of ['map', 'normalMap', 'roughnessMap', 'metalnessMap', 'emissiveMap', 'alphaMap', 'aoMap', 'bumpMap']) {
      const t = m[k];
      if (t && t.isTexture && !this._isSharedTex(t) && t.dispose) t.dispose();
    }
    if (m.dispose) m.dispose();
  }

  _disposeGroup(g) {
    g.traverse((o) => {
      if (o.isMesh) {
        if (o.geometry) o.geometry.dispose();
        if (Array.isArray(o.material)) o.material.forEach((m) => this._disposeMaterial(m));
        else if (o.material) this._disposeMaterial(o.material);
      }
    });
  }
}
