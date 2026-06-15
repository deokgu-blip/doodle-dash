// engine/renderer.js
// Three.js 3D render of the 2D physics plane (POC §4/§8).
//   - cube (dot-eye face) + extruded legs synced to Matter bodies each frame
//   - purple/magenta checkerboard track ribbon extruded from segments
//   - lime-green gradient background + 3/4 chase camera
// Colors use the POC §8 design tokens (placeholder materials; image assets later).

import * as THREE from './vendor/three.module.js';
import { PHYS_CONST } from './physics.js';
import { Path, makeHeadingFn, serpZ } from './path.js';

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

// ── HEADING-BASED PATH (the track GENUINELY turns in 3D, §C + bends) ─────────
// The track centre-line is a HEADING (yaw) PATH (see engine/path.js): the forward
// coordinate x is arc-length, and pathHeading(x) is the tangent yaw. The world position
// of a point at forward x, lateral L, height y is
//   Le = L + serpZ(x);  worldX = pathX(x) + Le·(-sin h),  worldZ = pathZ(x) + Le·(cos h),  worldY = -y
// (h = heading). The HEADING is built from TURNS ONLY — the gentle serpentine is a COSMETIC
// LATERAL offset (serpZ added to L in the transform), NOT a heading. So a real bend (a
// data-driven `turn`/`curve` region) yaws the cube + swings the camera, but the cosmetic
// serpentine sway does NEITHER: on a straight heading≡0 ⇒ worldZ = L + serpZ(x), the cube is
// UPRIGHT and the camera is SIDE-ON. (Folding serpZ into the heading was the "diagonal course
// / self-rotating cube" regression — see path.js.) Physics x / progress / win are untouched.
//
// COMPATIBILITY: `laneCurveZ(x)` is kept as a thin wrapper returning the lane centre's
// lateral world-z deflection. When a Path exists it delegates to path.laneCurveZ (= pathZ +
// serpZ projected); with NO Path (defensive / pre-build) it falls back to the OLD closed-form
// `CURVE_AMP·sin(CURVE_FREQ·x)`. With heading ≡ 0 the transform reduces to (x, -y, L + serpZ(x))
// — BYTE-IDENTICAL to the old (x, -y, laneZ + laneCurveZ(x)) placement (regression guard).
const CURVE_AMP = 2.5;        // legacy serpentine peak z deflection (matches path.SERP_AMP)
const CURVE_FREQ = 0.12;      // legacy serpentine spatial frequency (matches path.SERP_FREQ)
// Module-level "active path" the compat wrapper reads. Set by Renderer.buildTrack; the
// renderer always prefers `this.path` directly, this is only for laneCurveZ() callers
// outside a Renderer instance and for the export's back-compat.
let _activePath = null;
export function laneCurveZ(x) {
  return _activePath ? _activePath.laneCurveZ(x) : CURVE_AMP * Math.sin(x * CURVE_FREQ);
}

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

    // HEADING-BASED PATH: the curving centre-line (built per buildTrack from the track's
    // turn regions + the folded serpentine). All mesh placement + the camera go through
    // this.path.transform(...). Null until the first buildTrack (laneCurveZ falls back).
    this.path = null;
    // Alloc-free scratch objects for the per-frame transform (no per-frame GC).
    this._tp = { x: 0, y: 0, z: 0 };
    this._tpL = { x: 0, y: 0, z: 0 };
    this._tpR = { x: 0, y: 0, z: 0 };

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
    // ── ONE ROAD MATERIAL (ONE draw call per welded road mesh) ──────────────────────────
    // The whole road (stripe TOP + edge-coloured SIDES/risers/bottom/caps) renders from a
    // SINGLE unlit material now, so each welded lane mesh is ONE draw call instead of two
    // (the old top-strip + side-strip were two materials = two draws). It uses the two-zone
    // stripe texture (_makeStripes) + PER-VERTEX colour: top verts sample the STRIPE zone
    // with WHITE vertex colour (stripe shows at full), side/riser/bottom/cap verts sample the
    // WHITE swatch zone with the EDGE colour as vertex colour (white×edge = solid edge tint).
    // UNLIT (MeshBasicMaterial): the road ignores scene lighting (flat vivid colour at any
    // angle). DoubleSide so faces show regardless of winding (and the ㅁ end caps stay solid).
    this._roadMat = new THREE.MeshBasicMaterial({ map: this._stripeTex, vertexColors: true, side: THREE.DoubleSide });
    // edge colour as a normalised RGB triple reused for every side vertex colour (no alloc in
    // the hot build loop). trackEdge = 0x5E2480.
    this._edgeRGB = [((COL.trackEdge >> 16) & 255) / 255, ((COL.trackEdge >> 8) & 255) / 255, (COL.trackEdge & 255) / 255];
    // LEGACY top/side materials kept for any non-welded caller (none after this change). The
    // welded builders use _roadMat. (_topMat retained so older capture scripts referencing it
    // don't crash; both are protected as shared in _isSharedMat.)
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

    // ── BREAKING-BLOCK shared geometry + materials (built ONCE, reused for every block). ──
    // ONE unit BoxGeometry (scaled per block / per debris chip) shared by every standing
    // block AND every debris fragment, plus two Lambert materials: a slate-grey for the
    // intact STANDING blocks (a clear obstacle that contrasts the lime field + purple track,
    // distinct from the blue cube / amber balls) and a slightly darker grey for the broken
    // DEBRIS chips (so the rubble reads as "the same stone, smashed"). Lambert so the boxes
    // catch the key light and read 3D (top face brighter). Shared ⇒ never disposed on rebuild.
    this._blockGeo = new THREE.BoxGeometry(1, 1, 1); // unit box, scaled per block / per chip
    this._blockMat = new THREE.MeshLambertMaterial({ color: 0x9AA0A8 }); // slate-grey standing block
    this._debrisMat = new THREE.MeshLambertMaterial({ color: 0x7C828B }); // darker grey rubble chip
    // BREAKING-BLOCK meshes (built in buildTrack; positions/visibility synced each frame).
    this.blockMeshes = [];        // player lane standing blocks
    this.debrisMeshes = [];       // player lane debris chips
    this.rivalBlockMeshes = [];   // rival lane standing blocks
    this.rivalDebrisMeshes = [];  // rival lane debris chips
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
    // BREAKING-BLOCK: drop old block + debris meshes (shared geo/material kept — _isSharedMat).
    for (const m of this.blockMeshes) this.scene.remove(m);
    for (const m of this.debrisMeshes) this.scene.remove(m);
    for (const m of this.rivalBlockMeshes) this.scene.remove(m);
    for (const m of this.rivalDebrisMeshes) this.scene.remove(m);
    this.blockMeshes = []; this.debrisMeshes = [];
    this.rivalBlockMeshes = []; this.rivalDebrisMeshes = [];
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

    // ── HEADING-BASED PATH (built ONCE here; O(1) alloc-free lookups thereafter) ──
    // The centre-line yaw = the folded gentle serpentine + the track's data-driven turn
    // regions (physics.turnRegions: each segment's [x0,x1] arc-length span + turnRad). We
    // sample a FINE LUT over [start−margin, finish+margin] so bends + the cube's pre-roll/
    // overshoot are covered. The transform reduces to (x,-y,L) at heading 0 (regression).
    {
      const x0 = (physics ? physics.startX : track.startX) - 8;     // lead-in margin (cube starts at startX-3)
      const x1 = (physics ? physics.finishX : track.finishX) + 8;   // run-out margin
      const turns = (physics && physics.turnRegions) ? physics.turnRegions : [];
      // HEADING = TURNS ONLY (the serpentine is a COSMETIC lateral offset, never a heading).
      // ⇒ path.heading(x) == 0 everywhere except inside a real curve/turn region, so on
      // straights the cube stays UPRIGHT and the camera stays SIDE-ON (no self-rotation, no
      // diagonal course). The gentle serpentine is re-introduced as serpZFn below.
      const headingFn = makeHeadingFn(turns, /*serpentine*/ false);
      // ANCHOR at startX to the OLD world placement: pathX(startX)=startX, pathZ(startX)=0
      // (the turn-only centre-line carries no serpentine — that rides as the lateral offset),
      // so the straight lead-in sits EXACTLY where it did before. LUT step 0.25u — finer than
      // the ribbon sampling (RIBBON_DX 0.5) so the centre-line stays smooth through bends.
      const startX = physics ? physics.startX : track.startX;
      this.path = new Path(x0, x1, 0.25, headingFn, {
        anchorX: startX,
        anchorZ: 0,
        serpZFn: serpZ,   // cosmetic lateral sway (= old laneCurveZ); does NOT yaw/swing
      });
      _activePath = this.path;          // back-compat: the exported laneCurveZ() reads this
    }

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

    // SPLIT-PATH FORK — the HIGH ARCH (a hook-gated steep staircase up → flat top → staircase
    // down) rendered as its OWN ribbon ABOVE the lane. The LOW underpass route is part of the
    // floor ribbon (the dip valley is in physics._segs, so _surfaceProfile already drew it). We
    // draw BOTH so the fork visibly SPLITS at the mouth and MERGES at the rejoin in side view.
    // The non-committed road is purely visual (physics is the single committed height function),
    // so a LOW-route cube simply passes UNDER this high arch with no collision. One per lane.
    this._buildForks(physics, 0, topMat, sideMat);
    if (rivalSpec) this._buildForks(physics, this.rivalLaneZ, topMat, sideMat);
    this.scene.add(this.trackGroup);

    // ── player cube ──
    this.cubeMesh = this._buildCharacterCube(COL.player, COL.playerFace);
    this.scene.add(this.cubeMesh);

    // ── rival cube (our own opponent — distinct colour, on the parallel lane) ──
    if (rivalSpec) {
      this.rivalCubeMesh = this._buildCharacterCube(COL.rival, COL.rivalFace);
      // placeholder pose on the parallel lane at startX (overwritten each frame in syncRival).
      const rp = this.path.transform(physics ? physics.startX : 0, this.rivalLaneZ, 0, this._tp);
      this.rivalCubeMesh.position.set(rp.x, rp.y, rp.z);
      this.scene.add(this.rivalCubeMesh);
    } else {
      this.rivalCubeMesh = null;
    }

    // finish flag (simple marker at finishX) — spans both lanes when racing.
    this._buildFinish(track, rivalSpec, physics);

    // ── BALL-FIELD: build one reused sphere mesh per physics ball (player lane). The
    //    rival pile (its own physics field) is built when syncRival first sees it. ──
    this._buildBalls(physics, 0);

    // ── BREAKING-BLOCK: build one reused box mesh per standing block + per debris chip
    //    (player lane). The rival wall (its own physics field) is built lazily in syncRival. ──
    this._buildBlocks(physics, 0);
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
      const p = this.path.transform(b.x, laneZ, b.y, this._tp);
      mesh.position.set(p.x, p.y, p.z);
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
      const p = this.path.transform(b.x, laneZ, b.y, this._tp);
      meshes[i].position.set(p.x, p.y, p.z);
    }
  }

  /** Build the reusable box meshes for a walker's breaking-block wall + debris on the given
   * lane. One Mesh per standing block (sized to the block, sitting upright on the surface)
   * sharing the constructor's _blockGeo + _blockMat, and one small Mesh per debris fragment
   * sharing _blockGeo + _debrisMat. Geometry/material are shared ⇒ only the Mesh wrappers
   * are made here. Standing blocks are placed ONCE (they are static until broken — then
   * hidden); debris is placed (and shown/hidden) every frame in _syncBlocks. */
  _buildBlocks(physics, laneZ) {
    const isRival = Math.abs(laneZ) > 1e-3;
    const blkRef = isRival ? 'rivalBlockMeshes' : 'blockMeshes';
    const debRef = isRival ? 'rivalDebrisMeshes' : 'debrisMeshes';
    for (const m of this[blkRef]) this.scene.remove(m);
    for (const m of this[debRef]) this.scene.remove(m);
    this[blkRef] = []; this[debRef] = [];
    const blocks = physics.blocks;
    if (blocks && blocks.length) {
      for (let i = 0; i < blocks.length; i++) {
        const b = blocks[i];
        const mesh = new THREE.Mesh(this._blockGeo, this._blockMat);
        // a CLOSED box sized to the block; slightly under the path width in z so it reads on
        // the band. Centre y = mid-height (render y = -((topY+baseY)/2)); top = -topY.
        mesh.scale.set(b.w, b.h, Math.min(RIBBON_DEPTH * 0.9, b.w * 1.4));
        const cyPhys = (b.topY + b.baseY) / 2;   // physics-y of the block centre (+down)
        const p = this.path.transform(b.x, laneZ, cyPhys, this._tp);
        mesh.position.set(p.x, p.y, p.z);
        mesh.rotation.y = this.path.heading(b.x);  // box faces along the (turning) path
        mesh.visible = !b.broken;
        this.scene.add(mesh);
        this[blkRef].push(mesh);
      }
    }
    const debris = physics.debris;
    if (debris && debris.length) {
      for (let i = 0; i < debris.length; i++) {
        const d = debris[i];
        const mesh = new THREE.Mesh(this._blockGeo, this._debrisMat);
        const s = d.r * 2; // chip edge == 2× the half-size collision radius
        mesh.scale.set(s, s, s);
        const p = this.path.transform(d.x, laneZ, d.y, this._tp);
        mesh.position.set(p.x, p.y, p.z);
        mesh.visible = !!d.active;
        this.scene.add(mesh);
        this[debRef].push(mesh);
      }
    }
  }

  /** Update the standing blocks (hide a broken one) + debris chips (position + show once its
   * block has burst) from the live physics each frame. Position/visibility only — no geometry,
   * no allocation. The pile is small (blocks<=~6, debris<=24) ⇒ trivially cheap. A spinning
   * tumble is faked by deriving a render rotation from the chip's position (cheap, no state). */
  _syncBlocks(physics, laneZ, blockMeshes, debrisMeshes) {
    const blocks = physics.blocks;
    if (blocks && blockMeshes.length) {
      const n = Math.min(blocks.length, blockMeshes.length);
      for (let i = 0; i < n; i++) blockMeshes[i].visible = !blocks[i].broken;
    }
    const debris = physics.debris;
    if (debris && debrisMeshes.length) {
      const n = Math.min(debris.length, debrisMeshes.length);
      for (let i = 0; i < n; i++) {
        const d = debris[i], m = debrisMeshes[i];
        m.visible = !!d.active;
        if (!d.active) continue;
        const p = this.path.transform(d.x, laneZ, d.y, this._tp);
        m.position.set(p.x, p.y, p.z);
        // cheap tumble: rotate by a function of position so flying chips look like they spin
        // (no per-chip angular state — purely cosmetic, deterministic).
        m.rotation.z = d.x * 1.7 + i;
        m.rotation.x = d.y * 1.3;
      }
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
    const yHints = [];            // exact profile y for a profile point; null for a densified point
    const breakAfter = [];
    for (let i = 0; i < thinned.length - 1; i++) {
      const a = thinned[i], b = thinned[i + 1];
      xs.push(a.x); yHints.push(a.y); breakAfter.push(false);
      const span = b.x - a.x;
      const mid = (a.x + b.x) / 2;
      // a void (plank gap) span: mark a break, do NOT densify or bridge across it.
      if (isVoidSpan(a.x, b.x)) { breakAfter[breakAfter.length - 1] = true; continue; }
      // skip re-densification when this span lies inside a bumps window (keep it coarse).
      if (span > RIBBON_DX * 1.5 && !inBumps(mid)) {
        const n = Math.floor(span / RIBBON_DX);
        for (let k = 1; k < n; k++) {
          const t = k / n;
          xs.push(a.x + span * t); yHints.push(null); breakAfter.push(false);
        }
      }
    }
    {
      const last = thinned[thinned.length - 1];
      xs.push(last.x); yHints.push(last.y); breakAfter.push(false);
    }
    // surfaceY at a ring: PREFER the exact PROFILE y (yHint) when this ring is a profile point.
    // This is essential for STAIR RISERS — a riser is two profile points at the SAME x with the
    // lower-tread y and the upper-tread y; re-sampling physics.surfaceYAt(x) returns only the
    // HIGHEST surface at that x, collapsing the two into one ⇒ the riser would smear into a smooth
    // diagonal (no visible step edges). Using the stored profile y keeps each tread's own height,
    // so the riser stays a SHARP vertical step (the grip-point edges). Densified points (flats /
    // ramps / thinned bumps) carry yHint=null ⇒ they sample the true physics surface as before.
    let lastY = prof[0].y;
    const surfY = (x, yHint) => {
      if (yHint != null) { lastY = yHint; return yHint; }
      const y = physics.surfaceYAt(x);
      if (y != null) { lastY = y; return y; }
      return lastY;
    };
    const half = RIBBON_DEPTH / 2;

    // ── ONE WELDED ROAD MESH per lane, ONE material, ONE DRAW CALL (no z-fighting) ──────
    // The OLD builder made TWO separate strips (top strip + side strip). At every STAIR
    // RISER the two same-x rings made the TOP strip emit a vertical riser quad (plane
    // x=const) AND the SIDE strip emit its near-wall + far-wall + bottom quads — which,
    // because BOTH rings share that x, ALSO collapse into the SAME plane x=const. So 3 side
    // faces + 1 top face stacked coplanar at every riser ⇒ Z-FIGHTING ⇒ the "자글자글"
    // flickering seams on the moving phone (root cause confirmed: 15 coplanar [1,0,0] face
    // pairs across the two strips on a 5-step staircase). And being two meshes/materials it
    // was two draw calls per lane.
    //
    // THE FIX: build the whole road — tread tops, ramp/flat tops, the vertical RISER fronts,
    // the side walls, the bottom, the ㅁ end caps — as ONE welded BufferGeometry rendered by
    // ONE material (_roadMat). At a riser we emit the step front ONCE and SKIP the degenerate
    // zero-x-length side walls/bottom that used to collapse onto it ⇒ NO overlapping faces.
    // The look (purple stripe TOP, edge-coloured SIDES) comes from PER-VERTEX COLOUR + the
    // two-zone stripe texture: TOP-surface verts are WHITE + sample the stripe zone (stripe
    // shows full); SIDE verts are the EDGE colour + sample the white swatch (white×edge =
    // solid edge). nearTop/farTop are DUPLICATED (a white top copy + an edge side copy) so a
    // shared vertex never has to be both colours. One material ⇒ ONE draw call per lane.
    const pos = [], uv = [], col = [];
    const idxA = [];                       // single index buffer (one material, one draw)
    const uScale = RIBBON_USCALE;
    const TOP_V = 0.25, SIDE_V = 0.75;     // stripe zone (V<0.5) vs white swatch zone (V>=0.5)
    const E = this._edgeRGB;               // [r,g,b] edge colour for side verts
    const N = xs.length;
    // Per ring: 6 verts — 0,1 = TOP-surface copies (nearTop, farTop): WHITE, stripe UV (used
    // by the TOP face); 2,3,4,5 = SIDE copies (nearTop, nearBot, farTop, farBot): EDGE colour,
    // white-swatch UV (used by walls/bottom/risers/caps). Risers reference the SIDE copies so
    // the step front is a solid edge-coloured face (the reference look).
    const rb = (i) => i * 6;
    for (let i = 0; i < N; i++) {
      const x = xs[i];
      const sy = surfY(x, yHints[i]);       // physics surface y (+down) — exact profile y at risers
      const syB = sy + RIBBON_DOWN;         // slab bottom (dropped) in physics y
      // PATH TRANSFORM: band edges PERPENDICULAR to the heading at L = laneZ ± half (turns
      // with the path, no z-shear). At heading 0 ⇒ (x, -sy, laneZ∓half) — old placement.
      const Nt = this.path.transform(x, laneZ - half, sy, this._tpL);   // near top
      const Ft = this.path.transform(x, laneZ + half, sy, this._tpR);   // far top
      const Nb = this.path.transform(x, laneZ - half, syB, this._tpL);  // near bottom
      const Fb = this.path.transform(x, laneZ + half, syB, this._tpR);  // far bottom
      const u = x * uScale;
      // TOP-surface copies (white, stripe zone). V is constant in the stripe zone (stripes
      // vary only along U), so both rails sit at TOP_V.
      pos.push(Nt.x, Nt.y, Nt.z,  Ft.x, Ft.y, Ft.z);
      uv.push(u, TOP_V,  u, TOP_V);
      col.push(1, 1, 1,  1, 1, 1);
      // SIDE copies (edge colour, white-swatch zone): nearTop, nearBot, farTop, farBot.
      pos.push(Nt.x, Nt.y, Nt.z,  Nb.x, Nb.y, Nb.z,  Ft.x, Ft.y, Ft.z,  Fb.x, Fb.y, Fb.z);
      uv.push(u, SIDE_V,  u, SIDE_V,  u, SIDE_V,  u, SIDE_V);
      col.push(E[0], E[1], E[2],  E[0], E[1], E[2],  E[0], E[1], E[2],  E[0], E[1], E[2]);

      const breakBefore = i > 0 && breakAfter[i - 1];
      if (i > 0 && !breakBefore) {
        const p = rb(i - 1), q = rb(i);
        // SIDE-copy offsets within a ring: +2 nearTop, +3 nearBot, +4 farTop, +5 farBot.
        const isRiser = Math.abs(xs[i] - xs[i - 1]) < 1e-6;
        if (isRiser) {
          // RISER: the two rings share x ⇒ a vertical step FRONT. Emit it ONCE (edge colour),
          // SKIP the degenerate near/far/bottom quads that would collapse onto this plane and
          // z-fight (the old bug). Near/far walls stay continuous via the shared SIDE verts.
          idxA.push(p + 2, p + 4, q + 2,  q + 2, p + 4, q + 4);   // step front (edge colour)
        } else {
          // FLAT / RAMP / TREAD-TOP span: TOP face (white→stripe) + near/far walls + bottom
          // (edge colour). Distinct planes — no coplanar duplication.
          idxA.push(p + 0, q + 0, p + 1,  p + 1, q + 0, q + 1);   // TOP face (top copies 0,1)
          idxA.push(p + 2, p + 3, q + 2,  q + 2, p + 3, q + 3);   // near wall (-z)
          idxA.push(p + 4, q + 4, p + 5,  p + 5, q + 4, q + 5);   // far wall (+z)
          idxA.push(p + 3, p + 5, q + 3,  q + 3, p + 5, q + 5);   // bottom (-y)
        }
      }
      // ── END CAPS (ㅁ closure): cross-section quad at the lane ends + each side of a PLANKS
      //    void so every box is fully closed. Double-wound ⇒ solid from either face. Uses the
      //    SIDE copies (edge colour) so the cap is the same edge tone as the walls. ──
      const capHere = (i === 0 || i === N - 1) || breakBefore || breakAfter[i];
      if (capHere) {
        const b = rb(i);
        idxA.push(b + 2, b + 4, b + 5,  b + 2, b + 5, b + 3);
        idxA.push(b + 2, b + 5, b + 4,  b + 2, b + 3, b + 5);
      }
    }
    // ONE geometry, ONE mesh, ONE material ⇒ ONE draw call per lane. Welded — top + sides +
    // risers share one vertex buffer with NO duplicate coplanar faces ⇒ no z-fighting.
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(pos), 3));
    geo.setAttribute('uv', new THREE.BufferAttribute(new Float32Array(uv), 2));
    geo.setAttribute('color', new THREE.BufferAttribute(new Float32Array(col), 3));
    geo.setIndex(idxA);
    geo.computeVertexNormals();
    const mesh = new THREE.Mesh(geo, this._roadMat);
    mesh.userData.ribbon = true;
    this.trackGroup.add(mesh);
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

      // The board's UNDERSIDE sits at the head-room line (physics y = ceilingY); its
      // stripe TOP is RIBBON_DOWN above that (physics y = ceilingY − RIBBON_DOWN).
      const underPy = c.ceilingY;            // underside physics y (the surface a too-long leg hits)
      const topPy = c.ceilingY - RIBBON_DOWN; // bright stripe top of the overhead board (physics y)

      // ONE welded overhead board per tunnel, ONE material, ONE draw call (top stripes +
      // walls + underside). No risers (it's a flat board). Same per-vertex-colour scheme as
      // the floor ribbon: TOP-surface verts WHITE + stripe UV, SIDE verts EDGE colour + white
      // swatch UV. 6 verts/ring (2 top copies + 4 side copies) so a vert is never two colours.
      const pos = [], uv = [], col = [];
      const idxA = [];
      const E = this._edgeRGB, TOP_V = 0.25, SIDE_V = 0.75;
      const N = xs.length;
      const rb = (i) => i * 6;
      for (let i = 0; i < N; i++) {
        const x = xs[i];
        // PATH TRANSFORM: same lane centre + heading as the floor, edges perpendicular.
        const Nt = this.path.transform(x, laneZ - half, topPy, this._tpL);
        const Ft = this.path.transform(x, laneZ + half, topPy, this._tpR);
        const Nu = this.path.transform(x, laneZ - half, underPy, this._tpL);  // near underside
        const Fu = this.path.transform(x, laneZ + half, underPy, this._tpR);  // far underside
        const u = x * uScale;
        pos.push(Nt.x, Nt.y, Nt.z,  Ft.x, Ft.y, Ft.z);                        // TOP copies (white)
        uv.push(u, TOP_V,  u, TOP_V); col.push(1, 1, 1,  1, 1, 1);
        pos.push(Nt.x, Nt.y, Nt.z,  Nu.x, Nu.y, Nu.z,  Ft.x, Ft.y, Ft.z,  Fu.x, Fu.y, Fu.z); // SIDE copies (edge)
        uv.push(u, SIDE_V,  u, SIDE_V,  u, SIDE_V,  u, SIDE_V);
        col.push(E[0], E[1], E[2],  E[0], E[1], E[2],  E[0], E[1], E[2],  E[0], E[1], E[2]);
        if (i > 0) {
          const p = rb(i - 1), q = rb(i);
          idxA.push(p + 0, q + 0, p + 1,  p + 1, q + 0, q + 1);   // TOP (stripes; copies 0,1)
          idxA.push(p + 2, p + 3, q + 2,  q + 2, p + 3, q + 3);   // near wall
          idxA.push(p + 4, q + 4, p + 5,  p + 5, q + 4, q + 5);   // far wall
          idxA.push(p + 3, p + 5, q + 3,  q + 3, p + 5, q + 5);   // UNDERSIDE (head-room)
        }
        if (i === 0 || i === N - 1) {              // END CAPS (ㅁ closure)
          const b = rb(i);
          idxA.push(b + 2, b + 4, b + 5,  b + 2, b + 5, b + 3);
          idxA.push(b + 2, b + 5, b + 4,  b + 2, b + 3, b + 5);
        }
      }
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(pos), 3));
      geo.setAttribute('uv', new THREE.BufferAttribute(new Float32Array(uv), 2));
      geo.setAttribute('color', new THREE.BufferAttribute(new Float32Array(col), 3));
      geo.setIndex(idxA);
      geo.computeVertexNormals();
      const mesh = new THREE.Mesh(geo, this._roadMat);
      mesh.userData.ribbon = true;
      this.trackGroup.add(mesh);
    }
  }

  /** Build the SPLIT-PATH FORK's HIGH ARCH as its own track ribbon, ALWAYS (both routes are
   * always shown so the fork reads as a split that rejoins). Each fork's HIGH route is a
   * hook-gated steep staircase up → flat top → staircase down, given as physics seg tops
   * (forks[i].highSegs). We trace that profile into one ribbon (SAME purple stripe top, SAME
   * narrow width, SAME thin slab as the floor track), with the staircase RISERS drawn as sharp
   * vertical steps (visible step edges = the grip points), riding the same lane centre +
   * serpentine curve so the arch sits DIRECTLY over the lane. The LOW underpass route is part of
   * the floor ribbon (the dip valley is in physics._segs). The arch is purely VISUAL — physics is
   * the single committed height function, so a LOW-route cube passes UNDER it with no collision.
   * Static geometry built ONCE (no per-frame cost). Models on _buildRibbon's strip builder. */
  _buildForks(physics, laneZ, topMat, sideMat) {
    const forks = physics.forks;
    if (!forks || !forks.length) return;
    const half = RIBBON_DEPTH / 2;
    const uScale = RIBBON_USCALE, vRepeat = 1;     // SAME stripe mapping as the floor ribbon
    for (const f of forks) {
      // Build the HIGH route's surface PROFILE as [{x,y}] (physics y +down), emitting each
      // seg's two endpoints so the staircase risers become sharp vertical steps (a riser is two
      // samples at the same x). Sorted by x (the high segs are already in x order).
      const prof = [];
      const push = (x, y) => {
        const last = prof[prof.length - 1];
        if (last && Math.abs(last.x - x) < 1e-6 && Math.abs(last.y - y) < 1e-6) return;
        prof.push({ x, y });
      };
      for (const s of f.highSegs) { push(s.x0, s.topYa); push(s.x1, s.topYb); }
      if (prof.length < 2) continue;
      // densify long flat/ramp spans so the arch snakes with the lane (keep the exact profile
      // points — incl. risers — so the steps stay sharp). Same RIBBON_DX as the floor.
      const xs = [];
      for (let i = 0; i < prof.length - 1; i++) {
        const a = prof[i], b = prof[i + 1];
        xs.push(a.x);
        const span = b.x - a.x;
        if (span > RIBBON_DX * 1.5) {
          const n = Math.floor(span / RIBBON_DX);
          for (let k = 1; k < n; k++) xs.push(a.x + span * (k / n));
        }
      }
      xs.push(prof[prof.length - 1].x);
      // surfaceY along the HIGH route at an arbitrary x: linear-interp the profile (the high
      // route is not in physics.surfaceYAt unless committed, so sample the profile directly).
      let pi = 0;
      const profY = (x) => {
        while (pi < prof.length - 2 && prof[pi + 1].x < x) pi++;
        while (pi > 0 && prof[pi].x > x) pi--;
        const a = prof[pi], b = prof[pi + 1] || a;
        const dx = b.x - a.x;
        return dx > 1e-9 ? a.y + (b.y - a.y) * ((x - a.x) / dx) : a.y;
      };
      // ONE welded arch mesh per fork, ONE material, ONE draw call (top stripes + walls +
      // bottom + the staircase RISER fronts). Same per-vertex-colour scheme as the floor
      // ribbon: TOP-surface verts WHITE + stripe UV, SIDE verts EDGE colour + white swatch
      // UV; 6 verts/ring (2 top copies + 4 side copies). Risers emit ONE edge-coloured step
      // front and SKIP the degenerate side walls (no coplanar z-fight on the arch steps).
      const pos = [], uv = [], col = [];
      const idxA = [];
      const E = this._edgeRGB, TOP_V = 0.25, SIDE_V = 0.75;
      const N = xs.length;
      const rb = (i) => i * 6;
      for (let i = 0; i < N; i++) {
        const x = xs[i];
        const sy = profY(x);                       // arch surface physics y (+down)
        const syB = sy + RIBBON_DOWN;              // slab bottom (dropped) physics y
        // LATERAL z-SPLIT: the high arch veers off the lane centre at the still-low feet (so it
        // clears a low-route cube in z where it cannot clear it in y), and eases back over the
        // centre on the high flat top. The high-committed cube rides this SAME offset (renderer
        // sync.forkLateralAt) ⇒ WYSIWYG. 0 outside the fork ⇒ identical to before for the rest.
        const flat = physics.forkHighLatAt ? physics.forkHighLatAt(x) : 0;
        // PATH TRANSFORM: same lane centre + heading as the floor (+ the lateral split), edges perpendicular.
        const Nt = this.path.transform(x, laneZ + flat - half, sy, this._tpL);
        const Ft = this.path.transform(x, laneZ + flat + half, sy, this._tpR);
        const Nb = this.path.transform(x, laneZ + flat - half, syB, this._tpL);
        const Fb = this.path.transform(x, laneZ + flat + half, syB, this._tpR);
        const u = x * uScale;
        pos.push(Nt.x, Nt.y, Nt.z,  Ft.x, Ft.y, Ft.z);                        // TOP copies (white)
        uv.push(u, TOP_V,  u, TOP_V); col.push(1, 1, 1,  1, 1, 1);
        pos.push(Nt.x, Nt.y, Nt.z,  Nb.x, Nb.y, Nb.z,  Ft.x, Ft.y, Ft.z,  Fb.x, Fb.y, Fb.z); // SIDE copies (edge)
        uv.push(u, SIDE_V,  u, SIDE_V,  u, SIDE_V,  u, SIDE_V);
        col.push(E[0], E[1], E[2],  E[0], E[1], E[2],  E[0], E[1], E[2],  E[0], E[1], E[2]);
        if (i > 0) {
          const p = rb(i - 1), q = rb(i);
          const isRiser = Math.abs(xs[i] - xs[i - 1]) < 1e-6;
          if (isRiser) {
            idxA.push(p + 2, p + 4, q + 2,  q + 2, p + 4, q + 4);   // step front (edge colour)
          } else {
            idxA.push(p + 0, q + 0, p + 1,  p + 1, q + 0, q + 1);   // TOP (stripes; copies 0,1)
            idxA.push(p + 2, p + 3, q + 2,  q + 2, p + 3, q + 3);   // near wall
            idxA.push(p + 4, q + 4, p + 5,  p + 5, q + 4, q + 5);   // far wall
            idxA.push(p + 3, p + 5, q + 3,  q + 3, p + 5, q + 5);   // bottom
          }
        }
        if (i === 0 || i === N - 1) {              // END CAPS (ㅁ closure)
          const b = rb(i);
          idxA.push(b + 2, b + 4, b + 5,  b + 2, b + 5, b + 3);
          idxA.push(b + 2, b + 5, b + 4,  b + 2, b + 3, b + 5);
        }
      }
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(pos), 3));
      geo.setAttribute('uv', new THREE.BufferAttribute(new Float32Array(uv), 2));
      geo.setAttribute('color', new THREE.BufferAttribute(new Float32Array(col), 3));
      geo.setIndex(idxA);
      geo.computeVertexNormals();
      const mesh = new THREE.Mesh(geo, this._roadMat);
      mesh.userData.ribbon = true;
      this.trackGroup.add(mesh);
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
    // sit the flag ON the surface at finishX (physics y = surfaceY) and on the curving
    // band via the path transform so it stays planted as the track turns. The pole/flag
    // heights are added in render-y AFTER the transform (the transform's worldY = -physY).
    const surfY = (physics && physics.surfaceYAt(track.finishX)) || 0;
    const fx = track.finishX;
    for (const laneZ of lanes) {
      const base = this.path ? this.path.transform(fx, laneZ, surfY, this._tp) : { x: fx, y: -surfY, z: laneZ };
      const pole = new THREE.Mesh(
        new THREE.PlaneGeometry(0.15, 3),
        new THREE.MeshBasicMaterial({ color: 0xffffff })
      );
      pole.position.set(base.x, base.y + 1.5, base.z);
      this.trackGroup.add(pole);
      const flag = new THREE.Mesh(
        new THREE.PlaneGeometry(1.2, 0.8),
        new THREE.MeshBasicMaterial({ map: flagTex, side: THREE.DoubleSide })
      );
      // offset the flag a little FORWARD along the heading (was +0.6 in x).
      const fwd = this.path ? this.path.transform(fx + 0.6, laneZ, surfY, this._tpL) : { x: fx + 0.6, y: -surfY, z: laneZ };
      flag.position.set(fwd.x, base.y + 2.5, fwd.z);
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
      // LATERAL offset = lane centre + the side straddle (±LEG_Z_OFFSET). This is the
      // PERPENDICULAR-to-heading offset L the path transform applies per-frame in
      // _syncLegGroups (it depends on the live x → heading), so the two legs stay a clean
      // left/right pair as the path turns. RENDER INTERPOLATION: store the leg's
      // phaseOffset so _syncLegGroups can rebuild the INTERPOLATED world angle.
      // `lateral` = the perpendicular-to-heading offset the transform uses; `z` is kept as
      // an alias (== lateral) for back-compat: at heading 0 it IS the world z, and its sign
      // (left<0 / right>0) is exactly what the leg-straddle assertions check.
      const lateral = laneZ + l.side * LEG_Z_OFFSET;
      groups.push({ mesh: grp, body, side: l.side, phaseOffset: l.phaseOffset || 0, laneZ, lateral, z: lateral });
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
      // PATH TRANSFORM: cube on the lane centre (L=0) EXCEPT inside a fork COMMITTED to the HIGH
      // route, where it rides the arch's LATERAL z-split (forkLateralAt) so it stays WYSIWYG on the
      // sideways-then-over arch (the low road + all normal track stay centred ⇒ L=0). It yaws by
      // the heading so it FACES along the (turning) path, plus its fwd/back tilt about z (interpAngle).
      const lat = physics.forkLateralAt ? physics.forkLateralAt(ix) : 0;
      const p = this.path.transform(ix, lat, iy, this._tp);
      this.cubeMesh.position.set(p.x, p.y, p.z);
      this.cubeMesh.rotation.set(0, this.path.heading(ix), -physics.interpAngle(alpha));
    }
    this._syncLegGroups(this.legGroups, physics, alpha);
    // BALL-FIELD: drive the reused player-lane sphere meshes from the live pile.
    if (this.ballMeshes.length) this._syncBalls(physics, 0, this.ballMeshes);
    // BREAKING-BLOCK: drive the reused player-lane block + debris meshes from the live wall.
    if (this.blockMeshes.length || this.debrisMeshes.length) this._syncBlocks(physics, 0, this.blockMeshes, this.debrisMeshes);
  }

  /** Sync the rival cube + its legs (call every frame when racing). alpha: see sync(). */
  syncRival(rival, alpha = 0) {
    if (this.rivalCubeMesh && rival.cube) {
      // §C: rival lane base + serpentine curve at its own x. INTERPOLATED (see sync()).
      const ix = rival.interpX(alpha);
      const iy = rival.interpBodyY(alpha);
      // PATH TRANSFORM: rival on its parallel lane (L = rivalLaneZ), yawed to face the path.
      const p = this.path.transform(ix, this.rivalLaneZ, iy, this._tp);
      this.rivalCubeMesh.position.set(p.x, p.y, p.z);
      this.rivalCubeMesh.rotation.set(0, this.path.heading(ix), -rival.interpAngle(alpha));
    }
    this._syncLegGroups(this.rivalLegGroups, rival, alpha);
    // BALL-FIELD (rival lane): lazily build the rival pile meshes the first time we see
    // its field (the rival walker is built after the player buildTrack), then drive them.
    if (rival.ballCount && this.rivalBallMeshes.length !== rival.ballCount) {
      this._buildBalls(rival, this.rivalLaneZ);
    }
    if (this.rivalBallMeshes.length) this._syncBalls(rival, this.rivalLaneZ, this.rivalBallMeshes);
    // BREAKING-BLOCK (rival lane): lazily build the rival wall + debris meshes the first time
    // we see its field (the rival walker is built after the player buildTrack), then drive them.
    if ((rival.blockCount || rival.debrisCount) &&
        (this.rivalBlockMeshes.length !== rival.blockCount || this.rivalDebrisMeshes.length !== rival.debrisCount)) {
      this._buildBlocks(rival, this.rivalLaneZ);
    }
    if (this.rivalBlockMeshes.length || this.rivalDebrisMeshes.length) {
      this._syncBlocks(rival, this.rivalLaneZ, this.rivalBlockMeshes, this.rivalDebrisMeshes);
    }
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
      // PATH TRANSFORM: both legs share the cube x/y; the stored `lateral` (lane centre +
      // the side straddle) is the perpendicular-to-heading offset, so the legs stay a
      // clean left/right pair as the path turns. Inside a fork COMMITTED HIGH, add the arch's
      // LATERAL z-split so the legs ride the sideways-then-over arch WITH the cube (same offset,
      // 0 elsewhere). The drawn stroke spins about z in the path-local plane (interpLegAngle), and
      // the WHOLE group additionally yaws by the heading so that local plane stays oriented along
      // the (turning) path.
      const flat = (physics && physics.forkLateralAt) ? physics.forkLateralAt(bx) : 0;
      const p = this.path.transform(bx, lg.lateral + flat, by, this._tp);
      lg.mesh.position.set(p.x, p.y, p.z);
      // render y = -physY -> a CCW physics rotation appears CW on screen.
      const ang = physics ? physics.interpLegAngle(alpha, lg.phaseOffset) : body.angle;
      lg.mesh.rotation.set(0, this.path.heading(bx), -ang);
    }
  }

  /** §D — 3/4 chase camera that FOLLOWS THE HEADING so a real bend reads as a turn
   * (chase-cam swinging through the curve), while staying ~identical to today on
   * straights and SMOOTH (eased heading + eased vertical, no snap/shake). Reference
   * framing: the cube fills ~1/3 of the screen, lower-centre, big sky, the winding
   * ribbon receding ahead. Keeps the smooth _camY vertical glide and the render interp. */
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
    // HEADING FOLLOW: ease a _camHeading toward the path heading at the cube (same
    // dt-based low-pass as the vertical glide) so the camera SWINGS smoothly through a
    // bend — no snap, no shake. On a straight the heading is the tiny serpentine sway, so
    // the camera reads essentially identical to before. Frame-rate independent (60/120Hz).
    const tgtH = this.path ? this.path.heading(x) : 0;
    if (this._camHeading == null || !Number.isFinite(this._camHeading)) this._camHeading = tgtH;
    else this._camHeading += (tgtH - this._camHeading) * camA;
    const hc = this._camHeading;

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
    // HEADING-FOLLOW CHASE: build the camera offset in the PATH-LOCAL frame (the EXACT
    // same relative geometry as before — a bit BEHIND along the tangent, UP, and to the
    // +lateral/camera side), then rotate it by the eased heading `hc` so it SWINGS with
    // the path through a REAL bend. `hc` is the TURNS-ONLY heading (0 on straights/serpentine),
    // so on a straight (hc≈0) T=(1,0,0), Lat=(0,0,1) ⇒ the math below reduces to exactly the
    // old camX=x-3.2 / camZ=centreZ+10.6 / lookAt(x+1.5,…,centreZ) — the camera stays SIDE-ON
    // and never swings with the cosmetic serpentine wiggle.
    //   BEHIND_T : behind the cube along -tangent (was the -3.2 in x)
    //   SIDE_LAT : to the +z camera side along +lateral (was the 10.6/11.2 in z)
    //   FWD_T    : look a little ahead along +tangent (was the +1.5 in x)
    const BEHIND_T = -3.2;
    const SIDE_LAT = racing ? 11.2 : 10.6;
    const camYoff = racing ? 5.0 : 4.5;
    const FWD_T = 1.5;
    const lookLat = racing ? this.rivalLaneZ * 0.30 : 0;
    const cosH = Math.cos(hc), sinH = Math.sin(hc);
    // path-local basis in the horizontal (XZ) plane:
    //   tangent  T   = ( cosH, sinH)   (+x at hc=0)
    //   lateral  Lat = (-sinH, cosH)   (+z at hc=0, == the transform's +L direction)
    // CUBE LANE-CENTRE world XZ — taken from the SAME transform the cube uses (L=0), so it
    // INCLUDES the cosmetic serpentine lateral offset. The camera therefore tracks the cube's
    // gentle side-to-side sway (cube stays fixed in frame, the whole band sways), exactly like
    // the old pre-curve game's centreZ = laneZ + laneCurveZ(x).
    const cc = this.path ? this.path.transform(x, 0, 0, this._tp) : null;
    const cx = cc ? cc.x : x;
    const cz = cc ? cc.z : 0;
    // camera eye = centre + BEHIND·T + SIDE·Lat (horizontal) and camY (vertical).
    const camX = cx + BEHIND_T * cosH + SIDE_LAT * (-sinH);
    const camZ = cz + BEHIND_T * sinH + SIDE_LAT * (cosH);
    const camY = this._camY + camYoff;
    this.camera.position.set(camX, camY, camZ);
    // look at the cube a little ahead along the tangent (+ a small lateral when racing).
    const lookX = cx + FWD_T * cosH + lookLat * (-sinH);
    const lookZ = cz + FWD_T * sinH + lookLat * (cosH);
    this.camera.lookAt(lookX, this._camY + 0.2, lookZ);
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
    // TWO-ZONE texture so the WHOLE road renders in ONE draw call (one material) while still
    // showing the purple STRIPE top AND the solid EDGE-coloured sides:
    //   • V in [0, 0.5)  = the STRIPE zone: STRIPE_BANDS purple bands across U (the top rhythm).
    //   • V in [0.5, 1]  = a solid WHITE swatch. SIDE/riser/bottom/cap verts sample HERE and
    //     carry a per-vertex EDGE colour ⇒ white×edge = the solid edge colour (no stripe shows
    //     through). TOP verts carry WHITE vertex colour ⇒ the stripe shows at full vivid tone.
    // U wraps (RepeatWrapping) for the endless stripe rhythm; V is CLAMPED so the two zones
    // never bleed under wrap. NearestFilter keeps band edges + the zone split crisp.
    const c = document.createElement('canvas');
    c.width = 64; c.height = 16;           // top 8px = stripes, bottom 8px = white swatch
    const ctx = c.getContext('2d');
    const bandW = c.width / STRIPE_BANDS;
    const TONES = ['#8E3AAE', '#C24FD6'];  // --track-a (deep) ↔ --track-b (bright)
    for (let i = 0; i < STRIPE_BANDS; i++) {
      ctx.fillStyle = TONES[i % 2];
      ctx.fillRect(Math.round(i * bandW), 0, Math.ceil(bandW), 8);   // STRIPE zone (top half)
    }
    ctx.fillStyle = '#ffffff';
    ctx.fillRect(0, 8, c.width, 8);        // WHITE swatch (bottom half) ⇒ ×edge-colour on sides
    const tex = new THREE.CanvasTexture(c);
    tex.colorSpace = THREE.SRGBColorSpace;
    tex.wrapS = THREE.RepeatWrapping;      // endless stripe rhythm along U
    tex.wrapT = THREE.ClampToEdgeWrapping; // keep the two V zones from bleeding into each other
    tex.magFilter = THREE.NearestFilter;   // crisp band edges + zone split (no muddy blur)
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
    return m === this._legMat || m === this._roadMat || m === this._topMat || m === this._sideMat
      || m === this._ballMat || m === this._blockMat || m === this._debrisMat;
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
