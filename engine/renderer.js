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
};

const RIBBON_DEPTH = 6.0;     // z-extrusion of the track (wide so it reads as ground)
const RIBBON_DOWN = 0.5;      // how far below the surface the ribbon goes

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
    this.camera = new THREE.PerspectiveCamera(50, 1, 0.1, 200);

    this.cubeMesh = null;
    this.faceMesh = null;
    this.legGroups = [];     // [{ mesh, body, side }] — one extruded disc per wheel
    this.trackGroup = null;

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

  /** Build the track ribbon + ground from the physics floor bodies. */
  buildTrack(physics, track) {
    if (this.trackGroup) { this.scene.remove(this.trackGroup); this._disposeGroup(this.trackGroup); }
    if (this.cubeMesh) { this.scene.remove(this.cubeMesh); }
    this.trackGroup = new THREE.Group();

    // checkerboard texture for the ribbon top
    const checker = this._makeChecker();

    // One box per floor body (top surface), checker on top face.
    for (const b of physics.floorBodies) {
      const w = b.bounds.max.x - b.bounds.min.x;
      const h = b.bounds.max.y - b.bounds.min.y;
      const cx = b.position.x;
      const cyPhys = b.position.y; // physics y (+down)
      // render extends the slab down + gives it depth in z
      const geo = new THREE.BoxGeometry(w, h + RIBBON_DOWN, RIBBON_DEPTH);
      const sideMat = new THREE.MeshStandardMaterial({ color: COL.trackEdge, roughness: 0.8 });
      const topMat = new THREE.MeshStandardMaterial({ map: checker, roughness: 0.5 });
      // BoxGeometry material order: +x,-x,+y,-y,+z,-z. +y is top.
      const mats = [sideMat, sideMat, topMat, sideMat, sideMat, sideMat];
      const mesh = new THREE.Mesh(geo, mats);
      mesh.position.set(cx, -(cyPhys + RIBBON_DOWN / 2), 0);
      this.trackGroup.add(mesh);
    }
    this.scene.add(this.trackGroup);

    // ── player cube ──
    const cubeGeo = new THREE.BoxGeometry(PHYS_CONST.CUBE_SIZE, PHYS_CONST.CUBE_SIZE, PHYS_CONST.CUBE_SIZE * 0.9);
    const cubeMat = new THREE.MeshStandardMaterial({ color: COL.player, roughness: 0.45, metalness: 0.05 });
    this.cubeMesh = new THREE.Mesh(cubeGeo, cubeMat);
    // dot-eye face on +z
    const faceTex = this._makeFace();
    const faceMat = new THREE.MeshBasicMaterial({ map: faceTex, transparent: true });
    const faceGeo = new THREE.PlaneGeometry(PHYS_CONST.CUBE_SIZE * 0.92, PHYS_CONST.CUBE_SIZE * 0.92);
    this.faceMesh = new THREE.Mesh(faceGeo, faceMat);
    this.faceMesh.position.z = PHYS_CONST.CUBE_SIZE * 0.46;
    this.cubeMesh.add(this.faceMesh);
    this.scene.add(this.cubeMesh);

    // finish flag (simple marker at finishX)
    this._buildFinish(track);
  }

  _buildFinish(track) {
    const geo = new THREE.PlaneGeometry(0.15, 3);
    const mat = new THREE.MeshBasicMaterial({ color: 0xffffff });
    const pole = new THREE.Mesh(geo, mat);
    pole.position.set(track.finishX, 1.4, 0);
    this.trackGroup.add(pole);
    const flagTex = this._makeChecker(6);
    const flag = new THREE.Mesh(
      new THREE.PlaneGeometry(1.2, 0.8),
      new THREE.MeshBasicMaterial({ map: flagTex, side: THREE.DoubleSide })
    );
    flag.position.set(track.finishX + 0.6, 2.4, 0);
    this.trackGroup.add(flag);
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
    for (const lg of this.legGroups) { this.scene.remove(lg.mesh); this._disposeMesh(lg.mesh); }
    this.legGroups = [];
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
      // z offset by side so the two legs straddle the cube (read as two legs).
      this.legGroups.push({ mesh: grp, body, side: l.side, z: l.side * LEG_Z_OFFSET });
    }
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
      // ground (the reference look). render y = -physY.
      this.cubeMesh.position.set(p.x, -p.y, 0);
      this.cubeMesh.rotation.z = -physics.cube.angle;
    }
    for (const lg of this.legGroups) {
      const body = lg.body;
      // Both legs share the same physics x/y; the z offset gives the two-leg
      // (left/right) straddle look while each spins at its own (180°-offset)
      // angle — the alternating walk.
      lg.mesh.position.set(body.position.x, -body.position.y, lg.z);
      // render y = -physY -> a CCW physics rotation appears CW on screen.
      lg.mesh.rotation.z = -body.angle;
    }
  }

  /** 3/4 chase camera following the cube. Frames both the cube and the
   * track ribbon below it. */
  updateCamera(physics) {
    const x = physics.cube ? physics.cube.position.x : physics.startX;
    const y = physics.cube ? -physics.cube.position.y : 0;
    // 3/4 chase view. With CUBE_DROP the cube renders ~0.9 above the track
    // surface (render y 0). Aim between them and keep the camera behind & a bit
    // above so both the smiley cube and the magenta checkerboard ribbon are in
    // frame, ribbon receding into the distance.
    // 3/4 chase view: camera behind (-x) and above (+y), looking at the cube
    // with a downward tilt so the wide magenta checkerboard ribbon reads as the
    // ground the cube rolls on, receding ahead.
    const cubeRenderY = (physics.cube ? -physics.cube.position.y : 0.9);
    // Steeper, higher 3/4 chase. The earlier shallow (~25°) angle saw the flat
    // ribbon nearly edge-on so it hid behind the cube/legs; the reference views
    // the lane from ~40° above. Sit behind (-x), well above (+y), a bit to the
    // side (+z) for the 3/4 feel, and look DOWN the track ahead so the magenta
    // checkerboard reads clearly as the ground receding forward.
    // Pull the camera more to the +z side so the two legs straddling the cube
    // in depth (z = ±LEG_Z_OFFSET) are visibly separated (one near, one far),
    // not stacked dead-on. Keep it behind (-x) and above (+y) for the 3/4 chase.
    const camX = x - 5.5;
    const camY = cubeRenderY + 6.4;
    const camZ = 8.0;
    this.camera.position.set(camX, camY, camZ);
    // Aim only slightly ahead of the cube so the cube sits centered (not shoved
    // to the left edge) with the magenta lane receding ahead of it.
    this.camera.lookAt(x + 1.5, cubeRenderY - 1.0, 0);
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
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
      ctx.fillStyle = ((i + j) % 2 === 0) ? '#8E3AAE' : '#C24FD6';
      ctx.fillRect(i * s, j * s, s, s);
    }
    const tex = new THREE.CanvasTexture(c);
    tex.colorSpace = THREE.SRGBColorSpace;
    tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
    return tex;
  }

  _makeFace() {
    const c = document.createElement('canvas');
    c.width = c.height = 128;
    const ctx = c.getContext('2d');
    ctx.clearRect(0, 0, 128, 128);
    ctx.fillStyle = '#0E2A3A';
    // two dot eyes
    ctx.beginPath(); ctx.arc(46, 54, 10, 0, Math.PI * 2); ctx.fill();
    ctx.beginPath(); ctx.arc(82, 54, 10, 0, Math.PI * 2); ctx.fill();
    // smile
    ctx.lineWidth = 6; ctx.strokeStyle = '#0E2A3A'; ctx.lineCap = 'round';
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
