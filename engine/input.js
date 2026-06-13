// engine/input.js
// Draw-box pointer input (POC §5). Captures a stroke in the bottom draw box,
// normalizes it to the [-1,1]^2 box, and feeds it to the game as a new leg.
// Pointer Events => touch + mouse. Box-outside touches are ignored. Redraw
// replaces the previous leg (no start button — immediate).

export function attachDrawInput(drawCanvas, game, opts = {}) {
  const ctx = drawCanvas.getContext('2d');
  let drawing = false;
  let pts = [];      // raw stroke in canvas px
  const minPts = 3;

  const resize = () => {
    const r = drawCanvas.getBoundingClientRect();
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    drawCanvas.width = Math.max(2, Math.round(r.width * dpr));
    drawCanvas.height = Math.max(2, Math.round(r.height * dpr));
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    redrawStrokePreview();
  };

  const toLocal = (e) => {
    const r = drawCanvas.getBoundingClientRect();
    return { x: e.clientX - r.left, y: e.clientY - r.top, w: r.width, h: r.height };
  };

  const redrawStrokePreview = () => {
    const r = drawCanvas.getBoundingClientRect();
    ctx.clearRect(0, 0, r.width, r.height);
    if (pts.length < 2) return;
    ctx.lineWidth = 6; ctx.lineCap = 'round'; ctx.lineJoin = 'round';
    ctx.strokeStyle = '#1A1A1A';
    ctx.beginPath();
    ctx.moveTo(pts[0].x, pts[0].y);
    for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i].x, pts[i].y);
    ctx.stroke();
  };

  // Normalize raw px stroke to box [-1,1]^2 centered on the draw box.
  const normalize = () => {
    const r = drawCanvas.getBoundingClientRect();
    const cx = r.width / 2, cy = r.height / 2;
    const half = Math.min(r.width, r.height) / 2;
    return pts.map((p) => ({
      x: (p.x - cx) / half,
      y: (p.y - cy) / half, // canvas y-down; physics also normalizes consistently
    }));
  };

  const onDown = (e) => {
    drawing = true; pts = [];
    const l = toLocal(e);
    pts.push({ x: l.x, y: l.y });
    drawCanvas.setPointerCapture && drawCanvas.setPointerCapture(e.pointerId);
    redrawStrokePreview();
    // §A: the user STARTED drawing ⇒ enter bullet-time (whole game slows to 10%).
    // The game stays 'running' — the cube keeps creeping forward while you redraw.
    if (game.beginDraw) game.beginDraw();
    e.preventDefault();
  };
  const onMove = (e) => {
    if (!drawing) return;
    const l = toLocal(e);
    const last = pts[pts.length - 1];
    if (!last || Math.hypot(l.x - last.x, l.y - last.y) > 3) {
      pts.push({ x: l.x, y: l.y });
      redrawStrokePreview();
    }
    e.preventDefault();
  };
  const onUp = (e) => {
    if (!drawing) return;
    drawing = false;
    if (pts.length >= minPts) {
      const norm = normalize();
      // §A: apply the new leg — CONTINUE from the current x (walker.setLegStroke
      // preserves x mid-run), no restart. Then leave bullet-time → full speed.
      game.setLegStroke(norm);
      if (opts.onStroke) opts.onStroke(norm);
    }
    // §A: stroke finished (applied or too-short) ⇒ back to full speed.
    if (game.endDraw) game.endDraw();
    e.preventDefault();
  };

  drawCanvas.addEventListener('pointerdown', onDown);
  drawCanvas.addEventListener('pointermove', onMove);
  drawCanvas.addEventListener('pointerup', onUp);
  drawCanvas.addEventListener('pointercancel', onUp);
  window.addEventListener('resize', resize);
  resize();

  return { resize };
}
