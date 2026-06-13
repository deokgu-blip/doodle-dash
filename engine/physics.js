// engine/physics.js
// LOCOMOTION CORE — now a PROCEDURAL / KINEMATIC walker (see engine/walker.js).
//
// The original Draw Climber is NOT a rigid-body simulation; it is a designed
// "looks-like-physics" animation. We rewrote the locomotion from a Matter.js
// solver (which fought bottom penetration / slip / launches for 9+ rounds) to a
// deterministic kinematic walker with structural guarantees (0 penetration, 0
// slip, monotone length→speed, designed climb rule). The VALIDATED look & input
// are unchanged: the drawn-stroke thin pen LINE, the cube-centre anchor, two
// legs 180° out of phase, the input pipeline, the window.__DC headless API, the
// fixed timestep and the single-file build.
//
// This module simply re-exports the walker so every existing import
// (game.js, renderer.js, index.html) keeps working unchanged.
export { Physics, presetStroke, PHYS_CONST } from './walker.js';
