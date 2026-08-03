/*
 * verify_js.js — Vérifie la politique RL en chargeant les VRAIS fichiers du
 * navigateur (js/dynamics.js, js/controllers/*.js, js/pid.js) dans Node, puis
 * en simulant plusieurs épisodes en boucle fermée avec le RLController.
 *
 * Usage : node training/verify_js.js
 */
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const ROOT = path.join(__dirname, '..');
const sandbox = { window: {}, Math, console };
sandbox.window.QUAD = {};
vm.createContext(sandbox);

function load(rel) {
  const code = fs.readFileSync(path.join(ROOT, rel), 'utf8');
  vm.runInContext(code, sandbox, { filename: rel });
}

// Ordre de chargement identique à index.html (math -> dynamics -> pid -> base -> ctrls)
['js/math.js', 'js/dynamics.js', 'js/pid.js', 'js/controllers/base.js',
 'js/controllers/rl_weights.js', 'js/controllers/rl.js'].forEach(load);

const QUAD = sandbox.window.QUAD;
if (!QUAD.RL_WEIGHTS) {
  console.error('ERREUR : rl_weights.js non chargé (lancer l\'entraînement d\'abord).');
  process.exit(1);
}
console.log('Architecture réseau :', QUAD.RL_WEIGHTS.meta.arch.join(' -> '));

const model = new QUAD.Quadrotor2D();
const ctrl = new QUAD.RLController(model);

// Réplique la boucle physique du simulateur (dtPhys = 0.004, RK4 via model.step).
function runEpisode(start, ref, steps = 2500) {
  model.reset(start.slice());
  ctrl.reset();
  const dt = 0.004;
  let crashed = false;
  for (let i = 0; i < steps; i++) {
    const st = model.state;
    const cmd = ctrl.compute(st, ref, dt);
    const { T1, T2 } = model.mixToThrusts(cmd.F, cmd.M);
    model.step(dt, T1, T2, { fx: 0, fy: 0 });
    const s = model.state;
    if (s.y < -0.1 || Math.abs(s.th) > 1.4 || Math.abs(s.x) > 9) { crashed = true; break; }
  }
  const s = model.state;
  const err = Math.hypot(s.x - ref.x, s.y - ref.y);
  const speed = Math.hypot(s.vx, s.vy);
  return { err, speed, th: s.th, crashed, final: s };
}

const tests = [
  { start: [-1.5, 0, 0, 0, 0, 0], ref: { x: 0, y: 2.0 }, name: 'décollage->(0,2)' },
  { start: [-4, 0, 1, 0, 0, 0], ref: { x: 3, y: 5 }, name: 'diagonale ->(3,5)' },
  { start: [4, 0, 5, 0, 0.2, 0], ref: { x: -3, y: 1.5 }, name: 'descente ->(-3,1.5)' },
  { start: [0, 0, 3, 0, 0, 0], ref: { x: 0, y: 3 }, name: 'maintien (0,3)' },
  { start: [2, 1.5, 2, -1, -0.3, 0.5], ref: { x: 0, y: 4 }, name: 'perturbé ->(0,4)' },
];

let pass = 0;
const pad = (s, n) => String(s).padEnd(n);
const padL = (s, n) => String(s).padStart(n);
console.log('\n' + ['OK', pad('test', 22), padL('err final', 10), padL('vitesse', 8), padL('θ(deg)', 7), 'état'].join(' | '));
console.log('-'.repeat(78));
for (const t of tests) {
  const r = runEpisode(t.start, t.ref);
  const ok = !r.crashed && r.err < 0.5 && r.speed < 0.6;
  if (ok) pass++;
  console.log([
    ok ? ' ✓' : ' ✗',
    pad(t.name, 22),
    padL(r.err.toFixed(3) + ' m', 10),
    padL(r.speed.toFixed(3), 8),
    padL((r.th * 180 / Math.PI).toFixed(1), 7),
    r.crashed ? 'CRASH' : 'ok',
  ].join(' | '));
}
console.log('-'.repeat(78));
console.log(`\nRésultat : ${pass}/${tests.length} tests réussis (err<0.5m, vitesse<0.6m/s, pas de crash)`);
process.exit(pass === tests.length ? 0 : 2);
