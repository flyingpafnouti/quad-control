/*
 * main.js — Point d'entrée : câblage modèle / contrôleurs / UI / boucle.
 */
(function (QUAD) {
  'use strict';

  const { Quadrotor2D, Simulator, Renderer, ParamPanel } = QUAD;

  // --- Modèle physique ---
  const model = new Quadrotor2D();

  // --- Registre des contrôleurs (facile d'en ajouter d'autres ici) ---
  const controllers = {
    cascade: new QUAD.CascadePID(model),
    lqr: new QUAD.LQR(model),
    rl: new QUAD.RLController(model),
  };
  let current = controllers.cascade;

  // --- Simulateur ---
  const sim = new Simulator(model, current);
  sim.reset();

  // --- Rendu + panneau ---
  const canvas = document.getElementById('sim');
  const renderer = new Renderer(canvas, sim);
  const panel = new ParamPanel(
    document.getElementById('params'),
    () => current
  );
  panel.rebuild();

  /* ------------------------- Éléments UI ------------------------- */
  const $ = (id) => document.getElementById(id);
  const selCtrl = $('controller-select');
  const btnRun = $('btn-run');
  const btnReset = $('btn-reset');
  const windSlider = $('wind');
  const windVal = $('wind-val');

  selCtrl.addEventListener('change', () => {
    current = controllers[selCtrl.value];
    sim.setController(current);
    panel.rebuild();
  });

  btnRun.addEventListener('click', () => {
    sim.running = !sim.running;
    btnRun.textContent = sim.running ? '❚❚ Pause' : '▶ Démarrer';
    btnRun.classList.toggle('active', sim.running);
  });

  btnReset.addEventListener('click', () => {
    sim.reset();
  });

  windSlider.addEventListener('input', () => {
    sim.disturb.fx = parseFloat(windSlider.value);
    windVal.textContent = windSlider.value + ' N';
  });

  // Perturbation impulsionnelle (bouton "coup de vent")
  $('btn-kick').addEventListener('click', () => {
    const s = 6; // amplitude
    const fx = (Math.random() < 0.5 ? -1 : 1) * s;
    const fy = (Math.random() < 0.5 ? -1 : 1) * s * 0.6;
    sim.applyImpulse(fx, fy);
  });

  // Clic sur le canvas -> nouvelle consigne de position
  canvas.addEventListener('click', (e) => {
    const rect = canvas.getBoundingClientRect();
    const w = renderer.s2w(e.clientX - rect.left, e.clientY - rect.top);
    sim.ref.x = Math.max(-7, Math.min(7, w.x));
    sim.ref.y = Math.max(0.2, Math.min(7, w.y));
  });

  /* ------------------------- Télémétrie ------------------------- */
  const telEls = {
    t: $('tel-t'), x: $('tel-x'), y: $('tel-y'), th: $('tel-th'),
    F: $('tel-F'), M: $('tel-M'), T1: $('tel-T1'), T2: $('tel-T2'),
    thref: $('tel-thref'),
  };

  function updateTelemetry() {
    const s = model.state;
    const tm = sim.telemetry;
    telEls.t.textContent = sim.time.toFixed(2) + ' s';
    telEls.x.textContent = s.x.toFixed(2) + ' m';
    telEls.y.textContent = s.y.toFixed(2) + ' m';
    telEls.th.textContent = (s.th * 180 / Math.PI).toFixed(1) + '°';
    telEls.thref.textContent = (tm.thRef * 180 / Math.PI).toFixed(1) + '°';
    telEls.F.textContent = tm.F.toFixed(2) + ' N';
    telEls.M.textContent = tm.M.toFixed(3) + ' N·m';
    telEls.T1.textContent = tm.T1.toFixed(2) + ' N';
    telEls.T2.textContent = tm.T2.toFixed(2) + ' N';
  }

  /* ------------------------- Boucle temps réel ------------------------- */
  let last = performance.now();
  function loop(now) {
    const dt = (now - last) / 1000;
    last = now;
    sim.advance(dt);
    renderer.draw();
    updateTelemetry();
    requestAnimationFrame(loop);
  }
  requestAnimationFrame(loop);

  // expose pour le débogage en console
  QUAD._app = { model, sim, controllers, renderer };
})(window.QUAD || (window.QUAD = {}));
