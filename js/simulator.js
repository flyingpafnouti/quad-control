/*
 * simulator.js — Orchestration de la simulation.
 *
 * Relie le modèle physique et le contrôleur actif, avec un pas de temps
 * fixe (intégration déterministe indépendante du framerate d'affichage).
 * Gère la consigne, les perturbations, et enregistre la dernière télémétrie.
 */
(function (QUAD) {
  'use strict';

  class Simulator {
    constructor(model, controller) {
      this.model = model;
      this.controller = controller;
      this.ref = { x: 0, y: 2.0 };       // consigne de position
      this.dtPhys = 0.004;                // pas physique (250 Hz)
      this.time = 0;
      this.running = false;
      this.disturb = { fx: 0, fy: 0 };    // perturbation permanente (vent)
      this.impulse = { fx: 0, fy: 0, t: 0 }; // perturbation impulsionnelle
      this.telemetry = {
        F: 0, M: 0, T1: 0, T2: 0, thRef: 0,
      };
      // historique de trajectoire pour l'affichage
      this.trail = [];
      this.trailMax = 400;
    }

    setController(controller) {
      this.controller = controller;
      this.controller.reset();
    }

    reset(state) {
      this.model.reset(state);
      this.controller.reset();
      this.time = 0;
      this.trail.length = 0;
      this.telemetry = { F: 0, M: 0, T1: 0, T2: 0, thRef: 0 };
    }

    /** Applique une impulsion de force (N) pendant ~duration secondes. */
    applyImpulse(fx, fy, duration = 0.12) {
      this.impulse = { fx, fy, t: duration };
    }

    /** Fait avancer la simulation d'une durée réelle (s), en sous-pas fixes. */
    advance(realDt) {
      if (!this.running) return;
      // borne pour éviter la spirale de la mort après un onglet en arrière-plan
      let remaining = Math.min(realDt, 0.05);
      while (remaining > 1e-9) {
        const dt = Math.min(this.dtPhys, remaining);
        this._stepOnce(dt);
        remaining -= dt;
      }
    }

    _stepOnce(dt) {
      const state = this.model.state;

      // 1) Contrôleur -> commande (F, M)
      const cmd = this.controller.compute(state, this.ref, dt);
      const F = cmd.F;
      const M = cmd.M;

      // 2) Mixage -> poussées rotors saturées
      const { T1, T2 } = this.model.mixToThrusts(F, M);

      // 3) Perturbations : vent permanent + impulsion résiduelle
      let fx = this.disturb.fx;
      let fy = this.disturb.fy;
      if (this.impulse.t > 0) {
        fx += this.impulse.fx;
        fy += this.impulse.fy;
        this.impulse.t -= dt;
      }

      // 4) Intégration physique
      this.model.step(dt, T1, T2, { fx, fy });
      this.time += dt;

      // 5) Télémétrie + trace
      this.telemetry = { F, M, T1, T2, thRef: cmd.thRef != null ? cmd.thRef : 0 };
      const s = this.model.state;
      this.trail.push({ x: s.x, y: s.y });
      if (this.trail.length > this.trailMax) this.trail.shift();
    }
  }

  QUAD.Simulator = Simulator;
})(window.QUAD || (window.QUAD = {}));
