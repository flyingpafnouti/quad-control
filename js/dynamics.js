/*
 * dynamics.js — Modèle du quadrirotor planaire (birotor), 3 DDL.
 *
 * Repère physique : x vers la droite, y vers le HAUT, theta = angle de
 * tangage (rotation autour de l'axe hors-plan), positif dans le sens direct.
 *
 * Deux rotors situés en +L (droite) et -L (gauche) du centre de masse.
 *   Poussée totale : F  = T1 + T2        (>= 0, dirigée selon l'axe corps)
 *   Couple         : M  = L * (T1 - T2)
 *
 * Équations du mouvement :
 *   x''    = -(F/m) sin(theta)
 *   y''    =  (F/m) cos(theta) - g
 *   theta''=  M / Iz
 *
 * État : [x, vx, y, vy, theta, omega]
 * Entrées : T1, T2 (poussées des deux rotors, déjà saturées).
 */
(function (QUAD) {
  'use strict';

  class Quadrotor2D {
    constructor(params = {}) {
      this.p = Object.assign(
        {
          m: 1.0,        // masse (kg)
          L: 0.25,       // demi-envergure (m)
          Iz: 0.02,      // inertie autour de l'axe hors-plan (kg.m^2)
          g: 9.81,       // gravité (m/s^2)
          Tmax: 1.6,     // poussée max par rotor, en multiples de (m*g/2)
        },
        params
      );
      this.reset();
    }

    /** Poussée max absolue par rotor (N). Tmax est un facteur du hover/rotor. */
    rotorMax() {
      const { m, g, Tmax } = this.p;
      return Tmax * (m * g) / 2;
    }

    /** Poussée de vol stationnaire par rotor (N). */
    rotorHover() {
      return (this.p.m * this.p.g) / 2;
    }

    reset(state) {
      // état par défaut : posé au sol un peu à gauche, immobile
      this.s = state
        ? state.slice()
        : [-1.5, 0, 0.0, 0, 0, 0];
    }

    /** Dérivée de l'état pour des poussées (T1, T2) données. */
    deriv(s, T1, T2) {
      const { m, L, Iz, g } = this.p;
      const [, vx, , vy, th, om] = s;
      const F = T1 + T2;
      const Mz = L * (T1 - T2);
      const ax = -(F / m) * Math.sin(th);
      const ay = (F / m) * Math.cos(th) - g;
      const ath = Mz / Iz;
      return [vx, ax, vy, ay, om, ath];
    }

    /**
     * Intègre un pas dt avec RK4, à poussées constantes (T1, T2).
     * Applique aussi une force de perturbation externe (fx, fy) en N,
     * utile pour tester la robustesse des contrôleurs.
     */
    step(dt, T1, T2, disturb = { fx: 0, fy: 0 }) {
      const { m } = this.p;
      const df = (s) => {
        const d = this.deriv(s, T1, T2);
        // ajoute l'accélération due à la perturbation (repère monde)
        d[1] += disturb.fx / m;
        d[3] += disturb.fy / m;
        return d;
      };
      const s0 = this.s;
      const k1 = df(s0);
      const k2 = df(this._axpy(s0, k1, dt / 2));
      const k3 = df(this._axpy(s0, k2, dt / 2));
      const k4 = df(this._axpy(s0, k3, dt));
      const out = s0.slice();
      for (let i = 0; i < 6; i++) {
        out[i] += (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
      }
      this.s = out;
      return out;
    }

    _axpy(s, k, a) {
      const out = s.slice();
      for (let i = 0; i < 6; i++) out[i] += a * k[i];
      return out;
    }

    /** Accès nommé à l'état courant. */
    get state() {
      const [x, vx, y, vy, th, om] = this.s;
      return { x, vx, y, vy, th, om };
    }

    /** Conversion (F, M) -> (T1, T2) avec saturation des rotors. */
    mixToThrusts(F, M) {
      const { L } = this.p;
      let T1 = (F + M / L) / 2;
      let T2 = (F - M / L) / 2;
      const Tmax = this.rotorMax();
      T1 = Math.max(0, Math.min(Tmax, T1));
      T2 = Math.max(0, Math.min(Tmax, T2));
      return { T1, T2 };
    }
  }

  QUAD.Quadrotor2D = Quadrotor2D;
})(window.QUAD || (window.QUAD = {}));
