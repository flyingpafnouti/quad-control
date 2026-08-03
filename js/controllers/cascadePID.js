/*
 * controllers/cascadePID.js — Contrôleur en cascade à base de PID.
 *
 * Architecture (3 boucles imbriquées) :
 *
 *   Position Y ──PID──► accélération verticale désirée ──► F (compensée du tilt)
 *   Position X ──PID──► accélération horizontale désirée ──► angle theta_ref
 *                                          theta_ref ──PID──► couple M
 *
 * Détails :
 *  - Boucle altitude : a_y = PID_y(y_ref - y); F = m*(g + a_y)/cos(theta).
 *    La division par cos(theta) compense la perte de portance quand le quad
 *    est incliné.
 *  - Boucle position X : a_x = PID_x(x_ref - x). L'inclinaison nécessaire pour
 *    produire cette accélération est theta_ref = -atan2(a_x, g + a_y), bornée
 *    à ±tiltMax pour rester dans le domaine linéaire / sûr.
 *  - Boucle attitude : M = Iz * PID_theta(theta_ref - theta), terme D calculé
 *    sur la vitesse angulaire mesurée (omega) pour éviter le derivative kick.
 *
 * Les boucles internes (attitude) doivent être nettement plus rapides que les
 * externes (position) : c'est le principe de la cascade.
 */
(function (QUAD) {
  'use strict';

  const { PID, BaseController } = QUAD;

  class CascadePID extends BaseController {
    constructor(model) {
      super(model);
      this.name = 'Cascade PID';

      // Gains par défaut (réglables via sliders)
      this.pidX = new PID(1.1, 0.5, 1.7, { iMax: 6 });    // position horizontale
      this.pidY = new PID(6.0, 3.0, 4.0, { iMax: 8 });    // altitude
      this.pidTh = new PID(14.0, 0.0, 3.5, { iMax: 2 });  // attitude (tangage)

      this.tiltMax = 0.6; // inclinaison max autorisée (rad, ~34°)
    }

    reset() {
      this.pidX.reset();
      this.pidY.reset();
      this.pidTh.reset();
    }

    compute(state, ref, dt) {
      const { m, g, Iz } = this.model.p;

      // --- Boucle altitude (Y) ---
      const ay = this.pidY.update(ref.y - state.y, dt, state.vy);
      // F compensée de l'inclinaison ; on borne cos pour éviter la singularité
      const cth = Math.max(0.4, Math.cos(state.th));
      let F = (m * (g + ay)) / cth;
      if (F < 0) F = 0;

      // --- Boucle position (X) ---
      const ax = this.pidX.update(ref.x - state.x, dt, state.vx);
      // inclinaison désirée pour produire ax (signe : x'' = -(F/m) sin(theta))
      let thRef = -Math.atan2(ax, g + ay);
      thRef = Math.max(-this.tiltMax, Math.min(this.tiltMax, thRef));

      // --- Boucle attitude (theta) ---
      const M = Iz * this.pidTh.update(thRef - state.th, dt, state.om);

      // exposé pour la télémétrie
      this._thRef = thRef;
      return { F, M, thRef };
    }

    getParamSpec() {
      const g = (p) => [p.kp, p.ki, p.kd];
      return [
        {
          label: 'Altitude (Y)',
          params: [
            { key: 'y_kp', label: 'Kp', min: 0, max: 20, step: 0.1, value: this.pidY.kp },
            { key: 'y_ki', label: 'Ki', min: 0, max: 15, step: 0.1, value: this.pidY.ki },
            { key: 'y_kd', label: 'Kd', min: 0, max: 15, step: 0.1, value: this.pidY.kd },
          ],
        },
        {
          label: 'Position (X)',
          params: [
            { key: 'x_kp', label: 'Kp', min: 0, max: 5, step: 0.05, value: this.pidX.kp },
            { key: 'x_ki', label: 'Ki', min: 0, max: 3, step: 0.05, value: this.pidX.ki },
            { key: 'x_kd', label: 'Kd', min: 0, max: 5, step: 0.05, value: this.pidX.kd },
          ],
        },
        {
          label: 'Attitude (θ)',
          params: [
            { key: 'th_kp', label: 'Kp', min: 0, max: 40, step: 0.1, value: this.pidTh.kp },
            { key: 'th_ki', label: 'Ki', min: 0, max: 10, step: 0.1, value: this.pidTh.ki },
            { key: 'th_kd', label: 'Kd', min: 0, max: 12, step: 0.1, value: this.pidTh.kd },
          ],
        },
        {
          label: 'Limites',
          params: [
            { key: 'tiltMax', label: 'θ max (rad)', min: 0.1, max: 1.2, step: 0.01, value: this.tiltMax },
          ],
        },
      ];
      void g;
    }

    setParams(o) {
      const num = (v, d) => (v == null || isNaN(v) ? d : v);
      this.pidY.setGains(num(o.y_kp, this.pidY.kp), num(o.y_ki, this.pidY.ki), num(o.y_kd, this.pidY.kd));
      this.pidX.setGains(num(o.x_kp, this.pidX.kp), num(o.x_ki, this.pidX.ki), num(o.x_kd, this.pidX.kd));
      this.pidTh.setGains(num(o.th_kp, this.pidTh.kp), num(o.th_ki, this.pidTh.ki), num(o.th_kd, this.pidTh.kd));
      if (o.tiltMax != null) this.tiltMax = o.tiltMax;
    }
  }

  QUAD.CascadePID = CascadePID;
})(window.QUAD || (window.QUAD = {}));
