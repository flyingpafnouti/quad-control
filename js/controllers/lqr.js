/*
 * controllers/lqr.js — Régulateur linéaire quadratique (LQR).
 *
 * On linéarise le modèle autour du vol stationnaire (theta = 0, F = m*g, M = 0).
 * État réduit  : X = [x, vx, y, vy, theta, omega]
 * Entrées      : U = [dF, M]  avec dF = F - m*g  (écart de poussée au hover)
 *
 * Modèle linéarisé (petits angles, F ≈ m*g) :
 *   x'      = vx
 *   vx'     = -g * theta          (car x'' = -(F/m) sin θ ≈ -g θ)
 *   y'      = vy
 *   vy'     = dF / m              (car y'' = (F/m) cos θ - g ≈ dF/m)
 *   theta'  = omega
 *   omega'  = M / Iz
 *
 *   A = ∂X'/∂X ,  B = ∂X'/∂U
 *
 * Le gain K est obtenu en résolvant l'équation de Riccati algébrique.
 * On la résout par itération sur la version DISCRÉTISÉE du système
 * (Ad = I + A·h, Bd = B·h) avec des poids Qd = Q·h, Rd = R·h, ce qui
 * fait tendre le gain discret vers le gain LQR continu quand h est petit.
 *
 *   u = -K · (X - X_ref)      avec X_ref = [x_ref, 0, y_ref, 0, 0, 0]
 *   F = m*g + u[0]  ,  M = u[1]
 *
 * K est recalculé à chaque changement de poids (sliders) ou de paramètres
 * physiques du modèle.
 */
(function (QUAD) {
  'use strict';

  const { mat, BaseController } = QUAD;

  class LQR extends BaseController {
    constructor(model) {
      super(model);
      this.name = 'LQR';

      // Poids diagonaux Q (état) et R (commande), réglables par sliders.
      // Q pénalise : [x, vx, y, vy, theta, omega]
      this.q = { x: 12, vx: 4, y: 12, vy: 4, th: 6, om: 0.4 };
      // R pénalise : [dF, M]. R élevé => commande plus douce.
      this.r = { F: 0.5, M: 0.05 };

      this.K = null;
      this.recompute();
    }

    reset() {
      // Le LQR est un retour d'état statique : rien à réinitialiser,
      // mais on s'assure que le gain est à jour.
      if (!this.K) this.recompute();
    }

    /** Construit A et B linéarisés à partir des paramètres du modèle. */
    _buildAB() {
      const { m, Iz, g } = this.model.p;
      const A = mat.zeros(6, 6);
      // indices: 0:x 1:vx 2:y 3:vy 4:th 5:om
      A[0][1] = 1;       // x'  = vx
      A[1][4] = -g;      // vx' = -g theta
      A[2][3] = 1;       // y'  = vy
      A[4][5] = 1;       // th' = om
      // vy' et om' dépendent des entrées (voir B)

      const B = mat.zeros(6, 2);
      B[3][0] = 1 / m;   // vy' = dF/m
      B[5][1] = 1 / Iz;  // om' = M/Iz
      return { A, B };
    }

    /**
     * Résout le LQR discret par itération de Riccati et stocke K (2x6).
     * @param {number} h  pas de discrétisation (s)
     * @param {number} maxIter
     */
    recompute(h = 0.01, maxIter = 5000) {
      const { A, B } = this._buildAB();
      const I6 = mat.eye(6);
      const Ad = mat.add(I6, mat.scale(A, h));
      const Bd = mat.scale(B, h);
      const Q = mat.scale(mat.diag([this.q.x, this.q.vx, this.q.y, this.q.vy, this.q.th, this.q.om]), h);
      const R = mat.scale(mat.diag([this.r.F, this.r.M]), h);

      const AdT = mat.T(Ad);
      const BdT = mat.T(Bd);

      let P = mat.clone(Q);
      let K = null;
      for (let it = 0; it < maxIter; it++) {
        // S = R + Bd' P Bd            (2x2)
        const S = mat.add(R, mat.mul(mat.mul(BdT, P), Bd));
        const Sinv = mat.inv(S);
        // K = S^{-1} Bd' P Ad         (2x6)
        K = mat.mul(Sinv, mat.mul(mat.mul(BdT, P), Ad));
        // Pnew = Q + Ad'PAd - (Ad'PBd) K
        const AdTP = mat.mul(AdT, P);
        const Pnew = mat.add(
          Q,
          mat.sub(mat.mul(AdTP, Ad), mat.mul(mat.mul(AdTP, Bd), K))
        );
        if (mat.diffNorm(Pnew, P) < 1e-10) {
          P = Pnew;
          break;
        }
        P = Pnew;
      }
      this.K = K;
      return K;
    }

    compute(state, ref, dt) {
      const { m, g } = this.model.p;
      if (!this.K) this.recompute();

      // écart à l'équilibre désiré
      const dx = [
        state.x - ref.x,
        state.vx,
        state.y - ref.y,
        state.vy,
        state.th,
        state.om,
      ];
      const u = mat.mulVec(this.K, dx); // u = K·dx
      // u = -K·dx  => on inverse le signe
      const dF = -u[0];
      const M = -u[1];

      let F = m * g + dF;
      if (F < 0) F = 0;
      void dt;
      return { F, M, thRef: 0 };
    }

    getParamSpec() {
      return [
        {
          label: 'Q — pénalité état',
          params: [
            { key: 'q_x', label: 'x', min: 0.1, max: 60, step: 0.1, value: this.q.x },
            { key: 'q_vx', label: 'vx', min: 0, max: 30, step: 0.1, value: this.q.vx },
            { key: 'q_y', label: 'y', min: 0.1, max: 60, step: 0.1, value: this.q.y },
            { key: 'q_vy', label: 'vy', min: 0, max: 30, step: 0.1, value: this.q.vy },
            { key: 'q_th', label: 'θ', min: 0, max: 40, step: 0.1, value: this.q.th },
            { key: 'q_om', label: 'ω', min: 0, max: 10, step: 0.05, value: this.q.om },
          ],
        },
        {
          label: 'R — pénalité commande',
          params: [
            { key: 'r_F', label: 'F', min: 0.01, max: 5, step: 0.01, value: this.r.F },
            { key: 'r_M', label: 'M', min: 0.005, max: 2, step: 0.005, value: this.r.M },
          ],
        },
      ];
    }

    setParams(o) {
      const set = (obj, k, v) => { if (v != null && !isNaN(v)) obj[k] = v; };
      set(this.q, 'x', o.q_x); set(this.q, 'vx', o.q_vx);
      set(this.q, 'y', o.q_y); set(this.q, 'vy', o.q_vy);
      set(this.q, 'th', o.q_th); set(this.q, 'om', o.q_om);
      set(this.r, 'F', o.r_F); set(this.r, 'M', o.r_M);
      this.recompute(); // le gain dépend des poids -> recalcul
    }
  }

  QUAD.LQR = LQR;
})(window.QUAD || (window.QUAD = {}));
