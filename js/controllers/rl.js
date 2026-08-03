/*
 * controllers/rl.js — Politique apprise par renforcement (RL), exécutée en inférence.
 *
 * La politique est un petit perceptron multicouche (MLP) entraîné HORS-LIGNE
 * (PPO, cf. training/train_rl.py) sur un environnement qui réplique exactement
 * la dynamique de js/dynamics.js. Ici on ne fait que le "forward pass" :
 *
 *   obs   = [dx, vx, dy, vy, θ, ω] / obsScale        (dx = x - x_ref, dy = y - y_ref)
 *   h     = obs ; pour chaque couche : h = act(W·h + b)   (tanh, sauf la sortie : linéaire)
 *   a     = clip(sortie, -1, 1)          (2 actions dans [-1, 1])
 *   F     = m·g + a[0]·dfScale           (poussée totale, bornée >= 0)
 *   M     = a[1]·mScale                  (couple)
 *
 * Les échelles (obsScale, dfScale, mScale) sont embarquées dans rl_weights.js
 * et DOIVENT être identiques à celles utilisées à l'entraînement (quad_env.py).
 *
 * Si les poids ne sont pas chargés (entraînement non lancé), le contrôleur se
 * rabat sur un vol stationnaire neutre (F = m·g, M = 0).
 */
(function (QUAD) {
  'use strict';

  const { BaseController } = QUAD;

  class RLController extends BaseController {
    constructor(model) {
      super(model);
      this.name = 'RL (policy)';
      this._loadWeights();
    }

    _loadWeights() {
      const w = QUAD.RL_WEIGHTS || null;
      this.net = w;
      const meta = (w && w.meta) || {};
      this.obsScale = meta.obsScale || [3, 3, 3, 3, 0.6, 3];
      this.dfScale = meta.dfScale != null ? meta.dfScale : 6.0;
      this.mScale = meta.mScale != null ? meta.mScale : 1.5;
    }

    reset() {
      // Politique réactive sans état interne : rien à réinitialiser.
      // (Recharge les poids au cas où rl_weights.js aurait été (re)chargé après coup.)
      if (!this.net) this._loadWeights();
    }

    /** Forward pass du MLP : renvoie la sortie brute (avant clip). */
    _forward(x) {
      let a = x;
      const layers = this.net.layers;
      for (let i = 0; i < layers.length; i++) {
        const { W, b, act } = layers[i];
        const out = new Array(W.length);
        for (let r = 0; r < W.length; r++) {
          let s = b[r];
          const row = W[r];
          for (let c = 0; c < a.length; c++) s += row[c] * a[c];
          out[r] = act === 'tanh' ? Math.tanh(s) : s;
        }
        a = out;
      }
      return a;
    }

    compute(state, ref, dt) {
      const { m, g } = this.model.p;
      if (!this.net) {
        // Poids absents : hover neutre (lance l'entraînement pour activer la politique).
        return { F: m * g, M: 0, thRef: 0 };
      }

      const os = this.obsScale;
      const obs = [
        (state.x - ref.x) / os[0],
        state.vx / os[1],
        (state.y - ref.y) / os[2],
        state.vy / os[3],
        state.th / os[4],
        state.om / os[5],
      ];

      const a = this._forward(obs);
      const a0 = Math.max(-1, Math.min(1, a[0]));
      const a1 = Math.max(-1, Math.min(1, a[1]));

      let F = m * g + a0 * this.dfScale;
      if (F < 0) F = 0;
      const M = a1 * this.mScale;

      void dt;
      return { F, M, thRef: 0 };
    }

    // La politique n'a pas de gains réglables ; on expose les échelles d'action
    // en lecture/ajustement fin (par défaut = celles de l'entraînement).
    getParamSpec() {
      return [
        {
          label: 'Politique RL (échelles action)',
          params: [
            { key: 'dfScale', label: 'dF max', min: 1, max: 20, step: 0.5, value: this.dfScale },
            { key: 'mScale', label: 'M max', min: 0.1, max: 5, step: 0.1, value: this.mScale },
          ],
        },
      ];
    }

    setParams(o) {
      if (o.dfScale != null && !isNaN(o.dfScale)) this.dfScale = o.dfScale;
      if (o.mScale != null && !isNaN(o.mScale)) this.mScale = o.mScale;
    }
  }

  QUAD.RLController = RLController;
})(window.QUAD || (window.QUAD = {}));
