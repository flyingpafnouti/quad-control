/*
 * pid.js — Bloc PID réutilisable.
 *
 * Caractéristiques :
 *   - Terme intégral avec anti-windup (saturation de l'intégrale).
 *   - Terme dérivé calculable soit par différence finie de l'erreur,
 *     soit à partir d'une dérivée MESURÉE de la grandeur (évite le
 *     "derivative kick" sur les échelons de consigne).
 */
(function (QUAD) {
  'use strict';

  class PID {
    constructor(kp = 0, ki = 0, kd = 0, opts = {}) {
      this.kp = kp;
      this.ki = ki;
      this.kd = kd;
      this.iMax = opts.iMax != null ? opts.iMax : Infinity; // borne du terme I
      this.reset();
    }

    setGains(kp, ki, kd) {
      this.kp = kp;
      this.ki = ki;
      this.kd = kd;
    }

    reset() {
      this.integ = 0;
      this.prevErr = null;
    }

    /**
     * @param {number} err  erreur = consigne - mesure
     * @param {number} dt   pas de temps (s)
     * @param {number|null} measRate  dérivée MESURÉE de la grandeur (ex: vitesse).
     *        Si fournie, le terme D vaut -kd*measRate (pas de kick).
     * @returns {number} sortie du régulateur
     */
    update(err, dt, measRate = null) {
      // Terme intégral avec anti-windup par saturation
      this.integ += this.ki * err * dt;
      const iLim = this.iMax;
      if (this.integ > iLim) this.integ = iLim;
      else if (this.integ < -iLim) this.integ = -iLim;

      // Terme dérivé
      let d;
      if (measRate != null) {
        d = -this.kd * measRate;
      } else {
        const de = this.prevErr == null ? 0 : (err - this.prevErr) / dt;
        d = this.kd * de;
      }
      this.prevErr = err;

      return this.kp * err + this.integ + d;
    }
  }

  QUAD.PID = PID;
})(window.QUAD || (window.QUAD = {}));
