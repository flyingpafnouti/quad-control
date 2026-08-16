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
     * @param {{min:number,max:number}|null} sat  bornes de saturation de
     *        l'actionneur en aval. Active l'anti-windup par intégration
     *        conditionnelle (clamping) : l'intégrateur est GELÉ tant que la
     *        sortie sature et que l'erreur pousse encore dans le même sens.
     *        Empêche l'emballement responsable des longs dépassements.
     * @returns {number} sortie du régulateur (bornée à sat si fourni)
     */
    update(err, dt, measRate = null, sat = null) {
      // Terme dérivé (indépendant de l'intégrale)
      let d;
      if (measRate != null) {
        d = -this.kd * measRate;
      } else {
        const de = this.prevErr == null ? 0 : (err - this.prevErr) / dt;
        d = this.kd * de;
      }
      this.prevErr = err;

      // Intégration tentative
      const prevInteg = this.integ;
      this.integ += this.ki * err * dt;
      const iLim = this.iMax;
      if (this.integ > iLim) this.integ = iLim;
      else if (this.integ < -iLim) this.integ = -iLim;

      let out = this.kp * err + this.integ + d;

      // Anti-windup : si la sortie sature et que l'on pousse dans le même sens,
      // on annule l'incrément d'intégrale de ce pas (clamping).
      if (sat) {
        if ((out > sat.max && err > 0) || (out < sat.min && err < 0)) {
          this.integ = prevInteg;
          out = this.kp * err + this.integ + d;
        }
        if (out > sat.max) out = sat.max;
        else if (out < sat.min) out = sat.min;
      }
      return out;
    }
  }

  QUAD.PID = PID;
})(window.QUAD || (window.QUAD = {}));
