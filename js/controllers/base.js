/*
 * controllers/base.js — Interface commune à tous les contrôleurs.
 *
 * Un contrôleur reçoit l'état complet du quad et une consigne, et renvoie
 * une commande { F, M } (poussée totale + couple). Le simulateur se charge
 * ensuite de convertir en poussées rotors et d'appliquer la saturation.
 *
 * Pour ajouter un contrôleur, hériter de BaseController et implémenter :
 *   - compute(state, ref, dt) -> { F, M }
 *   - (optionnel) reset()
 *   - (optionnel) getParamSpec() -> descripteur des sliders à afficher
 *   - (optionnel) setParams(obj) pour appliquer les valeurs des sliders
 *
 * getParamSpec() renvoie un tableau de groupes :
 *   [{ label, params: [{ key, label, min, max, step, value }] }]
 */
(function (QUAD) {
  'use strict';

  class BaseController {
    /**
     * @param {Quadrotor2D} model  référence au modèle (masse, inertie, g…)
     */
    constructor(model) {
      this.model = model;
      this.name = 'Base';
    }

    /** Réinitialise l'état interne (intégrateurs, etc.). */
    reset() {}

    /**
     * @param {{x,vx,y,vy,th,om}} state état courant
     * @param {{x,y}} ref  consigne de position
     * @param {number} dt  pas de temps
     * @returns {{F:number, M:number}} commande
     */
    compute(/* state, ref, dt */) {
      return { F: 0, M: 0 };
    }

    /** Descripteur des paramètres réglables (pour l'UI). */
    getParamSpec() {
      return [];
    }

    /** Applique un objet { key: value } issu des sliders. */
    setParams(/* obj */) {}
  }

  QUAD.BaseController = BaseController;
})(window.QUAD || (window.QUAD = {}));
