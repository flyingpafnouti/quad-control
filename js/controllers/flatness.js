/*
 * controllers/flatness.js — Contrôleur par platitude différentielle.
 *
 * Le quadrirotor planaire est différentiellement PLAT, de sorties plates
 * (x, y) : connaissant l'accélération désirée du centre de masse, on inverse
 * EXACTEMENT la dynamique pour obtenir la poussée et l'inclinaison requises —
 * sans aucune approximation petits-angles. Le contrôleur reste donc valide à
 * *tout* angle (jusqu'au vol inversé), là où LQR / MPC linéaire crashent
 * au-delà de ~40°. C'est ce qui en fait le contrôleur le plus agressif
 * exploitable ici, à un coût de calcul quasi nul (formes closes).
 *
 * Structure en cascade (feedback linearization) :
 *
 *   Boucle externe en cascade (position → vitesse → accélération) avec profil
 *   de vitesse TEMPS-OPTIMAL (« sqrt-controller », cf. ArduPilot) :
 *     v_sp = clamp( sqrtController(p_ref - p, kp, aTraj), ±v_max )
 *     a_d  = kd·(v_sp - v)
 *   La consigne de vitesse suit l'enveloppe de décélération v = √(2·aTraj·|e|) :
 *   le drone freine exactement à l'accélération maximale aTraj pour arriver à
 *   vitesse nulle EN TEMPS MINIMAL et SANS overshoot (là où l'ancienne loi
 *   linéaire v_sp = kp·e freinait trop mollement → longue traîne + dépassement).
 *   Près de la cible (|e| < aTraj/kp²) on repasse en linéaire (pente kp) pour
 *   éviter la pente infinie de la racine → pas de broutement. C'est le profil
 *   bang-bang du Niveau 3, réalisé en boucle fermée (donc robuste au vent).
 *
 *   Inversion par platitude (EXACTE) du vecteur de poussée monde
 *     (ax_d, ay_d + g) :
 *     F   = m·‖(ax_d, ay_d + g)‖                (poussée requise, ≥ 0)
 *     θ_d = atan2(-ax_d, ay_d + g)              (inclinaison requise, tout angle)
 *     — cohérent avec la dynamique x''=-(F/m)sinθ, y''=(F/m)cosθ-g.
 *
 *   Saturation (priorité altitude) : on garde la poussée orientée vers le haut
 *   (composante verticale ≥ tvyMin·g) et on borne l'inclinaison à ±θ_max en
 *   limitant la composante horizontale à tvy·tan(θ_max). On ne « flippe » donc
 *   jamais (pas de vol inversé qui ferait chuter). Enfin on borne la norme du
 *   vecteur d'accélération à a_max = thrustLimit·g (⇒ F ≤ Tmax·m·g), ce qui
 *   respecte la butée des rotors.
 *
 *   Boucle interne (attitude θ → moment M), PD raide (séparation d'échelle) :
 *     M = Iz·[ kp_θ·wrap(θ_d - θ) - kd_θ·ω ]
 *
 * F et M sont ensuite convertis/saturés en poussées rotors par le mixeur.
 */
(function (QUAD) {
  'use strict';

  const { BaseController } = QUAD;

  /** Ramène un angle dans (-π, π]. */
  function wrapAngle(a) {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a <= -Math.PI) a += 2 * Math.PI;
    return a;
  }

  /**
   * « sqrt-controller » : consigne de vitesse temps-optimale pour un double
   * intégrateur (rejoindre l'erreur nulle à vitesse nulle en temps minimal).
   *   - loin de la cible : v = √(2·aLim·|e|)  (décélération au max aLim, bang-bang)
   *   - près de la cible (|e| < aLim/kp²) : v = kp·e  (linéaire, évite la pente
   *     infinie de la racine ⇒ pas de broutement).
   * Les deux branches se raccordent en valeur ET en pente (C¹).
   * @param {number} e    erreur de position (cible - position)
   * @param {number} kp   gain de la zone linéaire proche
   * @param {number} aLim accélération maximale du profil (m/s²)
   */
  function sqrtController(e, kp, aLim) {
    if (aLim <= 0 || kp <= 0) return kp * e;
    const linDist = aLim / (kp * kp);
    if (e > linDist) return Math.sqrt(2 * aLim * (e - linDist / 2));
    if (e < -linDist) return -Math.sqrt(2 * aLim * (-e - linDist / 2));
    return kp * e;
  }

  class Flatness extends BaseController {
    constructor(model) {
      super(model);
      this.name = 'Flatness';

      // Boucle position : profil de vitesse temps-optimal puis PD (vitesse→accél).
      //   aTraj : accélération max du profil (m/s²) — LEVIER D'AGRESSIVITÉ n°1.
      //   vMax  : vitesse de croisière maximale (m/s).
      //   kp    : gain de la zone linéaire proche cible (1/s).
      //   kd    : gain vitesse→accélération (1/s).
      //   ki    : action intégrale (rejette le vent constant) + anti-windup.
      this.pos = { aTraj: 6.0, vMax: 5.0, kp: 3.0, kd: 8.0, ki: 6.0 };
      this.iMaxAccel = 10.0;  // butée anti-windup sur la contribution intégrale (m/s²)
      this.iZone = 0.75;      // n'intègre que dans cette zone d'arrivée (m) — anti-windup
      this.ix = 0;            // intégrateurs d'erreur de position
      this.iy = 0;
      // Boucle attitude, volontairement rapide (≫ boucle position).
      this.att = { kpTh: 700, kdTh: 45 };
      // Limite de poussée du contrôleur, en multiples de g (a_max = tl·g).
      // Butée physique des rotors : Tmax·g (Tmax = 1.6 par défaut).
      this.thrustLimit = 1.6;
      // Inclinaison maximale autorisée (deg) — garantit qu'on ne flippe pas.
      this.thMaxDeg = 75;
      // Composante verticale minimale du vecteur poussée (× g) : garde
      // toujours de l'autorité (poussée non nulle) pour piloter l'attitude.
      this.tvyMin = 0.4;

      this.reset();
    }

    reset() {
      this.ix = 0;
      this.iy = 0;
    }

    compute(state, ref, dt) {
      const { m, Iz, g, Tmax } = this.model.p;
      const P = this.pos, A = this.att;

      // --- Boucle externe : profil de vitesse temps-optimal → PD → accél ---
      const vm = P.vMax;
      const clamp = (v) => (v > vm ? vm : v < -vm ? -vm : v);
      const ex = ref.x - state.x;
      const ey = ref.y - state.y;
      // Enveloppe de décélération temps-optimale (bang-bang), bornée à v_max.
      const vx_sp = clamp(sqrtController(ex, P.kp, P.aTraj));
      const vy_sp = clamp(sqrtController(ey, P.kp, P.aTraj));

      // Action intégrale avec double anti-windup :
      //  (1) on n'intègre que dans la ZONE D'ARRIVÉE (|erreur| < iZone), pour
      //      ne pas s'emballer pendant le transit agressif ;
      //  (2) on borne la contribution intégrale (ki·i) à ±iMaxAccel.
      const iCap = this.iMaxAccel;
      const clampI = (v) => (v > iCap ? iCap : v < -iCap ? -iCap : v);
      if (dt > 0) {
        if (Math.abs(ex) < this.iZone) this.ix = clampI(this.ix + P.ki * ex * dt);
        else this.ix = 0;
        if (Math.abs(ey) < this.iZone) this.iy = clampI(this.iy + P.ki * ey * dt);
        else this.iy = 0;
      }

      const ax_d = P.kd * (vx_sp - state.vx) + this.ix;
      const ay_d = P.kd * (vy_sp - state.vy) + this.iy;

      // --- Inversion par platitude : vecteur de poussée dans le repère monde ---
      let tvx = ax_d;
      // Priorité altitude : la poussée garde une composante vers le haut.
      let tvy = Math.max(ay_d + g, this.tvyMin * g);

      // Limite d'inclinaison : |θ_d| ≤ θ_max  ⇔  |tvx| ≤ tvy·tan(θ_max).
      const hMax = tvy * Math.tan((this.thMaxDeg * Math.PI) / 180);
      if (tvx > hMax) tvx = hMax;
      else if (tvx < -hMax) tvx = -hMax;

      // Saturation de la norme d'accélération (respecte la butée rotors).
      let amag = Math.hypot(tvx, tvy);
      const aMax = Math.min(this.thrustLimit, Tmax) * g;
      if (amag > aMax && amag > 1e-9) {
        const s = aMax / amag;
        tvx *= s; tvy *= s; amag = aMax;
      }

      const F = m * amag;                            // poussée requise (≥ 0)
      // θ_d exact, valide à tout angle (le scaling n'affecte pas l'atan2).
      const thRef = Math.atan2(-tvx, tvy);

      // --- Boucle interne : moment pour asservir θ → θ_d ---
      const eth = wrapAngle(thRef - state.th);
      const alpha = A.kpTh * eth - A.kdTh * state.om;
      const M = Iz * alpha;

      void dt;
      return { F, M, thRef };
    }

    getParamSpec() {
      return [
        {
          label: 'Boucle position (agressivité)',
          params: [
            { key: 'aTraj', label: 'a traj (m/s²)', min: 1, max: 16, step: 0.5, value: this.pos.aTraj },
            { key: 'vMax', label: 'v max (m/s)', min: 0.5, max: 12, step: 0.1, value: this.pos.vMax },
            { key: 'kp', label: 'kp (proche)', min: 0.5, max: 10, step: 0.1, value: this.pos.kp },
            { key: 'kd', label: 'kd v→a', min: 1, max: 25, step: 0.5, value: this.pos.kd },
            { key: 'ki', label: 'ki (anti-vent)', min: 0, max: 12, step: 0.1, value: this.pos.ki },
          ],
        },
        {
          label: 'Boucle attitude (rapidité)',
          params: [
            { key: 'kpTh', label: 'kp θ', min: 50, max: 2000, step: 10, value: this.att.kpTh },
            { key: 'kdTh', label: 'kd θ', min: 5, max: 200, step: 1, value: this.att.kdTh },
          ],
        },
        {
          label: 'Saturations',
          params: [
            { key: 'tl', label: 'poussée × g', min: 1.05, max: 1.6, step: 0.01, value: this.thrustLimit },
            { key: 'thmax', label: 'θ max (°)', min: 20, max: 89, step: 1, value: this.thMaxDeg },
          ],
        },
      ];
    }

    setParams(o) {
      const set = (obj, k, v) => { if (v != null && !isNaN(v)) obj[k] = v; };
      set(this.pos, 'aTraj', o.aTraj); set(this.pos, 'vMax', o.vMax);
      set(this.pos, 'kp', o.kp); set(this.pos, 'kd', o.kd); set(this.pos, 'ki', o.ki);
      set(this.att, 'kpTh', o.kpTh); set(this.att, 'kdTh', o.kdTh);
      set(this, 'thrustLimit', o.tl);
      set(this, 'thMaxDeg', o.thmax);
    }
  }

  QUAD.Flatness = Flatness;
})(window.QUAD || (window.QUAD = {}));
