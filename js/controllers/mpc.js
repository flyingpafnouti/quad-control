/*
 * controllers/mpc.js — Commande prédictive linéaire (MPC à horizon fuyant).
 *
 * Idée générale
 * -------------
 * À chaque pas, le MPC prédit la trajectoire de l'état sur un horizon de N pas
 * (de durée dt chacun) à l'aide d'un MODÈLE INTERNE linéarisé autour du vol
 * stationnaire, puis choisit la séquence de commandes qui minimise le coût
 *
 *   J = Σ_{k=1..N} (X_k - X_ref)' Q (X_k - X_ref) + Σ_{k=0..N-1} u_k' R u_k
 *       + (terminal) (X_N - X_ref)' (term·Q) (X_N - X_ref)
 *
 * Seule la PREMIÈRE commande u_0 est appliquée ; tout est recalculé au pas
 * suivant (principe de l'horizon fuyant / receding horizon).
 *
 * Modèle interne (réduit, comme le LQR)
 *   État   : X = [x, vx, y, vy, theta, omega]
 *   Entrée : u = [dF, M]   avec dF = F - m̂·g   (écart de poussée au hover)
 *   x'=vx ; vx'=-g·θ ; y'=vy ; vy'=dF/m̂ ; θ'=ω ; ω'=M/Î
 *
 * Model mismatch (réalisme)
 * -------------------------
 * Le contrôleur ne connaît PAS parfaitement le drone : il raisonne avec
 *   m̂ = m · massMismatch     et     Î = Iz · inertiaMismatch
 * qui diffèrent légèrement des vraies valeurs du modèle physique. Comme ce
 * MPC n'a pas d'intégrateur, un mismatch (ou du vent) laisse une petite
 * erreur statique — exactement ce qu'on observe sur un vrai système.
 *
 * Résolution condensée
 * --------------------
 * On empile la prédiction :  X̂ = Sx·x0 + Su·U   (U = [u_0;…;u_{N-1}])
 * Le coût quadratique non contraint se minimise en forme fermée :
 *   H·U = -Su'·Q̄·(Sx·x0 - X_refStack)   →   U* = -(H⁻¹ Su' Q̄)(Sx·x0 - X_refStack)
 * On ne garde que la 1ʳᵉ commande. Comme le gain ne dépend que du modèle, des
 * poids et de l'horizon, on précalcule tout hors ligne dans recompute() et on
 * réduit le pas temps réel à un simple produit gain × état :
 *   u_0 = -Kx·x0 + Kref_x·x_ref + Kref_y·y_ref
 * (les contraintes de poussée sont assurées en aval par la saturation rotors.)
 */
(function (QUAD) {
  'use strict';

  const { mat, BaseController } = QUAD;

  class MPC extends BaseController {
    constructor(model) {
      super(model);
      this.name = 'MPC';

      // --- Réglages d'horizon ---
      this.N = 20;        // nombre de pas de prédiction
      this.dtMpc = 0.03;  // pas de prédiction (s) → horizon = N·dtMpc

      // --- Poids diagonaux Q (état) et R (commande) ---
      this.q = { x: 14, vx: 4, y: 14, vy: 4, th: 8, om: 0.4 };
      this.r = { F: 0.4, M: 0.04 };
      this.term = 6;      // multiplicateur du poids terminal (sur Q)

      // --- Model mismatch (le MPC croit à un drone légèrement différent) ---
      // Léger biais par défaut (5 % de masse, 5 % d'inertie) → petite erreur
      // statique réaliste ; réglable jusqu'à ±30 % via les sliders.
      this.mm = { mass: 1.05, inertia: 0.95 };

      // Gains précalculés
      this.Kx = null;      // 2 x 6
      this.Kref_x = null;  // 2
      this.Kref_y = null;  // 2

      this.recompute();
    }

    reset() {
      if (!this.Kx) this.recompute();
    }

    /** Masse / inertie « crues » utilisées par le contrôleur (mismatch). */
    _hatParams() {
      const { m, Iz, g } = this.model.p;
      return { mHat: m * this.mm.mass, IHat: Iz * this.mm.inertia, g };
    }

    /** Modèle linéarisé continu (A 6x6, B 6x2) du drone « cru ». */
    _buildAB() {
      const { mHat, IHat, g } = this._hatParams();
      const A = mat.zeros(6, 6);
      A[0][1] = 1;    // x'  = vx
      A[1][4] = -g;   // vx' = -g·θ
      A[2][3] = 1;    // y'  = vy
      A[4][5] = 1;    // θ'  = ω
      const B = mat.zeros(6, 2);
      B[3][0] = 1 / mHat; // vy' = dF/m̂
      B[5][1] = 1 / IHat; // ω'  = M/Î
      return { A, B };
    }

    /** Écrit le bloc `blk` (r×c) à la position (bi,bj) dans la grande matrice. */
    _setBlock(big, bi, bj, blk) {
      const r = blk.length, c = blk[0].length;
      for (let i = 0; i < r; i++)
        for (let j = 0; j < c; j++) big[bi + i][bj + j] = blk[i][j];
    }

    /**
     * Précalcule les gains du MPC (Kx, Kref_x, Kref_y). À rappeler dès qu'un
     * paramètre (horizon, poids, mismatch, physique) change.
     */
    recompute() {
      const N = Math.max(2, Math.round(this.N));
      const h = this.dtMpc;
      const { A, B } = this._buildAB();

      // Discrétisation d'Euler : Ad = I + A·h, Bd = B·h
      const Ad = mat.add(mat.eye(6), mat.scale(A, h));
      const Bd = mat.scale(B, h);

      // Puissances de Ad : Adp[k] = Ad^k, k = 0..N
      const Adp = [mat.eye(6)];
      for (let k = 1; k <= N; k++) Adp[k] = mat.mul(Ad, Adp[k - 1]);

      // BdBlocks[k] = Ad^k · Bd (blocs de Su), k = 0..N-1
      const BdBlocks = [];
      for (let k = 0; k < N; k++) BdBlocks[k] = mat.mul(Adp[k], Bd);

      // Sx (6N x 6) : bloc-ligne i = Ad^{i+1}
      const Sx = mat.zeros(6 * N, 6);
      for (let i = 0; i < N; i++) this._setBlock(Sx, 6 * i, 0, Adp[i + 1]);

      // Su (6N x 2N) : bloc(i,j) = Ad^{i-j}·Bd pour i >= j, sinon 0
      const Su = mat.zeros(6 * N, 2 * N);
      for (let i = 0; i < N; i++)
        for (let j = 0; j <= i; j++)
          this._setBlock(Su, 6 * i, 2 * j, BdBlocks[i - j]);

      // Diagonales des poids : Q̄ (6N) et R̄ (2N)
      const qBlk = [this.q.x, this.q.vx, this.q.y, this.q.vy, this.q.th, this.q.om];
      const Qdiag = new Array(6 * N);
      for (let i = 0; i < N; i++) {
        const w = i === N - 1 ? this.term : 1; // poids terminal sur le dernier bloc
        for (let s = 0; s < 6; s++) Qdiag[6 * i + s] = qBlk[s] * w;
      }
      const Rdiag = new Array(2 * N);
      for (let j = 0; j < N; j++) {
        Rdiag[2 * j] = this.r.F;
        Rdiag[2 * j + 1] = this.r.M;
      }

      // M2 = Su'·Q̄   (2N x 6N)   — Q̄ diagonale ⇒ mise à l'échelle par colonne
      const SuT = mat.T(Su); // 2N x 6N
      const M2 = SuT.map((row) => row.map((v, k) => v * Qdiag[k]));

      // H = M2·Su + R̄   (2N x 2N)
      const H = mat.mul(M2, Su);
      for (let j = 0; j < 2 * N; j++) H[j][j] += Rdiag[j];

      // On n'a besoin que des 2 premières lignes de U* ⇒ des 2 lignes de H⁻¹
      const Hinv = mat.inv(H);
      const Hrow = [Hinv[0], Hinv[1]];            // 2 x 2N
      const Krow = mat.mul(Hrow, M2);             // 2 x 6N  (= (H⁻¹ Su' Q̄)[0:2])

      // Repli sur l'état / la consigne :
      //   u_0 = -Krow·(Sx·x0 - X_refStack) = -(Krow·Sx)·x0 + Krow·X_refStack
      this.Kx = mat.mul(Krow, Sx);                // 2 x 6
      // X_refStack ne contient x_ref (offset 0) et y_ref (offset 2) répétés
      this.Kref_x = [0, 0];
      this.Kref_y = [0, 0];
      for (let i = 0; i < N; i++) {
        for (let rIdx = 0; rIdx < 2; rIdx++) {
          this.Kref_x[rIdx] += Krow[rIdx][6 * i + 0];
          this.Kref_y[rIdx] += Krow[rIdx][6 * i + 2];
        }
      }
    }

    compute(state, ref, dt) {
      if (!this.Kx) this.recompute();
      const { mHat, g } = this._hatParams();

      const x0 = [state.x, state.vx, state.y, state.vy, state.th, state.om];

      // u_0 = -Kx·x0 + Kref_x·x_ref + Kref_y·y_ref
      const dF = -this._dot(this.Kx[0], x0) + this.Kref_x[0] * ref.x + this.Kref_y[0] * ref.y;
      const M  = -this._dot(this.Kx[1], x0) + this.Kref_x[1] * ref.x + this.Kref_y[1] * ref.y;

      // Le MPC applique la poussée de hover de son modèle CRU (⇒ biais réaliste)
      let F = mHat * g + dF;
      if (F < 0) F = 0;
      void dt;
      return { F, M, thRef: 0 };
    }

    _dot(row, v) {
      let s = 0;
      for (let j = 0; j < row.length; j++) s += row[j] * v[j];
      return s;
    }

    getParamSpec() {
      return [
        {
          label: 'Horizon de prédiction',
          params: [
            { key: 'N', label: 'N (pas)', min: 3, max: 40, step: 1, value: this.N },
            { key: 'dt', label: 'dt (s)', min: 0.01, max: 0.08, step: 0.005, value: this.dtMpc },
          ],
        },
        {
          label: 'Q — pénalité état',
          params: [
            { key: 'q_x', label: 'x', min: 0.1, max: 60, step: 0.1, value: this.q.x },
            { key: 'q_vx', label: 'vx', min: 0, max: 30, step: 0.1, value: this.q.vx },
            { key: 'q_y', label: 'y', min: 0.1, max: 60, step: 0.1, value: this.q.y },
            { key: 'q_vy', label: 'vy', min: 0, max: 30, step: 0.1, value: this.q.vy },
            { key: 'q_th', label: 'θ', min: 0, max: 40, step: 0.1, value: this.q.th },
            { key: 'q_om', label: 'ω', min: 0, max: 10, step: 0.05, value: this.q.om },
            { key: 'term', label: 'terminal ×', min: 1, max: 40, step: 0.5, value: this.term },
          ],
        },
        {
          label: 'R — pénalité commande',
          params: [
            { key: 'r_F', label: 'F', min: 0.01, max: 5, step: 0.01, value: this.r.F },
            { key: 'r_M', label: 'M', min: 0.005, max: 2, step: 0.005, value: this.r.M },
          ],
        },
        {
          label: 'Model mismatch (réalisme)',
          params: [
            { key: 'mm_m', label: 'masse ×', min: 0.7, max: 1.3, step: 0.01, value: this.mm.mass },
            { key: 'mm_I', label: 'inertie ×', min: 0.5, max: 1.5, step: 0.01, value: this.mm.inertia },
          ],
        },
      ];
    }

    setParams(o) {
      const set = (obj, k, v) => { if (v != null && !isNaN(v)) obj[k] = v; };
      if (o.N != null && !isNaN(o.N)) this.N = Math.round(o.N);
      set(this, 'dtMpc', o.dt);
      set(this.q, 'x', o.q_x); set(this.q, 'vx', o.q_vx);
      set(this.q, 'y', o.q_y); set(this.q, 'vy', o.q_vy);
      set(this.q, 'th', o.q_th); set(this.q, 'om', o.q_om);
      set(this, 'term', o.term);
      set(this.r, 'F', o.r_F); set(this.r, 'M', o.r_M);
      set(this.mm, 'mass', o.mm_m); set(this.mm, 'inertia', o.mm_I);
      this.recompute(); // gains dépendants des paramètres → recalcul
    }
  }

  QUAD.MPC = MPC;
})(window.QUAD || (window.QUAD = {}));
