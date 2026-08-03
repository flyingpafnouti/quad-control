/*
 * math.js — Utilitaires d'algèbre linéaire (matrices denses).
 * Les matrices sont des tableaux de tableaux (row-major) : m[i][j].
 * Sert essentiellement au calcul du LQR (équation de Riccati).
 */
(function (QUAD) {
  'use strict';

  const mat = {
    /** Matrice nulle r x c */
    zeros(r, c) {
      const M = new Array(r);
      for (let i = 0; i < r; i++) M[i] = new Array(c).fill(0);
      return M;
    },

    /** Matrice identité n x n */
    eye(n) {
      const M = mat.zeros(n, n);
      for (let i = 0; i < n; i++) M[i][i] = 1;
      return M;
    },

    /** Copie profonde */
    clone(A) {
      return A.map((row) => row.slice());
    },

    /** Diagonale à partir d'un vecteur */
    diag(v) {
      const n = v.length;
      const M = mat.zeros(n, n);
      for (let i = 0; i < n; i++) M[i][i] = v[i];
      return M;
    },

    /** Transposée */
    T(A) {
      const r = A.length, c = A[0].length;
      const M = mat.zeros(c, r);
      for (let i = 0; i < r; i++)
        for (let j = 0; j < c; j++) M[j][i] = A[i][j];
      return M;
    },

    /** Produit A (r x k) * B (k x c) */
    mul(A, B) {
      const r = A.length, k = A[0].length, c = B[0].length;
      const M = mat.zeros(r, c);
      for (let i = 0; i < r; i++) {
        for (let p = 0; p < k; p++) {
          const a = A[i][p];
          if (a === 0) continue;
          const Bp = B[p];
          const Mi = M[i];
          for (let j = 0; j < c; j++) Mi[j] += a * Bp[j];
        }
      }
      return M;
    },

    /** Somme A + B */
    add(A, B) {
      return A.map((row, i) => row.map((v, j) => v + B[i][j]));
    },

    /** Différence A - B */
    sub(A, B) {
      return A.map((row, i) => row.map((v, j) => v - B[i][j]));
    },

    /** Multiplication scalaire */
    scale(A, s) {
      return A.map((row) => row.map((v) => v * s));
    },

    /**
     * Inverse par élimination de Gauss-Jordan avec pivot partiel.
     * Suffisant pour les petites matrices bien conditionnées (ici 2x2).
     */
    inv(A) {
      const n = A.length;
      const M = A.map((row, i) =>
        row.concat(mat.eye(n)[i])
      ); // [A | I]
      for (let col = 0; col < n; col++) {
        // pivot partiel
        let piv = col;
        for (let r = col + 1; r < n; r++) {
          if (Math.abs(M[r][col]) > Math.abs(M[piv][col])) piv = r;
        }
        if (Math.abs(M[piv][col]) < 1e-12) {
          throw new Error('Matrice singulière lors de l\'inversion');
        }
        [M[col], M[piv]] = [M[piv], M[col]];
        const d = M[col][col];
        for (let j = 0; j < 2 * n; j++) M[col][j] /= d;
        for (let r = 0; r < n; r++) {
          if (r === col) continue;
          const f = M[r][col];
          if (f === 0) continue;
          for (let j = 0; j < 2 * n; j++) M[r][j] -= f * M[col][j];
        }
      }
      return M.map((row) => row.slice(n));
    },

    /** Norme de Frobenius de (A - B), pour tester une convergence */
    diffNorm(A, B) {
      let s = 0;
      for (let i = 0; i < A.length; i++)
        for (let j = 0; j < A[0].length; j++) {
          const d = A[i][j] - B[i][j];
          s += d * d;
        }
      return Math.sqrt(s);
    },

    /** Produit matrice (r x c) * vecteur (c) -> vecteur (r) */
    mulVec(A, v) {
      return A.map((row) => row.reduce((s, a, j) => s + a * v[j], 0));
    },
  };

  QUAD.mat = mat;
})(window.QUAD || (window.QUAD = {}));
