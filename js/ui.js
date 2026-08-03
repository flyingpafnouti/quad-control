/*
 * ui.js — Rendu canvas + construction dynamique des sliders + télémétrie.
 */
(function (QUAD) {
  'use strict';

  /* ------------------------------------------------------------------ */
  /* Rendu de la scène 2D                                                */
  /* ------------------------------------------------------------------ */
  class Renderer {
    constructor(canvas, sim) {
      this.canvas = canvas;
      this.ctx = canvas.getContext('2d');
      this.sim = sim;
      this.ppm = 90;          // pixels par mètre (zoom)
      this.originYfrac = 0.82; // position du sol dans le canvas (fraction)
      this._resize();
      window.addEventListener('resize', () => this._resize());
    }

    _resize() {
      const rect = this.canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      this.canvas.width = rect.width * dpr;
      this.canvas.height = rect.height * dpr;
      this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      this.W = rect.width;
      this.H = rect.height;
    }

    /** Monde (x droite, y haut) -> écran (px). Origine x au centre. */
    w2s(x, y) {
      return {
        sx: this.W / 2 + x * this.ppm,
        sy: this.H * this.originYfrac - y * this.ppm,
      };
    }

    /** Écran (px) -> monde. */
    s2w(sx, sy) {
      return {
        x: (sx - this.W / 2) / this.ppm,
        y: (this.H * this.originYfrac - sy) / this.ppm,
      };
    }

    draw() {
      const ctx = this.ctx;
      ctx.clearRect(0, 0, this.W, this.H);
      this._grid();
      this._ground();
      this._target();
      this._trail();
      this._quad();
    }

    _grid() {
      const ctx = this.ctx;
      ctx.save();
      ctx.strokeStyle = 'rgba(255,255,255,0.05)';
      ctx.lineWidth = 1;
      const step = this.ppm; // 1 m
      // verticales
      for (let x = -8; x <= 8; x++) {
        const { sx } = this.w2s(x, 0);
        ctx.beginPath();
        ctx.moveTo(sx, 0);
        ctx.lineTo(sx, this.H);
        ctx.stroke();
      }
      // horizontales
      for (let y = 0; y <= 8; y++) {
        const { sy } = this.w2s(0, y);
        ctx.beginPath();
        ctx.moveTo(0, sy);
        ctx.lineTo(this.W, sy);
        ctx.stroke();
      }
      ctx.restore();
    }

    _ground() {
      const ctx = this.ctx;
      const { sy } = this.w2s(0, 0);
      ctx.save();
      ctx.strokeStyle = 'rgba(120,200,160,0.6)';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(0, sy);
      ctx.lineTo(this.W, sy);
      ctx.stroke();
      ctx.restore();
    }

    _target() {
      const ctx = this.ctx;
      const { x, y } = this.sim.ref;
      const { sx, sy } = this.w2s(x, y);
      ctx.save();
      ctx.strokeStyle = '#ffcc44';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(sx, sy, 10, 0, Math.PI * 2);
      ctx.moveTo(sx - 16, sy);
      ctx.lineTo(sx + 16, sy);
      ctx.moveTo(sx, sy - 16);
      ctx.lineTo(sx, sy + 16);
      ctx.stroke();
      ctx.restore();
    }

    _trail() {
      const ctx = this.ctx;
      const t = this.sim.trail;
      if (t.length < 2) return;
      ctx.save();
      ctx.lineWidth = 2;
      ctx.strokeStyle = 'rgba(90,170,255,0.5)';
      ctx.beginPath();
      for (let i = 0; i < t.length; i++) {
        const { sx, sy } = this.w2s(t[i].x, t[i].y);
        if (i === 0) ctx.moveTo(sx, sy);
        else ctx.lineTo(sx, sy);
      }
      ctx.stroke();
      ctx.restore();
    }

    _quad() {
      const ctx = this.ctx;
      const s = this.sim.model.state;
      const p = this.sim.model.p;
      const { sx, sy } = this.w2s(s.x, s.y);
      const Lpx = p.L * this.ppm;

      ctx.save();
      ctx.translate(sx, sy);
      // rotation écran : y est inversé, donc l'angle visuel = -theta
      ctx.rotate(-s.th);

      // bras
      ctx.strokeStyle = '#dfe7ef';
      ctx.lineWidth = 5;
      ctx.lineCap = 'round';
      ctx.beginPath();
      ctx.moveTo(-Lpx, 0);
      ctx.lineTo(Lpx, 0);
      ctx.stroke();

      // corps central
      ctx.fillStyle = '#5aa0ff';
      ctx.beginPath();
      ctx.arc(0, 0, 8, 0, Math.PI * 2);
      ctx.fill();

      // rotors + vecteurs de poussée
      const Tmax = this.sim.model.rotorMax();
      const drawRotor = (dir, T) => {
        const rx = dir * Lpx;
        ctx.fillStyle = '#243447';
        ctx.strokeStyle = '#9fb3c8';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.ellipse(rx, -3, 12, 4, 0, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
        // vecteur poussée (vers le haut du corps)
        const len = Tmax > 0 ? (T / Tmax) * 46 : 0;
        ctx.strokeStyle = '#ff7a59';
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(rx, -4);
        ctx.lineTo(rx, -4 - len);
        ctx.stroke();
      };
      drawRotor(1, this.sim.telemetry.T1);
      drawRotor(-1, this.sim.telemetry.T2);

      ctx.restore();
    }
  }

  /* ------------------------------------------------------------------ */
  /* Panneau de sliders dynamique                                        */
  /* ------------------------------------------------------------------ */
  class ParamPanel {
    /**
     * @param {HTMLElement} container
     * @param {() => BaseController} getController
     */
    constructor(container, getController) {
      this.container = container;
      this.getController = getController;
    }

    /** (Re)construit les sliders à partir du contrôleur courant. */
    rebuild() {
      const ctrl = this.getController();
      const spec = ctrl.getParamSpec();
      this.container.innerHTML = '';

      spec.forEach((group) => {
        const g = document.createElement('div');
        g.className = 'param-group';
        const h = document.createElement('h4');
        h.textContent = group.label;
        g.appendChild(h);

        group.params.forEach((pr) => {
          const row = document.createElement('div');
          row.className = 'param-row';

          const lab = document.createElement('label');
          lab.textContent = pr.label;

          const slider = document.createElement('input');
          slider.type = 'range';
          slider.min = pr.min;
          slider.max = pr.max;
          slider.step = pr.step;
          slider.value = pr.value;

          const val = document.createElement('span');
          val.className = 'param-val';
          val.textContent = (+pr.value).toFixed(2);

          slider.addEventListener('input', () => {
            val.textContent = (+slider.value).toFixed(2);
            const obj = {};
            obj[pr.key] = parseFloat(slider.value);
            this.getController().setParams(obj);
          });

          row.appendChild(lab);
          row.appendChild(slider);
          row.appendChild(val);
          g.appendChild(row);
        });

        this.container.appendChild(g);
      });
    }
  }

  QUAD.Renderer = Renderer;
  QUAD.ParamPanel = ParamPanel;
})(window.QUAD || (window.QUAD = {}));
