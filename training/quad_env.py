"""
quad_env.py — Environnement Gymnasium répliquant EXACTEMENT la dynamique du
simulateur JS (js/dynamics.js + js/simulator.js), pour entraîner une politique RL.

Points de fidélité au JS :
  - EOM :  x'' = -(F/m) sin θ ,  y'' = (F/m) cos θ - g ,  θ'' = M / Iz
  - Intégration RK4 à poussées (T1, T2) constantes sur le pas.
  - Mixage (F, M) -> (T1, T2) puis saturation rotor : 0 <= Ti <= Tmax·mg/2.
  - Paramètres physiques identiques (m, L, Iz, g, Tmax).

L'observation et la mise à l'échelle des actions sont IDENTIQUES à js/controllers/rl.js
(voir OBS_SCALE, DF_SCALE, M_SCALE) — c'est la condition pour que la politique
apprise se comporte pareil dans le navigateur.
"""

import numpy as np
import gymnasium as gym
from gymnasium import spaces

# --- Paramètres physiques (miroir de js/dynamics.js) ---
M_MASS = 1.0
L_ARM = 0.25
IZ = 0.02
G = 9.81
TMAX = 1.6  # poussée max par rotor = TMAX * m*g/2

ROTOR_MAX = TMAX * (M_MASS * G) / 2.0  # N par rotor

# --- Échelles observation / action (DOIVENT matcher rl.js) ---
# obs = [dx, vx, dy, vy, theta, omega] chacune divisée par ces facteurs
OBS_SCALE = np.array([3.0, 3.0, 3.0, 3.0, 0.6, 3.0], dtype=np.float32)
DF_SCALE = 6.0   # F = m*g + a[0]*DF_SCALE
M_SCALE = 1.5    # M = a[1]*M_SCALE

# --- Bornes du monde (comme le clamp de la consigne dans main.js) ---
X_LIM = 7.0
Y_MIN = -0.05  # marge sous le sol : tolère un léger transitoire au décollage depuis y≈0
Y_MAX = 7.0


def _deriv(s, T1, T2):
    _, vx, _, vy, th, om = s
    F = T1 + T2
    Mz = L_ARM * (T1 - T2)
    ax = -(F / M_MASS) * np.sin(th)
    ay = (F / M_MASS) * np.cos(th) - G
    ath = Mz / IZ
    return np.array([vx, ax, vy, ay, om, ath], dtype=np.float64)


def _rk4(s, T1, T2, dt):
    k1 = _deriv(s, T1, T2)
    k2 = _deriv(s + 0.5 * dt * k1, T1, T2)
    k3 = _deriv(s + 0.5 * dt * k2, T1, T2)
    k4 = _deriv(s + dt * k3, T1, T2)
    return s + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


def _mix_to_thrusts(F, M):
    T1 = (F + M / L_ARM) / 2.0
    T2 = (F - M / L_ARM) / 2.0
    T1 = min(ROTOR_MAX, max(0.0, T1))
    T2 = min(ROTOR_MAX, max(0.0, T2))
    return T1, T2


class Quad2DEnv(gym.Env):
    """Tâche : rejoindre et stabiliser une consigne de position (x_ref, y_ref)."""

    metadata = {"render_modes": []}

    def __init__(self, dt=0.02, max_steps=500, seed=None):
        super().__init__()
        self.dt = dt
        self.max_steps = max_steps
        self.action_space = spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)
        # observation : état-consigne mis à l'échelle (6)
        high = np.full(6, 10.0, dtype=np.float32)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)
        self._rng = np.random.default_rng(seed)
        self.s = None
        self.ref = None
        self.steps = 0

    def _obs(self):
        dx = self.s[0] - self.ref[0]
        dy = self.s[2] - self.ref[1]
        raw = np.array(
            [dx, self.s[1], dy, self.s[3], self.s[4], self.s[5]], dtype=np.float32
        )
        return (raw / OBS_SCALE).astype(np.float32)

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        r = self._rng
        # état initial : position aléatoire, petites vitesses/angle.
        # y0 descend jusqu'au ras du sol pour apprendre le DÉCOLLAGE (l'état de
        # reset par défaut du simulateur est posé au sol, y=0).
        x0 = r.uniform(-5.0, 5.0)
        y0 = r.uniform(0.02, 6.0)
        self.s = np.array(
            [
                x0,
                r.uniform(-0.5, 0.5),
                y0,
                r.uniform(-0.5, 0.5),
                r.uniform(-0.2, 0.2),
                r.uniform(-0.3, 0.3),
            ],
            dtype=np.float64,
        )
        # consigne aléatoire
        self.ref = np.array(
            [r.uniform(-5.0, 5.0), r.uniform(1.0, 6.0)], dtype=np.float64
        )
        self.steps = 0
        return self._obs(), {}

    def step(self, action):
        a = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        F = M_MASS * G + a[0] * DF_SCALE
        if F < 0.0:
            F = 0.0
        Mz = a[1] * M_SCALE
        T1, T2 = _mix_to_thrusts(F, Mz)
        self.s = _rk4(self.s, T1, T2, self.dt)
        self.steps += 1

        dx = self.s[0] - self.ref[0]
        dy = self.s[2] - self.ref[1]
        dist = np.hypot(dx, dy)
        vx, vy, th, om = self.s[1], self.s[3], self.s[4], self.s[5]

        # Récompense façonnée POSITIVE par pas : voler doit toujours rapporter
        # plus que crasher tôt (sinon l'agent apprend le "suicide"). On combine
        #   - un bonus de survie constant,
        #   - un bonus de proximité gaussien (dans [0, 2]),
        #   - de petites pénalités continues (angle, vitesse, effort).
        proximity = 2.0 * float(np.exp(-(dist ** 2) / (2 * 0.6 ** 2)))  # 2 au centre -> 0 au loin
        reward = (
            1.0                                   # survie
            + proximity
            - 0.10 * (th ** 2)
            - 0.02 * (vx ** 2 + vy ** 2)
            - 0.01 * (om ** 2)
            - 0.02 * float(np.sum(a ** 2))
        )
        if dist < 0.15:
            reward += 1.0  # bonus supplémentaire : stationner dans la cible

        terminated = False
        # crash sol / renversement / hors-domaine : forte pénalité + fin d'épisode
        # (perdre le flux de récompense positif est déjà fortement dissuasif).
        if self.s[2] < Y_MIN or abs(th) > 1.2 or abs(self.s[0]) > X_LIM or self.s[2] > Y_MAX + 1.0:
            reward = -50.0
            terminated = True

        truncated = self.steps >= self.max_steps
        return self._obs(), float(reward), terminated, truncated, {}
