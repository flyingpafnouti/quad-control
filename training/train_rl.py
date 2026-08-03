"""
train_rl.py — Entraîne une politique PPO sur Quad2DEnv puis exporte les poids
du réseau de politique vers js/controllers/rl_weights.js (chargeable dans le navigateur).

Usage :
    uv run python train_rl.py --timesteps 800000
    uv run python train_rl.py --timesteps 200000 --envs 8   (rapide, moins bon)

Le réseau exporté est le MLP déterministe de la politique (moyenne de la gaussienne) :
    h = obs
    pour chaque couche cachée : h = tanh(W h + b)
    action = W_out h + b_out          (puis clip [-1,1] côté JS)
"""

import argparse
import json
import os

import numpy as np
import torch

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import BaseCallback

from quad_env import Quad2DEnv, OBS_SCALE, DF_SCALE, M_SCALE

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_JS = os.path.normpath(os.path.join(HERE, "..", "js", "controllers", "rl_weights.js"))


class ProgressCb(BaseCallback):
    """Log périodique de la récompense épisode moyenne."""

    def __init__(self, every=20000):
        super().__init__()
        self.every = every
        self._next = every

    def _on_step(self):
        if self.num_timesteps >= self._next:
            self._next += self.every
            ep = self.model.ep_info_buffer
            if ep:
                r = np.mean([e["r"] for e in ep])
                l = np.mean([e["l"] for e in ep])
                print(f"  [{self.num_timesteps:>8}]  ep_rew_mean={r:8.2f}  ep_len_mean={l:6.1f}")
        return True


def extract_layers(model):
    """Récupère les couches linéaires (policy_net cachées + action_net final)."""
    policy = model.policy
    layers = []

    seq = policy.mlp_extractor.policy_net  # Sequential(Linear, Tanh, Linear, Tanh, ...)
    for mod in seq:
        if isinstance(mod, torch.nn.Linear):
            layers.append(
                {
                    "W": mod.weight.detach().cpu().numpy().tolist(),
                    "b": mod.bias.detach().cpu().numpy().tolist(),
                    "act": "tanh",
                }
            )
    # couche de sortie : moyenne de l'action, linéaire
    out = policy.action_net
    layers.append(
        {
            "W": out.weight.detach().cpu().numpy().tolist(),
            "b": out.bias.detach().cpu().numpy().tolist(),
            "act": "linear",
        }
    )
    return layers


def export_js(model, path):
    layers = extract_layers(model)
    payload = {
        "meta": {
            "obs": ["dx", "vx", "dy", "vy", "theta", "omega"],
            "obsScale": OBS_SCALE.tolist(),
            "dfScale": DF_SCALE,
            "mScale": M_SCALE,
            "arch": [len(layers[0]["W"][0])] + [len(l["W"]) for l in layers],
        },
        "layers": layers,
    }
    js = (
        "/*\n"
        " * rl_weights.js — Poids de la politique RL entraînée (généré par training/train_rl.py).\n"
        " * NE PAS éditer à la main. Regénérer via : uv run python train_rl.py\n"
        " */\n"
        "(function (QUAD) {\n"
        "  'use strict';\n"
        "  QUAD.RL_WEIGHTS = "
        + json.dumps(payload)
        + ";\n"
        "})(window.QUAD || (window.QUAD = {}));\n"
    )
    with open(path, "w") as f:
        f.write(js)
    print(f"Poids exportés -> {path}")
    print(f"  architecture : {payload['meta']['arch']}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--timesteps", type=int, default=800000)
    ap.add_argument("--envs", type=int, default=8)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    env = make_vec_env(Quad2DEnv, n_envs=args.envs, seed=args.seed)

    model = PPO(
        "MlpPolicy",
        env,
        seed=args.seed,
        n_steps=2048,
        batch_size=512,
        gae_lambda=0.95,
        gamma=0.99,
        n_epochs=10,
        ent_coef=0.0,
        learning_rate=3e-4,
        clip_range=0.2,
        policy_kwargs=dict(net_arch=[64, 64], activation_fn=torch.nn.Tanh),
        verbose=0,
    )

    print(f"Entraînement PPO : {args.timesteps} pas, {args.envs} envs parallèles")
    model.learn(total_timesteps=args.timesteps, callback=ProgressCb(), progress_bar=False)

    # évaluation déterministe rapide
    eval_env = Quad2DEnv(seed=123)
    rews = []
    for ep in range(20):
        obs, _ = eval_env.reset(seed=1000 + ep)
        done = False
        total = 0.0
        while not done:
            act, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, _ = eval_env.step(act)
            total += r
            done = term or trunc
        rews.append(total)
    print(f"Éval déterministe (20 ép.) : moyenne={np.mean(rews):.1f}  min={np.min(rews):.1f}")

    export_js(model, OUT_JS)


if __name__ == "__main__":
    main()
