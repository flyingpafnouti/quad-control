# Entraînement du contrôleur RL

Politique de contrôle apprise par renforcement (PPO) pour le simulateur
quadrirotor 2D. La politique entraînée est exportée en JS et exécutée en
inférence dans le navigateur par `js/controllers/rl.js`.

## Fichiers

| Fichier | Rôle |
|---|---|
| `quad_env.py` | Environnement Gymnasium répliquant **exactement** la dynamique de `js/dynamics.js` (EOM, RK4, mixage + saturation rotors, paramètres physiques). Définit aussi les échelles obs/action partagées avec `rl.js`. |
| `train_rl.py` | Entraîne PPO puis exporte les poids vers `../js/controllers/rl_weights.js`. |
| `verify_js.js` | Charge les **vrais fichiers JS du navigateur** dans Node et simule des épisodes en boucle fermée pour valider la politique bout-en-bout. |

## Prérequis

- [uv](https://docs.astral.sh/uv/) (gestion de l'environnement Python)
- Node.js (pour `verify_js.js`)

## Utilisation

```bash
cd training

# Entraînement (~quelques minutes sur CPU) + export des poids JS
uv run python train_rl.py --timesteps 800000 --envs 8

# Vérification bout-en-bout via les fichiers JS réels
node verify_js.js
```

Puis ouvrir `../index.html` et choisir **« RL (policy) »** dans le sélecteur de contrôleur.

## Cohérence entraînement ↔ inférence (important)

La politique n'apprend rien sur la physique elle-même : elle apprend une
fonction `état → action`. Pour qu'elle se comporte pareil dans le navigateur,
trois choses **doivent** rester synchronisées entre `quad_env.py` et `rl.js` :

1. **La dynamique** (`m`, `L`, `Iz`, `g`, `Tmax`, EOM) — identique à `js/dynamics.js`.
2. **L'observation** : `[dx, vx, dy, vy, θ, ω] / OBS_SCALE`.
3. **Les échelles d'action** : `F = m·g + a[0]·DF_SCALE`, `M = a[1]·M_SCALE`.

Ces échelles sont embarquées dans `rl_weights.js` (bloc `meta`) et relues par
`rl.js`, donc un réentraînement qui les modifie se propage automatiquement.

## Récompense

Récompense par pas **positive** (bonus de survie + proximité gaussienne à la
cible, moins de petites pénalités d'angle/vitesse/effort), avec forte pénalité
et fin d'épisode en cas de crash sol / renversement / sortie de domaine. La
positivité est essentielle : avec une récompense par pas négative, l'agent
apprend à crasher tôt pour arrêter d'accumuler du négatif (« suicide »).
