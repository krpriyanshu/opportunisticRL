"""B2: standard reactive RL via a small, self-contained PPO. Depends only
on torch + numpy + the DynamicGraspEnv -- no Stable-Baselines3, per
docs/rl-formulation.md's Framework decision ("a small custom PPO" is an
explicitly-permitted option, chosen here to keep the dependency footprint
minimal and the algorithm inspectable).

This is a deliberately compact, textbook clipped-PPO with GAE. It is NOT
a novel algorithm (rl-formulation.md: "no custom RL framework", "B2 uses
standard published algorithms") -- the research contribution is the
opportunity representation (B4), not this. B2 exists as the reactive
control condition H1 is measured against.

PPO chosen over SAC per the spec's own reasoning, now confirmed rather
than assumed: the action space is Discrete(4), and PPO has simpler,
more standard discrete-action support (a categorical policy head) than
SAC, whose continuous formulation would need a discrete-SAC variant for
no benefit here.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Categorical

from opportunity_rl.algorithms.interface import EvaluationResult

# Training-episode seeds live in a high band, one contiguous stride per
# configured seed, so they are disjoint from the held-out evaluation seeds
# (`seed + 10_000 + episode`) by construction. See
# PPO._next_train_episode_seed.
TRAIN_SEED_BASE = 1_000_000
TRAIN_SEED_STRIDE = 1_000_000


@dataclass(frozen=True)
class PPOConfig:
    hidden_dim: int = 64
    learning_rate: float = 3e-4
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_epsilon: float = 0.2
    entropy_coef: float = 0.01
    value_coef: float = 0.5
    rollout_steps: int = 2048
    epochs_per_update: int = 10
    minibatch_size: int = 64
    max_grad_norm: float = 0.5
    seed: int = 0


class ActorCritic(nn.Module):
    """Shared-trunk actor-critic. A categorical policy head (discrete
    actions) and a scalar value head."""

    def __init__(self, obs_dim: int, n_actions: int, hidden_dim: int) -> None:
        super().__init__()
        self.trunk = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh(),
        )
        self.policy_head = nn.Linear(hidden_dim, n_actions)
        self.value_head = nn.Linear(hidden_dim, 1)

    def forward(self, obs: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        features = self.trunk(obs)
        return self.policy_head(features), self.value_head(features).squeeze(-1)

    def act(self, obs: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        logits, value = self.forward(obs)
        dist = Categorical(logits=logits)
        action = dist.sample()
        return action, dist.log_prob(action), value


def _compute_gae(
    rewards: np.ndarray,
    values: np.ndarray,
    dones: np.ndarray,
    last_value: float,
    gamma: float,
    gae_lambda: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Generalized Advantage Estimation. `dones` marks the *end* of an
    episode (terminated OR truncated) so the advantage trace stops at
    episode boundaries -- returns and advantages are computed per-episode,
    not across resets.

    Time-limit truncation is handled by the caller, which folds
    `gamma * V(final_obs)` into that step's reward before calling this
    (see _collect_rollout). That keeps this function a plain GAE over
    whatever rewards it is given, while still bootstrapping correctly
    through a cutoff the MDP itself would have continued past."""
    n = len(rewards)
    advantages = np.zeros(n, dtype=np.float32)
    gae = 0.0
    for t in reversed(range(n)):
        next_value = last_value if t == n - 1 else values[t + 1]
        next_non_terminal = 1.0 - dones[t]
        delta = rewards[t] + gamma * next_value * next_non_terminal - values[t]
        gae = delta + gamma * gae_lambda * next_non_terminal * gae
        advantages[t] = gae
    returns = advantages + values
    return advantages, returns


class PPO:
    """Implements RLAlgorithm (structurally). See module docstring."""

    def __init__(self, env, config: PPOConfig | None = None, device: str = "cpu") -> None:
        self._env = env
        self._cfg = config or PPOConfig()
        self._device = torch.device(device)

        torch.manual_seed(self._cfg.seed)
        np.random.seed(self._cfg.seed)

        obs_dim = int(env.observation_space.shape[0])
        n_actions = int(env.action_space.n)
        self._net = ActorCritic(obs_dim, n_actions, self._cfg.hidden_dim).to(self._device)
        self._optimizer = torch.optim.Adam(self._net.parameters(), lr=self._cfg.learning_rate)

        self._train_episode_index = 0
        self._obs, _ = self._env.reset(seed=self._next_train_episode_seed())

    def _next_train_episode_seed(self) -> int:
        """A deterministic, never-repeating seed for each training episode.

        Every training episode must be explicitly seeded: the simulator
        backend reseeds from OS entropy on an unseeded reset
        (`random.Random.seed(None)`), so a bare `env.reset()` -- which is
        what this class used to call between episodes -- made a whole
        training run irreproducible even with torch/numpy seeded. Two
        same-seed runs then diverged in their learned weights, and the
        B2/B4 arms of the H1 comparison did not see matched episode
        streams despite being handed the same seed.

        The band is chosen so training seeds can never collide with the
        held-out evaluation seeds (`seed + EVAL_SEED_OFFSET + episode`,
        i.e. the low tens of thousands), preserving rl-formulation.md
        Section 13's train/eval seed disjointness by construction rather
        than by luck."""
        seed = TRAIN_SEED_BASE + self._cfg.seed * TRAIN_SEED_STRIDE + self._train_episode_index
        self._train_episode_index += 1
        return seed

    # ---- RLAlgorithm API ----------------------------------------------
    def train(self, total_steps: int) -> None:
        steps_done = 0
        while steps_done < total_steps:
            batch = self._collect_rollout()
            self._update(batch)
            steps_done += self._cfg.rollout_steps

    def evaluate(self, n_episodes: int, seed: int) -> EvaluationResult:
        """Greedy (argmax) evaluation on fixed seeds -- never shares a
        seed with training (rl-formulation.md Section 13). Deterministic
        policy so the reported number reflects the learned policy, not
        exploration noise."""
        successes = 0
        returns = []
        lengths = []
        reason_counts: dict[str, int] = {}

        for ep in range(n_episodes):
            obs, _ = self._env.reset(seed=seed + ep)
            done = False
            ep_return = 0.0
            ep_len = 0
            while not done:
                with torch.no_grad():
                    logits, _ = self._net(self._to_tensor(obs))
                    action = int(torch.argmax(logits).item())
                obs, reward, terminated, truncated, info = self._env.step(action)
                ep_return += reward
                ep_len += 1
                done = terminated or truncated
                if done:
                    reason = info.get("termination_reason") or "none"
                    reason_counts[reason] = reason_counts.get(reason, 0) + 1
                    if reason == "success":
                        successes += 1
            returns.append(ep_return)
            lengths.append(ep_len)

        # Evaluation drives the SAME env this instance trains on, so it
        # leaves that env parked wherever the last eval episode ended
        # (often mid-episode, with a latched gripper and an eval-era
        # episode start time). Without this, `self._obs` still held the
        # pre-evaluation training observation while the env had moved on,
        # and the next rollout resumed inside the finished eval episode,
        # recording observations that did not correspond to the states
        # that produced their rewards. Re-resetting onto a fresh, seeded
        # training episode restores a clean, deterministic boundary.
        # Matters most for learning_curve_h1.py, which interleaves
        # train/evaluate at every budget.
        self._obs, _ = self._env.reset(seed=self._next_train_episode_seed())

        return EvaluationResult(
            n_episodes=n_episodes,
            success_rate=successes / n_episodes,
            mean_return=float(np.mean(returns)),
            mean_episode_length=float(np.mean(lengths)),
            termination_reason_counts=reason_counts,
        )

    def save_checkpoint(self, path: Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        torch.save(
            {"model": self._net.state_dict(), "optimizer": self._optimizer.state_dict()}, path
        )

    def load_checkpoint(self, path: Path) -> None:
        ckpt = torch.load(Path(path), map_location=self._device)
        self._net.load_state_dict(ckpt["model"])
        self._optimizer.load_state_dict(ckpt["optimizer"])

    # ---- internals -----------------------------------------------------
    def _to_tensor(self, obs: np.ndarray) -> torch.Tensor:
        return torch.as_tensor(obs, dtype=torch.float32, device=self._device).unsqueeze(0)

    def _collect_rollout(self) -> dict[str, torch.Tensor]:
        n = self._cfg.rollout_steps
        obs_dim = int(self._env.observation_space.shape[0])
        obs_buf = np.zeros((n, obs_dim), dtype=np.float32)
        action_buf = np.zeros(n, dtype=np.int64)
        logprob_buf = np.zeros(n, dtype=np.float32)
        reward_buf = np.zeros(n, dtype=np.float32)
        value_buf = np.zeros(n, dtype=np.float32)
        done_buf = np.zeros(n, dtype=np.float32)

        for t in range(n):
            obs_buf[t] = self._obs
            with torch.no_grad():
                action, logprob, value = self._net.act(self._to_tensor(self._obs))
            a = int(action.item())
            next_obs, reward, terminated, truncated, _ = self._env.step(a)

            action_buf[t] = a
            logprob_buf[t] = float(logprob.item())
            reward_buf[t] = reward
            value_buf[t] = float(value.item())
            done_buf[t] = 1.0 if (terminated or truncated) else 0.0

            if truncated and not terminated:
                # Time-limit bootstrap: the episode was cut off by the
                # max-duration limit, not by reaching an absorbing state,
                # so the value of what would have followed is real and
                # must not be discarded. Folding gamma * V(final_obs) into
                # this step's reward lets the advantage trace still break
                # at the boundary (done_buf above) without pretending the
                # future was worth zero. Most episodes here end in
                # timeout, so treating them as terminations -- which this
                # previously did -- systematically depressed value
                # estimates near the horizon.
                with torch.no_grad():
                    _, bootstrap_value = self._net(self._to_tensor(next_obs))
                reward_buf[t] += self._cfg.gamma * float(bootstrap_value.item())

            if terminated or truncated:
                self._obs, _ = self._env.reset(seed=self._next_train_episode_seed())
            else:
                self._obs = next_obs

        with torch.no_grad():
            _, last_value_t = self._net(self._to_tensor(self._obs))
        last_value = float(last_value_t.item())

        advantages, returns = _compute_gae(
            reward_buf, value_buf, done_buf, last_value, self._cfg.gamma, self._cfg.gae_lambda
        )
        # Normalize advantages (standard PPO stabilization).
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

        return {
            "obs": torch.as_tensor(obs_buf, device=self._device),
            "actions": torch.as_tensor(action_buf, device=self._device),
            "old_logprobs": torch.as_tensor(logprob_buf, device=self._device),
            "advantages": torch.as_tensor(advantages, device=self._device),
            "returns": torch.as_tensor(returns, device=self._device),
        }

    def _update(self, batch: dict[str, torch.Tensor]) -> None:
        n = batch["obs"].shape[0]
        indices = np.arange(n)
        for _ in range(self._cfg.epochs_per_update):
            np.random.shuffle(indices)
            for start in range(0, n, self._cfg.minibatch_size):
                mb = indices[start : start + self._cfg.minibatch_size]
                mb_idx = torch.as_tensor(mb, device=self._device)

                logits, values = self._net(batch["obs"][mb_idx])
                dist = Categorical(logits=logits)
                new_logprobs = dist.log_prob(batch["actions"][mb_idx])
                entropy = dist.entropy().mean()

                ratio = torch.exp(new_logprobs - batch["old_logprobs"][mb_idx])
                adv = batch["advantages"][mb_idx]
                clipped = torch.clamp(ratio, 1 - self._cfg.clip_epsilon, 1 + self._cfg.clip_epsilon)
                policy_loss = -torch.min(ratio * adv, clipped * adv).mean()
                value_loss = ((values - batch["returns"][mb_idx]) ** 2).mean()

                loss = (
                    policy_loss
                    + self._cfg.value_coef * value_loss
                    - self._cfg.entropy_coef * entropy
                )
                self._optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self._net.parameters(), self._cfg.max_grad_norm)
                self._optimizer.step()
