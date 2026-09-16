#!/usr/bin/env python3
"""M10/M11: H1 learning curves -- B2 vs B4 return as a function of
training budget, across seeds and (optionally) disturbance severities.

Motivation, from the M9 finding: at 100k steps B2 and B4 converge to
about the same return (the base observation eventually teaches the
reactive policy what it needs), but at a small budget B4 is clearly
ahead. So the H1 effect in these tasks is a SAMPLE-EFFICIENCY gain, not a
final-performance gain -- and the honest way to show that is a learning
curve, not a single end-point number. This script produces exactly that:
per-seed return for both policies at a schedule of training budgets, plus
a matplotlib figure with mean +/- std bands.

Also serves the M10 robustness question (H2 -- does the benefit grow with
disturbance magnitude?) by running the same curve across a severity
ladder of disturbance configs.

Everything is matched B2-vs-B4 exactly as in train_h1_comparison.py
(same PPO, same env, same seeds, same eval seeds); the only difference is
B4's appended opportunity block.
"""

from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from opportunity_rl.algorithms.ppo import PPO, PPOConfig  # noqa: E402
from opportunity_rl.env.env import DynamicGraspEnv, EnvConfig  # noqa: E402
from opportunity_rl.opportunity.estimator import (  # noqa: E402
    OpportunityConfig,
    OpportunityEstimator,
)
from opportunity_rl.provenance import git_commit, git_is_dirty  # noqa: E402
from opportunity_rl.sim.disturbance_config import load_disturbance_config  # noqa: E402
from opportunity_rl.sim.fast_simulator import FastSimulator  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[1]
RESULTS_DIR = REPO_ROOT / "results"
EVAL_SEED_OFFSET = 10_000


def _make_env(disturbance: str, grasp_steps: int, opportunity: bool) -> DynamicGraspEnv:
    sim = FastSimulator(load_disturbance_config(disturbance), grasp_duration_steps=grasp_steps)
    if opportunity:
        opp = OpportunityConfig()
        return DynamicGraspEnv(
            sim,
            config=EnvConfig(),
            opportunity_estimator=OpportunityEstimator(opp),
            opportunity_dim=opp.dim,
        )
    return DynamicGraspEnv(sim, config=EnvConfig())


def _curve_for_policy(
    disturbance: str,
    grasp_steps: int,
    opportunity: bool,
    seed: int,
    budgets: list[int],
    eval_episodes: int,
) -> list[float]:
    """Return the policy's mean_return evaluated after each cumulative
    budget in `budgets` (incremental training on one PPO instance)."""
    ppo = PPO(_make_env(disturbance, grasp_steps, opportunity), config=PPOConfig(seed=seed))
    returns = []
    trained = 0
    for budget in budgets:
        ppo.train(budget - trained)
        trained = budget
        returns.append(ppo.evaluate(eval_episodes, seed=seed + EVAL_SEED_OFFSET).mean_return)
    return returns


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--disturbances", type=str, nargs="+", default=["moving_target"])
    parser.add_argument("--grasp-duration-steps", type=int, default=3)
    parser.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    parser.add_argument("--budgets", type=int, nargs="+", default=[10_000, 20_000, 40_000, 80_000])
    parser.add_argument("--eval-episodes", type=int, default=200)
    parser.add_argument("--run-name", type=str, default="h1_learning_curve")
    args = parser.parse_args()

    run_dir = RESULTS_DIR / args.run_name
    run_dir.mkdir(parents=True, exist_ok=True)

    # data[disturbance][policy][seed] = list of returns over budgets
    data: dict = {}
    for disturbance in args.disturbances:
        data[disturbance] = {"B2": {}, "B4": {}}
        for seed in args.seeds:
            data[disturbance]["B2"][seed] = _curve_for_policy(
                disturbance,
                args.grasp_duration_steps,
                False,
                seed,
                args.budgets,
                args.eval_episodes,
            )
            data[disturbance]["B4"][seed] = _curve_for_policy(
                disturbance,
                args.grasp_duration_steps,
                True,
                seed,
                args.budgets,
                args.eval_episodes,
            )
            print(
                f"[{disturbance} seed{seed}] "
                f"B2={[round(x, 2) for x in data[disturbance]['B2'][seed]]} "
                f"B4={[round(x, 2) for x in data[disturbance]['B4'][seed]]}"
            )

    (run_dir / "curve_data.json").write_text(
        json.dumps(
            {
                "git_commit": git_commit(REPO_ROOT),
                "git_dirty": git_is_dirty(REPO_ROOT),
                "budgets": args.budgets,
                "grasp_duration_steps": args.grasp_duration_steps,
                "seeds": args.seeds,
                "data": data,
            },
            indent=2,
        )
    )
    _plot(data, args.budgets, args.disturbances, run_dir / "learning_curve.png")
    print(f"wrote {run_dir / 'learning_curve.png'} and curve_data.json")


def _mean_std(per_seed: dict, i: int) -> tuple[float, float]:
    vals = [per_seed[s][i] for s in per_seed]
    return statistics.mean(vals), (statistics.stdev(vals) if len(vals) > 1 else 0.0)


def _plot(data: dict, budgets: list[int], disturbances: list[str], out_path: Path) -> None:
    fig, axes = plt.subplots(
        1, len(disturbances), figsize=(5.5 * len(disturbances), 4.2), squeeze=False
    )
    for ax, disturbance in zip(axes[0], disturbances, strict=True):
        for policy, color in (("B2", "#1565c0"), ("B4", "#c0392b")):
            means = [_mean_std(data[disturbance][policy], i)[0] for i in range(len(budgets))]
            stds = [_mean_std(data[disturbance][policy], i)[1] for i in range(len(budgets))]
            lo = [m - s for m, s in zip(means, stds, strict=True)]
            hi = [m + s for m, s in zip(means, stds, strict=True)]
            ax.plot(
                budgets,
                means,
                "-o",
                color=color,
                label=f"{policy} ({'opportunity-aware' if policy == 'B4' else 'reactive'})",
            )
            ax.fill_between(budgets, lo, hi, color=color, alpha=0.15)
        ax.set_title(disturbance, fontsize=10)
        ax.set_xlabel("training env-steps")
        ax.set_ylabel("mean return (held-out eval)")
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)
    fig.suptitle(
        "H1: opportunity-aware (B4) vs reactive (B2) RL -- return vs training budget",
        fontsize=11,
    )
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
