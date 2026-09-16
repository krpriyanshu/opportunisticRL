#!/usr/bin/env python3
"""M9: the H1 controlled comparison -- B2 (reactive RL) vs B4
(opportunity-aware RL), matched in every way except the opportunity
signal, per docs/research-question.md's H1 statement.

What is held identical (the confound register, enforced by construction):
- same PPO algorithm and hyperparameters (PPOConfig)
- same environment, disturbance config, reward, decision rate
- same total training-step budget
- same seeds for training and (held-out, seed+10_000) evaluation
- same base observation schema

The ONLY difference: B4's env has an OpportunityEstimator injected, whose
output is APPENDED to the observation (never mixed into the base fields).
B2's env has no estimator. This is confound register item #1 realized
structurally -- verifiable by the fact that this script constructs the
two envs with the exact same arguments plus/minus the estimator.

Run for one seed:
    python3 experiments/train_h1_comparison.py --seed 0 \
        --disturbance moving_target --grasp-duration-steps 1

Grasp-duration-steps > 1 selects the "anticipation-required" task variant
(a committed multi-step grasp; see docs/implementation-status.md M7/M8),
where reacting to the current window is insufficient and anticipating
whether it will persist should matter -- IF the hypothesis holds. This
script reports the outcome either way; it does not assume a direction.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import time
from pathlib import Path

from opportunity_rl.algorithms.interface import EvaluationResult
from opportunity_rl.algorithms.ppo import PPO, PPOConfig
from opportunity_rl.env.env import DynamicGraspEnv, EnvConfig
from opportunity_rl.opportunity.estimator import OpportunityConfig, OpportunityEstimator
from opportunity_rl.provenance import git_commit, git_is_dirty
from opportunity_rl.sim.disturbance_config import load_disturbance_config
from opportunity_rl.sim.fast_simulator import FastSimulator

REPO_ROOT = Path(__file__).resolve().parents[1]
RESULTS_DIR = REPO_ROOT / "results"
EVAL_SEED_OFFSET = 10_000


def _make_sim(disturbance: str, grasp_duration_steps: int) -> FastSimulator:
    return FastSimulator(
        load_disturbance_config(disturbance), grasp_duration_steps=grasp_duration_steps
    )


def _make_b2_env(disturbance: str, grasp_duration_steps: int) -> DynamicGraspEnv:
    return DynamicGraspEnv(_make_sim(disturbance, grasp_duration_steps), config=EnvConfig())


def _make_b4_env(disturbance: str, grasp_duration_steps: int) -> DynamicGraspEnv:
    opp_cfg = OpportunityConfig()
    estimator = OpportunityEstimator(opp_cfg)
    return DynamicGraspEnv(
        _make_sim(disturbance, grasp_duration_steps),
        config=EnvConfig(),
        opportunity_estimator=estimator,
        opportunity_dim=opp_cfg.dim,
    )


def _train_and_eval(env, seed: int, total_steps: int, eval_episodes: int) -> EvaluationResult:
    ppo = PPO(env, config=PPOConfig(seed=seed))
    ppo.train(total_steps)
    return ppo.evaluate(eval_episodes, seed=seed + EVAL_SEED_OFFSET)


def _result_dict(r: EvaluationResult) -> dict:
    return {
        "success_rate": r.success_rate,
        "mean_return": r.mean_return,
        "mean_episode_length": r.mean_episode_length,
        "termination_reason_counts": r.termination_reason_counts,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--total-steps", type=int, default=100_000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--eval-episodes", type=int, default=200)
    parser.add_argument("--disturbance", type=str, default="moving_target")
    parser.add_argument("--grasp-duration-steps", type=int, default=1)
    parser.add_argument("--run-name", type=str, default=None)
    args = parser.parse_args()

    task_tag = f"{args.disturbance}_g{args.grasp_duration_steps}"
    run_name = args.run_name or f"h1_{task_tag}_seed{args.seed}_{int(time.time())}"
    run_dir = RESULTS_DIR / run_name
    run_dir.mkdir(parents=True, exist_ok=True)

    t0 = time.time()
    b2 = _train_and_eval(
        _make_b2_env(args.disturbance, args.grasp_duration_steps),
        args.seed,
        args.total_steps,
        args.eval_episodes,
    )
    b4 = _train_and_eval(
        _make_b4_env(args.disturbance, args.grasp_duration_steps),
        args.seed,
        args.total_steps,
        args.eval_episodes,
    )
    elapsed = time.time() - t0

    delta_return = b4.mean_return - b2.mean_return
    print(f"[H1 {task_tag} seed{args.seed}]")
    print(f"  B2 (reactive):          return={b2.mean_return:+.3f} success={b2.success_rate:.3f}")
    print(f"  B4 (opportunity-aware): return={b4.mean_return:+.3f} success={b4.success_rate:.3f}")
    print(
        f"  delta (B4 - B2) return = {delta_return:+.3f} "
        f"({'B4 better' if delta_return > 0 else 'B2 better or equal'})"
    )

    metadata = {
        "run_name": run_name,
        "git_commit": git_commit(REPO_ROOT),
        "git_dirty": git_is_dirty(REPO_ROOT),
        "seed": args.seed,
        "disturbance": args.disturbance,
        "grasp_duration_steps": args.grasp_duration_steps,
        "total_steps": args.total_steps,
        "eval_episodes": args.eval_episodes,
        "elapsed_seconds": round(elapsed, 1),
        "ppo_config": dataclasses.asdict(PPOConfig(seed=args.seed)),
        "opportunity_config": dataclasses.asdict(OpportunityConfig()),
        "results": {"B2": _result_dict(b2), "B4": _result_dict(b4)},
        "delta_return_b4_minus_b2": delta_return,
    }
    (run_dir / "metadata.json").write_text(json.dumps(metadata, indent=2))
    print(f"  metadata -> {run_dir / 'metadata.json'}")


if __name__ == "__main__":
    main()
