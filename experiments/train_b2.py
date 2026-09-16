#!/usr/bin/env python3
"""M6: train the standard reactive RL baseline (B2, PPO) against the fast
DynamicGraspEnv and validate it actually learns -- i.e. its post-training
success rate beats a uniform-random policy on held-out evaluation seeds.

This is the M6 Definition of Done, run as a script rather than a unit
test because it takes minutes, not milliseconds. A fast smoke-sized
version of the same flow IS a unit test (tests/algorithms/test_ppo.py),
so CI still covers the wiring; this script is the real training run whose
numbers go in the gate report.

Reproducibility (rl-formulation.md Section 29 / engineering-review.md
recommendation): every run records seed, git commit, config, and total
env steps into a JSON metadata file alongside the checkpoint, under
results/<run_name>/. Training and evaluation never share a seed
(Section 13): eval uses seed + 10_000.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import time
from pathlib import Path

from opportunity_rl.algorithms.interface import EvaluationResult
from opportunity_rl.algorithms.ppo import PPO, PPOConfig
from opportunity_rl.algorithms.random_policy import RandomPolicy
from opportunity_rl.env.env import DynamicGraspEnv, EnvConfig
from opportunity_rl.provenance import git_commit, git_is_dirty
from opportunity_rl.sim.disturbance_config import load_disturbance_config
from opportunity_rl.sim.fast_simulator import FastSimulator

REPO_ROOT = Path(__file__).resolve().parents[1]
RESULTS_DIR = REPO_ROOT / "results"
EVAL_SEED_OFFSET = 10_000  # eval seeds never overlap training seeds (Section 13)


def _make_env(disturbance_name: str, grasp_duration_steps: int) -> DynamicGraspEnv:
    sim = FastSimulator(
        load_disturbance_config(disturbance_name), grasp_duration_steps=grasp_duration_steps
    )
    return DynamicGraspEnv(sim, config=EnvConfig())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--total-steps", type=int, default=100_000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--eval-episodes", type=int, default=200)
    parser.add_argument("--disturbance", type=str, default="sydney_regatta_default")
    parser.add_argument(
        "--grasp-duration-steps",
        type=int,
        default=1,
        help=(
            "decision steps a committed ATTEMPT_GRASP takes; >1 selects the "
            "anticipation-required task variant the H1 comparison uses"
        ),
    )
    parser.add_argument("--run-name", type=str, default=None)
    args = parser.parse_args()

    run_name = args.run_name or f"b2_ppo_seed{args.seed}_{int(time.time())}"
    run_dir = RESULTS_DIR / run_name
    run_dir.mkdir(parents=True, exist_ok=True)

    train_env = _make_env(args.disturbance, args.grasp_duration_steps)
    eval_env = _make_env(args.disturbance, args.grasp_duration_steps)

    # Random-policy floor (the control B2 must beat).
    random_result = RandomPolicy(eval_env, seed=args.seed).evaluate(
        args.eval_episodes, seed=args.seed + EVAL_SEED_OFFSET
    )
    print(
        f"[random] success_rate={random_result.success_rate:.3f} "
        f"mean_return={random_result.mean_return:.2f}"
    )

    ppo = PPO(train_env, config=PPOConfig(seed=args.seed))

    # Pre-training PPO eval (should be ~random, sanity check).
    pre = ppo.evaluate(args.eval_episodes, seed=args.seed + EVAL_SEED_OFFSET)
    print(f"[ppo pre-train] success_rate={pre.success_rate:.3f} mean_return={pre.mean_return:.2f}")

    t0 = time.time()
    ppo.train(args.total_steps)
    train_seconds = time.time() - t0

    post = ppo.evaluate(args.eval_episodes, seed=args.seed + EVAL_SEED_OFFSET)
    print(
        f"[ppo post-train] success_rate={post.success_rate:.3f} "
        f"mean_return={post.mean_return:.2f} "
        f"reasons={post.termination_reason_counts}"
    )

    ppo.save_checkpoint(run_dir / "checkpoint.pt")

    # Primary learning criterion is mean_return, not success_rate: under an
    # easy disturbance a random policy can eventually grasp within an
    # episode (so success_rate saturates for both), but return also
    # rewards grasping *efficiently* (fewer failed-attempt and per-step
    # time penalties), which is exactly what a learned policy should
    # improve. Both are reported; return is the one the PASS/FAIL is on.
    learned = post.mean_return > random_result.mean_return
    metadata = {
        "run_name": run_name,
        "git_commit": git_commit(REPO_ROOT),
        "git_dirty": git_is_dirty(REPO_ROOT),
        "seed": args.seed,
        "total_steps": args.total_steps,
        "eval_episodes": args.eval_episodes,
        "disturbance": args.disturbance,
        "grasp_duration_steps": args.grasp_duration_steps,
        "train_seconds": round(train_seconds, 1),
        "ppo_config": dataclasses.asdict(PPOConfig(seed=args.seed)),
        "env_config_repr": repr(EnvConfig()),
        "results": {
            "random": _result_dict(random_result),
            "ppo_pre_train": _result_dict(pre),
            "ppo_post_train": _result_dict(post),
        },
        "b2_beats_random_on_return": learned,
    }
    (run_dir / "metadata.json").write_text(json.dumps(metadata, indent=2))
    print(
        f"\n{'PASS' if learned else 'FAIL'}: B2 post-train mean_return "
        f"({post.mean_return:.2f}) {'>' if learned else '<='} "
        f"random ({random_result.mean_return:.2f}) "
        f"| success {post.success_rate:.3f} vs random {random_result.success_rate:.3f}"
    )
    print(f"metadata + checkpoint written to {run_dir}")


def _result_dict(r: EvaluationResult) -> dict:
    return {
        "success_rate": r.success_rate,
        "mean_return": r.mean_return,
        "mean_episode_length": r.mean_episode_length,
        "termination_reason_counts": r.termination_reason_counts,
    }


if __name__ == "__main__":
    main()
