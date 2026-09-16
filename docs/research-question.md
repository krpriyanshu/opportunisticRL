# Research Question

## Status: H1 resolved (nuanced) — see per-hypothesis status below and [evaluation.md](evaluation.md)

**Headline (updated after the M9–M11 controlled experiment):**
opportunity-awareness makes the timing policy **learn faster but not end
up better** on this project's tasks. As a *sample-efficiency* aid H1 is
supported (significantly); as a *final-performance* claim it is not. Full
evidence, figures, and statistics: [evaluation.md](evaluation.md). This
result is on the fast kinematic backend only (threats to validity are
listed in that doc).

This document is the single source of truth for the research question this
repository exists to answer. Every other design decision in the codebase
(environment, reward, baselines, evaluation protocol) is subordinate to this
document. If an implementation choice would make the comparison below
invalid, that is a bug regardless of whether the code runs.

## The question

> Does explicitly reasoning about future interaction opportunities improve
> RL decision-making for dynamic robotic grasping under stochastic
> disturbances, compared with reactive decision-making?

## Why this question, and why grasping-from-a-moving-vessel

A manipulator mounted on a disturbed platform, grasping an independently
moving target, is a minimal setting where "when" is as important a decision
as "how." A reactive policy can only judge the present instant: is the
target reachable *right now*. But grasp quality is a trajectory through
time — a target can be approaching a favorable configuration, passing
through one, or about to leave it. A policy that has *some* representation
of where that trajectory is headed can, in principle, choose to wait for a
better moment or commit before a good moment passes, rather than reacting
to the current instant alone.

Marine/USV simulation and ROS2 are the *setting* that produces this
temporal structure cheaply (wave- and current-driven disturbance is a
natural, physically motivated source of a moving reference frame and a
moving target). They are infrastructure, not the object of study. The
scientific claim this project is built to support or refute is about
decision-making under temporal structure, not about marine robotics or
ROS2 integration.

## Hypothesis

**H1 (primary).** An RL policy whose observation includes a representation
of future interaction quality (an "opportunity" signal — see
[opportunity-representation](rl-formulation.md#future-opportunity-representation))
achieves higher grasp success rate and/or lower timing regret against an
oracle (see below) than a policy with an otherwise-identical observation,
network capacity, action space, reward, training budget, and disturbance
distribution, but without the opportunity signal.

> **H1 status: partially supported — split by training budget (M9–M11,
> [evaluation.md](evaluation.md)).** Matched B2 (reactive) vs B4
> (opportunity-aware) PPO, 3 seeds, 3 disturbance severities, held-out
> eval:
> - **Sample efficiency: supported.** At low training budgets (10k–20k
>   env-steps) B4 beats B2 in **17 of 18** paired comparisons (sign-test
>   p = 0.0001, mean Δreturn +0.38). B4 learns the timing decision much
>   faster.
> - **Final performance: not supported.** By 40k–80k steps the two
>   converge (B4 wins 8/18, p = 0.81, mean Δ +0.002); the opportunity
>   signal does not raise the ceiling.
> - Interpretation: the base observation (with velocity) already lets a
>   *learned* reactive policy infer future opportunity given enough data,
>   so the explicit signal helps as an inductive bias / head start, not
>   as a source of higher asymptotic performance. Success rate saturates
>   on these tasks and is uninformative; return is the metric used.

**H1 is stated as a controlled comparison, not an isolated claim about one
policy's performance.** A number like "82% success rate" means nothing on
its own; only the *difference* between the opportunity-aware and reactive
conditions, under matched conditions, is evidence for or against H1.

### Secondary questions (only pursued if H1's infrastructure supports them
cleanly — see [project-plan.md](project-plan.md) for sequencing)

- **H2.** Does the benefit of opportunity-awareness increase as disturbance
  magnitude increases? (Robustness / Milestone M10.)
  > **Status: rejected in this setting (M10).** The sample-efficiency
  > benefit is present at all three severities (mild/medium/severe) but
  > does not systematically grow with disturbance magnitude — the
  > low-budget advantage is if anything largest at mild/medium. See
  > [evaluation.md](evaluation.md).
- **H3.** Does an opportunity-aware policy generalize better to disturbance
  regimes and target dynamics outside its training distribution than a
  reactive policy? (Distribution shift / evaluation scenario E5.)
  > **Status: not tested.** Would require training on one severity and
  > evaluating on another (a train/test-distribution split). Deferred —
  > out of scope for the milestones built so far; recorded as the clear
  > next experiment.
- **H4.** How much of the achievable gap to an oracle with future ground
  truth does the opportunity-aware policy close, relative to the reactive
  policy? (Timing regret — see [evaluation.md](evaluation.md).)
  > **Status: not tested.** The oracle baseline (true-future timing
  > reference) was specified but not implemented; without it, timing
  > regret cannot be computed. Deferred.

H2 has a resolved result above; H3/H4 remain open and are honestly marked
as untested rather than folded into H1's conclusion. They are recorded
here so the environment and metrics already support them (per Milestone
M8's requirement that opportunity representations be swappable).

## What would count as support, and what would count as rejection

- **Supports H1:** opportunity-aware policy statistically significantly
  outperforms the reactive policy (paired across seeds, matched training
  budget and disturbance distribution) on primary metric (success rate)
  or timing regret, across a disturbance sweep, without leaking future
  ground truth (see [Confound 7](architecture.md#confound-register)).
- **Rejects H1:** no statistically significant difference, or the reactive
  policy matches/exceeds the opportunity-aware policy under matched
  conditions.
- **Inconclusive (not evidence either way):** any comparison where a
  confound listed in [architecture.md](architecture.md#confound-register)
  is present and uncorrected. An inconclusive result must be reported as
  inconclusive, not folded into "supports" or "rejects."

## Non-goals

This project is not trying to show that marine robotics, ROS2, or MoveIt2
integration is novel — they are reused, off-the-shelf infrastructure
(see [Section 32 boundary](architecture.md#python-c-and-ros2-moveit2-boundary)).
It is not trying to produce a deployable or hardware-validated grasping
system. It is not trying to demonstrate a new RL algorithm — B2/B3/B4 use
standard, published algorithms. The only claimed contribution is an
empirical answer, under a specific and documented experimental protocol,
to the question stated above.

## Related work

H1's novelty claim has been stress-tested against the closest prior work
found (marine surface-manipulator grasping, future-prediction-based
grasp timing, RL-for-marine-rescue, and general dynamic-grasping/handover
literature) — see
[docs/related-work-review.md](related-work-review.md). Summary: no paper
found combines (1) a learned, separately-observable *future interaction
quality* representation, (2) feeding a discrete commitment decision, and
(3) isolated via a matched controlled comparison the way H1 requires —
but the closest two papers (a hook-based aerial payload grasping paper
using optimal-time trajectory optimization, and an RL-for-sea-rescue
paper) should be addressed explicitly, not implicitly, in any eventual
write-up. Two papers in the closest physical setup (USV + manipulator
grasping) could not be fully read (paywalled) and are flagged as the
review's main open risk.

## Current status

The controlled H1 experiment (M9–M11) has been executed; per-hypothesis
statuses above are resolved from the measured results, with the confound
register (architecture.md) satisfied structurally and the full evidence
in [evaluation.md](evaluation.md). Summary:

- **H1 — partially supported (split by budget):** significant
  sample-efficiency benefit (17/18 low-budget paired wins, p=0.0001), no
  final-performance benefit (8/18, p=0.81).
- **H2 — rejected in this setting:** benefit does not grow with
  disturbance severity.
- **H3, H4 — not tested** (distribution-shift split and the oracle timing
  reference were not implemented; honestly marked open, not folded into
  H1).

Everything above is on the fast kinematic backend; the ROS2/Gazebo
backend implementing the same interface was validated for the physical
grasp (M1–M3) but not trained against. These are stated as threats to
validity in [evaluation.md](evaluation.md), not omitted. Any future
change to a hypothesis status must likewise cite an executed run.
