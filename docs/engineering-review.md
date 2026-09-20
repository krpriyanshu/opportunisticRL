# Engineering Audit (2026-08-11), with Outcomes

A software-engineering and research-infrastructure audit conducted at the M1–M2 boundary, before any RL code existed, asking whether the structure would hold once the baseline ladder, a disturbance sweep and repeated ablations turned the repository into dozens or hundreds of tracked runs.

**This is a dated, point-in-time audit, not a continuously maintained document.** It is retained because two of its predictions subsequently materialised as real defects, which is itself evidence about where this kind of project fails. Each finding below carries an **Outcome** line recording what actually happened. For current system status see [implementation-status.md](implementation-status.md); for current research status see [evaluation.md](evaluation.md).

## 1. Assessment at the time

The repository was unusually well prepared for long-term research engineering. The load-bearing decisions that projects of this kind typically get wrong or skip were already made correctly and in writing: the RL layer's independence from ROS2 was a real architectural boundary rather than an aspiration; a confound register existed that most published RL work lacks internally; and deferred work carried stated trigger conditions rather than simply being absent.

The gaps identified were almost entirely about infrastructure that was *correctly not yet built* — configuration, experiment tracking, results schema — but that needed a decision made in advance so it would not be improvised under pressure.

## 2. Fixed during the audit

1. **`python_version = "3.10"` could not type-check any file importing numpy.** Numpy's stubs use PEP 695 syntax unconditionally, which mypy rejects when targeting below 3.12. Latent until a numpy-using extra was added. Bumped to 3.12 to match CI. *Residual gap, not fixed:* `requires-python = ">=3.10"` is consequently unverified by anything, since CI tests only 3.12.
2. **Three pre-existing lint failures** in `scripts/analyze_manipulator_coupling.py` that would have failed CI on first push. Fixed directly.

## 3. Findings and outcomes

| # | Area | Severity at the time | Finding | Outcome |
|---|---|---|---|---|
| 1 | Git history | High | Zero commits existed; all work sat uncommitted in one working tree with no history and no rollback point. | **Resolved.** The repository now has history. |
| 2 | Configuration management | Medium, time-sensitive | No configuration system existed and, more importantly, no decision about one. Recommended plain dataclasses plus YAML overrides with the resolved configuration logged verbatim, and explicitly recommended *against* composition frameworks such as Hydra for a single-researcher project. | **Resolved as recommended.** `configs/disturbances/*.yaml` loaded into dataclasses; unknown names fail loudly. |
| 3 | Experiment tracking and results schema | Medium, time-sensitive | No on-disk schema for "one experiment" was specified. Recommended a metadata dataclass serialised to one JSON file per run directory, plus TensorBoard for curves. | **Partially resolved, and the resolution failed silently.** Per-run `metadata.json` was adopted, but its `git_commit` field recorded the literal string `"HEAD"` for every run because the helper did not check git's return code. The mechanism intended to guarantee reproducibility did not. TensorBoard was not adopted. |
| 4 | Seeding discipline | Medium | No single utility seeded Python's `random`, numpy and torch together. The predicted failure mode was "an experiment that looks seeded but has an unseeded component, silently breaking reproducibility while every log line claims a seed was set." Recommended one `set_global_seed` called from every entry point. | **Not resolved, and the predicted failure occurred.** `torch.manual_seed` and `np.random.seed` are called inside the PPO constructor; no shared utility exists and Python's `random` is not seeded. Training-episode resets were unseeded, so same-seed runs diverged. This is directly responsible for the non-reproducibility documented in [evaluation.md §2](evaluation.md#2-provenance-and-status-of-the-evidence). |
| 5 | Type-checking rigor | Low, rising | `disallow_untyped_defs = false` and a `continue-on-error` mypy step in CI. Recommended flipping both once real code existed. | **Half resolved.** `continue-on-error` was dropped and mypy now gates the build. `disallow_untyped_defs` remains false. |
| 6 | Combinatorial run scale | Low then, predicted to bite at M9–M10 | The baseline ladder times severities times seeds times budgets composes to a large run count with no stated plan for how runs get launched without being duplicated or lost track of. | **Not resolved, and the predicted failure occurred.** Two overlapping sweeps exist (`h1_severity_curves` at 3 seeds and `h1_severity_curves_n8` at 8). The larger superseding run was never incorporated into the reported results, and its existence materially affects the H2 verdict. |
| 7 | `docs/` navigability | Low | Nine documents spanned genuinely different purposes — specifications, running logs, literature reviews — with no single place to see which was which. Recommended a `docs/README.md` index once the count grew. | **Resolved.** See [docs/README.md](README.md). |
| 8 | Pre-commit feedback loop | Optional | Lint, format and type discipline were enforced in CI but not locally before a commit. | **Not adopted.** CI remains the only gate. |
| 9 | Citation metadata | Optional | No `CITATION.cff`, though the repository is framed as producing a research artefact. | **Declined deliberately.** |

## 4. Strengths identified as worth preserving

These were recorded not as gaps but as decisions that should survive implementation pressure. All four did.

1. **The `SimulationInterface` / `DynamicGraspEnv` split is genuine dependency inversion.** It was specified so that both a fast pure-Python backend and a ROS2/Gazebo backend could satisfy one Protocol, decided before the pressure to wire training directly to Gazebo arrived. In the event, only the fast backend was built — but the boundary held, and training never reached past the interface.
2. **The confound register does real work.** Most RL codebases acquire nothing equivalent until a reviewer asks whether both methods received the same compute budget. Pre-registering it, with a resolution mechanism per row, gave each milestone a checkable definition of scientific validity rather than merely of completion.
3. **The "smallest implementation" rule was followed rather than merely stated.** Several planned components were correctly never built; [project-plan.md §3](project-plan.md#3-decisions-deferred-and-what-became-of-them) records which, and why.
4. **The honesty norm was visibly followed in practice.** Dead ends are documented alongside fixes, and negative results are reported as negative. The clearest demonstration is that the headline research finding is a nuanced split rather than a clean confirmation, and is reported as such.

## 5. Retrospective lesson

The two findings that were not acted upon — 4 and 6 — are precisely the two whose predicted failures occurred, and together they account for the current provenance problem: runs that cannot be reproduced, and a superseding run that was lost track of. Both were rated below the severity that would have compelled immediate action, and both concerned *verification of the verification mechanism* rather than the mechanism itself. A metadata field that records provenance, and a seeding call that appears to fix a seed, are exactly the kind of machinery that looks correct until something independently checks it.
