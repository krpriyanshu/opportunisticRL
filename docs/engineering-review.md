# Engineering Review (2026-08-11)

A software-engineering and research-infrastructure audit of this
repository, conducted at the M0/M1→M2 boundary (before any RL code
exists), specifically asking: *will the current structure hold up once
the baseline ladder (B0–B4), a disturbance sweep, and repeated ablations
turn this into dozens-to-hundreds of tracked experiment runs?* This is a
point-in-time review, not a doc that gets kept continuously in sync with
the code the way `project-plan.md`/`implementation-status.md` are —
treat it as a dated audit, re-run this kind of pass again around M5 and
M9 rather than expecting this file to still be accurate by then.

## Bottom line

This repository is unusually well-prepared for long-term research
engineering — most of the load-bearing decisions a project normally
gets wrong (or skips) at this stage are already made *correctly* and
*in writing*: the RL layer's independence from ROS2/Gazebo is a real
architectural boundary (`SimulationInterface`, not just an aspiration),
there is a living confound register that most published RL papers don't
even have internally, and deferred work (`requirements/`, `configs/`,
`experiments/`) is deferred *with a stated trigger condition*, not just
absent. The gaps found are almost all about **infrastructure that is
correctly not-yet-built** (config system, experiment tracking, results
schema) needing a *decision made now* so it doesn't get improvised under
pressure once M5's environment lands — plus a handful of small, concrete
things that were fixed directly in this pass.

## Fixed in this pass (not just flagged)

| Finding | Fix |
|---|---|
| `[tool.mypy] python_version = "3.10"` cannot type-check any file that imports numpy — numpy's own stubs use PEP 695 `type` syntax unconditionally, and mypy rejects that syntax when targeting <3.12 regardless of what the code itself needs. This was latent (nothing imported numpy yet) until the new `viz` extra. | Bumped to `python_version = "3.12"`, matching what CI (`.github/workflows/ci.yml`) and this environment actually run. **Residual gap, not fixed**: `requires-python = ">=3.10"` in `[project]` is now unverified by anything — CI only tests 3.12. Either lower the claimed floor to match reality, or add a 3.10 CI job if 3.10 support is actually a requirement. |
| `scripts/analyze_manipulator_coupling.py` had 3 pre-existing `ruff check` failures (2× line-too-long, 1× `zip()` without `strict=`) that would fail CI's `ruff check .` step on the very first push — nothing has been committed yet, so this was invisible. | Fixed directly (wrapped lines, added `strict=True`). Whole repo now passes `ruff check .`, `ruff format --check .`, `mypy`, and `pytest` cleanly. |

## Findings, prioritized

| # | Area | Severity | Finding | Recommendation |
|---|---|---|---|---|
| 1 | **Git history** | High (practical risk, not a code-quality issue) | `git status` shows **zero commits** — every doc, the ROS2 package, the CI config, all of M0/M1's work exists only on disk, uncommitted, in a single working tree. Known to be deliberate (standing instruction: hold commits for explicit go-ahead), but worth stating plainly as a risk in an engineering review: there is currently no version history, no rollback point, and no protection against an accidental `rm`/overwrite. | Not acted on here (respecting the standing instruction) — but this is the single highest-value "long-term scalability" action available: an initial commit costs nothing and loses nothing, and every day it's deferred is a day of irreplaceable work sitting on one disk with no history. |
| 2 | **Config management** | Medium, time-sensitive (blocks M5 cleanly) | No config system exists yet — correctly deferred (`configs/` is M4 per `project-plan.md`), but *how* it will work isn't decided, and this decision shapes the confound register (#4, #6 reference "shared `configs/*.yaml` files"), the results schema, and the experiment-metadata format all at once. Deciding it late means retrofitting all three. | Recommend deciding **now**, before M4: plain Python `@dataclass` configs (as used in `scripts/visualize_opportunity_concept.py` in this pass) + YAML for per-experiment overrides, with the *resolved* config always printed/logged verbatim at run start. Explicitly recommend **against** Hydra/OmegaConf-style composition frameworks for a single-researcher project at this stage — they solve a multi-config-file composition problem this project doesn't have yet, at the cost of "what config is actually active" becoming harder to answer from a run's logs, which cuts directly against the reproducibility discipline already established elsewhere in this repo. Revisit only if experiment count/config complexity genuinely outgrows a flat dataclass. |
| 3 | **Experiment tracking & `results/` schema** | Medium, time-sensitive (blocks M6) | `docs/rl-formulation.md` and `architecture.md` commit to logging seed, git commit hash, environment version, and total env-steps (Section 29) per run, and `.gitignore` already anticipates `wandb/` — but no tool or on-disk schema is chosen. `requirements/README.md` correctly defers the *lock file* to M6, but the *run-metadata schema* (what JSON/dict shape captures "one experiment") isn't specified anywhere yet, and that's a decision independent of which lock-file tool is used. | Recommend, in the same "minimum heavy packages" spirit as this session's own choices: a small `ExperimentMetadata` dataclass (seed, git commit hash, resolved config, env step count, start/end time, baseline ID) serialized to one JSON file per run directory, plus **TensorBoard** (already a near-zero-cost dependency once `torch` is installed, fully local, no account/network needed) for live curves. Treat Weights & Biases as an explicit upgrade path *if* multi-machine or team-shared dashboards become necessary — not a default. Decide the `results/<experiment>/<baseline>/<seed>/` directory convention at the same time, before the first B2 run makes an ad hoc choice permanent by precedent. |
| 4 | **Seeding discipline** | Medium (blocks M5) | `SimulationInterface.reset(seed=...)` and `DynamicGraspEnv.reset(seed=...)` are specified, but there is no single utility anywhere (yet) that seeds Python's `random`, `numpy`, and `torch` together — the classic failure mode is an experiment that looks seeded (env is seeded) but has an unseeded network initialization or an unseeded exploration policy, silently breaking exact reproducibility while every log line claims a seed was set. | Recommend a single `set_global_seed(seed: int) -> None` in `src/opportunity_rl` be one of the first functions written at M5, called once at the top of every training/eval entry point, never reimplemented per-script. |
| 5 | **Type-checking rigor** | Low now, rising | `disallow_untyped_defs = false` and CI's mypy step has `continue-on-error: true`. Entirely reasonable today (there is almost no code to check), but both are exactly the kind of setting that quietly stays permissive forever unless revisited on purpose — and tensor-shape bugs (wrong axis, wrong dtype, silently-broadcast shapes) are one of the highest-cost-to-debug error classes in RL code, which type checking catches cheaply if enforced. | Recommend flipping both at M5: drop `continue-on-error` once there's real code for it to gate on, and enable `disallow_untyped_defs` at least for new `src/opportunity_rl` modules (mypy supports this per-module via `[[tool.mypy.overrides]]` if a blanket flip is too disruptive). |
| 6 | **Combinatorial run scale** | Low now, will bite at M9–M10 | Baseline ladder (B0–B4, 5 methods) × disturbance curriculum stages × multiple seeds × the 3 supported opportunity-representation shapes (Section 9) composes to 100+ tracked runs by the robustness sweep (M10). Nothing wrong exists today, but there's no stated plan for *how experiments get launched and not accidentally duplicated or lost track of* at that scale. | Not urgent — flagging so it's a deliberate decision at M9, not a scramble. A simple sequential/threaded local launcher driven by a list of resolved configs (each hashable, so a run can be skipped if its exact config+seed already has a completed `results/` entry) is almost certainly sufficient; no need to reach for a job queue/cluster scheduler unless single-machine compute actually becomes the bottleneck (measure, don't assume, per this project's own Section 51.8 ground rule). |
| 7 | **`docs/` navigability** | Low | 9 docs now exist, spanning genuinely different purposes (frozen specs: `research-question.md`/`architecture.md`/`rl-formulation.md`; living logs: `project-plan.md`/`implementation-status.md`/`environment.md`; literature reviews: `wave-simulation-literature-review.md`/`related-work-review.md`; this file). `README.md` links each in context, which works today, but doesn't scale gracefully — a reader has no single place to see "spec vs. log vs. review" at a glance. | Low priority; worth a `docs/README.md` index once `evaluation.md`, `ros2.md`, `experiments.md`, and `research-log.md` land (all still pending per `project-plan.md`) and the doc count roughly doubles. |
| 8 | **Pre-commit feedback loop** | Optional | Lint/format/type/test discipline is enforced in CI but not locally before a commit is made — a contributor (or agent) only finds out about a `ruff format` violation after pushing. | Optional: a `pre-commit` config running `ruff check --fix`, `ruff format`, and `mypy` would shorten this feedback loop. Not urgent given CI already gates `main`, and this is a low-traffic solo-research repo right now. |
| 9 | **Citation metadata** | Optional | No `CITATION.cff`. Minor, but this repo is explicitly framed as producing a research artifact (README section 1, `research-question.md`) that may eventually accompany a paper. | Trivial to add whenever the author list / target venue is known; not worth doing speculatively now. |

## Strengths worth explicitly preserving

Not gaps — these are decisions that should survive contact with M5+
implementation pressure unchanged, because they're the reason the review
above is this short:

- **The `SimulationInterface` / `DynamicGraspEnv` split is real
  dependency inversion, not just a diagram.** Both a fast pure-Python
  backend and the ROS2/Gazebo backend are specified to satisfy the exact
  same `Protocol`. This is what will make B0–B4 training runs fast
  enough to actually iterate on (nobody wants to wait on Gazebo real-time
  factor for every PPO minibatch), and it's decided *before* the
  pressure to just wire training directly to Gazebo for expedience
  arrives. Do not let M5 implementation shortcut this by having
  `DynamicGraspEnv` reach past the interface into ROS2 types "just this
  once."
- **The confound register (`architecture.md`) is doing real work.**
  Most RL research codebases don't have anything like this until a
  reviewer asks "wait, did both methods get the same compute budget?"
  after submission. Having it pre-registered, with an explicit
  resolution milestone per row, means each future milestone has a
  concrete, checkable Definition of Done for *scientific validity*, not
  just *code exists*. Keep updating rows in place with dated notes, per
  its own stated convention — don't let a milestone close with a row
  still silently open.
- **The "smallest implementation, no speculative generality" ground
  rule is actually being followed**, not just stated — `requirements/`,
  `configs/`, `experiments/`, `results/`, and five of the six named
  ROS2 packages are correctly absent right now, each with a written
  trigger condition for when it gets created. This is precisely what
  keeps a research repo navigable at month six instead of accumulating
  empty scaffolding nobody remembers the purpose of.
- **Reward/opportunity/feasibility logic is specified as single,
  shared, config-parameterized implementations** (`rl-formulation.md`
  Sections 10, 16) rather than "one version per baseline" — this is the
  single most common way baseline comparisons in RL papers turn out to
  be comparing implementations rather than methods, and it's designed
  out from the start here.
- **The M1 progress log's honesty norm** ("no experiment is described
  as having run... until it has actually been executed," "no simulator
  capability is assumed working until it has actually been exercised")
  is visibly followed in practice, not just stated — e.g. the
  `gz_ros2_control` namespace bug in `docs/environment.md` is documented
  with the dead ends included, not just the fix. Keep this up; it's the
  difference between docs that are trustworthy years later and docs that
  quietly rot into fiction.

## What this review deliberately did not do

Per this project's own ground rules (`project-plan.md`'s "Ground rules,"
`architecture.md`'s milestone gating), this review did **not**
preemptively build `configs/`, an experiment tracker, or a `results/`
schema — those are M4/M6 deliverables, and building them now, ungated by
an actual environment or training loop to validate them against, would
be exactly the "empty scaffolding" anti-pattern the project has
correctly avoided so far. The recommendations above are decisions to
make *before* those milestones start, not code to write today.
