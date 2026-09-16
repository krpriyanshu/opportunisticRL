# Requirements

`pyproject.toml` at the repository root is the canonical dependency
specification (`pip install -e .` / `pip install -e ".[dev]"` /
`pip install -e ".[rl]"`).

This directory is reserved for **pinned, reproducibility-grade lock
files** (Section 33) once there is a training pipeline whose results
depend on exact package versions — e.g. `requirements/rl-lock.txt`,
generated via `pip freeze` at the point a reported experiment was run,
alongside the git commit hash recorded in that experiment's metadata
(Section 29). It is intentionally empty until Milestone M6, when the
first training run exists to lock against; a lock file with nothing to
pin against would go stale immediately and mislead more than help.
