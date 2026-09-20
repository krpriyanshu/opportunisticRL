# Documentation Index

Documents are grouped by evidential role. The distinction matters when assessing a claim: a **specification** states what was intended, a **record** states what was observed, and a **review** states what the surrounding literature or engineering context implies. Where a specification and a record disagree, the record governs.

**Start here if you are assessing the research:** [research-question.md](research-question.md) for the question and hypothesis statuses, then [evaluation.md](evaluation.md) for the experiment and its limitations. Note that all reported results are currently marked **provisional**; [evaluation.md §2](evaluation.md#2-provenance-and-status-of-the-evidence) explains why.

## Research core

| Document | Role | Contents |
|---|---|---|
| [research-question.md](research-question.md) | Specification | The question, hypotheses H1–H4 with current status, and the pre-registered criteria for support and rejection. Authoritative; every other document is subordinate to it. |
| [evaluation.md](evaluation.md) | Record | The controlled H1 experiment: design, statistical treatment, results at 3 and 8 seeds, threats to validity, and the conditions under which the results may be cited. |
| [related-work-review.md](related-work-review.md) | Review | H1's novelty claim tested against the closest located prior work, with per-source confidence tagging. |

## System and method

| Document | Role | Contents |
|---|---|---|
| [architecture.md](architecture.md) | Specification | Layered design, the two-backend structure, language boundaries, and the confound register governing whether a comparison is admissible as evidence. |
| [rl-formulation.md](rl-formulation.md) | Specification + record | Action space, observation schema, reward, episode structure, opportunity representation, baseline ladder — and the collected divergences between specification and implementation. |
| [ros2.md](ros2.md) | Record | TF2 frame tree, coordinate and unit conventions, and the `marine_state_estimator` topic contract, all established by inspection of the running system. |
| [environment.md](environment.md) | Record | Installed versions, privileged install steps, and the environment defects encountered, with causes and fixes. Primarily a reproduction aid. |

## Programme and provenance

| Document | Role | Contents |
|---|---|---|
| [project-plan.md](project-plan.md) | Record | Methodological commitments, the milestone record, what was deferred or abandoned, and what remains before the results are defensible. |
| [implementation-status.md](implementation-status.md) | Record | Per-milestone validation: what was built, what was verified and how, and what remains unverified. |
| [engineering-review.md](engineering-review.md) | Review, dated | A point-in-time audit from 2026-08-11 with outcomes appended. Retained because two of its predictions later materialised as the defects now limiting the results. |

## Background

| Document | Role | Contents |
|---|---|---|
| [wave-simulation-literature-review.md](wave-simulation-literature-review.md) | Review | Paper-by-paper survey of ocean-wave modelling in marine-robotics simulators, including a source-verified account of what this project's VRX installation implements. |

## Figures and media

`images/` holds the published figure and the captured simulation media referenced from the documents above. The H1 learning-curve figure is generated from the 3-seed sample; see [evaluation.md §6](evaluation.md#6-learning-curves-and-per-cell-detail) for why that matters.

## Conventions

1. **Provisional status is stated, not implied.** Any result whose provenance cannot be established is marked provisional at the point of use rather than qualified once in a footnote.
2. **Divergences are recorded where the specification lives**, so a reader of the specification cannot miss them.
3. **Negative and null results are reported as such.** The headline finding of this work is a split — an effect on sample efficiency, none on final performance — and is stated that way throughout.
4. **Verification claims name their method.** "Verified" in these documents means observed in the running system or asserted by a test, and the record says which.
