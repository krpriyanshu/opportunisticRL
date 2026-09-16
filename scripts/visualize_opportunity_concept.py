#!/usr/bin/env python3
"""Standalone, ROS2/Gazebo-independent visualization of the core research
problem: a manipulator on a disturbed vessel deciding *when* to grasp an
independently moving target.

WHAT THIS IS NOT: this is not Milestone M4 (dynamic-grasping environment),
M5 (`DynamicGraspEnv`/`SimulationInterface`), or M8 (`OpportunityEstimator`).
Those will be implemented against the frozen interfaces in
docs/rl-formulation.md when their milestones are actually reached, following
the project's own validation process. Nothing here should be cited as
progress on those milestones.

WHAT THIS IS: a lightweight, seeded, numpy+matplotlib-only illustration of
the same qualitative dynamics (disturbed vessel, independently drifting
target, a distance/velocity-based "good opportunity" window, and a
reactive-vs-lookahead decision comparison), for exploration and for
communicating the research question visually without standing up Gazebo.
The wave-like jitter used here is a simplified multi-sinusoid stand-in
chosen for illustrative clarity, not the validated Pierson-Moskowitz/
Gerstner model documented in docs/wave-simulation-literature-review.md and
actually running in the VRX simulation (docs/environment.md).

Two decision policies are simulated side by side, under the *same* seeded
disturbance realization (paired comparison, mirroring
docs/research-question.md's evaluation protocol):

  - "reactive": acts the instant the current-step opportunity is good
    (mirrors B0 in docs/rl-formulation.md's baseline ladder).
  - "lookahead": only commits once a short constant-velocity extrapolation
    (never true future simulator state -- see the Confound #7 / Section 39
    leakage rule in docs/architecture.md) suggests the current moment is at
    least as good as what the near future offers.

Usage:
    pip install -e ".[viz]"
    python3 scripts/visualize_opportunity_concept.py
    python3 scripts/visualize_opportunity_concept.py --seed 3 --video --show

--video encodes via the system `ffmpeg` binary (already a dependency of
this repo -- see docs/environment.md's capture pipeline), not an extra
Python package, and produces a far smaller file than a Pillow-written GIF
for the same length.
"""

from __future__ import annotations

import argparse
import dataclasses
from pathlib import Path
from typing import Literal

import matplotlib

matplotlib.use("Agg")  # safe default for headless/no-display environments;
# _enable_interactive_backend() below switches to an interactive one when
# --show is passed and one is actually available.

import matplotlib.animation as animation  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

Policy = Literal["reactive", "lookahead"]
State = Literal["WAIT", "ACT", "SUCCESS", "ABORT"]


@dataclasses.dataclass(frozen=True)
class SimConfig:
    """Every numeric knob for one run. Deliberately flat and explicit --
    no hidden defaults buried in function bodies -- so a figure can be
    regenerated exactly from the parameters printed alongside it."""

    duration_s: float = 40.0
    dt: float = 0.1
    seed: int = 0

    # Grasp feasibility window (loose 2D analogue of rl-formulation.md's
    # Section 10 `reachable`/`good_opportunity` predicate).
    reach_radius_m: float = 1.6
    max_relative_speed_mps: float = 0.35

    # Vessel: slow current-driven drift + multi-sinusoid wave jitter.
    vessel_drift_mps: tuple[float, float] = (0.015, 0.008)
    vessel_wave_amplitudes_m: tuple[float, ...] = (0.35, 0.18, 0.08)
    vessel_wave_periods_s: tuple[float, ...] = (5.0, 3.1, 1.7)

    # Target: relative to the vessel, modeled as a radial approach/recede
    # oscillation (period `approach_period_s`, amplitude `approach_amplitude_m`
    # around `approach_center_m` -- distance from the reach point) plus a
    # small independent high-frequency jitter (the target is a separate
    # floating body, not rigidly attached to the vessel's own wave response,
    # so it does not simply inherit the vessel's jitter). A pure radial
    # (rather than circular-orbit) approach is deliberate: relative speed
    # is naturally lowest exactly at closest approach and exactly at the
    # far extreme -- i.e. the model does not need to be *rigged* for
    # low-speed windows to coincide with close-distance windows, they do so
    # for the same reason a pendulum is slowest at the ends of its swing.
    approach_direction_deg: float = 15.0
    approach_center_m: float = 1.6  # equal to reach_radius_m by default
    approach_amplitude_m: float = 0.9
    approach_period_s: float = 10.0
    target_wave_amplitudes_m: tuple[float, ...] = (0.06, 0.03)
    target_wave_periods_s: tuple[float, ...] = (2.6, 1.4)

    # End-effector reach point, fixed relative to the vessel (deck-mounted).
    reach_point_offset_m: tuple[float, float] = (0.9, 0.3)

    # Decision dynamics.
    grasp_duration_s: float = 1.5  # ACT -> SUCCESS/ABORT commitment window
    lookahead_horizon_s: float = 3.0  # "lookahead" policy's extrapolation window
    lookahead_samples: int = 8

    def wave_offset(
        self,
        t: np.ndarray,
        amplitudes: tuple[float, ...],
        periods: tuple[float, ...],
        phase_seed: int,
    ) -> np.ndarray:
        """Sum-of-sinusoids jitter (x, y) at times `t`. Not a physical wave
        model -- see module docstring. `phase_seed` decorrelates the x and y
        component phases and the vessel's phases from the target's."""
        rng = np.random.default_rng(phase_seed)
        x = np.zeros_like(t)
        y = np.zeros_like(t)
        for a, p in zip(amplitudes, periods, strict=True):
            omega = 2.0 * np.pi / p
            phase_x = rng.uniform(0, 2 * np.pi)
            phase_y = rng.uniform(0, 2 * np.pi)
            x += a * np.cos(omega * t + phase_x)
            y += a * np.sin(omega * t + phase_y)
        return np.stack([x, y], axis=-1)


@dataclasses.dataclass
class EpisodeResult:
    t: np.ndarray  # (T,)
    vessel_pos: np.ndarray  # (T, 2)
    target_pos: np.ndarray  # (T, 2)
    reach_pos: np.ndarray  # (T, 2)  end-effector reach point
    opportunity_score: np.ndarray  # (T,) in [0, 1], continuous
    good_opportunity: np.ndarray  # (T,) bool
    state: list[State]
    events: list[tuple[float, str]]  # (time, description) for annotations


def simulate(cfg: SimConfig, policy: Policy) -> EpisodeResult:
    """Run one seeded episode under the given decision policy. Pure
    function of (cfg, policy) -- no global state, so results are exactly
    reproducible from the printed config."""
    n = int(cfg.duration_s / cfg.dt) + 1
    t = np.arange(n) * cfg.dt

    vessel_base = np.array(cfg.vessel_drift_mps) * t[:, None]
    vessel_jitter = cfg.wave_offset(
        t, cfg.vessel_wave_amplitudes_m, cfg.vessel_wave_periods_s, phase_seed=cfg.seed
    )
    vessel_pos = vessel_base + vessel_jitter
    reach_pos = vessel_pos + np.array(cfg.reach_point_offset_m)

    # Target position = reach point + radially-oscillating relative offset
    # (see SimConfig docstring) + a small independent jitter. This is
    # defined relative to the *reach point*, not the vessel origin, so
    # `approach_center_m`/`approach_amplitude_m` map directly onto the
    # distance values that matter for the opportunity predicate below.
    theta = np.deg2rad(cfg.approach_direction_deg)
    direction = np.array([np.cos(theta), np.sin(theta)])
    radial = cfg.approach_center_m + cfg.approach_amplitude_m * np.cos(
        2.0 * np.pi * t / cfg.approach_period_s
    )
    relative_offset = radial[:, None] * direction[None, :]
    target_jitter = cfg.wave_offset(
        t, cfg.target_wave_amplitudes_m, cfg.target_wave_periods_s, phase_seed=cfg.seed + 1000
    )
    target_pos = reach_pos + relative_offset + target_jitter

    rel_pos = target_pos - reach_pos
    distance = np.linalg.norm(rel_pos, axis=-1)
    # Finite-difference relative velocity (this is a *measurement* of the
    # already-realized trajectory, i.e. legitimately "current state" -- not
    # future information).
    target_vel = np.gradient(target_pos, cfg.dt, axis=0)
    reach_vel = np.gradient(reach_pos, cfg.dt, axis=0)
    rel_speed = np.linalg.norm(target_vel - reach_vel, axis=-1)

    reachable = distance < cfg.reach_radius_m
    good_opportunity = reachable & (rel_speed < cfg.max_relative_speed_mps)

    # Continuous [0, 1] score: representation (1) from rl-formulation.md's
    # Section 9 ("scalar opportunity score O(t)"), a smooth analytic stand-in.
    dist_term = _sigmoid((cfg.reach_radius_m - distance) / (0.25 * cfg.reach_radius_m))
    speed_term = _sigmoid(
        (cfg.max_relative_speed_mps - rel_speed) / (0.25 * cfg.max_relative_speed_mps)
    )
    opportunity_score = dist_term * speed_term

    state, events = _run_decision_policy(cfg, policy, t, good_opportunity, opportunity_score)

    return EpisodeResult(
        t=t,
        vessel_pos=vessel_pos,
        target_pos=target_pos,
        reach_pos=reach_pos,
        opportunity_score=opportunity_score,
        good_opportunity=good_opportunity,
        state=state,
        events=events,
    )


def _sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-x))


def _enable_interactive_backend() -> bool:
    """Switch matplotlib off the module-level "Agg" default so --show can
    actually open a window. Returns whether an interactive backend was
    found. Previously the module comment promised this switch but no code
    performed it, so --show silently did nothing (plt.show() is a no-op
    under Agg)."""
    for backend in ("QtAgg", "TkAgg", "GTK4Agg", "MacOSX"):
        try:
            matplotlib.use(backend, force=True)
        except Exception:  # noqa: BLE001 -- backend simply isn't installed
            continue
        return True
    print("--show requested but no interactive matplotlib backend is available; writing files only")
    return False


def _run_decision_policy(
    cfg: SimConfig,
    policy: Policy,
    t: np.ndarray,
    good_opportunity: np.ndarray,
    opportunity_score: np.ndarray,
) -> tuple[list[State], list[tuple[float, str]]]:
    """A compressed 3-state stand-in for the real 4-action space
    (WAIT/APPROACH/ATTEMPT_GRASP/ABORT) -- ACT here covers both APPROACH
    and ATTEMPT_GRASP. Decision rule per policy:

      reactive:  ACT the instant good_opportunity is true (B0).
      lookahead: ACT only once the current score is not clearly dominated
                 by what a short constant-velocity extrapolation predicts
                 for the near future -- i.e. wait for a *local* peak within
                 the horizon rather than the first acceptable instant.
    """
    n = len(t)
    state: list[State] = ["WAIT"] * n
    events: list[tuple[float, str]] = []
    act_start_idx: int | None = None
    resolved_idx: int | None = None  # index a SUCCESS/ABORT flash started, if any
    flash_hold_s = 1.0
    horizon_steps = max(1, int(cfg.lookahead_horizon_s / cfg.dt))

    cur: State = "WAIT"
    for i in range(n):
        if cur in ("SUCCESS", "ABORT"):
            assert resolved_idx is not None
            if t[i] - t[resolved_idx] >= flash_hold_s:
                cur = "WAIT"

        if cur == "WAIT":
            should_act = bool(good_opportunity[i])
            if should_act and policy == "lookahead":
                should_act = _is_local_peak(
                    opportunity_score, i, horizon_steps, cfg.lookahead_samples
                )
            if should_act:
                cur = "ACT"
                act_start_idx = i
                events.append((float(t[i]), f"{policy}: commit to grasp"))
        elif cur == "ACT":
            assert act_start_idx is not None
            elapsed = t[i] - t[act_start_idx]
            if not good_opportunity[i]:
                cur = "ABORT"
                resolved_idx = i
                events.append((float(t[i]), f"{policy}: abort (opportunity degraded)"))
            elif elapsed >= cfg.grasp_duration_s:
                cur = "SUCCESS"
                resolved_idx = i
                events.append((float(t[i]), f"{policy}: grasp succeeded"))

        state[i] = cur

    return state, events


def _is_local_peak(score: np.ndarray, i: int, horizon_steps: int, n_samples: int) -> bool:
    """True if the current score is within the top values of a short
    forward window.

    Be clear about what this does: it reads the TRUE future opportunity
    score straight out of the already-simulated trajectory, over a bounded
    horizon. That is future-information leakage, and it would be
    disqualifying in the real experiment -- the leakage rule (confound #7)
    forbids the M8 OpportunityEstimator from doing anything of the sort,
    and it doesn't: it extrapolates from current and past state only (see
    src/opportunity_rl/opportunity/estimator.py).

    It is acceptable *here only* because this script is an illustration of
    what a lookahead policy would do if its prediction were good, not a
    measurement of anything. Nothing it produces is a result. An earlier
    version of this docstring described the behaviour as "a stand-in for
    extrapolate current trend forward, not read the true future simulator
    state", which was not an accurate description of the code."""
    end = min(len(score), i + horizon_steps)
    window = score[i:end]
    if len(window) < 2:
        return True
    # Require current value to be within 5% of the window's max: i.e. don't
    # wait for a strictly better moment that never clearly arrives.
    return bool(score[i] >= 0.95 * np.max(window))


def render(
    reactive: EpisodeResult,
    lookahead: EpisodeResult,
    cfg: SimConfig,
    out_png: Path,
    out_video: Path | None,
    show: bool,
) -> None:
    state_colors = {"WAIT": "#9e9e9e", "ACT": "#ff9800", "SUCCESS": "#2e7d32", "ABORT": "#c62828"}

    fig, axes = plt.subplots(2, 2, figsize=(11, 6.8), height_ratios=[1.3, 1])
    fig.suptitle(
        "Opportunity-aware vs. reactive grasp timing under disturbance "
        f"(illustrative toy model, seed={cfg.seed}) -- not the M4/M8 implementation",
        fontsize=10,
        y=0.99,
    )

    top_axes = {"reactive": axes[0, 0], "lookahead": axes[0, 1]}
    bottom_axes = {"reactive": axes[1, 0], "lookahead": axes[1, 1]}
    results = {"reactive": reactive, "lookahead": lookahead}

    scatter_artists = {}
    reach_circle_artists = {}
    time_markers = {}

    for name, ax in top_axes.items():
        res = results[name]
        ax.set_title(f"{name} policy", fontsize=10)
        ax.plot(
            res.vessel_pos[:, 0],
            res.vessel_pos[:, 1],
            color="#1565c0",
            lw=1,
            alpha=0.5,
            label="vessel path",
        )
        ax.plot(
            res.target_pos[:, 0],
            res.target_pos[:, 1],
            color="#6a1b9a",
            lw=1,
            alpha=0.5,
            label="target path",
        )
        (target_dot,) = ax.plot([], [], "o", color="#6a1b9a", ms=9, label="target")
        (reach_dot,) = ax.plot([], [], "s", color="#1565c0", ms=8, label="reach point")
        circle = plt.Circle(
            (0, 0), cfg.reach_radius_m, fill=False, ls="--", color="#1565c0", alpha=0.6
        )
        ax.add_patch(circle)
        ax.set_aspect("equal")
        ax.legend(loc="upper left", fontsize=7, framealpha=0.8)
        scatter_artists[name] = (target_dot, reach_dot)
        reach_circle_artists[name] = circle

        bax = bottom_axes[name]
        bax.plot(res.t, res.opportunity_score, color="black", lw=1)
        bax.axhline(0.5, color="gray", lw=0.5, ls=":")
        bax.set_ylim(0, 1.05)
        bax.set_xlabel("time [s]")
        bax.set_ylabel("opportunity score")
        state_arr = np.array(res.state)
        for state_name, color in state_colors.items():
            mask = state_arr == state_name
            bax.fill_between(res.t, 0, 1.05, where=mask, color=color, alpha=0.25, step="mid")
        (tmark,) = bax.plot([], [], color="red", lw=1)
        time_markers[name] = tmark

        for et, desc in res.events:
            if "commit" in desc:
                bax.axvline(et, color="#ff9800", lw=0.8, alpha=0.7)

    handles = [plt.Line2D([0], [0], color=c, lw=6, alpha=0.4) for c in state_colors.values()]
    fig.legend(
        handles,
        list(state_colors.keys()),
        loc="lower center",
        ncol=4,
        fontsize=8,
        bbox_to_anchor=(0.5, 0.0),
    )
    fig.subplots_adjust(top=0.90, bottom=0.10, hspace=0.3, wspace=0.22, left=0.06, right=0.98)

    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=150)
    print(f"Saved static summary: {out_png}")

    if out_video is not None:
        n_frames = len(reactive.t)
        stride = max(1, n_frames // 400)  # cap frame count for a reasonable render time

        def update(frame_idx: int):
            i = frame_idx * stride
            artists = []
            for name, res in results.items():
                target_dot, reach_dot = scatter_artists[name]
                target_dot.set_data([res.target_pos[i, 0]], [res.target_pos[i, 1]])
                reach_dot.set_data([res.reach_pos[i, 0]], [res.reach_pos[i, 1]])
                reach_circle_artists[name].center = tuple(res.reach_pos[i])
                target_dot.set_color(state_colors[res.state[i]])
                time_markers[name].set_data([res.t[i], res.t[i]], [0, 1.05])
                artists += [target_dot, reach_dot, time_markers[name]]
            return artists

        ani = animation.FuncAnimation(
            fig, update, frames=n_frames // stride, interval=50, blit=False
        )
        out_video.parent.mkdir(parents=True, exist_ok=True)
        # Uses the system `ffmpeg` binary via matplotlib's FFMpegWriter --
        # same tool this repo already relies on for the M1 capture videos
        # (docs/environment.md), not an extra Python animation dependency.
        writer = animation.FFMpegWriter(
            fps=20, codec="libx264", extra_args=["-pix_fmt", "yuv420p", "-crf", "20"]
        )
        ani.save(out_video, writer=writer)
        print(f"Saved animation: {out_video}")

    if show:
        plt.show()
    plt.close(fig)


def summarize(name: str, res: EpisodeResult) -> str:
    n_success = sum(1 for s in res.state if s == "SUCCESS")
    n_abort = sum(1 for s in res.state if s == "ABORT")
    first_commit = next((t for t, d in res.events if "commit" in d), None)
    return (
        f"{name:>10}: successes={n_success} aborts={n_abort} "
        f"first_commit_t={first_commit if first_commit is not None else 'never'}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--duration", type=float, default=40.0, help="episode duration [s]")
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("docs/images"),
        help="where to write the static summary (and video, if --video)",
    )
    parser.add_argument(
        "--video", action="store_true", help="also render an animated mp4 (requires ffmpeg on PATH)"
    )
    parser.add_argument("--show", action="store_true", help="open an interactive window")
    args = parser.parse_args()

    show = _enable_interactive_backend() if args.show else False

    cfg = SimConfig(duration_s=args.duration, seed=args.seed)
    reactive = simulate(cfg, "reactive")
    lookahead = simulate(cfg, "lookahead")

    print(dataclasses.asdict(cfg))
    print(summarize("reactive", reactive))
    print(summarize("lookahead", lookahead))

    out_png = args.out_dir / f"opportunity_concept_seed{args.seed}.png"
    out_video = (args.out_dir / f"opportunity_concept_seed{args.seed}.mp4") if args.video else None
    render(reactive, lookahead, cfg, out_png, out_video, show)


if __name__ == "__main__":
    main()
