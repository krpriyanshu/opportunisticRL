"""The frozen `RLAlgorithm` Protocol from docs/rl-formulation.md's
Interfaces section. Like SimulationInterface, promoted here from a
docs-only spec to a real typing.Protocol so any algorithm (B2's PPO
now, B3/B4 later) is structurally checked to conform.

    class RLAlgorithm(Protocol):
        def train(self, total_steps: int) -> None: ...
        def evaluate(self, n_episodes: int, seed: int) -> EvaluationResult: ...
        def save_checkpoint(self, path: Path) -> None: ...
        def load_checkpoint(self, path: Path) -> None: ...
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol


@dataclass(frozen=True)
class EvaluationResult:
    """What `evaluate` reports. Success rate is the primary H1 metric;
    the per-termination-reason breakdown feeds failure analysis
    (rl-formulation.md Section 41). Kept deliberately small and
    algorithm-agnostic."""

    n_episodes: int
    success_rate: float
    mean_return: float
    mean_episode_length: float
    termination_reason_counts: dict[str, int] = field(default_factory=dict)


class RLAlgorithm(Protocol):
    def train(self, total_steps: int) -> None: ...
    def evaluate(self, n_episodes: int, seed: int) -> EvaluationResult: ...
    def save_checkpoint(self, path: Path) -> None: ...
    def load_checkpoint(self, path: Path) -> None: ...
