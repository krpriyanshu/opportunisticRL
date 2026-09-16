"""Loads a disturbance config (vessel wave params + target dynamics)
from a YAML file in configs/disturbances/. This is the "shared
configs/disturbances/*.yaml files referenced by name" mechanism named
in docs/architecture.md's confound register (item #4) -- one config per
disturbance condition, loaded by name, never redefined inline per
experiment.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from opportunity_rl.sim.target_dynamics import TargetConfig
from opportunity_rl.sim.wave_disturbance import WaveConfig

REPO_ROOT = Path(__file__).resolve().parents[3]
DISTURBANCE_CONFIG_DIR = REPO_ROOT / "configs" / "disturbances"


@dataclass(frozen=True)
class DisturbanceConfig:
    name: str
    vessel_wave: WaveConfig
    target: TargetConfig


def _wave_config_from_dict(d: dict[str, Any]) -> WaveConfig:
    return WaveConfig(**d)


def _target_config_from_dict(d: dict[str, Any]) -> TargetConfig:
    d = dict(d)
    wave_dict = d.pop("wave", None)
    wave = _wave_config_from_dict(wave_dict) if wave_dict is not None else WaveConfig()
    d["start_position"] = tuple(d["start_position"])
    if "drift_velocity" in d:
        d["drift_velocity"] = tuple(d["drift_velocity"])
    return TargetConfig(wave=wave, **d)


def load_disturbance_config(name: str, directory: Path | None = None) -> DisturbanceConfig:
    """Load `<directory>/<name>.yaml` (default directory:
    configs/disturbances/). Raises FileNotFoundError with the searched
    path spelled out if the name doesn't exist -- a wrong config name is
    a research-integrity bug (silently falling back to defaults would
    make a run's config claim untrustworthy), so this fails loudly
    rather than substituting a default."""
    directory = directory or DISTURBANCE_CONFIG_DIR
    path = directory / f"{name}.yaml"
    if not path.exists():
        raise FileNotFoundError(f"no disturbance config named {name!r} at {path}")

    with path.open() as f:
        raw = yaml.safe_load(f)

    return DisturbanceConfig(
        name=name,
        vessel_wave=_wave_config_from_dict(raw["vessel_wave"]),
        target=_target_config_from_dict(raw["target"]),
    )
