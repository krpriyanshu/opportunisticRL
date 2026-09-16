"""Dynamic target motion (Milestone M4 -- the target is no longer
static, unlike M3's). The target is modeled as a small floating body:
a constant drift (current) plus its own wave response, reusing
wave_disturbance's Gerstner kinematics rather than a separate model,
since the target is physically the same kind of thing as the vessel
(a floating body in the same sea) just smaller and independently
positioned -- not because a moving target and a wave-bobbing vessel are
conceptually the same thing (they are not, and the target's motion is
never conditioned on the vessel's, since the whole research question
hinges on the target's motion being independent of the platform's, per
docs/research-question.md).

Not claimed to be a physically complete model of a floating object's
hydrodynamic response (that would require its own added-mass/damping
treatment, out of scope here) -- it is a disturbance-shaped stand-in
deliberately built from the *same*, already-tested primitive
(wave_disturbance.vessel_kinematics, including its drift handling) as
the vessel, so that "the target also experiences wave motion, just with
its own drift and response amplitude" is represented honestly rather
than the target moving in some unrelated, arbitrary way.
"""

from __future__ import annotations

from dataclasses import dataclass

from opportunity_rl.sim.state import Vec3
from opportunity_rl.sim.wave_disturbance import (
    WaveConfig,
    build_components,
    vessel_kinematics,
)


@dataclass(frozen=True)
class TargetConfig:
    start_position: Vec3
    drift_velocity: Vec3 = (0.0, 0.0, 0.0)  # m/s, constant (current-driven)
    wave: WaveConfig = WaveConfig(number=2, gain=0.15, period=4.0, steepness=0.2)


@dataclass
class TargetTrajectory:
    config: TargetConfig
    t0: float = 0.0

    def __post_init__(self) -> None:
        self._components = build_components(self.config.wave)

    def state_at(self, t: float) -> tuple[Vec3, Vec3]:
        """Returns (position, linear_velocity) at time t, world frame."""
        drift = self.config.drift_velocity
        kin = vessel_kinematics(
            self._components,
            rest_xy=(self.config.start_position[0], self.config.start_position[1]),
            t=t,
            t0=self.t0,
            tau=self.config.wave.tau,
            drift_xy=(drift[0], drift[1]),
        )
        position = (
            kin.position[0],
            kin.position[1],
            self.config.start_position[2] + drift[2] * t + kin.position[2],
        )
        velocity = (
            kin.linear_velocity[0],
            kin.linear_velocity[1],
            kin.linear_velocity[2] + drift[2],
        )
        return position, velocity
