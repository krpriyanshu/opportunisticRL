"""Vessel wave disturbance kinematics for the fast simulator backend.

This is a from-scratch, pure-Python port of the *same* trochoidal
(Gerstner) multi-component wave model VRX actually runs
(`ros2_ws/src/vrx/vrx_gz/src/Wavefield.cc`, a port of
`asv_wave_sim` by Rhys Mainwaring) -- not the illustrative toy model in
scripts/visualize_opportunity_concept.py, and not a new invention. Same
parameter names as VRX's own SDF `<wave>` element (model, number, scale,
angle, steepness, period, phase, direction, gain, tau) on purpose, so a
disturbance config here is directly comparable to a VRX world's wave
settings -- see docs/wave-simulation-literature-review.md for the full
derivation and the primary-source verification behind this model.

Deliberate difference from VRX's own implementation, and why it's fine:
VRX's `Wavefield` solves an *inverse* problem (given a world (x, y),
find the Gerstner-displaced surface point that lands there -- needed to
answer "how high is the water under this fixed point on the map", via a
Newton solver) because it renders a spatially-varying ocean mesh. This
module only needs the *forward* problem -- "where is the one floating
body whose rest position is (x0, y0) right now" -- which is the direct,
closed-form Gerstner parametrization with no iterative solve required,
and (a genuine advantage over the ROS2/Gazebo backend, which needed the
finite-difference workaround in docs/ros2.md) gives exact analytic
velocity by differentiating the same closed form, not by differencing
noisy samples.

Confidence note, in the interest of not overclaiming: the vertical
(height) term below is verified against `Wavefield::ComputeDepthSimply`
directly and matches it exactly. The steepness-scaled horizontal
(trochoidal) displacement term is the standard textbook Gerstner-wave
formula, consistent with VRX's own documented parameter semantics
("steepness in [0, 1], 0 = sine wave, 1 = full Gerstner trochoid") --
it is not claimed to be a bit-exact reproduction of the internal Newton
solver in `Wavefield::ComputeDepthDirectly`, which was not re-derived
line-by-line from memory here. If the two backends are ever compared
quantitatively (not just qualitatively), this is the first place to
re-verify.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

G = 9.81  # m/s^2, matches Wavefield.cc's DeepWaterDispersion* constant


@dataclass(frozen=True)
class WaveComponent:
    amplitude: float
    wavenumber: float
    angular_frequency: float
    phase: float
    steepness: float  # per-component q, in [0, 1]
    direction: tuple[float, float]  # unit vector


@dataclass(frozen=True)
class WaveConfig:
    """Same fields, names, and defaults as VRX's SDF `<wave>` element
    (Wavefield.hh's documented defaults) so this can be read directly
    off a VRX world file. `model` selects the component-generation
    method: "PMS" (Pierson-Moskowitz spectrum sampling) or "CWR"
    (constant wavelength-to-amplitude ratio, a non-physical geometric
    scaling -- see the literature review for why these differ)."""

    model: str = "PMS"
    number: int = 3
    scale: float = 1.1
    angle: float = 0.4
    steepness: float = 0.0
    amplitude: float = 0.0  # only used by CWR
    period: float = 5.0
    phase: float = 0.0
    direction: float = 0.0  # radians
    gain: float = 1.0  # only used by PMS
    tau: float = 2.0  # startup ramp time constant, seconds


def deep_water_wavenumber(omega: float) -> float:
    return omega * omega / G


def deep_water_omega(wavenumber: float) -> float:
    return math.sqrt(G * wavenumber)


def _pierson_moskowitz(omega: float, omega_p: float) -> float:
    alpha = 0.0081
    return alpha * G**2 / omega**5 * math.exp(-1.25 * (omega_p / omega) ** 4)


def _components_cwr(cfg: WaveConfig) -> list[WaveComponent]:
    omega_p = 2.0 * math.pi / cfg.period
    k_p = deep_water_wavenumber(omega_p)
    components = []
    for i in range(cfg.number):
        n = i - cfg.number // 2
        scale_factor = cfg.scale**n
        a = scale_factor * cfg.amplitude
        k = k_p / scale_factor
        omega = deep_water_omega(k)
        q = min(1.0, cfg.steepness / (a * k * cfg.number)) if a != 0.0 else 0.0
        theta = n * cfg.angle + cfg.direction
        components.append(
            WaveComponent(a, k, omega, cfg.phase, q, (math.cos(theta), math.sin(theta)))
        )
    return components


def _components_pms(cfg: WaveConfig) -> list[WaveComponent]:
    omega_p = 2.0 * math.pi / cfg.period
    omega_spacing = [
        omega_p * (1.0 - 1.0 / cfg.scale),
        omega_p * (cfg.scale - 1.0 / cfg.scale) / 2.0,
        omega_p * (cfg.scale - 1.0),
    ]
    components = []
    for i in range(cfg.number):
        n = i - 1
        scale_factor = cfg.scale**n
        omega = omega_p * scale_factor
        pm = _pierson_moskowitz(omega, omega_p)
        spacing = omega_spacing[i] if i < len(omega_spacing) else omega_spacing[-1]
        a = cfg.gain * math.sqrt(2.0 * pm * spacing)
        k = deep_water_wavenumber(omega)
        q = min(1.0, cfg.steepness / (a * k * cfg.number)) if a != 0.0 else 0.0
        theta = n * cfg.angle + cfg.direction
        components.append(
            WaveComponent(a, k, omega, cfg.phase, q, (math.cos(theta), math.sin(theta)))
        )
    return components


def build_components(cfg: WaveConfig) -> list[WaveComponent]:
    if cfg.model == "PMS":
        return _components_pms(cfg)
    if cfg.model == "CWR":
        return _components_cwr(cfg)
    raise ValueError(f"unknown wave model {cfg.model!r}, expected 'PMS' or 'CWR'")


@dataclass(frozen=True)
class VesselKinematics:
    position: tuple[float, float, float]
    linear_velocity: tuple[float, float, float]
    roll: float
    pitch: float


def startup_ramp(t: float, t0: float, tau: float) -> float:
    """Matches Wavefield.cc's exponential wave startup: waves grow in
    smoothly from a flat sea rather than snapping on at full amplitude."""
    if tau <= 0.0:
        return 1.0
    return 1.0 - math.exp(-(t - t0) / tau)


def vessel_kinematics(
    components: list[WaveComponent],
    rest_xy: tuple[float, float],
    t: float,
    t0: float = 0.0,
    tau: float = 2.0,
    slope_to_angle_gain: float = 1.0,
    drift_xy: tuple[float, float] = (0.0, 0.0),
) -> VesselKinematics:
    """Closed-form Gerstner displacement/velocity of a floating body whose
    rest (undisturbed) horizontal position is `rest_xy` *at t=0* and
    drifts at constant `drift_xy` (e.g. current-driven set/drift) --
    i.e. the true rest position at time t is `rest_xy + drift_xy * t`.
    See module docstring for the forward-vs-inverse-problem distinction
    from VRX's own `Wavefield::ComputeDepthDirectly`.

    A drifting rest position means the wave phase `theta` seen by the
    body has a *different* total time-derivative than a stationary
    body's (a real physical Doppler shift from moving through the wave
    field, not a modeling artifact): d(theta)/dt = k*dot(drift,dir) -
    omega, not just -omega. `omega_eff` below applies exactly that
    correction. Caught by test_velocity_matches_finite_difference_with_drift_and_waves
    disagreeing with a numeric check by a small but real, systematic
    amount when a first version of target_dynamics.py tried to add
    drift on top of this function's output externally instead of
    accounting for it here.
    """
    x0, y0 = rest_xy
    drift_x, drift_y = drift_xy
    rx = x0 + drift_x * t
    ry = y0 + drift_y * t
    ramp = startup_ramp(t, t0, tau)
    # d(ramp)/dt: needed because ramp itself varies with t, so the
    # product rule applies to velocity = d/dt[displacement(t) * ramp(t)]
    # -- a first implementation omitted this term (raw_velocity * ramp
    # only) and was caught by test_velocity_matches_finite_difference_of_position
    # disagreeing with a numeric check by several percent at moderate t,
    # not by floating-point noise.
    ramp_dot = math.exp(-(t - t0) / tau) / tau if tau > 0.0 else 0.0

    dx = dy = dz = 0.0
    vx = vy = vz = 0.0
    dzdx = dzdy = 0.0
    for c in components:
        dot = rx * c.direction[0] + ry * c.direction[1]
        theta = c.wavenumber * dot - c.angular_frequency * t + c.phase
        s, cosv = math.sin(theta), math.cos(theta)
        drift_dot_dir = drift_x * c.direction[0] + drift_y * c.direction[1]
        omega_eff = c.angular_frequency - c.wavenumber * drift_dot_dir

        dx += -c.steepness * c.amplitude * c.direction[0] * s
        dy += -c.steepness * c.amplitude * c.direction[1] * s
        dz += c.amplitude * cosv

        vx += c.steepness * c.amplitude * c.direction[0] * omega_eff * cosv
        vy += c.steepness * c.amplitude * c.direction[1] * omega_eff * cosv
        vz += c.amplitude * omega_eff * s

        dzdx += -c.amplitude * c.wavenumber * c.direction[0] * s
        dzdy += -c.amplitude * c.wavenumber * c.direction[1] * s

    position = (rx + dx * ramp, ry + dy * ramp, dz * ramp)
    velocity = (
        drift_x + vx * ramp + dx * ramp_dot,
        drift_y + vy * ramp + dy * ramp_dot,
        vz * ramp + dz * ramp_dot,
    )
    # Small-angle proxy for wave-slope-induced roll/pitch -- a secondary
    # effect (position/velocity are the primary research-relevant
    # disturbance), not claimed to match VRX's own (unmodeled here)
    # hydrodynamic roll/pitch response.
    roll = -slope_to_angle_gain * dzdy * ramp
    pitch = slope_to_angle_gain * dzdx * ramp
    return VesselKinematics(position=position, linear_velocity=velocity, roll=roll, pitch=pitch)


@dataclass
class WaveDisturbance:
    """Stateful convenience wrapper: build once from a config, then query
    at any simulation time. Holds no mutable state itself besides the
    precomputed components -- `vessel_kinematics_at` is a pure function
    of (rest_xy, t)."""

    config: WaveConfig
    t0: float = 0.0
    components: list[WaveComponent] = field(init=False)

    def __post_init__(self) -> None:
        self.components = build_components(self.config)

    def vessel_kinematics_at(self, rest_xy: tuple[float, float], t: float) -> VesselKinematics:
        return vessel_kinematics(self.components, rest_xy, t, self.t0, self.config.tau)
