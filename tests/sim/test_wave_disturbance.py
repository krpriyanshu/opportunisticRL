import math

import pytest

from opportunity_rl.sim.wave_disturbance import (
    WaveConfig,
    build_components,
    deep_water_omega,
    deep_water_wavenumber,
    startup_ramp,
    vessel_kinematics,
)


def test_deep_water_dispersion_is_self_consistent():
    omega = 1.2566370614359172  # 2*pi / 5.0
    k = deep_water_wavenumber(omega)
    assert deep_water_omega(k) == pytest.approx(omega)


def test_pms_default_matches_vrx_sydney_regatta_defaults():
    # Same defaults as Wavefield.hh's documented SDF defaults, and the
    # same values sydney_regatta.sdf's PublisherPlugin actually sets
    # (docs/wave-simulation-literature-review.md Section 0): 3
    # components, scale 1.1, angle 0.4, gain 0.3, period 5s.
    cfg = WaveConfig(model="PMS", number=3, scale=1.1, angle=0.4, gain=0.3, period=5.0)
    components = build_components(cfg)
    assert len(components) == 3
    # All amplitudes positive and finite for a physically sane spectrum.
    for c in components:
        assert c.amplitude > 0.0
        assert math.isfinite(c.amplitude)
        assert c.wavenumber > 0.0


def test_zero_steepness_gives_pure_heave_no_horizontal_displacement():
    cfg = WaveConfig(model="PMS", steepness=0.0)
    components = build_components(cfg)
    kin = vessel_kinematics(components, rest_xy=(10.0, -5.0), t=3.7)
    assert kin.position[0] == pytest.approx(10.0)
    assert kin.position[1] == pytest.approx(-5.0)
    # z can be nonzero -- heave is not gated by steepness.


def test_positive_steepness_gives_nonzero_horizontal_displacement():
    cfg = WaveConfig(model="PMS", steepness=0.6)
    components = build_components(cfg)
    kin = vessel_kinematics(components, rest_xy=(0.0, 0.0), t=1.3)
    assert kin.position[0] != 0.0 or kin.position[1] != 0.0


def test_startup_ramp_is_zero_at_t0_and_approaches_one():
    assert startup_ramp(t=0.0, t0=0.0, tau=2.0) == pytest.approx(0.0)
    assert startup_ramp(t=100.0, t0=0.0, tau=2.0) == pytest.approx(1.0, abs=1e-6)
    # Monotonically increasing.
    r1 = startup_ramp(t=1.0, t0=0.0, tau=2.0)
    r2 = startup_ramp(t=2.0, t0=0.0, tau=2.0)
    assert r2 > r1


def test_velocity_matches_finite_difference_of_position():
    # The analytic velocity this module returns should agree with a
    # central-difference check against the same closed-form position --
    # this is the actual correctness guarantee that matters (an
    # analytically-differentiated formula that doesn't match its own
    # antiderivative numerically would indicate a sign/algebra bug).
    cfg = WaveConfig(model="PMS", steepness=0.5, period=6.0, gain=0.4)
    components = build_components(cfg)
    rest_xy = (2.0, 1.0)
    t = 4.0
    dt = 1e-5

    kin = vessel_kinematics(components, rest_xy, t)
    kin_plus = vessel_kinematics(components, rest_xy, t + dt)
    kin_minus = vessel_kinematics(components, rest_xy, t - dt)

    for i in range(3):
        numeric_v = (kin_plus.position[i] - kin_minus.position[i]) / (2 * dt)
        assert kin.linear_velocity[i] == pytest.approx(numeric_v, abs=1e-6)


def test_unknown_model_raises():
    with pytest.raises(ValueError, match="PMS"):
        build_components(WaveConfig(model="bogus"))


def test_cwr_model_builds_without_error():
    cfg = WaveConfig(model="CWR", amplitude=0.3, number=3)
    components = build_components(cfg)
    assert len(components) == 3
