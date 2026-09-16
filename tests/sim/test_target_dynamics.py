import pytest

from opportunity_rl.sim.target_dynamics import TargetConfig, TargetTrajectory
from opportunity_rl.sim.wave_disturbance import WaveConfig


def test_zero_drift_zero_wave_target_stays_at_start():
    cfg = TargetConfig(
        start_position=(5.0, -2.0, 0.5),
        drift_velocity=(0.0, 0.0, 0.0),
        wave=WaveConfig(gain=0.0, amplitude=0.0),
    )
    traj = TargetTrajectory(cfg)
    pos, vel = traj.state_at(10.0)
    assert pos == pytest.approx((5.0, -2.0, 0.5))
    assert vel == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)


def test_pure_drift_no_wave_moves_linearly():
    cfg = TargetConfig(
        start_position=(0.0, 0.0, 0.0),
        drift_velocity=(0.1, -0.05, 0.0),
        wave=WaveConfig(gain=0.0, amplitude=0.0),
    )
    traj = TargetTrajectory(cfg)
    pos, vel = traj.state_at(20.0)
    assert pos == pytest.approx((2.0, -1.0, 0.0))
    assert vel == pytest.approx((0.1, -0.05, 0.0))


def test_velocity_matches_finite_difference_with_drift_and_waves():
    # The interesting case: drift makes the wave's own rest position
    # time-varying, so the analytic velocity must account for the chain
    # rule through that, not just the wave's own explicit t-dependence.
    cfg = TargetConfig(
        start_position=(3.0, -1.0, 0.2),
        drift_velocity=(0.08, 0.05, 0.0),
        wave=WaveConfig(number=2, gain=0.2, period=4.0, steepness=0.3),
    )
    traj = TargetTrajectory(cfg)
    t = 6.0
    dt = 1e-5

    pos, vel = traj.state_at(t)
    pos_plus, _ = traj.state_at(t + dt)
    pos_minus, _ = traj.state_at(t - dt)

    for i in range(3):
        numeric_v = (pos_plus[i] - pos_minus[i]) / (2 * dt)
        assert vel[i] == pytest.approx(numeric_v, abs=1e-6), f"axis {i}"
