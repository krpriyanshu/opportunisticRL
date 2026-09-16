from opportunity_rl.sim.episode import (
    EpisodeConfig,
    FeasibilityConfig,
    check_termination,
    evaluate_feasibility,
)
from opportunity_rl.sim.state import RobotState


def _robot(ee_position=(0.0, 0.0, 0.0)):
    return RobotState(
        ee_position=ee_position,
        ee_orientation=(0.0, 0.0, 0.0, 1.0),
        joint_positions=(0.0,) * 7,
        joint_velocities=(0.0,) * 7,
    )


def test_good_opportunity_true_when_close_and_slow():
    cfg = FeasibilityConfig(workspace_radius_m=0.85, max_relative_speed_mps=0.3)
    f = evaluate_feasibility(
        _robot(),
        target_relative_position=(0.2, 0.0, 0.0),
        target_relative_velocity=(0.05, 0.0, 0.0),
        cfg=cfg,
    )
    assert f.reachable
    assert f.good_opportunity


def test_good_opportunity_false_when_out_of_reach():
    cfg = FeasibilityConfig(workspace_radius_m=0.85, max_relative_speed_mps=0.3)
    f = evaluate_feasibility(
        _robot(),
        target_relative_position=(2.0, 0.0, 0.0),
        target_relative_velocity=(0.0, 0.0, 0.0),
        cfg=cfg,
    )
    assert not f.reachable
    assert not f.good_opportunity


def test_good_opportunity_false_when_too_fast_even_if_reachable():
    cfg = FeasibilityConfig(workspace_radius_m=0.85, max_relative_speed_mps=0.1)
    f = evaluate_feasibility(
        _robot(),
        target_relative_position=(0.1, 0.0, 0.0),
        target_relative_velocity=(1.0, 0.0, 0.0),
        cfg=cfg,
    )
    assert f.reachable
    assert not f.good_opportunity


def test_success_takes_priority_over_everything_else():
    ep_cfg = EpisodeConfig(max_duration_s=1.0, target_lost_radius_m=1.0)
    f = evaluate_feasibility(
        _robot(),
        target_relative_position=(5.0, 0.0, 0.0),
        target_relative_velocity=(0.0, 0.0, 0.0),
        cfg=FeasibilityConfig(),
    )
    reason = check_termination(
        sim_time=100.0,
        episode_start_time=0.0,
        feasibility=f,
        episode_cfg=ep_cfg,
        grasp_succeeded=True,
        collided=True,
        irrecoverable=True,
    )
    assert reason == "success"


def test_termination_reason_is_always_one_of_the_fixed_set():
    from opportunity_rl.sim.episode import TERMINATION_REASONS

    ep_cfg = EpisodeConfig(max_duration_s=1.0, target_lost_radius_m=1.0)
    f = evaluate_feasibility(
        _robot(),
        target_relative_position=(0.1, 0.0, 0.0),
        target_relative_velocity=(0.0, 0.0, 0.0),
        cfg=FeasibilityConfig(),
    )
    reason = check_termination(
        sim_time=0.5,
        episode_start_time=0.0,
        feasibility=f,
        episode_cfg=ep_cfg,
        grasp_succeeded=False,
        collided=False,
        irrecoverable=False,
    )
    assert reason is None or reason in TERMINATION_REASONS


def test_timeout_when_duration_exceeded_and_nothing_else_true():
    ep_cfg = EpisodeConfig(max_duration_s=10.0, target_lost_radius_m=5.0)
    f = evaluate_feasibility(
        _robot(),
        target_relative_position=(0.1, 0.0, 0.0),
        target_relative_velocity=(0.0, 0.0, 0.0),
        cfg=FeasibilityConfig(),
    )
    reason = check_termination(
        sim_time=15.0,
        episode_start_time=0.0,
        feasibility=f,
        episode_cfg=ep_cfg,
        grasp_succeeded=False,
        collided=False,
        irrecoverable=False,
    )
    assert reason == "timeout"


def test_target_lost_when_far_before_timeout():
    ep_cfg = EpisodeConfig(max_duration_s=60.0, target_lost_radius_m=1.0)
    f = evaluate_feasibility(
        _robot(),
        target_relative_position=(5.0, 0.0, 0.0),
        target_relative_velocity=(0.0, 0.0, 0.0),
        cfg=FeasibilityConfig(),
    )
    reason = check_termination(
        sim_time=1.0,
        episode_start_time=0.0,
        feasibility=f,
        episode_cfg=ep_cfg,
        grasp_succeeded=False,
        collided=False,
        irrecoverable=False,
    )
    assert reason == "target_lost"


def test_none_when_episode_should_continue():
    ep_cfg = EpisodeConfig(max_duration_s=60.0, target_lost_radius_m=5.0)
    f = evaluate_feasibility(
        _robot(),
        target_relative_position=(0.2, 0.0, 0.0),
        target_relative_velocity=(0.0, 0.0, 0.0),
        cfg=FeasibilityConfig(),
    )
    reason = check_termination(
        sim_time=1.0,
        episode_start_time=0.0,
        feasibility=f,
        episode_cfg=ep_cfg,
        grasp_succeeded=False,
        collided=False,
        irrecoverable=False,
    )
    assert reason is None
