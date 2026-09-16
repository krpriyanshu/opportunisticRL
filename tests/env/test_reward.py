"""Reward tests -- plain pytest, no gymnasium/torch needed."""

from opportunity_rl.env.reward import RewardConfig, compute_reward


def test_success_dominates_a_single_step():
    cfg = RewardConfig()
    r = compute_reward(
        cfg=cfg,
        grasp_succeeded=True,
        grasp_failed=False,
        collided=False,
        ee_motion_magnitude=0.0,
    )
    assert r.success == cfg.r_success
    assert r.total > 0


def test_time_cost_charged_every_step_even_on_wait():
    cfg = RewardConfig()
    # A "WAIT" step: nothing happens, no motion, no grasp.
    r = compute_reward(
        cfg=cfg,
        grasp_succeeded=False,
        grasp_failed=False,
        collided=False,
        ee_motion_magnitude=0.0,
    )
    assert r.time == cfg.lambda_time
    assert r.total < 0  # pure cost, no reward


def test_indefinite_waiting_is_worse_than_succeeding():
    # rl-formulation.md's binding requirement: an all-WAIT episode's
    # return must be strictly less than a successful episode's. Simulate
    # 100 WAIT steps vs one immediate success.
    cfg = RewardConfig()
    wait_return = sum(
        compute_reward(
            cfg=cfg,
            grasp_succeeded=False,
            grasp_failed=False,
            collided=False,
            ee_motion_magnitude=0.0,
        ).total
        for _ in range(100)
    )
    success_return = compute_reward(
        cfg=cfg,
        grasp_succeeded=True,
        grasp_failed=False,
        collided=False,
        ee_motion_magnitude=0.0,
    ).total
    assert wait_return < success_return


def test_failed_grasp_penalized_more_than_waiting():
    cfg = RewardConfig()
    failed = compute_reward(
        cfg=cfg,
        grasp_succeeded=False,
        grasp_failed=True,
        collided=False,
        ee_motion_magnitude=0.0,
    ).total
    waited = compute_reward(
        cfg=cfg,
        grasp_succeeded=False,
        grasp_failed=False,
        collided=False,
        ee_motion_magnitude=0.0,
    ).total
    assert failed < waited


def test_collision_is_the_heaviest_penalty_by_default():
    cfg = RewardConfig()
    r = compute_reward(
        cfg=cfg,
        grasp_succeeded=False,
        grasp_failed=False,
        collided=True,
        ee_motion_magnitude=0.0,
    )
    assert r.collision == cfg.lambda_collision


def test_components_sum_to_total():
    cfg = RewardConfig()
    r = compute_reward(
        cfg=cfg,
        grasp_succeeded=True,
        grasp_failed=False,
        collided=True,
        ee_motion_magnitude=0.5,
    )
    expected = r.success - r.failure - r.collision - r.time - r.motion
    assert r.total == expected
