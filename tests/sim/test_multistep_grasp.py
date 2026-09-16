"""Multi-step committed grasp maneuver tests (grasp_duration_steps > 1).
Numpy-only, no gymnasium/torch -- drives FastSimulator directly."""

from opportunity_rl.sim.disturbance_config import DisturbanceConfig
from opportunity_rl.sim.fast_simulator import (
    ATTEMPT_GRASP,
    DEFAULT_STANDBY_POSITION,
    WAIT,
    FastSimulator,
)
from opportunity_rl.sim.target_dynamics import TargetConfig
from opportunity_rl.sim.wave_disturbance import WaveConfig


def _still_target_sim(grasp_duration_steps):
    # Target parked at the standby EE position, no wave motion: it is a
    # good opportunity that PERSISTS indefinitely, so a committed
    # maneuver of any length should complete.
    cfg = DisturbanceConfig(
        name="still",
        vessel_wave=WaveConfig(gain=0.0, amplitude=0.0),
        target=TargetConfig(
            start_position=DEFAULT_STANDBY_POSITION,
            drift_velocity=(0.0, 0.0, 0.0),
            wave=WaveConfig(gain=0.0, amplitude=0.0),
        ),
    )
    return FastSimulator(cfg, grasp_duration_steps=grasp_duration_steps)


def test_multistep_grasp_completes_when_window_persists():
    sim = _still_target_sim(grasp_duration_steps=3)
    sim.reset(seed=0)
    # Commit; the maneuver takes 3 steps.
    sim.apply_action(ATTEMPT_GRASP)
    sim.step(0.5)
    assert not sim.get_robot_state().gripper_holding_target  # not done yet
    sim.apply_action(WAIT)  # committed: action ignored
    sim.step(0.5)
    assert not sim.get_robot_state().gripper_holding_target  # still in progress
    sim.apply_action(WAIT)
    sim.step(0.5)
    assert sim.get_robot_state().gripper_holding_target  # completed on 3rd step


def test_instant_grasp_default_still_one_step():
    # grasp_duration_steps=1 (default) preserves M6 behaviour: one-step
    # success.
    sim = _still_target_sim(grasp_duration_steps=1)
    sim.reset(seed=0)
    sim.apply_action(ATTEMPT_GRASP)
    assert sim.get_robot_state().gripper_holding_target


def test_multistep_grasp_fails_if_window_closes_midway():
    # A target that is a good opportunity now but whose window is shorter
    # than the maneuver: use a fast-oscillating target so good_opportunity
    # flips within the maneuver. We assert that IF good_opportunity is
    # false at some committed step, the maneuver reports a failure and
    # does not latch.
    cfg = DisturbanceConfig(
        name="brief",
        vessel_wave=WaveConfig(gain=0.0, amplitude=0.0),
        target=TargetConfig(
            start_position=DEFAULT_STANDBY_POSITION,
            drift_velocity=(0.0, 0.0, 0.0),
            # Large, fast oscillation: the good window is brief.
            wave=WaveConfig(model="CWR", number=2, amplitude=0.3, period=1.0, steepness=0.5),
        ),
    )
    sim = FastSimulator(cfg, grasp_duration_steps=5)

    saw_failure = False
    for seed in range(40):
        sim.reset(seed=seed)
        if not sim._good_opportunity_now():
            continue
        sim.apply_action(ATTEMPT_GRASP)  # commit at a good instant
        sim.step(0.5)
        for _ in range(5):
            sim.apply_action(WAIT)
            sim.step(0.5)
            r = sim.get_robot_state()
            if r.grasp_just_failed:
                saw_failure = True
                assert not r.gripper_holding_target
                break
            if r.gripper_holding_target:
                break
        if saw_failure:
            break
    # With a maneuver (5 * 0.5 = 2.5s) far longer than this target's brief
    # good window, at least one committed grasp must fail mid-maneuver.
    assert saw_failure
