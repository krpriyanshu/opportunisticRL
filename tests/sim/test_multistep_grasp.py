"""Multi-step committed grasp maneuver tests (grasp_duration_steps > 1).
Numpy-only, no gymnasium/torch -- drives FastSimulator directly."""

from opportunity_rl.sim.disturbance_config import DisturbanceConfig
from opportunity_rl.sim.fast_simulator import (
    ABORT,
    APPROACH,
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


def test_abort_cancels_committed_grasp_while_window_open():
    sim = _still_target_sim(grasp_duration_steps=3)
    sim.reset(seed=0)
    sim.apply_action(ATTEMPT_GRASP)  # commit
    sim.step(0.5)
    sim.apply_action(ABORT)  # cancel while the window is still open
    sim.step(0.5)
    r = sim.get_robot_state()
    assert not r.gripper_holding_target
    assert not r.grasp_just_failed  # a deliberate cancel is not a failed grasp

    # The maneuver is really gone: further WAITs must not complete it.
    for _ in range(3):
        sim.apply_action(WAIT)
        sim.step(0.5)
        assert not sim.get_robot_state().gripper_holding_target


def test_abort_returns_arm_to_standby():
    # Still target 0.3 m from standby: inside the 0.85 m workspace radius,
    # so it stays a good opportunity, but APPROACH visibly moves the arm.
    x, y, z = DEFAULT_STANDBY_POSITION
    cfg = DisturbanceConfig(
        name="still_offset",
        vessel_wave=WaveConfig(gain=0.0, amplitude=0.0),
        target=TargetConfig(
            start_position=(x - 0.3, y, z),
            drift_velocity=(0.0, 0.0, 0.0),
            wave=WaveConfig(gain=0.0, amplitude=0.0),
        ),
    )
    sim = FastSimulator(cfg, grasp_duration_steps=3)
    sim.reset(seed=0)
    sim.apply_action(APPROACH)
    sim.step(0.5)
    assert sim.get_robot_state().ee_position != DEFAULT_STANDBY_POSITION

    sim.apply_action(ATTEMPT_GRASP)  # commit
    sim.step(0.5)
    sim.apply_action(ABORT)  # cancel; arm should head back to standby
    for _ in range(10):
        sim.step(0.5)
    assert sim.get_robot_state().ee_position == DEFAULT_STANDBY_POSITION


def test_new_grasp_can_be_committed_after_abort():
    sim = _still_target_sim(grasp_duration_steps=3)
    sim.reset(seed=0)
    sim.apply_action(ATTEMPT_GRASP)
    sim.apply_action(ABORT)
    # A fresh 3-step maneuver from scratch.
    sim.apply_action(ATTEMPT_GRASP)
    assert not sim.get_robot_state().gripper_holding_target
    sim.apply_action(WAIT)
    assert not sim.get_robot_state().gripper_holding_target
    sim.apply_action(WAIT)
    assert sim.get_robot_state().gripper_holding_target


def test_abort_cannot_undo_a_grasp_whose_window_already_closed():
    # If the window has closed by the time ABORT is issued, the grasp has
    # already failed: ABORT must not erase the failure (otherwise a policy
    # could dodge every failure penalty by reacting to the current state).
    cfg = DisturbanceConfig(
        name="brief",
        vessel_wave=WaveConfig(gain=0.0, amplitude=0.0),
        target=TargetConfig(
            start_position=DEFAULT_STANDBY_POSITION,
            drift_velocity=(0.0, 0.0, 0.0),
            wave=WaveConfig(model="CWR", number=2, amplitude=0.3, period=1.0, steepness=0.5),
        ),
    )
    sim = FastSimulator(cfg, grasp_duration_steps=5)

    checked = False
    for seed in range(40):
        sim.reset(seed=seed)
        if not sim._good_opportunity_now():
            continue
        sim.apply_action(ATTEMPT_GRASP)
        for _ in range(5):
            sim.step(0.5)
            if not sim._good_opportunity_now():
                sim.apply_action(ABORT)  # too late: the window is shut
                r = sim.get_robot_state()
                assert r.grasp_just_failed
                assert not r.gripper_holding_target
                checked = True
                break
            sim.apply_action(WAIT)
            if sim.get_robot_state().gripper_holding_target:
                break
        if checked:
            break
    assert checked
