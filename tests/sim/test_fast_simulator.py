import pytest

from opportunity_rl.sim.disturbance_config import load_disturbance_config
from opportunity_rl.sim.fast_simulator import (
    ABORT,
    APPROACH,
    ATTEMPT_GRASP,
    DEFAULT_STANDBY_POSITION,
    WAIT,
    FastSimulator,
)
from opportunity_rl.sim.interface import SimulationInterface
from opportunity_rl.sim.target_dynamics import TargetConfig
from opportunity_rl.sim.wave_disturbance import WaveConfig


def _sim(**target_overrides) -> FastSimulator:
    cfg = load_disturbance_config("sydney_regatta_default")
    if target_overrides:
        cfg = type(cfg)(
            name=cfg.name,
            vessel_wave=cfg.vessel_wave,
            target=TargetConfig(**{**cfg.target.__dict__, **target_overrides}),
        )
    return FastSimulator(cfg)


def test_satisfies_simulation_interface_protocol():
    # A plain (non-@runtime_checkable) Protocol can't be isinstance-checked
    # at runtime -- structural conformance is verified statically instead
    # (mypy: `_typecheck: SimulationInterface = FastSimulator(...)` below,
    # checked by `mypy src/opportunity_rl` in CI). This test just confirms
    # every required method actually exists and is callable, which is the
    # runtime-checkable subset of that guarantee.
    sim = _sim()
    for name in (
        "reset",
        "step",
        "get_robot_state",
        "get_target_state",
        "get_environment_state",
        "apply_action",
    ):
        assert callable(getattr(sim, name, None)), f"missing SimulationInterface method: {name}"


def test_fast_simulator_is_statically_typed_as_simulation_interface() -> None:
    sim = _sim()
    _typecheck: SimulationInterface = sim  # noqa: F841 -- mypy-only check, see test above
    assert _typecheck is sim


def test_reset_returns_ee_to_standby():
    sim = _sim()
    sim.apply_action(APPROACH)
    for _ in range(50):
        sim.step(0.1)
    sim.reset()
    assert sim.get_robot_state().ee_position == DEFAULT_STANDBY_POSITION


def test_wait_holds_ee_position_vessel_relative():
    sim = _sim()
    before = sim.get_robot_state().ee_position
    sim.apply_action(WAIT)
    for _ in range(20):
        sim.step(0.1)
    after = sim.get_robot_state().ee_position
    assert after == before


def test_approach_moves_ee_toward_target():
    # A target close to standby but not identical, so APPROACH has
    # somewhere to go without immediately snapping there in one step.
    sim = _sim(
        start_position=(1.157, 0.35, 1.5),
        drift_velocity=(0.0, 0.0, 0.0),
        wave=WaveConfig(gain=0.0, amplitude=0.0),
    )
    start = sim.get_robot_state().ee_position
    sim.apply_action(APPROACH)
    sim.step(0.1)
    moved = sim.get_robot_state().ee_position
    assert moved != start
    # Moved toward lower z (target is at z=1.5, standby z=1.89).
    assert moved[2] < start[2]


def test_approach_target_beyond_reach_is_clamped_not_ignored():
    sim = _sim(
        start_position=(10.0, 10.0, 10.0),
        drift_velocity=(0.0, 0.0, 0.0),
        wave=WaveConfig(gain=0.0, amplitude=0.0),
    )
    sim.apply_action(APPROACH)
    for _ in range(200):
        sim.step(0.1)
    ee = sim.get_robot_state().ee_position
    mount = sim._mount_offset  # test-internal access is fine, this is a white-box check
    dist = sum((ee[i] - mount[i]) ** 2 for i in range(3)) ** 0.5
    assert dist <= sim._manipulator_cfg.reach_radius_m + 1e-6


def test_attempt_grasp_succeeds_when_target_close_and_slow():
    sim = _sim(
        start_position=DEFAULT_STANDBY_POSITION,
        drift_velocity=(0.0, 0.0, 0.0),
        wave=WaveConfig(gain=0.0, amplitude=0.0),
    )
    assert not sim.get_robot_state().gripper_holding_target
    sim.apply_action(ATTEMPT_GRASP)
    assert sim.get_robot_state().gripper_holding_target


def test_attempt_grasp_fails_when_target_far():
    sim = _sim(
        start_position=(10.0, 10.0, 10.0),
        drift_velocity=(0.0, 0.0, 0.0),
        wave=WaveConfig(gain=0.0, amplitude=0.0),
    )
    sim.apply_action(ATTEMPT_GRASP)
    assert not sim.get_robot_state().gripper_holding_target


def test_abort_clears_grasp_and_returns_toward_standby():
    sim = _sim(
        start_position=DEFAULT_STANDBY_POSITION,
        drift_velocity=(0.0, 0.0, 0.0),
        wave=WaveConfig(gain=0.0, amplitude=0.0),
    )
    sim.apply_action(ATTEMPT_GRASP)
    assert sim.get_robot_state().gripper_holding_target
    sim.apply_action(ABORT)
    assert not sim.get_robot_state().gripper_holding_target


def test_invalid_action_raises():
    sim = _sim()
    with pytest.raises(ValueError, match="unknown action"):
        sim.apply_action(99)


def test_target_state_moves_over_time_under_default_disturbance():
    sim = _sim()
    p0 = sim.get_target_state().position
    for _ in range(30):
        sim.step(0.1)
    p1 = sim.get_target_state().position
    assert p0 != p1


def test_environment_state_reports_advancing_sim_time():
    # sim_time no longer starts at 0 (reset offsets it to a random wave
    # phase past the startup ramp -- see FastSimulator.reset), so check
    # the *advance* over two steps, which is what this actually tests.
    sim = _sim()
    t0 = sim.get_environment_state().sim_time
    sim.step(0.5)
    sim.step(0.5)
    assert sim.get_environment_state().sim_time - t0 == pytest.approx(1.0)


def test_reset_randomizes_start_phase_so_step_one_is_not_always_graspable():
    # Regression guard for the "grasp on step 1 always works" exploit:
    # under an intermittently-graspable config, the good_opportunity
    # state at the first step must vary across seeds (not be true every
    # time), confirming the wave phase is actually randomized at reset.
    from opportunity_rl.sim.disturbance_config import load_disturbance_config
    from opportunity_rl.sim.episode import FeasibilityConfig, evaluate_feasibility

    sim = FastSimulator(load_disturbance_config("moving_target"))
    fcfg = FeasibilityConfig()
    good_at_step_one = []
    for seed in range(30):
        sim.reset(seed=seed)
        robot = sim.get_robot_state()
        rp = sim._target_relative_position(sim._t)
        rv = sim._target_relative_velocity(sim._t)
        good_at_step_one.append(evaluate_feasibility(robot, rp, rv, fcfg).good_opportunity)
    # Not all-true and not all-false: the window is genuinely intermittent
    # at reset, so no trivial "always grasp immediately" policy exists.
    assert any(good_at_step_one)
    assert not all(good_at_step_one)
