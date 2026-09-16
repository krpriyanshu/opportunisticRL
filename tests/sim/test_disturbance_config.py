import pytest

from opportunity_rl.sim.disturbance_config import load_disturbance_config


def test_loads_sydney_regatta_default():
    cfg = load_disturbance_config("sydney_regatta_default")
    assert cfg.name == "sydney_regatta_default"
    # Matches the real sydney_regatta.sdf world's own defaults, verified
    # from source in docs/wave-simulation-literature-review.md.
    assert cfg.vessel_wave.model == "PMS"
    assert cfg.vessel_wave.number == 3
    assert cfg.vessel_wave.period == pytest.approx(5.0)
    assert cfg.vessel_wave.gain == pytest.approx(0.3)
    assert cfg.vessel_wave.steepness == pytest.approx(0.0)
    assert cfg.target.start_position == (1.2, 0.5, 1.6)


def test_missing_config_raises_with_helpful_message():
    with pytest.raises(FileNotFoundError, match="does_not_exist"):
        load_disturbance_config("does_not_exist")
