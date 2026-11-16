"""Tests for the committed H1 analysis (src/opportunity_rl/h1_analysis.py).
Standard library only -- no numpy/torch needed."""

import json
import math

import pytest

from opportunity_rl.h1_analysis import (
    HIGH_BUDGETS,
    LOW_BUDGETS,
    CurveData,
    cell_summaries,
    format_report,
    independent_unit_test,
    pooled_test,
    sign_test_p,
)

BUDGETS = [10_000, 20_000, 40_000, 80_000]


def test_sign_test_matches_figures_reported_in_evaluation_md():
    # evaluation.md Section 5 reports these exact sign-test p-values.
    assert sign_test_p(21, 3) == pytest.approx(2.8e-4, rel=0.02)
    assert sign_test_p(9, 0) == pytest.approx(3.9e-3, rel=0.02)
    assert sign_test_p(14, 10) == pytest.approx(0.54, abs=0.01)
    assert sign_test_p(5, 4) == pytest.approx(1.0)
    assert sign_test_p(40, 8) == pytest.approx(3.3e-6, rel=0.05)


def test_sign_test_is_symmetric_and_bounded():
    assert sign_test_p(3, 21) == sign_test_p(21, 3)
    assert sign_test_p(0, 0) == 1.0
    assert sign_test_p(5, 5) == 1.0
    assert 0.0 < sign_test_p(1, 0) <= 1.0


def _raw(data, budgets=BUDGETS, commit="abc"):
    # Mirror what learning_curve_h1.py writes: seed keys become strings in JSON.
    return json.loads(json.dumps({"git_commit": commit, "budgets": budgets, "data": data}))


def _two_severity_data():
    # B4 ahead at low budgets in every unit, tied-ish at high budgets.
    return {
        "mild": {
            "B2": {0: [-1.0, 0.5, 0.70, 0.70], 1: [-1.0, 0.4, 0.70, 0.75]},
            "B4": {0: [-0.5, 0.6, 0.71, 0.69], 1: [-0.6, 0.6, 0.69, 0.76]},
        },
        "severe": {
            "B2": {0: [-1.2, 0.0, 0.55, 0.56], 1: [-1.2, 0.1, 0.56, 0.57]},
            "B4": {0: [-0.8, 0.5, 0.56, 0.55], 1: [-0.7, 0.4, 0.55, 0.58]},
        },
    }


def test_from_json_converts_string_seed_keys():
    data = CurveData.from_json(_raw(_two_severity_data()))
    assert data.seeds == [0, 1]
    assert data.disturbances == ["mild", "severe"]
    assert data.delta("mild", 0, 10_000) == pytest.approx(0.5)


def test_independent_units_average_budgets_within_a_seed():
    data = CurveData.from_json(_raw(_two_severity_data()))
    low = independent_unit_test(data, LOW_BUDGETS)
    # 2 severities x 2 seeds = 4 units, not 8 (budgets are repeated measures).
    assert low.n_units == 4
    assert low.n_favouring_b4 == 4
    expected = [
        (0.5 + 0.1) / 2,
        (0.4 + 0.2) / 2,
        (0.4 + 0.5) / 2,
        (0.5 + 0.3) / 2,
    ]
    assert low.mean_delta == pytest.approx(sum(expected) / 4)
    assert low.p_value == pytest.approx(sign_test_p(4, 0))


def test_pooled_counts_every_budget_separately():
    data = CurveData.from_json(_raw(_two_severity_data()))
    pooled = pooled_test(data, LOW_BUDGETS)
    assert pooled.n_units == 8
    assert pooled.n_favouring_b4 == 8


def test_high_budget_mixed_result():
    data = CurveData.from_json(_raw(_two_severity_data()))
    high = independent_unit_test(data, HIGH_BUDGETS)
    assert high.n_units == 4
    assert high.p_value > 0.05


def test_ties_are_excluded_from_the_sign_test():
    tied = {
        "d": {
            "B2": {0: [0.0, 0.0, 0.0, 0.0], 1: [0.0, 0.0, 0.0, 0.0]},
            "B4": {0: [0.0, 0.0, 0.0, 0.0], 1: [0.5, 0.5, 0.0, 0.0]},
        }
    }
    r = independent_unit_test(CurveData.from_json(_raw(tied)), LOW_BUDGETS)
    assert (r.n_favouring_b4, r.n_favouring_b2, r.n_ties) == (1, 0, 1)
    assert r.p_value == pytest.approx(1.0)


def test_unmatched_seed_is_not_paired():
    data_dict = _two_severity_data()
    data_dict["mild"]["B4"][7] = [1.0, 1.0, 1.0, 1.0]  # no B2 counterpart
    data = CurveData.from_json(_raw(data_dict))
    assert data.matched_seeds("mild") == [0, 1]
    assert independent_unit_test(data, LOW_BUDGETS).n_units == 4


def test_length_mismatch_fails_loudly():
    bad = {"d": {"B2": {0: [0.0, 0.0]}, "B4": {0: [0.0, 0.0, 0.0, 0.0]}}}
    with pytest.raises(ValueError):
        CurveData.from_json(_raw(bad))


def test_missing_regime_budgets_fail_loudly():
    only_high = {"d": {"B2": {0: [0.0]}, "B4": {0: [1.0]}}}
    data = CurveData.from_json(_raw(only_high, budgets=[80_000]))
    with pytest.raises(ValueError):
        independent_unit_test(data, LOW_BUDGETS)


def test_cell_summaries_and_report():
    data = CurveData.from_json(_raw(_two_severity_data()))
    cells = cell_summaries(data)
    assert len(cells) == 2 * len(BUDGETS)
    first = cells[0]
    assert (first.disturbance, first.budget) == ("mild", 10_000)
    assert first.mean_delta == pytest.approx(0.45)
    assert first.n_favouring_b4 == 2

    report = format_report(data)
    assert "4 / 4" in report  # low-budget independent units
    assert "mild" in report and "severe" in report
    assert "`abc`" in report
    assert not math.isnan(first.mean_b2)
