"""Consistency tests for the reduced-order surrogate and the saved Pareto front.

Run from the repository root:

    python -m pytest tests -q
"""

import csv
import importlib
import math
import os
import sys

import numpy as np
import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)


@pytest.fixture(scope="module")
def model():
    """Import the optimizer from the repository root (it loads profiles on import)."""
    old = os.getcwd()
    os.chdir(REPO_ROOT)
    try:
        yield importlib.import_module("mrfo_multiobjective_v2")
    finally:
        os.chdir(old)


def test_baseline_reproduces_documented_values(model):
    """The paper EMS1 point must reproduce the values quoted in README/§结果速览."""
    x = np.array([0.30, 0.70, 1.0, 200000.0 / 820300.0, 0.90])
    obj, stats = model.evaluate(x)

    assert stats["feasible"], "paper baseline must satisfy SOC >= SOC_min"
    assert stats["H2"] == pytest.approx(106.62, rel=0.01)
    assert obj[0] == pytest.approx(9_332_345, rel=0.01)
    assert obj[1] == pytest.approx(1.95, abs=0.05)
    assert obj[2] == pytest.approx(7.03, abs=0.05)


def test_battery_replacement_matches_wang_2011_closed_form(model):
    """N_rep = 14400 * cycles_per_journey / N_cycles_to_EoL must hold exactly."""
    x = np.array([0.30, 0.70, 1.0, 200000.0 / 820300.0, 0.90])
    obj, stats = model.evaluate(x)

    exp_term = 23257.0 * math.exp(-31230.0 / (8.314 * 293.0))
    n_cyc_eol = (0.20 * model.C_nom_Ah / exp_term) ** (1 / 0.56) / model.C_nom_Ah
    expected = 14400.0 * stats["cycles"] / n_cyc_eol

    assert obj[2] == pytest.approx(expected, rel=1e-6)
    assert n_cyc_eol == pytest.approx(2042, abs=2)


def test_saved_front_is_non_dominated_and_feasible(model):
    """The committed CSV must be a real Pareto front, not an arbitrary sample."""
    path = os.path.join(REPO_ROOT, "results", "pareto_front_v2.csv")
    with open(path, newline="") as fh:
        rows = list(csv.DictReader(fh))
    assert rows, "pareto_front_v2.csv is empty"

    objs = np.array([[float(r[k]) for k in ("TLC_EUR", "N_rep_FC", "N_rep_BAT")]
                     for r in rows])

    # No point may be the penalised version of an infeasible solution.
    assert np.all(objs[:, 0] < model.PENALTY_TLC)

    for p in range(len(objs)):
        for q in range(len(objs)):
            if p == q:
                continue
            strictly_better = np.any(objs[p] < objs[q])
            weakly_better = np.all(objs[p] <= objs[q])
            assert not (weakly_better and strictly_better), (
                f"row {q} is dominated by row {p}"
            )


def test_route_profile_reproduces_precomputed_arrays(model):
    """route_profile at the baseline mass must be bit-identical to the cached arrays.

    This is what keeps the five-variable study reproducible after adding the
    battery-size variable (bat_scale = 1 takes the precomputed path).
    """
    tau, p_trac, p_regen = model.route_profile(model.M)
    assert np.array_equal(tau, model.tau_orig)
    assert np.array_equal(p_trac, model.Pm_trac)
    assert np.array_equal(p_regen, model.Pm_regen)


def test_larger_pack_mass_increases_hydrogen(model, monkeypatch):
    """A bigger pack must pay for its mass; removing BATT_MASS must lower H2."""
    x = np.array([0.30, 0.70, 1.0, 200000.0 / 820300.0, 0.90, 2.5])
    _, with_mass = model.evaluate(x)
    monkeypatch.setattr(model, "BATT_MASS", 0.0)
    _, no_mass = model.evaluate(x)
    assert with_mass["H2"] > no_mass["H2"]


def test_infeasibility_penalty_mechanism(model, monkeypatch):
    """Raising the SOC floor above the achievable minimum must penalise the point.

    On this route the EMS state machine keeps SOC above ~0.31 for every vector in
    the decision box, so the penalty branch cannot be reached from a valid design
    point alone; the floor is therefore raised temporarily to exercise it.
    """
    x = np.array([0.30, 0.70, 1.0, 200000.0 / 820300.0, 0.90])
    obj_ok, stats_ok = model.evaluate(x)
    assert stats_ok["feasible"]

    monkeypatch.setattr(model, "SOC_MIN_FLOOR", 0.60)
    obj_bad, stats_bad = model.evaluate(x)

    assert not stats_bad["feasible"]
    assert obj_bad[0] == pytest.approx(obj_ok[0] + model.PENALTY_TLC)
    assert obj_bad[1] == pytest.approx(obj_ok[1] + model.PENALTY_REP)
    assert obj_bad[2] == pytest.approx(obj_ok[2] + model.PENALTY_REP)
