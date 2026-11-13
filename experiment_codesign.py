"""Experiment: co-design the fuel-cell size and the battery pack size.

The five-variable study (mrfo_multiobjective_v2.py) fences the battery at
1178.61 Ah, so the fuel cell is already at the "any smaller and the pack
collapses" feasibility boundary and the achievable TLC saving is ~7 %.

This script adds a sixth decision variable, ``bat_scale`` (pack capacity
relative to 1178.61 Ah), and lets the optimiser trade battery CAPEX against
fuel-cell sizing, battery wear and hydrogen use. A larger pack adds mass
(8920 kg per unit of ``bat_scale``, as in the paper's Table 10), so the route
power profile is recomputed for each candidate.

Bounds are 0.60-4.0x: beyond ~4x the fixed EMS1 state machine keeps SOC above
its 60 % resume threshold for the whole journey and the fuel cell never leaves
idle, which is a degenerate control regime rather than a meaningful design.

    python experiment_codesign.py

Writes results/codesign_pareto.csv and prints the achievable range compared
with the five-variable front.
"""

import csv
import os
import time

import numpy as np
from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.algorithms.moo.mopso_cd import MOPSO_CD
from pymoo.core.problem import Problem
from pymoo.optimize import minimize

import mrfo_multiobjective_v2 as m

LB = np.array([0.20, 0.60, 0.30, 0.15, 0.80, 0.60])
UB = np.array([0.35, 0.85, 1.10, 0.35, 0.95, 4.00])
BUDGET = 8000
POP = 40
SEEDS = [0, 1, 2]
TLC_SCALE = 1e6
PENALTY_TLC = 1e8 / TLC_SCALE


class CoDesignProblem(Problem):
    def __init__(self):
        super().__init__(n_var=6, n_obj=3, xl=0.0, xu=1.0)

    def _evaluate(self, X, out, *args, **kwargs):
        F = np.empty((X.shape[0], 3))
        for i, x in enumerate(X):
            obj, _ = m.evaluate(LB + x * (UB - LB))
            F[i] = obj
            F[i, 0] /= TLC_SCALE
        out["F"] = F


def non_dominated_mask(F):
    keep = np.ones(len(F), dtype=bool)
    for i in range(len(F)):
        if not keep[i]:
            continue
        for j in range(len(F)):
            if i == j or not keep[j]:
                continue
            if np.all(F[j] <= F[i]) and np.any(F[j] < F[i]):
                keep[i] = False
                break
    return keep


def main():
    os.makedirs("results", exist_ok=True)

    base_x = np.array([0.30, 0.70, 1.0, 200000.0 / 820300.0, 0.90, 1.0])
    base_o, base_s = m.evaluate(base_x)
    print(f"baseline (EMS1, 1178.61 Ah): TLC={base_o[0]:,.0f}  "
          f"FC={base_o[1]:.2f}  Bat={base_o[2]:.2f}")

    all_F, all_X = [], []
    for name, factory in (
        ("MOPSO", lambda: MOPSO_CD(pop_size=POP)),
        ("NSGA-II", lambda: NSGA2(pop_size=POP)),
    ):
        for seed in SEEDS:
            t0 = time.perf_counter()
            res = minimize(CoDesignProblem(), factory(), ("n_eval", BUDGET),
                           seed=seed, verbose=False)
            dt = time.perf_counter() - t0
            F = np.asarray(res.opt.get("F"))
            X = np.asarray(res.opt.get("X"))
            all_F.append(F)
            all_X.append(X)
            feas = F[F[:, 0] < PENALTY_TLC]
            print(f"{name:8s} seed={seed}  front={len(feas):3d}  "
                  f"bestTLC={feas[:,0].min()*TLC_SCALE:,.0f}  time={dt:5.1f}s")

    # Union of every run's non-dominated archive, then one more dominance pass:
    # the per-run archives are not comparable across seeds.
    F = np.vstack(all_F)
    X = np.vstack(all_X)
    ok = F[:, 0] < PENALTY_TLC
    F, X = F[ok], X[ok]
    keep = non_dominated_mask(F)
    F, X = F[keep], X[keep]
    print(f"\nunion: {len(F)} non-dominated designs over 6 runs")

    phys = LB + X * (UB - LB)
    order = np.argsort(F[:, 0])
    rows = []
    for k in order:
        rows.append([*phys[k], F[k, 0] * TLC_SCALE, F[k, 1], F[k, 2]])
    feas = F[order]
    phys = phys[order]

    with open("results/codesign_pareto.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["SOC_low", "SOC_high", "FC_nom_scale", "FC_idle_scale",
                    "SOC_regen", "bat_scale", "TLC_EUR", "N_rep_FC", "N_rep_BAT"])
        w.writerows(rows)

    print(f"TLC range        : {feas[:,0].min()*TLC_SCALE:,.0f} - "
          f"{feas[:,0].max()*TLC_SCALE:,.0f} EUR "
          f"({100*(feas[:,0].min()*TLC_SCALE-base_o[0])/base_o[0]:+.1f}% vs baseline)")
    print(f"FC replacements  : {feas[:,1].min():.2f} - {feas[:,1].max():.2f}")
    print(f"Battery repl.    : {feas[:,2].min():.2f} - {feas[:,2].max():.2f}")
    print(f"FC size          : {phys[:,2].min()*820.3:.0f} - "
          f"{phys[:,2].max()*820.3:.0f} kW")
    print(f"Battery capacity : {phys[:,5].min()*1178.61:.0f} - "
          f"{phys[:,5].max()*1178.61:.0f} Ah "
          f"({phys[:,5].min()*100:.0f}% - {phys[:,5].max()*100:.0f}%)")

    names = ["SOC_low", "SOC_high", "FC_nom_scale", "FC_idle_scale",
             "SOC_regen", "bat_scale"]
    span = UB - LB
    at_lb = np.mean(phys <= LB + 0.01 * span, axis=0)
    at_ub = np.mean(phys >= UB - 0.01 * span, axis=0)
    print("\nbound activity (share of front at bound):")
    for j, nm in enumerate(names):
        if at_lb[j] > 0 or at_ub[j] > 0:
            print(f"  {nm:14s} lower {at_lb[j]*100:4.0f}%  upper {at_ub[j]*100:4.0f}%")

    k = np.argmin(feas[:, 0])
    print("\ncheapest design:")
    print("  SOC_low=%.3f SOC_high=%.3f FC=%.0f kW idle=%.0f kW regen=%.3f "
          "battery=%.0f Ah" % (
              phys[k, 0], phys[k, 1], phys[k, 2] * 820.3, phys[k, 3] * 820.3,
              phys[k, 4], phys[k, 5] * 1178.61))


if __name__ == "__main__":
    main()
