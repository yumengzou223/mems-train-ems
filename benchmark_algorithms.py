"""Benchmark MO-MRFO against standard multi-objective algorithms.

All algorithms optimise the same reduced-order EMS model with the same
function-evaluation budget (2 360 evaluations), 3 seeds each. Reported:

  * hypervolume (HV) on objectives normalised by the pooled reference front
  * inverted generational distance (IGD) against that reference front
  * best TLC, front size and wall-clock time

Run from the repository root:

    python benchmark_algorithms.py

Outputs: results/benchmark.csv, results/benchmark_hv.png
"""

import csv
import os
import time

import numpy as np
from pymoo.algorithms.moo.moead import MOEAD
from pymoo.algorithms.moo.mopso_cd import MOPSO_CD
from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.algorithms.moo.spea2 import SPEA2
from pymoo.core.callback import Callback
from pymoo.core.problem import Problem
from pymoo.indicators.hv import HV
from pymoo.indicators.igd import IGD
from pymoo.optimize import minimize
from pymoo.util.ref_dirs import get_reference_directions

import mrfo_multiobjective_v2 as m

LB = np.array([0.20, 0.60, 0.85, 0.15, 0.80])
UB = np.array([0.35, 0.85, 1.10, 0.35, 0.95])
BUDGET = 2360          # 40 + 80 * 29 for MO-MRFO, matched by ("n_eval", BUDGET)
POP = 40
SEEDS = [0, 1, 2]
TLC_SCALE = 1e6        # objectives are scaled to [M EUR, count, count] so that
                       # decomposition-based algorithms (MOEA/D) are not dominated
                       # by the 1e6-magnitude cost objective
PENALTY_TLC = 1e8 / TLC_SCALE
REF_POINT = np.array([1.1, 1.1, 1.1])

ALGOS = ["MO-MRFO", "NSGA-II", "SPEA2", "MOEA/D", "MOPSO"]

# Categorical slots in fixed order (reference palette, light mode).
COLORS = {
    "MO-MRFO": "#2a78d6",
    "NSGA-II": "#eb6834",
    "SPEA2": "#1baf7a",
    "MOEA/D": "#eda100",
    "MOPSO": "#e87ba4",
}


class EMSProblem(Problem):
    """pymoo wrapper: unit-cube decision variables mapped to the EMS bounds."""

    def __init__(self):
        super().__init__(n_var=5, n_obj=3, xl=0.0, xu=1.0)

    def _evaluate(self, X, out, *args, **kwargs):
        F = np.empty((X.shape[0], 3))
        for i, x in enumerate(X):
            obj, _ = m.evaluate(LB + x * (UB - LB))
            F[i] = obj
            F[i, 0] /= TLC_SCALE
        out["F"] = F


class Snapshot(Callback):
    """Record the best-so-far front after each generation."""

    def __init__(self):
        super().__init__()
        self.frames = []

    def notify(self, algorithm):
        F = algorithm.opt.get("F") if algorithm.opt is not None else None
        if F is not None and len(F):
            self.frames.append((float(algorithm.evaluator.n_eval), F.copy()))


def make_algorithm(name):
    if name == "NSGA-II":
        return NSGA2(pop_size=POP)
    if name == "SPEA2":
        return SPEA2(pop_size=POP)
    if name == "MOEA/D":
        ref_dirs = get_reference_directions("das-dennis", 3, n_partitions=6)
        return MOEAD(ref_dirs=ref_dirs)
    if name == "MOPSO":
        return MOPSO_CD(pop_size=POP)
    raise ValueError(name)


def feasible(F):
    return F[np.asarray(F)[:, 0] < PENALTY_TLC]


def non_dominated(F):
    F = np.asarray(F)
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
    return F[keep]


def run_pymoo(name, seed):
    cb = Snapshot()
    t0 = time.perf_counter()
    res = minimize(EMSProblem(), make_algorithm(name), ("n_eval", BUDGET),
                   seed=seed, verbose=False, callback=cb)
    elapsed = time.perf_counter() - t0
    return np.asarray(res.F), cb.frames, elapsed


def run_mrfo(seed):
    history = []
    t0 = time.perf_counter()
    _, O, _, _ = m.mo_mrfo(POP, 29, LB, UB, seed=seed, history=history)
    elapsed = time.perf_counter() - t0
    O = np.asarray(O).copy()
    O[:, 0] /= TLC_SCALE
    scaled_hist = []
    for n_eval, F in history:
        F = F.copy()
        F[:, 0] /= TLC_SCALE
        scaled_hist.append((n_eval, F))
    return O, scaled_hist, elapsed


def main():
    os.makedirs("results", exist_ok=True)
    runs = {}
    for name in ALGOS:
        runs[name] = []
        for seed in SEEDS:
            if name == "MO-MRFO":
                F, hist, dt = run_mrfo(seed)
            else:
                F, hist, dt = run_pymoo(name, seed)
            runs[name].append(dict(seed=seed, F=feasible(F), hist=hist, time=dt))
            print(f"{name:8s} seed={seed}  front={len(feasible(F)):3d}  "
                  f"time={dt:5.1f}s  "
                  f"bestTLC={feasible(F)[:, 0].min() * TLC_SCALE:,.0f}")

    # Pooled non-dominated set = reference front for HV/IGD.
    pool = np.vstack([r["F"] for rs in runs.values() for r in rs])
    ref = non_dominated(pool)
    ideal, nadir = ref.min(axis=0), ref.max(axis=0)
    span = np.where(nadir - ideal > 0, nadir - ideal, 1.0)
    ref_n = (ref - ideal) / span

    hv = HV(ref_point=REF_POINT)
    igd = IGD(ref_n)

    rows = []
    for name, rs in runs.items():
        for r in rs:
            z = (r["F"] - ideal) / span
            rows.append(dict(
                algorithm=name, seed=r["seed"], n_front=len(r["F"]),
                HV=hv(z), IGD=igd(z), time_s=r["time"],
                best_TLC=r["F"][:, 0].min() * TLC_SCALE,
                min_FC_rep=r["F"][:, 1].min(),
                min_BAT_rep=r["F"][:, 2].min(),
            ))

    with open("results/benchmark.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    print("\n=== summary (mean +- std over seeds) ===")
    print(f"{'algorithm':8s} | {'HV':>16s} | {'IGD':>16s} | {'time [s]':>12s}")
    summary = {}
    for name in ALGOS:
        sub = [r for r in rows if r["algorithm"] == name]
        h = np.array([r["HV"] for r in sub])
        g = np.array([r["IGD"] for r in sub])
        t = np.array([r["time_s"] for r in sub])
        summary[name] = (h.mean(), h.std(), g.mean(), g.std(), t.mean())
        print(f"{name:8s} | {h.mean():.5f} +- {h.std():.5f} | "
              f"{g.mean():.5f} +- {g.std():.5f} | {t.mean():7.1f}")

    # HV convergence: mean +- std band over seeds.
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    SURFACE, INK, INK_2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#d9d8d4"
    fig, ax = plt.subplots(figsize=(7.6, 4.8), dpi=160)
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)

    for name in ALGOS:
        curves = []
        for r in runs[name]:
            xs, ys = [], []
            for n_eval, F in r["hist"]:
                z = (feasible(F) - ideal) / span
                if len(z):
                    xs.append(n_eval)
                    ys.append(hv(z))
            curves.append((np.array(xs), np.array(ys)))
        grid = np.linspace(POP, BUDGET, 120)
        interp = []
        for xs, ys in curves:
            order = np.argsort(xs)
            interp.append(np.interp(grid, xs[order], ys[order]))
        interp = np.vstack(interp)
        ax.plot(grid, interp.mean(axis=0), color=COLORS[name], linewidth=2,
                label=name)
        ax.fill_between(grid, interp.mean(axis=0) - interp.std(axis=0),
                        interp.mean(axis=0) + interp.std(axis=0),
                        color=COLORS[name], alpha=0.12, linewidth=0)

    ax.set_xlabel("Function evaluations", fontsize=9.5, color=INK_2)
    ax.set_ylabel("Normalized hypervolume", fontsize=9.5, color=INK_2)
    ax.set_title("MO-MRFO vs standard MOO algorithms on the EMS problem",
                 fontsize=12.5, color=INK, loc="left", pad=22)
    ax.text(0, 1.015, "mean and std band over 3 seeds; same evaluation budget",
            transform=ax.transAxes, fontsize=9, color=INK_2)
    ax.grid(True, color=GRID, linewidth=0.8, alpha=0.7)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=INK_2, labelsize=8.5)
    ax.legend(frameon=False, fontsize=8.5, labelcolor=INK_2, ncol=2)
    fig.tight_layout()
    fig.savefig("results/benchmark_hv.png", facecolor=SURFACE, bbox_inches="tight")
    print("\nSaved results/benchmark.csv and results/benchmark_hv.png")


if __name__ == "__main__":
    main()
