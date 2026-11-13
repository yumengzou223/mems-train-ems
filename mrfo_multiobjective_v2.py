"""MO-MRFO for the hydrogen fuel-cell hybrid train EMS.

Reduced-order surrogate of the Simulink/PLECS model: a four-state EMS state
machine drives a 0.5 s simulation of one 12 000 s journey, then the audited
degradation and life-cycle-cost model (compute_degradation.m) is evaluated.

Decision vector : [SOC_low, SOC_high, FC_nom_scale, FC_idle_scale, SOC_regen]
Objectives      : min TLC [EUR], min FC replacements, min battery replacements

Run from the repository root:  python mrfo_multiobjective_v2.py
Outputs: results/pareto_front_v2.csv, pareto_scatter_v2.png, pareto_parallel_v2.png
"""

import os
import math

import numpy as np
from numba import njit

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ============================================================================
# 1. Load the calibrated route profile (same source as mrfo_calibrated.py)
# ============================================================================
spd = np.loadtxt("speed_points.txt")
acc = np.loadtxt("accel_points.txt")
slp = np.loadtxt("slope_points.txt", delimiter="\t", skiprows=1)

t_full = spd[:, 0]
v_full = spd[:, 1]
a_full = acc[:, 1]

step = 5
t = t_full[::step]
v_ref = v_full[::step]
a_ref = a_full[::step]
dt = t[1] - t[0]

s_km = slp[:, 0]
s_perm = slp[:, 1]
_, idx = np.unique(s_km, return_index=True)
s_km = s_km[np.sort(idx)]
s_perm = s_perm[np.sort(idx)]
cum = np.cumsum(v_ref) * dt / 1000.0
slope = np.interp(cum, s_km, s_perm, left=s_perm[0], right=s_perm[-1]) / 1000.0


# ============================================================================
# 2. Model constants -- final calibrated values (A_res = 7500, DC/DC 92%)
# ============================================================================
M = 195000.0
BATT_MASS = 8920.0       # baseline pack mass [kg] (paper Table 10)
delta = 0.06
r_w = 0.42
gr = 2.72
eta_gear = 0.96
eta_trac = 0.70
eta_regen = 0.40
A_res = 7500.0
B_res = 7.722
C_res = 7.329
P_aux = 200000.0
ETA_DC = 0.92

C_nom_Ah = 1178.61
C_nom_As = C_nom_Ah * 3600.0
n_se = 405.0
V_nom = 1500.0
SOC_init = 0.90

P_nom_base = 820300.0

nc = 732.0
Ac = 1367.0
F_const = 96485.0
MH2 = 0.002016
stoch = 1.05
Id = np.array([0.0, 1.0, 2.0, 3.0, 3.5, 4.0])
Vd = np.array([1.05, 0.82, 0.72, 0.65, 0.35, 0.30])

FC_LOAD_SWITCH_FRAC = 0.30  # |Delta P_fc|/P_nom threshold for counting a load switch

# Economic horizon (paper Table 12/13): 2 journeys/day, 14400 journeys over 20 years.
JOURNEYS = 14400.0
YEARS = 20.0
JOURNEYS_PER_YEAR = JOURNEYS / YEARS          # 720, not 730
SOC_MIN_FLOOR = 0.30                           # battery minimum SOC (paper Sec. 3.1)
PENALTY_TLC = 1e8                              # infeasible solutions are dominated
PENALTY_REP = 1e3

bp = np.array([318.28, 447.46, 1114.12, 1591.58, 1909.86, 2228.14])
btbl = np.array([10750.0, 10750.0, 9670.0, 6270.0, 5220.0, 4150.0])


def fc_I(P):
    lo, hi = 0.0, 6000.0
    for _ in range(60):
        m = (lo + hi) / 2.0
        Vc = np.interp(m / Ac, Id, Vd)
        if nc * Vc * m > P:
            hi = m
        else:
            lo = m
    return (lo + hi) / 2.0


def h2_flow(P):
    if P <= 0.0:
        return 0.0
    return stoch * nc * fc_I(P) * MH2 / (2.0 * F_const)


# Precompute the fixed mechanical/power profile arrays (depend only on route).
F_resist = A_res + B_res * v_ref + C_res * v_ref * v_ref
F_tot = M * (1.0 + delta) * a_ref + F_resist + M * 9.81 * slope
tau_orig = np.clip(F_tot * r_w / (gr * eta_gear), -50000.0, 50000.0)
rpm = v_ref / r_w * 60.0 / (2.0 * np.pi) * gr
om = v_ref / r_w * gr

ra_max = np.interp(rpm, bp, btbl)
ra_max = np.where(rpm < 200.0, ra_max * np.maximum(rpm / 200.0, 0.0), ra_max)

Pm_trac = np.where(tau_orig >= 0.0, tau_orig * om / eta_trac, 0.0)
Pm_regen = (-np.minimum(np.abs(tau_orig), ra_max)) * om * eta_regen
stopped = v_ref < 0.5


def route_profile(m_train):
    """Traction/regenerative power profiles for a given train mass.

    Called when the battery pack is resized: a larger pack adds mass, which
    raises traction energy and therefore hydrogen use. At the baseline mass the
    precomputed arrays are reused, so the five-variable study is unchanged.
    """
    f_tot = m_train * (1.0 + delta) * a_ref + F_resist + m_train * 9.81 * slope
    tau = np.clip(f_tot * r_w / (gr * eta_gear), -50000.0, 50000.0)
    p_trac = np.where(tau >= 0.0, tau * om / eta_trac, 0.0)
    p_regen = (-np.minimum(np.abs(tau), ra_max)) * om * eta_regen
    return tau, p_trac, p_regen


@njit
def simulate_core(
    tau_orig, om, Pm_trac, Pm_regen, stopped,
    SOC_low, SOC_high, P_nom, P_idle, P_max, SOC_regen,
    switch_frac,
    h2n, h2i, h2x,
    C_nom_As, n_se, dt,
):
    N = tau_orig.shape[0]
    SOC = 0.90
    H2 = 0.0
    state = 1.0

    n1_trans = 0
    t1 = 0.0
    n2 = 0
    t2 = 0.0
    Ah_dis = 0.0
    SOC_min = SOC
    cnt1 = 0
    cnt3 = 0

    prev_fc_on = False
    prev_P_fc = 0.0

    for i in range(N):
        tau = tau_orig[i]
        if tau >= 0.0:
            Pm = Pm_trac[i]
        else:
            if SOC >= 0.90:
                Pm = 0.0
            else:
                Pm = Pm_regen[i]

        P_total = Pm + P_aux
        stopped_i = stopped[i]
        cur = state

        if cur == 1.0:
            if SOC < SOC_low and stopped_i:
                cur = 2.0
            elif (stopped_i and SOC > 0.60) or ((not stopped_i) and SOC > SOC_high):
                cur = 3.0
        elif cur == 2.0:
            if (not stopped_i) or SOC > SOC_low:
                cur = 1.0
        elif cur == 3.0:
            if SOC < 0.60:
                cur = 1.0
        else:
            cur = 1.0
        state = cur
        if cur == 1.0:
            cnt1 += 1
        elif cur == 3.0:
            cnt3 += 1

        if cur == 1.0:
            P_fc = P_nom
            h = h2n
        elif cur == 2.0:
            P_fc = P_max
            h = h2x
        elif cur == 3.0:
            P_fc = P_idle
            h = h2i
        else:
            P_fc = 0.0
            h = 0.0

        P_bat = P_total / 0.92 - P_fc
        if SOC >= SOC_regen and P_bat < 0.0:
            P_bat = 0.0

        fc_on = P_fc > 1000.0
        if i > 0:
            if fc_on != prev_fc_on:
                n1_trans += 1
            if abs(P_fc - prev_P_fc) > switch_frac * P_nom:
                n2 += 1
        if fc_on and P_fc <= P_idle + 50000.0:
            t1 += dt
        if P_fc > 0.8 * P_nom:
            t2 += dt
        prev_fc_on = fc_on
        prev_P_fc = P_fc

        lnS = math.log(max(SOC, 0.01))
        Uocv_cell = math.exp(
            1.422 + 0.2214 * lnS + 0.1829 * lnS ** 2 + 0.0745 * lnS ** 3
            + 0.0145 * lnS ** 4 + 0.0014 * lnS ** 5 + 0.00005 * lnS ** 6
        )
        Uocv = Uocv_cell * n_se
        I_bat = P_bat / Uocv if Uocv > 0.1 else 0.0
        SOC -= I_bat * dt / C_nom_As
        if SOC < 0.0:
            SOC = 0.0
        if SOC > 1.0:
            SOC = 1.0
        if SOC < SOC_min:
            SOC_min = SOC
        H2 += h * dt

        # Discharge throughput must use the actual pack voltage, not a fixed 1500 V
        # (matches compute_degradation.m: I = P_bat ./ max(V_batt, 1)).
        if I_bat > 0.0:
            Ah_dis += I_bat * dt

    n1 = n1_trans / 2.0
    t1_h = t1 / 3600.0
    t2_h = t2 / 3600.0
    Ah_dis_h = Ah_dis / 3600.0
    s1 = cnt1 / N
    s3 = cnt3 / N
    return H2, n1, t1_h, n2, t2_h, Ah_dis_h, SOC, SOC_min, s1, s3


def economics(H2, n1, t1_h, n2, t2_h, Ah_dis_h, SOC_end, P_nom, C_ah=None):
    if C_ah is None:
        C_ah = C_nom_Ah
    V1 = 13.79e-6
    U1 = 8.662e-6
    V2 = 0.4184e-6
    U2 = 10.00e-6
    k_deg = 1.72
    dV_rated = 0.4265

    Vdeg = n1 * V1 + t1_h * U1 + n2 * V2 + t2_h * U2
    TFC = dV_rated / (k_deg * Vdeg) if Vdeg > 0 else math.inf
    N_rep_FC = 14400.0 / TFC if TFC < math.inf else 0.0

    A_bat = 23257.0
    B_bat = 31230.0
    R_gas = 8.314
    T_bat = 293.0
    exp_term = A_bat * math.exp(-B_bat / (R_gas * T_bat))
    cycles = Ah_dis_h / C_ah
    N_cyc_EoL = (0.20 * C_ah / exp_term) ** (1.0 / 0.56) / C_ah
    N_life_bat = N_cyc_EoL / cycles if cycles > 0 else math.inf
    N_rep_BAT = 14400.0 / N_life_bat if N_life_bat < math.inf else 0.0

    # FC stack and battery pack are sized by the decision variables, so CAPEX
    # must follow them.
    CAPEX_FC = 190.0 * P_nom / 1000.0
    CAPEX_BAT = 84.0 * (C_ah * V_nom) / 1000.0
    r = 0.04
    n_years = 20.0
    PV = (1.0 - (1.0 + r) ** (-n_years)) / r

    OPEX_FC = 6.6 * H2 * JOURNEYS_PER_YEAR * PV
    C_batt_kWh = C_ah * V_nom / 1000.0
    E_grid_kWh = max(0.0, SOC_init - SOC_end) * C_batt_kWh / 0.975
    OPEX_BAT = 0.19 * E_grid_kWh * JOURNEYS_PER_YEAR * PV
    OPEX_MAINT = (CAPEX_FC + CAPEX_BAT) * 0.02 * PV

    REPLEX_FC = 0.0
    if TFC < math.inf:
        nfc = int(math.floor(14400.0 / TFC))
        for i in range(1, nfc + 1):
            tr = i * TFC / JOURNEYS_PER_YEAR
            if tr <= n_years:
                REPLEX_FC += CAPEX_FC / (1.0 + r) ** tr

    REPLEX_BAT = 0.0
    if N_life_bat < math.inf:
        nbat = int(math.floor(14400.0 / N_life_bat))
        for i in range(1, nbat + 1):
            tr = i * N_life_bat / JOURNEYS_PER_YEAR
            if tr <= n_years:
                REPLEX_BAT += CAPEX_BAT / (1.0 + r) ** tr

    TLC = CAPEX_FC + CAPEX_BAT + OPEX_FC + OPEX_BAT + OPEX_MAINT + REPLEX_FC + REPLEX_BAT
    return TLC, N_rep_FC, N_rep_BAT, Vdeg, cycles


def evaluate(x):
    """Evaluate one design.

    x = [SOC_low, SOC_high, FC_nom_scale, FC_idle_scale, SOC_regen, bat_scale?]
    ``bat_scale`` (pack capacity relative to 1178.61 Ah) is optional so that the
    original five-variable study stays reproducible.
    """
    SOC_low, SOC_high, fc_nom_s, fc_idle_s, SOC_regen = x[:5]
    bat_scale = float(x[5]) if len(x) > 5 else 1.0
    C_ah = C_nom_Ah * bat_scale
    P_nom = fc_nom_s * P_nom_base
    P_idle = fc_idle_s * P_nom_base
    P_max = P_nom * 1.1
    h2n = h2_flow(P_nom)
    h2i = h2_flow(P_idle)
    h2x = h2_flow(P_max)

    if abs(bat_scale - 1.0) > 1e-12:
        tau, pm_trac, pm_regen = route_profile(M + (bat_scale - 1.0) * BATT_MASS)
    else:
        tau, pm_trac, pm_regen = tau_orig, Pm_trac, Pm_regen

    H2, n1, t1_h, n2, t2_h, Ah_dis_h, SOC_end, SOC_min, s1, s3 = simulate_core(
        tau, om, pm_trac, pm_regen, stopped,
        SOC_low, SOC_high, P_nom, P_idle, P_max, SOC_regen,
        FC_LOAD_SWITCH_FRAC,
        h2n, h2i, h2x, C_ah * 3600.0, n_se, dt,
    )
    TLC, N_rep_FC, N_rep_BAT, Vdeg, cycles = economics(
        H2, n1, t1_h, n2, t2_h, Ah_dis_h, SOC_end, P_nom, C_ah
    )
    # The EMS must keep the pack above SOC_min; anything below is infeasible and
    # is pushed behind every feasible point instead of being silently accepted.
    feasible = SOC_min >= SOC_MIN_FLOOR - 1e-9
    stats = dict(
        H2=H2, SOC_end=SOC_end, SOC_min=SOC_min, Vdeg_uV=Vdeg * 1e6,
        cycles=cycles, P_nom_kW=P_nom / 1e3, P_idle_kW=P_idle / 1e3,
        State1=s1, State3=s3, feasible=feasible,
        bat_scale=bat_scale, C_ah=C_ah,
    )
    if feasible:
        return np.array([TLC, N_rep_FC, N_rep_BAT]), stats
    # Constant offset keeps the dominance relation total without distorting the
    # front: every feasible point wins on all three objectives.
    return np.array([TLC + PENALTY_TLC,
                     N_rep_FC + PENALTY_REP,
                     N_rep_BAT + PENALTY_REP]), stats


def dominates(a, b):
    return bool(np.all(a <= b)) and bool(np.any(a < b))


def fast_non_dominated_sort(objs):
    n = objs.shape[0]
    S = [[] for _ in range(n)]
    n_dom = [0] * n
    fronts = [[]]
    for p in range(n):
        for q in range(n):
            if p == q:
                continue
            if dominates(objs[p], objs[q]):
                S[p].append(q)
            elif dominates(objs[q], objs[p]):
                n_dom[p] += 1
        if n_dom[p] == 0:
            fronts[0].append(p)

    i = 0
    while fronts[i]:
        Q = []
        for p in fronts[i]:
            for q in S[p]:
                n_dom[q] -= 1
                if n_dom[q] == 0:
                    Q.append(q)
        i += 1
        fronts.append(Q)
    return fronts[:-1]


def crowding_distance(objs, front):
    if len(front) <= 2:
        return {idx: math.inf for idx in front}
    m = len(front)
    dist = {idx: 0.0 for idx in front}
    for k in range(objs.shape[1]):
        idx = sorted(front, key=lambda i: objs[i, k])
        dist[idx[0]] = math.inf
        dist[idx[-1]] = math.inf
        fmin = objs[idx[0], k]
        fmax = objs[idx[-1], k]
        if fmax - fmin == 0.0:
            continue
        for j in range(1, m - 1):
            dist[idx[j]] += (objs[idx[j + 1], k] - objs[idx[j - 1], k]) / (fmax - fmin)
    return dist


def mo_mrfo(n_pop, max_iter, lb, ub, seed=0, history=None):
    """MO-MRFO: MRFO operators, NSGA-II environmental selection.

    If ``history`` is a list, a snapshot ``(n_evals, objectives)`` is appended
    after initialisation and after every generation, for convergence benchmarking.
    """
    n_dim = lb.shape[0]
    S_som = 2.0
    rng = np.random.default_rng(seed)

    X = lb + rng.random((n_pop, n_dim)) * (ub - lb)
    O = np.zeros((n_pop, 3))
    for i in range(n_pop):
        O[i], _ = evaluate(X[i])
    if history is not None:
        history.append((float(n_pop), O.copy()))

    for it in range(max_iter):
        progress = it / max_iter
        fronts = fast_non_dominated_sort(O)
        front0 = fronts[0]
        cd = crowding_distance(O, front0)
        sorted_front0 = sorted(front0, key=lambda i: -cd[i])

        offspring = []
        for i in range(n_pop):
            leader = X[sorted_front0[rng.integers(0, len(sorted_front0))]]
            if rng.random() < 0.5:
                beta = 2.0 * math.exp(1.0 - progress) * math.sin(2.0 * math.pi * rng.random())
                if i == 0:
                    nx = leader + rng.random(n_dim) * (leader - X[i]) + beta * (leader - X[i])
                else:
                    nx = (X[i] + rng.random(n_dim) * (X[i - 1] - X[i])
                          + rng.random(n_dim) * (leader - X[i]) + beta * (leader - X[i]))
            else:
                alpha = 2.0 * rng.random() * math.sqrt(abs(math.log(rng.random())))
                if i == 0:
                    nx = X[i] + rng.random(n_dim) * (leader - X[i]) + alpha * (leader - X[i])
                else:
                    nx = X[i] + rng.random(n_dim) * (X[i - 1] - X[i]) + alpha * (leader - X[i])

            nx = np.clip(nx, lb, ub)
            if rng.random() < 0.15:
                mut = rng.integers(0, n_dim)
                nx[mut] = lb[mut] + rng.random() * (ub[mut] - lb[mut])
            offspring.append(nx)

        for i in range(n_pop):
            leader = X[sorted_front0[rng.integers(0, len(sorted_front0))]]
            r2 = rng.random(n_dim)
            r3 = rng.random(n_dim)
            sx = np.clip(X[i] + S_som * (r2 * leader - r3 * X[i]), lb, ub)
            offspring.append(sx)

        offspring = np.array(offspring)
        O_off = np.zeros((offspring.shape[0], 3))
        for i in range(offspring.shape[0]):
            O_off[i], _ = evaluate(offspring[i])

        all_X = np.vstack([X, offspring])
        all_O = np.vstack([O, O_off])
        all_fronts = fast_non_dominated_sort(all_O)

        selected = []
        for fr in all_fronts:
            if len(selected) + len(fr) <= n_pop:
                selected.extend(fr)
            else:
                cd = crowding_distance(all_O, fr)
                for idx in sorted(fr, key=lambda i: -cd[i]):
                    if len(selected) < n_pop:
                        selected.append(idx)
                break
            if len(selected) == n_pop:
                break

        X = all_X[selected]
        O = all_O[selected]
        if history is not None:
            history.append((float(n_pop + 2 * n_pop * (it + 1)), O.copy()))

    fronts = fast_non_dominated_sort(O)
    return X[fronts[0]], O[fronts[0]], X, O


def main():
    lb = np.array([0.20, 0.60, 0.85, 0.15, 0.80])
    ub = np.array([0.35, 0.85, 1.10, 0.35, 0.95])

    base_x = np.array([0.30, 0.70, 1.0, 200000.0 / P_nom_base, 0.90])
    base_o, base_s = evaluate(base_x)
    print("=== baseline (paper EMS1, final calibrated model) ===")
    print(
        f"TLC={base_o[0]:.0f} EUR  FC_rep={base_o[1]:.3f}  "
        f"Bat_rep={base_o[2]:.3f}  H2={base_s['H2']:.2f} kg"
    )
    print(
        f"  SOC_end={base_s['SOC_end']:.3f}  SOC_min={base_s['SOC_min']:.3f}  "
        f"Vdeg={base_s['Vdeg_uV']:.2f} uV  cycles={base_s['cycles']:.4f}  "
        f"feasible={base_s['feasible']}"
    )

    print("\n=== running MO-MRFO (NSGA-II selection + MRFO operators) ===")
    pf_X, pf_O, _, _ = mo_mrfo(n_pop=40, max_iter=30, lb=lb, ub=ub, seed=0)

    # Deduplicate and sort the final Pareto front by TLC for readability.
    rows = np.hstack([pf_X, pf_O])
    rows = np.unique(np.round(rows, 8), axis=0)
    pf_X = rows[:, :5]
    pf_O = rows[:, 5:]
    order = np.argsort(pf_O[:, 0])
    pf_X = pf_X[order]
    pf_O = pf_O[order]

    print(f"\nPareto front has {len(pf_O)} non-dominated solutions.\n")
    n_infeas = int(np.sum(pf_O[:, 0] >= PENALTY_TLC))
    if n_infeas:
        print(f"WARNING: {n_infeas} point(s) violate SOC_min >= {SOC_MIN_FLOOR:.2f}")
    else:
        print(f"All front members satisfy SOC_min >= {SOC_MIN_FLOOR:.2f}.")
    header = (
        "SOC_low SOC_high FC_nom_kW FC_idle_kW SOC_regen | "
        "TLC_EUR FC_rep Bat_rep"
    )
    print(header)
    for k in range(len(pf_O)):
        print(
            f"{pf_X[k,0]:.3f} {pf_X[k,1]:.3f} {pf_X[k,2]*P_nom_base/1e3:6.0f} "
            f"{pf_X[k,3]*P_nom_base/1e3:6.0f} {pf_X[k,4]:.3f} | "
            f"{pf_O[k,0]:8.0f} {pf_O[k,1]:5.2f} {pf_O[k,2]:6.2f}"
        )

    os.makedirs("results", exist_ok=True)
    np.savetxt(
        "results/pareto_front_v2.csv",
        np.hstack([pf_X, pf_O]),
        delimiter=",",
        header="SOC_low,SOC_high,FC_nom_scale,FC_idle_scale,SOC_regen,"
               "TLC_EUR,N_rep_FC,N_rep_BAT",
        comments="",
        fmt="%.8f",
    )
    print("\nSaved pareto_front_v2.csv")

    # Plot a 2D scatter matrix of the three objectives.
    names = ["TLC [EUR]", "FC replacements", "Battery replacements"]
    fig, axes = plt.subplots(3, 3, figsize=(10, 9))
    for i in range(3):
        for j in range(3):
            ax = axes[i, j]
            if i == j:
                ax.text(0.5, 0.5, names[i], ha="center", va="center", fontsize=9)
                ax.set_xticks([])
                ax.set_yticks([])
            else:
                ax.scatter(pf_O[:, j], pf_O[:, i], s=18, c="tab:blue", alpha=0.75)
                ax.scatter([base_o[j]], [base_o[i]], s=90, marker="*",
                           c="red", zorder=5, label="EMS1" if (i == 0 and j == 1) else None)
                ax.grid(True, alpha=0.3)
                if i == 2:
                    ax.set_xlabel(names[j], fontsize=8)
                if j == 0:
                    ax.set_ylabel(names[i], fontsize=8)
    fig.suptitle("MO-MRFO Pareto front (objective projections)", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig("results/pareto_scatter_v2.png", dpi=120)
    print("Saved pareto_scatter_v2.png")

    # Parallel coordinates of the Pareto front (normalized).
    fig2, ax2 = plt.subplots(figsize=(10, 5))
    obj_norm = (pf_O - pf_O.min(axis=0)) / (pf_O.max(axis=0) - pf_O.min(axis=0) + 1e-12)
    xs = np.arange(3)
    for row in obj_norm:
        ax2.plot(xs, row, color="tab:blue", alpha=0.4)
    ax2.plot(xs, (base_o - pf_O.min(axis=0)) /
             (pf_O.max(axis=0) - pf_O.min(axis=0) + 1e-12),
             color="red", linewidth=2.5, marker="*", markersize=12, label="EMS1")
    ax2.set_xticks(xs)
    ax2.set_xticklabels(names)
    ax2.set_ylabel("Normalized objective (lower is better)")
    ax2.set_title("MO-MRFO Pareto front (parallel coordinates)")
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    fig2.tight_layout()
    fig2.savefig("results/pareto_parallel_v2.png", dpi=120)
    print("Saved pareto_parallel_v2.png")


if __name__ == "__main__":
    main()
