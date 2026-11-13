import numpy as np

# ---- Load data (downsample to dt=0.5 for speed) ----
spd = np.loadtxt('speed_points.txt')
acc = np.loadtxt('accel_points.txt')
slp = np.loadtxt('slope_points.txt', delimiter='\t', skiprows=1)
t_full = spd[:, 0]; v_full = spd[:, 1]; a_full = acc[:, 1]
step = 5
t = t_full[::step]; v_ref = v_full[::step]; a_ref = a_full[::step]
dt = t[1] - t[0]
s_km = slp[:, 0]; s_perm = slp[:, 1]
_, idx = np.unique(s_km, return_index=True)
s_km = s_km[np.sort(idx)]; s_perm = s_perm[np.sort(idx)]
cum = np.cumsum(v_ref) * dt / 1000.0
slope = np.interp(cum, s_km, s_perm, left=s_perm[0], right=s_perm[-1]) / 1000.0

bp = np.array([318.28, 447.46, 1114.12, 1591.58, 1909.86, 2228.14])
btbl = np.array([10750.0, 10750.0, 9670.0, 6270.0, 5220.0, 4150.0])
def max_brake(rpm):
    return np.interp(rpm, bp, btbl)

nc = 732.0; Ac = 1367.0; F = 96485.0; MH2 = 0.002016
Id = np.array([0.0, 1.0, 2.0, 3.0, 3.5, 4.0])
Vd = np.array([1.05, 0.82, 0.72, 0.65, 0.35, 0.30])
def fc_I(P):
    lo, hi = 0.0, 6000.0
    for _ in range(60):
        m = (lo + hi) / 2
        Vc = np.interp(m / Ac, Id, Vd)
        if nc * Vc * m > P:
            hi = m
        else:
            lo = m
    return (lo + hi) / 2
def h2_flow(P):
    if P <= 0:
        return 0.0
    return 1.05 * nc * fc_I(P) * MH2 / (2.0 * F)

P_nom_base = 820300.0
M = 195000.0; delta = 0.06; r_w = 0.42; gr = 2.72
eta_gear = 0.96
eta_trac = 0.70
eta_regen = 0.40
A_res = 10500.0; B_res = 7.722; C_res = 7.329
P_aux = 200000.0
C_nom_Ah = 1178.61; C_nom_As = C_nom_Ah * 3600.0
n_se = 405
V_nom = 1500.0
SOC_init = 0.90

def simulate(x):
    SOC_low, SOC_high, fc_nom_s, fc_idle_s, SOC_regen = x
    P_nom = fc_nom_s * P_nom_base
    P_idle = fc_idle_s * P_nom_base
    P_max = P_nom * 1.1
    h2n = h2_flow(P_nom); h2i = h2_flow(P_idle); h2x = h2_flow(P_max)
    N = len(t)
    SOC = SOC_init
    H2 = 0.0
    state = 1.0
    P_fc_arr = np.empty(N)
    P_bat_arr = np.empty(N)
    inv = 1.0 / (gr * eta_gear)
    for i in range(N):
        v = v_ref[i]; a = a_ref[i]; sl = slope[i]
        F_res = A_res + B_res * v + C_res * v * v
        F_tot = M * (1 + delta) * a + F_res + M * 9.81 * sl
        tau = F_tot * r_w * inv
        tau = min(max(tau, -50000.0), 50000.0)
        rpm = v / r_w * 60.0 / (2 * np.pi) * gr
        om = v / r_w * gr
        if tau >= 0:
            corr = tau
        else:
            if SOC >= 0.90:
                corr = 0.0
            else:
                ra = max_brake(rpm)
                if rpm < 200.0:
                    ra *= max(rpm / 200.0, 0.0)
                corr = tau if abs(tau) <= ra else -ra
        Pm = (corr * om / eta_trac) if corr >= 0 else (corr * om * eta_regen)
        P_total = Pm + P_aux
        stopped = v < 0.5
        cur = state
        if cur == 1:
            if SOC < SOC_low and stopped:
                cur = 2
            elif (stopped and SOC > 0.60) or ((not stopped) and SOC > SOC_high):
                cur = 3
        elif cur == 2:
            if (not stopped) or SOC > SOC_low:
                cur = 1
        elif cur == 3:
            if SOC < 0.60:
                cur = 1
        else:
            cur = 1
        state = cur
        if cur == 1:
            P_fc = P_nom; h = h2n
        elif cur == 2:
            P_fc = P_max; h = h2x
        elif cur == 3:
            P_fc = P_idle; h = h2i
        else:
            P_fc = 0.0; h = 0.0
        P_bat = P_total - P_fc
        if SOC >= SOC_regen and P_bat < 0:
            P_bat = 0.0
        P_fc_arr[i] = P_fc
        P_bat_arr[i] = P_bat
        lnS = np.log(max(SOC, 0.01))
        Uocv = np.exp(1.422 + 0.2214*lnS + 0.1829*lnS**2 + 0.0745*lnS**3
                      + 0.0145*lnS**4 + 0.0014*lnS**5 + 0.00005*lnS**6) * n_se
        I_bat = P_bat / Uocv if Uocv > 0.1 else 0.0
        SOC -= I_bat * dt / C_nom_As
        SOC = min(max(SOC, 0.0), 1.0)
        H2 += h * dt
    return H2, P_fc_arr, P_bat_arr


def tlc_of(x):
    H2, P_fc, P_bat = simulate(x)
    P_nom = x[2] * P_nom_base
    fc_on = P_fc > 1000.0
    n1 = np.sum(np.diff(fc_on.astype(int)) != 0) / 2
    t1_h = np.sum((P_fc > 1000.0) & (P_fc <= 200000.0 + 50000.0)) * dt / 3600.0
    n2 = np.sum(np.abs(np.diff(P_fc)) > 0.3 * P_nom)
    t2_h = np.sum(P_fc > 0.8 * P_nom) * dt / 3600.0
    Vdeg = n1*13.79e-6 + t1_h*8.662e-6 + n2*0.4184e-6 + t2_h*10.00e-6
    TFC = 0.4265 / (1.72 * Vdeg) if Vdeg > 0 else np.inf
    N_rep_FC = 14400 / TFC if TFC < np.inf else 0.0
    A_bat = 23257.0; B_bat = 31230.0; Rg = 8.314; T_bat = 293.0
    exp_term = A_bat * np.exp(-B_bat / (Rg * T_bat))
    I_bat = P_bat / max(V_nom, 1.0)
    Ah_dis = np.sum(np.maximum(I_bat, 0.0)) * dt / 3600.0
    cycles = Ah_dis / C_nom_Ah
    N_cyc_EoL = (0.20 * C_nom_Ah / exp_term) ** (1 / 0.56) / C_nom_Ah
    N_life_bat = N_cyc_EoL / cycles if cycles > 0 else np.inf
    N_rep_BAT = 14400 / N_life_bat if N_life_bat < np.inf else 0.0
    CAPEX_FC = 190 * P_nom_base / 1000.0
    CAPEX_BAT = 84 * (C_nom_Ah * V_nom) / 1000.0
    r = 0.04; n_years = 20
    PV = (1 - (1 + r) ** (-n_years)) / r
    OPEX_FC = 6.6 * H2 * 730 * PV
    E_charge = abs(np.sum(np.minimum(P_bat, 0.0)) * dt / 3600.0) / 1000.0
    OPEX_BAT = 0.19 * E_charge * 730 * PV
    OPEX_MAINT = (CAPEX_FC + CAPEX_BAT) * 0.02 * PV
    REPLEX_FC = 0.0
    nfc = int(np.floor(14400 / TFC)) if TFC < np.inf else 0
    for i in range(1, nfc + 1):
        tr = i * TFC / 730.0
        if tr <= n_years:
            REPLEX_FC += CAPEX_FC / (1 + r) ** tr
    REPLEX_BAT = 0.0
    nbat = int(np.floor(14400 / N_life_bat)) if N_life_bat < np.inf else 0
    for i in range(1, nbat + 1):
        tr = i * N_life_bat / 730.0
        if tr <= n_years:
            REPLEX_BAT += CAPEX_BAT / (1 + r) ** tr
    TLC = CAPEX_FC + CAPEX_BAT + OPEX_FC + OPEX_BAT + OPEX_MAINT + REPLEX_FC + REPLEX_BAT
    return TLC, dict(H2=H2, Vdeg=Vdeg*1e6, N_rep_FC=N_rep_FC, N_rep_BAT=N_rep_BAT,
                     cycles=cycles, P_nom=P_nom/1e3, P_idle=x[3]*P_nom_base/1e3)


# ---- baseline (paper EMS1 params) ----
base_x = np.array([0.30, 0.70, 1.0, 200000.0/P_nom_base, 0.90])
base_TLC, base_s = tlc_of(base_x)
print('=== baseline (paper EMS1) ===')
print(f'TLC={base_TLC:.0f} H2={base_s["H2"]:.2f} Vdeg={base_s["Vdeg"]:.2f}uV '
      f'FC_rep={base_s["N_rep_FC"]:.2f} Bat_rep={base_s["N_rep_BAT"]:.2f} cyc={base_s["cycles"]:.3f}')

lb = np.array([0.20, 0.60, 0.85, 0.15, 0.80])
ub = np.array([0.35, 0.85, 1.10, 0.35, 0.95])
n_pop = 15; max_iter = 18; n_dim = 5
S_som = 2.0
rng = np.random.default_rng(0)
X = lb + rng.random((n_pop, n_dim)) * (ub - lb)
fitness = np.array([tlc_of(X[i])[0] for i in range(n_pop)])
order = np.argsort(fitness); X = X[order]; fitness = fitness[order]
best_x = X[0].copy(); best_f = fitness[0]
for it in range(max_iter):
    progress = it / max_iter
    new_X = X.copy(); new_F = fitness.copy()
    for i in range(n_pop):
        leader = best_x
        if rng.random() < 0.5:
            beta = 2 * np.exp(1 - progress) * np.sin(2 * np.pi * rng.random())
            if i == 1:
                nx = leader + rng.random(n_dim) * (leader - X[i]) + beta * (leader - X[i])
            else:
                nx = X[i] + rng.random(n_dim) * (X[i-1] - X[i]) + rng.random(n_dim) * (leader - X[i]) + beta * (leader - X[i])
        else:
            alpha = 2 * rng.random() * np.sqrt(abs(np.log(rng.random())))
            if i == 1:
                nx = X[i] + rng.random(n_dim) * (leader - X[i]) + alpha * (leader - X[i])
            else:
                nx = X[i] + rng.random(n_dim) * (X[i-1] - X[i]) + alpha * (leader - X[i])
        nx = np.clip(nx, lb, ub)
        nf = tlc_of(nx)[0]
        if nf < new_F[i]:
            new_X[i] = nx; new_F[i] = nf
    for i in range(n_pop):
        r2 = rng.random(n_dim); r3 = rng.random(n_dim)
        sx = np.clip(new_X[i] + S_som * (r2 * best_x - r3 * new_X[i]), lb, ub)
        sf = tlc_of(sx)[0]
        if sf < new_F[i]:
            new_X[i] = sx; new_F[i] = sf
    X = new_X; fitness = new_F
    order = np.argsort(fitness); X = X[order]; fitness = fitness[order]
    if fitness[0] < best_f:
        best_f = fitness[0]; best_x = X[0].copy()
    print(f'iter {it+1}/{max_iter}: best TLC = {best_f:.0f}')

best_TLC, best_s = tlc_of(best_x)
print('\n=== MRFO result ===')
print(f'best TLC = {best_TLC:.0f} EUR (baseline {base_TLC:.0f}, {100*(best_TLC-base_TLC)/base_TLC:+.1f}%)')
print(f'SOC_low={best_x[0]:.3f} SOC_high={best_x[1]:.3f} FC_nom={best_x[2]*P_nom_base/1e3:.0f}kW '
      f'FC_idle={best_x[3]*P_nom_base/1e3:.0f}kW SOC_regen={best_x[4]:.3f}')
print(f'H2={best_s["H2"]:.2f}kg Vdeg={best_s["Vdeg"]:.2f}uV FC_rep={best_s["N_rep_FC"]:.2f} Bat_rep={best_s["N_rep_BAT"]:.2f}')
