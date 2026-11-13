function [fc_deg, bat_deg, TLC] = compute_degradation(sim_data, params)
% COMPUTE_DEGRADATION - FC and battery degradation + TLC (audited)
% Changes vs. compute_degradation.m (2026-08-21):
%   1. Battery aging uses the paper's nonlinear Eq.(14):
%        Closs = A*exp(-B/(R*T)) * (cycles*Cbat)^0.56
%      and solves for the equivalent discharge cycles at 20% capacity
%      loss, then divides by per-journey cycles. The previous code used
%      (cycles*1.0)^0.56 and treated the single-journey Closs as a
%      linear percentage, which was wrong on both counts.
%   2. Nrep is reported as n_journeys/N_life (paper Table 12).
t = sim_data.t;
dt = t(2) - t(1);
P_fc = sim_data.P_fc;
P_bat = sim_data.P_bat;
H2_total = sim_data.H2(end);
n_years = 20;
n_journeys = 14400;
journeys_per_year = n_journeys / n_years;   % 720 (paper: 2 journeys/day x 360 days)
P_nom = params.fc.P_nom;
P_idle = params.fc.P_idle;
V1 = 13.79e-6;
U1 = 8.662e-6;
V2 = 0.4184e-6;
U2 = 10.00e-6;
k_deg = 1.72;
dV_rated = 0.4265;
fc_on = P_fc > 1000;
n1_per_journey = sum(diff(double(fc_on)) ~= 0) / 2;
t1_hours = sum(P_fc > 1000 & P_fc <= P_idle + 50000) * dt / 3600;
n2_per_journey = sum(abs(diff(P_fc)) > 0.3 * P_nom);
t2_hours = sum(P_fc > 0.8 * P_nom) * dt / 3600;
Vdeg_per_journey = n1_per_journey * V1 + t1_hours * U1 + n2_per_journey * V2 + t2_hours * U2;
if Vdeg_per_journey > 0
    TFC_journeys = dV_rated / (k_deg * Vdeg_per_journey);
else
    TFC_journeys = inf;
end
fc_deg.n1 = n1_per_journey;
fc_deg.t1_hours = t1_hours;
fc_deg.n2 = n2_per_journey;
fc_deg.t2_hours = t2_hours;
fc_deg.Vdeg = Vdeg_per_journey;
fc_deg.TFC_journeys = TFC_journeys;
fc_deg.N_rep = n_journeys / TFC_journeys;
A_bat = 23257;
B_bat = 31230;
R_gas = 8.314;
T_bat = 293;
C_nom_Ah = params.batt.C_nom_Ah;
exp_term = A_bat * exp(-B_bat / (R_gas * T_bat));
V_batt = sim_data.V_batt;
I_batt = P_bat ./ max(V_batt, 1.0);
Ah_discharge = sum(max(I_batt, 0)) * dt / 3600;
cycles_per_journey = Ah_discharge / C_nom_Ah;
N_cyc_EoL = (0.20 * C_nom_Ah / exp_term)^(1/0.56) / C_nom_Ah;
if cycles_per_journey > 0
    N_life_bat = N_cyc_EoL / cycles_per_journey;
else
    N_life_bat = inf;
end
Closs_Ah_1 = exp_term * (cycles_per_journey * C_nom_Ah)^0.56;
Closs_pct_1 = Closs_Ah_1 / C_nom_Ah * 100;
bat_deg.cycles_equiv = cycles_per_journey;
bat_deg.Ah_discharge = Ah_discharge;
bat_deg.Closs_pct_1journey = Closs_pct_1;
bat_deg.N_life = N_life_bat;
bat_deg.N_rep = n_journeys / N_life_bat;
r = 0.04;
CAPEX_FC = 190 * P_nom / 1000;
CAPEX_BAT = 84 * (params.batt.C_nom_Ah * params.batt.V_nom) / 1000;
PV_factor = (1 - (1+r)^(-n_years)) / r;
OPEX_FC = 6.6 * H2_total * journeys_per_year * PV_factor;
% Grid electricity = net battery discharge over the journey (recharged at the
% origin/terminal station from the grid). On a non-electrified route there is no
% catenary en-route, so only the net SOC drop is replenished from the grid; FC
% charging is already paid via H2 and regenerative braking is free.
SOC_init = sim_data.SOC(1);
SOC_end  = sim_data.SOC(end);
C_batt_kWh = params.batt.C_nom_Ah * params.batt.V_nom / 1000;
eta_charge = 0.975;  % grid->battery charging efficiency (paper E_el/net ~ 864/843)
E_grid_kWh = max(0, SOC_init - SOC_end) * C_batt_kWh / eta_charge;
OPEX_BAT = 0.19 * E_grid_kWh * journeys_per_year * PV_factor;
% Maintenance cost (supplementary material: M = CAPEX * mf, mf = 2%/year)
CAPEX_TOTAL = CAPEX_FC + CAPEX_BAT;
M_annual = CAPEX_TOTAL * 0.02;
OPEX_MAINT = M_annual * PV_factor;
REPLEX_FC = 0;
n_rep_fc_int = floor(n_journeys / TFC_journeys);
for i = 1:n_rep_fc_int
    t_rep = i * TFC_journeys / journeys_per_year;
    if t_rep <= n_years
        REPLEX_FC = REPLEX_FC + CAPEX_FC / (1+r)^t_rep;
    end
end
REPLEX_BAT = 0;
n_rep_bat_int = floor(n_journeys / N_life_bat);
for i = 1:n_rep_bat_int
    t_rep = i * N_life_bat / journeys_per_year;
    if t_rep <= n_years
        REPLEX_BAT = REPLEX_BAT + CAPEX_BAT / (1+r)^t_rep;
    end
end
TLC = CAPEX_FC + CAPEX_BAT + OPEX_FC + OPEX_BAT + OPEX_MAINT + REPLEX_FC + REPLEX_BAT;
fprintf('\n===== DEGRADATION & TLC SUMMARY (audited) =====\n');
fprintf('FC Degradation:\n');
fprintf('  Start-stop cycles (n1): %d\n', n1_per_journey);
fprintf('  Idling time (t1): %.2f h\n', t1_hours);
fprintf('  Load changes (n2): %d\n', n2_per_journey);
fprintf('  High power time (t2): %.2f h\n', t2_hours);
fprintf('  Vdeg/journey: %.4f microV\n', Vdeg_per_journey * 1e6);
fprintf('  FC lifetime: %.0f journeys\n', TFC_journeys);
fprintf('  FC replacements: %.2f\n', fc_deg.N_rep);
fprintf('\nBattery Degradation:\n');
fprintf('  Discharge Ah/journey: %.1f Ah\n', Ah_discharge);
fprintf('  Equivalent discharge cycles: %.4f\n', cycles_per_journey);
fprintf('  Closs after 1 journey: %.4f %%\n', Closs_pct_1);
fprintf('  Battery lifetime (20%% loss): %.0f journeys\n', N_life_bat);
fprintf('  Battery replacements: %.2f\n', bat_deg.N_rep);
fprintf('\nEconomics:\n');
fprintf('  CAPEX FC: %.0f EUR\n', CAPEX_FC);
fprintf('  CAPEX Bat: %.0f EUR\n', CAPEX_BAT);
fprintf('  OPEX FC (H2): %.0f EUR\n', OPEX_FC);
fprintf('  OPEX Bat (grid): %.0f EUR\n', OPEX_BAT);
fprintf('  OPEX Maintenance: %.0f EUR\n', OPEX_MAINT);
fprintf('  REPLEX FC: %.0f EUR\n', REPLEX_FC);
fprintf('  REPLEX Bat: %.0f EUR\n', REPLEX_BAT);
fprintf('  TLC: %.0f EUR\n', TLC);
fprintf('  Paper EMS1 TLC: 9,191,965 EUR\n');
fprintf('=================================================\n');
end
