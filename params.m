%% =========================================================================
%  Hydrogen Hybrid Train - Global Parameters Configuration
%  Based on Paper 3.1 (Pozzessere et al., IJHE 2026)
%
%  This file defines ALL physical parameters for the electrical subsystem.
%  Run this before battery_eecm.m, pemfc_model.m, or ems_module.m
% =========================================================================

function params = params()
    %% ==================== DC Bus (Table 5) ====================
    params.V_bus       = 3000;       % DC bus voltage [V]
    params.eta_dc      = 0.92;       % DC/DC converter efficiency [-]
    params.P_aux       = 200e3;      % Auxiliary power [W]
    params.M_train     = 195e3;      % Train mass [kg] (195 t)
    params.P_rated     = 1.45e6;     % Traction power limit [W]

    %% ==================== Battery Pack (Table 10) ====================
    params.batt.mass        = 8.92e3;    % Battery pack mass [kg]
    params.batt.V_nom       = 1500;      % Nominal pack voltage [V]
    params.batt.C_nom_Ah    = 1178.61;   % Nominal pack capacity [Ah]
    params.batt.P_nom       = 1767.9e3;  % Nominal pack power [W]
    params.batt.C_rate_max  = 4;         % Max charge C-rate [-]
    params.batt.SOC_min     = 0.30;      % Minimum SOC (EMS threshold)
    params.batt.SOC_max     = 0.70;      % Maximum SOC (overcharge threshold)
    params.batt.SOC_init    = 0.90;      % Initial SOC [-]  (matches build_simulink_model_v5 / paper Fig.12 convention)

    % --- Cell-level: Panasonic CGR18650 (Table 6) ---
    %   Cell nominal: 3.7V, 2.25Ah
    params.batt.cell_V_nom = 3.7;        % Cell nominal voltage [V]
    params.batt.cell_C_nom = 2.25;       % Cell nominal capacity [Ah]

    % --- Derived: series/parallel counts ---
    params.batt.n_se  = round(params.batt.V_nom / params.batt.cell_V_nom);   % ~405
    params.batt.n_par = round(params.batt.C_nom_Ah / params.batt.cell_C_nom); % ~524

    % --- Pack-level capacity [Ah] and [As] ---
    params.batt.C_nom_As = params.batt.C_nom_Ah * 3600;  % [A*s]

    % --- Table 6: Polynomial coefficients ---
    %   f(SOC) = exp(sum(a_k * ln(SOC)^k)), k=0..6
    %   Note: (cid:0) in PDF = minus sign; some coefficients are negative
    %
    %   Uocv:  a0=1.422, a1=-0.2214(?), ... (see notes below)
    %   IMPORTANT: The PDF extraction shows (cid:0) which likely
    %   represents a minus sign. The sign pattern follows typical
    %   battery model behavior.
    %
    %   Row format: [a0, a1, a2, a3, a4, a5, a6]
    %
    %   Uocv coefficients (7 values from Table 6):
    params.batt.Uocv_coeff = [1.422, ...
                               0.2214, ...    % a1 (positive: PDF Table 6 shows no (cid:0) prefix)
                               0.1829, ...   % a2
                               0.0745, ...   % a3
                               0.0145, ...   % a4
                               0.0014, ...   % a5
                               0.00005];     % a6

    %   R0_cell coefficients (7 values):
    %   PDF: (cid:0) 2.9384 (cid:0) 0.2328 (cid:0) 0.2109 (cid:0) 0.1294 (cid:0) 0.0302 0 0
    %   All non-zero terms are negative (cid:0 = minus)
    params.batt.R0_coeff = [-2.9384, ...
                            -0.2328, ...   % a1 (cid:0 = negative)
                            -0.2109, ...
                            -0.1294, ...
                            -0.0302, ...
                             0, 0];

    %   R1_cell coefficients:
    %   PDF: (cid:0) 3.4883 (cid:0) 1.2434 (cid:0) 0.5619 0.0044 0.0348 0 0
    params.batt.R1_coeff = [-3.4883, ...
                            -1.2434, ...   % a1 (cid:0 = negative)
                            -0.5619, ...
                             0.0044, ...
                             0.0348, ...
                             0, 0];

    %   C1_cell coefficients:
    %   PDF: (cid:0) 0.146 0.3731 1.6511 1.0513 0.1918 0 0
    params.batt.C1_coeff = [-0.146, ...
                             0.3731, ...
                             1.6511, ...
                             1.0513, ...
                             0.1918, ...
                             0, 0];

    %   R2_cell coefficients:
    %   PDF: (cid:0) 3.1892 (cid:0) 0.0486 1.4851 6.1491 7.0124 3.1645 0.4997
    params.batt.R2_coeff = [-3.1892, ...
                            -0.0486, ...   % a1 (cid:0 = negative)
                             1.4851, ...
                             6.1491, ...
                             7.0124, ...
                             3.1645, ...
                             0.4997];

    %   C2_cell coefficients:
    %   PDF: 6.9413 (cid:0) 6.5951 (cid:0) 29.577 (cid:0) 56.356 (cid:0) 46.582 (cid:0) 16.991 (cid:0) 2.2588
    params.batt.C2_coeff = [ 6.9413, ...
                            -6.5951, ...
                           -29.577, ...
                           -56.356, ...
                           -46.582, ...
                           -16.991, ...
                            -2.2588];

    % --- Time constants (eq. 4) ---
    %   tau_i = C_i_cell * R_i_cell (computed dynamically as function of SOC)

    % --- Battery degradation model (eq. 14-15) ---
    params.batt.deg.A      = 23257;        % Empirical factor [-]
    params.batt.deg.B      = 31230;        % Used as exp(-B/(R*T)); paper writes activation energy as -31230 (equivalent sign convention)
    params.batt.deg.R_gas  = 8.314;        % Gas constant [J/(mol*K)]
    params.batt.deg.T      = 293;          % Temperature [K] (20 degC)
    params.batt.deg.cycles_avg_rate = 1.3; % Average C-rate for cycle counting
    params.batt.deg_SOC_threshold = 0.80;  % Replacement threshold (80% capacity)

    %% ==================== Fuel Cell Stack (Table 10) ====================
    params.fc.P_nom        = 820.3e3;      % FC nominal power [W]
    params.fc.P_max        = 820.3e3 * 1.1;% FC max power (110%) [W]
    params.fc.P_idle       = 200e3;        % FC idle power (auxiliary only) [W]
    params.fc.H2_pressure  = 700;          % Max H2 pressure [bar]
    params.fc.mass_total   = 8.77e3;       % H2 storage + FC mass [kg]
    params.fc.T_stack      = 353;          % Stack temperature [K] (80 degC)

    % --- FC Polarization Curve (Fig. 5, 353K curve) ---
    %   Extracted via volcano-vision model from Fig.5
    %   Current density [A/cm^2] vs Cell voltage [V]
    params.fc.I_dens_data  = [0,    1.0,  2.0,  3.0,  3.5,  4.0];   % [A/cm^2]
    params.fc.V_cell_data  = [1.05, 0.82, 0.72, 0.65, 0.35, 0.30];  % [V] (353K)

    % --- FC stack scaling (eq. 16-18) ---
    %   n_cell and A_cell not given in paper (supplementary material)
    %   Reverse-engineer from nominal operating point:
    %   At nominal power 820.3 kW, assume operating at ~1.0 A/cm^2, ~0.82V/cell
    %   P_stack = V_cell * n_cell * I_dens * A_cell
    %   Also need V_stack reasonable for DC/DC to 3000V bus
    %   Assume V_stack_nom ~ 500-800V (typical for railway FC)
    %
    %   Choose: operating point at I_dens=1.0 A/cm^2, V_cell=0.82V
    %   P_stack = 820.3e3 W
    %   V_stack = n_cell * 0.82
    %   I_stack = 1.0 * A_cell
    %   P_stack = V_stack * I_stack = n_cell * 0.82 * A_cell
    %
    %   Constraint 1: n_cell * 0.82 * A_cell = 820.3e3
    %   Constraint 2: V_stack = n_cell * 0.82 should be < 1500V (battery nominal)
    %                 and reasonable for DC/DC boost to 3000V
    %   Choose V_stack ~ 600V => n_cell ~ 732
    %   Then A_cell = 820.3e3 / 600 = 1367 cm^2
    %
    %   For a heavy-duty PEMFC, ~700-800 cells and ~1300-1400 cm^2 is reasonable
    params.fc.n_cell  = 732;       % Number of cells in series [-]
    params.fc.A_cell  = 1367;      % Cell active area [cm^2]

    % --- Hydrogen consumption (eq. 19) ---
    params.fc.F        = 96485;     % Faraday constant [C/mol]
    params.fc.M_H2     = 0.002016;  % H2 molar mass [kg/mol] (2.016 g/mol)
    params.fc.stoch    = 1.05;      % Stoichiometric factor [-]  (calibrated to reproduce paper H2; matches build_simulink_model_v5)
    params.fc.H2_init  = 0;         % Initial H2 consumed [kg] (will be tracked)

    % --- H2 tank capacity ---
    %   From Table 14: consumption ~0.596 kg/km (EMS1) over 177.55 km
    %   => ~105.9 kg per journey. Tank should hold enough for at least 1 journey
    %   With 700 bar storage, typical tank ~150-200 kg for this application
    params.fc.H2_tank  = 200;       % H2 tank capacity [kg]

    % --- FC Degradation (Table 7, eq. 20-21) ---
    params.fc.deg.V1 = 13.79e-6;    % Start-stop degradation [V/cycle]
    params.fc.deg.U1 = 8.662e-6;    % Idling degradation [V/h]
    params.fc.deg.V2 = 0.4184e-6;   % Load change degradation [V/cycle]
    params.fc.deg.U2 = 10.00e-6;    % High-power degradation [V/h]
    params.fc.deg.k  = 1.72;        % Tuning coefficient [-]
    params.fc.deg.dV_max = 0.4265;    % Max per-cell voltage degradation [V]
                                     % Back-calc from paper: 5689*1.72*43.59e-6=0.4265

    %% ==================== Motor Map (Table 8) ====================
    params.motor.speed_rpm    = [318.28, 447.46, 1114.12, 1591.58, 1909.86, 2228.14];
    params.motor.trac_torque  = [31.54,  31.54,  13.50,   9.46,    7.88,    6.75] * 1e3;  % [N*m]
    params.motor.brake_torque = [-10.75, -10.75, -9.67,  -6.27,   -5.22,   -4.15] * 1e3;  % [N*m]

    %% ==================== Route (Table 9) ====================
    params.route.length    = 177.55e3;  % [m]
    params.route.n_stations = 37;
    params.route.speed_limit = 130 / 3.6;  % [m/s]
    params.route.avg_slope  = 0;       % [permille]

    %% ==================== Simulation ====================
    params.sim.dt = 0.1;              % Time step [s]
    params.sim.eta_trac  = 0.70;      % Traction electrical efficiency (calibrated; motor_power_code uses 1/0.70)
    params.sim.eta_brake = 0.40;      % Regenerative braking efficiency (calibrated; motor_power_code uses 0.40)

    %% ==================== Economic (Table 13) ====================
    params.econ.FC_cost_per_kW  = 190;      % [EUR/kW]
    params.econ.batt_cost_per_kWh = 84;     % [EUR/kWh]
    params.econ.H2_cost_per_kg   = 6.6;     % [EUR/kg]
    params.econ.elec_cost_per_kWh = 0.19;   % [EUR/kWh]
    params.econ.discount_rate    = 0.04;    % [-]
    params.econ.years            = 20;      % [years]
    params.econ.journeys_per_day = 2;

    %% ==================== Energy Balance (OPEXbat model) ====================
    % E_grid = E_total_per_journey - FC_electrical
    %   where FC_electrical = H2_consumed [kg] * LHV_H2 [kWh/kg] * fc_elec_efficiency
    %   LHV_H2 = 33.3 kWh/kg (lower heating value)
    %
    % Reverse-engineered from paper Table 13 OPEXbat:
    %   EMS1: OPEXbat=1,629,401 EUR -> E_grid=864.4 kWh/journey
    %   EMS2: OPEXbat=2,012,052 EUR -> E_grid=1,067.4 kWh/journey
    % Solving simultaneously yields: E_total=2723.5 kWh, eta=0.5274
    % NOTE: The three values below are legacy (reverse-engineered energy-balance
    % approach) and are NOT used by the final compute_degradation.m. The final
    % model computes OPEX_FC directly from H2 mass and OPEX_BAT from net SOC drop.
    params.econ.E_total_per_journey = 2723.5;  % [legacy] Total energy per journey [kWh]
    params.econ.LHV_H2              = 33.3;    % [legacy] H2 lower heating value [kWh/kg]
    params.econ.fc_elec_efficiency  = 0.5274;  % [legacy] FC electrical efficiency [-]

end
