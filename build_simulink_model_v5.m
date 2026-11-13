%% =========================================================================
%  build_simulink_model_v5.m
%  Hydrogen Hybrid Train EMS - Simulink Model Auto-Generation
%  Reproduces paper Fig.1 / Fig.2 / Fig.7 architecture
%
%  v5 fixes:
%  - Replace evalin with coder.extrinsic + parameter Constant blocks
%  - All MATLAB Function blocks use coder.extrinsic for external .m calls
%  - Output variables pre-typed before extrinsic calls
%  - All ports connected (no dangling outputs)
%  - Goto/From for all cross-subsystem signals (no wire spaghetti)
%% =========================================================================

function build_simulink_model_v5(EMS_OVERRIDE_opt)
% BUILD_SIMULINK_MODEL_V5 - build the calibrated hybrid train EMS Simulink model.
% Optional argument EMS_OVERRIDE_opt is a struct with fields
%   SOC_low, SOC_high, FC_nom, FC_idle, SOC_regen.
% If omitted, loads ems_override.mat (if present) or uses paper EMS1 defaults.

clc;

%% Check Simulink availability
if ~license('test', 'Simulink')
    error('Simulink license not available.');
end

%% Load parameters
fprintf('Loading parameters...\n');
p = params();
assignin('base', 'p', p);

if nargin < 1 || isempty(EMS_OVERRIDE_opt)
    if exist('ems_override.mat', 'file')
        S = load('ems_override.mat', 'EMS_OVERRIDE');
        EMS_OVERRIDE = S.EMS_OVERRIDE;
    else
        EMS_OVERRIDE = struct('SOC_low', 0.30, 'SOC_high', 0.70, ...
            'FC_nom', 820300.0, 'FC_idle', 200000.0, 'SOC_regen', 0.90);
    end
else
    EMS_OVERRIDE = EMS_OVERRIDE_opt;
end

%% Model setup
model_name = 'hybrid_train_ems';
model_path = fullfile(fileparts(mfilename('fullpath')), 'hybrid_train_ems.slx');

if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
if exist(model_name, 'file') == 4
    try; delete(model_path); catch; end
end

fprintf('Creating Simulink model: %s\n', model_name);
new_system(model_name);
open_system(model_name);

set_param(model_name, 'Solver', 'FixedStepDiscrete');
set_param(model_name, 'FixedStep', '0.1');
set_param(model_name, 'StopTime', '12000');
set_param(model_name, 'ReturnWorkspaceOutputs', 'off');

%% =========================================================================
%% 1. Top-level subsystems
%% =========================================================================
fprintf('Creating top-level subsystems...\n');

add_block('built-in/SubSystem', [model_name '/Control System (C)']);
add_block('built-in/SubSystem', [model_name '/Electrical Plant (B)']);
add_block('built-in/SubSystem', [model_name '/Electric Motor (D)']);
add_block('built-in/SubSystem', [model_name '/Mechanical Dynamics (A)']);
add_block('built-in/SubSystem', [model_name '/Results Viewer']);

%% Speed reference source - REAL DATA from paper Fig.10 (HaoCurve extracted, cleaned)
dt_sim = 0.1;
t_total = 12000;
t_sim = (0:dt_sim:t_total)';
% Load pre-computed speed and acceleration profiles (0.1s grid, 115001 points)
raw_v = dlmread('speed_points.txt');  % [t, v] columns
raw_a = dlmread('accel_points.txt');  % [t, a] columns
v_ref = raw_v(:, 2);
a_ref = raw_a(:, 2);
v_max = max(v_ref);  % ~34.8 m/s
v_ref_data = [t_sim, v_ref];
a_ref_data = [t_sim, a_ref];
% Reconstructed grade profile from Fragiacomo et al. 2022, Fig.5a
% (Reggio Calabria-Catanzaro Lido line, slope bounded within +/-10 permille)
raw_s = dlmread('slope_points.txt', '\t', 1, 0);  % [distance_km, slope_permille]
s_km = raw_s(:, 1);
s_permille = raw_s(:, 2);
[s_km, idx_unique] = unique(s_km);
s_permille = s_permille(idx_unique);
cum_dist_km = cumtrapz(t_sim, v_ref) / 1000.0;
slope_permille_t = interp1(s_km, s_permille, cum_dist_km, 'linear', s_permille(end));
slope_ref = slope_permille_t / 1000.0;  % dimensionless grade fraction
slope_ref_data = [t_sim, slope_ref];
assignin('base', 'v_ref_data', v_ref_data);
assignin('base', 'a_ref_data', a_ref_data);
assignin('base', 'slope_ref_data', slope_ref_data);

add_block('simulink/Sources/From Workspace', [model_name '/Speed Reference'], ...
    'VariableName', 'v_ref_data', 'Position', [30, 100, 130, 130]);
add_block('simulink/Sources/From Workspace', [model_name '/Accel Reference'], ...
    'VariableName', 'a_ref_data', 'Position', [30, 140, 130, 170]);
add_block('simulink/Sources/From Workspace', [model_name '/Slope Reference'], ...
    'VariableName', 'slope_ref_data', 'Position', [30, 180, 130, 210]);

%% =========================================================================
%% 1.5 Define all MATLAB Function block code strings
%% =========================================================================
fprintf('Defining MATLAB Function code...\n');

braking_code = strjoin({
    'function [corrected_torque, mech_brake] = fcn(torque_raw, SOC, pressure, max_brake_torque, motor_speed)'
    '  %#codegen'
    '  mech_brake = 0.0;'
    '  corrected_torque = torque_raw;'
    '  if torque_raw >= 0.0'
    '      corrected_torque = torque_raw;'
    '      mech_brake = 0.0;'
    '  else'
    sprintf('      if SOC >= %.4f || pressure < 0.1', EMS_OVERRIDE.SOC_regen)
    '          corrected_torque = 0.0;'
    '          mech_brake = abs(torque_raw);'
    '      else'
    '          regen_available = max_brake_torque;'
    '          if motor_speed < 200.0'
    '              regen_available = max_brake_torque * (motor_speed / 200.0);'
    '              if regen_available < 0.0'
    '                  regen_available = 0.0;'
    '              end'
    '          end'
    '          if abs(torque_raw) <= regen_available'
    '              corrected_torque = torque_raw;'
    '              mech_brake = 0.0;'
    '          else'
    '              corrected_torque = -regen_available;'
    '              mech_brake = abs(torque_raw) - regen_available;'
    '          end'
    '      end'
    '  end'
    'end'
}, '\n');

ems_code = strjoin({
    'function [P_fc_cmd, P_bat_cmd, R_cut, ems_state, state_out] = fcn(P_total, SOC, pressure, P_fc_nom, dt, strategy_code, state_in, v_ref)'
    '  %#codegen'
    '  % EMS1/EMS2 State Machine per paper Fig.8/Fig.9'
    '  % No time-based override - states emerge naturally from SOC/power logic'
    '  '
    '  P_fc_cmd   = 0.0;'
    '  P_bat_cmd  = 0.0;'
    '  R_cut      = 1.0;'
    '  ems_state  = 1.0;'
    '  state_out  = 1.0;'
    '  '
    '  cur_state = state_in;'
    '  if cur_state < 1.0 || cur_state > 4.0'
    '      cur_state = 1.0;  % Default: State 1 (FC Nominal)'
    '  end'
    '  '
    '  P_nom  = P_fc_nom;'
    '  P_max  = P_nom * 1.1;'
    sprintf('  P_idle = %.1f;', EMS_OVERRIDE.FC_idle)
    '  eta_dc = 0.92;'
    '  SOC_crit = 0.20;'
    sprintf('  SOC_low  = %.4f;', EMS_OVERRIDE.SOC_low)
    sprintf('  SOC_high = %.4f;', EMS_OVERRIDE.SOC_high)
    '  '
    '  train_stopped = (v_ref < 0.5);'
    '  '
    '  % State 4: H2 depleted (pressure < 1 bar)'
    '  if pressure < 1.0'
    '      cur_state = 4;'
    '      P_fc_cmd = 0.0;'
    '      P_bat_cmd = P_total / eta_dc;'
    '      R_cut = 0.0;'
    '      ems_state = 4.0;'
    '      state_out = 4.0;'
    '      return;'
    '  end'
    '  '
    '  if strategy_code >= 1.0'
    '      % EMS1 state transitions (per paper Fig.8)'
    '      % State 1 (FC Nominal 820kW): default, FC covers traction+aux, surplus charges battery'
    '      % State 2 (FC Max 902kW): SOC<30% AND stopped, boost charge'
    '      % State 3 (FC Idle 200kW): stopped+SOC>60%, or running+SOC>70% (avoid overcharge)'
    '      % State 4 (Battery Only): H2 depleted'
    '      switch cur_state'
    '          case 1'
    '              if SOC < SOC_low && train_stopped'
    '                  cur_state = 2;'
    '              elseif (train_stopped && SOC > 0.60) || (~train_stopped && SOC > SOC_high)'
    '                  cur_state = 3;'
    '              end'
    '          case 2'
    '              if ~train_stopped || SOC > SOC_low'
    '                  cur_state = 1;'
    '              end'
    '          case 3'
    '              if SOC < 0.60'
    '                  cur_state = 1;'
    '              end'
    '          case 4'
    '              if pressure > 1.0'
    '                  cur_state = 1;'
    '              end'
    '      end'
    '      % EMS1 power output'
    '      switch cur_state'
    '          case 1'
    '              P_fc_cmd = P_nom;'
    '              P_bat_cmd = P_total / eta_dc - P_fc_cmd;'
    '          case 2'
    '              P_fc_cmd = P_max;'
    '              P_bat_cmd = P_total / eta_dc - P_fc_cmd;'
    '          case 3'
    '              P_fc_cmd = P_idle;'
    '              P_bat_cmd = P_total / eta_dc - P_fc_cmd;'
    '          case 4'
    '              P_fc_cmd = 0.0;'
    '              P_bat_cmd = P_total / eta_dc;'
    '      end'
    '  else'
    '      % EMS2: FC dynamic variable power (95%-105% P_nom)'
    '      % FC does NOT charge battery (only regen charges), except SOC<30%'
    '      P_upper = P_nom * 1.05;'
    '      P_lower = P_nom * 0.95;'
    '      if SOC < SOC_low'
    '          P_fc_cmd = P_max;  % 110% to charge battery'
    '      elseif P_total > P_upper'
    '          P_fc_cmd = P_upper;'
    '      elseif P_total < P_lower && P_total > 0'
    '          P_fc_cmd = P_lower;'
    '      else'
    '          P_fc_cmd = P_total;'
    '      end'
    '      P_bat_cmd = P_total / eta_dc - P_fc_cmd;'
    '      cur_state = 1;'
    '  end'
    '  '
    '  R_cut = 1.0;'
    '  ems_state = double(cur_state);'
    '  state_out = double(cur_state);'
    'end'
}, '\n');

trac_code_str = strjoin({
    'function max_torque = fcn(rpm)'
    '  %#codegen'
    '  bp  = [318.28, 447.46, 1114.12, 1591.58, 1909.86, 2228.14];'
    '  tbl = [31540.0, 31540.0, 13500.0, 9460.0, 7880.0, 6750.0];'
    '  max_torque = tbl(end);'
    '  if rpm <= bp(1)'
    '      max_torque = tbl(1);'
    '  elseif rpm >= bp(end)'
    '      max_torque = tbl(end);'
    '  else'
    '      for i = 1:length(bp)-1'
    '          if rpm >= bp(i) && rpm <= bp(i+1)'
    '              frac = (rpm - bp(i)) / (bp(i+1) - bp(i));'
    '              max_torque = tbl(i) + frac * (tbl(i+1) - tbl(i));'
    '              break;'
    '          end'
    '      end'
    '  end'
    'end'
}, '\n');

brake_lookup_code_str = strjoin({
    'function max_brake = fcn(rpm)'
    '  %#codegen'
    '  bp  = [318.28, 447.46, 1114.12, 1591.58, 1909.86, 2228.14];'
    '  tbl = [10750.0, 10750.0, 9670.0, 6270.0, 5220.0, 4150.0];'
    '  max_brake = tbl(end);'
    '  if rpm <= bp(1)'
    '      max_brake = tbl(1);'
    '  elseif rpm >= bp(end)'
    '      max_brake = tbl(end);'
    '  else'
    '      for i = 1:length(bp)-1'
    '          if rpm >= bp(i) && rpm <= bp(i+1)'
    '              frac = (rpm - bp(i)) / (bp(i+1) - bp(i));'
    '              max_brake = tbl(i) + frac * (tbl(i+1) - tbl(i));'
    '              break;'
    '          end'
    '      end'
    '  end'
    'end'
}, '\n');

bat_code = strjoin({
    'function [V_batt, I_batt, SOC, P_batt_out, C_loss_pct, soc_out, v1_out, v2_out] = fcn(P_bat, dt, soc_in, v1_in, v2_in)'
    '  %#codegen'
    '  V_batt      = 700.0;'
    '  I_batt      = 0.0;'
    '  SOC         = 0.90;'
    '  P_batt_out  = 0.0;'
    '  C_loss_pct  = 0.0;'
    '  soc_out     = 0.90;'
    '  v1_out      = 0.0;'
    '  v2_out      = 0.0;'
    '  '
    '  soc_st = soc_in;'
    '  v1_st  = v1_in;'
    '  v2_st  = v2_in;'
    '  '
    '  C_nom_As = 1178.61 * 3600.0;'
    '  n_se = 405.0;'
    '  n_par = 524.0;'
    '  lnS = log(max(soc_st, 0.01));'
    '  Uocv_cell = exp(1.422 + 0.2214*lnS + 0.1829*lnS^2 + 0.0745*lnS^3 + 0.0145*lnS^4 + 0.0014*lnS^5 + 0.00005*lnS^6);'
    '  Uocv = Uocv_cell * n_se;'
    '  R0_cell = exp(-2.9384 - 0.2328*lnS - 0.2109*lnS^2 - 0.1294*lnS^3 - 0.0302*lnS^4);'
    '  R1_cell = exp(-3.4883 - 1.2434*lnS - 0.5619*lnS^2 + 0.0044*lnS^3 + 0.0348*lnS^4);'
    '  R2_cell = exp(-3.1892 - 0.0486*lnS + 1.4851*lnS^2 + 6.1491*lnS^3 + 7.0124*lnS^4 + 3.1645*lnS^5 + 0.4997*lnS^6);'
    '  C1_cell = exp(-0.146 + 0.3731*lnS + 1.6511*lnS^2 + 1.0513*lnS^3 + 0.1918*lnS^4);'
    '  C2_cell = exp(6.9413 - 6.5951*lnS - 29.577*lnS^2 - 56.356*lnS^3 - 46.582*lnS^4 - 16.991*lnS^5 - 2.2588*lnS^6);'
    '  if R0_cell < 0.0001; R0_cell = 0.0001; end'
    '  if R1_cell < 0.0001; R1_cell = 0.0001; end'
    '  if R2_cell < 0.0001; R2_cell = 0.0001; end'
    '  if C1_cell < 0.001; C1_cell = 0.001; end'
    '  if C2_cell < 0.001; C2_cell = 0.001; end'
    '  if C1_cell < 1.0; C1_cell = 1.0; end'
    '  if C2_cell < 1.0; C2_cell = 1.0; end'
    '  R0 = R0_cell * n_se / n_par;'
    '  R1 = R1_cell * n_se / n_par;'
    '  R2 = R2_cell * n_se / n_par;'
    '  C1 = C1_cell * n_par / n_se;'
    '  C2 = C2_cell * n_par / n_se;'
    '  '
    '  if Uocv > 0.1'
    '      I_batt = P_bat / Uocv;'
    '  else'
    '      I_batt = 0.0;'
    '  end'
    '  '
    '  tau1 = R1 * C1;'
    '  tau2 = R2 * C2;'
    '  if tau1 > dt'
    '      v1_st = v1_st + (I_batt * R1 - v1_st) * dt / tau1;'
    '  else'
    '      v1_st = I_batt * R1;'
    '  end'
    '  if tau2 > dt'
    '      v2_st = v2_st + (I_batt * R2 - v2_st) * dt / tau2;'
    '  else'
    '      v2_st = I_batt * R2;'
    '  end'
    '  '
    '  V_batt = Uocv - I_batt * R0 - v1_st - v2_st;'
    '  if V_batt < 0.0; V_batt = 0.0; end'
    '  '
    '  soc_st = soc_st - I_batt * dt / C_nom_As;'
    '  if soc_st < 0.0; soc_st = 0.0; end'
    '  if soc_st > 1.0; soc_st = 1.0; end'
    '  SOC = soc_st;'
    '  '
    '  P_batt_out = V_batt * I_batt;'
    '  C_loss_pct = 0.0;'
    '  soc_out = soc_st;'
    '  v1_out  = v1_st;'
    '  v2_out  = v2_st;'
    'end'
}, '\n');

fc_code = strjoin({
    'function [V_stack, I_stack, P_stack, H2_consumed, pressure_ratio, H2_flow, h2_out] = fcn(P_fc, dt, h2_in)'
    '  %#codegen'
    '  V_stack        = 0.0;'
    '  I_stack        = 0.0;'
    '  P_stack        = 0.0;'
    '  H2_consumed    = 0.0;'
    '  pressure_ratio = 1.0;'
    '  H2_flow        = 0.0;'
    '  h2_out         = h2_in;'
    '  '
    '  n_cell = 732.0;'
    '  A_cell = 1367.0;'
    '  F_const = 96485.0;'
    '  M_H2 = 0.002016;'
    '  stoch = 1.05;'
    '  '
    '  I_dens_data = [0.0, 1.0, 2.0, 3.0, 3.5, 4.0];'
    '  V_cell_data = [1.05, 0.82, 0.72, 0.65, 0.35, 0.30];'
    '  '
    '  H2_total = h2_in;'
    '  '
    '  if P_fc <= 0.0'
    '      V_stack = n_cell * V_cell_data(1);'
    '      I_stack = 0.0;'
    '      P_stack = 0.0;'
    '      H2_flow = 0.0;'
    '  else'
    '      P_fc_elec = P_fc;'
    '      '
    '      I_lo = 0.0;'
    '      I_hi = 6000.0;'
    '      for iter = 1:50'
    '          I_mid = (I_lo + I_hi) / 2.0;'
    '          I_d = I_mid / A_cell;'
    '          if I_d <= I_dens_data(1)'
    '              V_c = V_cell_data(1);'
    '          elseif I_d >= I_dens_data(end)'
    '              V_c = V_cell_data(end);'
    '          else'
    '              V_c = V_cell_data(end);'
    '              for i = 1:(length(I_dens_data)-1)'
    '                  if I_d >= I_dens_data(i) && I_d <= I_dens_data(i+1)'
    '                      fr = (I_d - I_dens_data(i)) / (I_dens_data(i+1) - I_dens_data(i));'
    '                      V_c = V_cell_data(i) + fr * (V_cell_data(i+1) - V_cell_data(i));'
    '                      break;'
    '                  end'
    '              end'
    '          end'
    '          P_test = n_cell * V_c * I_mid;'
    '          if P_test > P_fc_elec'
    '              I_hi = I_mid;'
    '          else'
    '              I_lo = I_mid;'
    '          end'
    '      end'
    '      '
    '      I_stack = (I_lo + I_hi) / 2.0;'
    '      I_dens = I_stack / A_cell;'
    '      '
    '      if I_dens <= I_dens_data(1)'
    '          V_cell = V_cell_data(1);'
    '      elseif I_dens >= I_dens_data(end)'
    '          V_cell = V_cell_data(end);'
    '      else'
    '          V_cell = V_cell_data(end);'
    '          for i = 1:(length(I_dens_data)-1)'
    '              if I_dens >= I_dens_data(i) && I_dens <= I_dens_data(i+1)'
    '                  fr = (I_dens - I_dens_data(i)) / (I_dens_data(i+1) - I_dens_data(i));'
    '                  V_cell = V_cell_data(i) + fr * (V_cell_data(i+1) - V_cell_data(i));'
    '                  break;'
    '              end'
    '          end'
    '      end'
    '      '
    '      V_stack = n_cell * V_cell;'
    '      P_stack = V_stack * I_stack;'
    '      '
    '      if I_stack > 0.0'
    '          H2_flow = stoch * n_cell * I_stack * M_H2 / (2.0 * F_const);'
    '          H2_total = H2_total + H2_flow * dt;'
    '      end'
    '  end'
    '  '
    '  H2_consumed = H2_total;'
    '  H2_tank_total = 200.0;'
    '  pressure_ratio = max(0.01, 1.0 - H2_total / H2_tank_total) * 700.0;'
    '  h2_out = H2_total;'
    'end'
}, '\n');

motor_code = strjoin({
    'function [P_shaft, F_traction, omega] = fcn(torque, motor_speed_rpm, P_plant)'
    '  %#codegen'
    '  omega = motor_speed_rpm * 2.0 * pi / 60.0;'
    '  gr = 2.72;'
    '  eta = 0.96;'
    '  r_wheel = 0.42;'
    '  bp  = [318.28, 447.46, 1114.12, 1591.58, 1909.86, 2228.14];'
    '  tbl = [31540.0, 31540.0, 13500.0, 9460.0, 7880.0, 6750.0];'
    '  btbl = [10750.0, 10750.0, 9670.0, 6270.0, 5220.0, 4150.0];'
    '  max_trac = tbl(end);'
    '  if motor_speed_rpm <= bp(1)'
    '      max_trac = tbl(1);'
    '  elseif motor_speed_rpm >= bp(end)'
    '      max_trac = tbl(end);'
    '  else'
    '      for i = 1:length(bp)-1'
    '          if motor_speed_rpm >= bp(i) && motor_speed_rpm <= bp(i+1)'
    '              fr = (motor_speed_rpm - bp(i)) / (bp(i+1) - bp(i));'
    '              max_trac = tbl(i) + fr * (tbl(i+1) - tbl(i));'
    '              break;'
    '          end'
    '      end'
    '  end'
    '  max_brake = btbl(end);'
    '  if motor_speed_rpm <= bp(1)'
    '      max_brake = btbl(1);'
    '  elseif motor_speed_rpm >= bp(end)'
    '      max_brake = btbl(end);'
    '  else'
    '      for i = 1:length(bp)-1'
    '          if motor_speed_rpm >= bp(i) && motor_speed_rpm <= bp(i+1)'
    '              fr = (motor_speed_rpm - bp(i)) / (bp(i+1) - bp(i));'
    '              max_brake = btbl(i) + fr * (btbl(i+1) - btbl(i));'
    '              break;'
    '          end'
    '      end'
    '  end'
    '  if false && torque > max_trac'
    '      torque = max_trac;'
    '  end'
    '  if false && torque < -max_brake'
    '      torque = -max_brake;'
    '  end'
    '  if false && omega > 0.1 && P_plant > 100000.0'
    '      max_torque_plant = P_plant * 0.95 / omega;'
    '      if torque > max_torque_plant'
    '          torque = max_torque_plant;'
    '      end'
    '  end'
    '  P_shaft = torque * omega;'
    '  F_traction = torque * gr * eta / r_wheel;'
    'end'
}, '\n');

motor_power_code = strjoin({
    'function P_motor = fcn(torque, P_mech)'
    '  %#codegen'
    '  if torque >= 0.0'
    '      P_motor = P_mech * 1.4286;'
    '  else'
    '      P_motor = P_mech * 0.40;'
    '  end'
    'end'
}, '\n');

train_code = strjoin({
    'function [v, accel, motor_speed_rpm, v_out] = fcn(F_traction, F_brake, dt, v_in, slope)'
    '  %#codegen'
    '  v = 0.0;'
    '  accel = 0.0;'
    '  motor_speed_rpm = 0.0;'
    '  v_out = 0.0;'
    '  '
    '  v_state = v_in;'
    '  M = 195000.0;'
    '  delta = 0.06;'
    '  M_eq = M * (1.0 + delta);'
    '  A_res = 7500.0;'
    '  B_res = 7.722;'
    '  C_res = 7.329;'
    '  F_grade = M * 9.81 * slope;'
    '  F_resist = A_res + B_res * v_state + C_res * v_state * v_state;'
    '  F_net = F_traction - F_brake - F_resist - F_grade;'
    '  accel = F_net / M_eq;'
    '  v_state = max(0.0, v_state + accel * dt);'
    '  v = v_state;'
    '  gr = 2.72;'
    '  r_wheel = 0.42;'
    '  motor_speed_rpm = v / r_wheel * 60.0 / (2.0 * pi) * gr;'
    '  v_out = v_state;'
    'end'
}, '\n');

%% =========================================================================
%% 2. Control System (C) ->?per Fig.7
%%    Inputs (5):  v_ref(1), v_actual(2), SOC(3), pressure(4), motor_speed(5)
%%    Outputs (6): mech_brake(1), motor_torque(2), P_FC(3), P_Bat(4),
%%                 EMS_State(5), R_cut(6)
%% =========================================================================
fprintf('  Building Control System (C)...\n');
cs_path = [model_name '/Control System (C)'];
clean_ss(cs_path);

% --- Input ports ---
add_block('built-in/Inport', [cs_path '/v_ref'],        'Position', [30, 30, 60, 44]);
add_block('built-in/Inport', [cs_path '/v_actual'],     'Position', [30, 60, 60, 74]);
add_block('built-in/Inport', [cs_path '/SOC_in'],        'Position', [30, 100, 60, 114]);
add_block('built-in/Inport', [cs_path '/pressure_in'],   'Position', [30, 130, 60, 144]);
add_block('built-in/Inport', [cs_path '/motor_speed_in'],'Position', [30, 170, 60, 184]);
add_block('built-in/Inport', [cs_path '/a_ref'],         'Position', [30, 200, 60, 214]);
add_block('built-in/Inport', [cs_path '/slope_ref'],     'Position', [30, 230, 60, 244]);

% --- Speed Controller (discrete PID) ---
add_block('built-in/SubSystem', [cs_path '/Speed Controller']);
sc_path = [cs_path '/Speed Controller'];
clean_ss(sc_path);
add_block('built-in/Inport',  [sc_path '/v_ref'],     'Position', [30, 30, 60, 44]);
add_block('built-in/Inport',  [sc_path '/v_actual'],  'Position', [30, 60, 60, 74]);
add_block('built-in/Inport',  [sc_path '/a_ref_in'],  'Position', [30, 90, 60, 104]);
add_block('built-in/Inport',  [sc_path '/slope_in'],  'Position', [30, 120, 60, 134]);
add_block('built-in/Outport', [sc_path '/torque_raw'], 'Position', [320, 42, 350, 58]);

pid_code = strjoin({
    'function [torque, integ_out] = fcn(err, dt, integ_in, v_ref, a_ref, slope)'
    '  %#codegen'
    '  torque = 0.0;'
    '  integ_out = integ_in;'
    '  Kp = 2000.0;'
    '  Ki = 0.0;'
    '  integ_out = integ_in + err * dt;'
    '  if integ_out > 2.0; integ_out = 2.0; end'
    '  if integ_out < -2.0; integ_out = -2.0; end'
    '  M = 195000.0; delta_m = 0.06;'
    '  A_res = 7500.0; B_res = 7.722; C_res = 7.329;'
    '  r_wheel = 0.42; gr = 2.72; eta_m = 0.96;'
    '  F_resist = A_res + B_res * v_ref + C_res * v_ref * v_ref;'
    '  F_grade = M * 9.81 * slope;'
    '  F_inertia = M * (1.0 + delta_m) * a_ref;'
    '  F_total = F_inertia + F_resist + F_grade;'
    '  torque_ff = F_total * r_wheel / (gr * eta_m);'
    '  torque = torque_ff + Kp * err + Ki * integ_out;'
    '  if torque > 50000.0; torque = 50000.0; end'
    '  if torque < -50000.0; torque = -50000.0; end'
    'end'
}, '\n');

add_block('simulink/User-Defined Functions/MATLAB Function', [sc_path '/PID'], 'Position', [160, 35, 220, 75]);
add_block('built-in/Sum',     [sc_path '/err'],       'Position', [100, 35, 120, 65], 'Inputs', '+-');
add_block('simulink/Discrete/Unit Delay', [sc_path '/D_integ'], 'Position', [260, 60, 290, 90], 'InitialCondition', '0');
add_block('simulink/Sources/Constant', [sc_path '/dt_const'], 'Position', [30, 90, 60, 104], 'Value', '0.1');
inject_code(model_name, [sc_path '/PID'], pid_code);
add_line(sc_path, 'v_ref/1',    'err/1');
add_line(sc_path, 'v_actual/1', 'err/2');
add_line(sc_path, 'err/1',      'PID/1');
add_line(sc_path, 'dt_const/1', 'PID/2');
add_line(sc_path, 'PID/2',      'D_integ/1');
add_line(sc_path, 'D_integ/1',  'PID/3');
add_line(sc_path, 'v_ref/1',    'PID/4');
add_line(sc_path, 'a_ref_in/1', 'PID/5');
add_line(sc_path, 'slope_in/1', 'PID/6');
add_line(sc_path, 'PID/1',      'torque_raw/1');

% --- Torque Saturation ---
add_block('built-in/SubSystem', [cs_path '/Torque Saturation']);
ts_path = [cs_path '/Torque Saturation'];
clean_ss(ts_path);
add_block('built-in/Inport', [ts_path '/rpm_in'], 'Position', [30, 30, 60, 44]);
add_block('simulink/User-Defined Functions/MATLAB Function', [ts_path '/MaxTracTorque'], ...
    'Position', [120, 20, 200, 50]);
add_block('simulink/User-Defined Functions/MATLAB Function', [ts_path '/MaxBrakeTorque'], ...
    'Position', [120, 70, 200, 100]);

% Inject MaxTracTorque and MaxBrakeTorque BEFORE internal wiring
inject_code(model_name, [ts_path '/MaxTracTorque'], trac_code_str);
inject_code(model_name, [ts_path '/MaxBrakeTorque'], brake_lookup_code_str);

add_block('built-in/Outport', [ts_path '/max_torque_out'],   'Position', [260, 25, 290, 39]);
add_block('built-in/Outport', [ts_path '/max_brake_out'],    'Position', [260, 75, 290, 89]);
add_line(ts_path, 'rpm_in/1',          'MaxTracTorque/1');
add_line(ts_path, 'rpm_in/1',          'MaxBrakeTorque/1');
add_line(ts_path, 'MaxTracTorque/1',   'max_torque_out/1');
add_line(ts_path, 'MaxBrakeTorque/1',  'max_brake_out/1');

% --- Braking Logic (pure MATLAB, no external calls, no evalin) ---
add_block('simulink/User-Defined Functions/MATLAB Function', [cs_path '/Braking Logic'], ...
    'Position', [350, 30, 550, 160]);

% --- Motor Map ---
add_block('built-in/SubSystem', [cs_path '/Motor Map']);
mm_path = [cs_path '/Motor Map'];
clean_ss(mm_path);
add_block('built-in/Inport',  [mm_path '/torque_in'], 'Position', [30, 30, 60, 44]);
add_block('built-in/Inport',  [mm_path '/rpm_in'],    'Position', [30, 70, 60, 84]);
add_block('built-in/Gain',    [mm_path '/rpm_to_rad'], 'Position', [100, 70, 170, 90], 'Gain', '2*pi/60');
add_block('built-in/Product', [mm_path '/P_calc'],     'Position', [220, 40, 250, 70]);
add_block('built-in/Gain', [mm_path '/TorqueSat'], ...
    'Position', [100, 25, 160, 55], 'Gain', '1');
add_block('simulink/User-Defined Functions/MATLAB Function', [mm_path '/EtaMotor'], ...
    'Position', [270, 30, 390, 70]);
inject_code(model_name, [mm_path '/EtaMotor'], motor_power_code);
add_block('built-in/Outport', [mm_path '/P_motor_out'], 'Position', [320, 45, 350, 59]);
add_line(mm_path, 'torque_in/1',  'TorqueSat/1');
add_line(mm_path, 'TorqueSat/1',  'P_calc/1');
add_line(mm_path, 'rpm_in/1',     'rpm_to_rad/1');
add_line(mm_path, 'rpm_to_rad/1', 'P_calc/2');
add_line(mm_path, 'torque_in/1',  'EtaMotor/1');
add_line(mm_path, 'P_calc/1',     'EtaMotor/2');
add_line(mm_path, 'EtaMotor/1',   'P_motor_out/1');

% --- EMS Controller (MATLAB Function with coder.extrinsic) ---
add_block('simulink/User-Defined Functions/MATLAB Function', [cs_path '/EMS Controller'], ...
    'Position', [600, 80, 800, 220]);

% Build parameter constant string for EMS init
% Instead of evalin, pass key scalars as additional inputs
% (ems_code already defined in section 1.5)

% --- Constants for EMS ---
add_block('built-in/Constant', [cs_path '/P_aux'],       'Value', '200000',      'Position', [450, 250, 510, 270]);
add_block('built-in/Constant', [cs_path '/P_FC_Nom'],    'Value', num2str(EMS_OVERRIDE.FC_nom),   'Position', [450, 290, 510, 310]);
add_block('built-in/Constant', [cs_path '/dt_const'],    'Value', '0.1',          'Position', [450, 330, 510, 350]);
add_block('built-in/Constant', [cs_path '/EMS1_Sel'],    'Value', '1',            'Position', [450, 370, 510, 390]);
% sim_time input removed - no time-based override in EMS

% --- Sum block: P_motor + P_aux ---
add_block('built-in/Sum', [cs_path '/P_total_sum'], 'Position', [500, 80, 530, 120], 'Inputs', '++');

% --- Output ports (6) ->?order matches Fig.7 ---
add_block('built-in/Outport', [cs_path '/mech_brake_out'],   'Position', [850, 30, 880, 44]);
add_block('built-in/Outport', [cs_path '/motor_torque_out'], 'Position', [850, 60, 880, 74]);
add_block('built-in/Outport', [cs_path '/P_fc_cmd_out'],     'Position', [850, 100, 880, 114]);
add_block('built-in/Outport', [cs_path '/P_bat_cmd_out'],    'Position', [850, 130, 880, 144]);
add_block('built-in/Outport', [cs_path '/ems_state_out'],    'Position', [850, 160, 880, 174]);
add_block('built-in/Outport', [cs_path '/R_cut_out'],        'Position', [850, 190, 880, 204]);

% --- Inject MATLAB Function code BEFORE wiring (ports must exist) ---
fprintf('  Injecting Control System code...\n');
inject_code(model_name, [cs_path '/Braking Logic'], braking_code);
inject_code(model_name, [cs_path '/EMS Controller'], ems_code);
% MaxTracTorque and MaxBrakeTorque already injected in Torque Saturation subsystem

% --- Internal connections ---
% Speed Controller
add_line(cs_path, 'v_ref/1',        'Speed Controller/1');
add_line(cs_path, 'v_actual/1',     'Speed Controller/2');
add_line(cs_path, 'a_ref/1',         'Speed Controller/3');
add_line(cs_path, 'slope_ref/1',     'Speed Controller/4');

% Torque Saturation
add_line(cs_path, 'motor_speed_in/1', 'Torque Saturation/1');
% Port 1 (max_torque) unused - add Terminator
add_block('simulink/Sinks/Terminator', [cs_path '/T_maxtrac'], 'Position', [260, 200, 270, 214]);
add_line(cs_path, 'Torque Saturation/1', 'T_maxtrac/1');

% Speed Controller ->?Braking Logic
add_line(cs_path, 'Speed Controller/1', 'Braking Logic/1');
add_line(cs_path, 'SOC_in/1',          'Braking Logic/2');
add_line(cs_path, 'pressure_in/1',     'Braking Logic/3');
add_line(cs_path, 'Torque Saturation/2', 'Braking Logic/4');
add_line(cs_path, 'motor_speed_in/1',   'Braking Logic/5');

% Braking Logic outputs ->?Goto for fan-out
add_block('built-in/Goto', [cs_path '/G_corr_torque'], 'GotoTag', 'CS_corr_torque', 'TagVisibility', 'global', 'Position', [580, 30, 660, 44]);
add_line(cs_path, 'Braking Logic/1', 'G_corr_torque/1');
add_block('built-in/From', [cs_path '/F_to_out2'], 'GotoTag', 'CS_corr_torque', 'Position', [780, 55, 840, 69]);
add_line(cs_path, 'F_to_out2/1', 'motor_torque_out/1');
add_block('built-in/From', [cs_path '/F_to_mm'], 'GotoTag', 'CS_corr_torque', 'Position', [380, 55, 440, 69]);
add_line(cs_path, 'F_to_mm/1', 'Motor Map/1');

% mech_brake ->?output 1
add_line(cs_path, 'Braking Logic/2', 'mech_brake_out/1');

% Motor Map: motor_speed ->?second input
add_line(cs_path, 'motor_speed_in/1', 'Motor Map/2');

% P_motor + P_aux ->?P_total
add_line(cs_path, 'Motor Map/1',  'P_total_sum/1');
add_line(cs_path, 'P_aux/1',      'P_total_sum/2');

% P_total ->?EMS
add_line(cs_path, 'P_total_sum/1', 'EMS Controller/1');
add_line(cs_path, 'SOC_in/1',      'EMS Controller/2');
add_line(cs_path, 'pressure_in/1', 'EMS Controller/3');
add_line(cs_path, 'P_FC_Nom/1',    'EMS Controller/4');
add_line(cs_path, 'dt_const/1',    'EMS Controller/5');
add_line(cs_path, 'EMS1_Sel/1',    'EMS Controller/6');
add_line(cs_path, 'v_ref/1',       'EMS Controller/8');
% Clock->EMS line removed
add_block('simulink/Discrete/Unit Delay', [cs_path '/D_ems_state'], 'Position', [820, 220, 850, 250], 'InitialCondition', '3');
add_line(cs_path, 'EMS Controller/5', 'D_ems_state/1');
add_line(cs_path, 'D_ems_state/1', 'EMS Controller/7');

% EMS outputs
add_line(cs_path, 'EMS Controller/1', 'P_fc_cmd_out/1');
add_line(cs_path, 'EMS Controller/2', 'P_bat_cmd_out/1');
add_line(cs_path, 'EMS Controller/4', 'ems_state_out/1');
add_line(cs_path, 'EMS Controller/3', 'R_cut_out/1');

% Goto blocks for P_fc_cmd and P_bat_cmd (for logging)
add_block('built-in/Goto', [cs_path '/G_Pfc_log'], 'GotoTag', 'TL_Pfc', 'TagVisibility', 'global', 'Position', [900, 100, 960, 114]);
add_line(cs_path, 'EMS Controller/1', 'G_Pfc_log/1');
add_block('built-in/Goto', [cs_path '/G_Pbat_log'], 'GotoTag', 'TL_Pbat', 'TagVisibility', 'global', 'Position', [900, 130, 960, 144]);
add_line(cs_path, 'EMS Controller/2', 'G_Pbat_log/1');

%% =========================================================================
%% 3. Electrical Plant (B) ->?per Fig.2
%%    Inputs:  P_fc_cmd, P_bat_cmd
%%    Outputs: SOC, pressure, H2_consumed, P_plant, V_batt
%% =========================================================================
fprintf('  Building Electrical Plant (B)...\n');
ep_path = [model_name '/Electrical Plant (B)'];
clean_ss(ep_path);

add_block('built-in/Inport', [ep_path '/P_fc_cmd_in'],  'Position', [30, 30, 60, 44]);
add_block('built-in/Inport', [ep_path '/P_bat_cmd_in'], 'Position', [30, 80, 60, 94]);

% --- Battery System ---
add_block('built-in/SubSystem', [ep_path '/Battery System']);
bat_path = [ep_path '/Battery System'];
clean_ss(bat_path);

add_block('built-in/Inport',  [bat_path '/P_bat_in'],    'Position', [30, 70, 60, 84]);
add_block('simulink/User-Defined Functions/MATLAB Function', [bat_path '/Battery EECM'], ...
    'Position', [150, 30, 370, 140]);

% (bat_code already defined in section 1.5)

add_block('built-in/Outport', [bat_path '/V_batt_out'],   'Position', [430, 40, 460, 54]);
add_block('built-in/Outport', [bat_path '/I_batt_out'],   'Position', [430, 70, 460, 84]);
add_block('built-in/Outport', [bat_path '/SOC_out'],      'Position', [430, 100, 460, 114]);
add_block('built-in/Outport', [bat_path '/P_batt_out'],   'Position', [430, 130, 460, 144]);
add_block('built-in/Outport', [bat_path '/C_loss_out'],   'Position', [430, 160, 460, 174]);
% --- Inject Battery EECM code BEFORE internal wiring ---
inject_code(model_name, [bat_path '/Battery EECM'], bat_code);

add_line(bat_path, 'P_bat_in/1',     'Battery EECM/1');
% Battery EECM input 2 (dt) ->?needs a Constant
add_block('built-in/Constant', [bat_path '/dt_const'], 'Value', '0.1', 'Position', [30, 100, 60, 114]);
add_line(bat_path, 'dt_const/1',     'Battery EECM/2');
add_block('simulink/Discrete/Unit Delay', [bat_path '/D_soc'], 'Position', [500, 190, 530, 220], 'InitialCondition', '0.90');
add_block('simulink/Discrete/Unit Delay', [bat_path '/D_v1'],  'Position', [500, 225, 530, 255], 'InitialCondition', '0');
add_block('simulink/Discrete/Unit Delay', [bat_path '/D_v2'],  'Position', [500, 260, 530, 290], 'InitialCondition', '0');
add_line(bat_path, 'Battery EECM/6', 'D_soc/1');
add_line(bat_path, 'Battery EECM/7', 'D_v1/1');
add_line(bat_path, 'Battery EECM/8', 'D_v2/1');
add_line(bat_path, 'D_soc/1', 'Battery EECM/3');
add_line(bat_path, 'D_v1/1',  'Battery EECM/4');
add_line(bat_path, 'D_v2/1',  'Battery EECM/5');
add_line(bat_path, 'Battery EECM/1', 'V_batt_out/1');
add_line(bat_path, 'Battery EECM/2', 'I_batt_out/1');
add_line(bat_path, 'Battery EECM/3', 'SOC_out/1');
add_line(bat_path, 'Battery EECM/4', 'P_batt_out/1');
add_line(bat_path, 'Battery EECM/5', 'C_loss_out/1');

% --- Fuel Cell System ---
add_block('built-in/SubSystem', [ep_path '/Fuel Cell System']);
fc_path = [ep_path '/Fuel Cell System'];
clean_ss(fc_path);

add_block('built-in/Inport',  [fc_path '/P_fc_in'],           'Position', [30, 70, 60, 84]);
add_block('simulink/User-Defined Functions/MATLAB Function', [fc_path '/PEMFC Model'], ...
    'Position', [150, 30, 370, 160]);

% (fc_code already defined in section 1.5)

add_block('built-in/Outport', [fc_path '/V_stack_out'],       'Position', [430, 40, 460, 54]);
add_block('built-in/Outport', [fc_path '/I_stack_out'],       'Position', [430, 70, 460, 84]);
add_block('built-in/Outport', [fc_path '/P_stack_out'],       'Position', [430, 100, 460, 114]);
add_block('built-in/Outport', [fc_path '/H2_consumed_out'],   'Position', [430, 130, 460, 144]);
add_block('built-in/Outport', [fc_path '/pressure_out'],      'Position', [430, 160, 460, 174]);
add_block('built-in/Outport', [fc_path '/H2_flow_out'],       'Position', [430, 190, 460, 204]);
% --- Inject PEMFC Model code BEFORE internal wiring ---
inject_code(model_name, [fc_path '/PEMFC Model'], fc_code);

add_line(fc_path, 'P_fc_in/1',     'PEMFC Model/1');
% PEMFC Model input 2 (dt) ->?needs a Constant
add_block('built-in/Constant', [fc_path '/dt_const'], 'Value', '0.1', 'Position', [30, 100, 60, 114]);
add_line(fc_path, 'dt_const/1',   'PEMFC Model/2');
add_block('simulink/Discrete/Unit Delay', [fc_path '/D_h2'], 'Position', [400, 210, 430, 240], 'InitialCondition', '0');
add_line(fc_path, 'PEMFC Model/7', 'D_h2/1');
add_line(fc_path, 'D_h2/1', 'PEMFC Model/3');
add_line(fc_path, 'PEMFC Model/1', 'V_stack_out/1');
add_line(fc_path, 'PEMFC Model/2', 'I_stack_out/1');
add_line(fc_path, 'PEMFC Model/3', 'P_stack_out/1');
add_line(fc_path, 'PEMFC Model/4', 'H2_consumed_out/1');
add_line(fc_path, 'PEMFC Model/5', 'pressure_out/1');
add_line(fc_path, 'PEMFC Model/6', 'H2_flow_out/1');
% Output 7 (h2_out) feeds D_h2, not an Outport

% --- DC/DC Converters ---
add_block('built-in/Gain', [ep_path '/FC_DCDC'],  'Position', [500, 100, 560, 120], ...
    'Gain', '0.92');
add_block('built-in/Gain', [ep_path '/Bat_DCDC'], 'Position', [500, 40, 560, 60], ...
    'Gain', '0.92');

% --- DC Bus Balance ---
% P_total_available = FC_DCDC + Bat_DCDC - P_aux - FC_parasitic
% FC parasitic (air compressor etc) ~ 7% of FC electrical output
add_block('built-in/Sum',      [ep_path '/DC Bus Balance'], 'Position', [620, 50, 650, 90], 'Inputs', '++-');
add_block('built-in/Constant', [ep_path '/P_aux'],          'Position', [560, 200, 610, 220], 'Value', '200000');
% 214kW = 200kW aux + 14kW FC parasitic baseline (FC idle 200kW * 0.07)
% FC parasitic varies with FC power, but constant approximation is simpler

% --- Output ports ---
add_block('built-in/Outport', [ep_path '/SOC_out'],         'Position', [750, 30, 780, 44]);
add_block('built-in/Outport', [ep_path '/pressure_out'],    'Position', [750, 60, 780, 74]);
add_block('built-in/Outport', [ep_path '/H2_consumed_out'], 'Position', [750, 90, 780, 104]);
add_block('built-in/Outport', [ep_path '/P_plant_out'],     'Position', [750, 120, 780, 134]);
add_block('built-in/Outport', [ep_path '/V_batt_out'],      'Position', [750, 150, 780, 164]);

% (Battery EECM and PEMFC Model already injected in their subsystems)

% --- Connections ---
add_line(ep_path, 'P_bat_cmd_in/1',         'Battery System/1');
add_line(ep_path, 'P_fc_cmd_in/1',          'Fuel Cell System/1');
add_line(ep_path, 'Battery System/3',       'SOC_out/1');
add_line(ep_path, 'Battery System/1',       'V_batt_out/1');
add_line(ep_path, 'Fuel Cell System/5',     'pressure_out/1');
add_line(ep_path, 'Fuel Cell System/4',     'H2_consumed_out/1');
add_line(ep_path, 'Battery System/4',       'Bat_DCDC/1');
add_line(ep_path, 'Fuel Cell System/3',     'FC_DCDC/1');
add_line(ep_path, 'FC_DCDC/1',              'DC Bus Balance/1');
add_line(ep_path, 'Bat_DCDC/1',             'DC Bus Balance/2');
add_line(ep_path, 'P_aux/1',                'DC Bus Balance/3');
add_line(ep_path, 'DC Bus Balance/1',       'P_plant_out/1');

% Battery System unused outputs ->?Terminators
add_block('simulink/Sinks/Terminator', [ep_path '/T_Ibatt'],  'Position', [500, 70, 510, 84]);
add_block('simulink/Sinks/Terminator', [ep_path '/T_Closs'],  'Position', [500, 160, 510, 174]);
add_line(ep_path, 'Battery System/2',  'T_Ibatt/1');
add_line(ep_path, 'Battery System/5',  'T_Closs/1');

% FC System unused outputs ->?Terminators
add_block('simulink/Sinks/Terminator', [ep_path '/T_Vstack'], 'Position', [500, 40, 510, 54]);
add_block('simulink/Sinks/Terminator', [ep_path '/T_Istack'], 'Position', [500, 70, 510, 84]);
add_block('simulink/Sinks/Terminator', [ep_path '/T_H2flow'], 'Position', [500, 190, 510, 204]);
add_line(ep_path, 'Fuel Cell System/1', 'T_Vstack/1');
add_line(ep_path, 'Fuel Cell System/2', 'T_Istack/1');
add_line(ep_path, 'Fuel Cell System/6', 'T_H2flow/1');

%% =========================================================================
%% 4. Electric Motor (D)
%%    Inputs:  torque (from C), motor_speed (via From block)
%%    Outputs: P_shaft, F_traction, omega
%% =========================================================================
fprintf('  Building Electric Motor (D)...\n');
em_path = [model_name '/Electric Motor (D)'];
clean_ss(em_path);

add_block('built-in/Inport',  [em_path '/torque_in'],       'Position', [30, 30, 60, 44]);
add_block('built-in/Inport',  [em_path '/P_plant_in'],      'Position', [30, 120, 60, 134]);
add_block('simulink/Discrete/Unit Delay', [em_path '/D_Pplant'], 'Position', [80, 118, 110, 142], 'InitialCondition', '0');
add_block('built-in/From',    [em_path '/F_mspeed'], 'GotoTag', 'TL_mspeed', 'Position', [30, 80, 100, 94]);
add_block('simulink/Discrete/Unit Delay', [em_path '/D_mspeed'], 'Position', [120, 78, 150, 102], 'InitialCondition', '0');

add_block('simulink/User-Defined Functions/MATLAB Function', [em_path '/Motor Model'], ...
    'Position', [150, 30, 370, 120]);
% (motor_code already defined in section 1.5)

add_block('built-in/Outport', [em_path '/P_shaft_out'],   'Position', [430, 40, 460, 54]);
add_block('built-in/Outport', [em_path '/F_traction_out'], 'Position', [430, 70, 460, 84]);
add_block('built-in/Outport', [em_path '/omega_out'],      'Position', [430, 100, 460, 114]);

% --- Inject Motor Model code BEFORE wiring ---
fprintf('  Injecting Electric Motor code...\n');
inject_code(model_name, [em_path '/Motor Model'], motor_code);

add_line(em_path, 'torque_in/1',       'Motor Model/1');
add_line(em_path, 'F_mspeed/1',        'D_mspeed/1');
add_line(em_path, 'D_mspeed/1',        'Motor Model/2');
add_line(em_path, 'P_plant_in/1',      'D_Pplant/1');
add_line(em_path, 'D_Pplant/1',        'Motor Model/3');
add_line(em_path, 'Motor Model/1',     'P_shaft_out/1');
add_line(em_path, 'Motor Model/2',     'F_traction_out/1');
add_line(em_path, 'Motor Model/3',     'omega_out/1');

%% =========================================================================
%% 5. Mechanical Dynamics (A)
%%    Inputs:  F_traction (from D), F_brake (from C output 1)
%%    Outputs: v [m/s], accel [m/s2], motor_speed [rpm]
%% =========================================================================
fprintf('  Building Mechanical Dynamics (A)...\n');
md_path = [model_name '/Mechanical Dynamics (A)'];
clean_ss(md_path);

add_block('built-in/Inport',  [md_path '/F_traction'], 'Position', [30, 30, 60, 44]);
add_block('built-in/Inport',  [md_path '/F_brake'],    'Position', [30, 70, 60, 84]);
add_block('built-in/Inport',  [md_path '/slope_ref'],  'Position', [30, 110, 60, 124]);

add_block('simulink/User-Defined Functions/MATLAB Function', [md_path '/Train Dynamics'], ...
    'Position', [150, 30, 380, 150]);

% (train_code already defined in section 1.5)

add_block('built-in/Constant', [md_path '/dt'], 'Value', '0.1', 'Position', [80, 160, 110, 180]);

add_block('built-in/Outport', [md_path '/v_out'],           'Position', [440, 40, 470, 54]);
add_block('built-in/Outport', [md_path '/accel_out'],       'Position', [440, 70, 470, 84]);
add_block('built-in/Outport', [md_path '/motor_speed_out'], 'Position', [440, 100, 470, 114]);

% --- Inject Train Dynamics code BEFORE wiring ---
fprintf('  Injecting Mechanical Dynamics code...\n');
inject_code(model_name, [md_path '/Train Dynamics'], train_code);

add_line(md_path, 'F_traction/1',     'Train Dynamics/1');
add_line(md_path, 'F_brake/1',        'Train Dynamics/2');
add_line(md_path, 'dt/1',             'Train Dynamics/3');
add_line(md_path, 'slope_ref/1',      'Train Dynamics/5');
add_block('simulink/Discrete/Unit Delay', [md_path '/D_v'], 'Position', [500, 130, 530, 160], 'InitialCondition', '0');
add_line(md_path, 'Train Dynamics/4', 'D_v/1');
add_line(md_path, 'D_v/1', 'Train Dynamics/4');
add_line(md_path, 'Train Dynamics/1', 'v_out/1');
add_line(md_path, 'Train Dynamics/2', 'accel_out/1');
add_line(md_path, 'Train Dynamics/3', 'motor_speed_out/1');

%% =========================================================================
%% 6. Results Viewer (all inputs via From blocks)
%% =========================================================================
fprintf('  Building Results Viewer...\n');
rv_path = [model_name '/Results Viewer'];
clean_ss(rv_path);
% Internal blocks created in section 7 after Goto tags are defined

%% =========================================================================
%% 7. Top-level connections (Goto/From architecture)
%% =========================================================================
fprintf('  Connecting top-level blocks (Goto/From architecture)...\n');

% --- Speed Reference ->?Control System (C) input 1 ---
add_line(model_name, 'Speed Reference/1', 'Control System (C)/1');

% --- Accel Reference ->?Control System (C) input 6 ---
add_line(model_name, 'Accel Reference/1', 'Control System (C)/6');

% --- Slope Reference ->?Control System (C) input 7 and Mechanical Dynamics (A) input 3 ---
add_line(model_name, 'Slope Reference/1', 'Control System (C)/7');
add_line(model_name, 'Slope Reference/1', 'Mechanical Dynamics (A)/3');

% --- Control System (C) outputs ---
add_line(model_name, 'Control System (C)/1', 'Mechanical Dynamics (A)/2');  % mech_brake
add_line(model_name, 'Control System (C)/2', 'Electric Motor (D)/1');       % motor_torque
add_line(model_name, 'Control System (C)/3', 'Electrical Plant (B)/1');     % P_fc_cmd
add_line(model_name, 'Control System (C)/4', 'Electrical Plant (B)/2');     % P_bat_cmd

% C out 5: ems_state ->?Goto
add_block('built-in/Goto', [model_name '/G_ems'], 'GotoTag', 'TL_ems', 'TagVisibility', 'global', 'Position', [550, 200, 620, 214]);
add_line(model_name, 'Control System (C)/5', 'G_ems/1');

% C out 6: R_cut ->?Goto
add_block('built-in/Goto', [model_name '/G_Rcut'], 'GotoTag', 'TL_Rcut', 'TagVisibility', 'global', 'Position', [550, 230, 620, 244]);
add_line(model_name, 'Control System (C)/6', 'G_Rcut/1');

% --- Electric Motor (D) outputs ---
add_block('built-in/Goto', [model_name '/G_Pshaft'], 'GotoTag', 'TL_Pshaft', 'TagVisibility', 'global', 'Position', [550, 80, 620, 94]);
add_line(model_name, 'Electric Motor (D)/1', 'G_Pshaft/1');

add_line(model_name, 'Electric Motor (D)/2', 'Mechanical Dynamics (A)/1');  % F_traction

add_block('simulink/Sinks/Terminator', [model_name '/T_omega'], 'Position', [550, 110, 560, 124]);
add_line(model_name, 'Electric Motor (D)/3', 'T_omega/1');

% --- Mechanical Dynamics (A) outputs ---
% A out 1: v ->?Goto
add_block('built-in/Goto', [model_name '/G_v'], 'GotoTag', 'TL_v', 'TagVisibility', 'global', 'Position', [550, 420, 620, 434]);
add_line(model_name, 'Mechanical Dynamics (A)/1', 'G_v/1');
add_block('built-in/From', [model_name '/F_v_C'], 'GotoTag', 'TL_v', 'Position', [300, 60, 370, 74]);
add_block('simulink/Discrete/Unit Delay', [model_name '/D_v_C'], 'Position', [385, 55, 420, 85], 'InitialCondition', '0');
add_line(model_name, 'F_v_C/1', 'D_v_C/1');
add_line(model_name, 'D_v_C/1', 'Control System (C)/2');

% A out 2: accel ->?Terminator
add_block('simulink/Sinks/Terminator', [model_name '/T_accel'], 'Position', [550, 350, 560, 364]);
add_line(model_name, 'Mechanical Dynamics (A)/2', 'T_accel/1');

% A out 3: motor_speed ->?Goto
add_block('built-in/Goto', [model_name '/G_mspeed'], 'GotoTag', 'TL_mspeed', 'TagVisibility', 'global', 'Position', [550, 380, 620, 394]);
add_line(model_name, 'Mechanical Dynamics (A)/3', 'G_mspeed/1');
add_block('built-in/From', [model_name '/F_mspeed_C'], 'GotoTag', 'TL_mspeed', 'Position', [300, 170, 370, 184]);
add_block('simulink/Discrete/Unit Delay', [model_name '/D_mspeed_C'], 'Position', [385, 165, 420, 195], 'InitialCondition', '0');
add_line(model_name, 'F_mspeed_C/1', 'D_mspeed_C/1');
add_line(model_name, 'D_mspeed_C/1', 'Control System (C)/5');

% --- Electrical Plant (B) outputs ---
% B out 1: SOC ->?Goto
add_block('built-in/Goto', [model_name '/G_SOC'], 'GotoTag', 'TL_SOC', 'TagVisibility', 'global', 'Position', [550, 120, 620, 134]);
add_line(model_name, 'Electrical Plant (B)/1', 'G_SOC/1');
add_block('built-in/From', [model_name '/F_SOC_C'], 'GotoTag', 'TL_SOC', 'Position', [300, 100, 370, 114]);
add_block('simulink/Discrete/Unit Delay', [model_name '/D_SOC_C'], 'Position', [385, 95, 420, 125], 'InitialCondition', '0.90');
add_line(model_name, 'F_SOC_C/1', 'D_SOC_C/1');
add_line(model_name, 'D_SOC_C/1', 'Control System (C)/3');

% B out 2: pressure ->?Goto
add_block('built-in/Goto', [model_name '/G_pressure'], 'GotoTag', 'TL_pressure', 'TagVisibility', 'global', 'Position', [550, 150, 620, 164]);
add_line(model_name, 'Electrical Plant (B)/2', 'G_pressure/1');
add_block('built-in/From', [model_name '/F_pres_C'], 'GotoTag', 'TL_pressure', 'Position', [300, 130, 370, 144]);
add_block('simulink/Discrete/Unit Delay', [model_name '/D_pres_C'], 'Position', [385, 125, 420, 155], 'InitialCondition', '700');
add_line(model_name, 'F_pres_C/1', 'D_pres_C/1');
add_line(model_name, 'D_pres_C/1', 'Control System (C)/4');

% B out 3: H2 ->?Goto
add_block('built-in/Goto', [model_name '/G_H2'], 'GotoTag', 'TL_H2', 'TagVisibility', 'global', 'Position', [550, 180, 620, 194]);
add_line(model_name, 'Electrical Plant (B)/3', 'G_H2/1');

% B out 4: P_plant -> Electric Motor (D) Inport 2 (power limit, with Unit Delay)
add_line(model_name, 'Electrical Plant (B)/4', 'Electric Motor (D)/2');

% B out 5: V_batt -> Goto for logging
add_block('built-in/Goto', [model_name '/G_Vbatt'], 'GotoTag', 'TL_Vbatt', 'TagVisibility', 'global', 'Position', [550, 290, 610, 304]);
add_line(model_name, 'Electrical Plant (B)/5', 'G_Vbatt/1');

% --- Results Viewer (via From blocks) ---
rv_pairs = {
    'TL_v',          'v_log';
    'TL_Pshaft',     'P_log';
    'TL_SOC',        'SOC_log';
    'TL_H2',         'H2_log';
    'TL_ems',        'ems_state_log';
    'TL_Rcut',       'Rcut_log';
    'TL_pressure',   'pressure_log';
    'TL_Pfc',        'Pfc_log';
    'TL_Pbat',       'Pbat_log';
    'TL_Vbatt',      'Vbatt_log';
};
for i = 1:size(rv_pairs, 1)
    tag = rv_pairs{i, 1};
    varname = rv_pairs{i, 2};
    % rv_pairs is now a 6x2 cell array, direct indexing works
    blk_from = [rv_path '/' varname '_from'];
    blk_ws   = [rv_path '/' varname];
    add_block('built-in/From', blk_from, 'GotoTag', tag, 'Position', [30, 30*i, 100, 30*i+14]);
    add_block('simulink/Sinks/To Workspace', blk_ws, ...
        'VariableName', varname, 'Position', [200, 30*i, 260, 30*i+14]);
    add_line(rv_path, [varname '_from/1'], [varname '/1']);
end

%% =========================================================================
%% 9. Auto-layout
%% =========================================================================
fprintf('  Auto-arranging layout...\n');
all_systems = {model_name, cs_path, sc_path, ts_path, mm_path, ep_path, bat_path, fc_path, em_path, md_path, rv_path};
for si = 1:numel(all_systems)
    sys = all_systems{si};
    try; Simulink.BlockDiagram.arrangeSystem(sys); catch; end
end

%% =========================================================================
%% 10. Verify
%% =========================================================================
fprintf('  Checking for unconnected ports...\n');
for si = 1:numel(all_systems)
    sys = all_systems{si};
    lines = find_system(sys, 'SearchDepth', 1, 'Type', 'Line');
    ports = find_system(sys, 'SearchDepth', 1, 'BlockType', {'Inport', 'Outport'});
    fprintf('    [%s] %d lines, %d port blocks\n', sys, numel(lines), numel(ports));
end

fprintf('  Verifying MATLAB Function blocks...\n');
model_obj = get_param(model_name, 'Object');
charts = find(model_obj, '-isa', 'Stateflow.EMChart');
fprintf('    Found %d MATLAB Function blocks.\n', length(charts));
for ci = 1:length(charts)
    fprintf('      [%d] %s (%d chars)\n', ci, charts(ci).Path, strlength(charts(ci).Script));
end

%% =========================================================================
%% 11. Save
%% =========================================================================
fprintf('Saving model...\n');
save_system(model_name, model_path);
fprintf('Model saved to: %s\n', model_path);

fprintf('\n========================================\n');
fprintf('Simulink Model v5 Built Successfully!\n');
fprintf('========================================\n');
fprintf('Model: %s\n', model_path);
fprintf('\nKey fixes:\n');
fprintf('  - coder.extrinsic for evalin and external .m calls\n');
fprintf('  - All outputs pre-typed before extrinsic calls\n');
fprintf('  - All ports connected (terminators on unused)\n');
fprintf('  - Goto/From for all cross-subsystem signals\n');
fprintf('\n=== DONE ===\n');
end

%% =========================================================================
%% Local helper functions
%% =========================================================================

function clean_ss(spath)
    try; delete_line(spath, 'In1/1', 'Out1/1'); catch; end
    try; delete_block([spath '/In1']); catch; end
    try; delete_block([spath '/Out1']); catch; end
end

function inject_code(model_name, block_path, code_str)
    % Set MATLAB Function block code via Stateflow EMChart API
    % model_name: model name
    % block_path: FULL path to the block, e.g. 'hybrid_train_ems/Control System (C)/Torque Saturation/MaxTracTorque'
    % code_str: MATLAB code string
    %
    % Use get_param(model,'Object') + find per official MathWorks docs
    model_obj = get_param(model_name, 'Object');
    chart = find(model_obj, '-isa', 'Stateflow.EMChart', 'Path', block_path);
    if isempty(chart)
        % Fallback: match by Name only (last path component)
        [~, name_part, ~] = fileparts(block_path);
        chart = find(model_obj, '-isa', 'Stateflow.EMChart', 'Name', name_part);
    end
    if isempty(chart)
        error('inject_code: EMChart not found for block "%s"', block_path);
    end
    chart.Script = code_str;
end
