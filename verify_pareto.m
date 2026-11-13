function verify_pareto(csv_path)
% VERIFY_PARETO  Cross-check Pareto-front members on the high-fidelity model.
%
%   verify_pareto                       % uses results/pareto_front_v2.csv
%   verify_pareto('path/to/front.csv')
%
% For each selected front member (plus the paper EMS1 baseline) the decision
% vector is injected into build_simulink_model_v5, one full 12 000 s journey is
% simulated, and degradation + TLC are recomputed with compute_degradation.m --
% the same equations (and the same constants) the Python surrogate implements.
%
% The point of this script is not to re-tune anything: it answers "does the
% cheap surrogate rank the points the same way the detailed model does?" by
% comparing H2, SOC_end, SOC_min, TLC and replacement counts side by side.
%
% Prerequisites: MATLAB R2018b+, Simulink, PLECS Blockset, and the files of
% this repository on the path (run from the repository root).

if nargin < 1 || isempty(csv_path)
    csv_path = fullfile(fileparts(mfilename('fullpath')), 'results', 'pareto_front_v2.csv');
end
if exist(csv_path, 'file') ~= 2
    error('verify_pareto:noFront', ...
        'Pareto front not found: %s\nRun mrfo_multiobjective_v2.py first.', csv_path);
end

cd(fileparts(mfilename('fullpath')));
addpath(fileparts(mfilename('fullpath')));

raw  = table2array(readtable(csv_path));   % readmatrix is R2019a+
X_pf = raw(:, 1:5);          % [SOC_low SOC_high FC_nom_s FC_idle_s SOC_regen]
O_pf = raw(:, 6:8);          % [TLC N_rep_FC N_rep_BAT]

% Three representative members: cheapest, median and most expensive TLC.
[~, order] = sort(O_pf(:, 1));
pick = unique(order(round(linspace(1, numel(order), min(3, numel(order))))));
X_sel = X_pf(pick, :);
O_sel = O_pf(pick, :);

% Paper EMS1 baseline, expressed in the same decision space.
X_base = [0.30, 0.70, 1.0, 200000.0 / 820300.0, 0.90];
names  = [arrayfun(@(k) sprintf('Pareto %d', k), 1:size(X_sel, 1), ...
                   'UniformOutput', false), {'EMS1 baseline'}];
X_all  = [X_sel; X_base];
O_ref  = [O_sel; NaN(1, 3)];        % baseline has no surrogate prediction in the CSV

sim_params = params();             % repository copy of the calibrated parameters
P_nom_base = sim_params.fc.P_nom;

simres = nan(size(X_all, 1), 8);   % H2 SOC_end SOC_min State1 State3 TLC FC_rep Bat_rep

for k = 1:size(X_all, 1)
    x = X_all(k, :);
    ov.SOC_low   = x(1);
    ov.SOC_high  = x(2);
    ov.FC_nom    = x(3) * P_nom_base;
    ov.FC_idle   = x(4) * P_nom_base;
    ov.SOC_regen = x(5);

    fprintf('\n===== %s =====\n', names{k});
    fprintf('  SOC_low=%.4f SOC_high=%.4f FC_nom=%.1f kW FC_idle=%.1f kW SOC_regen=%.4f\n', ...
        ov.SOC_low, ov.SOC_high, ov.FC_nom / 1e3, ov.FC_idle / 1e3, ov.SOC_regen);

    build_simulink_model_v5(ov);
    set_param('hybrid_train_ems', 'StopTime', '12000');
    simOut = sim('hybrid_train_ems', 'ReturnWorkspaceOutputs', 'on');

    sim_data.t      = getlog(simOut, 'tout');
    sim_data.P_fc   = getlog(simOut, 'Pfc_log');
    sim_data.P_bat  = getlog(simOut, 'Pbat_log');
    sim_data.H2     = getlog(simOut, 'H2_log');
    sim_data.SOC    = getlog(simOut, 'SOC_log');
    sim_data.V_batt = getlog(simOut, 'Vbatt_log');
    ems             = getlog(simOut, 'ems_state_log');

    % Degradation model must see the FC actually built into the model.
    sim_params.fc.P_nom  = ov.FC_nom;
    sim_params.fc.P_idle = ov.FC_idle;
    sim_params.fc.P_max  = 1.1 * ov.FC_nom;

    [fc_deg, bat_deg, TLC] = compute_degradation(sim_data, sim_params);

    simres(k, :) = [sim_data.H2(end), sim_data.SOC(end), min(sim_data.SOC), ...
                    mean(ems == 1) * 100, mean(ems == 3) * 100, ...
                    TLC, fc_deg.N_rep, bat_deg.N_rep];

    fprintf('  -> H2=%.2f kg  SOC_end=%.3f  SOC_min=%.3f  TLC=%.0f EUR  FC_rep=%.2f  Bat_rep=%.2f\n', ...
        simres(k, 1), simres(k, 2), simres(k, 3), simres(k, 6), ...
        simres(k, 7), simres(k, 8));
end

%% ------------------------------------------------------------------ report
fprintf('\n\n================= SURROGATE vs DETAILED MODEL =================\n');
fprintf('%-14s | %10s %10s %8s | %7s %7s %7s\n', ...
    'Point', 'TLC_py', 'TLC_sim', 'err[%]', 'FC_py', 'FC_sim', 'Bat_py');
for k = 1:size(X_all, 1)
    if isnan(O_ref(k, 1))
        tlc_py = '-'; err = '-';
    else
        tlc_py = sprintf('%10.0f', O_ref(k, 1));
        err    = sprintf('%8.2f', 100 * (simres(k, 6) - O_ref(k, 1)) / O_ref(k, 1));
    end
    fprintf('%-14s | %10s %10.0f %8s | %7.2f %7.2f %7.2f\n', names{k}, ...
        tlc_py, simres(k, 6), err, ...
        O_ref(k, 2), simres(k, 7), O_ref(k, 3));
end
fprintf('\n%-14s | %8s %8s | %8s %8s\n', ...
    'Point', 'Bat_sim', 'SOCend', 'SOCmin', 'State1%');
for k = 1:size(X_all, 1)
    fprintf('%-14s | %8.2f %8.3f | %8.3f %8.1f\n', names{k}, ...
        simres(k, 8), simres(k, 2), simres(k, 3), simres(k, 4));
end
fprintf('Paper EMS1 reference TLC: 9,191,965 EUR\n');
fprintf('===============================================================\n');

%% ------------------------------------------------------------ persist
out_dir = fullfile(fileparts(mfilename('fullpath')), 'results');
if exist(out_dir, 'dir') ~= 7
    mkdir(out_dir);
end
T = array2table([X_all, simres], 'VariableNames', ...
    {'SOC_low', 'SOC_high', 'FC_nom_scale', 'FC_idle_scale', 'SOC_regen', ...
     'H2_kg', 'SOC_end', 'SOC_min', 'State1_pct', 'State3_pct', ...
     'TLC_EUR', 'N_rep_FC', 'N_rep_BAT'});
T = [table(string(names'), 'VariableNames', {'point'}), T];   % addvars is R2019b+
writetable(T, fullfile(out_dir, 'verification_v2.csv'));
fprintf('Saved results/verification_v2.csv\n');
end


function y = getlog(simOut, name)
% GETLOG  Accept both timeseries and structure-with-time logging formats.
v = simOut.(name);
if isa(v, 'timeseries')
    y = v.Data;
elseif isstruct(v) && isfield(v, 'signals')
    y = v.signals.values;
else
    y = v;
end
y = double(squeeze(y));
y = y(:);
end
