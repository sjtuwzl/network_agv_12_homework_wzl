%% run_simulink_single_scan.m
% Single-AGV Simulink scan over packet-loss probability p.
% Compatible with To Workspace outputs saved as:
% - timeseries
% - Structure With Time
% - SimulationData.Signal (when possible)

clear; clc;

this_file = mfilename('fullpath');
this_dir = fileparts(this_file);
project_root = fileparts(this_dir);

mdl = fullfile(project_root, 'slimulink', 'single_agv_baseline.slx');
if ~exist(mdl, 'file')
    error('Model not found: %s', mdl);
end

sol_file = fullfile(project_root, 'agv_mss_solution.mat');
if ~exist(sol_file, 'file')
    error('Cannot find %s', sol_file);
end
S = load(sol_file);

% Feed variables used by Simulink blocks.
Ad = S.single.Ad; %#ok<NASGU>
Bd = S.single.Bd; %#ok<NASGU>
K = S.sol.K_delay_state; %#ok<NASGU>
Ts = S.Ts; %#ok<NASGU>
x0 = [1.0; -0.8; 0; 0]; %#ok<NASGU>

p_list = [0.05 0.20 0.35];
stop_time = 400;

results = struct([]);

for i = 1:numel(p_list)
    p = p_list(i); %#ok<NASGU>
    out = sim(mdl, 'StopTime', num2str(stop_time));

    x_obj = out.get('x_sim');
    ua_obj = out.get('ua_sim');
    g_obj = out.get('gamma_sim');

    [t, xv] = local_extract_ts(x_obj);
    [~, uav] = local_extract_ts(ua_obj);
    [~, gv] = local_extract_ts(g_obj);

    if size(xv, 2) < 2
        error('x_sim dimension error: expected at least 2 columns, got %dx%d', size(xv,1), size(xv,2));
    end
    if size(uav, 2) < 2
        error('ua_sim dimension error: expected at least 2 columns, got %dx%d', size(uav,1), size(uav,2));
    end

    pos_err = sqrt(xv(:,1).^2 + xv(:,2).^2);

    results(i).p = p;
    results(i).t = t;
    results(i).x = xv;
    results(i).ua = uav;
    results(i).gamma = gv;
    results(i).loss_rate_emp = 1 - mean(gv(:));
    results(i).final_pos_err = pos_err(end);
    results(i).decay_ratio = pos_err(end) / max(pos_err(1), 1e-12);
    results(i).control_rms = sqrt(mean(sum(uav.^2, 2)));
end

% Print summary
fprintf('\n=== Simulink single-AGV scan summary ===\n');
fprintf('   p     loss_emp   final_pos_err   decay_ratio   control_rms\n');
for i = 1:numel(results)
    fprintf(' %.2f    %.4f      %.4f         %.4f        %.4f\n', ...
        results(i).p, results(i).loss_rate_emp, results(i).final_pos_err, ...
        results(i).decay_ratio, results(i).control_rms);
end

% Save table to CSV
data_dir = fullfile(project_root, 'data');
if ~exist(data_dir, 'dir')
    mkdir(data_dir);
end
T = table( ...
    arrayfun(@(r) r.p, results)', ...
    arrayfun(@(r) r.loss_rate_emp, results)', ...
    arrayfun(@(r) r.final_pos_err, results)', ...
    arrayfun(@(r) r.decay_ratio, results)', ...
    arrayfun(@(r) r.control_rms, results)', ...
    'VariableNames', {'p','loss_rate_emp','final_pos_err','decay_ratio','control_rms'} ...
);
csv_file = fullfile(data_dir, 'simulink_single_scan_summary.csv');
writetable(T, csv_file);
fprintf('Saved summary CSV to: %s\n', csv_file);

% Save figures
pic_dir = fullfile(project_root, 'pic', 'simulink_pic');
if ~exist(pic_dir, 'dir')
    mkdir(pic_dir);
end

figure('Name','Simulink scan: position error');
hold on; grid on;
for i = 1:numel(results)
    e = sqrt(results(i).x(:,1).^2 + results(i).x(:,2).^2);
    plot(results(i).t, e, 'LineWidth', 1.4, ...
        'DisplayName', sprintf('p=%.2f', results(i).p));
end
xlabel('t (s)'); ylabel('||e_{pos}||');
legend('Location','northeast');
title('Single-AGV position error under different packet loss');
ax = gca;
if isprop(ax, 'Toolbar') && ~isempty(ax.Toolbar)
    ax.Toolbar.Visible = 'off';
end
exportgraphics(gcf, fullfile(pic_dir, 'simulink_single_scan_pos_err.png'), 'Resolution', 220);

figure('Name','Simulink gamma sample');
stairs(results(2).t, results(2).gamma, 'LineWidth', 1.2); grid on;
xlabel('t (s)'); ylabel('\gamma(k)');
title(sprintf('Packet success indicator (p=%.2f)', results(2).p));
ylim([-0.1, 1.1]);
ax = gca;
if isprop(ax, 'Toolbar') && ~isempty(ax.Toolbar)
    ax.Toolbar.Visible = 'off';
end
exportgraphics(gcf, fullfile(pic_dir, 'simulink_single_scan_gamma_sample.png'), 'Resolution', 220);

mat_file = fullfile(project_root, 'simulink_single_scan_results.mat');
save(mat_file, 'results', 'T', 'p_list', 'stop_time');
fprintf('Saved MAT results to: %s\n', mat_file);
fprintf('Saved figures to: %s\n', pic_dir);

function [t, v] = local_extract_ts(obj)
    % timeseries
    if isa(obj, 'timeseries')
        t = obj.Time;
        v = obj.Data;
        return;
    end

    % Structure With Time
    if isstruct(obj) && isfield(obj, 'time') && isfield(obj, 'signals')
        t = obj.time;
        if isfield(obj.signals, 'values')
            v = obj.signals.values;
        else
            error('Structure signal has no "values" field.');
        end
        return;
    end

    % SimulationData.Signal style
    if isobject(obj) && isprop(obj, 'Values')
        vals = obj.Values;
        if isa(vals, 'timeseries')
            t = vals.Time;
            v = vals.Data;
            return;
        end
    end

    error('Unsupported signal type for extraction: %s', class(obj));
end

