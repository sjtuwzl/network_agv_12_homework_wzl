%% run_simulink_12agv_check.m
% Quick validation for slimulink/x12_agv.slx
% - initialize workspace
% - simulate for 400s
% - check output dimensions
% - compute mean position error and empirical loss rate
% - export key figures/csv

clear; clc;

this_file = mfilename('fullpath');
this_dir = fileparts(this_file);
project_root = fileparts(this_dir);

run(fullfile(project_root, 'scripts', 'init_simulink_12agv_workspace.m'));

% Tunable parameters for quick experiments (edit here).
% These names should match Constant block values in x12_agv.slx.
params = struct( ...
    'p', 0.20, ...
    'd_sel', 2, ...
    'kc_pos', 0.08, ...
    'kc_vel', 0.10, ...
    'enable_coop', 1 ...
);
run_all_experiments = true;   % true: also run coop on/off compare + p-d scan

% Push overrides to base workspace so Simulink Constant blocks can resolve them.
param_names = fieldnames(params);
for ii = 1:numel(param_names)
    name_i = param_names{ii};
    assignin('base', name_i, params.(name_i));
end
fprintf('Applied params: p=%.2f, d_sel=%d, kc_pos=%.3f, kc_vel=%.3f, enable_coop=%d\n', ...
    params.p, params.d_sel, params.kc_pos, params.kc_vel, params.enable_coop);

mdl = fullfile(project_root, 'slimulink', 'x12_agv.slx');
if ~exist(mdl, 'file')
    error('Model not found: %s', mdl);
end

% Keep stop time consistent with your latest requirement
stop_time = 400;

% Run simulation
open_system(mdl);
local_apply_model_initial_conditions('x12_agv', 12);
out = sim('x12_agv', 'StopTime', num2str(stop_time)); %#ok<NASGU>

% Fetch outputs from base workspace (To Workspace blocks)
base_vars = evalin('base', 'who');
x_name = local_pick_var(base_vars, {'x_all','x_sim','x1_sim'});
u_name = local_pick_var(base_vars, {'ua_all','ua_sim','ua1_sim'});
g_name = local_pick_var(base_vars, {'gamma_all','gamma_sim','gamma1_sim'});
source = "base";

if isempty(x_name) || isempty(u_name) || isempty(g_name)
    % Fallback: auto-detect default names like outsimout3/4/5 by signal width.
    [x_name2, u_name2, g_name2] = local_autodetect_outsimout(base_vars);
    if ~isempty(x_name2), x_name = x_name2; end
    if ~isempty(u_name2), u_name = u_name2; end
    if ~isempty(g_name2), g_name = g_name2; end
end

if (isempty(x_name) || isempty(u_name) || isempty(g_name)) && ismember('out', base_vars)
    % Fallback: outputs may be inside SimulationOutput "out"
    out_obj = evalin('base', 'out');
    if isa(out_obj, 'Simulink.SimulationOutput')
        sim_vars = out_obj.who;
        x_name = local_pick_var(sim_vars, {'x_all','x_sim','x1_sim'});
        u_name = local_pick_var(sim_vars, {'ua_all','ua_sim','ua1_sim'});
        g_name = local_pick_var(sim_vars, {'gamma_all','gamma_sim','gamma1_sim'});
        if isempty(x_name) || isempty(u_name) || isempty(g_name)
            [x_name2, u_name2, g_name2] = local_autodetect_outsimout_in_simout(out_obj);
            if ~isempty(x_name2), x_name = x_name2; end
            if ~isempty(u_name2), u_name = u_name2; end
            if ~isempty(g_name2), g_name = g_name2; end
        end
        if ~isempty(x_name) && ~isempty(u_name) && ~isempty(g_name)
            source = "simout";
        end
    end
end

if isempty(x_name) || isempty(u_name) || isempty(g_name)
    fprintf('Current base workspace vars:\n');
    disp(base_vars);
    error(['Missing To Workspace outputs. Please set variable names to one of: ', ...
           'x_all/x_sim/x1_sim, ua_all/ua_sim/ua1_sim, gamma_all/gamma_sim/gamma1_sim ', ...
           'or keep outsimout* with widths 48/24/12.']);
end

if source == "base"
    fprintf('Using base vars: x=%s, ua=%s, gamma=%s\n', x_name, u_name, g_name);
    x_obj = evalin('base', x_name);
    u_obj = evalin('base', u_name);
    g_obj = evalin('base', g_name);
else
    fprintf('Using SimulationOutput(out) vars: x=%s, ua=%s, gamma=%s\n', x_name, u_name, g_name);
    out_obj = evalin('base', 'out');
    x_obj = out_obj.get(x_name);
    u_obj = out_obj.get(u_name);
    g_obj = out_obj.get(g_name);
end

[t, xv] = local_extract_ts(x_obj);
[~, uv] = local_extract_ts(u_obj);
[~, gv] = local_extract_ts(g_obj);

% Dimension checks
if size(xv,2) ~= 48
    warning('x_all expected 48 columns, got %d', size(xv,2));
end
if size(uv,2) ~= 24
    warning('ua_all expected 24 columns, got %d', size(uv,2));
end
if size(gv,2) ~= 12
    warning('gamma_all expected 12 columns, got %d', size(gv,2));
end

% Build reference stack [ref_1; ... ; ref_12]
ref_stack = zeros(48,1);
for i = 1:12
    ri = evalin('base', sprintf('ref_%d', i));
    ref_stack((i-1)*4 + (1:4)) = ri(:);
end

% Mean position error across 12 AGVs
steps = size(xv,1);
pos_err = zeros(12, steps);
for i = 1:12
    cols = (i-1)*4 + (1:4);
    ei = xv(:, cols) - ref_stack(cols).';
    pos_err(i,:) = sqrt(ei(:,1).^2 + ei(:,2).^2).';
end
mean_err = mean(pos_err, 1);

% Empirical loss rate per AGV and global
loss_rate_per_agv = 1 - mean(gv, 1);
loss_rate_global = mean(loss_rate_per_agv);

fprintf('\n=== Simulink 12-AGV check summary ===\n');
fprintf('x_all size      : %d x %d\n', size(xv,1), size(xv,2));
fprintf('ua_all size     : %d x %d\n', size(uv,1), size(uv,2));
fprintf('gamma_all size  : %d x %d\n', size(gv,1), size(gv,2));
fprintf('Initial mean pos err : %.4f\n', mean_err(1));
fprintf('Final mean pos err   : %.4f\n', mean_err(end));
fprintf('Decay ratio          : %.4f\n', mean_err(end)/max(mean_err(1), 1e-12));
fprintf('Empirical loss (avg) : %.4f\n', loss_rate_global);

% Save artifacts
pic_dir = fullfile(project_root, 'pic', 'simulink_pic');
data_dir = fullfile(project_root, 'data');
if ~exist(pic_dir, 'dir'), mkdir(pic_dir); end
if ~exist(data_dir, 'dir'), mkdir(data_dir); end
fig_note = '_加入防碰撞';

figure('Name','Simulink 12-AGV mean position error');
plot(t, mean_err, 'LineWidth', 1.6);
grid on; xlabel('t (s)'); ylabel('mean ||e_{pos}||');
title('Simulink 12-AGV mean position error');
fig_mean = fullfile(pic_dir, ['simulink_12agv_mean_pos_err' fig_note '.png']);
exportgraphics(gcf, fig_mean, 'Resolution', 220);

figure('Name','Simulink 12-AGV per-AGV final error');
bar(1:12, pos_err(:,end));
grid on; xlabel('AGV index'); ylabel('final ||e_{pos,i}||');
title('Final position error per AGV');
fig_bar = fullfile(pic_dir, ['simulink_12agv_final_err_bar' fig_note '.png']);
exportgraphics(gcf, fig_bar, 'Resolution', 220);

T = table((1:12).', loss_rate_per_agv.', pos_err(:,end), ...
    'VariableNames', {'agv_id','loss_rate_emp','final_pos_err'});
csv_file = fullfile(data_dir, 'simulink_12agv_check_summary.csv');
writetable(T, csv_file);

save(fullfile(project_root, 'simulink_12agv_check_results.mat'), ...
    't','xv','uv','gv','pos_err','mean_err','loss_rate_per_agv','loss_rate_global','stop_time');

fprintf('Saved figure: %s\n', fig_mean);
fprintf('Saved figure: %s\n', fig_bar);
fprintf('Saved CSV   : %s\n', csv_file);

if run_all_experiments
    fprintf('\n=== Extended experiments ===\n');
    fprintf('1) Running matched coop off/on comparison ...\n');

    rec_off = local_run_one_case(mdl, stop_time, params.p, params.d_sel, params.kc_pos, params.kc_vel, 0, ref_stack);
    rec_on  = local_run_one_case(mdl, stop_time, params.p, params.d_sel, params.kc_pos, params.kc_vel, 1, ref_stack);

    figure('Name','Simulink 12-AGV matched coop compare');
    plot(rec_off.t, rec_off.mean_err, 'LineWidth', 1.6, 'DisplayName', 'enable\_coop=0');
    hold on;
    plot(rec_on.t,  rec_on.mean_err,  'LineWidth', 1.6, 'DisplayName', 'enable\_coop=1');
    grid on; xlabel('t (s)'); ylabel('mean ||e_{pos}||');
    legend('Location', 'northeast');
    title(sprintf('12-AGV matched coop compare (p=%.2f, d=%d)', params.p, params.d_sel));
    fig_cmp = fullfile(pic_dir, ['simulink_12agv_matched_coop_compare' fig_note '.png']);
    exportgraphics(gcf, fig_cmp, 'Resolution', 220);

    Tcmp = table( ...
        [0;1], ...
        [rec_off.final_mean_err; rec_on.final_mean_err], ...
        [rec_off.decay_ratio; rec_on.decay_ratio], ...
        [rec_off.loss_rate_global; rec_on.loss_rate_global], ...
        [rec_off.min_pair_dist; rec_on.min_pair_dist], ...
        'VariableNames', {'enable_coop','final_mean_err','decay_ratio','loss_rate_emp','min_pair_dist'});
    cmp_csv = fullfile(data_dir, 'simulink_12agv_matched_coop_compare.csv');
    writetable(Tcmp, cmp_csv);
    fprintf('Saved compare figure: %s\n', fig_cmp);
    fprintf('Saved compare CSV   : %s\n', cmp_csv);

    fprintf('2) Running full p-d scan script ...\n');
    run(fullfile(project_root, 'scripts', 'run_simulink_12agv_pd_scan.m'));
    fprintf('Extended experiments done.\n');
end

function [t, v] = local_extract_ts(obj)
    if isa(obj, 'timeseries')
        t = obj.Time;
        v = obj.Data;
        v = local_normalize_ts_data(v, numel(t));
        return;
    end
    if isstruct(obj) && isfield(obj, 'time') && isfield(obj, 'signals')
        t = obj.time;
        v = obj.signals.values;
        v = local_normalize_ts_data(v, numel(t));
        return;
    end
    if isobject(obj) && isprop(obj, 'Values')
        vals = obj.Values;
        if isa(vals, 'timeseries')
            t = vals.Time;
            v = vals.Data;
            v = local_normalize_ts_data(v, numel(t));
            return;
        end
    end
    error('Unsupported signal type: %s', class(obj));
end

function v = local_normalize_ts_data(v, n_t)
    % Normalize signal data to [N x dim], where N = number of timestamps.
    if isa(v, 'embedded.fi')
        v = double(v);
    end

    if isempty(v)
        return;
    end

    sz = size(v);
    if isvector(v)
        v = v(:);
        return;
    end

    % Common timeseries case from vector signals: [dim x 1 x N]
    if ndims(v) >= 3
        if sz(end) == n_t
            v = reshape(v, [], n_t).';
            return;
        elseif sz(1) == n_t
            v = reshape(v, n_t, []);
            return;
        end
    end

    % 2-D fallback: transpose if time axis is the second dimension.
    if size(v,1) ~= n_t && size(v,2) == n_t
        v = v.';
    end
end

function name = local_pick_var(base_vars, candidates)
    name = '';
    for i = 1:numel(candidates)
        if ismember(candidates{i}, base_vars)
            name = candidates{i};
            return;
        end
    end
end

function [x_name, u_name, g_name] = local_autodetect_outsimout(base_vars)
    x_name = ''; u_name = ''; g_name = '';
    mask = startsWith(base_vars, 'outsimout');
    cands = base_vars(mask);
    for i = 1:numel(cands)
        vn = cands{i};
        try
            obj = evalin('base', vn);
            [~, v] = local_extract_ts(obj);
            w = size(v, 2);
            if w == 48 && isempty(x_name)
                x_name = vn;
            elseif w == 24 && isempty(u_name)
                u_name = vn;
            elseif w == 12 && isempty(g_name)
                g_name = vn;
            end
        catch
            % ignore unreadable candidate
        end
    end
end

function [x_name, u_name, g_name] = local_autodetect_outsimout_in_simout(out_obj)
    x_name = ''; u_name = ''; g_name = '';
    cands = out_obj.who;
    mask = startsWith(cands, 'outsimout');
    cands = cands(mask);
    for i = 1:numel(cands)
        vn = cands{i};
        try
            obj = out_obj.get(vn);
            [~, v] = local_extract_ts(obj);
            w = size(v, 2);
            if w == 48 && isempty(x_name)
                x_name = vn;
            elseif w == 24 && isempty(u_name)
                u_name = vn;
            elseif w == 12 && isempty(g_name)
                g_name = vn;
            end
        catch
            % ignore
        end
    end
end

function rec = local_run_one_case(mdl, stop_time, p, d_sel, kc_pos, kc_vel, enable_coop, ref_stack)
    assignin('base', 'p', p);
    assignin('base', 'd_sel', int32(d_sel));
    assignin('base', 'kc_pos', kc_pos);
    assignin('base', 'kc_vel', kc_vel);
    assignin('base', 'enable_coop', enable_coop);

    local_apply_model_initial_conditions(mdl, 12);
    out = sim(mdl, 'StopTime', num2str(stop_time));
    [x_obj, u_obj, g_obj] = local_pick_outputs_from_simout(out);

    [t, xv] = local_extract_ts(x_obj);
    [~, uv] = local_extract_ts(u_obj);
    [~, gv] = local_extract_ts(g_obj);

    steps = size(xv, 1);
    pos_err = zeros(12, steps);
    for k = 1:12
        cols = (k-1)*4 + (1:4);
        ei = xv(:, cols) - ref_stack(cols).';
        pos_err(k, :) = sqrt(ei(:,1).^2 + ei(:,2).^2).';
    end
    mean_err = mean(pos_err, 1);

    rec = struct();
    rec.t = t;
    rec.mean_err = mean_err;
    rec.final_mean_err = mean_err(end);
    rec.decay_ratio = mean_err(end) / max(mean_err(1), 1e-12);
    loss_rate_per_agv = 1 - mean(gv, 1);
    rec.loss_rate_global = mean(loss_rate_per_agv);
    rec.min_pair_dist = local_min_pair_distance(xv);
end

function [x_obj, u_obj, g_obj] = local_pick_outputs_from_simout(out_obj)
    names = out_obj.who;
    x_name = local_pick_var(names, {'x_all','x_sim','x1_sim'});
    u_name = local_pick_var(names, {'ua_all','ua_sim','ua1_sim'});
    g_name = local_pick_var(names, {'gamma_all','gamma_sim','gamma1_sim'});

    if isempty(x_name) || isempty(u_name) || isempty(g_name)
        [x2, u2, g2] = local_pick_outsimout_by_width(out_obj);
        if ~isempty(x2), x_name = x2; end
        if ~isempty(u2), u_name = u2; end
        if ~isempty(g2), g_name = g2; end
    end

    if isempty(x_name) || isempty(u_name) || isempty(g_name)
        error('Cannot identify x/ua/gamma outputs from SimulationOutput.');
    end

    x_obj = out_obj.get(x_name);
    u_obj = out_obj.get(u_name);
    g_obj = out_obj.get(g_name);
end

function [x_name, u_name, g_name] = local_pick_outsimout_by_width(out_obj)
    x_name = ''; u_name = ''; g_name = '';
    names = out_obj.who;
    mask = startsWith(names, 'outsimout');
    names = names(mask);
    for i = 1:numel(names)
        n = names{i};
        obj = out_obj.get(n);
        [~, v] = local_extract_ts(obj);
        w = size(v,2);
        if w == 48 && isempty(x_name), x_name = n; end
        if w == 24 && isempty(u_name), u_name = n; end
        if w == 12 && isempty(g_name), g_name = n; end
    end
end

function dmin = local_min_pair_distance(xv)
    steps = size(xv,1);
    dmin = inf;
    for k = 1:steps
        pos = zeros(12,2);
        for i = 1:12
            cols = (i-1)*4 + (1:2);
            pos(i,:) = xv(k, cols);
        end
        for i = 1:11
            for j = i+1:12
                dij = norm(pos(i,:) - pos(j,:));
                if dij < dmin
                    dmin = dij;
                end
            end
        end
    end
end

function local_apply_model_initial_conditions(mdl_name_or_path, N)
    [~, mdl_name, ext] = fileparts(mdl_name_or_path);
    if isempty(ext)
        mdl_name = mdl_name_or_path;
    end
    for i = 1:N
        blk = sprintf('%s/agv_%d/plant', mdl_name, i);
        try
            % Force each AGV plant to use its own initial condition variable.
            set_param(blk, 'X0', sprintf('x0_%d', i));
        catch
            warning('Cannot set initial condition for block: %s', blk);
        end
    end
end

