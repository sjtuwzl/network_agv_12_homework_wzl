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

mdl = fullfile(project_root, 'slimulink', 'x12_agv.slx');
if ~exist(mdl, 'file')
    error('Model not found: %s', mdl);
end

% Keep stop time consistent with your latest requirement
stop_time = 400;

% Run simulation
open_system(mdl);
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

figure('Name','Simulink 12-AGV mean position error');
plot(t, mean_err, 'LineWidth', 1.6);
grid on; xlabel('t (s)'); ylabel('mean ||e_{pos}||');
title('Simulink 12-AGV mean position error');
exportgraphics(gcf, fullfile(pic_dir, 'simulink_12agv_mean_pos_err.png'), 'Resolution', 220);

figure('Name','Simulink 12-AGV per-AGV final error');
bar(1:12, pos_err(:,end));
grid on; xlabel('AGV index'); ylabel('final ||e_{pos,i}||');
title('Final position error per AGV');
exportgraphics(gcf, fullfile(pic_dir, 'simulink_12agv_final_err_bar.png'), 'Resolution', 220);

T = table((1:12).', loss_rate_per_agv.', pos_err(:,end), ...
    'VariableNames', {'agv_id','loss_rate_emp','final_pos_err'});
csv_file = fullfile(data_dir, 'simulink_12agv_check_summary.csv');
writetable(T, csv_file);

save(fullfile(project_root, 'simulink_12agv_check_results.mat'), ...
    't','xv','uv','gv','pos_err','mean_err','loss_rate_per_agv','loss_rate_global','stop_time');

fprintf('Saved figure: %s\n', fullfile(pic_dir, 'simulink_12agv_mean_pos_err.png'));
fprintf('Saved figure: %s\n', fullfile(pic_dir, 'simulink_12agv_final_err_bar.png'));
fprintf('Saved CSV   : %s\n', csv_file);

function [t, v] = local_extract_ts(obj)
    if isa(obj, 'timeseries')
        t = obj.Time;
        v = obj.Data;
        return;
    end
    if isstruct(obj) && isfield(obj, 'time') && isfield(obj, 'signals')
        t = obj.time;
        v = obj.signals.values;
        return;
    end
    if isobject(obj) && isprop(obj, 'Values')
        vals = obj.Values;
        if isa(vals, 'timeseries')
            t = vals.Time;
            v = vals.Data;
            return;
        end
    end
    error('Unsupported signal type: %s', class(obj));
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

