%% run_simulink_12agv_collision_diagnose.m
% Diagnose min_pair_dist == 0 causes in slimulink/x12_agv.slx
% - run one simulation
% - locate worst pair and timestamp
% - detect nearly identical trajectories (possible x_all wiring issue)
% - export diagnostic figure/csv

clear; clc;

this_file = mfilename('fullpath');
this_dir = fileparts(this_file);
project_root = fileparts(this_dir);

run(fullfile(project_root, 'scripts', 'init_simulink_12agv_workspace.m'));

% ---- tunable settings ----
params = struct( ...
    'p', 0.20, ...
    'd_sel', 2, ...
    'kc_pos', 0.08, ...
    'kc_vel', 0.10, ...
    'enable_coop', 1 ...
);
stop_time = 400;
d_safe_min = 0.20;      % hard acceptance threshold
identical_tol = 1e-6;   % detect almost identical trajectories

fn = fieldnames(params);
for i = 1:numel(fn)
    assignin('base', fn{i}, params.(fn{i}));
end
fprintf('Applied params: p=%.2f, d=%d, kc_pos=%.3f, kc_vel=%.3f, enable_coop=%d\n', ...
    params.p, params.d_sel, params.kc_pos, params.kc_vel, params.enable_coop);

mdl = fullfile(project_root, 'slimulink', 'x12_agv.slx');
if ~exist(mdl, 'file')
    error('Model not found: %s', mdl);
end

open_system(mdl);
local_apply_model_initial_conditions('x12_agv', 12);
out = sim('x12_agv', 'StopTime', num2str(stop_time));
[x_obj, ~, ~] = local_pick_outputs(out);
[t, xv] = local_extract_ts(x_obj);

if size(xv,2) ~= 48
    error('x_all width mismatch. Expected 48, got %d', size(xv,2));
end

steps = size(xv, 1);
N = 12;
pair_rows = N*(N-1)/2;
Pair = table('Size', [pair_rows 7], ...
    'VariableTypes', {'double','double','double','double','double','double','logical'}, ...
    'VariableNames', {'agv_i','agv_j','d_min','t_at_d_min','idx_at_d_min','d_final','identical_traj'});

min_pair_dist_t = inf(steps,1);

r = 0;
global_dmin = inf;
global_i = NaN; global_j = NaN; global_idx = NaN;

for i = 1:N-1
    pi_xy = xv(:, (i-1)*4 + (1:2));
    for j = i+1:N
        r = r + 1;
        pj_xy = xv(:, (j-1)*4 + (1:2));

        dvec = sqrt(sum((pi_xy - pj_xy).^2, 2));
        [dmin_ij, idx_ij] = min(dvec);
        d_final = dvec(end);

        max_abs_diff = max(abs(pi_xy(:) - pj_xy(:)));
        identical_traj = max_abs_diff < identical_tol;

        Pair{r, :} = [i, j, dmin_ij, t(idx_ij), idx_ij, d_final, identical_traj];

        min_pair_dist_t = min(min_pair_dist_t, dvec);

        if dmin_ij < global_dmin
            global_dmin = dmin_ij;
            global_i = i;
            global_j = j;
            global_idx = idx_ij;
        end
    end
end

% sort by minimum distance ascending
Pair = sortrows(Pair, 'd_min', 'ascend');

% summary
fprintf('\n=== Collision Diagnose Summary ===\n');
fprintf('Global min pair distance = %.6f m\n', global_dmin);
fprintf('Worst pair              = AGV%d-AGV%d\n', global_i, global_j);
fprintf('At time                 = %.3f s (idx=%d)\n', t(global_idx), global_idx);
fprintf('Threshold d_safe_min    = %.3f m\n', d_safe_min);
if global_dmin >= d_safe_min
    fprintf('Safety check            = PASS\n');
else
    fprintf('Safety check            = FAIL\n');
end

dup_rows = Pair(Pair.identical_traj, :);
if isempty(dup_rows)
    fprintf('Identical trajectory pairs: none\n');
else
    fprintf('Identical trajectory pairs (possible x_all repeated wiring):\n');
    disp(dup_rows(:, {'agv_i','agv_j','d_min','d_final'}));
end

fprintf('\nTop 10 closest pairs:\n');
disp(Pair(1:min(10,height(Pair)), {'agv_i','agv_j','d_min','t_at_d_min','d_final','identical_traj'}));

% outputs
pic_dir = fullfile(project_root, 'pic', 'simulink_pic');
data_dir = fullfile(project_root, 'data');
if ~exist(pic_dir, 'dir'), mkdir(pic_dir); end
if ~exist(data_dir, 'dir'), mkdir(data_dir); end

figure('Name','12AGV min pair distance diagnose');
plot(t, min_pair_dist_t, 'LineWidth', 1.4, 'DisplayName', 'min pair distance');
hold on;
yline(d_safe_min, '--r', 'LineWidth', 1.2, 'DisplayName', sprintf('threshold=%.2f', d_safe_min));
grid on;
xlabel('t (s)');
ylabel('distance (m)');
title('12-AGV minimum pairwise distance over time');
legend('Location','best');
fig_file = fullfile(pic_dir, 'simulink_12agv_min_pair_dist_diagnose_加入防碰撞.png');
exportgraphics(gcf, fig_file, 'Resolution', 220);

pair_csv = fullfile(data_dir, 'simulink_12agv_pair_distance_diagnose.csv');
writetable(Pair, pair_csv);

Summary = table(global_dmin, global_i, global_j, t(global_idx), d_safe_min, global_dmin >= d_safe_min, ...
    'VariableNames', {'global_dmin','worst_agv_i','worst_agv_j','time_at_global_dmin','threshold','safety_pass'});
sum_csv = fullfile(data_dir, 'simulink_12agv_collision_summary.csv');
writetable(Summary, sum_csv);

fprintf('Saved figure: %s\n', fig_file);
fprintf('Saved pair CSV: %s\n', pair_csv);
fprintf('Saved summary CSV: %s\n', sum_csv);

function [x_obj, u_obj, g_obj] = local_pick_outputs(out_obj)
    names = out_obj.who;
    x_name = local_pick_name(names, {'x_all','x_sim','x1_sim'});
    u_name = local_pick_name(names, {'ua_all','ua_sim','ua1_sim'});
    g_name = local_pick_name(names, {'gamma_all','gamma_sim','gamma1_sim'});

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

function name = local_pick_name(names, candidates)
    name = '';
    for i = 1:numel(candidates)
        idx = strcmp(names, candidates{i});
        if any(idx)
            name = candidates{i};
            return;
        end
    end
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

    if ndims(v) >= 3
        if sz(end) == n_t
            v = reshape(v, [], n_t).';
            return;
        elseif sz(1) == n_t
            v = reshape(v, n_t, []);
            return;
        end
    end

    if size(v,1) ~= n_t && size(v,2) == n_t
        v = v.';
    end
end

function local_apply_model_initial_conditions(mdl_name, N)
    for i = 1:N
        blk = sprintf('%s/agv_%d/plant', mdl_name, i);
        try
            set_param(blk, 'X0', sprintf('x0_%d', i));
        catch
            warning('Cannot set initial condition for block: %s', blk);
        end
    end
end
