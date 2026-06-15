%% run_simulink_12agv_pd_scan.m
% 12-AGV Simulink scan over packet loss p and delay d_sel.
% Outputs:
% - data/simulink_12agv_pd_scan_summary.csv
% - pic/simulink_pic/simulink_12agv_pd_heatmap_decay.png
% - pic/simulink_pic/simulink_12agv_pd_heatmap_final_mean_err.png
% - pic/simulink_pic/simulink_12agv_state_curve_pXX_dY.png
% - pic/simulink_pic/simulink_12agv_control_curve_pXX_dY.png
% - pic/simulink_pic/simulink_12agv_mean_err_curve_pXX_dY.png

clear; clc;

this_file = mfilename('fullpath');
this_dir = fileparts(this_file);
project_root = fileparts(this_dir);

run(fullfile(project_root, 'scripts', 'init_simulink_12agv_workspace.m'));

mdl = fullfile(project_root, 'slimulink', 'x12_agv.slx');
if ~exist(mdl, 'file')
    error('Model not found: %s', mdl);
end
open_system(mdl);
local_apply_model_initial_conditions('x12_agv', 12);

% Scan set
p_list = [0.05, 0.20, 0.35];
d_list = [1, 2, 3];
stop_time = 400;

% Representative case for time-series curves
p_rep = 0.20;
d_rep = 2;

rows = numel(p_list) * numel(d_list);
T = table('Size', [rows 10], ...
    'VariableTypes', {'double','double','double','double','double','double','double','double','double','double'}, ...
    'VariableNames', {'p','d','loss_rate_emp','final_mean_err','decay_ratio','control_rms', ...
                      'settle_time_50pct','min_pair_dist','final_err_agv1','final_err_agv12'});

ref_stack = zeros(48, 1);
for i = 1:12
    ref_stack((i-1)*4 + (1:4)) = evalin('base', sprintf('ref_%d', i));
end

rec_rep = struct();
r = 0;
for ip = 1:numel(p_list)
    for id = 1:numel(d_list)
        r = r + 1;
        p = p_list(ip); %#ok<NASGU>
        d_sel = int32(d_list(id)); %#ok<NASGU>

        local_apply_model_initial_conditions('x12_agv', 12);
        out = sim(mdl, 'StopTime', num2str(stop_time));
        [x_obj, u_obj, g_obj] = local_pick_outputs(out);

        [t, xv] = local_extract_ts(x_obj);
        [~, uv] = local_extract_ts(u_obj);
        [~, gv] = local_extract_ts(g_obj);

        if size(xv,2) ~= 48 || size(uv,2) ~= 24 || size(gv,2) ~= 12
            error('Output width mismatch. Need x_all=48, ua_all=24, gamma_all=12.');
        end

        % Per-AGV position error
        steps = size(xv,1);
        pos_err = zeros(12, steps);
        for k = 1:12
            cols = (k-1)*4 + (1:4);
            ei = xv(:, cols) - ref_stack(cols).';
            pos_err(k, :) = sqrt(ei(:,1).^2 + ei(:,2).^2).';
        end
        mean_err = mean(pos_err, 1);

        loss_emp = 1 - mean(gv(:));
        final_mean_err = mean_err(end);
        decay_ratio = final_mean_err / max(mean_err(1), 1e-12);
        control_rms = sqrt(mean(sum(uv.^2, 2)));

        idx50 = find(mean_err <= 0.5 * mean_err(1), 1, 'first');
        if isempty(idx50)
            settle50 = inf;
        else
            settle50 = t(idx50);
        end

        min_pair_dist = local_min_pair_distance(xv);

        T{r, :} = [p, d_list(id), loss_emp, final_mean_err, decay_ratio, control_rms, ...
                   settle50, min_pair_dist, pos_err(1,end), pos_err(12,end)];

        if abs(p - p_rep) < 1e-12 && d_list(id) == d_rep
            rec_rep.t = t;
            rec_rep.xv = xv;
            rec_rep.uv = uv;
            rec_rep.mean_err = mean_err;
        end
    end
end

disp(T);

% Save table
data_dir = fullfile(project_root, 'data');
pic_dir = fullfile(project_root, 'pic', 'simulink_pic');
if ~exist(data_dir, 'dir'), mkdir(data_dir); end
if ~exist(pic_dir, 'dir'), mkdir(pic_dir); end
fig_note = '_加入防碰撞';

csv_file = fullfile(data_dir, 'simulink_12agv_pd_scan_summary.csv');
writetable(T, csv_file);

save(fullfile(project_root, 'simulink_12agv_pd_scan_results.mat'), ...
    'T', 'p_list', 'd_list', 'stop_time', 'p_rep', 'd_rep', 'rec_rep');

% Heatmap: decay ratio
grid_decay = nan(numel(p_list), numel(d_list));
grid_final = nan(numel(p_list), numel(d_list));
for ip = 1:numel(p_list)
    for id = 1:numel(d_list)
        idx = (T.p == p_list(ip)) & (T.d == d_list(id));
        grid_decay(ip, id) = T.decay_ratio(idx);
        grid_final(ip, id) = T.final_mean_err(idx);
    end
end

figure('Name','12AGV p-d heatmap decay');
imagesc(d_list, p_list, grid_decay);
set(gca, 'YDir', 'normal');
xlabel('delay d (samples)');
ylabel('packet loss p');
title('12-AGV decay ratio over (p,d)');
colorbar;
exportgraphics(gcf, fullfile(pic_dir, ['simulink_12agv_pd_heatmap_decay' fig_note '.png']), 'Resolution', 220);

figure('Name','12AGV p-d heatmap final mean err');
imagesc(d_list, p_list, grid_final);
set(gca, 'YDir', 'normal');
xlabel('delay d (samples)');
ylabel('packet loss p');
title('12-AGV final mean position error over (p,d)');
colorbar;
exportgraphics(gcf, fullfile(pic_dir, ['simulink_12agv_pd_heatmap_final_mean_err' fig_note '.png']), 'Resolution', 220);

% Representative time-series curves (AGV1 state/control)
if ~isempty(fieldnames(rec_rep))
    t = rec_rep.t;
    xv = rec_rep.xv;
    uv = rec_rep.uv;
    mean_err = rec_rep.mean_err;

    tag = sprintf('p%.2f_d%d', p_rep, d_rep);
    tag = strrep(tag, '.', 'p');

    figure('Name','12AGV representative state curves');
    plot(t, xv(:,1), t, xv(:,2), t, xv(:,3), t, xv(:,4), 'LineWidth', 1.1);
    grid on;
    xlabel('t (s)'); ylabel('state value');
    legend('p_x(AGV1)','p_y(AGV1)','v_x(AGV1)','v_y(AGV1)', 'Location', 'northeast');
    title(sprintf('12-AGV state curves (%s)', strrep(tag, '_', ', ')));
    exportgraphics(gcf, fullfile(pic_dir, ['simulink_12agv_state_curve_' tag fig_note '.png']), 'Resolution', 220);

    figure('Name','12AGV representative control curves');
    plot(t, uv(:,1), t, uv(:,2), 'LineWidth', 1.2);
    grid on;
    xlabel('t (s)'); ylabel('u_a');
    legend('u_x(AGV1)','u_y(AGV1)', 'Location', 'northeast');
    title(sprintf('12-AGV control output curves (%s)', strrep(tag, '_', ', ')));
    exportgraphics(gcf, fullfile(pic_dir, ['simulink_12agv_control_curve_' tag fig_note '.png']), 'Resolution', 220);

    figure('Name','12AGV representative mean error curve');
    plot(t, mean_err, 'LineWidth', 1.6);
    grid on;
    xlabel('t (s)'); ylabel('mean ||e_{pos}||');
    title(sprintf('12-AGV mean position error curve (%s)', strrep(tag, '_', ', ')));
    exportgraphics(gcf, fullfile(pic_dir, ['simulink_12agv_mean_err_curve_' tag fig_note '.png']), 'Resolution', 220);
end

fprintf('Saved CSV: %s\n', csv_file);
fprintf('Saved figures under: %s\n', pic_dir);

function [x_obj, u_obj, g_obj] = local_pick_outputs(out_obj)
    % Try SimulationOutput names first
    names = out_obj.who;
    x_name = local_pick_name(names, {'x_all','x_sim','x1_sim'});
    u_name = local_pick_name(names, {'ua_all','ua_sim','ua1_sim'});
    g_name = local_pick_name(names, {'gamma_all','gamma_sim','gamma1_sim'});

    % Fallback for outsimout*
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

function dmin = local_min_pair_distance(xv)
    % xv: [steps x 48], each AGV has [px py vx vy]
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

