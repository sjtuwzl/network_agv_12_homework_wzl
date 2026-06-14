%% run_simulink_pd_scan.m
% Simulink single-AGV scan over packet loss p and delay d (1/2/3).
% NOTE:
%   Model must expose two workspace variables:
%   1) p      : used by Compare block threshold
%   2) d_sel  : used by delay selector (1->u(k-1), 2->u(k-2), 3->u(k-3))

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
d_list = [1 2 3];
stop_time = 400;

rows = numel(p_list) * numel(d_list);
T = table('Size', [rows 7], ...
    'VariableTypes', {'double','double','double','double','double','double','double'}, ...
    'VariableNames', {'p','d','loss_rate_emp','final_pos_err','decay_ratio','control_rms','settle_time_50pct'});

records = struct([]);
r = 0;
for ip = 1:numel(p_list)
    for id = 1:numel(d_list)
        r = r + 1;
        p = p_list(ip); %#ok<NASGU>
        d_sel = d_list(id); %#ok<NASGU>

        out = sim(mdl, 'StopTime', num2str(stop_time));

        x_obj = out.get('x_sim');
        ua_obj = out.get('ua_sim');
        g_obj = out.get('gamma_sim');
        [t, xv] = local_extract_ts(x_obj);
        [~, uav] = local_extract_ts(ua_obj);
        [~, gv] = local_extract_ts(g_obj);

        pos_err = sqrt(xv(:,1).^2 + xv(:,2).^2);
        thr_rel = 0.5 * pos_err(1);
        idx50 = find(pos_err <= thr_rel, 1, 'first');
        if isempty(idx50)
            settle50 = inf;
        else
            settle50 = t(idx50);
        end

        loss_emp = 1 - mean(gv(:));
        final_pos = pos_err(end);
        decay = final_pos / max(pos_err(1), 1e-12);
        crms = sqrt(mean(sum(uav.^2, 2)));

        T{r, :} = [p, d_sel, loss_emp, final_pos, decay, crms, settle50];

        records(r).p = p; %#ok<SAGROW>
        records(r).d = d_sel;
        records(r).t = t;
        records(r).x = xv;
        records(r).ua = uav;
        records(r).gamma = gv;
    end
end

disp(T);

% Save CSV/MAT
data_dir = fullfile(project_root, 'data');
if ~exist(data_dir, 'dir')
    mkdir(data_dir);
end
csv_file = fullfile(data_dir, 'simulink_pd_scan_summary.csv');
writetable(T, csv_file);
mat_file = fullfile(project_root, 'simulink_pd_scan_results.mat');
save(mat_file, 'T', 'records', 'p_list', 'd_list', 'stop_time');

% Heatmap (decay ratio)
pic_dir = fullfile(project_root, 'pic', 'simulink_pic');
if ~exist(pic_dir, 'dir')
    mkdir(pic_dir);
end

grid_decay = nan(numel(p_list), numel(d_list));
for ip = 1:numel(p_list)
    for id = 1:numel(d_list)
        idx = (T.p == p_list(ip)) & (T.d == d_list(id));
        grid_decay(ip, id) = T.decay_ratio(idx);
    end
end

figure('Name', 'Simulink p-d decay heatmap');
imagesc(d_list, p_list, grid_decay);
set(gca, 'YDir', 'normal');
xlabel('delay d (samples)');
ylabel('packet loss p');
title('Simulink single-AGV decay ratio over (p,d)');
colorbar;
ax = gca;
if isprop(ax, 'Toolbar') && ~isempty(ax.Toolbar)
    ax.Toolbar.Visible = 'off';
end
exportgraphics(gcf, fullfile(pic_dir, 'simulink_pd_scan_heatmap_decay.png'), 'Resolution', 220);

fprintf('Saved CSV: %s\n', csv_file);
fprintf('Saved MAT: %s\n', mat_file);
fprintf('Saved figure: %s\n', fullfile(pic_dir, 'simulink_pd_scan_heatmap_decay.png'));

function [t, v] = local_extract_ts(obj)
    if isa(obj, 'timeseries')
        t = obj.Time;
        v = obj.Data;
        return;
    end
    if isstruct(obj) && isfield(obj, 'time') && isfield(obj, 'signals')
        t = obj.time;
        if isfield(obj.signals, 'values')
            v = obj.signals.values;
        else
            error('Structure signal has no "values" field.');
        end
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
    error('Unsupported signal type for extraction: %s', class(obj));
end

