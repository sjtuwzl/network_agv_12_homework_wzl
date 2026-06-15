%% run_step8_single_scan_wide.m
% Step 8 (single AGV wide scan):
% - Use solved K from agv_mss_solution.mat
% - Sweep wider (p,d) ranges with longer simulation horizon
% - Export MAT + CSV + heatmaps for stronger trend analysis

clear; clc;
this_file = mfilename('fullpath');
this_dir = fileparts(this_file);
project_root = fileparts(this_dir);
addpath(fullfile(project_root, 'src'));

sol_file = fullfile(project_root, 'agv_mss_solution.mat');
if ~exist(sol_file, 'file')
    error('Cannot find agv_mss_solution.mat. Run scripts/run_agv_mss_fixedK_search.m first.');
end
S = load(sol_file);

Ts = S.Ts;
single = S.single;
if isfield(S.sol, 'K_delay_state') && ~isempty(S.sol.K_delay_state)
    K = S.sol.K_delay_state;
else
    error('No usable K_delay_state found in solution file.');
end

% Wider scan setup
p_list = 0.05:0.10:0.65;
d_list = 0:1:8;
steps = 2000;
x0 = [1.0; -0.8; 0; 0];
x_ref = [0; 0; 0; 0];

rows = numel(p_list) * numel(d_list);
result_table = table('Size', [rows 11], ...
    'VariableTypes', {'double','double','double','double','double','double','double','double','double','double','double'}, ...
    'VariableNames', {'p','d','rmse_pos','rmse_vel','max_pos_err','control_rms','settle_time_abs','settle_time_50pct','loss_rate_emp','final_pos_err','decay_ratio'});

r = 0;
for ip = 1:numel(p_list)
    for id = 1:numel(d_list)
        r = r + 1;
        pp = p_list(ip);
        dd = d_list(id);
        sim = simulate_single_agv_network(single.Ad, single.Bd, K, dd, pp, steps, x0, x_ref, 1000 + r);
        met = compute_performance_metrics(sim, Ts);
        result_table{r, :} = [pp, dd, met.rmse_pos, met.rmse_vel, met.max_pos_err, met.control_rms, ...
                              met.settle_time, met.settle_time_50pct, met.loss_rate_empirical, ...
                              met.final_pos_err, met.decay_ratio];
    end
end

% Save outputs
data_dir = fullfile(project_root, 'data');
pic_dir = fullfile(project_root, 'pic');
if ~exist(data_dir, 'dir')
    mkdir(data_dir);
end
if ~exist(pic_dir, 'dir')
    mkdir(pic_dir);
end

mat_file = fullfile(project_root, 'step8_single_scan_wide_results.mat');
save(mat_file, 'result_table', 'p_list', 'd_list', 'steps', 'x0', 'x_ref', 'Ts');

csv_file = fullfile(data_dir, 'single_scan_wide_metrics.csv');
writetable(result_table, csv_file);

% Build grids for heatmaps
p_vals = unique(result_table.p)';
d_vals = unique(result_table.d)';
Np = numel(p_vals);
Nd = numel(d_vals);

grid_final = nan(Np, Nd);
grid_decay = nan(Np, Nd);
grid_settle50 = nan(Np, Nd);

for i = 1:Np
    for j = 1:Nd
        idx = (result_table.p == p_vals(i)) & (result_table.d == d_vals(j));
        if any(idx)
            row = result_table(idx, :);
            grid_final(i, j) = row.final_pos_err(1);
            grid_decay(i, j) = row.decay_ratio(1);
            grid_settle50(i, j) = row.settle_time_50pct(1);
        end
    end
end

figure('Name', 'Wide scan final_pos_err');
imagesc(d_vals, p_vals, grid_final);
set(gca, 'YDir', 'normal');
xlabel('delay d (samples)');
ylabel('packet loss p');
title(sprintf('Wide scan final position error (steps=%d)', steps));
colorbar;
exportgraphics(gcf, fullfile(pic_dir, 'wide_heatmap_final_pos_err.png'), 'Resolution', 220);

figure('Name', 'Wide scan decay_ratio');
imagesc(d_vals, p_vals, grid_decay);
set(gca, 'YDir', 'normal');
xlabel('delay d (samples)');
ylabel('packet loss p');
title(sprintf('Wide scan decay ratio (steps=%d)', steps));
colorbar;
exportgraphics(gcf, fullfile(pic_dir, 'wide_heatmap_decay_ratio.png'), 'Resolution', 220);

figure('Name', 'Wide scan settle_time_50pct');
imagesc(d_vals, p_vals, grid_settle50);
set(gca, 'YDir', 'normal');
xlabel('delay d (samples)');
ylabel('packet loss p');
title(sprintf('Wide scan 50%% settling time (steps=%d)', steps));
colorbar;
exportgraphics(gcf, fullfile(pic_dir, 'wide_heatmap_settle_time_50pct.png'), 'Resolution', 220);

% Quick summary
[best_decay, best_idx] = min(result_table.decay_ratio);
[worst_decay, worst_idx] = max(result_table.decay_ratio);
[best_final, best_final_idx] = min(result_table.final_pos_err);
[worst_final, worst_final_idx] = max(result_table.final_pos_err);

fprintf('Saved MAT: %s\n', mat_file);
fprintf('Saved CSV: %s\n', csv_file);
fprintf('Saved wide heatmaps to: %s\n\n', pic_dir);

fprintf('Best decay_ratio: p=%.2f, d=%d, decay_ratio=%.4f\n', ...
    result_table.p(best_idx), result_table.d(best_idx), best_decay);
fprintf('Worst decay_ratio: p=%.2f, d=%d, decay_ratio=%.4f\n', ...
    result_table.p(worst_idx), result_table.d(worst_idx), worst_decay);
fprintf('Best final_pos_err: p=%.2f, d=%d, final_pos_err=%.4f\n', ...
    result_table.p(best_final_idx), result_table.d(best_final_idx), best_final);
fprintf('Worst final_pos_err: p=%.2f, d=%d, final_pos_err=%.4f\n', ...
    result_table.p(worst_final_idx), result_table.d(worst_final_idx), worst_final);
