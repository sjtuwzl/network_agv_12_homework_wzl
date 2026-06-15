%% run_step7_export_network_scan.m
% Step 7:
% 1) Load single-AGV scan results from step6
% 2) Export table to CSV
% 3) Plot heatmaps for key metrics over (p, d)

clear; clc;
this_file = mfilename('fullpath');
this_dir = fileparts(this_file);
project_root = fileparts(this_dir);

in_file = fullfile(project_root, 'step6_single_scan_results.mat');
if ~exist(in_file, 'file')
    error('Cannot find step6_single_scan_results.mat. Run scripts/run_step6_single_scan.m first.');
end

pic_dir = fullfile(project_root, 'pic');
data_dir = fullfile(project_root, 'data');
if ~exist(pic_dir, 'dir')
    mkdir(pic_dir);
end
if ~exist(data_dir, 'dir')
    mkdir(data_dir);
end

S = load(in_file);
if ~isfield(S, 'result_table')
    error('result_table not found in step6_single_scan_results.mat.');
end

T = S.result_table;

% Export full scan table
csv_file = fullfile(data_dir, 'single_scan_metrics.csv');
writetable(T, csv_file);
fprintf('Saved metrics table to: %s\n', csv_file);

% Build metric grids
p_vals = unique(T.p)';
d_vals = unique(T.d)';
Np = numel(p_vals);
Nd = numel(d_vals);

grid_final = nan(Np, Nd);
grid_decay = nan(Np, Nd);
grid_settle50 = nan(Np, Nd);

for i = 1:Np
    for j = 1:Nd
        idx = (T.p == p_vals(i)) & (T.d == d_vals(j));
        if any(idx)
            row = T(idx, :);
            grid_final(i, j) = row.final_pos_err(1);
            grid_decay(i, j) = row.decay_ratio(1);
            grid_settle50(i, j) = row.settle_time_50pct(1);
        end
    end
end

% Heatmap 1: final position error
figure('Name', 'Heatmap final_pos_err');
imagesc(d_vals, p_vals, grid_final);
set(gca, 'YDir', 'normal');
xlabel('delay d (samples)');
ylabel('packet loss p');
title('Final position error over (p,d)');
colorbar;
exportgraphics(gcf, fullfile(pic_dir, 'heatmap_final_pos_err.png'), 'Resolution', 200);

% Heatmap 2: decay ratio
figure('Name', 'Heatmap decay_ratio');
imagesc(d_vals, p_vals, grid_decay);
set(gca, 'YDir', 'normal');
xlabel('delay d (samples)');
ylabel('packet loss p');
title('Decay ratio over (p,d)');
colorbar;
exportgraphics(gcf, fullfile(pic_dir, 'heatmap_decay_ratio.png'), 'Resolution', 200);

% Heatmap 3: 50% settling time
figure('Name', 'Heatmap settle_time_50pct');
imagesc(d_vals, p_vals, grid_settle50);
set(gca, 'YDir', 'normal');
xlabel('delay d (samples)');
ylabel('packet loss p');
title('50% settling time over (p,d)');
colorbar;
exportgraphics(gcf, fullfile(pic_dir, 'heatmap_settle_time_50pct.png'), 'Resolution', 200);

% Print quick textual summary
[best_decay, best_idx] = min(T.decay_ratio);
[worst_decay, worst_idx] = max(T.decay_ratio);

fprintf('\nBest decay_ratio case: p=%.2f, d=%d, decay_ratio=%.4f\n', ...
    T.p(best_idx), T.d(best_idx), best_decay);
fprintf('Worst decay_ratio case: p=%.2f, d=%d, decay_ratio=%.4f\n', ...
    T.p(worst_idx), T.d(worst_idx), worst_decay);

fprintf('\nSaved heatmaps to: %s\n', pic_dir);
