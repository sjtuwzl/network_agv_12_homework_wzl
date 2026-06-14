%% run_step10_compare_multi12_step6_step9.m
% Step 10:
% Compare independent multi-AGV (Step6) vs cooperative multi-AGV (Step9)

clear; clc;
this_file = mfilename('fullpath');
this_dir = fileparts(this_file);
project_root = fileparts(this_dir);

f6 = fullfile(project_root, 'step6_multi12_demo.mat');
f9 = fullfile(project_root, 'step9_multi12_coop_demo.mat');
if ~exist(f6, 'file')
    error('Missing step6_multi12_demo.mat. Run scripts/run_step6_multi12_demo.m first.');
end
if ~exist(f9, 'file')
    error('Missing step9_multi12_coop_demo.mat. Run scripts/run_step9_multi12_coop_demo.m first.');
end

S6 = load(f6);
S9 = load(f9);

if ~isfield(S6, 'mean_err') || ~isfield(S9, 'mean_err')
    error('mean_err missing in one of the MAT files.');
end

mean6 = S6.mean_err(:)';
mean9 = S9.mean_err(:)';
Ts6 = S6.Ts;
Ts9 = S9.Ts;

if abs(Ts6 - Ts9) > 1e-12
    warning('Ts differs between Step6 and Step9. Comparison uses own time axes.');
end

t6 = (0:numel(mean6)-1) * Ts6;
t9 = (0:numel(mean9)-1) * Ts9;

pic_dir = fullfile(project_root, 'pic');
data_dir = fullfile(project_root, 'data');
if ~exist(pic_dir, 'dir')
    mkdir(pic_dir);
end
if ~exist(data_dir, 'dir')
    mkdir(data_dir);
end

% Main comparison figure
figure('Name', 'Step6 vs Step9 mean error');
plot(t6, mean6, 'LineWidth', 1.8); hold on;
plot(t9, mean9, 'LineWidth', 1.8);
grid on;
xlabel('t (s)');
ylabel('mean ||e_{pos}||');
legend('Step6 independent', 'Step9 cooperative', 'Location', 'northeast');
title('Mean position error comparison: Step6 vs Step9');
exportgraphics(gcf, fullfile(pic_dir, 'compare_step6_step9_mean_err.png'), 'Resolution', 220);

% Scalar summary
decay6 = mean6(end) / max(mean6(1), 1e-12);
decay9 = mean9(end) / max(mean9(1), 1e-12);
final6 = mean6(end);
final9 = mean9(end);
improve_final = (final6 - final9) / max(final6, 1e-12) * 100;
improve_decay = (decay6 - decay9) / max(decay6, 1e-12) * 100;

if isfield(S9, 'min_pair_dist')
    min_dist9 = min(S9.min_pair_dist);
else
    min_dist9 = NaN;
end

summary = table(final6, final9, decay6, decay9, improve_final, improve_decay, min_dist9, ...
    'VariableNames', {'final_mean_err_step6','final_mean_err_step9','decay_step6','decay_step9', ...
                      'improve_final_err_pct','improve_decay_pct','min_pair_dist_step9'});

csv_file = fullfile(data_dir, 'compare_step6_step9_summary.csv');
writetable(summary, csv_file);

fprintf('Saved compare figure to: %s\n', fullfile(pic_dir, 'compare_step6_step9_mean_err.png'));
fprintf('Saved compare summary to: %s\n\n', csv_file);
disp(summary);
