%% run_step6_multi12_demo.m
% Step 6 (12 AGV stacked closed-loop demo with independent packet loss).

clear; clc;
this_file = mfilename('fullpath');
this_dir = fileparts(this_file);
project_root = fileparts(this_dir);
addpath(fullfile(project_root, 'src'));
pic_dir = fullfile(project_root, 'pic');
if ~exist(pic_dir, 'dir')
    mkdir(pic_dir);
end

sol_file = fullfile(project_root, 'agv_mss_solution.mat');
if ~exist(sol_file, 'file')
    legacy_file = fullfile(pwd, 'agv_mss_solution.mat');
    if exist(legacy_file, 'file')
        copyfile(legacy_file, sol_file);
        fprintf('Copied legacy solution file to project root: %s\n', sol_file);
    else
        error('Cannot find agv_mss_solution.mat. Run scripts/run_agv_mss_demo.m first.');
    end
end
S = load(sol_file);

Ts = S.Ts;
N = S.N;
single = S.single;
K = S.sol.K_delay_state; % 2x4

steps = 250;
p = S.p;
d = S.d;

n = single.n;
m = single.nu;

% Initial condition for all AGVs: different offsets
x = zeros(n*N, steps+1);
for i = 1:N
    base = (i-1)*n;
    x(base + (1:4), 1) = [0.4*cos(2*pi*i/N); 0.4*sin(2*pi*i/N); 0; 0];
end
x_ref = zeros(n*N, 1);

u_prev = zeros(m*N, 1);
u_act_log = zeros(m*N, steps);
pos_err_norm = zeros(N, steps+1);

% Delay buffers for each AGV, store past states
x_hist = zeros(n*N, steps+1);
x_hist(:,1) = x(:,1);

% Initialize k=0 position error norm
for i = 1:N
    ix = (i-1)*n + (1:n);
    e_i0 = x(ix, 1) - x_ref(ix);
    pos_err_norm(i, 1) = norm(e_i0(1:2));
end

rng(2026);
for k = 1:steps
    u_act = zeros(m*N, 1);
    for i = 1:N
        ix = (i-1)*n + (1:n);
        iu = (i-1)*m + (1:m);

        idx_delay = k - d;
        if idx_delay < 1
            x_del = x_hist(ix, 1);
        else
            x_del = x_hist(ix, idx_delay);
        end

        uc_del = K * (x_del - zeros(n,1));
        g = rand() > p;
        u_act(iu) = g * uc_del + (1-g) * u_prev(iu);
    end

    % Block-diagonal plant update
    for i = 1:N
        ix = (i-1)*n + (1:n);
        iu = (i-1)*m + (1:m);
        x(ix, k+1) = single.Ad * x(ix, k) + single.Bd * u_act(iu);
    end

    x_hist(:, k+1) = x(:, k+1);
    u_prev = u_act;
    u_act_log(:, k) = u_act;

    for i = 1:N
        ix = (i-1)*n + (1:n);
        e_i = x(ix, k+1) - x_ref(ix);
        pos_err_norm(i, k+1) = norm(e_i(1:2));
    end
end

t = (0:steps) * Ts;
figure('Name','12-AGV position error norms');
fig_err_all = gcf;
plot(t, pos_err_norm', 'LineWidth', 1.0);
grid on; xlabel('t (s)'); ylabel('||e_{pos,i}||');
title(sprintf('12-AGV position error norms, p=%.2f, d=%d', p, d));
saveas(fig_err_all, fullfile(pic_dir, sprintf('multi12_pos_err_all_p%.2f_d%d.png', p, d)));

mean_err = mean(pos_err_norm, 1);
figure('Name','12-AGV mean position error');
fig_err_mean = gcf;
plot(t, mean_err, 'LineWidth', 1.6); grid on;
xlabel('t (s)'); ylabel('mean ||e_{pos}||');
title('Mean position error across 12 AGVs');
saveas(fig_err_mean, fullfile(pic_dir, sprintf('multi12_mean_pos_err_p%.2f_d%d.png', p, d)));

out_file = fullfile(project_root, 'step6_multi12_demo.mat');
save(out_file, 'x', 'u_act_log', 'pos_err_norm', 'mean_err', 'Ts', 'p', 'd', 'N');
fprintf('Saved 12-AGV demo results to: %s\n', out_file);

fprintf('Initial mean position error: %.4f\n', mean_err(1));
fprintf('Final mean position error: %.4f\n', mean_err(end));
fprintf('Decay ratio (final/initial): %.4f\n', mean_err(end)/max(mean_err(1), 1e-12));
fprintf('Saved 12-AGV figures to: %s\n', pic_dir);
