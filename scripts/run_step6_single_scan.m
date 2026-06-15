%% run_step6_single_scan.m
% Step 6 (single AGV):
% 1) time-domain simulation under packet loss + delay
% 2) sweep p,d and export metrics

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
    % Backward compatibility: if previous run saved in current folder.
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
single = S.single;
if isfield(S.sol, 'K_delay_state') && ~isempty(S.sol.K_delay_state)
    K = S.sol.K_delay_state;
elseif isfield(S.sol, 'Kbar') && ~isempty(S.sol.Kbar)
    K = S.sol.Kbar * S.aug.C_pick';
else
    error('No usable gain found in S.sol.');
end
if isfield(S, 'rho')
    rho = S.rho;
else
    rho = NaN;
end

% Baseline scenario
p0 = S.p;
d0 = S.d;
steps = 800;
x0 = [1.0; -0.8; 0; 0];
x_ref = [0; 0; 0; 0];

sim0 = simulate_single_agv_network(single.Ad, single.Bd, K, d0, p0, steps, x0, x_ref, 1);
met0 = compute_performance_metrics(sim0, Ts);

fprintf('Baseline metrics (p=%.2f, d=%d, rho=%.3f):\n', p0, d0, rho);
disp(met0);

% Plot baseline curves
t = (0:steps) * Ts;
figure('Name', 'Single AGV states');
fig_states = gcf;
subplot(2,1,1); plot(t, sim0.x(1,:), t, sim0.x(2,:), 'LineWidth', 1.2); grid on;
xlabel('t (s)'); ylabel('position'); legend('p_x', 'p_y');
title(sprintf('Single AGV position, p=%.2f, d=%d', p0, d0));
hold on;
yline(0.02, '--r', '2cm threshold');
yline(-0.02, '--r');
subplot(2,1,2); plot(t, sim0.x(3,:), t, sim0.x(4,:), 'LineWidth', 1.2); grid on;
xlabel('t (s)'); ylabel('velocity'); legend('v_x', 'v_y');
saveas(fig_states, fullfile(pic_dir, sprintf('single_states_p%.2f_d%d.png', p0, d0)));

tu = (0:steps-1) * Ts;
figure('Name', 'Control and gamma');
fig_ctrl = gcf;
subplot(2,1,1); plot(tu, sim0.u_act(1,:), tu, sim0.u_act(2,:), 'LineWidth', 1.2); grid on;
xlabel('t (s)'); ylabel('u_a'); legend('u_x', 'u_y');
title('Applied control');
subplot(2,1,2); stairs(tu, sim0.gamma, 'LineWidth', 1.2); grid on;
xlabel('t (s)'); ylabel('\gamma(k)'); ylim([-0.1, 1.1]);
title('Packet success indicator');
saveas(fig_ctrl, fullfile(pic_dir, sprintf('single_control_gamma_p%.2f_d%d.png', p0, d0)));

% Sweep scenarios
p_list = [0.05, 0.15, 0.25, 0.35];
d_list = [0, 1, 2, 3];

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
        sim = simulate_single_agv_network(single.Ad, single.Bd, K, dd, pp, steps, x0, x_ref, 100 + r);
        met = compute_performance_metrics(sim, Ts);
        result_table{r, :} = [pp, dd, met.rmse_pos, met.rmse_vel, met.max_pos_err, met.control_rms, ...
                              met.settle_time, met.settle_time_50pct, met.loss_rate_empirical, ...
                              met.final_pos_err, met.decay_ratio];
    end
end

disp(result_table);

% Optional baseline without delayed-state feedback (use current state in K)
sim_curr = simulate_single_agv_network(single.Ad, single.Bd, K, d0, p0, steps, x0, x_ref, 2, false);
met_curr = compute_performance_metrics(sim_curr, Ts);
fprintf('\nCurrent-state feedback check (same K, p=%.2f, d=%d):\n', p0, d0);
disp(met_curr);

% Quick terminal error stats for debugging convergence behavior
pos_err_norm = sqrt(sim0.e(1,:).^2 + sim0.e(2,:).^2);
fprintf('Initial pos error norm: %.4f\n', pos_err_norm(1));
fprintf('Final pos error norm: %.4f\n', pos_err_norm(end));

out_file = fullfile(project_root, 'step6_single_scan_results.mat');
save(out_file, 'sim0', 'met0', 'sim_curr', 'met_curr', 'result_table', 'p_list', 'd_list', 'steps', 'x0', 'x_ref');
fprintf('Saved single-AGV sweep results to: %s\n', out_file);
fprintf('Saved baseline figures to: %s\n', pic_dir);
