%% run_step9_multi12_coop_demo.m
% Step 9:
% 12-AGV grouped cooperative control demo (3 groups x 4 AGVs)
% - local delayed-state feedback from MSS gain K
% - intra-group consensus/formation correction
% - packet loss + fixed delay with hold-last actuation

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
    error('Cannot find agv_mss_solution.mat. Run scripts/run_agv_mss_fixedK_search.m first.');
end
S = load(sol_file);

Ts = S.Ts;
N = S.N;
single = S.single;
K = S.sol.K_delay_state; % 2x4
p = S.p;
d = S.d;

if N ~= 12
    error('This demo is configured for N=12. Current N=%d.', N);
end

n = single.n;
m = single.nu;
steps = 500;

% Cooperative/safety parameters (defaults)
kc_pos = 0.10;
kc_vel = 0.35;
u_max = 1.2;
d_safe = 0.30;
k_rep = 0.45;

% If Step11 recommended params exist, load them automatically.
rec_file = fullfile(project_root, 'step11_recommended_params.mat');
if exist(rec_file, 'file')
    R = load(rec_file);
    if isfield(R, 'rec') && istable(R.rec) && height(R.rec) >= 1
        kc_pos = R.rec.kc_pos(1);
        kc_vel = R.rec.kc_vel(1);
        d_safe = R.rec.d_safe(1);
        k_rep = R.rec.k_rep(1);
        u_max = R.rec.u_max(1);
        fprintf('Loaded Step11 recommended params from: %s\n', rec_file);
    end
end
fprintf('Using params: kc_pos=%.3f, kc_vel=%.3f, d_safe=%.3f, k_rep=%.3f, u_max=%.3f\n', ...
    kc_pos, kc_vel, d_safe, k_rep, u_max);

% Grouping: 3 groups, each group has 4 AGVs
group_size = 4;
num_groups = 3;

% Desired group centers (can be adjusted according to workshop layout)
group_centers = [ ...
    -1.5,  0.0; ...
     0.0,  0.0; ...
     1.5,  0.0];

% Desired formation offsets within each group (square)
offsets = [ ...
    -0.25, -0.25; ...
     0.25, -0.25; ...
    -0.25,  0.25; ...
     0.25,  0.25];

% Build AGV-wise position references
pos_ref_all = zeros(2, N);
for g = 1:num_groups
    for r = 1:group_size
        idx = (g-1)*group_size + r;
        pos_ref_all(:, idx) = (group_centers(g, :) + offsets(r, :))';
    end
end

x_ref = zeros(n*N, 1);
for i = 1:N
    ix = (i-1)*n + (1:n);
    x_ref(ix) = [pos_ref_all(:, i); 0; 0];
end

% Initial positions: around a large circle (away from references)
x = zeros(n*N, steps+1);
for i = 1:N
    ang = 2*pi*i/N;
    ix = (i-1)*n + (1:n);
    x(ix, 1) = [2.0*cos(ang); 1.2*sin(ang); 0; 0];
end

u_prev = zeros(m*N, 1);
u_act_log = zeros(m*N, steps);
u_cmd_log = zeros(m*N, steps);
pos_err_norm = zeros(N, steps+1);
group_center_err = zeros(num_groups, steps+1);
min_pair_dist = zeros(1, steps+1);

x_hist = zeros(n*N, steps+1);
x_hist(:, 1) = x(:, 1);

% Build intra-group ring neighbors (two neighbors per AGV in each group)
neighbors = cell(N, 1);
for g = 1:num_groups
    base = (g-1)*group_size;
    for r = 1:group_size
        i = base + r;
        left = base + mod(r-2, group_size) + 1;
        right = base + mod(r, group_size) + 1;
        neighbors{i} = [left, right];
    end
end

% Init metrics
for i = 1:N
    ix = (i-1)*n + (1:n);
    e_i = x(ix, 1) - x_ref(ix);
    pos_err_norm(i, 1) = norm(e_i(1:2));
end
for g = 1:num_groups
    ids = (g-1)*group_size + (1:group_size);
    center_now = mean(pos_ref_all(:, ids), 2); %#ok<NASGU> % keep shape reference
    pos_now = zeros(2, group_size);
    for kk = 1:group_size
        i = ids(kk);
        ix = (i-1)*n + (1:n);
        pos_now(:, kk) = x(ix(1:2), 1);
    end
    center_now = mean(pos_now, 2);
    group_center_err(g, 1) = norm(center_now - group_centers(g, :)');
end
min_pair_dist(1) = compute_min_pair_distance(x(:,1), n, N);

rng(2026);
for k = 1:steps
    u_act = zeros(m*N, 1);
    u_cmd = zeros(m*N, 1);

    for i = 1:N
        ix = (i-1)*n + (1:n);
        iu = (i-1)*m + (1:m);

        % delayed state for local gain
        idx_delay = k - d;
        if idx_delay < 1
            x_del = x_hist(ix, 1);
        else
            x_del = x_hist(ix, idx_delay);
        end

        x_ref_i = x_ref(ix);
        u_local = K * (x_del - x_ref_i);

        % cooperative term (use current states for relative correction)
        p_i = x(ix(1:2), k);
        v_i = x(ix(3:4), k);
        p_ref_i = x_ref_i(1:2);
        u_coop = zeros(2,1);

        nei = neighbors{i};
        for jj = 1:numel(nei)
            j = nei(jj);
            jx = (j-1)*n + (1:n);
            p_j = x(jx(1:2), k);
            v_j = x(jx(3:4), k);
            p_ref_j = x_ref(jx(1:2));

            e_rel_p = (p_i - p_j) - (p_ref_i - p_ref_j);
            e_rel_v = (v_i - v_j);
            u_coop = u_coop - kc_pos * e_rel_p - kc_vel * e_rel_v;
        end

        % Repulsion term from all AGVs when too close
        u_rep = zeros(2,1);
        for j = 1:N
            if j == i
                continue;
            end
            jx = (j-1)*n + (1:n);
            p_j = x(jx(1:2), k);
            dp = p_i - p_j;
            dij = norm(dp);
            if dij < d_safe
                dir = dp / max(dij, 1e-6);
                u_rep = u_rep + k_rep * (d_safe - dij) * dir;
            end
        end

        uc = u_local + u_coop + u_rep;
        uc = min(max(uc, -u_max), u_max);
        gk = rand() > p; % success=1 with prob 1-p
        u_act(iu) = gk * uc + (1-gk) * u_prev(iu);
        u_cmd(iu) = uc;
    end

    % plant update
    for i = 1:N
        ix = (i-1)*n + (1:n);
        iu = (i-1)*m + (1:m);
        x(ix, k+1) = single.Ad * x(ix, k) + single.Bd * u_act(iu);
    end

    x_hist(:, k+1) = x(:, k+1);
    u_prev = u_act;
    u_act_log(:, k) = u_act;
    u_cmd_log(:, k) = u_cmd;

    for i = 1:N
        ix = (i-1)*n + (1:n);
        e_i = x(ix, k+1) - x_ref(ix);
        pos_err_norm(i, k+1) = norm(e_i(1:2));
    end

    for g = 1:num_groups
        ids = (g-1)*group_size + (1:group_size);
        pos_now = zeros(2, group_size);
        for kk = 1:group_size
            i = ids(kk);
            ix = (i-1)*n + (1:n);
            pos_now(:, kk) = x(ix(1:2), k+1);
        end
        center_now = mean(pos_now, 2);
        group_center_err(g, k+1) = norm(center_now - group_centers(g, :)');
    end

    min_pair_dist(k+1) = compute_min_pair_distance(x(:,k+1), n, N);
end

t = (0:steps) * Ts;
mean_err = mean(pos_err_norm, 1);

figure('Name', 'Step9 - all AGV position error norms');
plot(t, pos_err_norm', 'LineWidth', 1.0);
grid on; xlabel('t (s)'); ylabel('||e_{pos,i}||');
title(sprintf('Step9 cooperative 12-AGV, p=%.2f, d=%d', p, d));
exportgraphics(gcf, fullfile(pic_dir, sprintf('step9_all_pos_err_p%.2f_d%d.png', p, d)), 'Resolution', 220);

figure('Name', 'Step9 - mean position error');
plot(t, mean_err, 'LineWidth', 1.8);
grid on; xlabel('t (s)'); ylabel('mean ||e_{pos}||');
title('Step9 mean position error');
exportgraphics(gcf, fullfile(pic_dir, sprintf('step9_mean_pos_err_p%.2f_d%d.png', p, d)), 'Resolution', 220);

figure('Name', 'Step9 - group center error');
plot(t, group_center_err', 'LineWidth', 1.3);
grid on; xlabel('t (s)'); ylabel('group center error norm');
legend('Group1','Group2','Group3', 'Location', 'northeast');
title('Step9 group-center tracking errors');
exportgraphics(gcf, fullfile(pic_dir, sprintf('step9_group_center_err_p%.2f_d%d.png', p, d)), 'Resolution', 220);

figure('Name', 'Step9 - minimum pairwise distance');
plot(t, min_pair_dist, 'LineWidth', 1.6);
grid on; xlabel('t (s)'); ylabel('minimum pairwise distance');
title('Step9 minimum pairwise AGV distance');
exportgraphics(gcf, fullfile(pic_dir, sprintf('step9_min_pair_dist_p%.2f_d%d.png', p, d)), 'Resolution', 220);

tag = sprintf('kp%.2f_kv%.2f_ds%.2f_kr%.2f_um%.1f', kc_pos, kc_vel, d_safe, k_rep, u_max);
tag = strrep(tag, '.', 'p');
out_file = fullfile(project_root, ['step9_multi12_coop_demo_', tag, '.mat']);
save(out_file, 'x', 'x_ref', 'u_act_log', 'u_cmd_log', 'pos_err_norm', 'mean_err', ...
    'group_center_err', 'min_pair_dist', 'Ts', 'p', 'd', 'N', 'kc_pos', 'kc_vel', ...
    'group_centers', 'offsets', 'd_safe', 'k_rep', 'u_max');
% Keep a stable latest file name for downstream scripts.
copyfile(out_file, fullfile(project_root, 'step9_multi12_coop_demo.mat'));

fprintf('Saved Step9 cooperative results to: %s\n', out_file);
fprintf('Saved Step9 figures to: %s\n', pic_dir);
fprintf('Initial mean position error: %.4f\n', mean_err(1));
fprintf('Final mean position error: %.4f\n', mean_err(end));
fprintf('Decay ratio (final/initial): %.4f\n', mean_err(end)/max(mean_err(1), 1e-12));
fprintf('Minimum pairwise distance over trajectory: %.4f\n', min(min_pair_dist));
if min(min_pair_dist) < 0.20
    fprintf('Safety check: WARNING (min distance < 0.20 m)\n');
else
    fprintf('Safety check: PASS (min distance >= 0.20 m)\n');
end

function dmin = compute_min_pair_distance(xk, n, N)
    pos = zeros(2, N);
    for ii = 1:N
        ix = (ii-1)*n + (1:n);
        pos(:, ii) = xk(ix(1:2));
    end
    dmin = inf;
    for i = 1:N-1
        for j = i+1:N
            dij = norm(pos(:, i) - pos(:, j));
            if dij < dmin
                dmin = dij;
            end
        end
    end
end
