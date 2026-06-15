%% run_step10b_compare_matched_on_step9_scenario.m
% Step 10B:
% Fair comparison on the SAME scenario (initial state, reference, packet sequence):
% - baseline: independent local delayed feedback (Step6-style)
% - coop_ideal: local + cooperative(repulsion) with CURRENT states (idealized)
% - coop_delay: local + cooperative(repulsion) with DELAYED states (realistic)
%
% coop_use_delay: toggles which cooperative mode to simulate.

clear; clc;
this_file = mfilename('fullpath');
this_dir = fileparts(this_file);
project_root = fileparts(this_dir);

f9 = fullfile(project_root, 'step9_multi12_coop_demo.mat');
fsol = fullfile(project_root, 'agv_mss_solution.mat');
if ~exist(f9, 'file')
    error('Missing step9_multi12_coop_demo.mat. Run scripts/run_step9_multi12_coop_demo.m first.');
end
if ~exist(fsol, 'file')
    error('Missing agv_mss_solution.mat. Run scripts/run_agv_mss_fixedK_search.m first.');
end

S9 = load(f9);
Ssol = load(fsol);

single = Ssol.single;
K = Ssol.sol.K_delay_state;
N = S9.N;
Ts = S9.Ts;
p = S9.p;
d = S9.d;
steps = size(S9.u_act_log, 2);

n = single.n;
m = single.nu;
x0 = S9.x(:,1);
x_ref = S9.x_ref;

% ===== Toggle: cooperative mode =====
coop_use_delay = false;  % true = realistic (delayed neighbors); false = idealized (current neighbors)
% ====================================

% Cooperative parameters
if isfield(S9, 'kc_pos'); kc_pos = S9.kc_pos; else; kc_pos = 0.10; end
if isfield(S9, 'kc_vel'); kc_vel = S9.kc_vel; else; kc_vel = 0.35; end
if isfield(S9, 'd_safe'); d_safe = S9.d_safe; else; d_safe = 0.30; end
if isfield(S9, 'k_rep'); k_rep = S9.k_rep; else; k_rep = 0.45; end
if isfield(S9, 'u_max'); u_max = S9.u_max; else; u_max = 1.2; end

if mod(N,4) ~= 0
    error('This script assumes group size=4. Current N=%d', N);
end
group_size = 4;
num_groups = N / group_size;

% Build intra-group ring neighbors
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

% Shared Bernoulli packet-success sequence for fairness
rng(2026);
gamma = rand(N, steps) > p;

% -------- baseline simulation (independent local only) --------
x_base = zeros(n*N, steps+1);
x_base(:,1) = x0;
u_prev_base = zeros(m*N, 1);
xhist_base = zeros(n*N, steps+1);
xhist_base(:,1) = x0;
pos_err_base = zeros(N, steps+1);
min_dist_base = zeros(1, steps+1);

for i = 1:N
    ix = (i-1)*n + (1:n);
    ei = x_base(ix,1) - x_ref(ix);
    pos_err_base(i,1) = norm(ei(1:2));
end
min_dist_base(1) = compute_min_pair_distance(x_base(:,1), n, N);

for k = 1:steps
    u_act = zeros(m*N,1);
    for i = 1:N
        ix = (i-1)*n + (1:n);
        iu = (i-1)*m + (1:m);
        idx_delay = k - d;
        if idx_delay < 1
            x_del = xhist_base(ix,1);
        else
            x_del = xhist_base(ix,idx_delay);
        end
        uc = K * (x_del - x_ref(ix));
        gk = gamma(i,k);
        u_act(iu) = gk * uc + (1-gk) * u_prev_base(iu);
    end
    for i = 1:N
        ix = (i-1)*n + (1:n);
        iu = (i-1)*m + (1:m);
        x_base(ix,k+1) = single.Ad * x_base(ix,k) + single.Bd * u_act(iu);
    end
    xhist_base(:,k+1) = x_base(:,k+1);
    u_prev_base = u_act;
    for i = 1:N
        ix = (i-1)*n + (1:n);
        ei = x_base(ix,k+1) - x_ref(ix);
        pos_err_base(i,k+1) = norm(ei(1:2));
    end
    min_dist_base(k+1) = compute_min_pair_distance(x_base(:,k+1), n, N);
end

% -------- cooperative simulation (local + coop + repulsion) --------
x_coop = zeros(n*N, steps+1);
x_coop(:,1) = x0;
u_prev_coop = zeros(m*N,1);
xhist_coop = zeros(n*N, steps+1);
xhist_coop(:,1) = x0;
pos_err_coop = zeros(N, steps+1);
min_dist_coop = zeros(1, steps+1);

for i = 1:N
    ix = (i-1)*n + (1:n);
    ei = x_coop(ix,1) - x_ref(ix);
    pos_err_coop(i,1) = norm(ei(1:2));
end
min_dist_coop(1) = compute_min_pair_distance(x_coop(:,1), n, N);

for k = 1:steps
    u_act = zeros(m*N,1);
    for i = 1:N
        ix = (i-1)*n + (1:n);
        iu = (i-1)*m + (1:m);
        idx_delay = k - d;
        if idx_delay < 1
            x_del = xhist_coop(ix,1);
        else
            x_del = xhist_coop(ix,idx_delay);
        end

        x_ref_i = x_ref(ix);
        u_local = K * (x_del - x_ref_i);

        if coop_use_delay
            % realistic: self uses current state, neighbors use delayed
            p_i = x_coop(ix(1:2),k);
            v_i = x_coop(ix(3:4),k);
        else
            % idealized: both self and neighbors use current states
            p_i = x_coop(ix(1:2),k);
            v_i = x_coop(ix(3:4),k);
        end
        p_ref_i = x_ref_i(1:2);

        u_coop = zeros(2,1);
        nei = neighbors{i};
        for jj = 1:numel(nei)
            j = nei(jj);
            jx = (j-1)*n + (1:n);
            if coop_use_delay
                idx_nb_delay = k - d;
                if idx_nb_delay < 1
                    x_nb_del = xhist_coop(jx,1);
                else
                    x_nb_del = xhist_coop(jx,idx_nb_delay);
                end
                p_j = x_nb_del(1:2);
                v_j = x_nb_del(3:4);
            else
                p_j = x_coop(jx(1:2),k);
                v_j = x_coop(jx(3:4),k);
            end
            p_ref_j = x_ref(jx(1:2));
            e_rel_p = (p_i - p_j) - (p_ref_i - p_ref_j);
            e_rel_v = (v_i - v_j);
            u_coop = u_coop - kc_pos * e_rel_p - kc_vel * e_rel_v;
        end

        % Repulsion: always uses current state (local sensors)
        u_rep = zeros(2,1);
        for j = 1:N
            if j == i
                continue;
            end
            jx = (j-1)*n + (1:n);
            p_j_rep = x_coop(jx(1:2),k);
            dp = p_i - p_j_rep;
            dij = norm(dp);
            if dij < d_safe
                dir = dp / max(dij, 1e-6);
                u_rep = u_rep + k_rep * (d_safe - dij) * dir;
            end
        end

        uc = u_local + u_coop + u_rep;
        uc = min(max(uc, -u_max), u_max);

        gk = gamma(i,k);
        u_act(iu) = gk * uc + (1-gk) * u_prev_coop(iu);
    end
    for i = 1:N
        ix = (i-1)*n + (1:n);
        iu = (i-1)*m + (1:m);
        x_coop(ix,k+1) = single.Ad * x_coop(ix,k) + single.Bd * u_act(iu);
    end
    xhist_coop(:,k+1) = x_coop(:,k+1);
    u_prev_coop = u_act;
    for i = 1:N
        ix = (i-1)*n + (1:n);
        ei = x_coop(ix,k+1) - x_ref(ix);
        pos_err_coop(i,k+1) = norm(ei(1:2));
    end
    min_dist_coop(k+1) = compute_min_pair_distance(x_coop(:,k+1), n, N);
end

mean_base = mean(pos_err_base,1);
mean_coop = mean(pos_err_coop,1);
t = (0:steps) * Ts;

final_base = mean_base(end);
final_coop = mean_coop(end);
decay_base = final_base / max(mean_base(1), 1e-12);
decay_coop = final_coop / max(mean_coop(1), 1e-12);
improve_final_pct = (final_base - final_coop) / max(final_base, 1e-12) * 100;
improve_decay_pct = (decay_base - decay_coop) / max(decay_base, 1e-12) * 100;

if coop_use_delay
    delay_tag = '_delayed';
    coop_label = 'Coop(delayed)';
else
    delay_tag = '_ideal';
    coop_label = 'Coop(ideal)';
end

pic_dir = fullfile(project_root, 'pic');
data_dir = fullfile(project_root, 'data');
if ~exist(pic_dir, 'dir'); mkdir(pic_dir); end
if ~exist(data_dir, 'dir'); mkdir(data_dir); end

figure('Name','Matched compare mean error');
plot(t, mean_base, 'LineWidth', 1.7); hold on;
plot(t, mean_coop, 'LineWidth', 1.7);
grid on;
xlabel('t (s)'); ylabel('mean ||e_{pos}||');
legend('Independent', coop_label, 'Location', 'northeast');
title(sprintf('Matched comparison, coop %s', delay_tag(2:end)));
exportgraphics(gcf, fullfile(pic_dir, ['compare_matched_step9_scenario_mean_err', delay_tag, '.png']), 'Resolution', 220);

figure('Name','Matched compare min pair distance');
plot(t, min_dist_base, 'LineWidth', 1.5); hold on;
plot(t, min_dist_coop, 'LineWidth', 1.5);
yline(0.20, '--r', '0.20m safety threshold');
grid on;
xlabel('t (s)'); ylabel('minimum pairwise distance');
legend('Independent', coop_label, 'Location', 'best');
title(sprintf('Safety comparison, coop %s', delay_tag(2:end)));
exportgraphics(gcf, fullfile(pic_dir, ['compare_matched_step9_scenario_min_dist', delay_tag, '.png']), 'Resolution', 220);

summary = table(final_base, final_coop, decay_base, decay_coop, improve_final_pct, improve_decay_pct, ...
    min(min_dist_base), min(min_dist_coop), ...
    'VariableNames', {'final_mean_err_independent','final_mean_err_cooperative', ...
    'decay_independent','decay_cooperative','improve_final_err_pct','improve_decay_pct', ...
    'min_pair_dist_independent','min_pair_dist_cooperative'});

csv_file = fullfile(data_dir, ['compare_matched_step9_scenario_summary', delay_tag, '.csv']);
writetable(summary, csv_file);

if coop_use_delay
    fprintf('coop_use_delay = 1 (realistic delayed)\n');
else
    fprintf('coop_use_delay = 0 (idealized)\n');
end
fprintf('Saved matched compare figure(mean error): %s\n', fullfile(pic_dir, ['compare_matched_step9_scenario_mean_err', delay_tag, '.png']));
fprintf('Saved matched compare figure(min dist): %s\n', fullfile(pic_dir, ['compare_matched_step9_scenario_min_dist', delay_tag, '.png']));
fprintf('Saved matched compare summary: %s\n\n', csv_file);
disp(summary);

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
