%% run_step11_tune_coop_safety.m
% Step 11:
% Tune cooperative + repulsion parameters on the matched Step9 scenario.
% Goal: satisfy safety (min pair distance >= threshold) while keeping
% cooperative tracking improvement.
%
% coop_use_delay: if true, cooperative/repulsion use delayed states
%                 (realistic); if false, use current states (idealized).

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

% ===== Toggle: whether cooperative/repulsion use delayed states =====
coop_use_delay = true;  % true = realistic (neighbors also delayed); false = idealized
% ===================================================================

% Group topology assumption: 3 groups x 4 AGVs
if mod(N,4) ~= 0
    error('This tuning script assumes group size=4. Current N=%d', N);
end
group_size = 4;
num_groups = N / group_size;

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

% Baseline-independent result for reference (same scenario, same gamma)
rng(2026);
gamma = rand(N, steps) > p;
[final_ind, decay_ind, min_dist_ind] = simulate_independent(single, K, d, x0, x_ref, gamma);

fprintf('Reference (independent) on matched scenario:\n');
fprintf('  final_mean_err = %.4f\n', final_ind);
fprintf('  decay_ratio    = %.4f\n', decay_ind);
fprintf('  min_pair_dist  = %.4f\n\n', min_dist_ind);

% Grid to tune (moderate size, can expand later)
kc_pos_list = [0.04, 0.08, 0.12, 0.16];
kc_vel_list = [0.10, 0.20, 0.30];
d_safe_list = [0.25, 0.35, 0.45];
k_rep_list = [0.8, 1.2, 1.8, 2.4];
u_max_list = [1.0, 1.2];

rows = numel(kc_pos_list) * numel(kc_vel_list) * numel(d_safe_list) * numel(k_rep_list) * numel(u_max_list);
T = table('Size', [rows 12], ...
    'VariableTypes', {'double','double','double','double','double','double','double','double','double','double','double','double'}, ...
    'VariableNames', {'kc_pos','kc_vel','d_safe','k_rep','u_max','final_mean_err','decay_ratio', ...
    'min_pair_dist','improve_final_pct','improve_decay_pct','safe_flag','score'});

r = 0;
for a = 1:numel(kc_pos_list)
    for b = 1:numel(kc_vel_list)
        for c = 1:numel(d_safe_list)
            for e = 1:numel(k_rep_list)
                for f = 1:numel(u_max_list)
                    r = r + 1;
                    kc_pos = kc_pos_list(a);
                    kc_vel = kc_vel_list(b);
                    d_safe = d_safe_list(c);
                    k_rep = k_rep_list(e);
                    u_max = u_max_list(f);

                    [final_coop, decay_coop, min_dist_coop] = simulate_coop( ...
                        single, K, d, x0, x_ref, gamma, neighbors, ...
                        kc_pos, kc_vel, d_safe, k_rep, u_max, coop_use_delay);

                    improve_final = (final_ind - final_coop) / max(final_ind, 1e-12) * 100;
                    improve_decay = (decay_ind - decay_coop) / max(decay_ind, 1e-12) * 100;
                    safe_flag = double(min_dist_coop >= 0.20);

                    % Score: prioritize safety first, then smaller final error.
                    % If unsafe, subtract a heavy penalty based on distance violation.
                    if safe_flag > 0.5
                        score = final_coop;
                    else
                        score = final_coop + 10.0 * (0.20 - min_dist_coop);
                    end

                    T{r, :} = [kc_pos, kc_vel, d_safe, k_rep, u_max, final_coop, decay_coop, ...
                               min_dist_coop, improve_final, improve_decay, safe_flag, score];
                end
            end
        end
    end
end

% Sort by score (ascending)
T = sortrows(T, 'score', 'ascend');

% Save artifacts
data_dir = fullfile(project_root, 'data');
if ~exist(data_dir, 'dir')
    mkdir(data_dir);
end
if coop_use_delay
    delay_tag = '_delayed';
else
    delay_tag = '_ideal';
end
csv_file = fullfile(data_dir, ['step11_tune_coop_safety_results', delay_tag, '.csv']);
writetable(T, csv_file);

if coop_use_delay
    fprintf('coop_use_delay = 1 (realistic delayed)\n');
else
    fprintf('coop_use_delay = 0 (idealized)\n');
end
fprintf('Saved tuning table to: %s\n', csv_file);

% Print top candidates
topk = min(10, height(T));
fprintf('\nTop %d candidates (sorted by score):\n', topk);
disp(T(1:topk, :));

% Pick recommended candidate: safe and minimal final_mean_err
safe_idx = find(T.safe_flag > 0.5);
if ~isempty(safe_idx)
    T_safe = T(safe_idx, :);
    [~, ii] = min(T_safe.final_mean_err);
    rec = T_safe(ii, :);
    fprintf('Recommended SAFE params:\n');
else
    rec = T(1, :);
    fprintf('No safe candidate reached min_pair_dist>=0.20. Recommended best-effort params:\n');
end
disp(rec);

% Save recommended params for easy loading
rec_file = fullfile(project_root, 'step11_recommended_params.mat');
save(rec_file, 'rec', 'final_ind', 'decay_ind', 'min_dist_ind', 'Ts', 'p', 'd');
fprintf('Saved recommendation to: %s\n', rec_file);

function [final_mean, decay_ratio, min_pair_dist] = simulate_independent(single, K, d, x0, x_ref, gamma)
    n = single.n;
    m = single.nu;
    N = size(gamma, 1);
    steps = size(gamma, 2);

    x = zeros(n*N, steps+1);
    x(:,1) = x0;
    xhist = zeros(n*N, steps+1);
    xhist(:,1) = x0;
    u_prev = zeros(m*N, 1);
    pos_err = zeros(N, steps+1);
    min_pair = zeros(1, steps+1);

    for i = 1:N
        ix = (i-1)*n + (1:n);
        ei = x(ix,1) - x_ref(ix);
        pos_err(i,1) = norm(ei(1:2));
    end
    min_pair(1) = compute_min_pair_distance(x(:,1), n, N);

    for k = 1:steps
        u_act = zeros(m*N,1);
        for i = 1:N
            ix = (i-1)*n + (1:n);
            iu = (i-1)*m + (1:m);
            idx_delay = k - d;
            if idx_delay < 1
                x_del = xhist(ix,1);
            else
                x_del = xhist(ix,idx_delay);
            end
            uc = K * (x_del - x_ref(ix));
            gk = gamma(i,k);
            u_act(iu) = gk * uc + (1-gk) * u_prev(iu);
        end
        for i = 1:N
            ix = (i-1)*n + (1:n);
            iu = (i-1)*m + (1:m);
            x(ix,k+1) = single.Ad * x(ix,k) + single.Bd * u_act(iu);
        end
        xhist(:,k+1) = x(:,k+1);
        u_prev = u_act;
        for i = 1:N
            ix = (i-1)*n + (1:n);
            ei = x(ix,k+1) - x_ref(ix);
            pos_err(i,k+1) = norm(ei(1:2));
        end
        min_pair(k+1) = compute_min_pair_distance(x(:,k+1), n, N);
    end

    mean_err = mean(pos_err, 1);
    final_mean = mean_err(end);
    decay_ratio = final_mean / max(mean_err(1), 1e-12);
    min_pair_dist = min(min_pair);
end

function [final_mean, decay_ratio, min_pair_dist] = simulate_coop(single, K, d, x0, x_ref, gamma, neighbors, kc_pos, kc_vel, d_safe, k_rep, u_max, coop_use_delay)
    n = single.n;
    m = single.nu;
    N = size(gamma, 1);
    steps = size(gamma, 2);

    x = zeros(n*N, steps+1);
    x(:,1) = x0;
    xhist = zeros(n*N, steps+1);
    xhist(:,1) = x0;
    u_prev = zeros(m*N,1);
    pos_err = zeros(N, steps+1);
    min_pair = zeros(1, steps+1);

    for i = 1:N
        ix = (i-1)*n + (1:n);
        ei = x(ix,1) - x_ref(ix);
        pos_err(i,1) = norm(ei(1:2));
    end
    min_pair(1) = compute_min_pair_distance(x(:,1), n, N);

    for k = 1:steps
        u_act = zeros(m*N,1);
        for i = 1:N
            ix = (i-1)*n + (1:n);
            iu = (i-1)*m + (1:m);
            idx_delay = k - d;
            if idx_delay < 1
                x_del = xhist(ix,1);
            else
                x_del = xhist(ix,idx_delay);
            end

            x_ref_i = x_ref(ix);
            u_local = K * (x_del - x_ref_i);

            % cooperative term: self=current, neighbors=delayed
            p_i = x(ix(1:2),k);
            v_i = x(ix(3:4),k);
            p_ref_i = x_ref_i(1:2);

            u_coop = zeros(2,1);
            nei = neighbors{i};
            for jj = 1:numel(nei)
                j = nei(jj);
                jx = (j-1)*n + (1:n);
                if coop_use_delay
                    idx_nb_delay = k - d;
                    if idx_nb_delay < 1
                        x_nb_del = xhist(jx,1);
                    else
                        x_nb_del = xhist(jx,idx_nb_delay);
                    end
                    p_j = x_nb_del(1:2);
                    v_j = x_nb_del(3:4);
                else
                    p_j = x(jx(1:2),k);
                    v_j = x(jx(3:4),k);
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
                p_j_rep = x(jx(1:2),k);
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
            u_act(iu) = gk * uc + (1-gk) * u_prev(iu);
        end

        for i = 1:N
            ix = (i-1)*n + (1:n);
            iu = (i-1)*m + (1:m);
            x(ix,k+1) = single.Ad * x(ix,k) + single.Bd * u_act(iu);
        end
        xhist(:,k+1) = x(:,k+1);
        u_prev = u_act;

        for i = 1:N
            ix = (i-1)*n + (1:n);
            ei = x(ix,k+1) - x_ref(ix);
            pos_err(i,k+1) = norm(ei(1:2));
        end
        min_pair(k+1) = compute_min_pair_distance(x(:,k+1), n, N);
    end

    mean_err = mean(pos_err, 1);
    final_mean = mean_err(end);
    decay_ratio = final_mean / max(mean_err(1), 1e-12);
    min_pair_dist = min(min_pair);
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
