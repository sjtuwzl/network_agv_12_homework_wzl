%% run_agv_mss_fixedK_search.m
% Robust alternative when joint K,P synthesis is numerically unstable:
% 1) Search structured delayed-state PD gains K_delay
% 2) Verify MSS by solving linear LMI in P only

clear; clc;
this_file = mfilename('fullpath');
this_dir = fileparts(this_file);
project_root = fileparts(this_dir);
addpath(fullfile(project_root, 'src'));
setup_solver_paths(project_root);

Ts = 0.05;
m = 80;
N = 12;
d = 2;
p = 0.2;
rho_try = [0.01, 0.005, 0.002, 0.001, 0.0];

single = build_single_agv_discrete(Ts, m);
stacked = build_stacked_multiagv_model(single, N);
aug = build_augmented_packet_delay_model(single.Ad, single.Bd, d, "loss-hold");

fprintf('Using fixed-K search with MSS verification.\n');
fprintf('which sdpvar: %s\n', which('sdpvar'));
fprintf('which optimize: %s\n', which('optimize'));
fprintf('which sedumi: %s\n', which('sedumi'));

% Structured PD-like K:
% u_x = -kp*px - kv*vx, u_y = -kp*py - kv*vy
% Stage-1: coarse grid for speed
kp_grid = linspace(0.01, 4.0, 40);
kv_grid = linspace(0.02, 6.0, 45);

best = struct('found', false);

for rr = 1:numel(rho_try)
    rho = rho_try(rr);
    fprintf('\n--- rho=%.4f ---\n', rho);
    for ikp = 1:numel(kp_grid)
        kp = kp_grid(ikp);
        for ikv = 1:numel(kv_grid)
            kv = kv_grid(ikv);
            Kd = [-kp, 0, -kv, 0;
                   0, -kp, 0, -kv];
            out = verify_mss_fixedK_yalmip(aug, p, Kd, struct( ...
                'solver', "sedumi", ...
                'rho', rho, ...
                'eps_pd', 1e-8, ...
                'verbose', false, ...
                'normalize_trace', true));
            if out.feasible
                fprintf('Found feasible K at rho=%.4f, kp=%.4f, kv=%.4f\n', rho, kp, kv);
                best.found = true;
                best.kp = kp;
                best.kv = kv;
                best.sol = out;
                break;
            end
        end
        if best.found, break; end
    end
    if best.found, break; end
end

% Stage-2 fallback: if p=0.2,d=2 fails, relax scenario to find a working baseline.
if ~best.found
    fprintf('\nNo feasible K for (p=%.2f,d=%d). Start fallback search on milder network cases...\n', p, d);
    fallback_cases = [0.15 2; 0.10 2; 0.10 1; 0.05 1; 0.05 0];
    for c = 1:size(fallback_cases, 1)
        p_try = fallback_cases(c, 1);
        d_try = fallback_cases(c, 2);
        aug_try = build_augmented_packet_delay_model(single.Ad, single.Bd, d_try, "loss-hold");
        fprintf('Trying fallback case p=%.2f, d=%d ...\n', p_try, d_try);
        for rr = 1:numel(rho_try)
            rho = rho_try(rr);
            for ikp = 1:numel(kp_grid)
                kp = kp_grid(ikp);
                for ikv = 1:numel(kv_grid)
                    kv = kv_grid(ikv);
                    Kd = [-kp, 0, -kv, 0;
                           0, -kp, 0, -kv];
                    out = verify_mss_fixedK_yalmip(aug_try, p_try, Kd, struct( ...
                        'solver', "sedumi", ...
                        'rho', rho, ...
                        'eps_pd', 1e-8, ...
                        'verbose', false, ...
                        'normalize_trace', true));
                    if out.feasible
                        fprintf('Fallback feasible: p=%.2f, d=%d, rho=%.4f, kp=%.4f, kv=%.4f\n', p_try, d_try, rho, kp, kv);
                        best.found = true;
                        best.kp = kp;
                        best.kv = kv;
                        best.sol = out;
                        p = p_try;
                        d = d_try;
                        aug = aug_try;
                        break;
                    end
                end
                if best.found, break; end
            end
            if best.found, break; end
        end
        if best.found, break; end
    end
end

if ~best.found
    error('Fixed-K search failed. Try reducing Ts, mass m, or expanding kp/kv grids.');
end

sol = best.sol;
K_delay = sol.K_delay_state;
fprintf('\nMSS feasible (fixed-K verification). K_delay =\n');
disp(K_delay);

sol_out = fullfile(project_root, 'agv_mss_solution.mat');
rho = sol.rho; %#ok<NASGU>
save(sol_out, 'single', 'stacked', 'aug', 'sol', 'Ts', 'm', 'N', 'd', 'p', 'rho');
fprintf('Saved results to: %s\n', sol_out);
