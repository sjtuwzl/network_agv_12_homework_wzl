%% run_agv_mss_demo.m
% End-to-end demo:
% Step 1) single AGV discretization and 12-AGV stacking
% Step 2) packet loss + fixed delay model
% Step 3) augmented switched-delay closed-loop form
% Step 4-5) MSS LMI and K solve with YALMIP + SDP solver

clear; clc;
this_file = mfilename('fullpath');
this_dir = fileparts(this_file);
project_root = fileparts(this_dir);
addpath(fullfile(project_root, 'src'));
setup_solver_paths(project_root);

% Optional: if user places YALMIP under project_root/third_party/yalmip
yalmip_local = fullfile(project_root, 'third_party', 'yalmip');
if exist(yalmip_local, 'dir')
    addpath(genpath(yalmip_local), '-begin');
end

%% User-tunable parameters
Ts = 0.05;      % sampling time (s)
m = 80;         % AGV mass
N = 12;         % number of AGVs
d = 2;          % fixed input delay (samples)
p = 0.2;        % packet loss probability, 0<=p<1
rho_try = [0.01, 0.005, 0.002, 0.001, 0.0]; % fallback list

% Prefer SeDuMi release (contains precompiled windows binaries).
% Only use SDPT3 when its mex backend is available.
solver_list = ["sedumi"];
if ~isempty(which('sqlp')) && ~isempty(which('mexmat'))
    solver_list = ["sedumi", "sdpt3"];
else
    fprintf('SDPT3 mex backend not detected, skip SDPT3 and use SeDuMi only.\n');
end

fprintf('=== Step 1: Single AGV and %d-AGV stacked model ===\n', N);
single = build_single_agv_discrete(Ts, m);
stacked = build_stacked_multiagv_model(single, N);
fprintf('single: n=%d, nu=%d\n', single.n, single.nu);
fprintf('stacked: n=%d, nu=%d\n', stacked.n, stacked.nu);

fprintf('\n=== Step 2-3: Delay+loss augmented model (single AGV) ===\n');
aug = build_augmented_packet_delay_model(single.Ad, single.Bd, d, "loss-hold");
fprintf('augmented state dimension nx_aug=%d\n', aug.nx_aug);

fprintf('\n=== Step 4-5: MSS LMI solve ===\n');
fprintf('which sdpvar: %s\n', which('sdpvar'));
fprintf('which optimize: %s\n', which('optimize'));
fprintf('which sdpsettings: %s\n', which('sdpsettings'));
fprintf('which sedumi: %s\n', which('sedumi'));
fprintf('which sqlp: %s\n', which('sqlp'));

if isempty(which('sdpvar')) || isempty(which('sdpsettings'))
    error(['YALMIP is not installed/found. Please run:\n', ...
           'run(''scripts/install_yalmip_and_solvers.m'')\n', ...
           'or manually add YALMIP path first.']);
end

sol = [];
for rr = 1:numel(rho_try)
    rho = rho_try(rr);
    fprintf('\n--- Trying rho = %.4f ---\n', rho);
    for s = solver_list
        try
            fprintf('Trying solver: %s\n', s);
            sol = solve_mss_lmi_yalmip(aug, p, struct( ...
                'solver', s, ...
                'verbose', true, ...
                'rho', rho, ...
                'enforce_delay_structure', true, ...
                'normalize_trace', true, ...
                'objective_mode', "feasibility"));
            if sol.feasible
                fprintf('Solver %s found feasible solution at rho=%.4f.\n', s, rho);
                break;
            else
                fprintf('Solver %s infeasible/problem=%d (%s)\n', s, sol.problem, sol.info);
            end
        catch ME
            fprintf('Solver %s failed with error: %s\n', s, ME.message);
        end
    end
    if ~isempty(sol) && isfield(sol, 'feasible') && sol.feasible
        break;
    end
end

if isempty(sol) || ~isfield(sol, 'feasible') || ~sol.feasible
    error('No feasible solution found. Check YALMIP and SDP solver installation.');
end

K_delay = sol.K_delay_state;
fprintf('\nMSS feasible. K on delayed state x(k-d):\n');
disp(K_delay);
fprintf('rho = %.4f, enforce_delay_structure = %d\n', sol.rho, sol.enforce_delay_structure);

sol_out = fullfile(project_root, 'agv_mss_solution.mat');
save(sol_out, 'single', 'stacked', 'aug', 'sol', 'Ts', 'm', 'N', 'd', 'p', 'rho');
fprintf('Saved results to: %s\n', sol_out);
