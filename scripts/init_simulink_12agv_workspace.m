%% init_simulink_12agv_workspace.m
% Initialize base-workspace variables for:
% - slimulink/single_agv_baseline.slx
% - slimulink/12_agv.slx (12-AGV extension)

clearvars -except ans;
clc;

this_file = mfilename('fullpath');
this_dir = fileparts(this_file);
project_root = fileparts(this_dir);

sol_file = fullfile(project_root, 'agv_mss_solution.mat');
if ~exist(sol_file, 'file')
    error('Cannot find %s. Run scripts/run_agv_mss_fixedK_search.m first.', sol_file);
end
S = load(sol_file);

% Core plant/controller parameters used by Simulink blocks
Ad = S.single.Ad; %#ok<NASGU>
Bd = S.single.Bd; %#ok<NASGU>
K  = S.sol.K_delay_state; %#ok<NASGU>
Ts = S.Ts; %#ok<NASGU>

% Network settings (can be overridden before sim)
p = 0.20; %#ok<NASGU>      % packet-loss probability
d_sel = 2; %#ok<NASGU>     % selected fixed delay branch: 1/2/3

% Cooperative-control defaults (for x12_agv.slx Constant blocks)
% IMPORTANT: set Constant block values to variable names (kc_pos, kc_vel, enable_coop)
kc_pos = 0.08; %#ok<NASGU>
kc_vel = 0.10; %#ok<NASGU>
enable_coop = 1; %#ok<NASGU>

% Single-AGV defaults
x0 = [1.0; -0.8; 0; 0]; %#ok<NASGU>
ref = [0; 0; 0; 0]; %#ok<NASGU>

% 12-AGV defaults
N = 12; %#ok<NASGU>

% Three group centers (front/middle/rear sections)
centers = [-1.5, 0.0;
            0.0, 0.0;
            1.5, 0.0];

% 4-AGV square formation offsets in each group
offs = [-0.25, -0.25;
         0.25, -0.25;
        -0.25,  0.25;
         0.25,  0.25];

% Initial states and references for AGV1..AGV12
X0_all = zeros(4,12); %#ok<NASGU>
REF_all = zeros(4,12); %#ok<NASGU>
for i = 1:12
    % Use (i-1) to avoid endpoint ambiguity at 2*pi
    ang = 2*pi*(i-1)/12;
    x0_i = [1.2*cos(ang); 1.2*sin(ang); 0; 0];
    assignin('base', sprintf('x0_%d', i), x0_i);
    X0_all(:, i) = x0_i;

    g = floor((i-1)/4) + 1;
    r = mod(i-1, 4) + 1;
    pr = centers(g, :) + offs(r, :);
    ref_i = [pr(1); pr(2); 0; 0];
    assignin('base', sprintf('ref_%d', i), ref_i);
    REF_all(:, i) = ref_i;
end
assignin('base', 'X0_all', X0_all);
assignin('base', 'REF_all', REF_all);

% Quick sanity check: minimum initial pair distance should be > 0
min_d0 = inf;
for i = 1:11
    for j = i+1:12
        dij = norm(X0_all(1:2,i) - X0_all(1:2,j));
        if dij < min_d0
            min_d0 = dij;
        end
    end
end
assignin('base', 'min_init_pair_dist', min_d0);

fprintf('Workspace initialized for Simulink models.\n');
fprintf(['Loaded: Ad, Bd, K, Ts, p, d_sel, kc_pos, kc_vel, enable_coop, ', ...
         'x0, ref, x0_1..x0_12, ref_1..ref_12\n']);
fprintf('Initial-position sanity: min pair distance at t=0 from x0_i = %.4f m\n', min_d0);
fprintf('Tip: run this before sim:\n');
fprintf('  run(''scripts/init_simulink_12agv_workspace.m'')\n');
