%% modify_simulink_add_coop.m
% Add cooperative and repulsion logic to x12_agv.slx
% This script programmatically modifies the Simulink model

clear; clc;
this_file = mfilename('fullpath');
this_dir = fileparts(this_file);
project_root = fileparts(this_dir);

% Load workspace variables
run(fullfile(project_root, 'scripts', 'init_simulink_12agv_workspace.m'));

% Open the model
model_name = 'x12_agv';
model_path = fullfile(project_root, 'slimulink', [model_name, '.slx']);
load_system(model_path);

fprintf('Modifying %s...\n', model_path);

% Cooperative parameters
kc_pos = 0.08;
kc_vel = 0.10;
d_safe = 0.45;
k_rep = 2.4;

% Group topology: 3 groups x 4 AGVs, ring within each group
group_size = 4;
num_groups = 3;

% Build neighbor list (ring topology)
neighbors = cell(12, 1);
for g = 1:num_groups
    base = (g-1)*group_size;
    for r = 1:group_size
        i = base + r;
        left = base + mod(r-2, group_size) + 1;
        right = base + mod(r, group_size) + 1;
        neighbors{i} = [left, right];
    end
end

% For each AGV subsystem, add cooperative and repulsion logic
for i = 1:12
    agv_subsystem = sprintf('x12_agv/agv_%d', i);

    fprintf('Adding coop/repulsion to AGV %d...\n', i);

    % Add input ports for neighbor states (2 neighbors x 4 states = 8 inputs)
    % We'll add 2 input ports: p_neighbor1 (4-dim) and p_neighbor2 (4-dim)
    % Actually, let's add them as a single 8-dim input for simplicity

    % Add cooperative gain blocks
    % u_coop = -kc_pos * e_rel_p - kc_vel * e_rel_v
    % e_rel_p = (p_i - p_j) - (p_ref_i - p_ref_j)
    % e_rel_v = (v_i - v_j)

    % Add repulsion calculation
    % u_rep = k_rep * (d_safe - dij) * dir, if dij < d_safe

    % For now, let's add a MATLAB Function block that computes both
    % This is cleaner than adding many individual blocks

    % Add MATLAB Function block for cooperative + repulsion
    func_block = add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [agv_subsystem, '/coop_repulsion'], ...
        'Position', [800, 200, 950, 300]);

    % Set the function code
    set_param(func_block, 'MATLABFcn', sprintf([
        'function [u_coop, u_rep] = coop_repulsion(p_i, v_i, p_j1, v_j1, p_j2, v_j2, ', ...
        'p_ref_i, p_ref_j1, p_ref_j2, p_all, kc_pos, kc_vel, d_safe, k_rep)\n', ...
        '%% Cooperative term (ring topology, 2 neighbors)\n', ...
        'e_rel_p1 = (p_i - p_j1) - (p_ref_i - p_ref_j1);\n', ...
        'e_rel_v1 = (v_i - v_j1);\n', ...
        'e_rel_p2 = (p_i - p_j2) - (p_ref_i - p_ref_j2);\n', ...
        'e_rel_v2 = (v_i - v_j2);\n', ...
        'u_coop = -kc_pos * (e_rel_p1 + e_rel_p2) - kc_vel * (e_rel_v1 + e_rel_v2);\n', ...
        '\n', ...
        '%% Repulsion term (all other AGVs)\n', ...
        'u_rep = zeros(2,1);\n', ...
        'for j = 1:size(p_all,2)\n', ...
        '    dp = p_i - p_all(:,j);\n', ...
        '    dij = norm(dp);\n', ...
        '    if dij < d_safe && dij > 1e-6\n', ...
        '        dir = dp / dij;\n', ...
        '        u_rep = u_rep + k_rep * (d_safe - dij) * dir;\n', ...
        '    end\n', ...
        'end\n', ...
        'end\n'
    ]));

    % Add constant blocks for parameters
    kc_pos_block = add_block('simulink/Sources/Constant', ...
        [agv_subsystem, '/kc_pos'], ...
        'Position', [800, 100, 850, 130], ...
        'Value', num2str(kc_pos));

    kc_vel_block = add_block('simulink/Sources/Constant', ...
        [agv_subsystem, '/kc_vel'], ...
        'Position', [860, 100, 910, 130], ...
        'Value', num2str(kc_vel));

    d_safe_block = add_block('simulink/Sources/Constant', ...
        [agv_subsystem, '/d_safe'], ...
        'Position', [920, 100, 970, 130], ...
        'Value', num2str(d_safe));

    k_rep_block = add_block('simulink/Sources/Constant', ...
        [agv_subsystem, '/k_rep'], ...
        'Position', [980, 100, 1030, 130], ...
        'Value', num2str(k_rep));

    fprintf('  Added MATLAB Function block and constants\n');
end

% Save the modified model
save_system(model_path);
fprintf('Saved modified model to: %s\n', model_path);

% Close the model
close_system(model_name, 0);

fprintf('Done! Please open the model in Simulink to:\n');
fprintf('1. Add input ports for neighbor states\n');
fprintf('2. Connect neighbor outputs to inputs\n');
fprintf('3. Sum local + coop + repulsion terms\n');
fprintf('4. Add saturation block for u_max\n');
