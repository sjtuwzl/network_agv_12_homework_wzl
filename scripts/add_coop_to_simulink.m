%% add_coop_to_simulink.m
% Add cooperative and repulsion logic to x12_agv.slx programmatically
% Run this AFTER opening the model in Simulink

clear; clc;

% Cooperative parameters
KC_POS = 0.08;
KC_VEL = 0.10;
D_SAFE = 0.45;
K_REP = 2.4;
U_MAX = 1.0;

% Group topology: 3 groups x 4 AGVs, ring within each group
GROUP_SIZE = 4;
NUM_GROUPS = 3;

% Build neighbor list (ring topology)
neighbors = cell(12, 1);
for g = 1:NUM_GROUPS
    base = (g-1)*GROUP_SIZE;
    for r = 1:GROUP_SIZE
        i = base + r;
        left = base + mod(r-2, GROUP_SIZE) + 1;
        right = base + mod(r, GROUP_SIZE) + 1;
        neighbors{i} = [left, right];
    end
end

fprintf('Neighbor topology:\n');
for i = 1:12
    fprintf('  AGV %d: neighbors [%d, %d]\n', i, neighbors{i}(1), neighbors{i}(2));
end

% Open the model
model_name = 'x12_agv';
if ~bd_isloaded(model_name)
    load_system(fullfile(fileparts(mfilename('fullpath')), '..', 'slimulink', [model_name, '.slx']));
end

% For each AGV subsystem, add cooperative/repulsion blocks
for i = 1:12
    agv_subsystem = sprintf('%s/agv_%d', model_name, i);

    fprintf('\nAdding coop/repulsion to AGV %d...\n', i);

    % Get current position of the subsystem
    pos = get_param(agv_subsystem, 'Position');

    % --- Add MATLAB Function block for coop + repulsion ---
    func_block = add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [agv_subsystem, '/coop_repulsion'], ...
        'Position', [pos(3)-200, pos(2)+50, pos(3)-50, pos(2)+150]);

    % Set the function code
    set_param(func_block, 'MATLABFcn', sprintf([
        'function [u_coop, u_rep] = coop_repulsion(p_i, v_i, p_j1, v_j1, p_j2, v_j2, ', ...\n'
        'p_ref_i, p_ref_j1, p_ref_j2, kc_pos, kc_vel, d_safe, k_rep)\n'
        '%% Cooperative term (ring topology, 2 neighbors)\n'
        'e_rel_p1 = (p_i - p_j1) - (p_ref_i - p_ref_j1);\n'
        'e_rel_v1 = (v_i - v_j1);\n'
        'e_rel_p2 = (p_i - p_j2) - (p_ref_i - p_ref_j2);\n'
        'e_rel_v2 = (v_i - v_j2);\n'
        'u_coop = -kc_pos * (e_rel_p1 + e_rel_p2) - kc_vel * (e_rel_v1 + e_rel_v2);\n'
        '\n'
        '%% Repulsion term (all other AGVs - simplified: just neighbors for now)\n'
        'u_rep = zeros(2,1);\n'
        'for j = 1:2\n'
        '    if j == 1\n'
        '        dp = p_i - p_j1;\n'
        '    else\n'
        '        dp = p_i - p_j2;\n'
        '    end\n'
        '    dij = norm(dp);\n'
        '    if dij < d_safe && dij > 1e-6\n'
        '        dir = dp / dij;\n'
        '        u_rep = u_rep + k_rep * (d_safe - dij) * dir;\n'
        '    end\n'
        'end\n'
        'end\n'
    ]));

    % Add constant blocks for parameters
    param_blocks = {'kc_pos', num2str(KC_POS);
                    'kc_vel', num2str(KC_VEL);
                    'd_safe', num2str(D_SAFE);
                    'k_rep', num2str(K_REP)};

    for pb = 1:size(param_blocks, 1)
        const_block = add_block('simulink/Sources/Constant', ...
            [agv_subsystem, '/', param_blocks{pb, 1}], ...
            'Position', [pos(3)-250+pb*30, pos(2)-20, pos(3)-220+pb*30, pos(2)+10], ...
            'Value', param_blocks{pb, 2});
    end

    % Add saturation block for u_max
    sat_block = add_block('simulink/Discontinuities/Saturation', ...
        [agv_subsystem, '/u_saturation'], ...
        'Position', [pos(3)-100, pos(2)+150, pos(3)-50, pos(2)+190], ...
        'UpperLimit', sprintf('[%f; %f]', U_MAX, U_MAX), ...
        'LowerLimit', sprintf('[-%f; -%f]', U_MAX, U_MAX));

    % Add sum block for u_local + u_coop + u_rep
    sum_block = add_block('simulink/Math Operations/Sum', ...
        [agv_subsystem, '/sum_coop'], ...
        'Position', [pos(3)-250, pos(2)+150, pos(3)-220, pos(2)+180], ...
        'Inputs', '|+++');

    fprintf('  Added blocks to %s\n', agv_subsystem);
end

% Save the modified model
save_system(model_name);
fprintf('\nSaved modified model. Please open in Simulink to:\n');
fprintf('1. Add input ports for neighbor states to each AGV subsystem\n');
fprintf('2. Connect neighbor outputs to the new inputs\n');
fprintf('3. Wire sum_coop output to the plant input\n');
fprintf('4. Test the model\n');
