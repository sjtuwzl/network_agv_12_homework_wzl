function setup_solver_paths(project_root)
%SETUP_SOLVER_PATHS Try to add common YALMIP/solver folders.
%
% Priority:
% 1) project_root/third_party/yalmip
% 2) project_root/third_party/sedumi_release/sedumi (precompiled release)
% 3) project_root/third_party/sedumi
% 4) project_root/third_party/sdpt3 (only if mex binaries exist)

    if nargin < 1 || isempty(project_root)
        project_root = pwd;
    end

    yalmip_dir = fullfile(project_root, 'third_party', 'yalmip');
    sedumi_release_dir = fullfile(project_root, 'third_party', 'sedumi_release', 'sedumi');
    sedumi_dir = fullfile(project_root, 'third_party', 'sedumi');
    sdpt3_dir = fullfile(project_root, 'third_party', 'sdpt3');

    if exist(yalmip_dir, 'dir')
        addpath(genpath(yalmip_dir), '-begin');
    end

    % Prefer release bundle with precompiled binaries.
    if exist(sedumi_release_dir, 'dir')
        addpath(genpath(sedumi_release_dir), '-begin');
    elseif exist(sedumi_dir, 'dir')
        addpath(genpath(sedumi_dir), '-begin');
    end

    % Add SDPT3 only when compiled binaries are available.
    mexmat_candidates = { ...
        fullfile(sdpt3_dir, 'sdpt3-master', 'Solver', 'Mexfun', ['mexmat.', mexext]), ...
        fullfile(sdpt3_dir, 'Solver', 'Mexfun', ['mexmat.', mexext]) ...
    };
    has_sdpt3_mex = false;
    for i = 1:numel(mexmat_candidates)
        if exist(mexmat_candidates{i}, 'file')
            has_sdpt3_mex = true;
            break;
        end
    end
    if exist(sdpt3_dir, 'dir') && has_sdpt3_mex
        addpath(genpath(sdpt3_dir), '-begin');
    end
end
