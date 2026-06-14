%% install_yalmip_and_solvers.m
% One-click installer for YALMIP + SDP solvers (SDPT3, SeDuMi).
% It downloads zip packages into project_root/third_party and adds paths.

clear; clc;

this_file = mfilename('fullpath');
this_dir = fileparts(this_file);
project_root = fileparts(this_dir);
third_party = fullfile(project_root, 'third_party');

if ~exist(third_party, 'dir')
    mkdir(third_party);
end

fprintf('Project root: %s\n', project_root);
fprintf('Third-party dir: %s\n\n', third_party);

% Download sources (official public repos)
pkgs = { ...
    struct('name', 'yalmip', 'url', 'https://github.com/yalmip/YALMIP/archive/refs/heads/master.zip'), ...
    struct('name', 'sdpt3',  'url', 'https://github.com/sqlp/sdpt3/archive/refs/heads/master.zip'), ...
    struct('name', 'sedumi', 'url', 'https://github.com/sqlp/sedumi/archive/refs/heads/master.zip') ...
};

for i = 1:numel(pkgs)
    pkg = pkgs{i};
    fprintf('Downloading %s ...\n', pkg.name);
    zip_file = fullfile(third_party, [pkg.name, '.zip']);
    try
        websave(zip_file, pkg.url);
    catch ME
        warning('Failed to download %s: %s', pkg.name, ME.message);
        continue;
    end

    fprintf('Unzipping %s ...\n', pkg.name);
    out_dir = fullfile(third_party, pkg.name);
    if exist(out_dir, 'dir')
        rmdir(out_dir, 's');
    end
    mkdir(out_dir);
    unzip(zip_file, out_dir);

    % Most repos unzip to a single top-level folder; add all subpaths.
    addpath(genpath(out_dir), '-begin');
end

fprintf('\nPath check after installation:\n');
fprintf('which sdpvar: %s\n', which('sdpvar'));
fprintf('which optimize: %s\n', which('optimize'));
fprintf('which sdpsettings: %s\n', which('sdpsettings'));
fprintf('which sqlp: %s\n', which('sqlp'));       % sdpt3 marker
fprintf('which sedumi: %s\n', which('sedumi'));   % sedumi marker

fprintf('\nIf YALMIP is found, now run:\n');
fprintf('run(''scripts/run_agv_mss_demo.m'')\n');
