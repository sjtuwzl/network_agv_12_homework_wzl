%% install_sedumi_release.m
% Download SeDuMi official release bundle with precompiled binaries.

clear; clc;

this_file = mfilename('fullpath');
this_dir = fileparts(this_file);
project_root = fileparts(this_dir);
third_party = fullfile(project_root, 'third_party');
if ~exist(third_party, 'dir')
    mkdir(third_party);
end

zip_file = fullfile(third_party, 'sedumi_release.zip');
dst_dir = fullfile(third_party, 'sedumi_release');

fprintf('Downloading SeDuMi release bundle...\n');
websave(zip_file, 'https://github.com/sqlp/sedumi/releases/latest/download/sedumi.zip', weboptions('Timeout', 120));

if exist(dst_dir, 'dir')
    rmdir(dst_dir, 's');
end
mkdir(dst_dir);
unzip(zip_file, dst_dir);

addpath(genpath(fullfile(dst_dir, 'sedumi')), '-begin');
fprintf('which sedumi: %s\n', which('sedumi'));
fprintf('which choltmpsiz: %s\n', which('choltmpsiz'));
fprintf('Installed SeDuMi release to: %s\n', dst_dir);
