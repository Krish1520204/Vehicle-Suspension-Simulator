%% =========================================================================
%  RUN_DASHBOARD.M
%  Launches the interactive MATLAB Vehicle Suspension Simulator &
%  Bump Analyzer Dashboard.
% =========================================================================

clear;
clc;
close all;

% Set up project directory paths
proj_root = fileparts(mfilename('fullpath'));
addpath(fullfile(proj_root, 'config'));
addpath(fullfile(proj_root, 'models'));
addpath(fullfile(proj_root, 'road_profiles'));
addpath(fullfile(proj_root, 'simulation'));
addpath(fullfile(proj_root, 'utils'));
addpath(fullfile(proj_root, 'plotting'));
addpath(fullfile(proj_root, 'analysis'));
addpath(fullfile(proj_root, 'animation'));
addpath(fullfile(proj_root, 'dashboard'));

fprintf('========================================================================\n');
fprintf('     VEHICLE SUSPENSION SIMULATOR: INTERACTIVE BUMP ANALYZER DASHBOARD   \n');
fprintf('========================================================================\n\n');
fprintf('Launching interactive UI Dashboard...\n');

fig = suspension_dashboard();

fprintf('Dashboard launched successfully.\n');
