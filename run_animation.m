%% =========================================================================
%  RUN_ANIMATION.M
%  Runs the synchronized 2D physics-driven vehicle suspension animation
%  based on actual 4-DOF half-car ODE45 simulation results.
% =========================================================================

clear;
clc;
close all;

% Set up paths
proj_root = fileparts(mfilename('fullpath'));
addpath(fullfile(proj_root, 'config'));
addpath(fullfile(proj_root, 'models'));
addpath(fullfile(proj_root, 'road_profiles'));
addpath(fullfile(proj_root, 'simulation'));
addpath(fullfile(proj_root, 'utils'));
addpath(fullfile(proj_root, 'plotting'));
addpath(fullfile(proj_root, 'analysis'));
addpath(fullfile(proj_root, 'animation'));

fprintf('========================================================================\n');
fprintf('     VEHICLE SUSPENSION SIMULATOR: 2D PHYSICS-DRIVEN ANIMATION          \n');
fprintf('========================================================================\n\n');

%% 1. Configure Vehicle & Multi-Bump Road Scenario
params = get_vehicle_params('sedan');

% Road profile with multiple consecutive bumps
road_cfg.type          = 'half_sine'; % Selectable: 'half_sine', 'triangular', 'rectangular'
road_cfg.num_bumps     = 6;           % 6 consecutive bumps
road_cfg.H             = 0.055;       % Bump height: 55 mm (0.055 m)
road_cfg.W             = 0.65;        % Bump length: 650 mm (0.65 m)
road_cfg.spacing       = 1.35;        % Spacing between bumps: 1.35 m
road_cfg.t_start       = 0.35;        % Bump onset time: 0.35 s
road_cfg.v             = 10.0;        % Vehicle forward speed: 10 m/s (36 km/h)
road_cfg.smoothing_ratio = 0.08;

tspan = [0, 2.2];      % 2.2 seconds simulation window
x0    = zeros(8, 1);
ode_opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', 1e-3);

fprintf('Vehicle: %s Sedan (Wheelbase L = %.2f m)\n', upper(params.preset_name), params.L);
fprintf('Road Scenario: %s (%d Bumps, Height = %.1f mm, Speed = %.1f km/h)\n', ...
        upper(road_cfg.type), road_cfg.num_bumps, road_cfg.H * 1000, road_cfg.v * 3.6);
fprintf('Wheelbase Time Delay: %.4f s\n\n', params.L / road_cfg.v);

%% 2. Solve 4-DOF Half-Car Model with ODE45
fprintf('Running ODE45 numerical integration...\n');
sim = run_simulation(params, road_cfg, tspan, x0, ode_opts);

%% 3. Display KPIs & Performance Summary
kpi = calculate_kpis(sim);
print_performance_report(kpi);

%% 4. Configure & Launch Synchronized Animation
anim_cfg.playback_speed = 0.60; % 0.6x speed for clear observation of spring/damper motion
anim_cfg.fps            = 25;   % 25 frames per second
anim_cfg.save_gif       = true; % Save animated GIF
anim_cfg.gif_filename   = 'outputs/suspension_animation.gif';
anim_cfg.save_snapshot  = true; % Save high-res representative frame snapshot
anim_cfg.snapshot_name  = 'outputs/suspension_animation_frame.png';
anim_cfg.show_labels    = true; % Show CG, dimensions L_f, L_r, and pitch theta

fprintf('Launching animated simulation visualization...\n');
anim = animate_vehicle_suspension(sim, anim_cfg);

fprintf('\nAnimation completed successfully.\n');
fprintf('  - Animated GIF saved to: %s\n', anim.gif_path);
fprintf('  - Snapshot frame saved to: %s\n', anim.snapshot_path);
