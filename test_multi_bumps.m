%% =========================================================================
%  TEST_MULTI_BUMPS.M
%  Validation & Comparative Analysis of 3 Continuous-Time Multi-Bump Profiles
%    1. Rectangular Multi-Bump
%    2. Triangular Multi-Bump
%    3. Half-Sinusoidal Multi-Bump
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

fprintf('========================================================================\n');
fprintf('     MULTI-BUMP CONTINUOUS-TIME ROAD PROFILES & 4-DOF INTEGRATION       \n');
fprintf('========================================================================\n\n');

%% 1. Vehicle Configuration
params = get_vehicle_params('sedan');
fprintf('Vehicle Configuration: %s (Wheelbase L = %.2f m)\n\n', ...
        upper(params.preset_name), params.L);

%% 2. Common Multi-Bump Road Parameters
base_cfg.num_bumps       = 8;      % 8 consecutive bumps (in 7-10 range)
base_cfg.H               = 0.05;   % Bump height: 50 mm (0.05 m)
base_cfg.W               = 0.60;   % Bump width: 600 mm (0.6 m)
base_cfg.spacing         = 1.20;   % Gap between bumps: 1.2 m
base_cfg.t_start         = 0.40;   % First bump onset time for front wheel: 0.4 s
base_cfg.v               = 12.5;   % Vehicle speed: 12.5 m/s (45 km/h)
base_cfg.smoothing_ratio = 0.08;   % 8% transition smoothing for C^1 continuity

pitch = base_cfg.W + base_cfg.spacing; % 1.8 m pitch
tau   = params.L / base_cfg.v;         % Rear-wheel delay (s)

fprintf('Road Excitation Parameters:\n');
fprintf('  Number of Consecutive Bumps : %d\n', base_cfg.num_bumps);
fprintf('  Bump Amplitude (H)          : %.1f mm\n', base_cfg.H * 1000);
fprintf('  Bump Length (W)             : %.1f mm\n', base_cfg.W * 1000);
fprintf('  Bump Spacing / Gap          : %.2f m\n', base_cfg.spacing);
fprintf('  Pitch (Length + Spacing)    : %.2f m\n', pitch);
fprintf('  Vehicle Forward Speed (v)   : %.1f m/s (%.1f km/h)\n', ...
        base_cfg.v, base_cfg.v * 3.6);
fprintf('  Wheelbase Time Delay (tau)  : %.4f s (%.1f ms)\n\n', tau, tau * 1000);

%% 3. Display Bump Encounter Timetable
[~, ~, ~, ~, info] = road_multi_bump(0, base_cfg, params);
fprintf('========================================================================\n');
fprintf('                  BUMP ENCOUNTER TIMETABLE (FRONT vs REAR)              \n');
fprintf('========================================================================\n');
fprintf(' %-5s | %-12s | %-12s | %-12s | %-12s\n', ...
        'Bump', 'Front Enter', 'Front Exit', 'Rear Enter', 'Rear Exit');
fprintf('------------------------------------------------------------------------\n');
for k = 1:base_cfg.num_bumps
    fprintf('  #%-4d|   %6.3f s   |   %6.3f s   |   %6.3f s   |   %6.3f s\n', ...
            k, info.t_f_enter(k), info.t_f_exit(k), info.t_r_enter(k), info.t_r_exit(k));
end
fprintf('========================================================================\n\n');

%% 4. Define Profiles & Simulation Settings
profile_types = {'rectangular', 'triangular', 'half_sine'};
profile_labels = {'Rectangular Multi-Bump', 'Triangular Multi-Bump', 'Half-Sinusoidal Multi-Bump'};

tspan = [0, 3.0];    % 0 to 3 seconds simulation time
x0    = zeros(8, 1); % Initial equilibrium state
ode_opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', 1e-3);

sim_results = cell(length(profile_types), 1);

%% 5. Execute Simulation for All 3 Road Profiles
fprintf('Running simulations for all 3 continuous-time multi-bump profiles...\n');
for p_idx = 1:length(profile_types)
    p_type = profile_types{p_idx};
    cfg = base_cfg;
    cfg.type = p_type;
    cfg.profile_shape = p_type;

    fprintf('  [%d/3] Simulating %s...', p_idx, profile_labels{p_idx});
    sim_results{p_idx} = run_simulation(params, cfg, tspan, x0, ode_opts);
    sim_results{p_idx}.label = profile_labels{p_idx};
end
fprintf('All 3 simulations completed successfully.\n\n');

%% 6. Comparative Performance Metrics Table
fprintf('========================================================================================\n');
fprintf('                 COMPARATIVE SUSPENSION PERFORMANCE & RIDE METRICS                      \n');
fprintf('========================================================================================\n');
fprintf(' %-28s | %-16s | %-16s | %-16s\n', ...
        'Metric', 'Rectangular', 'Triangular', 'Half-Sinusoidal');
fprintf('----------------------------------------------------------------------------------------\n');

m_rect = sim_results{1}.metrics;
m_tri  = sim_results{2}.metrics;
m_sine = sim_results{3}.metrics;

fprintf(' %-28s | %13.2f    | %13.2f    | %13.2f   \n', ...
        'RMS Heave Accel (m/s^2)', m_rect.rms_heave_accel, m_tri.rms_heave_accel, m_sine.rms_heave_accel);
fprintf(' %-28s | %13.2f    | %13.2f    | %13.2f   \n', ...
        'Peak Heave Accel (m/s^2)', m_rect.peak_heave_accel, m_tri.peak_heave_accel, m_sine.peak_heave_accel);
fprintf(' %-28s | %13.2f    | %13.2f    | %13.2f   \n', ...
        'RMS Driver Accel (m/s^2)', m_rect.rms_driver_accel, m_tri.rms_driver_accel, m_sine.rms_driver_accel);
fprintf(' %-28s | %13.2f    | %13.2f    | %13.2f   \n', ...
        'Peak Driver Accel (m/s^2)', m_rect.peak_driver_accel, m_tri.peak_driver_accel, m_sine.peak_driver_accel);
fprintf(' %-28s | %13.2f    | %13.2f    | %13.2f   \n', ...
        'RMS Pitch Accel (rad/s^2)', m_rect.rms_pitch_accel, m_tri.rms_pitch_accel, m_sine.rms_pitch_accel);
fprintf(' %-28s | %13.2f    | %13.2f    | %13.2f   \n', ...
        'Peak Pitch Accel (rad/s^2)', m_rect.peak_pitch_accel, m_tri.peak_pitch_accel, m_sine.peak_pitch_accel);
fprintf(' %-28s | %13.2f    | %13.2f    | %13.2f   \n', ...
        'Peak Pitch Angle (deg)', m_rect.peak_pitch_angle_deg, m_tri.peak_pitch_angle_deg, m_sine.peak_pitch_angle_deg);
fprintf('----------------------------------------------------------------------------------------\n');
fprintf(' %-28s | %13.1f    | %13.1f    | %13.1f   \n', ...
        'Front Peak Travel (mm)', m_rect.peak_susp_travel_f * 1000, m_tri.peak_susp_travel_f * 1000, m_sine.peak_susp_travel_f * 1000);
fprintf(' %-28s | %13.1f    | %13.1f    | %13.1f   \n', ...
        'Rear Peak Travel (mm)', m_rect.peak_susp_travel_r * 1000, m_tri.peak_susp_travel_r * 1000, m_sine.peak_susp_travel_r * 1000);
fprintf(' %-28s | %13.2f    | %13.2f    | %13.2f   \n', ...
        'Front Peak Force (kN)', m_rect.peak_susp_force_f / 1000, m_tri.peak_susp_force_f / 1000, m_sine.peak_susp_force_f / 1000);
fprintf(' %-28s | %13.2f    | %13.2f    | %13.2f   \n', ...
        'Rear Peak Force (kN)', m_rect.peak_susp_force_r / 1000, m_tri.peak_susp_force_r / 1000, m_sine.peak_susp_force_r / 1000);
fprintf('----------------------------------------------------------------------------------------\n');
fprintf(' %-28s | %13.2f    | %13.2f    | %13.2f   \n', ...
        'Front Min Tyre Ratio (-)', m_rect.min_tyre_ratio_f, m_tri.min_tyre_ratio_f, m_sine.min_tyre_ratio_f);
fprintf(' %-28s | %13.2f    | %13.2f    | %13.2f   \n', ...
        'Rear Min Tyre Ratio (-)', m_rect.min_tyre_ratio_r, m_tri.min_tyre_ratio_r, m_sine.min_tyre_ratio_r);
fprintf('========================================================================================\n\n');

%% 7. Generate Figure 1: Road Input Profiles Comparison (Front vs Rear)
t_eval = linspace(0, 3.0, 3000);
c_blue = [0.00, 0.45, 0.74];
c_red  = [0.85, 0.33, 0.10];

fig1 = figure('Name', 'Multi-Bump Road Profiles Comparison', ...
              'NumberTitle', 'off', ...
              'Color', [0 0 0], ...
              'Position', [80, 80, 1100, 750]);

for p_idx = 1:length(profile_types)
    p_type = profile_types{p_idx};
    cfg = base_cfg;
    cfg.profile_shape = p_type;
    [z0f_ev, dz0f_ev, z0r_ev, dz0r_ev] = road_multi_bump(t_eval, cfg, params);

    subplot(3, 1, p_idx);
    plot(t_eval, z0f_ev * 1000, 'Color', c_blue, 'LineWidth', 1.8, 'DisplayName', 'Front Wheel z_{0f}');
    hold on;
    plot(t_eval, z0r_ev * 1000, '--', 'Color', c_red, 'LineWidth', 1.6, 'DisplayName', sprintf('Rear Wheel z_{0r} (\\tau = %.3f s)', tau));
    grid on;
    ylabel('Elevation (mm)');
    xlim([0, 2.5]);
    ylim([-5, 60]);
    title(sprintf('Profile %d: %s (8 Bumps, H = %.0f mm, W = %.0f mm, Gap = %.1f m, v = %.1f km/h)', ...
          p_idx, profile_labels{p_idx}, base_cfg.H * 1000, base_cfg.W * 1000, base_cfg.spacing, base_cfg.v * 3.6));
    legend('Location', 'northeast');
end
xlabel('Time (s)');

% Save Figure 1
out_dir = fullfile(proj_root, 'outputs');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end
save_path1 = fullfile(out_dir, 'road_profiles_comparison.png');
try
    exportgraphics(fig1, save_path1, 'Resolution', 300);
catch
    saveas(fig1, save_path1);
end
fprintf('Saved road profiles comparison plot to: %s\n', save_path1);

%% 8. Generate Figure 2: Vehicle Dynamic Response Comparison
c_palette = {[0.00, 0.45, 0.74], [0.85, 0.33, 0.10], [0.47, 0.67, 0.19]}; % Blue, Red, Green

fig2 = figure('Name', '4-DOF Response Comparison: Rectangular vs Triangular vs Half-Sine', ...
              'NumberTitle', 'off', ...
              'Color', [0 0 0], ...
              'Position', [120, 60, 1250, 800]);

% 1. Sprung Mass Heave Displacement
subplot(3, 2, 1);
hold on;
for p_idx = 1:3
    res = sim_results{p_idx};
    plot(res.t, res.z_s * 1000, 'Color', c_palette{p_idx}, 'LineWidth', 1.6, 'DisplayName', res.label);
end
grid on;
ylabel('Heave z_s (mm)');
title('Sprung Mass CG Heave Displacement');
legend('Location', 'northeast');
xlim([0, 2.5]);

% 2. Body Pitch Angle
subplot(3, 2, 2);
hold on;
for p_idx = 1:3
    res = sim_results{p_idx};
    plot(res.t, res.theta_deg, 'Color', c_palette{p_idx}, 'LineWidth', 1.6, 'DisplayName', res.label);
end
grid on;
ylabel('Pitch Angle (deg)');
title('Sprung Mass Body Pitch Angle');
legend('Location', 'northeast');
xlim([0, 2.5]);

% 3. Sprung Mass Heave Acceleration
subplot(3, 2, 3);
hold on;
for p_idx = 1:3
    res = sim_results{p_idx};
    plot(res.t, res.ddz_s, 'Color', c_palette{p_idx}, 'LineWidth', 1.5, 'DisplayName', res.label);
end
grid on;
ylabel('Accel (m/s^2)');
title('Sprung Mass Heave Acceleration (CG)');
legend('Location', 'northeast');
xlim([0, 2.5]);

% 4. Driver Position Acceleration
subplot(3, 2, 4);
hold on;
for p_idx = 1:3
    res = sim_results{p_idx};
    plot(res.t, res.ddz_driver, 'Color', c_palette{p_idx}, 'LineWidth', 1.5, 'DisplayName', res.label);
end
grid on;
ylabel('Accel (m/s^2)');
title('Driver Position Vertical Acceleration');
legend('Location', 'northeast');
xlim([0, 2.5]);

% 5. Front Suspension Deflection (Working Space)
subplot(3, 2, 5);
hold on;
for p_idx = 1:3
    res = sim_results{p_idx};
    plot(res.t, res.susp_def_f * 1000, 'Color', c_palette{p_idx}, 'LineWidth', 1.5, 'DisplayName', res.label);
end
yline(80, 'k--', 'LineWidth', 1, 'DisplayName', 'Jounce Limit');
yline(-80, 'k--', 'LineWidth', 1, 'DisplayName', 'Rebound Limit');
grid on;
xlabel('Time (s)');
ylabel('Travel (mm)');
title('Front Suspension Travel (Working Space)');
legend('Location', 'northeast');
xlim([0, 2.5]);

% 6. Front Tyre Normal Contact Load
subplot(3, 2, 6);
hold on;
for p_idx = 1:3
    res = sim_results{p_idx};
    plot(res.t, res.F_z_tf / 1000, 'Color', c_palette{p_idx}, 'LineWidth', 1.5, 'DisplayName', res.label);
end
yline(params.W_tf_static / 1000, 'k:', 'LineWidth', 1.2, 'DisplayName', 'Static Normal Load');
yline(0, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Lift-off (Loss of Grip)');
grid on;
xlabel('Time (s)');
ylabel('Load (kN)');
title('Front Tyre Total Normal Contact Load');
legend('Location', 'northeast');
xlim([0, 2.5]);

% Save Figure 2
save_path2 = fullfile(out_dir, 'multi_bump_simulation_comparison.png');
try
    exportgraphics(fig2, save_path2, 'Resolution', 300);
catch
    saveas(fig2, save_path2);
end
fprintf('Saved multi-bump dynamic response comparison plot to: %s\n', save_path2);

fprintf('\nMulti-bump test execution and validation complete.\n');
