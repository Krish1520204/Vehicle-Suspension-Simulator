%% =========================================================================
%  RUN_PARAMETER_STUDY.M
%  Demonstrates dynamic updating of all 7 simulation graphs and numerical KPIs
%  when vehicle and road parameters change using actual ODE45 integration.
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

fprintf('========================================================================\n');
fprintf('       PARAMETER VARIATION STUDY & DYNAMIC GRAPH UPDATING               \n');
fprintf('========================================================================\n\n');

%% Study 1: Effect of Suspension Damping (Soft vs Nominal vs Stiff)
fprintf('--- STUDY 1: Suspension Damping Variation ---\n');
base_params = get_vehicle_params('sedan');

road_cfg.type          = 'half_sine';
road_cfg.num_bumps     = 4;
road_cfg.H             = 0.05;  % 50 mm bump
road_cfg.W             = 0.60;  % 600 mm width
road_cfg.spacing       = 1.40;  % 1.4 m gap
road_cfg.t_start       = 0.30;
road_cfg.v             = 12.5;  % 45 km/h
road_cfg.smoothing_ratio = 0.08;

tspan = [0, 2.5];
x0    = zeros(8, 1);
ode_opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', 1e-3);

damping_scales = [0.5, 1.0, 1.6];
damping_labels = {'Soft / Under-damped (0.5x)', 'Nominal Sedan (1.0x)', 'Sport / Firm (1.6x)'};
damping_colors = {[0.85, 0.33, 0.10], [0.00, 0.45, 0.74], [0.47, 0.67, 0.19]};

sim_damping = cell(length(damping_scales), 1);
kpi_damping = cell(length(damping_scales), 1);

for i = 1:length(damping_scales)
    p_mod = base_params;
    p_mod.c_sf = base_params.c_sf * damping_scales(i);
    p_mod.c_sr = base_params.c_sr * damping_scales(i);

    fprintf('  Simulating %s (c_sf = %.0f N*s/m, c_sr = %.0f N*s/m)...\n', ...
            damping_labels{i}, p_mod.c_sf, p_mod.c_sr);

    sim_damping{i} = run_simulation(p_mod, road_cfg, tspan, x0, ode_opts);
    kpi_damping{i} = calculate_kpis(sim_damping{i});
end

%% Display Study 1 Numerical KPI Comparison
fprintf('\n========================================================================================\n');
fprintf('              NUMERICAL PERFORMANCE INDICATORS: DAMPING VARIATION                       \n');
fprintf('========================================================================================\n');
fprintf(' %-30s | %-16s | %-16s | %-16s\n', ...
        'Performance Indicator', damping_labels{1}, damping_labels{2}, damping_labels{3});
fprintf('----------------------------------------------------------------------------------------\n');
fprintf(' %-30s | %13.2f mm  | %13.2f mm  | %13.2f mm \n', ...
        'Maximum Body Displacement', kpi_damping{1}.max_body_disp_mm, kpi_damping{2}.max_body_disp_mm, kpi_damping{3}.max_body_disp_mm);
fprintf(' %-30s | %13.2f m/s^2| %13.2f m/s^2| %13.2f m/s^2\n', ...
        'Maximum Vertical Accel (CG)', kpi_damping{1}.max_heave_accel_mps2, kpi_damping{2}.max_heave_accel_mps2, kpi_damping{3}.max_heave_accel_mps2);
fprintf(' %-30s | %13.2f g   | %13.2f g   | %13.2f g  \n', ...
        'Maximum Acceleration (g)', kpi_damping{1}.max_heave_accel_g, kpi_damping{2}.max_heave_accel_g, kpi_damping{3}.max_heave_accel_g);
fprintf(' %-30s | %13.2f m/s^2| %13.2f m/s^2| %13.2f m/s^2\n', ...
        'RMS Heave Acceleration', kpi_damping{1}.rms_heave_accel_mps2, kpi_damping{2}.rms_heave_accel_mps2, kpi_damping{3}.rms_heave_accel_mps2);
fprintf(' %-30s | %13.2f deg  | %13.2f deg  | %13.2f deg \n', ...
        'Maximum Body Pitch Angle', kpi_damping{1}.max_pitch_angle_deg, kpi_damping{2}.max_pitch_angle_deg, kpi_damping{3}.max_pitch_angle_deg);
fprintf(' %-30s | %13.1f mm  | %13.1f mm  | %13.1f mm \n', ...
        'Front Maximum Travel', kpi_damping{1}.max_travel_front_mm, kpi_damping{2}.max_travel_front_mm, kpi_damping{3}.max_travel_front_mm);
fprintf(' %-30s | %13.1f mm  | %13.1f mm  | %13.1f mm \n', ...
        'Rear Maximum Travel', kpi_damping{1}.max_travel_rear_mm, kpi_damping{2}.max_travel_rear_mm, kpi_damping{3}.max_travel_rear_mm);
fprintf(' %-30s | %13.2f kN  | %13.2f kN  | %13.2f kN \n', ...
        'Peak Front Susp Force', kpi_damping{1}.max_susp_force_front_kN, kpi_damping{2}.max_susp_force_front_kN, kpi_damping{3}.max_susp_force_front_kN);
fprintf('========================================================================================\n\n');

%% Plot Study 1 Comparison Figure
fig_damping = figure('Name', 'Damping Variation Sensitivity Study', ...
                     'NumberTitle', 'off', 'Color', [0 0 0], 'Position', [100, 50, 1300, 850]);

% 1. Body Vertical Displacement
subplot(3, 2, 1); hold on;
for i = 1:3
    plot(sim_damping{i}.t, sim_damping{i}.z_s * 1000, 'Color', damping_colors{i}, 'LineWidth', 1.8, 'DisplayName', damping_labels{i});
end
grid on; ylabel('Displacement (mm)', 'FontWeight', 'bold');
title('Vehicle Body Vertical Displacement (Heave)', 'FontWeight', 'bold');
legend('Location', 'northeast'); xlim([0, 2.5]);

% 2. Body Pitch Angle
subplot(3, 2, 2); hold on;
for i = 1:3
    plot(sim_damping{i}.t, sim_damping{i}.theta_deg, 'Color', damping_colors{i}, 'LineWidth', 1.8, 'DisplayName', damping_labels{i});
end
grid on; ylabel('Pitch Angle (deg)', 'FontWeight', 'bold');
title('Vehicle Body Pitch Angle', 'FontWeight', 'bold');
legend('Location', 'northeast'); xlim([0, 2.5]);

% 3. Front Suspension Deflection
subplot(3, 2, 3); hold on;
for i = 1:3
    plot(sim_damping{i}.t, sim_damping{i}.susp_def_f * 1000, 'Color', damping_colors{i}, 'LineWidth', 1.8, 'DisplayName', damping_labels{i});
end
yline(80, 'k--', 'LineWidth', 1, 'DisplayName', 'Bumpstop Limit (80 mm)');
grid on; ylabel('Deflection (mm)', 'FontWeight', 'bold');
title('Front Suspension Deflection', 'FontWeight', 'bold');
legend('Location', 'northeast'); xlim([0, 2.5]);

% 4. Rear Suspension Deflection
subplot(3, 2, 4); hold on;
for i = 1:3
    plot(sim_damping{i}.t, sim_damping{i}.susp_def_r * 1000, 'Color', damping_colors{i}, 'LineWidth', 1.8, 'DisplayName', damping_labels{i});
end
yline(80, 'k--', 'LineWidth', 1, 'DisplayName', 'Bumpstop Limit (80 mm)');
grid on; ylabel('Deflection (mm)', 'FontWeight', 'bold');
title('Rear Suspension Deflection', 'FontWeight', 'bold');
legend('Location', 'northeast'); xlim([0, 2.5]);

% 5. Body Vertical Acceleration
subplot(3, 2, 5); hold on;
for i = 1:3
    plot(sim_damping{i}.t, sim_damping{i}.ddz_s, 'Color', damping_colors{i}, 'LineWidth', 1.8, 'DisplayName', damping_labels{i});
end
grid on; xlabel('Time (s)', 'FontWeight', 'bold'); ylabel('Accel (m/s^2)', 'FontWeight', 'bold');
title('Vehicle Body Vertical Acceleration', 'FontWeight', 'bold');
legend('Location', 'northeast'); xlim([0, 2.5]);

% 6. Front Wheel Displacement
subplot(3, 2, 6); hold on;
for i = 1:3
    plot(sim_damping{i}.t, sim_damping{i}.z_uf * 1000, 'Color', damping_colors{i}, 'LineWidth', 1.8, 'DisplayName', damping_labels{i});
end
grid on; xlabel('Time (s)', 'FontWeight', 'bold'); ylabel('Wheel Displ (mm)', 'FontWeight', 'bold');
title('Front Wheel Displacement', 'FontWeight', 'bold');
legend('Location', 'northeast'); xlim([0, 2.5]);

out_dir = fullfile(proj_root, 'outputs');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end
save_path_damp = fullfile(out_dir, 'parameter_variation_damping.png');
try
    exportgraphics(fig_damping, save_path_damp, 'Resolution', 300);
catch
    saveas(fig_damping, save_path_damp);
end
fprintf('Saved damping variation sensitivity plot to: %s\n', save_path_damp);

%% Study 2: Generate Full 7-Graph Analysis on Nominal Sedan
fprintf('\n--- Generating Complete 7-Graph Analysis Module for Nominal Setup ---\n');
fig_analysis = plot_suspension_analysis(sim_damping{2}, true, 'suspension_analysis_module.png');
print_performance_report(kpi_damping{2});

fprintf('Parameter study and simulation analysis module execution complete.\n');
