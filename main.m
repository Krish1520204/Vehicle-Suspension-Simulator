%% =========================================================================
%  VEHICLE SUSPENSION SYSTEM SIMULATOR (MATLAB)
%  Phase 1: 4-DOF Half-Car Dynamic Model & Mathematical Foundation
% =========================================================================
%  Degrees of Freedom (4-DOF):
%    1. z_s   : Sprung mass heave displacement (body vertical motion at CG)
%    2. theta : Sprung mass pitch angular displacement
%    3. z_uf  : Front unsprung mass vertical displacement (front wheel)
%    4. z_ur  : Rear unsprung mass vertical displacement (rear wheel)
%
%  Features:
%    - Full 8-state state-space ODE formulation solved with ODE45
%    - Time delay between front and rear axle based on vehicle speed & wheelbase
%    - Comprehensive calculation of suspension forces, accelerations, and dynamic tyre loads
%    - Modal analysis (eigenvalues, damping ratios, natural frequencies)
%    - Publication-grade multi-panel engineering visualization
% =========================================================================

clear;
clc;
close all;

% Set up paths to project subfolders
proj_root = fileparts(mfilename('fullpath'));
addpath(fullfile(proj_root, 'config'));
addpath(fullfile(proj_root, 'models'));
addpath(fullfile(proj_root, 'road_profiles'));
addpath(fullfile(proj_root, 'simulation'));
addpath(fullfile(proj_root, 'utils'));

fprintf('========================================================================\n');
fprintf('       VEHICLE SUSPENSION SIMULATOR: 4-DOF HALF-CAR DYNAMICS            \n');
fprintf('========================================================================\n\n');

%% 1. Load Vehicle Parameters
% Preset options: 'sedan', 'suv', 'sport'
vehicle_type = 'sedan';
params = get_vehicle_params(vehicle_type);

fprintf('Vehicle Configuration: %s\n', upper(vehicle_type));
fprintf('  Sprung Mass (m_s)             : %.1f kg\n', params.m_s);
fprintf('  Pitch Moment of Inertia (I_y) : %.1f kg*m^2\n', params.I_y);
fprintf('  Front Unsprung Mass (m_uf)    : %.1f kg\n', params.m_uf);
fprintf('  Rear Unsprung Mass (m_ur)     : %.1f kg\n', params.m_ur);
fprintf('  Front Suspension Stiffness    : %.0f N/m\n', params.k_sf);
fprintf('  Rear Suspension Stiffness     : %.0f N/m\n', params.k_sr);
fprintf('  Front Suspension Damping      : %.0f N*s/m\n', params.c_sf);
fprintf('  Rear Suspension Damping       : %.0f N*s/m\n', params.c_sr);
fprintf('  Wheelbase (L = a + b)         : %.2f m (a=%.2f m, b=%.2f m)\n\n', ...
        params.L, params.a, params.b);

%% 2. Perform Mathematical Model Validation & Modal Analysis
modal = validate_model(params);

%% 3. Configure Road Profile Excitation
% Type options:
%   - Multi-Bump: 'half_sine', 'triangular', 'rectangular', or 'multi_bump'
%   - Single/Standard: 'bump', 'step', 'sine', 'iso8608'
road_cfg.type          = 'half_sine'; % Selectable: 'rectangular', 'triangular', 'half_sine', 'bump'
road_cfg.num_bumps     = 8;           % Number of consecutive bumps (7-10)
road_cfg.H             = 0.05;        % Bump height: 50 mm (0.05 m)
road_cfg.W             = 0.60;        % Bump length: 600 mm (0.6 m)
road_cfg.spacing       = 1.20;        % Gap between consecutive bumps (m)
road_cfg.t_start       = 0.40;        % First bump onset time for front axle (s)
road_cfg.v             = 12.5;        % Vehicle forward speed: 12.5 m/s (45 km/h)
road_cfg.smoothing_ratio = 0.08;      % Smooth C^1 transition fraction

wheelbase_delay = params.L / road_cfg.v;
fprintf('Road Scenario: %s Profile\n', upper(road_cfg.type));
fprintf('  Forward Speed (v)             : %.1f m/s (%.1f km/h)\n', ...
        road_cfg.v, road_cfg.v * 3.6);
fprintf('  Bump Height (H)               : %.1f mm\n', road_cfg.H * 1000);
fprintf('  Bump Length (W)               : %.1f mm\n', road_cfg.W * 1000);
if isfield(road_cfg, 'num_bumps')
    fprintf('  Consecutive Bumps (N)         : %d\n', road_cfg.num_bumps);
    fprintf('  Bump Spacing (Gap)            : %.2f m\n', road_cfg.spacing);
end
fprintf('  Wheelbase Time Delay (tau)    : %.4f s\n\n', wheelbase_delay);

%% 4. Define Integration Interval & Initial Conditions
tspan = [0, 3.0];             % Integration time window: 0 to 3.0 seconds
x0    = zeros(8, 1);          % System starts from static equilibrium

% Set solver tolerances
ode_opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', 1e-3);

%% 5. Execute ODE45 Integration
sim = run_simulation(params, road_cfg, tspan, x0, ode_opts);

%% 6. Display Simulation Results & Performance Metrics
m = sim.metrics;
fprintf('\n========================================================================\n');
fprintf('                 SIMULATION PERFORMANCE & RIDE METRICS                  \n');
fprintf('========================================================================\n');
fprintf(' Ride Comfort Metrics:\n');
fprintf('   - Sprung Mass Heave Accel (RMS) : %6.2f m/s^2\n', m.rms_heave_accel);
fprintf('   - Sprung Mass Heave Accel (Peak): %6.2f m/s^2\n', m.peak_heave_accel);
fprintf('   - Driver Position Accel (RMS)   : %6.2f m/s^2\n', m.rms_driver_accel);
fprintf('   - Driver Position Accel (Peak)  : %6.2f m/s^2\n', m.peak_driver_accel);
fprintf('   - Body Pitch Acceleration (RMS) : %6.2f rad/s^2\n', m.rms_pitch_accel);
fprintf('   - Body Pitch Acceleration (Peak): %6.2f rad/s^2\n', m.peak_pitch_accel);
fprintf('   - Max Pitch Angle               : %6.2f deg\n', m.peak_pitch_angle_deg);
fprintf('------------------------------------------------------------------------\n');
fprintf(' Suspension Working Space (Travel):\n');
fprintf('   - Front Peak Travel             : %6.1f mm\n', m.peak_susp_travel_f * 1000);
fprintf('   - Rear Peak Travel              : %6.1f mm\n', m.peak_susp_travel_r * 1000);
fprintf('   - Front Peak Suspension Force   : %6.2f kN\n', m.peak_susp_force_f / 1000);
fprintf('   - Rear Peak Suspension Force    : %6.2f kN\n', m.peak_susp_force_r / 1000);
fprintf('------------------------------------------------------------------------\n');
fprintf(' Road Holding & Ground Contact (Wheel Hop):\n');
fprintf('   - Front Minimum Tyre Load Ratio : %6.2f (Lift-off: %s)\n', ...
        m.min_tyre_ratio_f, string(m.wheel_lift_front));
fprintf('   - Rear Minimum Tyre Load Ratio  : %6.2f (Lift-off: %s)\n', ...
        m.min_tyre_ratio_r, string(m.wheel_lift_rear));
fprintf('========================================================================\n\n');

%% 7. Generate & Save Engineering Plots
fig = plot_simulation_results(sim, true);
set(fig, 'Color', 'k');
fprintf('Phase 1 mathematical model execution and validation complete.\n');
