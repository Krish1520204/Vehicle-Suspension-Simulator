function kpi = calculate_kpis(sim)
% CALCULATE_KPIS Extracts and formats comprehensive numerical performance indicators (KPIs)
% from actual ODE45 simulation results.
%
% Indicators calculated:
%   - Maximum body vertical displacement (heave at CG, front body, rear body)
%   - Maximum body vertical acceleration (CG and driver position) in m/s^2 and g
%   - Maximum suspension deflection / travel (front and rear) in mm
%   - Bumpstop travel margin and percent utilization (relative to 80 mm limit)
%   - Maximum body pitch angle (deg) and pitch angular acceleration (rad/s^2)
%   - RMS accelerations and ISO 2631-1 ride comfort classification
%   - Dynamic tyre contact ratios and ground contact retention assessment
%
% Input:
%   sim - Simulation results structure from run_simulation.m
%
% Output:
%   kpi - Struct containing formatted numerical performance indicators

    m = sim.metrics;
    p = sim.params;
    r = sim.road_cfg;
    g = 9.81;

    kpi.vehicle_type = p.preset_name;
    kpi.road_type    = r.type;
    kpi.speed_mps    = r.v;
    kpi.speed_kmh    = r.v * 3.6;

    % 1. Body Displacements
    kpi.max_body_disp_mm       = m.peak_body_disp * 1000;
    kpi.max_front_body_disp_mm = m.peak_body_disp_f * 1000;
    kpi.max_rear_body_disp_mm  = m.peak_body_disp_r * 1000;
    kpi.rms_body_disp_mm       = m.rms_body_disp * 1000;

    % 2. Body Accelerations
    kpi.max_heave_accel_mps2   = m.peak_heave_accel;
    kpi.max_heave_accel_g      = m.peak_heave_accel / g;
    kpi.rms_heave_accel_mps2   = m.rms_heave_accel;
    kpi.max_driver_accel_mps2  = m.peak_driver_accel;
    kpi.max_driver_accel_g     = m.peak_driver_accel / g;
    kpi.rms_driver_accel_mps2  = m.rms_driver_accel;

    % ISO 2631-1 Comfort Classification
    a_rms = m.rms_driver_accel;
    if a_rms < 0.315
        kpi.iso2631_comfort = 'Not uncomfortable';
    elseif a_rms < 0.63
        kpi.iso2631_comfort = 'A little uncomfortable';
    elseif a_rms < 1.00
        kpi.iso2631_comfort = 'Fairly uncomfortable';
    elseif a_rms < 1.60
        kpi.iso2631_comfort = 'Uncomfortable';
    elseif a_rms < 2.50
        kpi.iso2631_comfort = 'Very uncomfortable';
    else
        kpi.iso2631_comfort = 'Extremely uncomfortable';
    end

    % 3. Pitch Dynamics
    kpi.max_pitch_angle_deg    = m.peak_pitch_angle_deg;
    kpi.max_pitch_accel_rads2  = m.peak_pitch_accel;
    kpi.rms_pitch_accel_rads2  = m.rms_pitch_accel;

    % 4. Suspension Working Space (Travel)
    limit_mm = m.jounce_limit * 1000; % 80 mm
    kpi.jounce_limit_mm        = limit_mm;
    kpi.max_travel_front_mm    = m.peak_susp_travel_f * 1000;
    kpi.max_travel_rear_mm     = m.peak_susp_travel_r * 1000;
    kpi.travel_util_front_pct  = m.travel_util_f_pct;
    kpi.travel_util_rear_pct   = m.travel_util_r_pct;
    kpi.travel_margin_front_mm = limit_mm - kpi.max_travel_front_mm;
    kpi.travel_margin_rear_mm  = limit_mm - kpi.max_travel_rear_mm;

    % 5. Suspension Dynamic Forces
    kpi.max_susp_force_front_kN = m.peak_susp_force_f / 1000;
    kpi.max_susp_force_rear_kN  = m.peak_susp_force_r / 1000;

    % 6. Wheel Hop & Road Holding
    kpi.max_wheel_disp_front_mm = m.peak_wheel_disp_f * 1000;
    kpi.max_wheel_disp_rear_mm  = m.peak_wheel_disp_r * 1000;
    kpi.min_tyre_ratio_front    = m.min_tyre_ratio_f;
    kpi.min_tyre_ratio_rear     = m.min_tyre_ratio_r;
    kpi.wheel_lift_front        = m.wheel_lift_front;
    kpi.wheel_lift_rear         = m.wheel_lift_rear;
    kpi.road_contact_maintained = ~m.wheel_lift_front && ~m.wheel_lift_rear;
end
