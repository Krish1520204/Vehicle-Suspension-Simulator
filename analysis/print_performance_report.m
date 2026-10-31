function print_performance_report(kpi)
% PRINT_PERFORMANCE_REPORT Prints a formatted tabular report of all numerical performance indicators.
%
% Input:
%   kpi - Struct from calculate_kpis.m (or simulation result struct)

    if isfield(kpi, 'metrics')
        kpi = calculate_kpis(kpi);
    end

    fprintf('\n========================================================================================\n');
    fprintf('                    VEHICLE SUSPENSION PERFORMANCE & KPI ANALYSIS                       \n');
    fprintf('========================================================================================\n');
    fprintf(' Vehicle Preset       : %-15s | Road Scenario        : %-15s\n', ...
            upper(kpi.vehicle_type), upper(kpi.road_type));
    fprintf(' Vehicle Forward Speed: %6.1f km/h (%5.1f m/s)\n', ...
            kpi.speed_kmh, kpi.speed_mps);
    fprintf('----------------------------------------------------------------------------------------\n');
    fprintf(' %-36s | %-16s | %-24s\n', 'Performance Indicator', 'Simulated Value', 'Engineering Context');
    fprintf('----------------------------------------------------------------------------------------\n');

    % Body Displacements
    fprintf(' Maximum Body Heave Displacement      |   %7.2f mm    | Peak CG vertical excursion\n', ...
            kpi.max_body_disp_mm);
    fprintf(' Front Body Point Displacement        |   %7.2f mm    | Maximum front overhang heave\n', ...
            kpi.max_front_body_disp_mm);
    fprintf(' Rear Body Point Displacement         |   %7.2f mm    | Maximum rear overhang heave\n', ...
            kpi.max_rear_body_disp_mm);
    fprintf(' RMS Body Vertical Displacement       |   %7.2f mm    | Overall body motion energy\n', ...
            kpi.rms_body_disp_mm);
    fprintf('----------------------------------------------------------------------------------------\n');

    % Accelerations
    fprintf(' Maximum Body Vertical Acceleration   |   %7.2f m/s^2 | %5.2f g at vehicle CG\n', ...
            kpi.max_heave_accel_mps2, kpi.max_heave_accel_g);
    fprintf(' Maximum Driver Seat Acceleration     |   %7.2f m/s^2 | %5.2f g perceived by driver\n', ...
            kpi.max_driver_accel_mps2, kpi.max_driver_accel_g);
    fprintf(' RMS Heave Acceleration (ISO 2631)    |   %7.2f m/s^2 | Comfort index\n', ...
            kpi.rms_heave_accel_mps2);
    fprintf(' RMS Driver Position Acceleration     |   %7.2f m/s^2 | Level: %s\n', ...
            kpi.rms_driver_accel_mps2, kpi.iso2631_comfort);
    fprintf('----------------------------------------------------------------------------------------\n');

    % Pitch
    fprintf(' Maximum Body Pitch Angle             |   %7.2f deg   | Angular attitude variation\n', ...
            kpi.max_pitch_angle_deg);
    fprintf(' Maximum Body Pitch Acceleration      |   %7.2f rad/s^2| Peak rotational jolt\n', ...
            kpi.max_pitch_accel_rads2);
    fprintf('----------------------------------------------------------------------------------------\n');

    % Suspension Working Space
    fprintf(' Maximum Front Suspension Travel      |   %7.1f mm    | %4.1f%% of %.0f mm bumpstop limit\n', ...
            kpi.max_travel_front_mm, kpi.travel_util_front_pct, kpi.jounce_limit_mm);
    fprintf(' Front Bumpstop Clearance Margin      |   %7.1f mm    | Remaining travel buffer\n', ...
            kpi.travel_margin_front_mm);
    fprintf(' Maximum Rear Suspension Travel       |   %7.1f mm    | %4.1f%% of %.0f mm bumpstop limit\n', ...
            kpi.max_travel_rear_mm, kpi.travel_util_rear_pct, kpi.jounce_limit_mm);
    fprintf(' Rear Bumpstop Clearance Margin       |   %7.1f mm    | Remaining travel buffer\n', ...
            kpi.travel_margin_rear_mm);
    fprintf(' Peak Front Dynamic Suspension Force  |   %7.2f kN    | Load transmitted to chassis\n', ...
            kpi.max_susp_force_front_kN);
    fprintf(' Peak Rear Dynamic Suspension Force   |   %7.2f kN    | Load transmitted to chassis\n', ...
            kpi.max_susp_force_rear_kN);
    fprintf('----------------------------------------------------------------------------------------\n');

    % Wheel Hop & Contact
    fprintf(' Maximum Front Wheel Displacement     |   %7.1f mm    | Unsprung mass motion\n', ...
            kpi.max_wheel_disp_front_mm);
    fprintf(' Maximum Rear Wheel Displacement      |   %7.1f mm    | Unsprung mass motion\n', ...
            kpi.max_wheel_disp_rear_mm);
    fprintf(' Front Min Dynamic Tyre Load Ratio    |   %7.2f       | R < 0 indicates tyre lift-off\n', ...
            kpi.min_tyre_ratio_front);
    fprintf(' Rear Min Dynamic Tyre Load Ratio     |   %7.2f       | R < 0 indicates tyre lift-off\n', ...
            kpi.min_tyre_ratio_rear);

    if kpi.road_contact_maintained
        status_msg = 'PASSED (Continuous tyre-road contact maintained)';
    else
        status_msg = 'WARNING (Transient tyre lift-off / loss of road grip detected)';
    end
    fprintf(' Road Holding / Tyre Contact Status   | %-37s\n', status_msg);
    fprintf('========================================================================================\n\n');
end
