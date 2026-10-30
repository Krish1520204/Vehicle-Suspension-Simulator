function fig = plot_simulation_results(sim, save_flag)
% PLOT_SIMULATION_RESULTS Generates engineering plots of the 4-DOF suspension simulation.
%
% Visualizes:
%   1. Road Profile (Front & Rear inputs with wheelbase delay)
%   2. Sprung Mass Displacements (CG Heave, Front & Rear body points)
%   3. Pitch Angle and Pitch Rate
%   4. Unsprung Mass Displacements (Wheel hop) vs Road
%   5. Body Heave & Driver Seat Vertical Acceleration
%   6. Suspension Forces (Front & Rear)
%   7. Suspension Working Space / Deflection (Travel)
%   8. Total Tyre Normal Loads (Contact retention check)
%
% Inputs:
%   sim       - Struct output from run_simulation or calculate_outputs
%   save_flag - Boolean: if true, saves figure to 'outputs/simulation_results.png' [default: false]

    if nargin < 2
        save_flag = false;
    end

    t = sim.t;
    p = sim.params;

    % Create figure window with modern dimensions
    fig = figure('Name', '4-DOF Half-Car Suspension Simulation Results', ...
                 'NumberTitle', 'off', ...
                 'Color', [1 1 1], ...
                 'Position', [100, 50, 1300, 850]);

    % Define consistent color palette
    c_blue   = [0.00, 0.45, 0.74];
    c_red    = [0.85, 0.33, 0.10];
    c_gold   = [0.93, 0.69, 0.13];
    c_purple = [0.49, 0.18, 0.56];
    c_green  = [0.47, 0.67, 0.19];
    c_dark   = [0.15, 0.15, 0.15];

    % 1. Road Elevation Profile
    subplot(4, 2, 1);
    plot(t, sim.z0f * 1000, 'Color', c_blue, 'LineWidth', 1.8, 'DisplayName', 'Front Road z_{0f}');
    hold on;
    plot(t, sim.z0r * 1000, '--', 'Color', c_red, 'LineWidth', 1.8, 'DisplayName', 'Rear Road z_{0r}');
    grid on;
    ylabel('Elevation (mm)');
    title('Road Surface Profile Input (with Wheelbase Delay)');
    legend('Location', 'northeast');
    xlim([t(1), t(end)]);

    % 2. Sprung Mass Displacements
    subplot(4, 2, 2);
    plot(t, sim.z_s * 1000, 'Color', c_dark, 'LineWidth', 2.0, 'DisplayName', 'CG Heave z_s');
    hold on;
    plot(t, sim.z_sf * 1000, 'Color', c_blue, 'LineWidth', 1.4, 'DisplayName', 'Front Body z_{sf}');
    plot(t, sim.z_sr * 1000, 'Color', c_red, 'LineWidth', 1.4, 'DisplayName', 'Rear Body z_{sr}');
    grid on;
    ylabel('Displacement (mm)');
    title('Sprung Mass Vertical Displacements');
    legend('Location', 'northeast');
    xlim([t(1), t(end)]);

    % 3. Pitch Angle Response
    subplot(4, 2, 3);
    plot(t, sim.theta_deg, 'Color', c_purple, 'LineWidth', 1.8, 'DisplayName', 'Pitch Angle \theta');
    grid on;
    ylabel('Pitch (deg)');
    title(sprintf('Body Pitch Angle (Peak: %.2f deg)', sim.metrics.peak_pitch_angle_deg));
    xlim([t(1), t(end)]);

    % 4. Unsprung Mass (Wheel) Motion vs Road
    subplot(4, 2, 4);
    plot(t, sim.z_uf * 1000, 'Color', c_blue, 'LineWidth', 1.6, 'DisplayName', 'Front Wheel z_{uf}');
    hold on;
    plot(t, sim.z_ur * 1000, 'Color', c_red, 'LineWidth', 1.6, 'DisplayName', 'Rear Wheel z_{ur}');
    plot(t, sim.z0f * 1000, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'DisplayName', 'Front Road');
    grid on;
    ylabel('Displacement (mm)');
    title('Unsprung Mass (Wheel Hop) Displacements');
    legend('Location', 'northeast');
    xlim([t(1), t(end)]);

    % 5. Accelerations (Ride Comfort)
    subplot(4, 2, 5);
    plot(t, sim.ddz_s, 'Color', c_dark, 'LineWidth', 1.8, 'DisplayName', 'CG Heave \ddot{z}_s');
    hold on;
    plot(t, sim.ddz_driver, 'Color', c_gold, 'LineWidth', 1.4, 'DisplayName', 'Driver Pos \ddot{z}_{drv}');
    grid on;
    ylabel('Accel (m/s^2)');
    title(sprintf('Body Accelerations (RMS: %.2f m/s^2, Peak: %.2f m/s^2)', ...
          sim.metrics.rms_heave_accel, sim.metrics.peak_heave_accel));
    legend('Location', 'northeast');
    xlim([t(1), t(end)]);

    % 6. Suspension Forces
    subplot(4, 2, 6);
    plot(t, sim.F_sf / 1000, 'Color', c_blue, 'LineWidth', 1.6, 'DisplayName', 'Front Force F_{sf}');
    hold on;
    plot(t, sim.F_sr / 1000, 'Color', c_red, 'LineWidth', 1.6, 'DisplayName', 'Rear Force F_{sr}');
    grid on;
    ylabel('Force (kN)');
    title(sprintf('Suspension Dynamic Forces (Peak: F_f=%.1f kN, F_r=%.1f kN)', ...
          sim.metrics.peak_susp_force_f/1000, sim.metrics.peak_susp_force_r/1000));
    legend('Location', 'northeast');
    xlim([t(1), t(end)]);

    % 7. Suspension Travel / Deflection (Working Space)
    subplot(4, 2, 7);
    plot(t, sim.susp_def_f * 1000, 'Color', c_blue, 'LineWidth', 1.6, 'DisplayName', 'Front Deflection');
    hold on;
    plot(t, sim.susp_def_r * 1000, 'Color', c_red, 'LineWidth', 1.6, 'DisplayName', 'Rear Deflection');
    yline(80, 'k--', 'LineWidth', 1, 'DisplayName', 'Jounce Limit');
    yline(-80, 'k--', 'LineWidth', 1, 'DisplayName', 'Rebound Limit');
    grid on;
    xlabel('Time (s)');
    ylabel('Deflection (mm)');
    title(sprintf('Suspension Working Space (Front Peak: %.1f mm, Rear Peak: %.1f mm)', ...
          sim.metrics.peak_susp_travel_f * 1000, sim.metrics.peak_susp_travel_r * 1000));
    legend('Location', 'northeast');
    xlim([t(1), t(end)]);

    % 8. Total Tyre Normal Loads (Road Holding / Contact Check)
    subplot(4, 2, 8);
    plot(t, sim.F_z_tf / 1000, 'Color', c_blue, 'LineWidth', 1.6, 'DisplayName', 'Front Tyre Load F_{z,tf}');
    hold on;
    plot(t, sim.F_z_tr / 1000, 'Color', c_red, 'LineWidth', 1.6, 'DisplayName', 'Rear Tyre Load F_{z,tr}');
    yline(p.W_tf_static / 1000, ':', 'Color', c_blue, 'LineWidth', 1.2, 'DisplayName', 'Front Static W');
    yline(p.W_tr_static / 1000, ':', 'Color', c_red, 'LineWidth', 1.2, 'DisplayName', 'Rear Static W');
    yline(0, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Zero Normal Load (Lift-off)');
    grid on;
    xlabel('Time (s)');
    ylabel('Normal Load (kN)');
    title(sprintf('Tyre Normal Ground Contact Loads (Min Ratio: F=%.2f, R=%.2f)', ...
          sim.metrics.min_tyre_ratio_f, sim.metrics.min_tyre_ratio_r));
    legend('Location', 'northeast');
    xlim([t(1), t(end)]);

    % Save plot if requested
    if save_flag
        script_dir = fileparts(mfilename('fullpath'));
        out_dir = fullfile(fileparts(script_dir), 'outputs');
        if ~exist(out_dir, 'dir')
            mkdir(out_dir);
        end
        save_path = fullfile(out_dir, 'simulation_results.png');
        try
            exportgraphics(fig, save_path, 'Resolution', 300);
        catch
            saveas(fig, save_path);
        end
        fprintf('Saved simulation plot to: %s\n', save_path);
    end
end
