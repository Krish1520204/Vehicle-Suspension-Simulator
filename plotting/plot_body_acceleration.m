function h = plot_body_acceleration(ax, sim)
% PLOT_BODY_ACCELERATION Plots vehicle body vertical acceleration (CG heave and driver position).
%
% Inputs:
%   ax  - (Optional) Axes handle. If empty, creates a new figure.
%   sim - Simulation results struct containing .t, .ddz_s, .ddz_driver, .metrics
%
% Output:
%   h   - Graphic handle to the axes

    if nargin < 1 || isempty(ax)
        fig = figure('Name', 'Vehicle Body Vertical Acceleration', 'Color', [1 1 1]);
        ax = gca;
    end

    t        = sim.t;
    ddz_s    = sim.ddz_s;      % CG heave acceleration (m/s^2)
    ddz_drv  = sim.ddz_driver; % Driver seat acceleration (m/s^2)
    m        = sim.metrics;
    g        = 9.81;

    c_dark = [0.15, 0.15, 0.15];
    c_gold = [0.93, 0.69, 0.13];

    hold(ax, 'on');
    plot(ax, t, ddz_s, 'Color', c_dark, 'LineWidth', 1.8, 'DisplayName', 'Body CG Heave \ddot{z}_s');
    plot(ax, t, ddz_drv, 'Color', c_gold, 'LineWidth', 1.4, 'DisplayName', 'Driver Seat Pos \ddot{z}_{drv}');
    yline(ax, 0, 'k:', 'LineWidth', 0.8, 'DisplayName', 'Zero Acceleration Baseline');
    yline(ax, m.peak_heave_accel, 'r--', 'LineWidth', 1.0, ...
          'DisplayName', sprintf('Peak Heave: +%.2f m/s^2 (%.2f g)', m.peak_heave_accel, m.peak_heave_accel / g));
    yline(ax, -m.peak_heave_accel, 'r--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    hold(ax, 'off');

    grid(ax, 'on');
    box(ax, 'on');
    xlabel(ax, 'Time (s)', 'FontWeight', 'bold');
    ylabel(ax, 'Acceleration (m/s^2)', 'FontWeight', 'bold');
    title(ax, sprintf('6. Vehicle Body Vertical Acceleration (Peak: %.2f m/s^2 [%.2f g] | RMS: %.2f m/s^2)', ...
          m.peak_heave_accel, m.peak_heave_accel / g, m.rms_heave_accel), 'FontSize', 11, 'FontWeight', 'bold');
    legend(ax, 'Location', 'northeast', 'FontSize', 8);
    xlim(ax, [t(1), t(end)]);

    % Add KPI annotation badge
    kpi_str = sprintf('Max CG: %.2f m/s^2 (%.2f g) | Max Driver: %.2f m/s^2 (%.2f g) | RMS Heave: %.2f m/s^2', ...
                      m.peak_heave_accel, m.peak_heave_accel / g, ...
                      m.peak_driver_accel, m.peak_driver_accel / g, ...
                      m.rms_heave_accel);
    text(ax, 0.02, 0.90, kpi_str, 'Units', 'normalized', 'FontSize', 8, ...
         'BackgroundColor', [0.95 0.95 0.95 0.8], 'EdgeColor', [0.7 0.7 0.7]);

    h = ax;
end
