function h = plot_wheel_displacement(ax, sim)
% PLOT_WHEEL_DISPLACEMENT Plots front and rear unsprung mass (wheel) vertical displacements vs road.
%
% Inputs:
%   ax  - (Optional) Axes handle. If empty, creates a new figure.
%   sim - Simulation results struct containing .t, .z_uf, .z_ur, .z0f, .z0r, .metrics
%
% Output:
%   h   - Graphic handle to the axes

    if nargin < 1 || isempty(ax)
        fig = figure('Name', 'Front and Rear Wheel Displacement', 'Color', [1 1 1]);
        ax = gca;
    end

    t    = sim.t;
    z_uf = sim.z_uf * 1000; % Convert to mm
    z_ur = sim.z_ur * 1000; % Convert to mm
    z0f  = sim.z0f * 1000;  % Convert to mm
    z0r  = sim.z0r * 1000;  % Convert to mm
    m    = sim.metrics;

    c_blue = [0.00, 0.45, 0.74];
    c_red  = [0.85, 0.33, 0.10];
    c_gray = [0.60, 0.60, 0.60];

    hold(ax, 'on');
    plot(ax, t, z_uf, 'Color', c_blue, 'LineWidth', 1.8, 'DisplayName', 'Front Wheel Displ z_{uf}');
    plot(ax, t, z_ur, 'Color', c_red, 'LineWidth', 1.8, 'DisplayName', 'Rear Wheel Displ z_{ur}');
    plot(ax, t, z0f, ':', 'Color', c_gray, 'LineWidth', 1.2, 'DisplayName', 'Front Road Surface');
    plot(ax, t, z0r, '--', 'Color', [0.8 0.6 0.6], 'LineWidth', 1.0, 'DisplayName', 'Rear Road Surface');
    yline(ax, 0, 'k:', 'LineWidth', 0.8, 'HandleVisibility', 'off');
    hold(ax, 'off');

    grid(ax, 'on');
    box(ax, 'on');
    xlabel(ax, 'Time (s)', 'FontWeight', 'bold');
    ylabel(ax, 'Displacement (mm)', 'FontWeight', 'bold');
    title(ax, sprintf('7. Front & Rear Wheel Displacement (Front Max: %.1f mm | Rear Max: %.1f mm)', ...
          m.peak_wheel_disp_f * 1000, m.peak_wheel_disp_r * 1000), 'FontSize', 11, 'FontWeight', 'bold');
    legend(ax, 'Location', 'northeast', 'FontSize', 8);
    xlim(ax, [t(1), t(end)]);

    % Add KPI annotation badge
    kpi_str = sprintf('Front Peak: %.1f mm | Rear Peak: %.1f mm | Wheel Hop fn ~ 11 Hz', ...
                      m.peak_wheel_disp_f * 1000, m.peak_wheel_disp_r * 1000);
    text(ax, 0.02, 0.90, kpi_str, 'Units', 'normalized', 'FontSize', 8, ...
         'BackgroundColor', [0.95 0.95 0.95 0.8], 'EdgeColor', [0.7 0.7 0.7]);

    h = ax;
end
