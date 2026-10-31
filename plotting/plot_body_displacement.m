function h = plot_body_displacement(ax, sim)
% PLOT_BODY_DISPLACEMENT Plots vehicle body vertical displacement (heave at CG & axles).
%
% Inputs:
%   ax  - (Optional) Axes handle to plot into. If empty, creates a new figure.
%   sim - Simulation results struct containing .t, .z_s, .z_sf, .z_sr, .metrics
%
% Output:
%   h   - Graphic handle to the axes

    if nargin < 1 || isempty(ax)
        fig = figure('Name', 'Vehicle Body Vertical Displacement', 'Color', [1 1 1]);
        ax = gca;
    end

    t    = sim.t;
    z_s  = sim.z_s * 1000;  % Convert to mm
    z_sf = sim.z_sf * 1000; % Convert to mm
    z_sr = sim.z_sr * 1000; % Convert to mm
    m    = sim.metrics;

    c_dark = [0.10, 0.10, 0.10];
    c_blue = [0.00, 0.45, 0.74];
    c_red  = [0.85, 0.33, 0.10];

    hold(ax, 'on');
    plot(ax, t, z_s, 'Color', c_dark, 'LineWidth', 2.0, 'DisplayName', 'Body Heave at CG (z_s)');
    plot(ax, t, z_sf, 'Color', c_blue, 'LineWidth', 1.4, 'DisplayName', 'Front Body Point (z_{sf} = z_s + a\theta)');
    plot(ax, t, z_sr, 'Color', c_red, 'LineWidth', 1.4, 'DisplayName', 'Rear Body Point (z_{sr} = z_s - b\theta)');
    yline(ax, 0, 'k:', 'LineWidth', 0.8, 'DisplayName', 'Static Equilibrium');
    hold(ax, 'off');

    grid(ax, 'on');
    box(ax, 'on');
    xlabel(ax, 'Time (s)', 'FontWeight', 'bold');
    ylabel(ax, 'Displacement (mm)', 'FontWeight', 'bold');
    title(ax, sprintf('2. Vehicle Body Vertical Displacement (Peak: %.1f mm | RMS: %.1f mm)', ...
          m.peak_body_disp * 1000, m.rms_body_disp * 1000), 'FontSize', 11, 'FontWeight', 'bold');
    legend(ax, 'Location', 'northeast', 'FontSize', 9);
    xlim(ax, [t(1), t(end)]);

    % Add KPI annotation badge
    kpi_str = sprintf('Max Heave: %.1f mm | Front Max: %.1f mm | Rear Max: %.1f mm', ...
                      m.peak_body_disp * 1000, m.peak_body_disp_f * 1000, m.peak_body_disp_r * 1000);
    text(ax, 0.02, 0.90, kpi_str, 'Units', 'normalized', 'FontSize', 8, ...
         'BackgroundColor', [0.95 0.95 0.95 0.8], 'EdgeColor', [0.7 0.7 0.7]);

    h = ax;
end
