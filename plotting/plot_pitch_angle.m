function h = plot_pitch_angle(ax, sim)
% PLOT_PITCH_ANGLE Plots vehicle body pitch angular displacement.
%
% Inputs:
%   ax  - (Optional) Axes handle. If empty, creates a new figure.
%   sim - Simulation results struct containing .t, .theta_deg, .metrics
%
% Output:
%   h   - Graphic handle to the axes

    if nargin < 1 || isempty(ax)
        fig = figure('Name', 'Vehicle Body Pitch Angle', 'Color', [1 1 1]);
        ax = gca;
    end

    t     = sim.t;
    theta = sim.theta_deg; % Pitch in degrees
    m     = sim.metrics;

    c_purple = [0.49, 0.18, 0.56];

    hold(ax, 'on');
    plot(ax, t, theta, 'Color', c_purple, 'LineWidth', 1.8, 'DisplayName', 'Body Pitch Angle \theta');
    yline(ax, 0, 'k:', 'LineWidth', 0.8, 'DisplayName', 'Level Horizon (\theta = 0)');
    yline(ax, m.peak_pitch_angle_deg, '--', 'Color', [0.8 0.2 0.2], 'LineWidth', 1.0, ...
          'DisplayName', sprintf('Peak Nose-Up: +%.2f^\\circ', m.peak_pitch_angle_deg));
    yline(ax, -m.peak_pitch_angle_deg, '--', 'Color', [0.2 0.6 0.2], 'LineWidth', 1.0, ...
          'DisplayName', sprintf('Peak Nose-Down: -%.2f^\\circ', m.peak_pitch_angle_deg));
    hold(ax, 'off');

    grid(ax, 'on');
    box(ax, 'on');
    xlabel(ax, 'Time (s)', 'FontWeight', 'bold');
    ylabel(ax, 'Pitch Angle (deg)', 'FontWeight', 'bold');
    title(ax, sprintf('3. Vehicle Body Pitch Angle (Max: %.2f^\\circ | RMS: %.2f^\\circ)', ...
          m.peak_pitch_angle_deg, rms(theta)), 'FontSize', 11, 'FontWeight', 'bold');
    legend(ax, 'Location', 'northeast', 'FontSize', 9);
    xlim(ax, [t(1), t(end)]);

    % Add KPI annotation badge
    kpi_str = sprintf('Max Pitch: %.2f^\\circ | RMS Pitch Accel: %.2f rad/s^2', ...
                      m.peak_pitch_angle_deg, m.rms_pitch_accel);
    text(ax, 0.02, 0.90, kpi_str, 'Units', 'normalized', 'FontSize', 8, ...
         'BackgroundColor', [0.95 0.95 0.95 0.8], 'EdgeColor', [0.7 0.7 0.7]);

    h = ax;
end
