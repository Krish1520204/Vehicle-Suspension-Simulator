function h = plot_rear_suspension_deflection(ax, sim)
% PLOT_REAR_SUSPENSION_DEFLECTION Plots rear suspension working space travel and bumpstop limits.
%
% Inputs:
%   ax  - (Optional) Axes handle. If empty, creates a new figure.
%   sim - Simulation results struct containing .t, .susp_def_r, .metrics
%
% Output:
%   h   - Graphic handle to the axes

    if nargin < 1 || isempty(ax)
        fig = figure('Name', 'Rear Suspension Deflection', 'Color', [1 1 1]);
        ax = gca;
    end

    t          = sim.t;
    def_r      = sim.susp_def_r * 1000; % Convert to mm (compression positive)
    m          = sim.metrics;
    limit_mm   = m.jounce_limit * 1000; % 80 mm
    peak_def_r = m.peak_susp_travel_r * 1000;
    util_pct   = m.travel_util_r_pct;

    c_red = [0.85, 0.33, 0.10];

    hold(ax, 'on');
    plot(ax, t, def_r, 'Color', c_red, 'LineWidth', 1.8, 'DisplayName', 'Rear Suspension Travel \delta_{sr}');
    yline(ax, 0, 'k:', 'LineWidth', 0.8, 'DisplayName', 'Nominal Ride Height');
    yline(ax, limit_mm, 'r--', 'LineWidth', 1.2, 'DisplayName', sprintf('Jounce Limit (+%.0f mm)', limit_mm));
    yline(ax, -limit_mm, 'm--', 'LineWidth', 1.2, 'DisplayName', sprintf('Rebound Limit (-%.0f mm)', limit_mm));
    hold(ax, 'off');

    grid(ax, 'on');
    box(ax, 'on');
    xlabel(ax, 'Time (s)', 'FontWeight', 'bold');
    ylabel(ax, 'Deflection (mm)', 'FontWeight', 'bold');
    title(ax, sprintf('5. Rear Suspension Deflection (Max: %.1f mm | Utilized: %.1f%%)', ...
          peak_def_r, util_pct), 'FontSize', 11, 'FontWeight', 'bold');
    legend(ax, 'Location', 'northeast', 'FontSize', 8);
    xlim(ax, [t(1), t(end)]);
    ylim(ax, [-limit_mm * 1.2, limit_mm * 1.2]);

    % Add KPI annotation badge
    kpi_str = sprintf('Max Travel: %.1f mm | Peak Force: %.2f kN | Margin: %.1f mm', ...
                      peak_def_r, m.peak_susp_force_r / 1000, limit_mm - peak_def_r);
    text(ax, 0.02, 0.90, kpi_str, 'Units', 'normalized', 'FontSize', 8, ...
         'BackgroundColor', [0.95 0.95 0.95 0.8], 'EdgeColor', [0.7 0.7 0.7]);

    h = ax;
end
