function h = plot_front_suspension_deflection(ax, sim)
% PLOT_FRONT_SUSPENSION_DEFLECTION Plots front suspension working space travel and bumpstop limits.
%
% Inputs:
%   ax  - (Optional) Axes handle. If empty, creates a new figure.
%   sim - Simulation results struct containing .t, .susp_def_f, .metrics
%
% Output:
%   h   - Graphic handle to the axes

    if nargin < 1 || isempty(ax)
        fig = figure('Name', 'Front Suspension Deflection', 'Color', [1 1 1]);
        ax = gca;
    end

    t          = sim.t;
    def_f      = sim.susp_def_f * 1000; % Convert to mm (compression positive)
    m          = sim.metrics;
    limit_mm   = m.jounce_limit * 1000; % 80 mm
    peak_def_f = m.peak_susp_travel_f * 1000;
    util_pct   = m.travel_util_f_pct;

    c_blue = [0.00, 0.45, 0.74];

    hold(ax, 'on');
    plot(ax, t, def_f, 'Color', c_blue, 'LineWidth', 1.8, 'DisplayName', 'Front Suspension Travel \delta_{sf}');
    yline(ax, 0, 'k:', 'LineWidth', 0.8, 'DisplayName', 'Nominal Ride Height');
    yline(ax, limit_mm, 'r--', 'LineWidth', 1.2, 'DisplayName', sprintf('Jounce Limit (+%.0f mm)', limit_mm));
    yline(ax, -limit_mm, 'm--', 'LineWidth', 1.2, 'DisplayName', sprintf('Rebound Limit (-%.0f mm)', limit_mm));
    hold(ax, 'off');

    grid(ax, 'on');
    box(ax, 'on');
    xlabel(ax, 'Time (s)', 'FontWeight', 'bold');
    ylabel(ax, 'Deflection (mm)', 'FontWeight', 'bold');
    title(ax, sprintf('4. Front Suspension Deflection (Max: %.1f mm | Utilized: %.1f%%)', ...
          peak_def_f, util_pct), 'FontSize', 11, 'FontWeight', 'bold');
    legend(ax, 'Location', 'northeast', 'FontSize', 8);
    xlim(ax, [t(1), t(end)]);
    ylim(ax, [-limit_mm * 1.2, limit_mm * 1.2]);

    % Add KPI annotation badge
    kpi_str = sprintf('Max Travel: %.1f mm | Peak Force: %.2f kN | Margin: %.1f mm', ...
                      peak_def_f, m.peak_susp_force_f / 1000, limit_mm - peak_def_f);
    text(ax, 0.02, 0.90, kpi_str, 'Units', 'normalized', 'FontSize', 8, ...
         'BackgroundColor', [0.95 0.95 0.95 0.8], 'EdgeColor', [0.7 0.7 0.7]);

    h = ax;
end
