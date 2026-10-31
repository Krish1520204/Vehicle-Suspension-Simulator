function h = plot_road_profile(ax, sim)
% PLOT_ROAD_PROFILE Plots front and rear wheel road displacement profiles with time delay.
%
% Inputs:
%   ax  - (Optional) Axes handle to plot into. If empty or not provided, creates a new figure.
%   sim - Simulation results struct containing .t, .z0f, .z0r, .params, .road_cfg
%
% Output:
%   h   - Graphic handle to the axes

    if nargin < 1 || isempty(ax)
        fig = figure('Name', 'Road Displacement Profile', 'Color', [1 1 1]);
        ax = gca;
    end

    t   = sim.t;
    z0f = sim.z0f * 1000; % Convert to mm
    z0r = sim.z0r * 1000; % Convert to mm
    tau = sim.params.L / sim.road_cfg.v;
    H_max = max(max(abs(z0f)), max(abs(z0r)));

    c_blue = [0.00, 0.45, 0.74];
    c_red  = [0.85, 0.33, 0.10];

    hold(ax, 'on');
    plot(ax, t, z0f, 'Color', c_blue, 'LineWidth', 1.8, 'DisplayName', 'Front Road Profile z_{0f}');
    plot(ax, t, z0r, '--', 'Color', c_red, 'LineWidth', 1.8, 'DisplayName', sprintf('Rear Road Profile z_{0r} (\\tau = %.3f s)', tau));
    hold(ax, 'off');

    grid(ax, 'on');
    box(ax, 'on');
    xlabel(ax, 'Time (s)', 'FontWeight', 'bold');
    ylabel(ax, 'Road Elevation (mm)', 'FontWeight', 'bold');
    title(ax, sprintf('1. Road Displacement Profile (Peak: %.1f mm, Speed: %.1f km/h)', ...
          H_max, sim.road_cfg.v * 3.6), 'FontSize', 11, 'FontWeight', 'bold');
    legend(ax, 'Location', 'northeast', 'FontSize', 9);
    xlim(ax, [t(1), t(end)]);

    % Add KPI annotation badge
    kpi_str = sprintf('v = %.1f km/h | \\tau = %.1f ms | H_{max} = %.1f mm', ...
                      sim.road_cfg.v * 3.6, tau * 1000, H_max);
    text(ax, 0.02, 0.90, kpi_str, 'Units', 'normalized', 'FontSize', 8, ...
         'BackgroundColor', [0.95 0.95 0.95 0.8], 'EdgeColor', [0.7 0.7 0.7]);

    h = ax;
end
