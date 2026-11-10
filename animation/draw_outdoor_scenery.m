function draw_outdoor_scenery(ax, x_view_min, x_view_max, road_cfg, params)
% DRAW_OUTDOOR_SCENERY Renders an outdoor environment with sky, clouds, buildings, trees, and road.
%
% Elements:
%   - Realistic sky with atmospheric gradient and fluffy cumulus clouds
%   - Distant city skyline with varied building silhouettes and window grids
%   - Midground lush green trees and streetlamps along the roadside
%   - Paved asphalt road featuring actual multiple road bumps and hatched roadbed
%
% Inputs:
%   ax                    - Axes handle
%   x_view_min, x_view_max - Current horizontal camera view range (m)
%   road_cfg              - Road profile configuration struct
%   params                - Vehicle parameters struct

    % Scene vertical bounds
    z_sky_top   = 3.8;
    z_horizon   = 0.70;
    z_ground_ref = 0.0;
    z_earth_bot = -1.2;

    W_view = x_view_max - x_view_min;

    %% 1. Sky Background (Layered vertical sky gradient)
    y_sky = linspace(z_horizon, z_sky_top, 8);
    for i = 1:length(y_sky)-1
        frac = (y_sky(i) - z_horizon) / (z_sky_top - z_horizon);
        % Interpolate from pale horizon blue to deeper atmosphere blue
        c_sky = (1 - frac) * [0.72, 0.88, 0.98] + frac * [0.25, 0.55, 0.85];
        patch('XData', [x_view_min, x_view_max, x_view_max, x_view_min], ...
              'YData', [y_sky(i), y_sky(i), y_sky(i+1), y_sky(i+1)], ...
              'FaceColor', c_sky, 'EdgeColor', 'none', 'Parent', ax);
    end

    %% 2. Distant Clouds
    % Generate clouds deterministically across space
    cloud_spacing = 7.0; % meters between cloud clusters
    first_cloud_x = floor(x_view_min / cloud_spacing) * cloud_spacing;
    last_cloud_x  = ceil(x_view_max / cloud_spacing) * cloud_spacing;

    for xc = first_cloud_x : cloud_spacing : last_cloud_x
        draw_single_cloud(ax, xc + 1.2, 2.9, 1.4, 0.40);
        draw_single_cloud(ax, xc + 4.5, 3.3, 1.8, 0.48);
    end

    %% 3. Distant City Skyline / Buildings
    bldg_spacing = 2.5;
    bldg_x_start = floor(x_view_min / bldg_spacing) * bldg_spacing;
    bldg_x_end   = ceil(x_view_max / bldg_spacing) * bldg_spacing;

    for xb = bldg_x_start : bldg_spacing : bldg_x_end
        % Deterministic pseudo-random height based on position
        seed_val = mod(round(xb * 7), 5);
        bldg_h = 1.30 + seed_val * 0.28;
        bldg_w = 2.10;

        % Soft atmospheric building color
        c_bldg = [0.65, 0.72, 0.80] - seed_val * 0.03;
        patch('XData', [xb, xb + bldg_w, xb + bldg_w, xb], ...
              'YData', [z_horizon, z_horizon, z_horizon + bldg_h, z_horizon + bldg_h], ...
              'FaceColor', c_bldg, 'EdgeColor', [0.55, 0.62, 0.70], ...
              'LineWidth', 0.8, 'Parent', ax);

        % Subtle window grid
        num_floors = 3 + seed_val;
        for fl = 1:num_floors
            y_win = z_horizon + 0.15 + (fl - 1) * 0.28;
            for col = 1:3
                x_win = xb + 0.35 + (col - 1) * 0.55;
                patch('XData', [x_win, x_win+0.25, x_win+0.25, x_win], ...
                      'YData', [y_win, y_win, y_win+0.16, y_win+0.16], ...
                      'FaceColor', [0.85, 0.90, 0.96], 'EdgeColor', 'none', 'Parent', ax);
            end
        end
    end

    %% 4. Midground Trees & Roadside Greenery
    tree_spacing = 3.8;
    tree_x_start = floor(x_view_min / tree_spacing) * tree_spacing;
    tree_x_end   = ceil(x_view_max / tree_spacing) * tree_spacing;

    for xt = tree_x_start : tree_spacing : tree_x_end
        draw_tree(ax, xt + 0.8, z_horizon);
    end

    % Grass Verge between background and road
    patch('XData', [x_view_min, x_view_max, x_view_max, x_view_min], ...
          'YData', [0.15, 0.15, z_horizon, z_horizon], ...
          'FaceColor', [0.38, 0.65, 0.30], 'EdgeColor', 'none', 'Parent', ax);

    %% 5. Road Surface & Multiple Road Bumps
    % Sample continuous road profile across the viewing window
    N_road_pts = 600;
    x_road = linspace(x_view_min, x_view_max, N_road_pts);

    % Compute road elevation from actual simulation road function
    t_fake = x_road / road_cfg.v;
    [z0f_road, ~, ~, ~] = get_road_profile(t_fake, road_cfg, params);
    z_road_surface = z0f_road; % Real road elevation matching simulation

    % Asphalt Road Layer (dark paved surface)
    z_road_bed = -0.35;
    patch('XData', [x_road, fliplr(x_road)], ...
          'YData', [z_road_surface, repmat(z_road_bed, 1, length(x_road))], ...
          'FaceColor', [0.22, 0.24, 0.27], 'EdgeColor', 'none', 'Parent', ax);

    % Surface edge outline
    plot(ax, x_road, z_road_surface, 'Color', [0.10, 0.12, 0.14], 'LineWidth', 2.0);

    % Road White Dashed Centerline
    stripe_len  = 1.2;
    stripe_gap  = 1.2;
    stripe_step = stripe_len + stripe_gap;
    str_x_start = floor(x_view_min / stripe_step) * stripe_step;

    for xs = str_x_start : stripe_step : x_view_max
        x_st = linspace(xs, xs + stripe_len, 20);
        % Evaluate road elevation along stripe
        t_st = x_st / road_cfg.v;
        [z_st, ~, ~, ~] = get_road_profile(t_st, road_cfg, params);
        plot(ax, x_st, z_st - 0.15, 'Color', [0.95, 0.95, 0.95], 'LineWidth', 2.5);
    end

    %% 6. Underground Foundation with Diagonal Hatching (matches reference image style)
    patch('XData', [x_view_min, x_view_max, x_view_max, x_view_min], ...
          'YData', [z_road_bed, z_road_bed, z_earth_bot, z_earth_bot], ...
          'FaceColor', [0.92, 0.92, 0.92], 'EdgeColor', [0.70, 0.70, 0.70], ...
          'LineWidth', 1.0, 'Parent', ax);

    % Diagonal hatching lines (similar to engineering ground symbol in reference)
    hatch_spacing = 0.50;
    hatch_x_start = floor((x_view_min - 2) / hatch_spacing) * hatch_spacing;
    for xh = hatch_x_start : hatch_spacing : (x_view_max + 2)
        x1 = xh;
        z1 = z_road_bed;
        x2 = xh - 0.75;
        z2 = z_earth_bot;
        plot(ax, [x1, x2], [z1, z2], 'Color', [0.75, 0.75, 0.75], 'LineWidth', 0.9);
    end
end

%% Helper: Render a Puffy Cloud
function draw_single_cloud(ax, xc, zc, w_cloud, h_cloud)
    % 3 overlapping ellipses
    th = linspace(0, 2*pi, 30);
    c_cloud = [1.0, 1.0, 1.0]; % Soft white

    % Center lobe
    x1 = xc + (w_cloud * 0.45) * cos(th);
    z1 = zc + (h_cloud * 0.45) * sin(th);
    patch('XData', x1, 'YData', z1, 'FaceColor', c_cloud, 'FaceAlpha', 0.88, 'EdgeColor', 'none', 'Parent', ax);

    % Left lobe
    x2 = (xc - w_cloud * 0.28) + (w_cloud * 0.32) * cos(th);
    z2 = (zc - h_cloud * 0.12) + (h_cloud * 0.32) * sin(th);
    patch('XData', x2, 'YData', z2, 'FaceColor', c_cloud, 'FaceAlpha', 0.88, 'EdgeColor', 'none', 'Parent', ax);

    % Right lobe
    x3 = (xc + w_cloud * 0.28) + (w_cloud * 0.34) * cos(th);
    z3 = (zc - h_cloud * 0.08) + (h_cloud * 0.34) * sin(th);
    patch('XData', x3, 'YData', z3, 'FaceColor', c_cloud, 'FaceAlpha', 0.88, 'EdgeColor', 'none', 'Parent', ax);
end

%% Helper: Render an Outdoor Tree
function draw_tree(ax, xt, z_base)
    % Tree trunk (wood brown)
    trunk_w = 0.16;
    trunk_h = 0.65;
    patch('XData', [xt - trunk_w/2, xt + trunk_w/2, xt + trunk_w/2, xt - trunk_w/2], ...
          'YData', [z_base, z_base, z_base + trunk_h, z_base + trunk_h], ...
          'FaceColor', [0.45, 0.32, 0.20], 'EdgeColor', [0.35, 0.22, 0.12], ...
          'LineWidth', 0.8, 'Parent', ax);

    % Foliage layers (layered green canopy)
    th = linspace(0, 2*pi, 25);
    R_crown = 0.52;

    % Lower darker green crown
    x_c1 = xt + R_crown * cos(th);
    z_c1 = (z_base + trunk_h + 0.15) + (R_crown * 0.70) * sin(th);
    patch('XData', x_c1, 'YData', z_c1, ...
          'FaceColor', [0.18, 0.52, 0.22], 'EdgeColor', 'none', 'Parent', ax);

    % Upper vibrant green crown
    x_c2 = xt + (R_crown * 0.80) * cos(th);
    z_c2 = (z_base + trunk_h + 0.45) + (R_crown * 0.65) * sin(th);
    patch('XData', x_c2, 'YData', z_c2, ...
          'FaceColor', [0.28, 0.68, 0.28], 'EdgeColor', 'none', 'Parent', ax);
end
