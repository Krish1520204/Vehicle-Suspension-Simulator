function draw_wheel(ax, center_xy, rotation_angle, R_wheel, R_rim)
% DRAW_WHEEL Renders a realistic automotive wheel with rubber tire, rim, and rotating spokes.
%
% Inputs:
%   ax             - Axes handle
%   center_xy      - [X, Z] coordinates of wheel center (m)
%   rotation_angle - Current wheel rotation angle in radians (for rolling effect)
%   R_wheel        - Outer tire radius [default: 0.32 m]
%   R_rim          - Inner rim radius [default: 0.20 m]

    if nargin < 4 || isempty(R_wheel); R_wheel = 0.32; end
    if nargin < 5 || isempty(R_rim);   R_rim   = 0.20; end

    Xc = center_xy(1);
    Zc = center_xy(2);

    theta = linspace(0, 2*pi, 60);

    % Colors
    c_tire_rubber = [0.22, 0.23, 0.25]; % Dark vulcanized rubber
    c_tread_edge  = [0.12, 0.13, 0.14];
    c_rim_metal   = [0.78, 0.82, 0.86]; % Machined alloy silver
    c_rim_shadow  = [0.60, 0.65, 0.70];
    c_hub_center  = [0.25, 0.30, 0.35];

    % 1. Outer Rubber Tire
    X_outer = Xc + R_wheel * cos(theta);
    Z_outer = Zc + R_wheel * sin(theta);
    patch('XData', X_outer, 'YData', Z_outer, ...
          'FaceColor', c_tire_rubber, 'EdgeColor', c_tread_edge, ...
          'LineWidth', 1.5, 'Parent', ax);

    % Tire Sidewall Highlight ring
    R_sidewall = (R_wheel + R_rim) / 2;
    plot(ax, Xc + R_sidewall * cos(theta), Zc + R_sidewall * sin(theta), ...
         ':', 'Color', [0.35, 0.36, 0.38], 'LineWidth', 0.8);

    % 2. Alloy Rim Disk
    X_rim = Xc + R_rim * cos(theta);
    Z_rim = Zc + R_rim * sin(theta);
    patch('XData', X_rim, 'YData', Z_rim, ...
          'FaceColor', c_rim_metal, 'EdgeColor', c_rim_shadow, ...
          'LineWidth', 1.2, 'Parent', ax);

    % 3. Rotating 5-Spoke Wheel Pattern
    num_spokes = 5;
    spoke_angles = rotation_angle + linspace(0, 2*pi, num_spokes + 1);
    R_spoke_inner = 0.06;
    R_spoke_outer = R_rim * 0.90;

    for i = 1:num_spokes
        phi = spoke_angles(i);
        phi_perp = phi + pi/2;
        w_spoke = 0.022; % Spoke thickness

        % Spoke polygon corners
        s1 = [Xc + R_spoke_inner * cos(phi) - w_spoke/2 * cos(phi_perp), ...
              Zc + R_spoke_inner * sin(phi) - w_spoke/2 * sin(phi_perp)];
        s2 = [Xc + R_spoke_inner * cos(phi) + w_spoke/2 * cos(phi_perp), ...
              Zc + R_spoke_inner * sin(phi) + w_spoke/2 * sin(phi_perp)];
        s3 = [Xc + R_spoke_outer * cos(phi) + w_spoke/3 * cos(phi_perp), ...
              Zc + R_spoke_outer * sin(phi) + w_spoke/3 * sin(phi_perp)];
        s4 = [Xc + R_spoke_outer * cos(phi) - w_spoke/3 * cos(phi_perp), ...
              Zc + R_spoke_outer * sin(phi) - w_spoke/3 * sin(phi_perp)];

        patch('XData', [s1(1), s2(1), s3(1), s4(1)], ...
              'YData', [s1(2), s2(2), s3(2), s4(2)], ...
              'FaceColor', [0.88, 0.92, 0.96], 'EdgeColor', [0.55, 0.60, 0.65], ...
              'LineWidth', 0.8, 'Parent', ax);
    end

    % 4. Center Hub Cap
    R_hub = 0.050;
    X_hub = Xc + R_hub * cos(theta);
    Z_hub = Zc + R_hub * sin(theta);
    patch('XData', X_hub, 'YData', Z_hub, ...
          'FaceColor', c_hub_center, 'EdgeColor', [0.15 0.15 0.20], ...
          'LineWidth', 1.0, 'Parent', ax);

    % Small center lug nuts
    num_lugs = 5;
    lug_angles = spoke_angles(1:end-1) + pi/num_spokes;
    R_lugs = 0.032;
    for k = 1:num_lugs
        xlug = Xc + R_lugs * cos(lug_angles(k));
        zlug = Zc + R_lugs * sin(lug_angles(k));
        plot(ax, xlug, zlug, 'o', 'MarkerSize', 3, 'MarkerFaceColor', [0.9 0.9 0.9], 'MarkerEdgeColor', 'k');
    end
end
