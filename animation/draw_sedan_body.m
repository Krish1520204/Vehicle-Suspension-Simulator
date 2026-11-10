function draw_sedan_body(ax, X_cg, Z_cg, theta, a, b, show_labels)
% DRAW_SEDAN_BODY Renders an aerodynamic sedan in world coordinates rotated by pitch theta.
%
% Visual style closely matches the reference engineering diagram:
%   - Sky-blue curved sedan body profile
%   - Front and rear side windows with B-pillar
%   - Door handles and panel seam lines
%   - Aerodynamic headlights and taillights
%   - Wheel arches with generous suspension clearance
%   - Center of Gravity (CG) marker (quartered circle) with L_f, L_r, and theta indicators
%
% Inputs:
%   ax          - Axes handle
%   X_cg, Z_cg  - World coordinates of vehicle Center of Gravity (m)
%   theta       - Pitch angle of vehicle sprung mass (radians, positive nose-up)
%   a           - Distance from CG to front axle (m)
%   b           - Distance from CG to rear axle (m)
%   show_labels - (Optional) Boolean to render CG, dimension lines, and labels [default: true]

    if nargin < 7 || isempty(show_labels); show_labels = true; end

    % Rotation matrix for pitch angle theta
    R_mat = [cos(theta), -sin(theta);
             sin(theta),  cos(theta)];

    % Helper inline function to transform local (x', z') coordinates into world coordinates
    transform = @(xy) (R_mat * xy' + [X_cg; Z_cg])';

    % Color Palette
    c_body_paint    = [0.55, 0.77, 0.92]; % Sky-blue body paint (matches reference)
    c_body_outline  = [0.38, 0.58, 0.75]; % Slightly darker outline
    c_window_glass  = [0.91, 0.96, 0.99]; % Light tinted window glass
    c_window_frame  = [0.30, 0.45, 0.60];
    c_headlight_glow= [1.00, 0.92, 0.45]; % Warm headlight lens
    c_taillight_red = [0.90, 0.20, 0.20]; % Red rear taillight
    c_door_handle   = [0.45, 0.68, 0.85];

    %% 1. Define Exterior Sedan Body Contour (Facing Right)
    % Coordinates in meters relative to CG (0, 0)
    % Front axle is at +a, rear axle is at -b

    % Rear wheel arch parameters
    R_arch = 0.42;
    theta_rear_arch = linspace(pi, 0, 16);
    x_rear_arch = -b + R_arch * cos(theta_rear_arch);
    z_rear_arch = -0.22 + R_arch * sin(theta_rear_arch);

    % Front wheel arch parameters
    theta_front_arch = linspace(pi, 0, 16);
    x_front_arch = a + R_arch * cos(theta_front_arch);
    z_front_arch = -0.22 + R_arch * sin(theta_front_arch);

    body_pts = [
        % Front lower chin & bumper
        2.05, -0.22;
        2.22, -0.12;
        2.28,  0.05;
        2.25,  0.15; % Front nose tip
        % Hood
        2.15,  0.25;
        1.65,  0.31;
        0.85,  0.40; % Windshield base
        % A-Pillar & Windshield
        0.25,  0.88;
        % Roof
        -0.15, 0.91;
        -0.65, 0.89;
        % C-Pillar & Fastback Rear Glass
        -1.35, 0.46;
        % Trunk deck & integrated spoiler
        -1.95, 0.41;
        -2.22, 0.38;
        -2.28, 0.24; % Rear tail
        % Rear bumper
        -2.28, -0.05;
        -2.15, -0.20;
        -1.95, -0.22; % Rear lower skirt
    ];

    % Append rear wheel arch (from left to right along bottom)
    for k = 1:length(x_rear_arch)
        body_pts = [body_pts; x_rear_arch(k), z_rear_arch(k)];
    end

    % Rocker panel (side skirt between arches)
    body_pts = [body_pts;
        -b + R_arch, -0.22;
         a - R_arch, -0.22;
    ];

    % Append front wheel arch
    for k = 1:length(x_front_arch)
        body_pts = [body_pts; x_front_arch(k), z_front_arch(k)];
    end

    % Front lower chin connection
    body_pts = [body_pts;
        a + R_arch, -0.22;
        2.05,       -0.22
    ];

    % Transform and render main car body
    P_body_world = transform(body_pts);
    patch('XData', P_body_world(:,1), 'YData', P_body_world(:,2), ...
          'FaceColor', c_body_paint, 'EdgeColor', c_body_outline, ...
          'LineWidth', 2.0, 'Parent', ax);

    %% 2. Windows (Front and Rear Side Windows with B-Pillar)
    % Front side window
    front_win = [
         0.80,  0.42;
         0.28,  0.84;
        -0.03,  0.86;
        -0.03,  0.42
    ];
    P_fwin = transform(front_win);
    patch('XData', P_fwin(:,1), 'YData', P_fwin(:,2), ...
          'FaceColor', c_window_glass, 'EdgeColor', c_window_frame, ...
          'LineWidth', 1.5, 'Parent', ax);

    % Rear side window
    rear_win = [
        -0.10,  0.42;
        -0.10,  0.86;
        -0.65,  0.84;
        -1.25,  0.47;
        -0.80,  0.42
    ];
    P_rwin = transform(rear_win);
    patch('XData', P_rwin(:,1), 'YData', P_rwin(:,2), ...
          'FaceColor', c_window_glass, 'EdgeColor', c_window_frame, ...
          'LineWidth', 1.5, 'Parent', ax);

    % Window tint reflections
    plot(ax, [P_fwin(1,1), P_fwin(2,1)], [P_fwin(1,2), P_fwin(2,2)], ...
         'Color', [1 1 1 0.7], 'LineWidth', 2.5);

    %% 3. Door Seam Lines & Door Handles
    % B-pillar door divider line
    p_bp_top = transform([-0.065, 0.86]);
    p_bp_bot = transform([-0.065, -0.22]);
    plot(ax, [p_bp_top(1), p_bp_bot(1)], [p_bp_top(2), p_bp_bot(2)], ...
         'Color', c_body_outline, 'LineWidth', 1.2);

    % Front door handle
    h_f = transform([0.35, 0.22]);
    rectangle(ax, 'Position', [h_f(1)-0.06, h_f(2)-0.015, 0.12, 0.03], ...
              'Curvature', [1, 1], 'FaceColor', c_door_handle, 'EdgeColor', c_body_outline);

    % Rear door handle
    h_r = transform([-0.55, 0.22]);
    rectangle(ax, 'Position', [h_r(1)-0.06, h_r(2)-0.015, 0.12, 0.03], ...
              'Curvature', [1, 1], 'FaceColor', c_door_handle, 'EdgeColor', c_body_outline);

    %% 4. Headlights and Taillights
    % Headlight
    headlight = [
        2.24,  0.08;
        2.25,  0.18;
        2.10,  0.24;
        2.05,  0.15
    ];
    P_hl = transform(headlight);
    patch('XData', P_hl(:,1), 'YData', P_hl(:,2), ...
          'FaceColor', c_headlight_glow, 'EdgeColor', [0.8 0.7 0.2], ...
          'LineWidth', 1.0, 'Parent', ax);

    % Taillight
    taillight = [
        -2.28,  0.18;
        -2.28,  0.30;
        -2.15,  0.35;
        -2.12,  0.22
    ];
    P_tl = transform(taillight);
    patch('XData', P_tl(:,1), 'YData', P_tl(:,2), ...
          'FaceColor', c_taillight_red, 'EdgeColor', [0.6 0.1 0.1], ...
          'LineWidth', 1.0, 'Parent', ax);

    %% 5. Center of Gravity (CG) Marker & Engineering Indicators
    if show_labels
        % Quartered black/white CG symbol
        R_cg = 0.075;
        th_q = linspace(0, pi/2, 20);

        % Quadrant 1 (White): 0 to pi/2
        q1 = transform([R_cg * cos(th_q); R_cg * sin(th_q)]');
        patch('XData', [X_cg; q1(:,1); X_cg], 'YData', [Z_cg; q1(:,2); Z_cg], ...
              'FaceColor', [1 1 1], 'EdgeColor', 'k', 'LineWidth', 1.0, 'Parent', ax);

        % Quadrant 2 (Black): pi/2 to pi
        th_q2 = th_q + pi/2;
        q2 = transform([R_cg * cos(th_q2); R_cg * sin(th_q2)]');
        patch('XData', [X_cg; q2(:,1); X_cg], 'YData', [Z_cg; q2(:,2); Z_cg], ...
              'FaceColor', [0.15 0.15 0.15], 'EdgeColor', 'k', 'LineWidth', 1.0, 'Parent', ax);

        % Quadrant 3 (White): pi to 3pi/2
        th_q3 = th_q + pi;
        q3 = transform([R_cg * cos(th_q3); R_cg * sin(th_q3)]');
        patch('XData', [X_cg; q3(:,1); X_cg], 'YData', [Z_cg; q3(:,2); Z_cg], ...
              'FaceColor', [1 1 1], 'EdgeColor', 'k', 'LineWidth', 1.0, 'Parent', ax);

        % Quadrant 4 (Black): 3pi/2 to 2pi
        th_q4 = th_q + 3*pi/2;
        q4 = transform([R_cg * cos(th_q4); R_cg * sin(th_q4)]');
        patch('XData', [X_cg; q4(:,1); X_cg], 'YData', [Z_cg; q4(:,2); Z_cg], ...
              'FaceColor', [0.15 0.15 0.15], 'EdgeColor', 'k', 'LineWidth', 1.0, 'Parent', ax);

        % Dimension Arrows: L_f (a) to front axle and L_r (b) to rear axle
        P_axle_f = transform([a, 0]);
        P_axle_r = transform([-b, 0]);

        % Centerline dimension axis
        plot(ax, [P_axle_r(1), P_axle_f(1)], [P_axle_r(2), P_axle_f(2)], ...
             'k--', 'LineWidth', 1.0);

        % Arrows
        quiver(ax, X_cg, Z_cg, P_axle_f(1) - X_cg, P_axle_f(2) - Z_cg, 0, ...
               'Color', 'k', 'LineWidth', 1.2, 'MaxHeadSize', 0.15);
        quiver(ax, X_cg, Z_cg, P_axle_r(1) - X_cg, P_axle_r(2) - Z_cg, 0, ...
               'Color', 'k', 'LineWidth', 1.2, 'MaxHeadSize', 0.15);

        % Dimension Labels
        text(ax, (X_cg + P_axle_f(1))/2, Z_cg + 0.10, sprintf('L_f = %.2f m', a), ...
             'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
        text(ax, (X_cg + P_axle_r(1))/2, Z_cg + 0.10, sprintf('L_r = %.2f m', b), ...
             'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

        % Pitch Angle theta curved arrow indicator above CG
        arc_r = 0.35;
        arc_th = linspace(pi/4, 3*pi/4, 25);
        P_arc = transform([arc_r * cos(arc_th); arc_r * sin(arc_th) + 0.10]');
        plot(ax, P_arc(:,1), P_arc(:,2), 'Color', [0.5 0.5 0.5], 'LineWidth', 1.8);
        text(ax, X_cg, Z_cg + 0.52, '\theta', 'FontSize', 12, 'FontWeight', 'bold', ...
             'HorizontalAlignment', 'center', 'Color', [0.2 0.2 0.2]);
    end
end
