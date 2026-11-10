function draw_suspension_spring_damper(ax, P_hub, P_mount, spring_offset, damper_offset)
% DRAW_SUSPENSION_SPRING_DAMPER Renders a mechanical coil spring and telescoping damper.
%
% Dynamically stretches and compresses based on the current hub and body mount positions.
%
% Inputs:
%   ax            - Axes handle
%   P_hub         - [X, Z] lower mount position at the wheel hub center (m)
%   P_mount       - [X, Z] upper mount position on the vehicle chassis (m)
%   spring_offset - (Optional) Lateral offset for spring centerline [default: -0.05 m]
%   damper_offset - (Optional) Lateral offset for damper centerline [default: +0.05 m]

    if nargin < 4 || isempty(spring_offset); spring_offset = -0.055; end
    if nargin < 5 || isempty(damper_offset); damper_offset = +0.055; end

    % Vector from hub to mount
    V = P_mount - P_hub;
    L_curr = norm(V);
    if L_curr < 0.05; return; end
    u_axis = V / L_curr;                       % Unit vector along suspension strut
    u_perp = [-u_axis(2), u_axis(1)];          % Unit vector perpendicular to strut

    % Color palette
    c_spring = [0.20, 0.30, 0.40];             % Metallic dark slate
    c_damper_cyl = [0.35, 0.40, 0.45];         % Damper body
    c_damper_rod = [0.85, 0.88, 0.92];         % Chrome piston rod

    %% 1. Draw Coil Spring (Zigzag representation)
    P_s_bottom = P_hub   + spring_offset * u_perp;
    P_s_top    = P_mount + spring_offset * u_perp;

    N_coils = 7;
    R_coil  = 0.045; % Coil half-width in meters

    % Normalized coordinates along spring: [0 to 1]
    % Straight ends: 10% bottom, 10% top; active coils: 80%
    s_frac = [0.0, 0.10];
    w_frac = [0.0, 0.0];

    coil_steps = linspace(0.12, 0.88, N_coils * 2);
    for k = 1:length(coil_steps)
        s_frac = [s_frac, coil_steps(k)];
        if mod(k, 2) == 1
            w_frac = [w_frac, +R_coil];
        else
            w_frac = [w_frac, -R_coil];
        end
    end
    s_frac = [s_frac, 0.90, 1.0];
    w_frac = [w_frac, 0.0, 0.0];

    % Convert to world coordinates
    X_spring = P_s_bottom(1) + s_frac * (P_s_top(1) - P_s_bottom(1)) + w_frac * u_perp(1);
    Z_spring = P_s_bottom(2) + s_frac * (P_s_top(2) - P_s_bottom(2)) + w_frac * u_perp(2);

    % Draw spring shadow and main coil
    plot(ax, X_spring, Z_spring, 'Color', c_spring, 'LineWidth', 2.8);
    plot(ax, X_spring, Z_spring, 'Color', [0.45, 0.65, 0.85], 'LineWidth', 1.2);

    %% 2. Draw Telescoping Damper (Dashpot)
    P_d_bottom = P_hub   + damper_offset * u_perp;
    P_d_top    = P_mount + damper_offset * u_perp;

    % Dimensions
    R_cyl = 0.025;   % Cylinder radius (half-width)
    R_rod = 0.012;   % Piston rod radius
    L_cyl = L_curr * 0.52; % Cylinder occupies ~52% of strut length

    % Lower Cylinder (attached to lower hub mount)
    P_cyl_top = P_d_bottom + L_cyl * u_axis;

    % Cylinder polygon: 4 corners
    c1 = P_d_bottom - R_cyl * u_perp;
    c2 = P_d_bottom + R_cyl * u_perp;
    c3 = P_cyl_top  + R_cyl * u_perp;
    c4 = P_cyl_top  - R_cyl * u_perp;

    % Upper Piston Rod (extends from upper body mount down into cylinder)
    r1 = P_cyl_top  - R_rod * u_perp;
    r2 = P_cyl_top  + R_rod * u_perp;
    r3 = P_d_top    + R_rod * u_perp;
    r4 = P_d_top    - R_rod * u_perp;

    % Render Rod
    patch('XData', [r1(1), r2(1), r3(1), r4(1)], ...
          'YData', [r1(2), r2(2), r3(2), r4(2)], ...
          'FaceColor', c_damper_rod, 'EdgeColor', [0.5 0.5 0.5], ...
          'LineWidth', 1.0, 'Parent', ax);

    % Render Cylinder Housing
    patch('XData', [c1(1), c2(1), c3(1), c4(1)], ...
          'YData', [c1(2), c2(2), c3(2), c4(2)], ...
          'FaceColor', c_damper_cyl, 'EdgeColor', [0.2 0.2 0.25], ...
          'LineWidth', 1.2, 'Parent', ax);

    % Seal ring highlight at cylinder mouth
    plot(ax, [c3(1), c4(1)], [c3(2), c4(2)], 'Color', [0.15, 0.15, 0.20], 'LineWidth', 2.0);

    %% 3. Mount Bushings (Top and Bottom Pins)
    draw_pin_bushing(ax, P_s_bottom, 0.018);
    draw_pin_bushing(ax, P_s_top,    0.018);
    draw_pin_bushing(ax, P_d_bottom, 0.018);
    draw_pin_bushing(ax, P_d_top,    0.018);
end

%% Helper: Render Pin Bushing
function draw_pin_bushing(ax, pt, r)
    th = linspace(0, 2*pi, 20);
    plot(ax, pt(1) + r * cos(th), pt(2) + r * sin(th), ...
         'Color', [0.2 0.2 0.2], 'LineWidth', 1.2);
end
