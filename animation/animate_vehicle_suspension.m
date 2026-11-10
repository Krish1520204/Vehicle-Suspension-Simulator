function anim = animate_vehicle_suspension(sim, anim_cfg)
% ANIMATE_VEHICLE_SUSPENSION Renders a synchronized physics-driven 2D animation
% of the 4-DOF half-car suspension system traversing road bumps.
%
% Visual features:
%   - Recognizable modern sedan with body, windows, doors, wheels, headlights, and CG marker
%   - Mechanical coil springs and dashpot dampers compressing and extending dynamically
%   - Independent front and rear wheel hop motion driven by simulation states z_uf and z_ur
%   - Sprung mass heave (z_s) and pitch (theta) calculated from ODE45
%   - Realistic outdoor scenery: sky gradient, clouds, buildings, trees, and road with bumps
%   - Car occupies ~25% of the horizontal window width with horizontal camera tracking
%   - Real-time engineering telemetry HUD overlay
%
% Inputs:
%   sim      - Simulation results struct from run_simulation.m
%   anim_cfg - (Optional) Configuration struct:
%                .playback_speed : Playback speed multiplier [default: 0.6]
%                .fps            : Frames per second [default: 30]
%                .save_gif       : Boolean, export animated GIF [default: true]
%                .gif_filename   : Output GIF path [default: 'outputs/suspension_animation.gif']
%                .save_snapshot  : Boolean, export snapshot frame [default: true]
%                .snapshot_name  : Snapshot image path [default: 'outputs/suspension_animation_frame.png']
%                .show_labels    : Render dimension lines and CG label [default: true]
%
% Output:
%   anim     - Struct containing figure handle and export status

    if nargin < 2; anim_cfg = struct(); end
    if ~isfield(anim_cfg, 'playback_speed'); anim_cfg.playback_speed = 0.60; end
    if ~isfield(anim_cfg, 'fps');            anim_cfg.fps            = 30;   end
    if ~isfield(anim_cfg, 'save_gif');       anim_cfg.save_gif       = true; end
    if ~isfield(anim_cfg, 'gif_filename');   anim_cfg.gif_filename   = 'outputs/suspension_animation.gif'; end
    if ~isfield(anim_cfg, 'save_snapshot');  anim_cfg.save_snapshot  = true; end
    if ~isfield(anim_cfg, 'snapshot_name');  anim_cfg.snapshot_name  = 'outputs/suspension_animation_frame.png'; end
    if ~isfield(anim_cfg, 'show_labels');    anim_cfg.show_labels    = true; end

    % Extract simulation arrays
    t_sim = sim.t;
    p     = sim.params;
    r_cfg = sim.road_cfg;

    % Vehicle physical parameters
    a     = p.a;     % Distance to front axle (1.15 m)
    b     = p.b;     % Distance to rear axle (1.45 m)
    L     = p.L;     % Wheelbase (2.60 m)
    v     = r_cfg.v; % Forward velocity (m/s)
    R_w   = 0.32;    % Wheel radius (m)
    h_cg_static = 0.58; % Static CG height above ground level (m)
    z_mount_local = 0.18; % Upper suspension mount height relative to CG

    % Total car length is ~4.60 m
    % To keep the car at ~25% of window width: Window Width = 4.60 / 0.25 = 18.4 m
    window_span_x = 18.4;
    cam_lead_x    = 10.0; % Forward view margin ahead of CG
    cam_trail_x   = window_span_x - cam_lead_x; % 8.4 m behind CG

    % Uniform resampled time vector for smooth fixed-frame animation
    fps    = anim_cfg.fps;
    t_end  = t_sim(end);
    t_anim = 0 : (1/fps) : t_end;
    N_frames = length(t_anim);

    % Interpolate simulation states to uniform animation time
    z_s_anim   = interp1(t_sim, sim.z_s,   t_anim, 'pchip');
    theta_anim = interp1(t_sim, sim.theta, t_anim, 'pchip');
    z_uf_anim  = interp1(t_sim, sim.z_uf,  t_anim, 'pchip');
    z_ur_anim  = interp1(t_sim, sim.z_ur,  t_anim, 'pchip');
    ddz_s_anim = interp1(t_sim, sim.ddz_s, t_anim, 'pchip');
    susp_f_anim= interp1(t_sim, sim.susp_def_f, t_anim, 'pchip');
    susp_r_anim= interp1(t_sim, sim.susp_def_r, t_anim, 'pchip');

    % Create figure window
    fig = figure('Name', '4-DOF Vehicle Suspension Simulator: Dynamic Animation', ...
                 'NumberTitle', 'off', ...
                 'Color', [1 1 1], ...
                 'Position', [60, 40, 1360, 760]);
    ax = axes('Parent', fig);
    set(ax, 'Units', 'pixels', 'Position', [70, 50, 1220, 640]);

    % Ensure outputs directory exists
    script_dir = fileparts(mfilename('fullpath'));
    proj_root  = fileparts(script_dir);
    out_dir    = fullfile(proj_root, 'outputs');
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    gif_full_path      = fullfile(proj_root, anim_cfg.gif_filename);
    snapshot_full_path = fullfile(proj_root, anim_cfg.snapshot_name);

    fprintf('Starting animation rendering (%d frames at %d FPS)...\n', N_frames, fps);

    % Select snapshot frame index (when vehicle is actively negotiating mid-bumps)
    % Typically near t = 1.0 s or maximum suspension deflection
    [~, snap_frame_idx] = max(abs(susp_f_anim));
    if snap_frame_idx < 10 || snap_frame_idx > N_frames - 10
        snap_frame_idx = round(N_frames * 0.40);
    end

    % Main Animation Loop
    for k = 1:N_frames
        tk = t_anim(k);

        % 1. Vehicle Rigid Body Kinematics
        % Longitudinal position of CG: x_cg = v * t
        X_cg = v * tk;
        Z_cg = h_cg_static + z_s_anim(k);
        th_k = theta_anim(k);

        % Rotation matrix for current body pitch
        R_pitch = [cos(th_k), -sin(th_k);
                   sin(th_k),  cos(th_k)];

        % 2. Independent Wheel Hub Kinematics (Unsprung Masses)
        % Front wheel hub center
        X_wf = X_cg + a * cos(th_k);
        Z_wf = R_w  + z_uf_anim(k);

        % Rear wheel hub center
        X_wr = X_cg - b * cos(th_k);
        Z_wr = R_w  + z_ur_anim(k);

        % 3. Upper Suspension Mount Points on Rotated Chassis
        P_mount_f_local = [a; z_mount_local];
        P_mount_r_local = [-b; z_mount_local];
        P_mount_f = [X_cg; Z_cg] + R_pitch * P_mount_f_local;
        P_mount_r = [X_cg; Z_cg] + R_pitch * P_mount_r_local;

        % Wheel rolling rotation angle: phi = -X / R_w
        rot_angle = -X_cg / R_w;

        % 4. Camera View Limits (Car is 25% of visible width)
        x_min_cam = X_cg - cam_trail_x;
        x_max_cam = X_cg + cam_lead_x;
        z_min_cam = -0.75;
        z_max_cam = 3.60;

        % Clear axes for fresh frame
        cla(ax);
        hold(ax, 'on');

        % 5. Render Outdoor Background Scenery & Road Bumps
        draw_outdoor_scenery(ax, x_min_cam, x_max_cam, r_cfg, p);

        % 6. Render Mechanical Suspension Elements (Springs & Dampers)
        % Drawn inside the wheel openings connecting hub to body mount
        draw_suspension_spring_damper(ax, [X_wf, Z_wf], P_mount_f');
        draw_suspension_spring_damper(ax, [X_wr, Z_wr], P_mount_r');

        % 7. Render Front & Rear Wheels
        draw_wheel(ax, [X_wf, Z_wf], rot_angle, R_w, 0.20);
        draw_wheel(ax, [X_wr, Z_wr], rot_angle, R_w, 0.20);

        % 8. Render Main Sedan Body, Windows, Doors, Lights, & CG Marker
        draw_sedan_body(ax, X_cg, Z_cg, th_k, a, b, anim_cfg.show_labels);

        % 9. Telemetry & Simulation Data HUD (Top Left Overlay)
        hud_str = sprintf(['{\\bf VEHICLE SUSPENSION DYNAMICS HUD}\n' ...
                           'Time (t)         : %5.2f s\n' ...
                           'Forward Velocity : %5.1f km/h\n' ...
                           'Body Heave (z_s) : %+6.1f mm\n' ...
                           'Pitch Angle (\\theta): %+6.2f^\\circ\n' ...
                           'Heave Accel      : %+6.2f m/s^2\n' ...
                           'Front Travel     : %+6.1f mm\n' ...
                           'Rear Travel      : %+6.1f mm\n' ...
                           'Front Wheel Hop  : %+6.1f mm\n' ...
                           'Rear Wheel Hop   : %+6.1f mm'], ...
                           tk, v * 3.6, z_s_anim(k)*1000, th_k*(180/pi), ...
                           ddz_s_anim(k), susp_f_anim(k)*1000, susp_r_anim(k)*1000, ...
                           z_uf_anim(k)*1000, z_ur_anim(k)*1000);

        text(ax, 0.02, 0.96, hud_str, 'Units', 'normalized', ...
             'FontSize', 9, 'FontName', 'Courier New', 'FontWeight', 'bold', ...
             'BackgroundColor', [0.97, 0.98, 1.0], ...
             'EdgeColor', [0.3, 0.45, 0.65], 'LineWidth', 1.2, ...
             'VerticalAlignment', 'top');

        % Top Right Scenario Badge
        info_badge = sprintf(['Vehicle: %s Sedan (m_s = %.0f kg)\n' ...
                              'Road Profile: %s (%d Bumps, H = %.0f mm)\n' ...
                              'Scale: Car \\approx 25%% Screen Width'], ...
                              upper(p.preset_name), p.m_s, upper(r_cfg.type), ...
                              r_cfg.num_bumps, r_cfg.H * 1000);
        text(ax, 0.98, 0.96, info_badge, 'Units', 'normalized', ...
             'FontSize', 9, 'HorizontalAlignment', 'right', ...
             'BackgroundColor', [1.0, 1.0, 1.0], ...
             'EdgeColor', [0.7 0.7 0.7], 'VerticalAlignment', 'top');

        % Set view limits and clean box
        xlim(ax, [x_min_cam, x_max_cam]);
        ylim(ax, [z_min_cam, z_max_cam]);
        daspect(ax, [1, 1, 1]);
        axis(ax, 'off');
        hold(ax, 'off');

        drawnow;

        % Capture snapshot frame at peak suspension activity
        if anim_cfg.save_snapshot && k == snap_frame_idx
            try
                exportgraphics(fig, snapshot_full_path, 'Resolution', 250);
                fprintf('Captured representative animation snapshot frame at t = %.2f s -> %s\n', ...
                        tk, snapshot_full_path);
            catch
                saveas(fig, snapshot_full_path);
            end
        end

        % Save animated GIF frame (sampling every 2 frames for compact file size)
        if anim_cfg.save_gif && mod(k, 2) == 1
            frame = getframe(fig);
            im = frame2im(frame);
            [imind, cm] = rgb2ind(im, 256);
            gif_delay = (2 / fps) / anim_cfg.playback_speed;

            if k == 1
                imwrite(imind, cm, gif_full_path, 'gif', 'Loopcount', inf, 'DelayTime', gif_delay);
            else
                imwrite(imind, cm, gif_full_path, 'gif', 'WriteMode', 'append', 'DelayTime', gif_delay);
            end
        end
    end

    fprintf('Animation complete. GIF saved to: %s\n', gif_full_path);
    anim.fig = fig;
    anim.gif_path = gif_full_path;
    anim.snapshot_path = snapshot_full_path;
end
