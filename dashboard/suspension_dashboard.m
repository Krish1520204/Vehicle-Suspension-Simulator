function fig = suspension_dashboard()
% SUSPENSION_DASHBOARD Professional Interactive MATLAB Dashboard for 4-DOF
% Vehicle Suspension System Simulator & Bump Analyzer.
%
% Visual UI design follows the reference engineering layout:
%   - Top Header Card: "BUMP ANALYZER" Title Box
%   - Left Control & Specification Panels:
%       * Road profile dropdown: Rectangular, Triangular, Half-sinusoidal
%       * Bump amplitude slider (with live mm display)
%       * Vehicle travelling speed slider (with live km/h and m/s display)
%       * Suspension damping coefficient slider (with live N*s/m display)
%       * Suspension natural frequency slider (with live Hz and calculated k_s display)
%       * Interactive Action Buttons: [Start], [Pause], [Resume], [Reset]
%       * Vehicle type selector & complete physical specifications readout grid
%       * Numerical performance indicator card (Live & Peak KPIs, ISO 2631 comfort)
%   - Right Visualization Panels:
%       * Real-time 2D Vehicle Suspension Animation (uiaxes):
%           - Realistic sedan body with windows, doors, wheels, rotating spokes, and lights
%           - Dynamic mechanical coil springs and telescoping dampers
%           - Outdoor scenery: sky gradient, clouds, buildings, trees, road, and bumps
%           - Exact 25% window scale ratio with smooth horizontal tracking
%       * 5 Live Engineering Response Graphs (uiaxes):
%           1. Bump Geometry & Road Input (front vs rear road profile)
%           2. Live Body Vertical Displacement (heave z_s, front z_sf, rear z_sr)
%           3. Live Body Acceleration (heave accel and driver position accel)
%           4. Front & Rear Suspension Deflection (with +/- 80 mm limits)
%           5. Pitch Response (pitch angle and pitch angular rate)
%           - Synchronized vertical time cursors updating in real-time across all graphs
%
% Output:
%   fig - Handle to the created uifigure

    %% 1. Establish Environment & Path Dependencies
    proj_root = fileparts(fileparts(mfilename('fullpath')));
    if isempty(proj_root)
        proj_root = pwd;
    end
    addpath(fullfile(proj_root, 'config'));
    addpath(fullfile(proj_root, 'models'));
    addpath(fullfile(proj_root, 'road_profiles'));
    addpath(fullfile(proj_root, 'simulation'));
    addpath(fullfile(proj_root, 'utils'));
    addpath(fullfile(proj_root, 'plotting'));
    addpath(fullfile(proj_root, 'analysis'));
    addpath(fullfile(proj_root, 'animation'));

    %% 2. Color Palette & Styling Tokens
    % Inspired by the reference "BUMP ANALIZER" theme with modern refined tones
    c_bg_main       = [0.88, 0.92, 0.96]; % Light blue-slate background
    c_card_bg       = [0.97, 0.98, 1.00]; % Clean white card background
    c_card_border   = [0.70, 0.78, 0.88]; % Subtle border
    c_navy_brand    = [0.06, 0.28, 0.52]; % Dark blue accent
    c_text_dark     = [0.12, 0.16, 0.22]; % Slate text
    c_text_muted    = [0.35, 0.42, 0.50]; % Muted text
    c_btn_start     = [0.12, 0.58, 0.32]; % Emerald Green
    c_btn_pause     = [0.85, 0.55, 0.10]; % Warm Amber
    c_btn_resume    = [0.08, 0.48, 0.75]; % Cerulean Blue
    c_btn_reset     = [0.45, 0.50, 0.58]; % Slate Gray

    %% 3. Calculate Adaptive Screen Sizing
    screen_size = get(groot, 'ScreenSize'); % [left, bottom, width, height]
    screen_w    = screen_size(3);
    screen_h    = screen_size(4);

    fig_w = min(1560, screen_w - 60);
    fig_h = min(940, screen_h - 80);
    fig_x = max(30, round((screen_w - fig_w) / 2));
    fig_y = max(30, round((screen_h - fig_h) / 2));

    % Create modern UIFigure
    fig = uifigure('Name', 'Vehicle Suspension Simulator - Bump Analyzer', ...
                   'Position', [fig_x, fig_y, fig_w, fig_h], ...
                   'Color', c_bg_main, ...
                   'AutoResizeChildren', 'off');

    %% 4. App State Architecture
    app = struct();
    app.fig = fig;
    app.is_simulating = false;
    app.is_playing    = false;
    app.current_frame = 1;
    app.total_frames  = 0;
    app.fps           = 25;
    app.timer         = [];

    % Default Simulation Configurations
    app.vehicle_preset = 'sedan';
    app.params         = get_vehicle_params('sedan');
    
    app.road_cfg = struct();
    app.road_cfg.type            = 'half_sine';
    app.road_cfg.num_bumps       = 6;
    app.road_cfg.H               = 0.055; % 55 mm
    app.road_cfg.W               = 0.65;  % 0.65 m
    app.road_cfg.spacing         = 1.35;  % 1.35 m
    app.road_cfg.t_start         = 0.35;
    app.road_cfg.v               = 10.0;  % 36 km/h (10 m/s)
    app.road_cfg.smoothing_ratio = 0.08;

    app.tspan    = [0, 2.2];
    app.ode_opts = odeset('RelTol', 1e-5, 'AbsTol', 1e-7, 'MaxStep', 2e-3);
    app.sim      = [];
    app.kpi      = [];
    app.t_anim   = [];

    %% 5. Build Top Header Banner ("BUMP ANALIZER")
    % Matches the distinct framed header box from the reference image
    hdr_h = 44;
    pnl_hdr = uipanel(fig, 'Position', [round(fig_w*0.28), fig_h - hdr_h - 10, round(fig_w*0.44), hdr_h], ...
                      'BackgroundColor', [1.0, 1.0, 1.0], ...
                      'BorderColor', c_navy_brand, ...
                      'BorderWidth', 2);

    uilabel(pnl_hdr, 'Position', [0, 4, pnl_hdr.Position(3), 32], ...
            'Text', 'BUMP ANALYZER', ...
            'HorizontalAlignment', 'center', ...
            'FontName', 'Arial', ...
            'FontSize', 19, ...
            'FontWeight', 'bold', ...
            'FontAngle', 'italic', ...
            'FontColor', c_navy_brand);

    %% 6. Layout Panes Geometry
    margin_x = 14;
    margin_y = 12;
    top_y    = fig_h - hdr_h - 22;

    % Left Column (Controls & Specs) Width
    left_w = 440;
    left_h = top_y - margin_y;

    % Right Area (Animation & Graphs)
    right_x = left_w + margin_x * 2;
    right_w = fig_w - right_x - margin_x;
    right_h = left_h;

    %% 7. Left Panel: Controls, Specifications & KPIs
    pnl_left = uipanel(fig, 'Position', [margin_x, margin_y, left_w, left_h], ...
                       'BackgroundColor', c_card_bg, ...
                       'BorderColor', c_card_border, ...
                       'BorderWidth', 1, ...
                       'Title', '');

    % Sub-Section A: Interactive Controls (Top of Left Panel)
    y_ctrl = left_h - 18;

    uilabel(pnl_left, 'Position', [14, y_ctrl - 20, 200, 20], ...
            'Text', 'SIMULATION CONTROLS', ...
            'FontName', 'Arial', 'FontSize', 12, 'FontWeight', 'bold', ...
            'FontColor', c_navy_brand);
    y_ctrl = y_ctrl - 28;

    % 1. Road Profile Dropdown
    uilabel(pnl_left, 'Position', [14, y_ctrl - 18, 140, 18], ...
            'Text', 'Road Profile:', 'FontWeight', 'bold', 'FontSize', 11, 'FontColor', c_text_dark);
    app.dd_profile = uidropdown(pnl_left, 'Position', [150, y_ctrl - 20, left_w - 168, 24], ...
                                'Items', {'Half-sinusoidal', 'Triangular', 'Rectangular'}, ...
                                'ItemsData', {'half_sine', 'triangular', 'rectangular'}, ...
                                'Value', 'half_sine', ...
                                'ValueChangedFcn', @(src, event) on_control_changed(fig));
    y_ctrl = y_ctrl - 34;

    % 2. Bump Amplitude Slider
    uilabel(pnl_left, 'Position', [14, y_ctrl - 18, 140, 18], ...
            'Text', 'Bump Amplitude (H):', 'FontWeight', 'bold', 'FontSize', 10.5, 'FontColor', c_text_dark);
    app.lbl_amp = uilabel(pnl_left, 'Position', [left_w - 110, y_ctrl - 18, 95, 18], ...
                          'Text', '55.0 mm', 'HorizontalAlignment', 'right', ...
                          'FontWeight', 'bold', 'FontSize', 10.5, 'FontColor', c_navy_brand);
    y_ctrl = y_ctrl - 20;
    app.sld_amp = uislider(pnl_left, 'Position', [20, y_ctrl - 10, left_w - 40, 3], ...
                           'Limits', [15, 110], 'Value', 55, ...
                           'MajorTicks', [20, 40, 60, 80, 100], ...
                           'ValueChangedFcn', @(src, event) on_amp_slider_changed(fig));
    y_ctrl = y_ctrl - 27;

    % 3. Vehicle Speed Slider
    uilabel(pnl_left, 'Position', [14, y_ctrl - 18, 150, 18], ...
            'Text', 'Vehicle Speed (v):', 'FontWeight', 'bold', 'FontSize', 10.5, 'FontColor', c_text_dark);
    app.lbl_spd = uilabel(pnl_left, 'Position', [left_w - 170, y_ctrl - 18, 155, 18], ...
                          'Text', '36.0 km/h (10.0 m/s)', 'HorizontalAlignment', 'right', ...
                          'FontWeight', 'bold', 'FontSize', 10.5, 'FontColor', c_navy_brand);
    y_ctrl = y_ctrl - 20;
    app.sld_spd = uislider(pnl_left, 'Position', [20, y_ctrl - 10, left_w - 40, 3], ...
                           'Limits', [10, 90], 'Value', 36, ...
                           'MajorTicks', [10, 30, 50, 70, 90], ...
                           'ValueChangedFcn', @(src, event) on_spd_slider_changed(fig));
    y_ctrl = y_ctrl - 27;

    % 4. Damping Coefficient Slider
    uilabel(pnl_left, 'Position', [14, y_ctrl - 18, 160, 18], ...
            'Text', 'Damping Coeff (c_sf):', 'FontWeight', 'bold', 'FontSize', 10.5, 'FontColor', c_text_dark);
    app.lbl_damp = uilabel(pnl_left, 'Position', [left_w - 150, y_ctrl - 18, 135, 18], ...
                           'Text', '1600 N·s/m', 'HorizontalAlignment', 'right', ...
                           'FontWeight', 'bold', 'FontSize', 10.5, 'FontColor', c_navy_brand);
    y_ctrl = y_ctrl - 20;
    app.sld_damp = uislider(pnl_left, 'Position', [20, y_ctrl - 10, left_w - 40, 3], ...
                            'Limits', [600, 3600], 'Value', 1600, ...
                            'MajorTicks', [800, 1600, 2400, 3200], ...
                            'ValueChangedFcn', @(src, event) on_damp_slider_changed(fig));
    y_ctrl = y_ctrl - 27;

    % 5. Natural Frequency Slider
    uilabel(pnl_left, 'Position', [14, y_ctrl - 18, 160, 18], ...
            'Text', 'Natural Freq (f_n):', 'FontWeight', 'bold', 'FontSize', 10.5, 'FontColor', c_text_dark);
    app.lbl_fn = uilabel(pnl_left, 'Position', [left_w - 170, y_ctrl - 18, 155, 18], ...
                         'Text', '1.30 Hz (k = 27.2 kN/m)', 'HorizontalAlignment', 'right', ...
                         'FontWeight', 'bold', 'FontSize', 10.5, 'FontColor', c_navy_brand);
    y_ctrl = y_ctrl - 20;
    app.sld_fn = uislider(pnl_left, 'Position', [20, y_ctrl - 10, left_w - 40, 3], ...
                          'Limits', [0.8, 2.6], 'Value', 1.30, ...
                          'MajorTicks', [0.8, 1.2, 1.6, 2.0, 2.4], ...
                          'ValueChangedFcn', @(src, event) on_fn_slider_changed(fig));
    y_ctrl = y_ctrl - 27;

    % 6. Action Control Buttons: Start, Pause, Resume, Reset
    btn_w = round((left_w - 40) / 4);
    btn_h = 28;
    app.btn_start  = uibutton(pnl_left, 'push', 'Position', [14, y_ctrl - btn_h, btn_w, btn_h], ...
                              'Text', 'Start', 'FontWeight', 'bold', 'FontSize', 11, ...
                              'BackgroundColor', c_btn_start, 'FontColor', [1, 1, 1], ...
                              'ButtonPushedFcn', @(src, event) on_btn_start(fig));

    app.btn_pause  = uibutton(pnl_left, 'push', 'Position', [14 + btn_w + 4, y_ctrl - btn_h, btn_w, btn_h], ...
                              'Text', 'Pause', 'FontWeight', 'bold', 'FontSize', 11, ...
                              'BackgroundColor', c_btn_pause, 'FontColor', [1, 1, 1], ...
                              'Enable', 'off', ...
                              'ButtonPushedFcn', @(src, event) on_btn_pause(fig));

    app.btn_resume = uibutton(pnl_left, 'push', 'Position', [14 + (btn_w + 4)*2, y_ctrl - btn_h, btn_w, btn_h], ...
                              'Text', 'Resume', 'FontWeight', 'bold', 'FontSize', 11, ...
                              'BackgroundColor', c_btn_resume, 'FontColor', [1, 1, 1], ...
                              'Enable', 'off', ...
                              'ButtonPushedFcn', @(src, event) on_btn_resume(fig));

    app.btn_reset  = uibutton(pnl_left, 'push', 'Position', [14 + (btn_w + 4)*3, y_ctrl - btn_h, btn_w, btn_h], ...
                              'Text', 'Reset', 'FontWeight', 'bold', 'FontSize', 11, ...
                              'BackgroundColor', c_btn_reset, 'FontColor', [1, 1, 1], ...
                              'ButtonPushedFcn', @(src, event) on_btn_reset(fig));
    y_ctrl = y_ctrl - btn_h - 12;

    % Sub-Section B: Vehicle Specifications Table (Matches reference image)
    uilabel(pnl_left, 'Position', [14, y_ctrl - 18, 200, 18], ...
            'Text', 'VEHICLE SPECIFICATIONS', ...
            'FontName', 'Arial', 'FontSize', 12, 'FontWeight', 'bold', ...
            'FontColor', c_navy_brand);
    y_ctrl = y_ctrl - 24;

    % Vehicle Type selector
    uilabel(pnl_left, 'Position', [14, y_ctrl - 18, 120, 18], ...
            'Text', 'Select vehicle type:', 'FontWeight', 'bold', 'FontSize', 10.5, 'FontColor', c_text_dark);
    app.dd_vehicle = uidropdown(pnl_left, 'Position', [140, y_ctrl - 20, left_w - 158, 22], ...
                                'Items', {'Vehicle of Segment C (Sedan)', 'Vehicle of Segment D (SUV)', 'Sports Coupe'}, ...
                                'ItemsData', {'sedan', 'suv', 'sport'}, ...
                                'Value', 'sedan', ...
                                'ValueChangedFcn', @(src, event) on_vehicle_changed(fig));
    y_ctrl = y_ctrl - 28;

    % 2-Column Table of Physical Parameters (Faithful to reference image layout)
    col1_x = 14;
    col2_x = round(left_w / 2) + 6;
    field_w = 64;

    app.fields = struct();
    param_list_left = {
        'Percentage of Front Weight', 'pct_front', '55.8 %';
        'Front Susp Stiffness',       'k_sf',      '27164';
        'Rear Susp Stiffness',        'k_sr',      '21544';
        'Rear Damping Coeff',         'c_sr',      '1500';
        'Front Damping Coeff',        'c_sf',      '1600';
        'Sprung Mass (kg)',           'm_s',       '730';
    };

    param_list_right = {
        'Front Unsprung mass',        'm_uf',      '45';
        'Rear Unsprung mass',         'm_ur',      '40';
        'Rear Tyre Stiffness',        'k_tr',      '190000';
        'Front Tyre Stiffness',       'k_tf',      '190000';
        'Moment of Inertia',          'I_y',       '1250';
        'Wheelbase (m)',              'L',         '2.60';
    };

    y_grid_start = y_ctrl;
    for row = 1:size(param_list_left, 1)
        y_r = y_grid_start - (row - 1)*20;
        % Left col
        uilabel(pnl_left, 'Position', [col1_x, y_r - 18, 135, 16], ...
                'Text', param_list_left{row, 1}, 'FontSize', 9.5, 'FontColor', c_text_dark);
        app.fields.(param_list_left{row, 2}) = uieditfield(pnl_left, 'text', ...
                'Position', [col1_x + 138, y_r - 18, field_w, 18], ...
                'Value', param_list_left{row, 3}, 'Editable', 'off', ...
                'FontSize', 9.5, 'HorizontalAlignment', 'center', ...
                'BackgroundColor', [0.94, 0.96, 0.98]);

        % Right col
        uilabel(pnl_left, 'Position', [col2_x, y_r - 18, 130, 16], ...
                'Text', param_list_right{row, 1}, 'FontSize', 9.5, 'FontColor', c_text_dark);
        app.fields.(param_list_right{row, 2}) = uieditfield(pnl_left, 'text', ...
                'Position', [col2_x + 132, y_r - 18, field_w, 18], ...
                'Value', param_list_right{row, 3}, 'Editable', 'off', ...
                'FontSize', 9.5, 'HorizontalAlignment', 'center', ...
                'BackgroundColor', [0.94, 0.96, 0.98]);
    end
    y_ctrl = y_grid_start - size(param_list_left, 1)*20 - 10;

    % Sub-Section C: Numerical Performance Indicators (KPIs)
    uilabel(pnl_left, 'Position', [14, y_ctrl - 18, 220, 18], ...
            'Text', 'PERFORMANCE INDICATORS (KPIS)', ...
            'FontName', 'Arial', 'FontSize', 12, 'FontWeight', 'bold', ...
            'FontColor', c_navy_brand);
    y_ctrl = y_ctrl - 20;

    pnl_kpi = uipanel(pnl_left, 'Position', [12, 8, left_w - 24, y_ctrl - 8], ...
                      'BackgroundColor', [1.0, 1.0, 1.0], ...
                      'BorderColor', c_card_border);

    app.kpi_labels = struct();
    kpi_defs = {
        'Max Body Heave Disp:',      'max_heave',   '-- mm';
        'Max Vertical Accel (CG):',   'max_accel',   '-- m/s^2 (-- g)';
        'Driver Seat Accel (RMS):',   'rms_accel',   '-- m/s^2';
        'ISO 2631 Ride Comfort:',     'comfort',     '--';
        'Max Body Pitch Angle:',      'max_pitch',   '-- deg';
        'Front Travel Util (80mm):',  'front_trav',  '-- mm (-- %)';
        'Rear Travel Util (80mm):',   'rear_trav',   '-- mm (-- %)';
        'Dynamic Road Grip Status:',  'grip_status', '--';
    };

    y_kpi_curr = pnl_kpi.Position(4) - 18;
    row_gap = 19;
    for k = 1:size(kpi_defs, 1)
        uilabel(pnl_kpi, 'Position', [8, y_kpi_curr, 155, 16], ...
                'Text', kpi_defs{k, 1}, 'FontSize', 9.2, 'FontWeight', 'bold', 'FontColor', c_text_dark);
        app.kpi_labels.(kpi_defs{k, 2}) = uilabel(pnl_kpi, ...
                'Position', [165, y_kpi_curr, pnl_kpi.Position(3) - 172, 16], ...
                'Text', kpi_defs{k, 3}, 'FontSize', 9.2, 'FontWeight', 'bold', ...
                'HorizontalAlignment', 'right', 'FontColor', c_navy_brand);
        y_kpi_curr = y_kpi_curr - row_gap;
    end

    %% 8. Right Area: Live Vehicle Animation & Engineering Response Graphs
    % Split vertically: Top = Animation UIAxes, Bottom = Response Graphs Grid
    anim_h = round(right_h * 0.40);
    graph_h = right_h - anim_h - margin_y;

    % Top Container Panel: Animation
    pnl_anim = uipanel(fig, 'Position', [right_x, right_h - anim_h + margin_y, right_w, anim_h], ...
                       'BackgroundColor', c_card_bg, ...
                       'BorderColor', c_card_border, ...
                       'Title', 'LIVE 4-DOF VEHICLE SUSPENSION ANIMATION & ROAD TRAVERSAL', ...
                       'FontWeight', 'bold', 'FontSize', 11, 'ForegroundColor', c_navy_brand);

    app.ax_anim = uiaxes(pnl_anim, 'Position', [8, 8, right_w - 18, anim_h - 32]);
    app.ax_anim.Box = 'on';
    app.ax_anim.XTick = [];
    app.ax_anim.YTick = [];
    app.ax_anim.XColor = 'none';
    app.ax_anim.YColor = 'none';

    % Bottom Container Panel: Live Graphs Grid (2 Rows x 3 Columns / 2+3 layout)
    pnl_graphs = uipanel(fig, 'Position', [right_x, margin_y, right_w, graph_h], ...
                         'BackgroundColor', c_card_bg, ...
                         'BorderColor', c_card_border, ...
                         'Title', 'LIVE DYNAMIC RESPONSE GRAPHS & BUMP GEOMETRY', ...
                         'FontWeight', 'bold', 'FontSize', 11, 'ForegroundColor', c_navy_brand);

    % Configure 5 axes inside pnl_graphs:
    % Row 1 (top): Bump Geometry (matches reference), Live Body Displacement, Live Acceleration
    % Row 2 (bot): Front/Rear Suspension Deflection, Pitch Response
    g_inner_w = right_w - 18;
    g_inner_h = graph_h - 32;

    row_h = round((g_inner_h - 28) / 2);
    col3_w = round((g_inner_w - 40) / 3);
    col2_w = round((g_inner_w - 30) / 2);

    y_row1 = row_h + 24;
    y_row2 = 12;

    % Axis 1: Bump Geometry & Road Input (Faithful to "Bump geometry" plot in reference)
    app.ax_road = uiaxes(pnl_graphs, 'Position', [12, y_row1, col3_w, row_h]);
    title(app.ax_road, 'Road Bump Geometry', 'FontSize', 10, 'FontWeight', 'bold', 'Color', c_navy_brand);
    xlabel(app.ax_road, 'Time (s)', 'FontSize', 9);
    ylabel(app.ax_road, 'Height (m)', 'FontSize', 9);
    grid(app.ax_road, 'on');

    % Axis 2: Live Body Heave Displacement
    app.ax_disp = uiaxes(pnl_graphs, 'Position', [12 + col3_w + 14, y_row1, col3_w, row_h]);
    title(app.ax_disp, 'Live Body Heave Displacement', 'FontSize', 10, 'FontWeight', 'bold', 'Color', c_navy_brand);
    xlabel(app.ax_disp, 'Time (s)', 'FontSize', 9);
    ylabel(app.ax_disp, 'Disp (mm)', 'FontSize', 9);
    grid(app.ax_disp, 'on');

    % Axis 3: Live Body Acceleration
    app.ax_accel = uiaxes(pnl_graphs, 'Position', [12 + (col3_w + 14)*2, y_row1, col3_w, row_h]);
    title(app.ax_accel, 'Live Body Acceleration (CG & Driver)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', c_navy_brand);
    xlabel(app.ax_accel, 'Time (s)', 'FontSize', 9);
    ylabel(app.ax_accel, 'Accel (m/s^2)', 'FontSize', 9);
    grid(app.ax_accel, 'on');

    % Axis 4: Front & Rear Suspension Deflection
    app.ax_defl = uiaxes(pnl_graphs, 'Position', [12, y_row2, col2_w, row_h]);
    title(app.ax_defl, 'Suspension Travel & Deflection (\pm80 mm Limits)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', c_navy_brand);
    xlabel(app.ax_defl, 'Time (s)', 'FontSize', 9);
    ylabel(app.ax_defl, 'Deflection (mm)', 'FontSize', 9);
    grid(app.ax_defl, 'on');

    % Axis 5: Pitch Response
    app.ax_pitch = uiaxes(pnl_graphs, 'Position', [12 + col2_w + 16, y_row2, col2_w, row_h]);
    title(app.ax_pitch, 'Vehicle Body Pitch Response', 'FontSize', 10, 'FontWeight', 'bold', 'Color', c_navy_brand);
    xlabel(app.ax_pitch, 'Time (s)', 'FontSize', 9);
    ylabel(app.ax_pitch, 'Pitch Angle (deg)', 'FontSize', 9);
    grid(app.ax_pitch, 'on');

    % Create moving vertical time cursors on all 5 graphs
    app.cursors = struct();
    app.cursors.road  = xline(app.ax_road, 0, 'r--', 'LineWidth', 1.6);
    app.cursors.disp  = xline(app.ax_disp, 0, 'r--', 'LineWidth', 1.6);
    app.cursors.accel = xline(app.ax_accel, 0, 'r--', 'LineWidth', 1.6);
    app.cursors.defl  = xline(app.ax_defl, 0, 'r--', 'LineWidth', 1.6);
    app.cursors.pitch = xline(app.ax_pitch, 0, 'r--', 'LineWidth', 1.6);

    %% 9. Initialize Simulation & Plot Initial Frame
    app = compute_and_update_simulation(app);
    render_animation_frame(app, 1);

    % Save app state in figure handle
    fig.UserData = app;

    % Configure clean figure shutdown to safely delete active timer
    fig.CloseRequestFcn = @(src, event) on_figure_close(fig);
end

%% =========================================================================
%  CALLBACK FUNCTIONS & EVENT HANDLERS
%  =========================================================================

function on_amp_slider_changed(fig)
    app = fig.UserData;
    val = round(app.sld_amp.Value * 10) / 10;
    app.lbl_amp.Text = sprintf('%.1f mm', val);
    app.road_cfg.H = val / 1000;
    fig.UserData = app;
end

function on_spd_slider_changed(fig)
    app = fig.UserData;
    val = round(app.sld_spd.Value);
    v_mps = val / 3.6;
    app.lbl_spd.Text = sprintf('%.1f km/h (%.1f m/s)', val, v_mps);
    app.road_cfg.v = v_mps;
    fig.UserData = app;
end

function on_damp_slider_changed(fig)
    app = fig.UserData;
    val = round(app.sld_damp.Value / 50) * 50;
    app.lbl_damp.Text = sprintf('%d N·s/m', val);
    app.params.c_sf = val;
    app.params.c_sr = round(val * (1500 / 1600));
    update_specs_table(app);
    fig.UserData = app;
end

function on_fn_slider_changed(fig)
    app = fig.UserData;
    fn = round(app.sld_fn.Value * 20) / 20;
    m_s = app.params.m_s;
    L = app.params.L;
    a = app.params.a;
    b = app.params.b;
    
    k_equiv = m_s * (2 * pi * fn)^2;
    k_sf = round(k_equiv * (b / L));
    k_sr = round(k_equiv * (a / L));
    
    app.lbl_fn.Text = sprintf('%.2f Hz (k = %.1f kN/m)', fn, k_sf / 1000);
    app.params.k_sf = k_sf;
    app.params.k_sr = k_sr;
    update_specs_table(app);
    fig.UserData = app;
end

function on_control_changed(fig)
    app = fig.UserData;
    app.road_cfg.type = app.dd_profile.Value;
    fig.UserData = app;
end

function on_vehicle_changed(fig)
    app = fig.UserData;
    preset = app.dd_vehicle.Value;
    app.vehicle_preset = preset;
    app.params = get_vehicle_params(preset);
    
    % Update natural frequency & damping sliders to match preset
    fn_nominal = (1 / (2*pi)) * sqrt((app.params.k_sf + app.params.k_sr) / app.params.m_s);
    app.sld_fn.Value = min(2.5, max(0.8, fn_nominal));
    app.lbl_fn.Text = sprintf('%.2f Hz (k = %.1f kN/m)', fn_nominal, app.params.k_sf / 1000);

    app.sld_damp.Value = min(3600, max(600, app.params.c_sf));
    app.lbl_damp.Text = sprintf('%d N·s/m', app.params.c_sf);

    update_specs_table(app);
    fig.UserData = app;
end

function update_specs_table(app)
    p = app.params;
    pct_f = (p.b / p.L) * 100;
    app.fields.pct_front.Value = sprintf('%.1f %%', pct_f);
    app.fields.k_sf.Value      = sprintf('%d', round(p.k_sf));
    app.fields.k_sr.Value      = sprintf('%d', round(p.k_sr));
    app.fields.c_sf.Value      = sprintf('%d', round(p.c_sf));
    app.fields.c_sr.Value      = sprintf('%d', round(p.c_sr));
    app.fields.m_s.Value       = sprintf('%d', round(p.m_s));
    app.fields.m_uf.Value      = sprintf('%d', round(p.m_uf));
    app.fields.m_ur.Value      = sprintf('%d', round(p.m_ur));
    app.fields.k_tf.Value      = sprintf('%d', round(p.k_tf));
    app.fields.k_tr.Value      = sprintf('%d', round(p.k_tr));
    app.fields.I_y.Value       = sprintf('%d', round(p.I_y));
    app.fields.L.Value         = sprintf('%.2f', p.L);
end

function on_btn_start(fig)
    app = fig.UserData;
    % Stop active timer if running
    stop_timer(app);

    % Run numerical ODE45 integration and plot responses
    app = compute_and_update_simulation(app);

    % Start animation playback timer
    app.current_frame = 1;
    app.is_playing    = true;
    app.btn_pause.Enable  = 'on';
    app.btn_resume.Enable = 'off';
    fig.UserData = app;

    start_playback_timer(app);
end

function on_btn_pause(fig)
    app = fig.UserData;
    stop_timer(app);
    app.is_playing = false;
    app.btn_pause.Enable  = 'off';
    app.btn_resume.Enable = 'on';
    fig.UserData = app;
end

function on_btn_resume(fig)
    app = fig.UserData;
    if ~isempty(app.sim) && app.current_frame < app.total_frames
        app.is_playing = true;
        app.btn_pause.Enable  = 'on';
        app.btn_resume.Enable = 'off';
        fig.UserData = app;
        start_playback_timer(app);
    end
end

function on_btn_reset(fig)
    app = fig.UserData;
    stop_timer(app);
    app.is_playing = false;
    app.btn_pause.Enable  = 'off';
    app.btn_resume.Enable = 'off';

    % Restore nominal default settings
    app.dd_vehicle.Value = 'sedan';
    app.vehicle_preset   = 'sedan';
    app.params           = get_vehicle_params('sedan');

    app.dd_profile.Value = 'half_sine';
    app.road_cfg.type    = 'half_sine';
    app.sld_amp.Value    = 55;
    app.lbl_amp.Text     = '55.0 mm';
    app.road_cfg.H       = 0.055;

    app.sld_spd.Value    = 36;
    app.lbl_spd.Text     = '36.0 km/h (10.0 m/s)';
    app.road_cfg.v       = 10.0;

    app.sld_damp.Value   = 1600;
    app.lbl_damp.Text    = '1600 N·s/m';

    app.sld_fn.Value     = 1.30;
    app.lbl_fn.Text      = '1.30 Hz (k = 27.2 kN/m)';

    update_specs_table(app);
    app = compute_and_update_simulation(app);
    render_animation_frame(app, 1);
    fig.UserData = app;
end

%% =========================================================================
%  SIMULATION INTEGRATION & GRAPH RENDERING
%  =========================================================================

function app = compute_and_update_simulation(app)
    % Execute 4-DOF ODE45 numerical integration
    x0 = zeros(8, 1);
    app.sim = run_simulation(app.params, app.road_cfg, app.tspan, x0, app.ode_opts);

    % Compute KPIs
    app.kpi = calculate_kpis(app.sim);
    update_kpi_card(app);

    % Pre-compute resampled time grid for 25 FPS playback
    t_end = app.sim.t(end);
    app.t_anim = 0 : (1 / app.fps) : t_end;
    app.total_frames = length(app.t_anim);
    app.current_frame = 1;

    % Resample simulation states uniformly
    app.anim_resampled = struct();
    app.anim_resampled.z_s      = interp1(app.sim.t, app.sim.z_s, app.t_anim, 'pchip');
    app.anim_resampled.theta    = interp1(app.sim.t, app.sim.theta, app.t_anim, 'pchip');
    app.anim_resampled.z_uf     = interp1(app.sim.t, app.sim.z_uf, app.t_anim, 'pchip');
    app.anim_resampled.z_ur     = interp1(app.sim.t, app.sim.z_ur, app.t_anim, 'pchip');
    app.anim_resampled.delta_sf = interp1(app.sim.t, app.sim.delta_sf, app.t_anim, 'pchip');
    app.anim_resampled.delta_sr = interp1(app.sim.t, app.sim.delta_sr, app.t_anim, 'pchip');

    % Plot full traces on the 5 response graphs
    app = plot_response_graphs(app);
end

function update_kpi_card(app)
    k = app.kpi;
    app.kpi_labels.max_heave.Text   = sprintf('%.2f mm', k.max_body_disp_mm);
    app.kpi_labels.max_accel.Text   = sprintf('%.2f m/s^2 (%.2f g)', k.max_heave_accel_mps2, k.max_heave_accel_g);
    app.kpi_labels.rms_accel.Text   = sprintf('%.2f m/s^2', k.rms_driver_accel_mps2);
    app.kpi_labels.comfort.Text     = k.iso2631_comfort;
    app.kpi_labels.max_pitch.Text   = sprintf('%.2f deg', k.max_pitch_angle_deg);
    app.kpi_labels.front_trav.Text  = sprintf('%.1f mm (%.1f %%)', k.max_travel_front_mm, k.travel_util_front_pct);
    app.kpi_labels.rear_trav.Text   = sprintf('%.1f mm (%.1f %%)', k.max_travel_rear_mm, k.travel_util_rear_pct);

    if k.min_tyre_ratio_front >= 0 && k.min_tyre_ratio_rear >= 0
        app.kpi_labels.grip_status.Text = 'Safe (Grip Maintained)';
        app.kpi_labels.grip_status.FontColor = [0.12, 0.58, 0.32];
    else
        app.kpi_labels.grip_status.Text = 'Warning (Transient Lift-off)';
        app.kpi_labels.grip_status.FontColor = [0.85, 0.20, 0.15];
    end
end

function app = plot_response_graphs(app)
    sim = app.sim;
    t   = sim.t;

    % Palette for traces
    c_blue   = [0.08, 0.45, 0.82];
    c_red    = [0.85, 0.22, 0.18];
    c_green  = [0.15, 0.65, 0.35];
    c_orange = [0.90, 0.50, 0.10];
    c_purple = [0.55, 0.20, 0.70];

    % 1. Bump Geometry & Road Input
    cla(app.ax_road);
    hold(app.ax_road, 'on');
    plot(app.ax_road, t, sim.z_0f * 1000, 'Color', c_red, 'LineWidth', 1.6, 'DisplayName', 'Front Wheel z_{0f}');
    plot(app.ax_road, t, sim.z_0r * 1000, 'Color', c_blue, 'LineWidth', 1.4, 'LineStyle', '--', 'DisplayName', 'Rear Wheel z_{0r}');
    xlabel(app.ax_road, 'Time (s)', 'FontSize', 8.5);
    ylabel(app.ax_road, 'Height (mm)', 'FontSize', 8.5);
    legend(app.ax_road, 'Location', 'northeast', 'FontSize', 7.5);
    grid(app.ax_road, 'on');
    xlim(app.ax_road, [0, t(end)]);
    app.cursors.road = xline(app.ax_road, 0, 'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');

    % 2. Live Body Heave Displacement
    cla(app.ax_disp);
    hold(app.ax_disp, 'on');
    plot(app.ax_disp, t, sim.z_s * 1000, 'Color', c_blue, 'LineWidth', 1.6, 'DisplayName', 'Heave z_s (CG)');
    plot(app.ax_disp, t, sim.z_sf * 1000, 'Color', c_orange, 'LineWidth', 1.2, 'DisplayName', 'Front Mount z_{sf}');
    plot(app.ax_disp, t, sim.z_sr * 1000, 'Color', c_purple, 'LineWidth', 1.2, 'DisplayName', 'Rear Mount z_{sr}');
    xlabel(app.ax_disp, 'Time (s)', 'FontSize', 8.5);
    ylabel(app.ax_disp, 'Disp (mm)', 'FontSize', 8.5);
    legend(app.ax_disp, 'Location', 'northeast', 'FontSize', 7.5);
    grid(app.ax_disp, 'on');
    xlim(app.ax_disp, [0, t(end)]);
    app.cursors.disp = xline(app.ax_disp, 0, 'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');

    % 3. Live Body Acceleration
    cla(app.ax_accel);
    hold(app.ax_accel, 'on');
    plot(app.ax_accel, t, sim.ddz_s, 'Color', c_red, 'LineWidth', 1.5, 'DisplayName', 'CG Heave d^2z_s/dt^2');
    plot(app.ax_accel, t, sim.ddz_driver, 'Color', c_orange, 'LineWidth', 1.2, 'LineStyle', '-.', 'DisplayName', 'Driver Seat');
    yline(app.ax_accel, 0.63, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    yline(app.ax_accel, -0.63, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    xlabel(app.ax_accel, 'Time (s)', 'FontSize', 8.5);
    ylabel(app.ax_accel, 'Accel (m/s^2)', 'FontSize', 8.5);
    legend(app.ax_accel, 'Location', 'northeast', 'FontSize', 7.5);
    grid(app.ax_accel, 'on');
    xlim(app.ax_accel, [0, t(end)]);
    app.cursors.accel = xline(app.ax_accel, 0, 'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');

    % 4. Suspension Deflection
    cla(app.ax_defl);
    hold(app.ax_defl, 'on');
    plot(app.ax_defl, t, sim.delta_sf * 1000, 'Color', c_blue, 'LineWidth', 1.5, 'DisplayName', 'Front \delta_{sf}');
    plot(app.ax_defl, t, sim.delta_sr * 1000, 'Color', c_green, 'LineWidth', 1.5, 'DisplayName', 'Rear \delta_{sr}');
    yline(app.ax_defl, 80, 'r--', 'LineWidth', 1.2, 'DisplayName', '+80mm Jounce Limit');
    yline(app.ax_defl, -80, 'r--', 'LineWidth', 1.2, 'DisplayName', '-80mm Rebound Limit');
    xlabel(app.ax_defl, 'Time (s)', 'FontSize', 8.5);
    ylabel(app.ax_defl, 'Deflection (mm)', 'FontSize', 8.5);
    legend(app.ax_defl, 'Location', 'northeast', 'FontSize', 7.5);
    grid(app.ax_defl, 'on');
    xlim(app.ax_defl, [0, t(end)]);
    ylim(app.ax_defl, [-100, 100]);
    app.cursors.defl = xline(app.ax_defl, 0, 'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');

    % 5. Pitch Response
    cla(app.ax_pitch);
    hold(app.ax_pitch, 'on');
    theta_deg = sim.theta * (180 / pi);
    dtheta_degs = sim.dtheta * (180 / pi);
    plot(app.ax_pitch, t, theta_deg, 'Color', c_purple, 'LineWidth', 1.6, 'DisplayName', 'Pitch Angle \theta (deg)');
    plot(app.ax_pitch, t, dtheta_degs, 'Color', c_orange, 'LineWidth', 1.2, 'LineStyle', '--', 'DisplayName', 'Pitch Rate d\theta/dt (deg/s)');
    xlabel(app.ax_pitch, 'Time (s)', 'FontSize', 8.5);
    ylabel(app.ax_pitch, 'Angle (deg) / Rate (deg/s)', 'FontSize', 8.5);
    legend(app.ax_pitch, 'Location', 'northeast', 'FontSize', 7.5);
    grid(app.ax_pitch, 'on');
    xlim(app.ax_pitch, [0, t(end)]);
    app.cursors.pitch = xline(app.ax_pitch, 0, 'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
end

%% =========================================================================
%  PLAYBACK TIMER & FRAME RENDERING
%  =========================================================================

function start_playback_timer(app)
    % Initialize MATLAB timer running at ~25 Hz
    stop_timer(app);
    period = max(0.035, 1 / app.fps);
    app.timer = timer('ExecutionMode', 'fixedSpacing', ...
                      'Period', period, ...
                      'BusyMode', 'drop', ...
                      'TimerFcn', @(~, ~) on_timer_tick(app.fig));
    app.fig.UserData = app;
    start(app.timer);
end

function stop_timer(app)
    if isfield(app, 'timer') && ~isempty(app.timer) && isvalid(app.timer)
        stop(app.timer);
        delete(app.timer);
    end
end

function on_timer_tick(fig)
    if ~isvalid(fig)
        return;
    end

    % Retrieve updated state from figure UserData
    app = fig.UserData;
    if isempty(app) || ~isfield(app, 'is_playing') || ~app.is_playing
        return;
    end

    if app.current_frame > app.total_frames
        % Loop animation or pause
        app.current_frame = 1;
    end

    render_animation_frame(app, app.current_frame);

    app.current_frame = app.current_frame + 1;
    fig.UserData = app;
end

function render_animation_frame(app, k)
    if isempty(app.anim_resampled); return; end

    t_now = app.t_anim(k);
    p     = app.params;
    r_cfg = app.road_cfg;
    v     = r_cfg.v;
    a     = p.a;
    b     = p.b;
    R_w   = 0.32;
    h_cg_static = 0.58;
    z_mount_local = 0.18;

    % Current kinematic state
    z_s_k   = app.anim_resampled.z_s(k);
    theta_k = app.anim_resampled.theta(k);
    z_uf_k  = app.anim_resampled.z_uf(k);
    z_ur_k  = app.anim_resampled.z_ur(k);

    % Horizontal car travel
    X_cg = v * t_now;
    Z_cg = h_cg_static + z_s_k;

    % Wheel centers
    X_wf = X_cg + a * cos(theta_k);
    Z_wf = R_w + z_uf_k;
    X_wr = X_cg - b * cos(theta_k);
    Z_wr = R_w + z_ur_k;

    % Wheel spoke rotation angle
    rot_angle = -(v * t_now) / R_w;

    % Chassis suspension mount positions
    R_mat = [cos(theta_k), -sin(theta_k); sin(theta_k), cos(theta_k)];
    mount_f_world = [X_cg; Z_cg] + R_mat * [a; z_mount_local];
    mount_r_world = [X_cg; Z_cg] + R_mat * [-b; z_mount_local];

    % Camera bounds (keeping car at 25% of window width)
    window_span_x = 18.4;
    cam_lead_x    = 10.0;
    cam_trail_x   = window_span_x - cam_lead_x;
    x_view_min    = X_cg - cam_trail_x;
    x_view_max    = X_cg + cam_lead_x;

    % Render scene in ax_anim
    cla(app.ax_anim);
    hold(app.ax_anim, 'on');

    % 1. Outdoor Background Scenery & Bumps
    draw_outdoor_scenery(app.ax_anim, x_view_min, x_view_max, r_cfg, p);

    % 2. Suspension Springs & Dampers
    draw_suspension_spring_damper(app.ax_anim, [X_wr, Z_wr], mount_r_world');
    draw_suspension_spring_damper(app.ax_anim, [X_wf, Z_wf], mount_f_world');

    % 3. Wheels
    draw_wheel(app.ax_anim, [X_wr, Z_wr], rot_angle, R_w, 0.20);
    draw_wheel(app.ax_anim, [X_wf, Z_wf], rot_angle, R_w, 0.20);

    % 4. Sedan Body & CG marker
    draw_sedan_body(app.ax_anim, X_cg, Z_cg, theta_k, a, b, true);

    % 5. Real-Time Telemetry HUD Overlay
    hud_x = x_view_min + 0.35;
    hud_str = sprintf(['{\\bf\\color[rgb]{0.06,0.28,0.52}DYNAMICS TELEMETRY}\n', ...
                       'Time: %5.2f s | Speed: %4.1f km/h\n', ...
                       'Heave z_s: %+5.1f mm | Pitch \\theta: %+4.2f^\\circ\n', ...
                       'Front Travel: %+5.1f mm | Rear: %+5.1f mm\n', ...
                       'Front Hop z_{uf}: %+4.1f mm | Rear: %+4.1f mm'], ...
                       t_now, v * 3.6, ...
                       z_s_k * 1000, theta_k * (180/pi), ...
                       app.anim_resampled.delta_sf(k) * 1000, ...
                       app.anim_resampled.delta_sr(k) * 1000, ...
                       z_uf_k * 1000, z_ur_k * 1000);

    text(app.ax_anim, hud_x, 3.42, hud_str, ...
         'FontSize', 8.5, 'FontName', 'Arial', ...
         'BackgroundColor', [1.0, 1.0, 1.0], ...
         'EdgeColor', [0.70, 0.78, 0.88], ...
         'Margin', 4, 'VerticalAlignment', 'top');

    % Axis view limits
    xlim(app.ax_anim, [x_view_min, x_view_max]);
    ylim(app.ax_anim, [-0.55, 3.65]);
    app.ax_anim.Visible = 'off';

    % Update moving time cursors on the 5 response graphs
    if isvalid(app.cursors.road);  app.cursors.road.Value  = t_now; end
    if isvalid(app.cursors.disp);  app.cursors.disp.Value  = t_now; end
    if isvalid(app.cursors.accel); app.cursors.accel.Value = t_now; end
    if isvalid(app.cursors.defl);  app.cursors.defl.Value  = t_now; end
    if isvalid(app.cursors.pitch); app.cursors.pitch.Value = t_now; end
end

function on_figure_close(fig)
    if isvalid(fig)
        app = fig.UserData;
        if ~isempty(app)
            stop_timer(app);
        end
        delete(fig);
    end
end
