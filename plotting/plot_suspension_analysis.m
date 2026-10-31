function fig = plot_suspension_analysis(sim, save_flag, filename)
% PLOT_SUSPENSION_ANALYSIS Comprehensive multi-panel plotting module for the 4-DOF suspension model.
%
% Generates all 7 required engineering graphs using actual ODE45 simulation results:
%   1. Road displacement profile (front & rear wheels with delay)
%   2. Vehicle body vertical displacement (heave at CG & axle locations)
%   3. Vehicle body pitch angle (degrees)
%   4. Front suspension deflection with jounce/rebound limits
%   5. Rear suspension deflection with jounce/rebound limits
%   6. Vehicle body vertical acceleration (CG heave & driver position)
%   7. Front and rear wheel displacement (wheel hop vs road)
%   8. Numerical Performance Indicator & KPI Summary Dashboard
%
% Features:
%   - All graphs dynamically reflect the simulation results (re-runnable upon parameter change)
%   - Clearly labelled axes, standard SI units, legends, and descriptive titles
%   - Displays numerical KPIs: max body displacement, max acceleration, max travel, etc.
%   - Built from separate, modular, and reusable sub-plotting functions
%
% Inputs:
%   sim       - Simulation results struct from run_simulation.m
%   save_flag - (Optional) Boolean, if true saves plot to outputs/ [default: false]
%   filename  - (Optional) Name of output image file [default: 'suspension_analysis.png']
%
% Output:
%   fig       - Figure handle

    if nargin < 2 || isempty(save_flag)
        save_flag = false;
    end
    if nargin < 3 || isempty(filename)
        filename = 'suspension_analysis.png';
    end

    m = sim.metrics;
    p = sim.params;
    r = sim.road_cfg;
    g = 9.81;

    % Create modern widescreen figure
    fig = figure('Name', '4-DOF Half-Car Suspension System Analysis', ...
                 'NumberTitle', 'off', ...
                 'Color', [1 1 1], ...
                 'Position', [50, 40, 1400, 900]);

    % Top Title Banner
    scenario_str = sprintf('4-DOF Half-Car Suspension Simulation Analysis | Vehicle: %s | Road: %s (v = %.1f km/h)', ...
                           upper(p.preset_name), upper(r.type), r.v * 3.6);
    annotation(fig, 'textbox', [0.05, 0.955, 0.90, 0.035], ...
               'String', scenario_str, ...
               'FontSize', 13, 'FontWeight', 'bold', ...
               'HorizontalAlignment', 'center', ...
               'VerticalAlignment', 'middle', ...
               'EdgeColor', 'none', ...
               'BackgroundColor', [0.93, 0.95, 0.98]);

    % Define 4x2 subplot layout with custom margins
    ax1 = subplot(4, 2, 1); plot_road_profile(ax1, sim);
    ax2 = subplot(4, 2, 2); plot_body_displacement(ax2, sim);
    ax3 = subplot(4, 2, 3); plot_pitch_angle(ax3, sim);
    ax4 = subplot(4, 2, 4); plot_front_suspension_deflection(ax4, sim);
    ax5 = subplot(4, 2, 5); plot_rear_suspension_deflection(ax5, sim);
    ax6 = subplot(4, 2, 6); plot_body_acceleration(ax6, sim);
    ax7 = subplot(4, 2, 7); plot_wheel_displacement(ax7, sim);

    % Panel 8: Numerical Performance Indicators & KPI Summary Card
    ax8 = subplot(4, 2, 8);
    cla(ax8);
    set(ax8, 'Color', [0.97, 0.98, 1.0], 'XColor', 'none', 'YColor', 'none');
    hold(ax8, 'on');

    % Header
    text(ax8, 0.50, 0.93, 'NUMERICAL PERFORMANCE INDICATORS (KPIs)', ...
         'FontSize', 11, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
         'Color', [0.05, 0.20, 0.45]);

    % Format KPI text lines
    line1 = sprintf('\\bullet Maximum Body Heave Displacement :  {\\bf%6.2f mm}', m.peak_body_disp * 1000);
    line2 = sprintf('\\bullet Maximum CG Vertical Acceleration :  {\\bf%6.2f m/s^2}  (%.2f g)', ...
                    m.peak_heave_accel, m.peak_heave_accel / g);
    line3 = sprintf('\\bullet Maximum Driver Seat Acceleration :  {\\bf%6.2f m/s^2}  (%.2f g)', ...
                    m.peak_driver_accel, m.peak_driver_accel / g);
    line4 = sprintf('\\bullet Maximum Front Suspension Travel   :  {\\bf%6.1f mm}  (%.1f%% of 80mm limit)', ...
                    m.peak_susp_travel_f * 1000, m.travel_util_f_pct);
    line5 = sprintf('\\bullet Maximum Rear Suspension Travel    :  {\\bf%6.1f mm}  (%.1f%% of 80mm limit)', ...
                    m.peak_susp_travel_r * 1000, m.travel_util_r_pct);
    line6 = sprintf('\\bullet Maximum Body Pitch Angle          :  {\\bf%6.2f deg}', m.peak_pitch_angle_deg);
    line7 = sprintf('\\bullet Peak Dynamic Suspension Force     :  Front: {\\bf%.2f kN} | Rear: {\\bf%.2f kN}', ...
                    m.peak_susp_force_f / 1000, m.peak_susp_force_r / 1000);
    line8 = sprintf('\\bullet RMS Heave Acceleration (Ride)     :  {\\bf%6.2f m/s^2}', m.rms_heave_accel);

    if m.wheel_lift_front || m.wheel_lift_rear
        contact_str = '{\\bf\\color[rgb]{0.85,0.2,0.1}WHEEL LIFT-OFF DETECTED (Loss of Grip)}';
    else
        contact_str = '{\\bf\\color[rgb]{0.1,0.6,0.2}CONTINUOUS GROUND CONTACT MAINTAINED}';
    end
    line9 = sprintf('\\bullet Road Holding Status               :  %s', contact_str);

    text_block = sprintf('%s\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n%s', ...
                         line1, line2, line3, line4, line5, line6, line7, line8, line9);

    text(ax8, 0.05, 0.45, text_block, 'FontSize', 9, 'VerticalAlignment', 'middle', ...
         'FontName', 'Helvetica');

    box(ax8, 'on');
    title(ax8, '8. Performance Summary & Quantitative Assessment', ...
          'FontSize', 11, 'FontWeight', 'bold');
    hold(ax8, 'off');

    % Adjust subplots spacing
    set(gcf, 'Units', 'Normalized');

    % Save figure if requested
    if save_flag
        script_dir = fileparts(mfilename('fullpath'));
        out_dir = fullfile(fileparts(script_dir), 'outputs');
        if ~exist(out_dir, 'dir')
            mkdir(out_dir);
        end
        save_path = fullfile(out_dir, filename);
        try
            exportgraphics(fig, save_path, 'Resolution', 300);
        catch
            saveas(fig, save_path);
        end
        fprintf('Saved suspension analysis plot to: %s\n', save_path);
    end
end
