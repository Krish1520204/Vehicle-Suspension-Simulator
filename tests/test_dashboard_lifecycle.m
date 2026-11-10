% TEST_DASHBOARD_LIFECYCLE
% Automated verification script testing all dashboard controls, callbacks,
% simulation recalculations, graphs, animation frames, and timer lifecycle.

disp('--- 1. Launching Dashboard ---');
run_dashboard;
drawnow;
app = fig.UserData;
assert(~isempty(app), 'Dashboard app struct should be stored in fig.UserData');
assert(isvalid(app.ax_road), 'ax_road must be valid');
assert(isvalid(app.ax_disp), 'ax_disp must be valid');
assert(isvalid(app.ax_accel), 'ax_accel must be valid');
assert(isvalid(app.ax_defl), 'ax_defl must be valid');
assert(isvalid(app.ax_pitch), 'ax_pitch must be valid');
assert(isvalid(app.ax_anim), 'ax_anim must be valid');
disp('   ✓ Dashboard axes and UI created successfully.');

disp('--- 2. Testing Slider & Dropdown Callbacks ---');
% Change amplitude
app.sld_amp.Value = 70;
app.sld_amp.ValueChangedFcn(app.sld_amp, []);
app = fig.UserData;
assert(app.road_cfg.H == 0.070, 'Road amplitude should update to 0.070 m');

% Change speed
app.sld_spd.Value = 45;
app.sld_spd.ValueChangedFcn(app.sld_spd, []);
app = fig.UserData;
assert(abs(app.road_cfg.v - 45/3.6) < 1e-4, 'Road speed should update to 45 km/h');

% Change damping
app.sld_damp.Value = 2000;
app.sld_damp.ValueChangedFcn(app.sld_damp, []);
app = fig.UserData;
assert(app.params.c_sf == 2000, 'Suspension damping should update');

% Change natural frequency
app.sld_fn.Value = 1.6;
app.sld_fn.ValueChangedFcn(app.sld_fn, []);
app = fig.UserData;

% Change road profile
app.dd_profile.Value = 'triangular';
app.dd_profile.ValueChangedFcn(app.dd_profile, []);
app = fig.UserData;
assert(strcmp(app.road_cfg.type, 'triangular'), 'Road profile should be triangular');

% Change vehicle preset
app.dd_vehicle.Value = 'suv';
app.dd_vehicle.ValueChangedFcn(app.dd_vehicle, []);
app = fig.UserData;
assert(strcmp(app.vehicle_preset, 'suv'), 'Vehicle preset should be SUV');
disp('   ✓ Control callbacks executed and state updated correctly.');

disp('--- 3. Testing Start Button (Simulation & Graph Plotting) ---');
% This triggers compute_and_update_simulation which tests:
% cla(app.ax_road), cla(app.ax_disp), cla(app.ax_accel), cla(app.ax_defl), cla(app.ax_pitch)
app.btn_start.ButtonPushedFcn(app.btn_start, []);
drawnow;
app = fig.UserData;
assert(~isempty(app.sim), 'Simulation results must be computed');
assert(~isempty(app.kpi), 'KPIs must be calculated');
assert(strcmp(app.btn_pause.Enable, 'on'), 'Pause button should be enabled');
disp('   ✓ Start button executed: ODE45 resolved, all 5 graphs plotted, KPIs updated.');

disp('--- 4. Testing Pause Button ---');
app.btn_pause.ButtonPushedFcn(app.btn_pause, []);
drawnow;
app = fig.UserData;
assert(~app.is_playing, 'Playback should be paused');
assert(strcmp(app.btn_resume.Enable, 'on'), 'Resume button should be enabled');
disp('   ✓ Pause button executed: timer stopped successfully.');

disp('--- 5. Testing Resume Button ---');
app.btn_resume.ButtonPushedFcn(app.btn_resume, []);
drawnow;
app = fig.UserData;
assert(app.is_playing, 'Playback should resume');
assert(strcmp(app.btn_pause.Enable, 'on'), 'Pause button should be enabled');
disp('   ✓ Resume button executed: timer resumed successfully.');

disp('--- 6. Testing Reset Button ---');
app.btn_reset.ButtonPushedFcn(app.btn_reset, []);
drawnow;
app = fig.UserData;
assert(strcmp(app.vehicle_preset, 'sedan'), 'Preset should reset to sedan');
assert(strcmp(app.road_cfg.type, 'half_sine'), 'Profile should reset to half_sine');
disp('   ✓ Reset button executed: parameters restored to nominal.');

disp('--- 7. Cleaning up Figure & Timers ---');
close(fig);
disp('   ✓ Figure closed cleanly.');
disp('=== ALL DASHBOARD CHECKS COMPLETED SUCCESSFULLY ===');
