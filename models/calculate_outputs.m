function out = calculate_outputs(t, X, params, road_func)
% CALCULATE_OUTPUTS Computes all derived physical quantities, forces, accelerations,
% and ride comfort / road holding metrics from the state history.
%
% Inputs:
%   t         - Time vector (N x 1)
%   X         - State trajectory matrix (N x 8)
%   params    - Vehicle parameters struct
%   road_func - Function handle: @(t) -> [z0f, dz0f, z0r, dz0r]
%
% Output:
%   out       - Struct containing all kinematic, dynamic, force, and metric arrays

    N = length(t);

    % Extract states
    z_s    = X(:, 1);
    theta  = X(:, 2);
    z_uf   = X(:, 3);
    z_ur   = X(:, 4);
    dz_s   = X(:, 5);
    dtheta = X(:, 6);
    dz_uf  = X(:, 7);
    dz_ur  = X(:, 8);

    % Vehicle physical parameters
    m_s  = params.m_s;
    I_y  = params.I_y;
    m_uf = params.m_uf;
    m_ur = params.m_ur;
    k_sf = params.k_sf;
    k_sr = params.k_sr;
    c_sf = params.c_sf;
    c_sr = params.c_sr;
    k_tf = params.k_tf;
    k_tr = params.k_tr;
    c_tf = params.c_tf;
    c_tr = params.c_tr;
    a    = params.a;
    b    = params.b;

    % Evaluate road profile for all time points
    [z0f, dz0f, z0r, dz0r] = road_func(t);

    % Ensure column vectors
    z0f  = z0f(:);
    dz0f = dz0f(:);
    z0r  = z0r(:);
    dz0r = dz0r(:);

    % Sprung mass kinematics at front and rear axle locations
    z_sf  = z_s + a * theta;
    dz_sf = dz_s + a * dtheta;

    z_sr  = z_s - b * theta;
    dz_sr = dz_s - b * dtheta;

    % Suspension deflections and velocities (compression positive)
    susp_def_f  = z_uf - z_sf;
    susp_vdef_f = dz_uf - dz_sf;

    susp_def_r  = z_ur - z_sr;
    susp_vdef_r = dz_ur - dz_sr;

    % Suspension forces (applied upwards on body, downwards on wheels)
    F_sf = k_sf * susp_def_f + c_sf * susp_vdef_f;
    F_sr = k_sr * susp_def_r + c_sr * susp_vdef_r;

    % Tyre deflections and dynamic forces
    tyre_def_f = z0f - z_uf;
    tyre_def_r = z0r - z_ur;

    F_tf = k_tf * tyre_def_f + c_tf * (dz0f - dz_uf);
    F_tr = k_tr * tyre_def_r + c_tr * (dz0r - dz_ur);

    % Total normal tyre ground contact forces (static + dynamic)
    % Positive means downward normal load pressing on road
    F_z_tf = params.W_tf_static + F_tf;
    F_z_tr = params.W_tr_static + F_tr;

    % Tyre load ratio (dynamic to static: R < 0 means wheel lift-off)
    tyre_load_ratio_f = F_z_tf / params.W_tf_static;
    tyre_load_ratio_r = F_z_tr / params.W_tr_static;

    % Accelerations
    % Heave acceleration of sprung mass CG
    ddz_s = (F_sf + F_sr) / m_s;

    % Pitch angular acceleration of sprung mass
    ddtheta = (a * F_sf - b * F_sr) / I_y;

    % Unsprung mass accelerations
    ddz_uf = (-F_sf + F_tf) / m_uf;
    ddz_ur = (-F_sr + F_tr) / m_ur;

    % Accelerations at vehicle body front and rear axle points
    ddz_sf = ddz_s + a * ddtheta;
    ddz_sr = ddz_s - b * ddtheta;

    % Accelerations at typical driver position (e.g. 0.4m forward of CG)
    d_driver = 0.4;
    ddz_driver = ddz_s + d_driver * ddtheta;

    % Store in output structure
    out.t = t;

    % Displacements
    out.z_s       = z_s;
    out.theta     = theta;
    out.theta_deg = theta * (180 / pi);
    out.z_uf      = z_uf;
    out.z_ur      = z_ur;
    out.z_sf      = z_sf;
    out.z_sr      = z_sr;

    % Velocities
    out.dz_s      = dz_s;
    out.dtheta    = dtheta;
    out.dz_uf     = dz_uf;
    out.dz_ur     = dz_ur;
    out.dz_sf     = dz_sf;
    out.dz_sr     = dz_sr;

    % Accelerations
    out.ddz_s      = ddz_s;
    out.ddtheta    = ddtheta;
    out.ddtheta_deg = ddtheta * (180 / pi);
    out.ddz_uf     = ddz_uf;
    out.ddz_ur     = ddz_ur;
    out.ddz_sf     = ddz_sf;
    out.ddz_sr     = ddz_sr;
    out.ddz_driver = ddz_driver;

    % Suspension kinematics and forces
    out.susp_def_f  = susp_def_f;
    out.susp_def_r  = susp_def_r;
    out.delta_sf    = susp_def_f;  % Standard mathematical alias
    out.delta_sr    = susp_def_r;  % Standard mathematical alias
    out.susp_vdef_f = susp_vdef_f;
    out.susp_vdef_r = susp_vdef_r;
    out.F_sf        = F_sf;
    out.F_sr        = F_sr;

    % Tyre kinematics and forces
    out.z0f               = z0f;
    out.z0r               = z0r;
    out.z_0f              = z0f;   % Standard mathematical alias
    out.z_0r              = z0r;   % Standard mathematical alias
    out.dz0f              = dz0f;
    out.dz0r              = dz0r;
    out.dz_0f             = dz0f;  % Standard mathematical alias
    out.dz_0r             = dz0r;  % Standard mathematical alias
    out.tyre_def_f        = tyre_def_f;
    out.tyre_def_r        = tyre_def_r;
    out.F_tf              = F_tf;
    out.F_tr              = F_tr;
    out.F_z_tf            = F_z_tf;
    out.F_z_tr            = F_z_tr;
    out.tyre_load_ratio_f = tyre_load_ratio_f;
    out.tyre_load_ratio_r = tyre_load_ratio_r;

    % Performance and Ride Comfort Metrics
    metrics.peak_body_disp        = max(abs(z_s));
    metrics.peak_body_disp_f      = max(abs(z_sf));
    metrics.peak_body_disp_r      = max(abs(z_sr));
    metrics.rms_body_disp         = rms(z_s);

    metrics.rms_heave_accel       = rms(ddz_s);
    metrics.peak_heave_accel      = max(abs(ddz_s));
    metrics.rms_pitch_accel       = rms(ddtheta);
    metrics.peak_pitch_accel      = max(abs(ddtheta));
    metrics.peak_pitch_angle_deg  = max(abs(out.theta_deg));
    metrics.rms_driver_accel      = rms(ddz_driver);
    metrics.peak_driver_accel     = max(abs(ddz_driver));

    metrics.peak_susp_travel_f    = max(abs(susp_def_f));
    metrics.peak_susp_travel_r    = max(abs(susp_def_r));
    metrics.jounce_limit          = 0.080; % 80 mm standard bumpstop limit
    metrics.travel_util_f_pct     = (metrics.peak_susp_travel_f / metrics.jounce_limit) * 100;
    metrics.travel_util_r_pct     = (metrics.peak_susp_travel_r / metrics.jounce_limit) * 100;

    metrics.peak_wheel_disp_f     = max(abs(z_uf));
    metrics.peak_wheel_disp_r     = max(abs(z_ur));

    metrics.min_tyre_ratio_f      = min(tyre_load_ratio_f);
    metrics.min_tyre_ratio_r      = min(tyre_load_ratio_r);
    metrics.wheel_lift_front      = any(tyre_load_ratio_f <= 0);
    metrics.wheel_lift_rear       = any(tyre_load_ratio_r <= 0);

    metrics.peak_susp_force_f     = max(abs(F_sf));
    metrics.peak_susp_force_r     = max(abs(F_sr));

    out.metrics = metrics;
    out.params  = params;
end
