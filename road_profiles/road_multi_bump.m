function [z0f, dz0f, z0r, dz0r, info] = road_multi_bump(t, cfg, params)
% ROAD_MULTI_BUMP Generates multiple consecutive continuous-time road bumps.
%
% Supported bump shapes:
%   1. 'rectangular'   : Flat-top bump with smooth C^1 cosine rise and fall
%   2. 'triangular'    : Isosceles triangle with smooth parabolic fillets at base and apex
%   3. 'half_sine'     : Half-sinusoidal wave with smooth C^1 boundary transitions
%
% Features:
%   - Configurable number of consecutive bumps (default: 8, range: 7-10)
%   - Configurable bump amplitude (H), length (W), and gap spacing (S)
%   - Uses vehicle forward speed (v) to calculate front-wheel encounter times
%   - Implements exact rear-wheel time delay: tau = L / v (wheelbase / speed)
%   - Ensures front and rear wheels traverse the exact same road profile
%   - Smooth C^1 transitions prevent non-physical derivative singularities in ODE45
%
% Inputs:
%   t      - Simulation time (s) [scalar or vector]
%   cfg    - Struct with fields:
%              .profile_shape   : 'rectangular', 'triangular', 'half_sine' [default: 'half_sine']
%              .num_bumps       : Number of bumps [default: 8]
%              .H               : Bump amplitude / height (m) [default: 0.05]
%              .W               : Bump length / width (m) [default: 0.60]
%              .spacing         : Gap between consecutive bumps (m) [default: 1.20]
%              .t_start         : Onset time of first bump for front wheel (s) [default: 0.50]
%              .v               : Vehicle forward speed (m/s) [default: 12.5 (45 km/h)]
%              .smoothing_ratio : Transition smoothing fraction (0 < r < 0.25) [default: 0.08]
%   params - Vehicle parameters struct containing .L (wheelbase)
%
% Outputs:
%   z0f, dz0f - Front wheel road elevation (m) and vertical velocity (m/s)
%   z0r, dz0r - Rear wheel road elevation (m) and vertical velocity (m/s)
%   info      - Struct containing timetable, spatial layout, and bump geometry

    % 1. Parameter Validation & Defaults
    if nargin < 2; cfg = struct(); end
    if ~isfield(cfg, 'profile_shape')
        if isfield(cfg, 'type') && ismember(lower(cfg.type), {'rectangular', 'triangular', 'half_sine', 'halfsine'})
            cfg.profile_shape = lower(cfg.type);
        else
            cfg.profile_shape = 'half_sine';
        end
    end
    if ~isfield(cfg, 'num_bumps');       cfg.num_bumps = 8;       end
    if ~isfield(cfg, 'H');               cfg.H = 0.05;           end % 50 mm bump
    if ~isfield(cfg, 'W');               cfg.W = 0.60;           end % 600 mm bump length
    if ~isfield(cfg, 'spacing');         cfg.spacing = 1.20;     end % 1.2 m gap between bumps
    if ~isfield(cfg, 't_start');         cfg.t_start = 0.50;     end % First bump starts at t = 0.5 s
    if ~isfield(cfg, 'v');               cfg.v = 12.5;           end % 12.5 m/s = 45 km/h
    if ~isfield(cfg, 'smoothing_ratio'); cfg.smoothing_ratio = 0.08; end

    % Extract configuration
    shape   = lower(cfg.profile_shape);
    N_bumps = cfg.num_bumps;
    H       = cfg.H;
    W       = cfg.W;
    S       = cfg.spacing;
    t_start = cfg.t_start;
    v       = cfg.v;
    r_smooth = cfg.smoothing_ratio;

    % Pitch between bump starts
    pitch = W + S;

    % Wheelbase and time delay
    L   = params.L;
    tau = L / v;

    % 2. Precompute Spatial Layout along the Road (m)
    x_start = v * t_start;
    bump_x_starts = x_start + (0 : N_bumps - 1) * pitch;
    bump_x_ends   = bump_x_starts + W;

    % Front wheel encounter times (s)
    t_f_enter = bump_x_starts / v;
    t_f_exit  = bump_x_ends / v;

    % Rear wheel encounter times (s)
    t_r_enter = t_f_enter + tau;
    t_r_exit  = t_f_exit + tau;

    % Package timetable info if requested
    if nargout >= 5
        info.profile_shape   = shape;
        info.num_bumps       = N_bumps;
        info.H               = H;
        info.W               = W;
        info.spacing         = S;
        info.pitch           = pitch;
        info.vehicle_speed   = v;
        info.wheelbase       = L;
        info.wheelbase_delay = tau;
        info.bump_x_starts   = bump_x_starts;
        info.bump_x_ends     = bump_x_ends;
        info.t_f_enter       = t_f_enter;
        info.t_f_exit        = t_f_exit;
        info.t_r_enter       = t_r_enter;
        info.t_r_exit        = t_r_exit;
    end

    % 3. Evaluate Road Elevation and Spatial Derivatives
    % Longitudinal position of front wheel: x_f = v * t
    % Longitudinal position of rear wheel:  x_r = v * (t - tau)
    x_f = v * t;
    x_r = v * (t - tau);

    [z0f, dz0f_dx] = evaluate_road_spatial(x_f, bump_x_starts, W, H, shape, r_smooth);
    [z0r, dz0r_dx] = evaluate_road_spatial(x_r, bump_x_starts, W, H, shape, r_smooth);

    % Convert spatial derivative to time derivative: dz/dt = v * (dz/dx)
    dz0f = v * dz0f_dx;
    dz0r = v * dz0r_dx;

    % Preserve orientation of input t
    if iscolumn(t)
        z0f  = z0f(:);
        dz0f = dz0f(:);
        z0r  = z0r(:);
        dz0r = dz0r(:);
    end
end

%% Helper Function: Spatial Road Evaluation
function [z_road, dz_dx] = evaluate_road_spatial(x_vec, bump_x_starts, W, H, shape, r_smooth)
% Evaluates road elevation z(x) and spatial slope dz/dx at positions x_vec.

    z_road = zeros(size(x_vec));
    dz_dx  = zeros(size(x_vec));

    num_bumps = length(bump_x_starts);

    for k = 1:num_bumps
        x_k_start = bump_x_starts(k);
        x_k_end   = x_k_start + W;

        % Identify query positions inside bump k
        mask = (x_vec >= x_k_start) & (x_vec <= x_k_end);
        if ~any(mask(:))
            continue;
        end

        % Local coordinate along the bump: u in [0, W]
        u = x_vec(mask) - x_k_start;

        switch shape
            case {'rectangular', 'rect'}
                [z_k, dz_k] = eval_rectangular_bump(u, W, H, r_smooth);

            case {'triangular', 'tri', 'triangle'}
                [z_k, dz_k] = eval_triangular_bump(u, W, H, r_smooth);

            case {'half_sine', 'halfsine', 'sine_bump'}
                [z_k, dz_k] = eval_halfsine_bump(u, W, H, r_smooth);

            otherwise
                error('Unsupported multi-bump shape "%s". Choose: rectangular, triangular, half_sine', shape);
        end

        z_road(mask) = z_k;
        dz_dx(mask)  = dz_k;
    end
end

%% Profile 1: Rectangular with Smooth C^1 Filleted Edges
function [z, dz_du] = eval_rectangular_bump(u, W, H, r_smooth)
% Flat-top rectangular pulse with smooth cosine rising and falling transitions.
% Transition length: delta = r_smooth * W.

    delta = max(r_smooth * W, 0.005); % Minimum 5 mm transition

    z     = zeros(size(u));
    dz_du = zeros(size(u));

    % Zone 1: Rising flank [0, delta]
    m1 = (u >= 0) & (u < delta);
    if any(m1)
        z(m1)     = (H / 2) * (1 - cos(pi * u(m1) / delta));
        dz_du(m1) = (H * pi / (2 * delta)) * sin(pi * u(m1) / delta);
    end

    % Zone 2: Flat plateau [delta, W - delta]
    m2 = (u >= delta) & (u <= W - delta);
    if any(m2)
        z(m2)     = H;
        dz_du(m2) = 0;
    end

    % Zone 3: Falling flank [W - delta, W]
    m3 = (u > W - delta) & (u <= W);
    if any(m3)
        u_fall = u(m3) - (W - delta);
        z(m3)     = (H / 2) * (1 + cos(pi * u_fall / delta));
        dz_du(m3) = -(H * pi / (2 * delta)) * sin(pi * u_fall / delta);
    end
end

%% Profile 2: Triangular with Smooth Parabolic Fillets at Base & Apex
function [z, dz_du] = eval_triangular_bump(u, W, H, r_smooth)
% Isosceles triangle with smooth parabolic base fillet and rounded apex.
% Ensures C^1 continuity across all transition points.

    half_W = W / 2;
    delta  = max(r_smooth * half_W, 0.005); % Fillet radius length

    % Linear flank slope: m_slope
    % Height at transition: H_fillet = (m_slope * delta) / 2
    % Peak apex height: H = m_slope * (half_W - delta) + m_slope * delta = m_slope * half_W
    m_slope = H / half_W;

    z     = zeros(size(u));
    dz_du = zeros(size(u));

    % 1. Rising half: u in [0, half_W]
    m_rise = (u >= 0) & (u <= half_W);
    if any(m_rise)
        u_r = u(m_rise);
        z_r = zeros(size(u_r));
        dz_r = zeros(size(u_r));

        % Base round: [0, delta] -> parabolic: z = (m / (2*delta)) * u^2
        m_base = u_r < delta;
        z_r(m_base)  = (m_slope / (2 * delta)) * (u_r(m_base).^2);
        dz_r(m_base) = (m_slope / delta) * u_r(m_base);

        % Linear flank: [delta, half_W - delta]
        m_lin = (u_r >= delta) & (u_r <= half_W - delta);
        z_r(m_lin)  = (m_slope * delta / 2) + m_slope * (u_r(m_lin) - delta);
        dz_r(m_lin) = m_slope;

        % Apex round: [half_W - delta, half_W] -> inverted parabola to peak H
        m_apex = u_r > half_W - delta;
        dist_to_apex = half_W - u_r(m_apex);
        z_r(m_apex)  = H - (m_slope / (2 * delta)) * (dist_to_apex.^2);
        dz_r(m_apex) = (m_slope / delta) * dist_to_apex;

        z(m_rise)     = z_r;
        dz_du(m_rise) = dz_r;
    end

    % 2. Falling half: u in [half_W, W] (symmetric reflection)
    m_fall = (u > half_W) & (u <= W);
    if any(m_fall)
        % Map to rising half coordinate: u_mirror = W - u
        u_mirror = W - u(m_fall);
        [z_m, dz_m] = eval_triangular_bump(u_mirror, W, H, r_smooth);
        z(m_fall)     = z_m;
        dz_du(m_fall) = -dz_m; % Invert slope for descent
    end
end

%% Profile 3: Half-Sinusoidal with Smooth C^1 Boundary Transitions
function [z, dz_du] = eval_halfsine_bump(u, W, H, r_smooth)
% Half-sine pulse: z = H * sin(pi * u / W)
% Equipped with smooth cosine edge blending in [0, delta] and [W-delta, W]
% ensuring z'(0) = 0 and z'(W) = 0 for stable ODE45 numerical integration.

    delta = max(r_smooth * W, 0.005);

    % Base half-sine wave
    z_base  = H * sin(pi * u / W);
    dz_base = (pi * H / W) * cos(pi * u / W);

    z     = z_base;
    dz_du = dz_base;

    % Smooth entry at start: [0, delta]
    m_start = (u >= 0) & (u < delta);
    if any(m_start)
        u_s = u(m_start);
        w_s   = (1 / 2) * (1 - cos(pi * u_s / delta));
        dw_s  = (pi / (2 * delta)) * sin(pi * u_s / delta);

        z(m_start)     = z_base(m_start) .* w_s;
        dz_du(m_start) = dz_base(m_start) .* w_s + z_base(m_start) .* dw_s;
    end

    % Smooth exit at end: [W - delta, W]
    m_end = (u > W - delta) & (u <= W);
    if any(m_end)
        u_e = u(m_end);
        w_e  = (1 / 2) * (1 + cos(pi * (u_e - (W - delta)) / delta));
        dw_e = -(pi / (2 * delta)) * sin(pi * (u_e - (W - delta)) / delta);

        z(m_end)     = z_base(m_end) .* w_e;
        dz_du(m_end) = dz_base(m_end) .* w_e + z_base(m_end) .* dw_e;
    end
end
