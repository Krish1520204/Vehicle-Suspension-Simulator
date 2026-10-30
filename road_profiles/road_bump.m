function [z0f, dz0f, z0r, dz0r] = road_bump(t, cfg, params)
% ROAD_BUMP Evaluates road elevation and velocity for a smooth speed bump.
%
% Profile formulation (smooth cosine bump with continuous derivatives):
%   z0(x) = (H/2) * (1 - cos(2*pi*x / W)) for 0 <= x <= W
%   dz0/dt = (pi * H * v / W) * sin(2*pi*v*(t - t_start) / W)
%
% Rear wheel experiences identical profile delayed by tau = L / v.
%
% Inputs:
%   t      - Current simulation time (s) [scalar or vector]
%   cfg    - Struct with fields:
%              .H       : Bump height (m) [default: 0.08]
%              .W       : Bump length / width (m) [default: 0.8]
%              .t_start : Start time for front wheel (s) [default: 0.5]
%              .v       : Vehicle speed (m/s) [default: 12.5 (45 km/h)]
%   params - Vehicle parameters struct containing .L (wheelbase)
%
% Outputs:
%   z0f, dz0f - Front wheel road elevation (m) and vertical velocity (m/s)
%   z0r, dz0r - Rear wheel road elevation (m) and vertical velocity (m/s)

    % Default parameters if not provided
    if ~isfield(cfg, 'H');       cfg.H = 0.08;       end
    if ~isfield(cfg, 'W');       cfg.W = 0.80;       end
    if ~isfield(cfg, 't_start'); cfg.t_start = 0.5;  end
    if ~isfield(cfg, 'v');       cfg.v = 12.5;       end

    H       = cfg.H;
    W       = cfg.W;
    t_start = cfg.t_start;
    v       = cfg.v;
    tau     = params.L / v;   % Wheelbase delay (s)
    T_bump  = W / v;          % Duration of bump traversal (s)

    % Front wheel time relative to bump onset
    tf = t - t_start;
    mask_f = (tf >= 0) & (tf <= T_bump);

    z0f  = zeros(size(t));
    dz0f = zeros(size(t));

    z0f(mask_f)  = (H / 2) * (1 - cos(2 * pi * v * tf(mask_f) / W));
    dz0f(mask_f) = (pi * H * v / W) * sin(2 * pi * v * tf(mask_f) / W);

    % Rear wheel time relative to bump onset
    tr = t - t_start - tau;
    mask_r = (tr >= 0) & (tr <= T_bump);

    z0r  = zeros(size(t));
    dz0r = zeros(size(t));

    z0r(mask_r)  = (H / 2) * (1 - cos(2 * pi * v * tr(mask_r) / W));
    dz0r(mask_r) = (pi * H * v / W) * sin(2 * pi * v * tr(mask_r) / W);
end
