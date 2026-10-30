function [z0f, dz0f, z0r, dz0r] = road_step(t, cfg, params)
% ROAD_STEP Evaluates road elevation and velocity for a smooth step input (e.g. curb or pothole edge).
%
% Uses a smooth sigmoid transition to ensure continuous velocities and stable ODE integration:
%   z0(t) = (H / 2) * (1 + tanh(k_steep * (t - t_step)))
%   dz0/dt = (H * k_steep / 2) * sech^2(k_steep * (t - t_step))
%
% Inputs:
%   t      - Current simulation time (s) [scalar or vector]
%   cfg    - Struct with fields:
%              .H       : Step height (m) [default: 0.05]
%              .t_step  : Onset time for front wheel (s) [default: 0.5]
%              .rise_time : Transition duration (s) [default: 0.02]
%              .v       : Vehicle speed (m/s) [default: 12.5]
%   params - Vehicle parameters struct containing .L (wheelbase)

    if ~isfield(cfg, 'H');         cfg.H = 0.05;         end
    if ~isfield(cfg, 't_step');    cfg.t_step = 0.5;    end
    if ~isfield(cfg, 'rise_time'); cfg.rise_time = 0.02; end
    if ~isfield(cfg, 'v');         cfg.v = 12.5;         end

    H      = cfg.H;
    t_step = cfg.t_step;
    v      = cfg.v;
    tau    = params.L / v;
    k      = 4 / cfg.rise_time;  % Steepness factor for tanh (96% rise in rise_time)

    % Front wheel
    tf   = t - t_step;
    z0f  = (H / 2) * (1 + tanh(k * tf));
    dz0f = (H * k / 2) * (sech(k * tf).^2);

    % Rear wheel delayed by tau
    tr   = t - t_step - tau;
    z0r  = (H / 2) * (1 + tanh(k * tr));
    dz0r = (H * k / 2) * (sech(k * tr).^2);
end
