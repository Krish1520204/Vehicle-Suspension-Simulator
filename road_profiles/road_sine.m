function [z0f, dz0f, z0r, dz0r] = road_sine(t, cfg, params)
% ROAD_SINE Evaluates road elevation and velocity for an undulating sinusoidal road profile.
%
% Formulation:
%   Spatial wavelength = lambda (m)
%   Temporal frequency omega = 2 * pi * v / lambda (rad/s)
%   z0(t) = A * sin(omega * t)
%   dz0/dt = A * omega * cos(omega * t)
%
% Inputs:
%   t      - Simulation time (s) [scalar or vector]
%   cfg    - Struct with fields:
%              .A      : Amplitude (m) [default: 0.03]
%              .lambda : Wavelength (m) [default: 5.0]
%              .v      : Vehicle speed (m/s) [default: 12.5]
%   params - Vehicle parameters struct containing .L (wheelbase)

    if ~isfield(cfg, 'A');      cfg.A = 0.03;     end
    if ~isfield(cfg, 'lambda'); cfg.lambda = 5.0; end
    if ~isfield(cfg, 'v');      cfg.v = 12.5;     end

    A      = cfg.A;
    v      = cfg.v;
    lambda = cfg.lambda;
    omega  = 2 * pi * v / lambda;  % Excitation angular frequency (rad/s)
    tau    = params.L / v;         % Wheelbase time delay (s)

    % Front wheel
    z0f  = A * sin(omega * t);
    dz0f = A * omega * cos(omega * t);

    % Rear wheel delayed by tau
    z0r  = A * sin(omega * (t - tau));
    dz0r = A * omega * cos(omega * (t - tau));
end
