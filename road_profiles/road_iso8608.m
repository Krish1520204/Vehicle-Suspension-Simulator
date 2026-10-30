function [z0f, dz0f, z0r, dz0r] = road_iso8608(t, cfg, params)
% ROAD_ISO8608 Generates random road profile based on ISO 8608 roughness classification.
%
% Generates road elevation using harmonic superposition:
%   z(x) = sum_k [ A_k * cos(2*pi*n_k*x + phi_k) ]
%   dz/dt = - sum_k [ A_k * 2*pi*n_k*v * sin(2*pi*n_k*x + phi_k) ]
%
% where A_k = sqrt(2 * G_d(n_k) * delta_n)
%       G_d(n) = G_d(n0) * (n / n0)^(-w)
%
% Inputs:
%   t      - Simulation time (s) [scalar or vector]
%   cfg    - Struct with fields:
%              .road_class : 'A', 'B', 'C', or 'D' [default: 'B']
%              .v          : Vehicle speed (m/s) [default: 12.5]
%              .seed       : Random seed for reproducibility [default: 42]
%   params - Vehicle parameters struct containing .L (wheelbase)

    persistent iso_data;

    if ~isfield(cfg, 'road_class'); cfg.road_class = 'B'; end
    if ~isfield(cfg, 'v');          cfg.v = 12.5;         end
    if ~isfield(cfg, 'seed');       cfg.seed = 42;        end

    % Check if persistent data needs initialization or re-generation
    init_needed = isempty(iso_data) || ...
                  ~strcmp(iso_data.road_class, cfg.road_class) || ...
                  iso_data.seed ~= cfg.seed;

    if init_needed
        % ISO 8608 roughness coefficient G_d(n0) in m^3 [n0 = 0.1 cycles/m]
        switch upper(cfg.road_class)
            case 'A', G0 = 16e-6;   % Very good (smooth highway)
            case 'B', G0 = 64e-6;   % Good (minor highway)
            case 'C', G0 = 256e-6;  % Average (paved road with roughness)
            case 'D', G0 = 1024e-6; % Poor (unpaved / damaged road)
            otherwise, G0 = 64e-6;
        end

        n0 = 0.1;           % Reference spatial frequency (cycles/m)
        w  = 2.0;           % Waviness exponent
        N  = 200;           % Number of harmonic wave components
        n_min = 0.01;       % Min spatial frequency (wavelength = 100 m)
        n_max = 5.0;        % Max spatial frequency (wavelength = 0.2 m)

        % Frequency discretization
        delta_n = (n_max - n_min) / N;
        n_k = linspace(n_min + delta_n/2, n_max - delta_n/2, N)';

        % Power Spectral Density
        Gd_k = G0 * (n_k ./ n0).^(-w);

        % Amplitudes
        A_k = sqrt(2 * Gd_k * delta_n);

        % Fixed random phases for repeatability
        rng(cfg.seed);
        phi_k = 2 * pi * rand(N, 1);

        iso_data.road_class = cfg.road_class;
        iso_data.seed       = cfg.seed;
        iso_data.n_k        = n_k;
        iso_data.A_k        = A_k;
        iso_data.phi_k      = phi_k;
    end

    v   = cfg.v;
    tau = params.L / v;
    n_k   = iso_data.n_k;
    A_k   = iso_data.A_k;
    phi_k = iso_data.phi_k;

    % Front wheel longitudinal position: x_f = v * t
    xf = v * t(:)'; % Row vector (1 x Nt)
    % Rear wheel longitudinal position: x_r = v * (t - tau)
    xr = v * (t(:)' - tau);

    % Argument matrices (N x Nt)
    arg_f = 2 * pi * (n_k * xf) + phi_k;
    arg_r = 2 * pi * (n_k * xr) + phi_k;

    % Road elevation (sum over harmonics)
    z0f = sum(A_k .* cos(arg_f), 1);
    z0r = sum(A_k .* cos(arg_r), 1);

    % Road vertical velocity: dz/dt = -sum(A_k * 2*pi*n_k*v * sin(arg))
    dz0f = -sum((A_k .* (2 * pi * n_k * v)) .* sin(arg_f), 1);
    dz0r = -sum((A_k .* (2 * pi * n_k * v)) .* sin(arg_r), 1);

    % Reshape outputs to match input t orientation
    if iscolumn(t)
        z0f  = z0f';
        dz0f = dz0f';
        z0r  = z0r';
        dz0r = dz0r';
    end
end
