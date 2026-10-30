function params = get_vehicle_params(preset)
% GET_VEHICLE_PARAMS Returns physical parameters for a 4-DOF half-car suspension model.
%
% Syntax:
%   params = get_vehicle_params()          % Default preset: 'sedan'
%   params = get_vehicle_params(preset)    % Options: 'sedan', 'suv', 'hatchback', 'sport'
%
% State vector definition:
%   x = [z_s; theta; z_uf; z_ur; dz_s; dtheta; dz_uf; dz_ur]
%   z_s:   Sprung mass vertical heave at CG (m, positive up)
%   theta: Sprung mass pitch angle (rad, positive nose-up)
%   z_uf:  Front unsprung mass vertical displacement (m, positive up)
%   z_ur:  Rear unsprung mass vertical displacement (m, positive up)

    if nargin < 1 || isempty(preset)
        preset = 'sedan';
    end

    switch lower(preset)
        case 'sedan'
            % Typical D-segment passenger car (half-car parameters: symmetric side)
            % Values represent one side (half of total vehicle) or full half-car (2 wheels)
            % Note: For a 2D half-car (bicycle model view), mass is total vehicle mass
            % or half vehicle mass. Here we use half vehicle mass (one side: 1 front, 1 rear wheel).
            params.m_s   = 730;       % Sprung mass (kg) [half of ~1460 kg body]
            params.I_y   = 1250;      % Pitch moment of inertia (kg*m^2) [half-car]
            params.m_uf  = 45;        % Front unsprung mass (kg) [wheel + hub + brake assembly]
            params.m_ur  = 40;        % Rear unsprung mass (kg)
            params.k_sf  = 28000;     % Front suspension stiffness (N/m)
            params.k_sr  = 26000;     % Rear suspension stiffness (N/m)
            params.c_sf  = 1600;      % Front suspension damping coefficient (N*s/m)
            params.c_sr  = 1500;      % Rear suspension damping coefficient (N*s/m)
            params.k_tf  = 190000;    % Front tyre vertical stiffness (N/m)
            params.k_tr  = 190000;    % Rear tyre vertical stiffness (N/m)
            params.c_tf  = 150;       % Front tyre damping coefficient (N*s/m)
            params.c_tr  = 150;       % Rear tyre damping coefficient (N*s/m)
            params.a     = 1.15;      % Distance from CG to front axle (m)
            params.b     = 1.45;      % Distance from CG to rear axle (m)

        case 'suv'
            % Mid-size SUV with stiffer springs, higher damping, and heavier body
            params.m_s   = 1050;      % Sprung mass (kg)
            params.I_y   = 1900;      % Pitch moment of inertia (kg*m^2)
            params.m_uf  = 60;        % Front unsprung mass (kg)
            params.m_ur  = 55;        % Rear unsprung mass (kg)
            params.k_sf  = 38000;     % Front suspension stiffness (N/m)
            params.k_sr  = 36000;     % Rear suspension stiffness (N/m)
            params.c_sf  = 2400;      % Front suspension damping (N*s/m)
            params.c_sr  = 2200;      % Rear suspension damping (N*s/m)
            params.k_tf  = 230000;    % Front tyre stiffness (N/m)
            params.k_tr  = 230000;    % Rear tyre stiffness (N/m)
            params.c_tf  = 200;       % Front tyre damping (N*s/m)
            params.c_tr  = 200;       % Rear tyre damping (N*s/m)
            params.a     = 1.25;      % Distance from CG to front axle (m)
            params.b     = 1.55;      % Distance from CG to rear axle (m)

        case 'sport'
            % High-performance sports car: lower mass, high stiffness, firm damping
            params.m_s   = 600;       % Sprung mass (kg)
            params.I_y   = 950;       % Pitch moment of inertia (kg*m^2)
            params.m_uf  = 38;        % Front unsprung mass (kg)
            params.m_ur  = 36;        % Rear unsprung mass (kg)
            params.k_sf  = 42000;     % Front suspension stiffness (N/m)
            params.k_sr  = 40000;     % Rear suspension stiffness (N/m)
            params.c_sf  = 2800;      % Front suspension damping (N*s/m)
            params.c_sr  = 2600;      % Rear suspension damping (N*s/m)
            params.k_tf  = 220000;    % Front tyre stiffness (N/m)
            params.k_tr  = 220000;    % Rear tyre stiffness (N/m)
            params.c_tf  = 180;       % Front tyre damping (N*s/m)
            params.c_tr  = 180;       % Rear tyre damping (N*s/m)
            params.a     = 1.10;      % Distance from CG to front axle (m)
            params.b     = 1.35;      % Distance from CG to rear axle (m)

        otherwise
            error('Unknown vehicle preset "%s". Choose from: sedan, suv, sport', preset);
    end

    % Derived geometric and kinematic parameters
    params.L = params.a + params.b;   % Total wheelbase (m)
    params.g = 9.81;                  % Acceleration due to gravity (m/s^2)
    params.preset_name = preset;

    % Static normal load calculations (N)
    % Moments about rear contact: W_front_sprung = m_s * g * (b / L)
    params.W_sf_static = params.m_s * params.g * (params.b / params.L);
    params.W_sr_static = params.m_s * params.g * (params.a / params.L);

    % Total static load on tyres (sprung + unsprung)
    params.W_tf_static = params.W_sf_static + params.m_uf * params.g;
    params.W_tr_static = params.W_sr_static + params.m_ur * params.g;

    % Static deflections (m)
    params.delta_sf_static = params.W_sf_static / params.k_sf;
    params.delta_sr_static = params.W_sr_static / params.k_sr;
    params.delta_tf_static = params.W_tf_static / params.k_tf;
    params.delta_tr_static = params.W_tr_static / params.k_tr;
end
