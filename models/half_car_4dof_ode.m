function dxdt = half_car_4dof_ode(t, x, params, road_func)
% HALF_CAR_4DOF_ODE State-space ODE function for 4-DOF half-car suspension model.
%
% System State Vector:
%   x(1) = z_s       : Sprung mass heave displacement at CG (m)
%   x(2) = theta     : Sprung mass pitch angle (rad)
%   x(3) = z_uf      : Front unsprung mass displacement (m)
%   x(4) = z_ur      : Rear unsprung mass displacement (m)
%   x(5) = dz_s      : Sprung mass heave velocity (m/s)
%   x(6) = dtheta    : Sprung mass pitch angular velocity (rad/s)
%   x(7) = dz_uf     : Front unsprung mass velocity (m/s)
%   x(8) = dz_ur     : Rear unsprung mass velocity (m/s)
%
% Inputs:
%   t         - Current time (s)
%   x         - State vector (8x1)
%   params    - Vehicle parameter struct (from get_vehicle_params)
%   road_func - Function handle: @(t) -> [z0f, dz0f, z0r, dz0r]
%
% Output:
%   dxdt      - Time derivative of state vector (8x1)

    % Extract states
    z_s    = x(1);
    theta  = x(2);
    z_uf   = x(3);
    z_ur   = x(4);
    dz_s   = x(5);
    dtheta = x(6);
    dz_uf  = x(7);
    dz_ur  = x(8);

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

    % Evaluate road profile at current time
    [z0f, dz0f, z0r, dz0r] = road_func(t);

    % Sprung mass kinematics at suspension attachment points
    % Small angle assumption: sin(theta) ~ theta, cos(theta) ~ 1
    z_sf  = z_s + a * theta;
    dz_sf = dz_s + a * dtheta;

    z_sr  = z_s - b * theta;
    dz_sr = dz_s - b * dtheta;

    % Relative suspension deflections and velocities (compression positive)
    delta_sf  = z_uf - z_sf;
    ddelta_sf = dz_uf - dz_sf;

    delta_sr  = z_ur - z_sr;
    ddelta_sr = dz_ur - dz_sr;

    % Suspension forces acting on sprung mass (upward positive)
    F_sf = k_sf * delta_sf + c_sf * ddelta_sf;
    F_sr = k_sr * delta_sr + c_sr * ddelta_sr;

    % Tyre dynamic vertical forces acting on wheels (upward positive)
    F_tf = k_tf * (z0f - z_uf) + c_tf * (dz0f - dz_uf);
    F_tr = k_tr * (z0r - z_ur) + c_tr * (dz0r - dz_ur);

    % Equations of Motion (Accelerations)
    % 1. Sprung mass vertical heave acceleration
    ddz_s = (F_sf + F_sr) / m_s;

    % 2. Sprung mass pitch angular acceleration (moment about CG)
    ddtheta = (a * F_sf - b * F_sr) / I_y;

    % 3. Front unsprung mass vertical acceleration
    ddz_uf = (-F_sf + F_tf) / m_uf;

    % 4. Rear unsprung mass vertical acceleration
    ddz_ur = (-F_sr + F_tr) / m_ur;

    % Assemble state derivative vector
    dxdt = [dz_s; dtheta; dz_uf; dz_ur; ddz_s; ddtheta; ddz_uf; ddz_ur];
end
