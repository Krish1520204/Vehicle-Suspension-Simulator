function sim_result = run_simulation(params, road_cfg, tspan, x0, ode_options)
% RUN_SIMULATION Integrates the 4-DOF half-car model using ODE45.
%
% Syntax:
%   sim_result = run_simulation(params, road_cfg, tspan, x0, ode_options)
%
% Inputs:
%   params      - Vehicle parameter struct (from get_vehicle_params)
%   road_cfg    - Road excitation struct (with .type, .v, etc.)
%   tspan       - [t_start, t_end] integration window (s) [default: [0, 3.0]]
%   x0          - Initial state vector (8x1) [default: zeros(8,1)]
%   ode_options - MATLAB odeset struct [default: RelTol=1e-6, AbsTol=1e-8]
%
% Output:
%   sim_result  - Struct containing time history, all kinematic/dynamic outputs,
%                 road profiles, and ride comfort / road holding metrics.

    % Default inputs
    if nargin < 3 || isempty(tspan)
        tspan = [0, 3.0];
    end
    if nargin < 4 || isempty(x0)
        x0 = zeros(8, 1);
    end
    if nargin < 5 || isempty(ode_options)
        ode_options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', 1e-3);
    end

    % Construct road function handle
    road_func = @(t) get_road_profile(t, road_cfg, params);

    % Define ODE handle
    ode_fun = @(t, x) half_car_4dof_ode(t, x, params, road_func);

    % Solve system equations of motion using ode45
    fprintf('Running ODE45 integration for 4-DOF Half-Car Model...\n');
    tic;
    [t, X] = ode45(ode_fun, tspan, x0, ode_options);
    elapsed_time = toc;
    fprintf('Simulation completed in %.3f seconds (%d time steps).\n', elapsed_time, length(t));

    % Compute derived quantities, forces, accelerations, and performance metrics
    sim_result = calculate_outputs(t, X, params, road_func);
    sim_result.road_cfg = road_cfg;
    sim_result.elapsed_time = elapsed_time;
end
