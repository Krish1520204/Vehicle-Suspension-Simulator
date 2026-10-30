function [z0f, dz0f, z0r, dz0r] = get_road_profile(t, cfg, params)
% GET_ROAD_PROFILE Dispatches evaluation to specific road profile functions.
%
% Inputs:
%   t      - Current simulation time (s)
%   cfg    - Struct with .type ('bump', 'step', 'sine', 'iso8608') and type-specific parameters
%   params - Vehicle parameters struct
%
% Outputs:
%   z0f, dz0f - Front wheel road elevation (m) and vertical velocity (m/s)
%   z0r, dz0r - Rear wheel road elevation (m) and vertical velocity (m/s)

    if ~isfield(cfg, 'type')
        cfg.type = 'bump';
    end

    switch lower(cfg.type)
        case 'bump'
            [z0f, dz0f, z0r, dz0r] = road_bump(t, cfg, params);
        case 'step'
            [z0f, dz0f, z0r, dz0r] = road_step(t, cfg, params);
        case {'sine', 'sinusoidal'}
            [z0f, dz0f, z0r, dz0r] = road_sine(t, cfg, params);
        case {'iso', 'iso8608', 'random'}
            [z0f, dz0f, z0r, dz0r] = road_iso8608(t, cfg, params);
        otherwise
            error('Unknown road profile type "%s". Choose: bump, step, sine, iso8608', cfg.type);
    end
end
