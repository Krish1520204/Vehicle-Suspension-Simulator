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
        case {'rectangular', 'rect', 'triangular', 'tri', 'triangle', ...
              'half_sine', 'halfsine', 'multi_bump', 'multibump'}
            % Forward to multi-bump engine
            cfg_mb = cfg;
            if ~isfield(cfg_mb, 'profile_shape')
                if ismember(lower(cfg.type), {'rectangular', 'rect'})
                    cfg_mb.profile_shape = 'rectangular';
                elseif ismember(lower(cfg.type), {'triangular', 'tri', 'triangle'})
                    cfg_mb.profile_shape = 'triangular';
                elseif ismember(lower(cfg.type), {'half_sine', 'halfsine'})
                    cfg_mb.profile_shape = 'half_sine';
                end
            end
            [z0f, dz0f, z0r, dz0r] = road_multi_bump(t, cfg_mb, params);
        otherwise
            error('Unknown road profile type "%s". Choose: rectangular, triangular, half_sine, bump, step, sine, iso8608', cfg.type);
    end
end
