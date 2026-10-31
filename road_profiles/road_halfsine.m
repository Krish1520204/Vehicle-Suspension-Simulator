function [z0f, dz0f, z0r, dz0r, info] = road_halfsine(t, cfg, params)
% ROAD_HALFSINE Evaluates multiple consecutive half-sinusoidal road bumps.
%
% Wrapper around road_multi_bump with profile_shape = 'half_sine'.

    if nargin < 2; cfg = struct(); end
    cfg.profile_shape = 'half_sine';
    [z0f, dz0f, z0r, dz0r, info] = road_multi_bump(t, cfg, params);
end
