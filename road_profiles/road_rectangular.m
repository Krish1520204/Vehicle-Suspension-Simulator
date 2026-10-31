function [z0f, dz0f, z0r, dz0r, info] = road_rectangular(t, cfg, params)
% ROAD_RECTANGULAR Evaluates multiple consecutive rectangular road bumps.
%
% Wrapper around road_multi_bump with profile_shape = 'rectangular'.

    if nargin < 2; cfg = struct(); end
    cfg.profile_shape = 'rectangular';
    [z0f, dz0f, z0r, dz0r, info] = road_multi_bump(t, cfg, params);
end
