function [z0f, dz0f, z0r, dz0r, info] = road_triangular(t, cfg, params)
% ROAD_TRIANGULAR Evaluates multiple consecutive triangular road bumps.
%
% Wrapper around road_multi_bump with profile_shape = 'triangular'.

    if nargin < 2; cfg = struct(); end
    cfg.profile_shape = 'triangular';
    [z0f, dz0f, z0r, dz0r, info] = road_multi_bump(t, cfg, params);
end
