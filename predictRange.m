function [g_hat, hit_point, valid] = predictRange(p, theta_i, occMap, mscale, offset_m, max_range, sensor_offset)
% predictRange  Ray-cast a single LiDAR beam against the occupancy grid.
%   Returns the predicted range, the world coords of the hit cell, and a
%   validity flag (false if no hit before max_range or outside the map).

    phi = p(3);
    xso = sensor_offset(1); yso = sensor_offset(2);
    xs  = p(1) + xso*cos(phi) - yso*sin(phi);
    ys  = p(2) + xso*sin(phi) + yso*cos(phi);

    cb = cos(phi + theta_i);
    sb = sin(phi + theta_i);

    [H, W]  = size(occMap);
    step    = 0.5 / mscale;
    N_steps = floor(max_range / step);

    g_hat     = max_range;
    hit_point = [NaN; NaN];
    valid     = false;

    for k = 1:N_steps
        d  = k * step;
        xq = xs + d*cb;
        yq = ys + d*sb;
        i  = round((xq + offset_m)*mscale) + 1;
        j  = round((yq + offset_m)*mscale) + 1;

        if i < 1 || i > H || j < 1 || j > W
            return;
        end
        if occMap(i, j)
            g_hat     = d;
            hit_point = [(i-1)/mscale - offset_m; (j-1)/mscale - offset_m];
            valid     = true;
            return;
        end
    end
end
